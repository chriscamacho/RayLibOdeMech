#include "vehicle.h"
#include <stdlib.h>
#include <math.h>

/**
 * @file vehicle.c
 * @brief Vehicle simulation implementation
 *
 * This file provides a complete vehicle simulation using ODE's
 * Hinge2 joints to simulate wheels with suspension. The vehicle
 * consists of a chassis body connected to wheel bodies via
 * simulated suspension joints.
 *
 * @author Chris Camacho (codifies - http://bedroomcoders.co.uk/)
 * @date 2021-2026
 *
 * @section vehicle_structure Vehicle Structure
 *
 * The vehicle is composed of:
 * - 1 chassis body (box shape)
 * - 4 wheel bodies (cylinder shape)
 * - 1 anti-sway mass (lowered center of gravity)
 * - 4 Hinge2 joints (suspension + steering)
 * - 1 Fixed joint (anti-sway attachment)
 *
 * @section control Control
 *
 * Use UpdateVehicle() to control:
 * - accel: Forward/backward acceleration 
 * - steer: Steering angle
 */


/**
 * @brief Create a physics-based vehicle
 *
 * Creates a complete vehicle simulation with chassis, wheels,
 * and suspension. The vehicle uses Hinge2 joints for realistic
 * wheel suspension and steering mechanics.
 *
 * @param pctx Pointer to physics context
 * @param ctx Pointer to graphics context
 * @param pos Initial position of the vehicle
 * @param carScale Dimensions of the chassis (x=length, y=height, z=width)
 * @param wheelRadius Radius of the wheels
 * @param wheelWidth Width of the wheels
 * @return Pointer to newly created vehicle structure
 *
 * @note Front wheels are driving wheels
 * @note Front wheels also provide steering
 * @note Anti-sway mass lowers center of gravity to prevent tipping
 *
 * @see UpdateVehicle
 * @see FreeVehicle
 */
vehicle* CreateVehicle(PhysicsContext* pctx, struct GraphicsContext* ctx, Vector3 pos, Vector3 carScale, float wheelRadius, float wheelWidth)
{
    // Extract ODE world and space from the framework context
    dWorldID world = pctx->world;
    dSpaceID space = pctx->space;

    vehicle* car = RL_MALLOC(sizeof(vehicle));
    
    car->bodyCount = 6; 

    // Get textures for vehicle parts
    Texture* chassisTex = &ctx->boxTextures[0];      // crate.png
    Texture* wheelTex = &ctx->cylinderTextures[1];   // cylinder2.png
    Texture* markerTex = &ctx->boxTextures[1];       // grid.png

    // Calculate wheel radius relative to car height if not explicitly specified
    // Don't scale down wheels that are appropriately sized
    float scaledWheelRadius = wheelRadius;

    // Main Chassis
    dMass m;
    dMassSetBox(&m, 1, carScale.x, carScale.y, carScale.z);
    dMassAdjust(&m, 150); // 150kg chassis
    
    car->bodies[0] = dBodyCreate(world);
    dBodySetMass(car->bodies[0], &m);
    dBodySetAutoDisableFlag(car->bodies[0], 0);
    dBodySetPosition(car->bodies[0], pos.x, pos.y, pos.z);

    car->geoms[0] = dCreateBox(space, carScale.x, carScale.y, carScale.z);
    dGeomSetBody(car->geoms[0], car->bodies[0]);
    dGeomSetData(car->geoms[0], CreateGeomInfo(true, chassisTex, 1.0f, 1.0f));
    
    // Front indicator (visual aid to tell front from back)
    dGeomID frontGeom = dCreateBox(space, 0.2, 0.2, 0.2);
    dGeomSetBody(frontGeom, car->bodies[0]);
    dGeomSetOffsetPosition(frontGeom, carScale.x/2 - 0.1, carScale.y/2 + 0.1, 0);
    dGeomSetData(frontGeom, CreateGeomInfo(true, markerTex, 1.0f, 1.0f));
    car->geoms[6] = frontGeom;
    
    // Anti-Sway / Low CoG Mass (Body 5)
    // This mass lowers the center of gravity to prevent tipping
    car->bodies[5] = dBodyCreate(world);
    dMassSetSphere(&m, 1, 0.5);
    dMassAdjust(&m, 100); 
    dBodySetMass(car->bodies[5], &m);
    dBodySetPosition(car->bodies[5], pos.x, pos.y - 3.0, pos.z);
	// no geom for this body it doesn't collide

    
    // Fixed joint to attach balance mass to chassis
    car->joints[4] = dJointCreateFixed(world, 0); 
    dJointAttach(car->joints[4], car->bodies[0], car->bodies[5]);
    dJointSetFixed(car->joints[4]);
    
    // Wheels (Bodies 1-4)
    dMassSetCylinder(&m, 1, 3, scaledWheelRadius, wheelWidth);
    dMassAdjust(&m, 2); 
    
    // Calculate wheel positions relative to car scale
    // X is forward/back (length), Z is left/right (width)
    float wheelYOffset = -0.5;  // Fixed offset below chassis center - match original
    float offsetX = carScale.x * 0.35;  // Length-wise (front/rear) along X
    float offsetZ = carScale.z * 0.7;   // Width-wise (left/right) along Z - wider stance
    float wheelOffsets[4][3] = {
        {  offsetX, wheelYOffset, -offsetZ }, // Front Left
        {  offsetX, wheelYOffset,  offsetZ }, // Front Right
        { -offsetX, wheelYOffset, -offsetZ }, // Rear Left
        { -offsetX, wheelYOffset,  offsetZ }  // Rear Right
    };

	for(int i = 0; i < 4; i++)
	{
		int bIdx = i + 1;
		car->bodies[bIdx] = dBodyCreate(world);
		dBodySetMass(car->bodies[bIdx], &m);
		dBodySetFiniteRotationMode(car->bodies[bIdx], 1);
		dBodySetAutoDisableFlag(car->bodies[bIdx], 0);
		dBodySetPosition(car->bodies[bIdx], pos.x + wheelOffsets[i][0], pos.y + wheelOffsets[i][1], pos.z + wheelOffsets[i][2]);

		car->geoms[bIdx] = dCreateCylinder(space, scaledWheelRadius, wheelWidth);
		dGeomSetBody(car->geoms[bIdx], car->bodies[bIdx]);
		dGeomSetData(car->geoms[bIdx], CreateGeomInfo(true, wheelTex, 1.0f, 1.0f));
		geomInfo* gi = dGeomGetData(car->geoms[bIdx]);
		gi->surface = &gSurfaces[SURFACE_RUBBER];

        // Hinge2 Joints (Joints 0-3) ---
        car->joints[i] = dJointCreateHinge2(world, 0);
        dJointAttach(car->joints[i], car->bodies[0], car->bodies[bIdx]);
        dJointSetHinge2Anchor(car->joints[i], pos.x + wheelOffsets[i][0], pos.y + wheelOffsets[i][1], pos.z + wheelOffsets[i][2]);
        
        dReal axis1[] = { 0, 1, 0 }; // Steering axis (Y - up/down)
        dReal axis2[] = { 0 , 0, ((i % 2) == 0) ? -1 : 1 };
        dJointSetHinge2Axes(car->joints[i], axis1, axis2);

        // Soften suspension to help prevent the car from bouncing or flipping
        dJointSetHinge2Param(car->joints[i], dParamSuspensionERP, 0.9);
        dJointSetHinge2Param(car->joints[i], dParamSuspensionCFM, 0.002);

		if (i < 2) { // Front wheel steering limits
			dJointSetHinge2Param(car->joints[i], dParamLoStop, -.75);
			dJointSetHinge2Param(car->joints[i], dParamHiStop, .75);
			dJointSetHinge2Param(car->joints[i], dParamFMax, 15000);
		} else { // Rear wheels - LOCK steering axis completely
			dJointSetHinge2Param(car->joints[i], dParamLoStop, 0.0);
			dJointSetHinge2Param(car->joints[i], dParamHiStop, 0.0);  // Same as LoStop = locked
		}

    }

    // Link bodies to entities so drawBodies and FreeVehicle function correctly
    for (int i = 0; i < 6; i++) {
        entity* ent = RL_MALLOC(sizeof(entity));
        ent->body = car->bodies[i];
        dBodySetData(car->bodies[i], ent);
        ent->node = clistAddNode(pctx->objList, ent); //
    }

    return car;
}

/**
 * @brief Update vehicle control inputs
 *
 * Applies acceleration and steering to the vehicle. This should
 * be called every frame with current control inputs.
 *
 * @param car Pointer to the vehicle structure
 * @param accel Acceleration input (-1.0 to 1.0, negative for reverse)
 * @param steer Steering input (-1.0 to 1.0, negative for left)
 *
 * @note Set accel to 0 for coasting (wheels spin free)
 * @note Front wheels provide steering and all-wheel drive
 */
void UpdateVehicle(vehicle *car, float accel, float steer)
{
    // Match original force values
    float driveTorque = 800.0f; 
    float steeringForce = 800.0f;//500.0f;
    float steerFactor = 10.0f;
    
    // If no acceleration input, let the wheels spin free (coasting)
    // Otherwise, it acts like a permanent brake
    float currentTorque = (accel == 0) ? 0.0f : driveTorque;

    // Front Wheels (0, 1) - Driving
    
    dJointSetHinge2Param(car->joints[0], dParamVel2, -accel);
    dJointSetHinge2Param(car->joints[1], dParamVel2, accel);
    dJointSetHinge2Param(car->joints[0], dParamFMax2, currentTorque);
    dJointSetHinge2Param(car->joints[1], dParamFMax2, currentTorque);
    
    // Rear Wheels (2, 3) - Driving
    dJointSetHinge2Param(car->joints[2], dParamVel2, -accel);
    dJointSetHinge2Param(car->joints[3], dParamVel2, accel);
    dJointSetHinge2Param(car->joints[2], dParamFMax2, currentTorque);
    dJointSetHinge2Param(car->joints[3], dParamFMax2, currentTorque);
    
    
    // Steering Logic    
    for(int i = 0; i < 2; i++) {
		dReal v = steer - dJointGetHinge2Angle1(car->joints[i]);
		v *= steerFactor;
		
		dJointSetHinge2Param(car->joints[i], dParamFMax, steeringForce);
		dJointSetHinge2Param(car->joints[i], dParamVel, v);
	}
    

}

/**@brief this releases all resources in use by a vehicle struct
 * 
 * @param pctx the physics context
 * @param car the vehicle to free
 * 
 */
void FreeVehicle(PhysicsContext* pctx, vehicle* car)
{
    if (!car) return;

    for (int i = 0; i < car->bodyCount; i++) {
        // Get the entity wrapper attached to the ODE body
        entity* ent = (entity*)dBodyGetData(car->bodies[i]);
        
        if (ent) {
            // Remove the node from the framework's render list
            clistDeleteNode(pctx->objList, &ent->node);
            FreeBodyAndGeoms(car->bodies[i]); 
            RL_FREE(ent);
        }
    }
    RL_FREE(car);
}

/** @brief flips an upside down vehicle
 * 
 * @param car the vehicle to flip
 * 
 * @note this is a quick hack, it violently pops the vehicle back
 * upright - there are better ways of doing this!
 */
void UnflipVehicle (vehicle *car)
{
    const dReal* cp = dBodyGetPosition(car->bodies[0]);
    dBodySetPosition(car->bodies[0], cp[0], cp[1]+2, cp[2]);

    const dReal* R = dBodyGetRotation(car->bodies[0]);
    dReal newR[16];
    dRFromEulerAngles(newR, 0, -atan2(-R[2],R[0]) , 0);
    dBodySetRotation(car->bodies[0], newR);
    
    // wheel offsets
    // TODO make configurable & use in vehicle set up 
    // these should be close enough however...
    dReal wheelOffsets[4][3] = {
           { +1.2, -.6, -1 },
           { +1.2, -.6, +1 },
           { -1.2, -.6, -1 },
           { -1.2, -.6, +1 }
        };

    for (int i=1; i<5; i++) {
        dVector3 pb;
        dBodyGetRelPointPos(car->bodies[0], wheelOffsets[i-1][0], wheelOffsets[i-1][1], wheelOffsets[i-1][2], pb);
        dBodySetPosition(car->bodies[i], pb[0], pb[1], pb[2]);
    }

}
