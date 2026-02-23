/*
 * Copyright (c) 2021-2026 Chris Camacho (codifies -  http://bedroomcoders.co.uk/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/**
 * @file raylibODE.c
 * @brief Core physics and rendering integration for RayLib-ODE framework
 *
 * This file contains the main framework functions for creating physics bodies,
 * handling collisions, and rendering geometric primitives. It serves as the
 * bridge between RayLib's rendering system and ODE's physics engine.
 *
 * @author Chris Camacho (codifies - http://bedroomcoders.co.uk/)
 * @date 2021-2026
 *
 * @mainpage RayLibOdeMech Framework
 *
 * @section overview Overview
 *
 * RayLibOdeMech is a physics framework that combines RayLib for graphics
 * rendering with ODE (Open Dynamics Engine) for physics simulation. It
 * provides an easy-to-use API for creating 3D physics simulations with
 * various primitive shapes and complex composite objects.
 *
 * The main functionality is documented here @ref raylibODE.c
 *
 * @section preperation Compiling ODE for the framework
 *
 * ODE is linked in statically for convienience.
 *
 * get ODE from https://bitbucket.org/odedevs/ode/downloads/
 *
 * extract ode 0.16.6 into a directory at the same level as this project (see below)
 *
 * ln -s ode-0.16.6 ode<br>
 * cd ode
 *
 * I'd suggest building it with this configuration<br>
 * ./configure --enable-ou --enable-libccd --with-box-cylinder=libccd --with-drawstuff=none --disable-demos --with-libccd=internal
 *
 * and run make, you should then be set to compile this project
 *
 * Your file structure should look like this
 *
 * ode<br>
 * raylib<br>
 * RayLibOdeMech
 *
 *
 * @section features Features
 *
 * - Physics body creation (boxes, spheres, cylinders, capsules)
 * - Composite shapes (dumbbell example shape)
 * - Static trimesh support for arbitrary geometry
 * - Ray picking for mouse interaction
 * - Automatic collision detection and response
 * - rotor and piston joint examples
 *
 *
 * @section usage Basic Usage
 *
 * @subsection initialization Initialization
 *
 * @code
 * // Initialize graphics and physics
 *  PhysicsContext* physCtx = CreatePhysics();
 *  GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
 *  SetupCamera(graphics);
 * @endcode
 *
 * @subsection creating_objects Creating Objects
 *
 * @code
 * // Add a box to the physics world
 * Vector3 size = {1.0f, 1.0f, 1.0f};
 * Vector3 pos = {0.0f, 5.0f, 0.0f};
 * Vector3 rot = {0.0f, 0.0f, 0.0f};
 * entity* box = CreateBox(physCtx, &gfxCtx, size, pos, rot, 10.0f);
 *
 * // Create a static ground box
 * dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
 * dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
 * dGeomSetData(planeGeom, CreateGeomInfo(true, graphics->groundTexture, 25.0f, 25.0f));
 * clistAddNode(physCtx->statics, planeGeom);
 * @endcode
 *
 * @subsection simulation Simulation Loop
 *
 * @code
 * // In your main loop
 * while (!WindowShouldClose()) {
 *     // Step physics
 *     StepPhysics(physCtx);
 *     UpdateExampleCamera(graphics);
 *
 *     // Render
 *     BeginDrawing();
 *         ClearBackground(RAYWHITE);
 *         BeginMode3D(graphics->camera);
 *             drawBodies(graphics, physCtx);
 *             drawStatics(graphics, physCtx);
 *         EndMode3D();
 *     EndDrawing();
 * }
 * @endcode
 *
 * @subsection cleanup Cleanup
 *
 * @code
 * // Clean up resources
 * FreePhysics(physCtx);
 * FreeGraphics(graphics);
 * CloseWindow();
 * @endcode
 *
 * @section dependencies Dependencies
 *
 * - RayLib 5.0+
 * - ODE (Open Dynamics Engine) 0.16+
 *
 * @section license License
 *
 * MIT License - See LICENSE file for details
 */
 
 /**
 * @example arm.c
 * @par
 * control a robot arm with IO, KL, ,. keys and G to release grabber
 *  
 * @example car.c
 * @par
 * use the cursor keys to control a simple vehicle over a trimesh
 *  
 * @example derby.c
 * @par
 * a whole bunch of cars, following a figure of 8 path and making
 * no attempt to avoid collisions !
 *  
 * @example fountain.c
 * @par
 * many shapes being created and destroyed, show using a sphere
 * as a trigger area
 *  
 * @example piston.c
 * @par
 * A simple piston, press O and P to move the pistons, the main body
 * of the piston is achored to the world so it doesn't move
 * 
 * @example placer.c
 * @par
 * place new shapes in the world, and push them about <br>
 * shows how to do ray cast detection
 *  
 * @example ragdolls.c
 * @par
 * press space to move them around (random upwards and sideways
 * force applied to head)
 *  
 * @example rotor.c
 * @par
 * shows the rotor convenience function
 *  
 * @example template.c
 * @par
 * This example does nothing! it has a static ground box and camera
 * it is intended to be copied and used for your own experiments
 * 
 * @example terrain.c
 * @par
 * shows shapes colliding and coming to rest on a static trimesh
 */
#include <stdlib.h>
#include <math.h>
#include <string.h>  // memset
#include "raylibODE.h"
#include "collision.h"
#include "surface.h"

/**
 * @brief Physics simulation time slice
 *
 * The fixed time step used for physics simulation.
 * ODE recommends using fixed time steps for consistent results.
 * 1/240 = ~240 Hz update rate.
 * the framework will step as many times as it needs to to
 * keep up with realtime in this sized steps
 */
const float physSlice = 1.0 / 240.0;

/**
 * @brief Generate a random float in the specified range
 *
 * @param min Minimum value (inclusive)
 * @param max Maximum value (inclusive)
 * @return Random float between min and max
 *
 * @note Uses rand() from stdlib - consider seeding with srand() for better randomness
 */
float rndf(float min, float max)
{
    return ((float)rand() / (float)(RAND_MAX)) * (max - min) + min;
}

/**
 * @brief Step the physics simulation forward
 *
 * This function advances the physics simulation by stepping the world
 * forward using fixed time steps. It handles the frame time accumulation
 * and performs multiple physics steps if necessary to maintain simulation
 * stability and accuracy.
 *
 * @param physCtx Pointer to the physics context containing the world and state
 * @return Number of physics steps performed this frame
 *
 * @note Uses dWorldQuickStep for faster but less accurate simulation
 * @note Maximum number of steps is limited by maxPsteps to prevent spiral of death
 *
 * @see PhysicsContext
 * @see dWorldQuickStep
 */
int StepPhysics(PhysicsContext* physCtx)
{
	int pSteps = 0;
	physCtx->frameTime += GetFrameTime();
	while (physCtx->frameTime > physSlice) {
		// check for collisions
		dSpaceCollide(physCtx->space, physCtx, &nearCallback);

		// step the world
		// although this does steps itself, doing it multiple times
		// allows for smoother physics and gives us a way to sync
		// physics time to everything else.
		dWorldQuickStep(physCtx->world, physSlice);  // NB fixed time step is important
		dJointGroupEmpty(physCtx->contactgroup);

		physCtx->frameTime -= physSlice;
		pSteps++;
		if (pSteps > maxPsteps) {
			physCtx->frameTime = 0;
			break;
		}
	}
	return pSteps;
}



// --- HELPER: Common Entity Initialization ---
// Internal helper to reduce boilerplate for all shape functions

/**
 * @brief Create a base entity with physics body
 *
 * Internal helper function that creates a new entity with an ODE body
 * and adds it to the physics world's object list. This reduces code
 * duplication across all shape creation functions.
 *
 * @param ctx Pointer to the physics context
 * @return Pointer to newly created entity
 *
 * @note Internal function - but might be useful for manual creation
 */
entity* CreateBaseEntity(PhysicsContext* ctx) {
    dBodyID bdy = dBodyCreate(ctx->world);
    entity* ent = RL_MALLOC(sizeof(entity));
    ent->body = bdy;
    dBodySetData(bdy, ent);
    ent->node = clistAddNode(ctx->objList, ent);
    return ent;
}

// TODO the geom only creation functions should be used by the
// create entity function CreateBall CreateBox etc

// just an isolated geom, most useful to add to existing bodies
dGeomID CreateSphereGeom(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, Vector3 pos)
{
    dGeomID geom = dCreateSphere(ctx->space, radius);
    dGeomSetPosition(geom, pos.x, pos.y, pos.z);

    Texture* tex = &gfxCtx->sphereTextures[(int)rndf(0, 3)];
    geomInfo* gi = CreateGeomInfo(true, tex, 1.0f, 1.0f);
    dGeomSetData(geom, gi);

    return geom;
}

// isolated cylinder geom
dGeomID CreateCylinderGeom(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, float length, Vector3 pos)
{
    // ODE Cylinders are aligned along the Z-axis by default
    dGeomID geom = dCreateCylinder(ctx->space, radius, length);
    dGeomSetPosition(geom, pos.x, pos.y, pos.z);

    Texture* tex = &gfxCtx->cylinderTextures[(int)rndf(0, 2)];
    geomInfo* gi = CreateGeomInfo(true, tex, 1.0f, 1.0f);
    dGeomSetData(geom, gi);

    return geom;
}

dGeomID CreateBoxGeom(PhysicsContext* ctx, GraphicsContext* gfxCtx, Vector3 size, Vector3 pos)
{
	dGeomID geom = dCreateBox(ctx->space, size.x, size.y, size.z);
	dGeomSetPosition(geom, pos.x, pos.y, pos.z);
	
	Texture* tex = &gfxCtx->boxTextures[(int)rndf(0, 2)];
    geomInfo* gi = CreateGeomInfo(true, tex, 1.0f, 1.0f);
    dGeomSetData(geom, gi);

    return geom;
}

// --- BOX ---

/**
 * @brief Add a box-shaped physics body to the world
 *
 * Creates a new dynamic box in the physics simulation with the specified
 * dimensions, position, rotation, and mass. The box is automatically
 * added to the rendering list and collision detection system.
 *
 * @param ctx Pointer to the physics context
 * @param gfxCtx Pointer to the graphics context for texture assignment
 * @param size Dimensions of the box (width, height, depth)
 * @param pos Initial position of the box center
 * @param rot Initial rotation (Euler angles: pitch, yaw, roll)
 * @param mass Mass of the box (0 for static object)
 * @return Pointer to the newly created entity
 *
 * @note Box is assigned a random texture from the available box textures
 * @see entity
 * @see PhysicsContext
 * @see GraphicsContext
 */
entity* CreateBox(PhysicsContext* ctx, GraphicsContext* gfxCtx, Vector3 size, Vector3 pos, Vector3 rot, float mass)
{
    entity* ent = CreateBaseEntity(ctx);
    dMatrix3 R;
    dMass m;

    dGeomID geom = dCreateBox(ctx->space, size.x, size.y, size.z);
    dMassSetBox(&m, mass, size.x, size.y, size.z);

    dBodySetPosition(ent->body, pos.x, pos.y, pos.z);
    dRFromEulerAngles(R, rot.x, rot.y, rot.z);
    dBodySetRotation(ent->body, R);

    dGeomSetBody(geom, ent->body);
    dBodySetMass(ent->body, &m);

    Texture* tex = &gfxCtx->boxTextures[(int)rndf(0, 2)];
    dGeomSetData(geom, CreateGeomInfo(true, tex, 1.0f, 1.0f));

    return ent;
}

// --- SPHERE ---

/**
 * @brief Add a sphere-shaped physics body to the world
 *
 * Creates a new dynamic sphere in the physics simulation with the specified
 * radius, position, rotation, and mass. Spheres are efficient for collision
 * detection and are commonly used for balls and round objects.
 *
 * @param ctx Pointer to the physics context
 * @param gfxCtx Pointer to the graphics context for texture assignment
 * @param radius Radius of the sphere
 * @param pos Initial position of the sphere center
 * @param rot Initial rotation (Euler angles)
 * @param mass Mass of the sphere (0 for static object)
 * @return Pointer to the newly created entity
 *
 * @note Sphere is assigned a random texture from the available sphere textures
 * @see entity
 * @see PhysicsContext
 */
entity* CreateSphere(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, Vector3 pos, Vector3 rot, float mass)
{
    entity* ent = CreateBaseEntity(ctx);
    dMatrix3 R;
    dMass m;

    dGeomID geom = dCreateSphere(ctx->space, radius);
    dMassSetSphere(&m, mass, radius);

    dBodySetPosition(ent->body, pos.x, pos.y, pos.z);
    dRFromEulerAngles(R, rot.x, rot.y, rot.z);
    dBodySetRotation(ent->body, R);

    dGeomSetBody(geom, ent->body);
    dBodySetMass(ent->body, &m);

    Texture* tex = &gfxCtx->sphereTextures[(int)rndf(0, 3)];
    dGeomSetData(geom, CreateGeomInfo(true, tex, 1.0f, 1.0f));

    return ent;
}

// --- CYLINDER ---

/**
 * @brief Add a cylinder-shaped physics body to the world
 *
 * Creates a new dynamic cylinder in the physics simulation with the specified
 * radius, length, position, rotation, and mass. ODE cylinders are aligned
 * along the Z-axis by default.
 *
 * @param ctx Pointer to the physics context
 * @param gfxCtx Pointer to the graphics context for texture assignment
 * @param radius Radius of the cylinder
 * @param length Length of the cylinder (along Z-axis)
 * @param pos Initial position of the cylinder center
 * @param rot Initial rotation (Euler angles)
 * @param mass Mass of the cylinder (0 for static object)
 * @return Pointer to the newly created entity
 *
 * @note Cylinder is aligned along Z-axis as per ODE convention
 * @note Cylinder is assigned a random texture from the available cylinder textures
 * @see entity
 */
entity* CreateCylinder(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, float length, Vector3 pos, Vector3 rot, float mass) 
{
    entity* ent = CreateBaseEntity(ctx);
    dMatrix3 R;
    dMass m;

    // ODE Cylinders are aligned along the Z-axis by default
    dGeomID geom = dCreateCylinder(ctx->space, radius, length);
    dMassSetCylinder(&m, mass, 3, radius, length); // 3 = Z-axis orientation

    dBodySetPosition(ent->body, pos.x, pos.y, pos.z);
    dRFromEulerAngles(R, rot.x, rot.y, rot.z);
    dBodySetRotation(ent->body, R);

    dGeomSetBody(geom, ent->body);
    dBodySetMass(ent->body, &m);

    Texture* tex = &gfxCtx->cylinderTextures[(int)rndf(0, 2)];
    dGeomSetData(geom, CreateGeomInfo(true, tex, 1.0f, 1.0f));

    return ent;
}

// --- CAPSULE ---

/**
 * @brief Add a capsule-shaped physics body to the world
 *
 * Creates a new dynamic capsule in the physics simulation. Capsules are
 * useful for character controllers and objects that need smooth
 * collision with the ground.
 *
 * @param ctx Pointer to the physics context
 * @param gfxCtx Pointer to the graphics context for texture assignment
 * @param radius Radius of the capsule caps
 * @param length Length of the cylindrical portion
 * @param pos Initial position of the capsule center
 * @param rot Initial rotation (Euler angles)
 * @param mass Mass of the capsule (0 for static object)
 * @return Pointer to the newly created entity
 *
 * @note Capsule combines a cylinder with two hemispherical ends
 * @see entity
 */
entity* CreateCapsule(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, float length, Vector3 pos, Vector3 rot, float mass) {
    entity* ent = CreateBaseEntity(ctx);
    dMatrix3 R;
    dMass m;

    dGeomID geom = dCreateCapsule(ctx->space, radius, length);
    dMassSetCapsule(&m, mass, 3, radius, length);

    dBodySetPosition(ent->body, pos.x, pos.y, pos.z);
    dRFromEulerAngles(R, rot.x, rot.y, rot.z);
    dBodySetRotation(ent->body, R);

    dGeomSetBody(geom, ent->body);
    dBodySetMass(ent->body, &m);

    Texture* tex = &gfxCtx->cylinderTextures[(int)rndf(0, 2)];
    dGeomSetData(geom, CreateGeomInfo(true, tex, 1.0f, 1.0f));

    return ent;
}

// --- DUMBBELL (Composite) ---
// possibly not that useful but shows how to create your own shapes to cover
// more complex visual meshes to enable fast collision.

/**
 * @brief Add a dumbbell-shaped composite physics body
 *
 * Creates a composite physics body consisting of a cylindrical shaft
 * with two spherical ends. This demonstrates how to create custom
 * composite shapes by combining multiple geometries.
 *
 * @param ctx Pointer to the physics context
 * @param gfxCtx Pointer to the graphics context for texture assignment
 * @param shaftRad Radius of the cylindrical shaft
 * @param shaftLen Length of the cylindrical shaft
 * @param endRad Radius of the end spheres
 * @param pos Initial position of the dumbbell center
 * @param rot Initial rotation (Euler angles)
 * @param mass Total mass of the dumbbell
 * @return Pointer to the newly created entity
 *
 * @note This is an example of creating composite shapes
 * @note Mass is distributed: 50% shaft, 25% each end
 * @see entity
 */
entity* CreateDumbbell(PhysicsContext* ctx, GraphicsContext* gfxCtx, float shaftRad, float shaftLen, float endRad, Vector3 pos, Vector3 rot, float mass) {
    entity* ent = CreateBaseEntity(ctx);
    dMatrix3 R;
    dMass m, mTotal, mSphere;

    // 1. Shaft (Cylinder)
    dGeomID gShaft = dCreateCylinder(ctx->space, shaftRad, shaftLen);
    dMassSetCylinder(&mTotal, mass * 0.5f, 3, shaftRad, shaftLen);
    dGeomSetBody(gShaft, ent->body);

    // 2. Ends (Spheres)
    float offset = shaftLen / 2.0f;
    dGeomID gEnd1 = dCreateSphere(ctx->space, endRad);
    dGeomID gEnd2 = dCreateSphere(ctx->space, endRad);
    dGeomSetBody(gEnd1, ent->body);
    dGeomSetBody(gEnd2, ent->body);
    dGeomSetOffsetPosition(gEnd1, 0, 0, offset);
    dGeomSetOffsetPosition(gEnd2, 0, 0, -offset);

    // Combine Masses
    dMassSetSphere(&mSphere, mass * 0.25f, endRad);
    m = mSphere; dMassTranslate(&m, 0, 0, offset); dMassAdd(&mTotal, &m);
    m = mSphere; dMassTranslate(&m, 0, 0, -offset); dMassAdd(&mTotal, &m);

    dBodySetMass(ent->body, &mTotal);
    dBodySetPosition(ent->body, pos.x, pos.y, pos.z);
    dRFromEulerAngles(R, rot.x, rot.y, rot.z);
    dBodySetRotation(ent->body, R);

    Texture* tex = &gfxCtx->cylinderTextures[(int)rndf(0, 2)];
    dGeomSetData(gShaft, CreateGeomInfo(true, tex, 1.0f, 1.0f));
    dGeomSetData(gEnd1, CreateGeomInfo(true, tex, 1.0f, 1.0f));
    dGeomSetData(gEnd2, CreateGeomInfo(true, tex, 1.0f, 1.0f));

    return ent;
}

/**
 * @brief Add a random physics object at the specified position
 *
 * Creates a random physics body (box, sphere, cylinder, capsule, or dumbbell)
 * at the given position. Useful for testing and creating dynamic scenes
 * without manually specifying each object.
 *
 * @param ctx Pointer to the physics context
 * @param gfxCtx Pointer to the graphics context
 * @param pos Position where the random object should be created
 * @param mask which shapes to choose from SHAPE_ALL for all of them SHAPE & ~SHAPE_CAPSULE for all but capsule etc
 * @return Pointer to the newly created entity
 * 
 *
 * @note Distribution: 20% boxes, 20% spheres, 20% cylinders, 20% capsules, 20% dumbbells
 * @note Mass is fixed at 10.0f for all random objects
 * @note Each type has randomized dimensions within reasonable ranges
 */
 

entity* CreateRandomEntity(PhysicsContext* ctx, GraphicsContext* gfxCtx, Vector3 pos, unsigned char mask)
{
    unsigned char typ;
    
    do {
		float c = rndf(0, 1);
		if (c < 0.20) {  // Box
			typ = SHAPE_BOX;
		} else if (c < 0.40) {  // Sphere
			typ = SHAPE_SPHERE;
		} else if (c < 0.60) {  // Cylinder
			typ = SHAPE_CYLINDER;
		} else if (c < 0.80) {  // Capsule
			typ = SHAPE_CAPSULE;
		} else {  // Dumbbell
			typ = SHAPE_DUMBBELL;
		}
		if (mask & typ) break;
	} while(true);
    Vector3 rot = (Vector3){rndf(0, 6.28), rndf(0, 6.28), rndf(0, 6.28)};
    float mass = 10.0f;

    if (typ == SHAPE_BOX) {  // Box
        Vector3 s = (Vector3){rndf(0.25, .5), rndf(0.25, .5), rndf(0.25, .5)};
        return CreateBox(ctx, gfxCtx, s, pos, rot, mass);

    } else if (typ == SHAPE_SPHERE) {  // Sphere
        return CreateSphere(ctx, gfxCtx, rndf(0.25, .4), pos, rot, mass);

    } else if (typ == SHAPE_CYLINDER) {  // Cylinder
        return CreateCylinder(ctx, gfxCtx, rndf(0.125, .5), rndf(0.4, 1), pos, rot, mass);

    } else if (typ == SHAPE_CAPSULE) {  // Capsule
        return CreateCapsule(ctx, gfxCtx, rndf(0.125, .3), rndf(0.4, 1), pos, rot, mass);

    } else {  // Dumbbell
        float sRad = 0.1f;
        float sLen = rndf(.8, 1.2);
        float eRad = rndf(0.1, 0.2) + sRad;
        return CreateDumbbell(ctx, gfxCtx, sRad, sLen, eRad, pos, rot, mass);
    }
}

/**
 * @brief Create a static trimesh collision geometry from a model
 *
 * Creates a static trimesh (triangle mesh) collision geometry from a RayLib
 * model. This allows arbitrary 3D models to be used as static collision
 * objects in the physics simulation. The mesh is optimized for collision
 * detection but does not move or respond to physics.
 *
 * @param physCtx Pointer to the physics context
 * @param gfxCtx Pointer to the graphics context
 * @param model RayLib model to create collision geometry from
 * @param tex Optional texture to apply to the mesh (can be NULL)
 * @param uvScale UV scaling factor for texture tiling
 * @return Pointer to the created node in the statics list
 *
 * @note The mesh is static and cannot be moved after creation
 * @note Supports custom textures and UV scaling for visual appearance
 * @note Stores model data for rendering with custom shaders
 *
 * @see GraphicsContext
 * @see Model
 */
cnode_t* CreateStaticTrimesh(PhysicsContext* physCtx, GraphicsContext* gfxCtx, Model model, Texture* tex, float uvScale)
{
    int nV = model.meshes[0].vertexCount;

    // Setup ODE Data
    int* indices = RL_MALLOC(nV * sizeof(int));
    for (int i = 0; i < nV; i++) indices[i] = i;

    dTriMeshDataID triData = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(triData, model.meshes[0].vertices, 3 * sizeof(float), nV,
                                indices, nV, 3 * sizeof(int));

    dGeomID geom = dCreateTriMesh(physCtx->space, triData, NULL, NULL, NULL);

    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = *tex;
    model.materials[0].shader = gfxCtx->shader;

    // Setup Metadata
    geomInfo* gi = CreateGeomInfo(true, tex, uvScale, uvScale);
    gi->visual = model; // Stores the textured/shader-ready model
    gi->indices = indices;
    gi->triData = triData;
    dGeomSetData(geom, gi);

    return clistAddNode(physCtx->statics, geom);
}

/** @brief Helper to allocate geomInfo with collision flag, optional texture, and UV scale
 this is useful when greating your own custom bodies for special purposes
 If this is attached to a geom on a body that is in the global entity list then this
 allocation will be automagically cleaned up.

 @param collidable should this geom cause collisions
 @param texture which texture to use for the geom
 @param uvScaleU texture scaling
 @param uvScaleV texture scaling
*/
geomInfo* CreateGeomInfo(bool collidable, Texture* texture, float uvScaleU, float uvScaleV)
{
    geomInfo* gi = RL_MALLOC(sizeof(geomInfo));
    memset(gi, 0, sizeof(geomInfo)); // ensure visual for example is clear

    gi->collidable = collidable;
    gi->texture = texture;
    gi->uvScaleU = uvScaleU;
    gi->uvScaleV = uvScaleV;
    gi->hew = WHITE;
    gi->surface = &gSurfaces[SURFACE_WOOD];
    return gi;
}


// Callback for ODE to test ray against other geoms
static void rayCallback(void* data, dGeomID o1, dGeomID o2) {
    RayHit* hit = (RayHit*)data;
    dContact contact;

    // We only care about collisions involving the ray (o1) and something else (o2)
    // o2 must not be another ray or a plane if you want to pick objects only
    if (dCollide(o1, o2, 1, &contact.geom, sizeof(dContact)) > 0) {
        if (contact.geom.depth < hit->depth) {
            hit->depth = contact.geom.depth;
            hit->geom = o2;
            hit->pos = (Vector3){contact.geom.pos[0], contact.geom.pos[1], contact.geom.pos[2]};
        }
    }
}

/** @brief this will return the first entity that the camera is directly pointed at
 * while useful in the examples and likely for other things, it also serves as an example
 * of how you can do this yourself for other things
 *
 * @param physCtx the physics context (global physics info)
 * @param gfxCtx the graphics context (global info usful for graphics)
 * @param hitPoint pass a pointer to a Vector3 this will be set with the location of the intersecton
 *
 * @returns a pointer to the entity that was found.

 */
entity* PickEntity(PhysicsContext* physCtx, GraphicsContext* gfxCtx, Vector3* hitPoint) {

    Vector2 screenCenter = { GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
    Ray ray = GetMouseRay(screenCenter, gfxCtx->camera);

    // Create a temporary ODE ray
    float rayLength = 1000.0f;
    dGeomID odeRay = dCreateRay(physCtx->space, rayLength);
    dGeomRaySet(odeRay, ray.position.x, ray.position.y, ray.position.z,
                        ray.direction.x, ray.direction.y, ray.direction.z);

    // Setup hit tracking
    RayHit hit = { 0 };
    hit.depth = rayLength; // Start with max distance

    // Collide the ray against everything in the space
    dSpaceCollide2(odeRay, (dGeomID)physCtx->space, &hit, &rayCallback);

    // Cleanup the temporary ray
    dGeomDestroy(odeRay);

    if (hit.geom != NULL) {
        if (hitPoint) *hitPoint = hit.pos;

        // Get the body attached to the geom
        dBodyID bdy = dGeomGetBody(hit.geom);
        if (bdy) {
            // Retrieve our entity pointer from the body data
            return (entity*)dBodyGetData(bdy);
        }
    }

    return NULL;
}

/** @brief given a body it will remove it and its geoms from ODE's world
 * 
 * @note this is intended to be an internal function that might have use externally
 * you would normally use FreeEntity
 * 
 * As well as freeing all associated metadata it will also remove the body and
 * its geoms from the physics world
 * 
 * @param bdy the dBodyID you want to free the attached meta data for
 * 
 */

void FreeBodyAndGeoms(dBodyID bdy)
{
	dGeomID geom = dBodyGetFirstGeom(bdy);
	while(geom) {
		dGeomID next = dBodyGetNextGeom(geom); // get next now as about to destroy...
		geomInfo* gi = dGeomGetData(geom);
		if (gi) free(gi);
		dGeomSetBody(geom, 0);
		dGeomDestroy(geom);
		geom = next;
	}

	dBodyDestroy(bdy);
}

// position rotation scale all done with the models transform...
// TODO check there isn't a new raylib function that does this now _pro _ex or similar!
void DrawModelTinted(Model model, Color tint)
{

    for (int i = 0; i < model.meshCount; i++)
    {
        Color color = model.materials[model.meshMaterial[i]].maps[MATERIAL_MAP_DIFFUSE].color;

        Color colorTint = WHITE;
        colorTint.r = (unsigned char)((((float)color.r/255.0)*((float)tint.r/255.0))*255.0f);
        colorTint.g = (unsigned char)((((float)color.g/255.0)*((float)tint.g/255.0))*255.0f);
        colorTint.b = (unsigned char)((((float)color.b/255.0)*((float)tint.b/255.0))*255.0f);
        colorTint.a = (unsigned char)((((float)color.a/255.0)*((float)tint.a/255.0))*255.0f);

        model.materials[model.meshMaterial[i]].maps[MATERIAL_MAP_DIFFUSE].color = colorTint;
        DrawMesh(model.meshes[i], model.materials[model.meshMaterial[i]], model.transform);
        model.materials[model.meshMaterial[i]].maps[MATERIAL_MAP_DIFFUSE].color = color;
    }
}


// these two just convert to column major and minor
/** @brief set an ODE matrix to the same as a Raylib Matrix
 * 
 * @param m a pointer to the Raylib matrix
 * @param R a pointer to an ODE matrix
 */
void RayToOdeMat(Matrix* m, dReal* R) {
    R[ 0] = m->m0;   R[ 1] = m->m4;   R[ 2] = m->m8;    R[ 3] = 0;
    R[ 4] = m->m1;   R[ 5] = m->m5;   R[ 6] = m->m9;    R[ 7] = 0;
    R[ 8] = m->m2;   R[ 9] = m->m6;   R[10] = m->m10;   R[11] = 0;
    R[12] = 0;       R[13] = 0;       R[14] = 0;        R[15] = 1;
}

/** @brief sets a raylib matrix from an ODE rotation matrix
 * 
 * @param R a pointer to the ODE matrix
 * @param m a pointer to a Raylib Matrix
 */
void OdeToRayMat(const dReal* R, Matrix* m)
{
    m->m0 = R[0];  m->m1 = R[4];  m->m2 = R[8];      m->m3 = 0;
    m->m4 = R[1];  m->m5 = R[5];  m->m6 = R[9];      m->m7 = 0;
    m->m8 = R[2];  m->m9 = R[6];  m->m10 = R[10];    m->m11 = 0;
    m->m12 = 0;    m->m13 = 0;    m->m14 = 0;        m->m15 = 1;
}

// called by draw all geoms

void DrawGeom(dGeomID geom, struct GraphicsContext* ctx)
{
    geomInfo* gi = (geomInfo*)dGeomGetData(geom);
    if (!gi) return; // Silently bail if no metadata

    // If it's a trigger, don't render anything
    if (gi->triggerOnCollide) return;

	// if its texture has been nulled its an invisible
	if (!gi->texture) return;

    const dReal* pos = dGeomGetPosition(geom);
    const dReal* rot = dGeomGetRotation(geom);
    int class = dGeomGetClass(geom);

    Matrix matRot;
    OdeToRayMat(rot, &matRot);
    Matrix matTran = MatrixTranslate(pos[0], pos[1], pos[2]);
    Matrix matWorld = MatrixMultiply(matRot, matTran);

    Color c = gi->hew;

    // Handle Texture/Shader setup once per geom
    if (gi->texture) {
		// TODO cache shader location
        int uvLoc = GetShaderLocation(ctx->shader, "texCoordScale");
        Vector2 uvScale = { gi->uvScaleU, gi->uvScaleV };
        SetShaderValue(ctx->shader, uvLoc, &uvScale.x, SHADER_UNIFORM_VEC2);

        // assign the texture to whichever models might be used
        if (class == dBoxClass) ctx->box.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = *gi->texture;
        else if (class == dSphereClass) ctx->ball.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = *gi->texture;
        else if (class == dCylinderClass) ctx->cylinder.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = *gi->texture;
        else if (class == dCapsuleClass) {
            ctx->cylinder.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = *gi->texture;
            ctx->ball.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = *gi->texture;
        }
    }

    if (gi->visual.meshCount) {
		gi->visual.transform = matWorld;
		DrawModelTinted(gi->visual, c);
	} else {

		if (class == dBoxClass) {
			dVector3 size;
			dGeomBoxGetLengths(geom, size);
			ctx->box.transform = MatrixMultiply(MatrixScale(size[0], size[1], size[2]), matWorld);
			DrawModelTinted(ctx->box, c);
		}
		else if (class == dSphereClass) {
			float r = dGeomSphereGetRadius(geom);
			ctx->ball.transform = MatrixMultiply(MatrixScale(r*2, r*2, r*2), matWorld);
			DrawModelTinted(ctx->ball, c);
		}
		else if (class == dCylinderClass) {
			dReal l, r;
			dGeomCylinderGetParams(geom, &r, &l);
			ctx->cylinder.transform = MatrixMultiply(MatrixScale(r*2, r*2, l), matWorld);
			DrawModelTinted(ctx->cylinder, c);
		}
		else if (class == dCapsuleClass) {
			dReal l, r;
			dGeomCapsuleGetParams(geom, &r, &l);
			float d = r * 2;

			// Cylinder
			ctx->cylinder.transform = MatrixMultiply(MatrixScale(d, d, l), matWorld);
			DrawModelTinted(ctx->cylinder, c);

			// Cap 1 (Local Z+)
			Matrix matCap1 = MatrixMultiply(MatrixTranslate(0, 0, l/2), matWorld);
			ctx->ball.transform = MatrixMultiply(MatrixScale(d, d, d), matCap1);
			DrawModelTinted(ctx->ball, c);

			// Cap 2 (Local Z-)
			Matrix matCap2 = MatrixMultiply(MatrixTranslate(0, 0, -l/2), matWorld);
			ctx->ball.transform = MatrixMultiply(MatrixScale(d, d, d), matCap2);
			DrawModelTinted(ctx->ball, c);
		}

	}
}


/** @brief crates a rotor joint
 * the joint is created with the anchor at the from enties location
 *
 * @param physCtx physics context holds all the physics globals
 * @param from the anchor entity for the joint
 * @param to if this is null the joint attaches to the world and is static
 * @param axis this determines the direction of rotation for the rotor
 * you can think of it as the direction of the axle
 *
 * @note joints will get tidied up during world destruction, do destroy them
 * if you no longer require them, in the arm example a joint is created and
 * destroyed every time an object is caught and released
 */
dJointID CreateRotor(PhysicsContext* physCtx, entity* from, entity* to, Vector3 axis)
{
	// Use a Hinge to rotate around (no stops)
	dJointID rotor = dJointCreateHinge(physCtx->world, 0);
	if (!to) {
		dJointAttach(rotor, from->body, 0);
	} else {
		dJointAttach(rotor, from->body, to->body);
	}

	const dReal* pos = dBodyGetPosition(from->body);


	dJointSetHingeAnchor(rotor, pos[0], pos[1], pos[2]); // world coordinates
	dJointSetHingeAxis(rotor, axis.x, axis.y, axis.z);

	dJointSetHingeParam(rotor, dParamVel, 0.0);
	dJointSetHingeParam(rotor, dParamFMax, 100000.0); // POOOWWWWEEERRRR!

	return rotor;
}


/**
 * @brief changes the hew (tint) of all anentities geoms
 *
 * go through each geom attached to the entities body and change
 * their hew or tint
 *
 * @param ent pointer to the entity
 * @param c new colour to use
 *
 */
void SetEntityHew(entity* ent, Color c)
{
	dGeomID geom = dBodyGetFirstGeom(ent->body);
	while(geom) {
		geomInfo* gi = dGeomGetData(geom);
		gi->hew = c;
		geom = dBodyGetNextGeom(geom);
	}
}

/**
 * @brief changes the surface material of all an entities geoms
 *
 * go through each geom attached to the entities body and change
 * their surface properties
 *
 * @param ent pointer to the entity
 * @param mat new surface material to use
 *
 */
void SetEntitySurfaces(entity* ent, SurfaceMaterial* mat)
{
	dGeomID geom = dBodyGetFirstGeom(ent->body);
	while(geom) {
		geomInfo* gi = dGeomGetData(geom);
		gi->surface = mat;
		geom = dBodyGetNextGeom(geom);
	}
}

void DrawBodies(struct GraphicsContext* ctx, PhysicsContext* pctx)
{
	cnode_t* node = pctx->objList->head;

    while (node != NULL) {
		entity* e = (entity*)node->data;
		dBodyID bdy = e->body;
		DrawBodyGeoms(bdy, ctx);

		node = node->next;
	}
}

void DrawBodyGeoms(dBodyID bdy, struct GraphicsContext* ctx)
{
	dGeomID geom = dBodyGetFirstGeom(bdy);
	while(geom) {
		dGeomID next = dBodyGetNextGeom(geom);

		DrawGeom(geom, ctx);
		geom = next;
	}

}

/**
 * @brief draws the static geometries
 * 
 * As static geometries have no bodies they are handled a little different
 * this will draw all the dGeomID's that are registed on the physics context
 * static lists
 * @note all geoms on the statics list must have attached geomInfo
 * 
 * @param ctx the graphics context
 * @param pctx the physics context
 */
 
void DrawStatics(struct GraphicsContext* ctx, PhysicsContext* pctx)
{
	cnode_t* node = pctx->statics->head;

    while (node != NULL) {
		dGeomID geom = node->data;
		DrawGeom(geom, ctx);
		node = node->next;
	}
}

/** @brief frees an entity
 *
 * this is essentially destroying the entity it is no longer part of the world
 * @note if iterrating through the global entity list while calling this,
 * ensure you have stored the next node in the chain, for use at the end of the
 * while loop as obviously node->next won't work
 *
 * @param physCtx physics context
 * @param ent the entity to destroy
 */

void FreeEntity(PhysicsContext* physCtx, entity* ent)
{
	FreeBodyAndGeoms(ent->body);
	clistDeleteNode(physCtx->objList, &ent->node);
	free(ent);
}

/** @brief creates a piston using two entities as its base on extending sections
 * 
 * both entities should be positioned and oriented before calling this, they MUST NOT
 * be in exactly the same position, as the vector between them is used as the axis for
 * the piston
 * 
 * @param physCtx the physics context
 * @param entA the base entity
 * @param entB the extending entity
 * @param strength the maximum force the piston can use
 * 
 */
dJointID CreatePiston(PhysicsContext* physCtx, entity* entA, entity* entB, float strength)
{
    dJointID joint = dJointCreateSlider(physCtx->world, 0);

    dBodyID bodyA = entA->body;
    dBodyID bodyB = (entB != NULL) ? entB->body : 0; // 0 = world
    dJointAttach(joint, bodyA, bodyB);
    
    ((geomInfo*)dGeomGetData(dBodyGetFirstGeom(bodyA)))->surface = &gSurfaces[SURFACE_METAL];
    ((geomInfo*)dGeomGetData(dBodyGetFirstGeom(bodyB)))->surface = &gSurfaces[SURFACE_METAL];

    dReal axis[3];
    const dReal* posA = dBodyGetPosition(bodyA);

    if (entB != NULL) {
        const dReal* posB = dBodyGetPosition(bodyB);
        axis[0] = posA[0] - posB[0];
        axis[1] = posA[1] - posB[1];
        axis[2] = posA[2] - posB[2];
    } else {
        // Default to "Up" or a specific direction if B is world
        axis[0] = 0; axis[1] = 1; axis[2] = 0;
    }

    // Normalize and set the axis
    dReal length = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    if (length > 0) {
        dJointSetSliderAxis(joint, axis[0]/length, axis[1]/length, axis[2]/length);
    }

    dJointSetSliderParam(joint, dParamVel, 0.0);
	dJointSetSliderParam(joint, dParamFMax, strength);
	


    return joint;
}

/**
 * @brief sets the limits of travel for a piston
 * 
 * min must be smaller than max values can be negative
 * 
 * @param joint the ODE joint id of the joint you want to change
 * @param min the low end stop point
 * @param max the high end stop point
 * 
 */
void SetPistonLimits(dJointID joint, float min, float max)
{
    dJointSetSliderParam(joint, dParamLoStop, min);
    dJointSetSliderParam(joint, dParamHiStop, max);
}

void FreeMultiPiston(MultiPiston* mp)
{
	free(mp->joints);
	free(mp->sections);
	free(mp);
}

MultiPiston* CreateMultiPiston(PhysicsContext* physCtx, GraphicsContext* graphics, 
                               Vector3 pos, Vector3 direction, int count, 
                               float sectionLen, float baseWidth, float strength) 
{
    MultiPiston* mp = malloc(sizeof(MultiPiston));
    mp->count = count;
    mp->sections = malloc(sizeof(entity*) * count);
    mp->joints = malloc(sizeof(dJointID) * (count - 1));

    Vector3 dir = Vector3Normalize(direction);

    for (int i = 0; i < count; i++) {
        float scale = 1.0f - (i * 0.1f);
        Vector3 size = { sectionLen, baseWidth * scale, baseWidth * scale };
        
        // stagger each section 0.1 units along the direction vector
        Vector3 sPos = {
            pos.x + (dir.x * i * 0.1f),
            pos.y + (dir.y * i * 0.1f),
            pos.z + (dir.z * i * 0.1f)
        };

        mp->sections[i] = CreateBox(physCtx, graphics, size, sPos, (Vector3){0,0,0}, 4);
        
        // Orient the box to face the direction of travel
        SetBodyOrientation(mp->sections[i]->body, dir);

        dGeomID g = dBodyGetFirstGeom(mp->sections[i]->body);
        dGeomSetCategoryBits(g, PISTON_GROUP);
        dGeomSetCollideBits(g, WORLD_GROUP);

        if (i > 0) {
            mp->joints[i-1] = CreatePiston(physCtx, mp->sections[i-1], mp->sections[i], strength);
            SetPistonLimits(mp->joints[i-1], 0, sectionLen - 0.3f);
        }
    }
    return mp;
}

/** @brief set a MultiPiston's velocity
 * 
 * While setting all the join velocities in the piston it also
 * ensures all the bodies are active
 * 
 * @param mp the multipiston you want to change the speed of
 * @param velocity the speed you want the piston set to
 * 
 */
void SetMultiPistonVelocity(MultiPiston* mp, float velocity) {
    if (!mp) return;
    for (int i = 0; i < mp->count - 1; i++) {
        dJointSetSliderParam(mp->joints[i], dParamVel, velocity);
        dBodyEnable(mp->sections[i+1]->body);
    }
}

/**
 * @brief sets to bodies orientation to align with a vector
 * 
 * @param body the ODE body id you want to orient
 * @param direction the vector you want the body oriented along
 * 
 */
void SetBodyOrientation(dBodyID body, Vector3 direction) {
	Vector3 source = {1, 0, 0}; // The default "length" axis of your box
    direction = Vector3Normalize(direction);

    // Find the rotation axis (perpendicular to both vectors)
    Vector3 rotAxis = Vector3CrossProduct(source, direction);
    float cosTheta = Vector3DotProduct(source, direction);
    
    // Calculate the angle in radians
    float angle = acosf(fminf(1.0f, fmaxf(-1.0f, cosTheta)));

    // Handle the "Parallel" case (if target is -source, cross product is zero)
    if (Vector3Length(rotAxis) < 0.001f) {
        if (cosTheta < 0) {
            // Target is exactly opposite, rotate 180 degrees around Any perpendicular axis
            rotAxis = (Vector3){0, 1, 0};
            angle = PI;
        } else {
            // Target is same as source, no rotation needed
            return;
        }
    }

    dQuaternion q;
    dQFromAxisAndAngle(q, rotAxis.x, rotAxis.y, rotAxis.z, angle);
    
    dBodySetQuaternion(body, q);
}
/**
 * @brief pins an entity to the world
 * 
 * This joint will attempt to keep the position and orientation of
 * the body static, position and rotate the body as needed before
 * calling this
 * 
 * @param physCtx	physics context
 * @param ent	The entity to pin to the world
 * 
 * @returns the joint id of the constraint 
 */
dJointID PinEntityToWorld(PhysicsContext* physCtx, entity* ent)
{
	dJointID pin = dJointCreateFixed (physCtx->world, 0);
    dJointAttach(pin, ent->body, 0);
    dJointSetFixed(pin);
    return pin;
}

dJointID PinEntities(PhysicsContext* physCtx, entity* entA, entity* entB)
{
	dJointID pin = dJointCreateFixed (physCtx->world, 0);
    dJointAttach(pin, entA->body, entB->body);
    dJointSetFixed(pin);
    return pin;
}

void SetBodyOrientationEuler(dBodyID bdy, float p, float y, float r)
{
	dMatrix3 R;
	dRFromEulerAngles(R, p, y, r);
	dBodySetRotation(bdy, R);
}

void SetGeomOrientationEuler(dGeomID g, float p, float y, float r)
{
	dMatrix3 R;
	dRFromEulerAngles(R, p, y, r);
	dGeomSetRotation(g, R);
}

RotorPID CreateRotorPID(float p, float i, float d, float lo, float hi) 
{
    RotorPID pid = {0};
    pid.Kp = p;
    pid.Ki = i;
    pid.Kd = d;
    pid.antiWindup = 10.0f; // Prevents the motor from "over-charging" if stuck
    pid.lo = lo;
    pid.hi = hi;

    return pid;
}


void UpdateRotorPID(RotorPID* pid, dJointID joint) 
{
    if (pid->targetAngle < pid->lo) pid->targetAngle = pid->lo;
    if (pid->targetAngle > pid->hi) pid->targetAngle = pid->hi;
    float currentAngle = dJointGetHingeAngle(joint);
    
    float error = pid->targetAngle - currentAngle;
    while (error >  PI) error -= 2.0f * PI;
    while (error < -PI) error += 2.0f * PI;

    // Integral term with Anti-Windup
    pid->integral += error;
    //if (pid->integral >  pid->antiWindup) pid->integral =  pid->antiWindup;
    //if (pid->integral < -pid->antiWindup) pid->integral = -pid->antiWindup;

    // Derivative term
    float derivative = (error - pid->lastError);
    pid->lastError = error;

    // Calculate Output (Velocity)
    float outputVel = (pid->Kp * error) + 
                      (pid->Ki * pid->integral) + 
                      (pid->Kd * derivative);
    dJointSetHingeParam(joint, dParamVel, outputVel);
}

