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

/** @file pidJoint.c */

#include "init.h"
#include "exampleCamera.h"

#define screenWidth 1920/1.2
#define screenHeight 1080/1.2



int main(void)
{
	// init
    PhysicsContext* physCtx = CreatePhysics();
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
    SetupCamera(graphics);

    // Create ground plane
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics->groundTexture, 25.0f, 25.0f));
    clistAddNode(physCtx->statics, planeGeom);

	// Create random simple objects with random textures
	//for (int i = 0; i < NUM_OBJ; i++) {
	//	CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)}, SHAPE_ALL);
	//}


	entity* rotor = CreateCylinder(physCtx, graphics, 2.f, 1.f, (Vector3){0,.6,0}, (Vector3){M_PI_2,0,0}, 100);
	dJointID rotorJoint = CreateRotor(physCtx, rotor, 0, (Vector3){0,1,0});
	dBodySetAutoDisableFlag(rotor->body, 0);
	
	entity* elivate = CreateSphere(physCtx, graphics, 1.f, (Vector3){0,1.6,0}, (Vector3){M_PI_2,0,0}, 1);
	dJointID elivateJoint = CreateRotor(physCtx, elivate, rotor, (Vector3){1,0,0});
	dBodySetAutoDisableFlag(elivate->body, 0);

    
    // create a multi piston and pin it to the world
    // extends from .22 to 5.54 with the furthest extent being plus section height / 2
    // bottom for first section is section[0] position minus section height / 2
    MultiPiston* upperArm = CreateMultiPiston(physCtx, graphics, 
                               (Vector3){0,3.5,0}, 	// position
                               (Vector3){0,1,0}, 	// direction (or axis) of the piston
                               4, 					// number of sections
                               2, 				// distance each section travels
                               1, 					// base width
                               1000);				// stength
	dBodySetAutoDisableFlag(upperArm->sections[0]->body, 0);
    
    PinEntities(physCtx, elivate, upperArm->sections[0]);
    
    entity* elbow = CreateSphere(physCtx, graphics, 1.f, (Vector3){0,5.8,0}, (Vector3){M_PI_2,0,0}, 1);
	dJointID elbowJoint = CreateRotor(physCtx, elbow, upperArm->sections[upperArm->count-1], (Vector3){1,0,0});
	dBodySetAutoDisableFlag(elbow->body, 0);

	
	MultiPiston* foreArm = CreateMultiPiston(physCtx, graphics, 
                               (Vector3){0,7.8,0}, 	// position
                               (Vector3){0,1,0}, 	// direction (or axis) of the piston
                               4, 					// number of sections
                               2, 				// distance each section travels
                               1, 					// base width
                               1000);				// stength
	dBodySetAutoDisableFlag(foreArm->sections[0]->body, 0);
    
    PinEntities(physCtx, elbow, foreArm->sections[0]);
    
    RotorPID elivatePID = CreateRotorPID(100.0f, 1.6f, 3.f, -M_PI_2 + 0.2f, M_PI_2 - 0.2f);
    RotorPID elbowPID = CreateRotorPID(100.0f, 1.6f,3.f, -M_PI_2 + 0.2f, M_PI_2 - 0.2f);
    
    entity* grabber = CreateSphere(physCtx, graphics, 1.f, (Vector3){0,10.2f,0}, (Vector3){M_PI_2,0,0}, 1);
    PinEntities(physCtx, grabber, foreArm->sections[foreArm->count-1]);
    
    while (!WindowShouldClose())
    {
		
		dJointSetHingeParam(rotorJoint, dParamVel, 0);
        if (IsKeyDown(KEY_I)) dJointSetHingeParam(rotorJoint, dParamVel, 1.0);
		if (IsKeyDown(KEY_O)) dJointSetHingeParam(rotorJoint, dParamVel, -1.0);
		
        if (IsKeyDown(KEY_K)) elivatePID.targetAngle += 0.01;
		if (IsKeyDown(KEY_L)) elivatePID.targetAngle -= 0.01;
		UpdateRotorPID(&elivatePID, elivateJoint);

		SetMultiPistonVelocity(upperArm, 0);
        if (IsKeyDown(KEY_COMMA)) SetMultiPistonVelocity(upperArm, 1);
		if (IsKeyDown(KEY_PERIOD)) SetMultiPistonVelocity(upperArm, -1);
		
        if (IsKeyDown(KEY_Y)) elbowPID.targetAngle += 0.01;
		if (IsKeyDown(KEY_U)) elbowPID.targetAngle -= 0.01;
		UpdateRotorPID(&elbowPID, elbowJoint);

		SetMultiPistonVelocity(foreArm, 0);
        if (IsKeyDown(KEY_H)) SetMultiPistonVelocity(foreArm, 1);
		if (IsKeyDown(KEY_J)) SetMultiPistonVelocity(foreArm, -1);
        
        bool spcdn = IsKeyDown(KEY_SPACE); 
		cnode_t* node = physCtx->objList->head;
        while (node != NULL) {
			entity* ent = node->data;
            dBodyID bdy = ent->body;
            cnode_t* next = node->next; // get the next node now in case we delete this one
            const dReal* pos = dBodyGetPosition(bdy);
			
			if (spcdn) {
                // apply force if the space key is held
                const dReal* v = dBodyGetLinearVel(bdy);
                if (v[1] < 10 && pos[1]<10) { // cap upwards velocity and don't let it get too high
                    dBodyEnable (bdy); // case its gone to sleep
                    dMass mass;
                    dBodyGetMass (bdy, &mass);
                    // give some object more force than others
                    float f = rndf(8,20) * mass.mass;
                    dBodyAddForce(bdy, rndf(-f,f), f*10, rndf(-f,f));
                }
            }
            
            if(pos[1]<-10) {
                FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
                CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)}, SHAPE_ALL);
            }
            
            node = next;
        }

        UpdateExampleCamera(graphics);
        StepPhysics(physCtx);

        // drawing
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(graphics->camera);
                DrawBodies(graphics, physCtx);
                DrawStatics(graphics, physCtx);
            EndMode3D();

            //DrawText("Press O and P to move the piston", 10, 40, 20, RAYWHITE);
            float ea = dJointGetHingeAngle(elivateJoint);
            DrawText(TextFormat("Elivate angle %f",ea), 10, 120, 20, WHITE);
            ea = dJointGetHingeAngle(rotorJoint);
            DrawText(TextFormat("Rotor angle %f",ea), 10, 140, 20, WHITE);
            ea = dJointGetHingeAngle(elbowJoint);
            DrawText(TextFormat("elbow angle %f",ea), 10, 160, 20, WHITE);

			{
				const dReal* op1 = dBodyGetPosition(foreArm->sections[0]->body);
				const dReal* op2 = dBodyGetPosition(foreArm->sections[foreArm->count-1]->body);
				Vector3 dv = (Vector3){op1[0]-op2[0], op1[1]-op2[1], op1[2]-op2[2]};
				float l = Vector3Length(dv);
				DrawText(TextFormat("forarm length %f", l), 10, 180, 20, WHITE);
			}

			{
				const dReal* op1 = dBodyGetPosition(upperArm->sections[0]->body);
				const dReal* op2 = dBodyGetPosition(upperArm->sections[upperArm->count-1]->body);
				Vector3 dv = (Vector3){op1[0]-op2[0], op1[1]-op2[1], op1[2]-op2[2]};
				float l = Vector3Length(dv);
				DrawText(TextFormat("upperArm length %f", l), 10, 200, 20, WHITE);
			}
			
			{
				const dReal* op1 = dBodyGetPosition(grabber->body);
				DrawText(TextFormat("grabber position %f, %f, %f", op1[0], op1[1], op1[2]), 10, 220, 20, WHITE);
			}

        EndDrawing();
    }


	FreeMultiPiston(upperArm);
    FreePhysics(physCtx);
    FreeGraphics(graphics);
    CloseWindow();
    return 0;
}
