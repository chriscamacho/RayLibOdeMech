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
 
#include <stdlib.h>
#include <stdbool.h>

#include "init.h"
#include "exampleCamera.h"

#define screenWidth 1920/1.2
#define screenHeight 1080/1.2

// the floor
dGeomID planeGeom = 0;

dJointID attachment = 0;
entity* grabber = 0;

// Physics context, holds all physics state
PhysicsContext* physCtx = NULL;

// TODO look at having user data passed to triggerCallback somehow to avoid globals...

// this will end up joining to the first collided in a frame (could be any!)
void triggerCallback(dGeomID trigger, dGeomID intruder) {
	(void)trigger;
	if (intruder == planeGeom) return;
	geomInfo* gi = dGeomGetData(intruder);
	if (gi) {
		gi->hew = RED;
		if (!attachment && !IsKeyDown(KEY_G)) {
			dBodyID bdy = dGeomGetBody(intruder);
			attachment = dJointCreateBall (physCtx->world, 0);
			
			const dReal* pos = dBodyGetPosition(bdy);
			
			dJointSetBallAnchor (attachment, pos[1], pos[2], pos[3]);
			dJointAttach(attachment, bdy, grabber->body);
		}
	}
}


int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------


    
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE");
    SetupCamera(graphics);
    physCtx = CreatePhysics();

	// set up for the items in the world
    //--------------------------------------------------------------------------------------

    // Create ground "plane"
    planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics->groundTexture, 25.0f, 25.0f));

	clistAddNode(physCtx->statics, planeGeom);
	
	
	// Create random simple objects with random textures
	for (int i = 0; i < NUM_OBJ/4; i++) {
		CreateRandomEntity(physCtx, graphics, (Vector3){rndf(5, 11), rndf(6, 12), rndf(-3, 3)});
	}

	// creation of the arm

	entity* platform = CreateCylinder(physCtx, graphics, 1,//radius
		.5,//length
		(Vector3){0,1.5,0}, // pos
		(Vector3){0,0,0}, // rot
		4); // mass



	dMatrix3 Rr;
	dRFromAxisAndAngle(Rr, 1, 0, 0, M_PI/2.0);
	dBodySetRotation (platform->body, Rr);

	dJointID platform_joint = CreateRotor(physCtx, platform,0, (Vector3){0,1,0});
	
	// platform boot is just cosmetic to hide the joint
	dGeomID plat_boot = CreateSphereGeom(physCtx, graphics, .6, (Vector3){0,0,0});
	dGeomSetBody(plat_boot, platform->body);
	dGeomSetOffsetPosition(plat_boot, 0, 0, -0.5);
	geomInfo* pbgi = dGeomGetData(plat_boot);
	pbgi->collidable = false;


	
	entity* rotor2 = CreateBox(physCtx, graphics, 
		(Vector3){.5, 4,.5}, // size
		(Vector3){0,2,0}, // pos
		(Vector3){0,0,0}, // rot
		 4); // mass

	dGeomID rgeom2 = dBodyGetFirstGeom(rotor2->body);
	dGeomSetOffsetPosition(rgeom2, 0, 2, 0);

	dJointID rotor_joint2 = CreateRotor(physCtx, rotor2, platform,(Vector3){1,0,0});
	
	
	entity* rotor3 = CreateBox(physCtx, graphics, 
		(Vector3){.5, 4,.5}, // size
		(Vector3){0,6,0}, // pos
		(Vector3){0,0,0}, // rot
		 4); // mass

	dGeomID rgeom3 = dBodyGetFirstGeom(rotor3->body);
	dGeomSetOffsetPosition(rgeom3, 0, 2, 0);

	dJointID rotor_joint3 = CreateRotor(physCtx, rotor3, rotor2,(Vector3){1,0,0});
	
	
	// a ball for a grabber includes a sensor to grab stuff automatically
	grabber = CreateSphere(physCtx, graphics, .5, (Vector3){0,10.5,0}, (Vector3){0,0,0}, 1);
	dBodySetAngularDamping(grabber->body, 0.5f); // make it stiffer
	
	dJointID grabber_joint = dJointCreateBall (physCtx->world, 0);
	dJointSetBallAnchor (grabber_joint, 0, 10.5, 0);
	dJointAttach(grabber_joint, grabber->body, rotor3->body);

	
	// create just a geom
	dGeomID grab_sensor = CreateSphereGeom(physCtx, graphics, .6, (Vector3){0,0,0});
	
	// set the trigger callback for the geom its now invisible 
	// and other things don't collide with it - but it will register "collisions"
    geomInfo* triggergi = dGeomGetData(grab_sensor);
    triggergi->triggerOnCollide = triggerCallback;
   	dGeomSetBody(grab_sensor, grabber->body); // attach the sensor to the grabber
	
    float physTime = 0;

    //--------------------------------------------------------------------------------------
    //
    // Main game loop
    //
    //--------------------------------------------------------------------------------------
    while (!WindowShouldClose())            // Detect window close button or ESC key
    {
        //--------------------------------------------------------------------------------------
        // Update
        //----------------------------------------------------------------------------------
		
		// baked in controls (example only camera!)
		UpdateExampleCamera(graphics);
		
		if (IsKeyDown(KEY_G)) {
			if (attachment) {
				dJointDestroy(attachment);
				attachment = 0;
			}
		}
        
        dJointSetHingeParam(platform_joint, dParamVel, 0.0);
        dJointSetHingeParam(rotor_joint2, dParamVel, 0.0);
        dJointSetHingeParam(rotor_joint3, dParamVel, 0.0);

        if (IsKeyDown(KEY_I)) {
			dJointSetHingeParam(platform_joint, dParamVel, 1.0);
			dBodyEnable(platform->body);
		}
		
		if (IsKeyDown(KEY_O)) {
			dJointSetHingeParam(platform_joint, dParamVel, -1.0);
			dBodyEnable(platform->body);
		}
        
        if (IsKeyDown(KEY_K)) {
			dJointSetHingeParam(rotor_joint2, dParamVel, 1.0);
			dBodyEnable(rotor2->body);
		}
		
		if (IsKeyDown(KEY_L)) {
			dJointSetHingeParam(rotor_joint2, dParamVel, -1.0);
			dBodyEnable(rotor2->body);
		}
        
        if (IsKeyDown(KEY_COMMA)) {
			dJointSetHingeParam(rotor_joint3, dParamVel, 1.0);
			dBodyEnable(rotor3->body);
		}
		
		if (IsKeyDown(KEY_PERIOD)) {
			dJointSetHingeParam(rotor_joint3, dParamVel, -1.0);
			dBodyEnable(rotor3->body);
		}
		        
        bool spcdn = IsKeyDown(KEY_SPACE);  // cache space key status (don't look up for each object iterration    
        cnode_t* node = physCtx->objList->head;

        while (node != NULL) {
			entity* ent = node->data;
            dBodyID bdy = ent->body;
            cnode_t* next = node->next; // get the next node now in case we delete this one
            
            // reset entities for setting by collision
            SetEntityHew(ent, WHITE);
            
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
                CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)});
            }
            
            node = next;
        }

		// Step the physics
        //----------------------------------------------------------------------------------

        physTime = GetTime(); 
        int pSteps = StepPhysics(physCtx);
        physTime = GetTime() - physTime;    


        // Draw
        //----------------------------------------------------------------------------------
     
        BeginDrawing();

        ClearBackground(BLACK);

        BeginMode3D(graphics->camera);
			DrawBodies(graphics, physCtx);
			DrawStatics(graphics, physCtx);
        EndMode3D();


        if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
        DrawText(TextFormat("%2i FPS", GetFPS()), 10, 20, 20, WHITE);
        DrawText("Press SPACE to apply force to objects", 10, 60, 20, WHITE);

        DrawText(TextFormat("Phys steps per frame %i",pSteps), 10, 120, 20, WHITE);
        DrawText(TextFormat("Phys time per frame %f",physTime), 10, 140, 20, WHITE);
        DrawText(TextFormat("total time per frame %f",GetFrameTime()), 10, 160, 20, WHITE);

        EndDrawing();

    }

    
    // De-Initialization
    //--------------------------------------------------------------------------------------
    FreePhysics(physCtx);
    FreeGraphics(graphics);

    CloseWindow();              // Close window and OpenGL context
    
    return 0;
}



