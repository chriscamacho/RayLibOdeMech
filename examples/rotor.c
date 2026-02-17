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

#include "init.h"
#include "exampleCamera.h"

#define screenWidth 1920/1.2
#define screenHeight 1080/1.2

// TODO add constrain helpers to raylibODE ....!


int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------

    // Physics context, holds all physics state
    PhysicsContext* physCtx = NULL;
    
    GraphicsContext graphics = { 0 };
    InitGraphics(&graphics, screenWidth, screenHeight, "Raylib and OpenDE");
    
	initCamera(&graphics);

    physCtx = InitPhysics();

	// set up for the items in the world
    //--------------------------------------------------------------------------------------

    // Create ground "plane"
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics.groundTexture, 25.0f, 25.0f));

	clistAddNode(physCtx->statics, planeGeom);
	
	// Create random simple objects with random textures
	for (int i = 0; i < NUM_OBJ; i++) {
		addRandomPhys(physCtx, &graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)});
	}
	
	
	entity* rotor = addBox(physCtx, &graphics, 
		(Vector3){7, 1.75,.5}, // size
		(Vector3){0,1.5,0}, // pos
		(Vector3){0,0,0}, // rot
		 4); // mass

	// offset the geom so it is being rotated from one end
	dGeomID rgeom = dBodyGetFirstGeom(rotor->body);
	dGeomSetOffsetPosition(rgeom, 3.5, 0, 0);
	
	dJointID joint_hinge = createRotor(physCtx, rotor, 0, (Vector3){0,1,0});

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
		updateCamera(&graphics);
        
        if (IsKeyDown(KEY_P)) {
			// will get slowed if pushing too much...
			dJointSetHingeParam(joint_hinge, dParamVel, 3.0);
		} else {
			dJointSetHingeParam(joint_hinge, dParamVel, 0.5);
		}
        
        bool spcdn = IsKeyDown(KEY_SPACE);  // cache space key status (don't look up for each object iterration    
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
                FreeBodyAndGeoms(bdy);
                free(ent);
                clistDeleteNode(physCtx->objList, &node);
                addRandomPhys(physCtx, &graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)});

            }
            
            node = next;
        }

		// Step the physics
        //----------------------------------------------------------------------------------

        physTime = GetTime(); 
        int pSteps = stepPhysics(physCtx);
        physTime = GetTime() - physTime;    


        // Draw
        //----------------------------------------------------------------------------------
     
        BeginDrawing();

        ClearBackground(BLACK);

        BeginMode3D(graphics.camera);
			drawBodies(&graphics, physCtx);
			drawStatics(&graphics, physCtx);
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
    CleanupPhysics(physCtx);
    CleanupGraphics(&graphics);

    CloseWindow();              // Close window and OpenGL context
    
    return 0;
}



