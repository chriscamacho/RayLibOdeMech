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

dGeomID planeGeom;

// geoms inside a trigger geom will be tinted red

void triggerCallback(dGeomID trigger, dGeomID intruder) {
	(void)trigger;
	if (intruder == planeGeom) return;
	geomInfo* gi = dGeomGetData(intruder);
	if (gi) {
		gi->hew = RED;
	}
}

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
    planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dMatrix3 R_plane;
    dRFromAxisAndAngle(R_plane, 1, 0, -1, M_PI * 0.125);
    dGeomSetRotation(planeGeom, R_plane);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics.groundTexture, 25.0f, 25.0f));

	clistAddNode(physCtx->statics, planeGeom);
	
	// Create random simple objects with random textures
	for (int i = 0; i < NUM_OBJ; i++) {
		addRandomPhys(physCtx, &graphics, (Vector3){rndf(5, 11), rndf(6, 12), rndf(-3, 3)});
	}

	// creation of a trigger area
	
	Vector3 TrigPos = (Vector3){5,-1,0};
	float TrigSize = 2.0;
	
	// interestingly because of how collision works, you can get a trigger
	// with an entity is outside the area, but only when it is colliding with 
	// another object that is colliding with the trigger ( or with another 
	// collider in a chain ending in the trigger ....! )
	
	// TODO investigate this as a potential game mechanic
	
	dGeomID TriggerGeom = dCreateSphere(physCtx->space, TrigSize);
    dGeomSetPosition(TriggerGeom, TrigPos.x, TrigPos.y, TrigPos.z);
    geomInfo* triggergi = MemAlloc(sizeof(geomInfo));
    triggergi->triggerOnCollide = triggerCallback;
    dGeomSetData(TriggerGeom, triggergi);
    
    clistAddNode(physCtx->statics, TriggerGeom);
    

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
        
        bool spcdn = IsKeyDown(KEY_SPACE);  // cache space key status (don't look up for each object iterration    
        cnode_t* node = physCtx->objList->head;

        while (node != NULL) {
			entity* ent = node->data;
            dBodyID bdy = ent->body;
            
			setEntityHew(ent, WHITE);
            
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
                // would be more efficient to just reuse the object and
                // reposition it with zeroed velocities
                // but this is used to aid testing
				FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
                addRandomPhys(physCtx, &graphics, (Vector3){rndf(5, 11), rndf(6, 12), rndf(-3, 3)});
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
			DrawSphereWires(TrigPos, TrigSize, 8, 8, RED); 
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
