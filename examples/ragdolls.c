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
 
#include "raylibODE.h"
#include "ragdoll.h"

#define screenWidth 1920/1.2
#define screenHeight 1080/1.2

#define NRAGDOLLS 16


int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------

    PhysicsContext* physCtx = CreatePhysics();
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
    SetupCamera(graphics);
    


	// set up for the items in the world
    //--------------------------------------------------------------------------------------

    // Create ground "plane"
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dMatrix3 R_plane;
    //dRFromAxisAndAngle(R_plane, 1, 0, -1, M_PI * 0.125);
    dRFromAxisAndAngle(R_plane, 0, 0, 1, 0);
    dGeomSetRotation(planeGeom, R_plane);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics->groundTexture, 25.0f, 25.0f));

	clistAddNode(physCtx->statics, planeGeom);

	RagDoll* rd[NRAGDOLLS];
	for (int i=0; i<NRAGDOLLS; i++) {
		rd[i] = CreateRagdoll(physCtx, graphics, GetRagdollSpawnPosition());
	}
	
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
		
		// baked in controls (example camera)
		UpdateCameraControl(graphics);
        
        bool spcdn = IsKeyDown(KEY_SPACE);  // cache space key status (don't look up for each object iterration    
        
        // Apply lifting force to ragdolls when space is held
        if (spcdn) {
            for (int i = 0; i < NRAGDOLLS; i++) {
                if (rd[i] && rd[i]->bodies[RAGDOLL_HEAD]) {
                    dBodyEnable(rd[i]->bodies[RAGDOLL_HEAD]);
                    // Calculate total mass of all body parts in the ragdoll
                    float totalMass = 0.0f;
                    for (int j = 0; j < rd[i]->bodyCount; j++) {
                        if (rd[i]->bodies[j]) {
                            dMass partMass;
                            dBodyGetMass(rd[i]->bodies[j], &partMass);
                            totalMass += partMass.mass;
                        }
                    }
                    // Lift force based on total ragdoll mass (60 * total mass)
                    float liftForce = 60.0f * totalMass;
                    dBodyAddForce(rd[i]->bodies[RAGDOLL_HEAD], 
                                  rndf(-10, 10), liftForce + rndf(-5, 5), rndf(-10, 10));
                }
            }
        }
        
        // Reset rag dolls if they fall off the plane
        for (int i = 0; i < NRAGDOLLS; i++) {
            if (rd[i] && rd[i]->bodies[RAGDOLL_TORSO]) {
                const dReal* pos = dBodyGetPosition(rd[i]->bodies[RAGDOLL_TORSO]);
                if (pos[1] < -10) {
                    // Re-create rag doll at a new random spawn position
                    FreeRagdoll(physCtx, rd[i]); // remove framework resources
                    rd[i] = CreateRagdoll(physCtx, graphics, GetRagdollSpawnPosition());
                }
            }
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
    for (int i=0; i<NRAGDOLLS; i++) {
		FreeRagdoll(physCtx,rd[i]);
	}
    FreePhysics(physCtx);
    FreeGraphics(graphics);

    CloseWindow();              // Close window and OpenGL context
    
    return 0;
}
