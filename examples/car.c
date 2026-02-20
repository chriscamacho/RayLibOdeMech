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

#include "init.h"
#include "exampleCamera.h"


#define screenWidth 1920/1.2
#define screenHeight 1080/1.2




int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------

    // Physics context, holds all physics state
    PhysicsContext* physCtx = CreatePhysics();
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE");
	SetupCamera(graphics);

	// set up for the items in the world
    //--------------------------------------------------------------------------------------

	// the graphical mesh for the ground must be unloaded by the user
	Model ground = LoadModel("data/ground2.obj");
	
	// framework looks after the physics stuff and rendering
	CreateStaticTrimesh(physCtx, graphics, ground, &graphics->groundTexture, 2.5f);


	// vehicle* CreateVehicle(PhysicsContext* pctx, struct GraphicsContext* ctx, Vector3 pos, Vector3 carScale, float wheelRadius, float wheelWidth);
	vehicle* car = CreateVehicle(physCtx, graphics, (Vector3){15, 6, 15},(Vector3){3.5, 0.5, 2.4}, .8, .6);


	// Create random simple objects with random textures
	for (int i = 0; i < NUM_OBJ; i++) {
		CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)}, SHAPE_ALL);
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
		
		// baked in controls (example only camera!)
		UpdateVehicleCamera(graphics, car);
        
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
                    float f = rndf(80,200) * mass.mass;
                    dBodyAddForce(bdy, rndf(-f,f), f*10, rndf(-f,f));
                }
            }

            
            if(pos[1]<-10) {
                // would be more efficient to just reuse the object and
                // reposition it with zeroed velocities
                // but this is used to aid testing
				FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
                CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)}, SHAPE_ALL);
            }
            
            node = next;
        }
		
		float accel = 0;
		if (IsKeyDown(KEY_DOWN)) accel = -100.0f;
		if (IsKeyDown(KEY_UP)) accel = 100.0f;
		float steer = 0;
		if (IsKeyDown(KEY_LEFT)) steer = -0.4f;
		if (IsKeyDown(KEY_RIGHT)) steer = 0.4f;

		UpdateVehicle(car, accel, steer);
    
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
    
    FreeVehicle(physCtx, car);
    UnloadModel(ground);

    FreePhysics(physCtx);
    FreeGraphics(graphics);

    CloseWindow();              // Close window and OpenGL context
    
    return 0;
}
