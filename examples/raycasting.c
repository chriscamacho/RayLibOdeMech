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
	
	graphics->camera.position.y = graphics->camera.position.y  - 8.f;

	// set up for the items in the world
    //--------------------------------------------------------------------------------------

    // Create ground "plane"
    
    Texture dotTex = LoadTexture("data/dot.png");
    
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    SetGeomOrientationEuler(planeGeom, -22.5f, 0, 0);
    
    geomInfo* groundInfo = CreateGeomInfo(true, &dotTex, 25.0f, 25.0f);
    groundInfo->surface = &gSurfaces[SURFACE_ICE];
    dGeomSetData(planeGeom, groundInfo);
    
	clistAddNode(physCtx->statics, planeGeom);

	
	RayCast* redCast = CreateRayCast(12, (Vector3){-5,.5,0},(Vector3){1,0,0}, 3);
	// pre calculate end point as we're not moving the ray around
	// (used for drawing the visual)
	Vector3 redEnd = Vector3Scale(redCast->direction, redCast->length);
	redEnd = Vector3Add(redEnd, redCast->position);

	RayCast* greenCast = CreateRayCast(12, (Vector3){-2,.5,0},(Vector3){1,0,0}, 3);
	Vector3 greenEnd = Vector3Scale(greenCast->direction, greenCast->length);
	greenEnd = Vector3Add(greenEnd, greenCast->position);

	RayCast* blueCast = CreateRayCast(12, (Vector3){-2,-0.5,2},(Vector3){1,0,0}, 7);
	Vector3 blueEnd = Vector3Scale(blueCast->direction, blueCast->length);
	blueEnd = Vector3Add(blueEnd, blueCast->position);
		
    float physTime = 0;
	int frameCount = 0;
    
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
        
        cnode_t* node = physCtx->objList->head;


		
        while (node != NULL) {
			entity* ent = node->data;
            dBodyID bdy = ent->body;
                        
            cnode_t* next = node->next; // get the next node now in case we delete this one
            const dReal* pos = dBodyGetPosition(bdy);

            if(pos[1]<-10) {
				FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
            }
            
            node = next;
        }

        if (frameCount % 60 == 0) {
			for (int i=0; i < 3; i++) {
				Vector3 pos = (Vector3){-4.0f + i *4.0f, 6.5, -7.5};
				entity* box = CreateBox(physCtx, graphics, (Vector3){1,1,1}, pos, Vector3Zero(), 20.f);
				SetEntitySurfaces(box, &gSurfaces[SURFACE_ICE]);
			}
		}

		// Step the physics
        //----------------------------------------------------------------------------------

        physTime = GetTime(); 
        int pSteps = StepPhysics(physCtx);
        physTime = GetTime() - physTime;    

		CastRay(physCtx, redCast);
		for (int i = 0; i < redCast->count; i++) {
			geomInfo* gi = dGeomGetData(redCast->hits[i].geom);
			if (gi) gi->hew = RED;
		}
		CastRay(physCtx, greenCast);
		for (int i = 0; i < greenCast->count; i++) {
			geomInfo* gi = dGeomGetData(greenCast->hits[i].geom);
			if (gi) gi->hew = GREEN;
		}
		
		CastRay(physCtx, blueCast);
		for (int i = 0; i < blueCast->count; i++) {
			geomInfo* gi = dGeomGetData(blueCast->hits[i].geom);
			if (gi) gi->hew = BLUE;
		}
        // Draw
        //----------------------------------------------------------------------------------
		
        BeginDrawing();

        ClearBackground(BLACK);

        BeginMode3D(graphics->camera);
			DrawBodies(graphics, physCtx);
			DrawStatics(graphics, physCtx);
			
			// rays probably more useful invisible but in this example we show them
			DrawLine3D(redCast->position, redEnd, RED);
			DrawLine3D(greenCast->position, greenEnd, GREEN);
			DrawLine3D(blueCast->position, blueEnd, BLUE);
        EndMode3D();


        if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
        DrawText(TextFormat("%2i FPS", GetFPS()), 10, 20, 20, WHITE);
        DrawText(TextFormat("Phys steps per frame %i",pSteps), 10, 120, 20, WHITE);
        DrawText(TextFormat("Phys time per frame %f",physTime), 10, 140, 20, WHITE);
        DrawText(TextFormat("total time per frame %f",GetFrameTime()), 10, 160, 20, WHITE);

        EndDrawing();

		frameCount++;
    }

    UnloadTexture(dotTex);
    free(redCast);
    free(greenCast);
    free(blueCast);
    // De-Initialization
    //--------------------------------------------------------------------------------------
    FreePhysics(physCtx);
    FreeGraphics(graphics);

    CloseWindow();              // Close window and OpenGL context
    
    return 0;
}
