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

// 0x01 = world 0x02 = pistons

#define WALL_GROUP   0x00000004
#define ROTOR_GROUP   0x00000008

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
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE+2, PLANE_THICKNESS, PLANE_SIZE+2);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    geomInfo* groundInfo = CreateGeomInfo(true, &graphics->groundTexture, 50.0f, 50.0f);
    groundInfo->surface = &gSurfaces[SURFACE_EARTH];
    dGeomSetData(planeGeom, groundInfo);
    dGeomSetCategoryBits(planeGeom, WALL_GROUP);
    dGeomSetCollideBits(planeGeom, WORLD_GROUP); // Ignore WALL_ROTOR (and WALL!)

	clistAddNode(physCtx->statics, planeGeom);
	
	// Create random simple objects with random textures
	for (int i = 0; i < NUM_OBJ; i++) {
		entity* e = CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)}, SHAPE_ALL & ~SHAPE_DUMBBELL);
		geomInfo* gi = dGeomGetData(dBodyGetFirstGeom(e->body));
		gi->surface = &gSurfaces[SURFACE_RUBBER];
	}
	
	
	entity* rotor = CreateBox(physCtx, graphics, 
		(Vector3){7, 6,1}, // size
		(Vector3){0,0,0}, // pos
		(Vector3){0,0,0}, // rot
		 4); // mass

	// offset the geom so it is being rotated from one end
	dGeomID rgeom = dBodyGetFirstGeom(rotor->body);
	dGeomSetOffsetPosition(rgeom, 3.5, 0, 0);
	geomInfo* gi = dGeomGetData(rgeom);
	gi->surface = &gSurfaces[SURFACE_ICE];
	dGeomSetCategoryBits(rgeom, ROTOR_GROUP);
	dGeomSetCollideBits(rgeom, WORLD_GROUP); // Ignore WALL_GROUP (and rotors too)
	
	dJointID joint_hinge = CreateRotor(physCtx, rotor, 0, (Vector3){0,1,0});

    float physTime = 0;

    //--------------------------------------------------------------------------------------
    //
    // Main game loop
    //
    //--------------------------------------------------------------------------------------
    float rotorSpeed = -1;
    bool wiper = false;
    while (!WindowShouldClose())            // Detect window close button or ESC key
    {
        //--------------------------------------------------------------------------------------
        // Update
        //----------------------------------------------------------------------------------
		
		// baked in controls (example only camera!)
		UpdateExampleCamera(graphics);
        if (IsKeyPressed(KEY_O)) {
			wiper = !wiper;
		}
		
		if (wiper) {
			float ha = dJointGetHingeAngle (joint_hinge);
			if (ha < -M_PI_2/2) rotorSpeed = fabs(rotorSpeed);
			if (ha > M_PI_2/2) rotorSpeed = -fabs(rotorSpeed);
		}
		
        if (IsKeyPressed(KEY_R)) {
			rotorSpeed = -rotorSpeed;
		}
        
        if (IsKeyDown(KEY_P)) {
			// will get slowed if pushing too much...
			dJointSetHingeParam(joint_hinge, dParamVel, rotorSpeed*4.0f);
		} else {
			dJointSetHingeParam(joint_hinge, dParamVel, rotorSpeed);
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
                FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
                entity* e = CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)}, SHAPE_ALL & ~SHAPE_DUMBBELL);
				geomInfo* gi = dGeomGetData(dBodyGetFirstGeom(e->body));
				gi->surface = &gSurfaces[SURFACE_RUBBER];
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



