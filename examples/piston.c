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
	for (int i = 0; i < NUM_OBJ; i++) {
		CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)});
	}


	// manually create a 3 section piston
	// TLDR skip down to the multi piston creation !
	entity* box1 = CreateBox(physCtx, graphics,(Vector3){4,1,1}, (Vector3){0,1.6,0}, (Vector3){0,0,0}, 2);
	entity* box2 = CreateBox(physCtx, graphics,(Vector3){4,.9,.9}, (Vector3){.1,1.7,.2}, (Vector3){0,0,0}, 2);
	entity* box3 = CreateBox(physCtx, graphics,(Vector3){4,.8,.8}, (Vector3){.2,1.8,.4}, (Vector3){0,0,0}, 2);

	// the direction each box is offset - the axis of the piston
	Vector3 dir = (Vector3){.1,.1,.2};
	dir = Vector3Normalize(dir);
	
	SetBodyOrientation(box1->body, dir); 
	SetBodyOrientation(box2->body, dir); 
	SetBodyOrientation(box3->body, dir); 
	// anchor box1 to the world
    PinEntityToWorld(physCtx, box1);


    dJointID piston1 = CreatePiston(physCtx, box1, box2,1000);
    SetPistonLimits(piston1, 0, 3.7);
    dJointID piston2 = CreatePiston(physCtx, box2, box3,1000);
    SetPistonLimits(piston2, 0, 3.7);
    
    
    // because box 1,2 and 3 intersect we must filter out their collisions
    // normally with joints the two attached bodies don't collide, however
    // this doesn't help with box 1 vs box 3 for example
    dGeomID g1 = dBodyGetFirstGeom(box1->body);
    dGeomID g2 = dBodyGetFirstGeom(box2->body);
    dGeomID g3 = dBodyGetFirstGeom(box3->body);

	dGeomSetCategoryBits(g1, PISTON_GROUP);
	dGeomSetCollideBits(g1, WORLD_GROUP);

	dGeomSetCategoryBits(g2, PISTON_GROUP);
	dGeomSetCollideBits(g2, WORLD_GROUP);

	dGeomSetCategoryBits(g3, PISTON_GROUP);
	dGeomSetCollideBits(g3, WORLD_GROUP);    
	

    
    // create a multi piston and pin it to the world
    MultiPiston* mp = CreateMultiPiston(physCtx, graphics, 
                               (Vector3){4,1.4,0}, 	// position
                               (Vector3){.5,.5,0}, 	// direction (or axis) of the piston
                               6, 					// number of sections
                               0.5, 				// distance each section travels
                               2, 					// base width
                               1000);				// stength

    PinEntityToWorld(physCtx, mp->sections[0]);
    
    
    
    while (!WindowShouldClose())
    {
        dJointSetSliderParam(piston1, dParamVel, 0.0);
        dJointSetSliderParam(piston2, dParamVel, 0.0);
        SetMultiPistonVelocity(mp, 0);
        
        float pSpeed = 1;
        if (IsKeyDown(KEY_LEFT_ALT)) pSpeed = 4;
        
        if (IsKeyDown(KEY_I)) {
            dJointSetSliderParam(piston1, dParamVel, -pSpeed);
            dJointSetSliderParam(piston2, dParamVel, -pSpeed);
            dBodyEnable(box2->body);
			SetMultiPistonVelocity(mp, -pSpeed/16.0);
        }
        if (IsKeyDown(KEY_O)) {
            dJointSetSliderParam(piston1, dParamVel, pSpeed);
            dJointSetSliderParam(piston2, dParamVel, pSpeed);
            dBodyEnable(box2->body);
			SetMultiPistonVelocity(mp, pSpeed/16.0);
        }
        
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
                CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)});
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

            DrawText("Press O and P to move the piston", 10, 40, 20, RAYWHITE);

        EndDrawing();
    }

    FreePhysics(physCtx);
    FreeGraphics(graphics);
    CloseWindow();
    return 0;
}
