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
    PhysicsContext* physCtx = NULL;
    GraphicsContext graphics = { 0 };
    InitGraphics(&graphics, screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
    initCamera(&graphics);
    physCtx = InitPhysics();

    // Create ground plane
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics.groundTexture, 25.0f, 25.0f));
    clistAddNode(physCtx->statics, planeGeom);

	// Create random simple objects with random textures
	for (int i = 0; i < NUM_OBJ; i++) {
		addRandomPhys(physCtx, &graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)});
	}

	entity* box1 = addBox(physCtx, &graphics,(Vector3){4,1,1}, (Vector3){0,2,0}, (Vector3){0,0,0}, 2);
	entity* box2 = addBox(physCtx, &graphics,(Vector3){4,.9,.9}, (Vector3){.1,2,0}, (Vector3){0,0,0}, 2);
	entity* box3 = addBox(physCtx, &graphics,(Vector3){4,.8,.8}, (Vector3){.2,2,0}, (Vector3){0,0,0}, 2);

	// anchor box1 to the world
    dJointID pin1 = dJointCreateFixed (physCtx->world, 0);
    dJointAttach(pin1, box1->body, 0);
    dJointSetFixed (pin1);


    dJointID piston1 = CreatePiston(physCtx, box1, box2);
    SetPistonLimits(piston1, 0, 3.9);
    
    dJointID piston2 = CreatePiston(physCtx, box2, box3);
    SetPistonLimits(piston2, 0, 3.9);
    
    
    // because box 1,2 and 3 intersect we must filter out their collision
    // normally with joints the two attached bodies don't collide, however
    // this doesn't help with box 1 vs box 3 for example
	#define WORLD         0x0001
    #define PISTON_GROUP  0x0002
    
    dGeomID g1 = dBodyGetFirstGeom(box1->body);
    dGeomID g2 = dBodyGetFirstGeom(box2->body);
    dGeomID g3 = dBodyGetFirstGeom(box3->body);

	dGeomSetCategoryBits(g1, PISTON_GROUP);
	dGeomSetCollideBits(g1, WORLD);

	dGeomSetCategoryBits(g2, PISTON_GROUP);
	dGeomSetCollideBits(g2, WORLD);

	dGeomSetCategoryBits(g3, PISTON_GROUP);
	dGeomSetCollideBits(g3, WORLD);    
    
    
    while (!WindowShouldClose())
    {
        dJointSetSliderParam(piston1, dParamVel, 0.0);
        dJointSetSliderParam(piston2, dParamVel, 0.0);
        
        float pSpeed = 1;
        if (IsKeyDown(KEY_LEFT_ALT)) pSpeed = 32;
        
        if (IsKeyDown(KEY_I)) {
            dJointSetSliderParam(piston1, dParamVel, -pSpeed);
            dJointSetSliderParam(piston2, dParamVel, -pSpeed);
            dBodyEnable(box1->body);
        }
        if (IsKeyDown(KEY_O)) {
            dJointSetSliderParam(piston1, dParamVel, pSpeed);
            dJointSetSliderParam(piston2, dParamVel, pSpeed);
            dBodyEnable(box1->body);
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
                addRandomPhys(physCtx, &graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)});
            }
            
            node = next;
        }

        updateCamera(&graphics);
        stepPhysics(physCtx);

        // drawing
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(graphics.camera);
                drawBodies(&graphics, physCtx);
                drawStatics(&graphics, physCtx);
            EndMode3D();

            DrawText("Press O and P to move the piston", 10, 40, 20, RAYWHITE);

        EndDrawing();
    }

    CleanupPhysics(physCtx);
    CleanupGraphics(&graphics);
    CloseWindow();
    return 0;
}
