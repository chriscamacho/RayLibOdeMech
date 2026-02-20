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
    PhysicsContext* physCtx = CreatePhysics();
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
    SetupCamera(graphics);
    

    // Create ground plane
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics->groundTexture, 25.0f, 25.0f));
    clistAddNode(physCtx->statics, planeGeom);

    // Initial random objects
    for (int i = 0; i < NUM_OBJ; i++) {
        CreateRandomEntity(physCtx, graphics, (Vector3){rndf(-3, 3), rndf(6, 12), rndf(-3, 3)}, SHAPE_ALL);
    }

    while (!WindowShouldClose())
    {
        UpdateExampleCamera(graphics);

        // placing new objects
        Vector3 forward = Vector3Normalize(Vector3Subtract(graphics->camera.target, graphics->camera.position));
        Vector3 spawnPos = Vector3Add(graphics->camera.position, Vector3Scale(forward, 3.0f));
        Vector3 defaultRot = { 0, GetCameraYaw(), 0 };

        if (IsKeyPressed(KEY_ONE))   CreateBox(physCtx, graphics, (Vector3){0.5, 0.5, 0.5}, spawnPos, defaultRot, 10.0f);
        if (IsKeyPressed(KEY_TWO))   CreateSphere(physCtx, graphics, 0.4f, spawnPos, defaultRot, 10.0f);
        if (IsKeyPressed(KEY_THREE)) CreateCylinder(physCtx, graphics, 0.3f, 1.0f, spawnPos, defaultRot, 10.0f);
        if (IsKeyPressed(KEY_FOUR))  CreateCapsule(physCtx, graphics, 0.3f, 1.0f, spawnPos, defaultRot, 10.0f);
        if (IsKeyPressed(KEY_FIVE))  CreateDumbbell(physCtx, graphics, 0.1f, 1.0f, 0.3f, spawnPos, defaultRot, 10.0f);

        // picking / pushing
        Vector3 hitPoint = { 0 };
        entity* picked = PickEntity(physCtx, graphics, &hitPoint);
        bool hasHit = (picked != NULL);

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && hasHit && picked->body != NULL) {
            dBodyEnable(picked->body);
            Vector3 dir = Vector3Normalize(Vector3Subtract(hitPoint, graphics->camera.position));
            Vector3 force = Vector3Scale(dir, 500.0f); 
            dBodyAddForceAtPos(picked->body, force.x, force.y, force.z, hitPoint.x, hitPoint.y, hitPoint.z);
        }

		bool spcdn = IsKeyDown(KEY_SPACE);  // cache space key status (don't look up for each object iterration  

        // check world bounds and add force if needed
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


            if(pos[1] < -10) {
                FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
            }
            node = next;
        }

        StepPhysics(physCtx);

        // drawing
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(graphics->camera);
                DrawBodies(graphics, physCtx);
                DrawStatics(graphics, physCtx);

                // Target indicator (Red dot)
                if (hasHit) { 
					DrawSphere(hitPoint, 0.05f, RED);
                } else {
					// Spawn point preview (Grey dot)
					DrawSphereEx(spawnPos, 0.05f, 8, 8, DARKGRAY);
				}
            EndMode3D();

            DrawText("1-5: Spawn Objects | LMB: Push | Space: Apply Force", 10, 40, 20, RAYWHITE);

        EndDrawing();
    }

    FreePhysics(physCtx);
    FreeGraphics(graphics);
    CloseWindow();
    return 0;
}
