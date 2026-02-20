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
 
#include <math.h>

#include "init.h"
#include "exampleCamera.h"

#define screenWidth 1920/1.2
#define screenHeight 1080/1.2

static const Vector3 gravPoint = {0.0f,8.0f,0.0f};
static const float gravSize = 6.0f; 
static const Vector3 G = {0.0f ,-9.8f, 0.0f};

Vector3 GetSpawnOnSphereBorder(Vector3 center, float radius) 
{
    Vector3 randDir = { 
        rndf(-100, 100), 
        rndf(-100, 100), 
        rndf(-100, 100) 
    };
    
    randDir = Vector3Normalize(randDir);
    
    float spawnDist = radius * 0.9f; 
    Vector3 offset = Vector3Scale(randDir, spawnDist);
    return Vector3Add(center, offset);
}

void CreateOrbiter(PhysicsContext* physCtx, GraphicsContext* graphics)
{
	Vector3 p = GetSpawnOnSphereBorder(gravPoint, gravSize * 0.9f); 
	entity* e = CreateRandomEntity(physCtx, graphics, p);
	
	Vector3 toSpawn = Vector3Subtract(p, gravPoint);
	Vector3 fDir = Vector3Normalize(toSpawn);
	
	// Create a random tangent (perpendicular to the radius)
	Vector3 axis = { rndf(-1,1), rndf(-1,1), rndf(-1,1)}; 
	if (fabsf(Vector3DotProduct(fDir, axis)) > 0.9f) axis = (Vector3){ 1, 0, 0 };
	
	Vector3 tangent = Vector3Normalize(Vector3CrossProduct(fDir, axis));

	// Calculate Orbital Speed: v = sqrt( (GravityAccel * Radius) )
	// This gives a roughly stable circular orbit
	float orbitSpeed = sqrtf(Vector3Length(G) * Vector3Length(toSpawn));

	// Apply some(!) of the initial linear velocity to the ODE body
	Vector3 velocity = Vector3Scale(tangent, orbitSpeed / 8.0f);
	dBodySetLinearVel(e->body, velocity.x, velocity.y, velocity.z);
}



int main(void)
{
	// init
    PhysicsContext* physCtx = CreatePhysics();
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
    SetupCamera(graphics);

	
	// Create random simple objects with random textures
	// deliberatly make them all fall out of the world
	for (int i = 0; i < NUM_OBJ / 6; i++) {
		CreateOrbiter(physCtx, graphics);
	}
	
	dVector3 g;
	dWorldGetGravity(physCtx->world, g);
	
    while (!WindowShouldClose())
    {
        UpdateExampleCamera(graphics);
        StepPhysics(physCtx);

		bool spcdn = IsKeyPressed(KEY_SPACE);  // cache space key status (don't look up for each object iterration  
		cnode_t* node = physCtx->objList->head;
        while (node != NULL) 
        {
			entity* ent = node->data;
            dBodyID bdy = ent->body;
            cnode_t* next = node->next; // get the next node now in case we delete this one
            const dReal* pos = dBodyGetPosition(bdy);
            
			Vector3* pv = (Vector3*)pos;	// not sure if I like this!
            Vector3 dist = Vector3Subtract(gravPoint, *pv);
            float d = Vector3Length(dist);
            
			if (spcdn) {
                // apply an outward force if the space key is pressed

				dBodyEnable (bdy); // case its gone to sleep
				dMass mass;
				dBodyGetMass (bdy, &mass);
				// give some object more force than others
				float f = rndf(100,10) * mass.mass;
				Vector3 fv = Vector3Normalize(dist);
				fv = Vector3Negate(fv);
				fv = Vector3Scale(fv, f);
				dBodyAddForce(bdy, fv.x, fv.y, fv.z);
            }
            

            if (d < gravSize) {
				dBodySetGravityMode(bdy, 0);

				Vector3 f = Vector3Normalize(dist);
				Vector3Scale(f, Vector3Length(G));
				dMass M;
				dBodyGetMass(bdy, &M);
				f = Vector3Scale(f, M.mass);
				if (d<2) {
					// this is just to make it more interesting so they
					// don't all clump in the centre it also helps
					// to regenrate the object orbit by adding more energy
					f = Vector3Negate(f); // push away if too close
					f = Vector3Scale(f, 8.0f);
					
					// as an alternatice you can use this with a smaller radius
					//FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
					//CreateOrbiter(physCtx, graphics);
					//node = next;
					//continue;
				}
				dBodyAddForce(bdy, f.x, f.y, f.z);
			} else {
				dBodySetGravityMode(bdy, 1); // normal gravity
			}
                        
            if(pos[1]<-10) {
                FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
				CreateOrbiter(physCtx, graphics);
            }
            
            node = next;
        }

        // drawing
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(graphics->camera);
                DrawBodies(graphics, physCtx);
                DrawStatics(graphics, physCtx);
                DrawSphereWires(gravPoint, gravSize, 8, 8, BLUE);
                DrawGrid(100,10); // for context (no ground!)
            EndMode3D();

            DrawText("Gravity manipulation", 10, 40, 20, RAYWHITE);

        EndDrawing();
    }

    FreePhysics(physCtx);
    FreeGraphics(graphics);
    CloseWindow();
    return 0;
}
