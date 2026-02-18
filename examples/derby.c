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
#include <math.h>

#include "init.h"
#include "exampleCamera.h"


#define screenWidth 1920/1.2
#define screenHeight 1080/1.2

float GetSteerAngle(Vector2 carPos, Vector2 carForward, Vector2 targetPos) {

    Vector2 toTarget = Vector2Subtract(targetPos, carPos);
    Vector2 forwardNorm = Vector2Normalize(carForward);
    Vector2 targetNorm = Vector2Normalize(toTarget);
    float dot = Vector2DotProduct(forwardNorm, targetNorm);
    
    // Clamp the dot product to avoid NaN errors with acos() due to precision
    dot = Clamp(dot, -1.0f, 1.0f);
    
    float angle = acosf(dot);
    
    // Determine the sign (Left vs Right) using a 2D Cross Product (Perp-Dot)
    // Formula: (Ax * By) - (Ay * Bx)
    float cross = (forwardNorm.x * targetNorm.y) - (forwardNorm.y * targetNorm.x);
    
    if (cross < 0) {
        angle = -angle;
    }
    
    return angle; // Angle is in radians
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
	graphics.camera.position = (Vector3){80,40,0};
	

    physCtx = InitPhysics();

	// set up for the items in the world
    //--------------------------------------------------------------------------------------

	// the graphical mesh for the ground must be unloaded by the user
	Model ground = LoadModel("data/ground.obj");
	

	// framework looks after the physics stuff and rendering
	CreateStaticTrimesh(physCtx, &graphics, ground, &graphics.groundTexture, 2.5f);

    // Create ground plane
    dGeomID planeGeom = dCreateBox(physCtx->space, 1000, PLANE_THICKNESS, 1000);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics.groundTexture, 50.0f, 50.0f));
    clistAddNode(physCtx->statics, planeGeom);

	Model carBody = LoadModel("data/car-body.obj");

	// make a figure of 8 path
	#define MAXPATH 32
	Vector3 path[MAXPATH];
	float a = 0;
		
	for (int i=0; i<MAXPATH; i++) {
		path[i] = (Vector3){sin(a*2)*50, 2, cos(a)*80};
		a+=(M_PI*2)/MAXPATH;
	}
	
	#define MAXCAR 12
	a=0;
	vehicle* cars[MAXCAR];
	int carTarget[MAXCAR];


	// position cars around path
	int step = 2; 

	for (int j = 0; j < MAXCAR; j++) {
		int i = j * step;
		if (i>MAXPATH) i-=MAXPATH;

		
		dVector3 pos = { path[i].x, path[i].y, path[i].z };

		// Wrap around to the start of the path for the last point
		int nextIdx = (i + 1) % MAXPATH;
		carTarget[j] = nextIdx;
		
		cars[j] = CreateVehicle(physCtx, &graphics, (Vector3){pos[0],pos[1]+1,pos[2]},// pos, 
			(Vector3){6, 1.2, 3},// car body size
			.85, .6); // wheel radius/width

		// TODO find an easier way as reliable as this and make a function for it...
		Vector3 forward = Vector3Normalize(Vector3Subtract(path[carTarget[j]], path[i]));
		Vector3 worldUp = { 0.0f, 1.0f, 0.0f };
		
		Vector3 right = Vector3Normalize(Vector3CrossProduct(worldUp, forward));
		Vector3 actualUp = Vector3CrossProduct(forward, right);

		// for historical reasons, i think the cars travel along their X axis
		// and I cannot bare to bring myself to ripping apart all the vehicle
		// code (it works!)
		Matrix worldRot = {
			// note to self stop hacking matrices till they do what they want
			// this is not how a matrix is normally set up ! 
			forward.x, forward.y, forward.z, 0.0f,
			actualUp.x, actualUp.y, actualUp.z, 0.0f,
			-right.x,   -right.y,   -right.z,   0.0f,
			0.0f,      0.0f,      0.0f,      1.0f
		};

		Quaternion q = QuaternionFromMatrix(worldRot);
		
		dQuaternion odeQ = { q.w, q.x, q.y, q.z };
		dBodySetQuaternion(cars[j]->bodies[0], odeQ);
		// end of TODO

		unflipVehicle(cars[j]); // hack to get everything else in the car to align!

		geomInfo* gi = dGeomGetData(cars[j]->geoms[0]);
		gi->visual = carBody;
		
		// use the front marker as the top part of the body
		dGeomBoxSetLengths (cars[j]->geoms[6], 3, 2, 3);
		dGeomSetOffsetPosition(cars[j]->geoms[6],-1,1,0);
		gi = dGeomGetData(cars[j]->geoms[6]);
		gi->texture = 0; // make it invisible (still collides)
	}

    float physTime = 0;
    int frameCount = 0;

    //--------------------------------------------------------------------------------------
    //
    // Main game loop
    //
    //--------------------------------------------------------------------------------------
    while (!WindowShouldClose())            // Detect window close button or ESC key
    {
		frameCount++;
        //--------------------------------------------------------------------------------------
        // Update
        //----------------------------------------------------------------------------------
		
		// baked in controls (example only camera!)
		//updateVehicleCamera(&graphics, car);
		updateCamera(&graphics);
        

		for (int i=0; i<MAXCAR; i++) {
			const dReal* pos = dBodyGetPosition(cars[i]->bodies[0]);
			Vector2 carPos = (Vector2){pos[0],pos[2]};
			Vector2 target = (Vector2){path[carTarget[i]].x,path[carTarget[i]].z};
			const dReal* R = dBodyGetRotation(cars[i]->bodies[0]);


			Matrix rR;
			odeToRayMat(R, &rR);
			Vector3 f3 = (Vector3){1,0,0};
			f3 = Vector3Transform(f3,rR);
			Vector2 forward = (Vector2){f3.x,f3.z};
			
			float s = GetSteerAngle(carPos, forward, target);
			// each cars speed is modified by how much they steer.
			UpdateVehicle(cars[i], 16+(15-abs((int)(s*10))), s);

			
			if (Vector2Distance(carPos, target) < 8.0) {
				carTarget[i]++;
				if (carTarget[i]>MAXPATH-1) carTarget[i] = 0;
			}
			
			if (frameCount % 350 == 0) {
				// detect over turned
				f3 = (Vector3){0,1,0};
				f3 = Vector3Transform(f3,rR);
				if (f3.y<0) {
					unflipVehicle (cars[i]);
				}
			}
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
			
			for (int i=0; i<MAXPATH; i++) {
				DrawCylinder(path[i], .2,.2,8,1,YELLOW);
			}

			/*
			for (int i=0; i<MAXCAR; i++) {
				const dReal* pos = dBodyGetPosition(cars[i]->bodies[0]);
				Vector3 from = (Vector3){pos[0],pos[1],pos[2]};
				DrawLine3D(from, path[carTarget[i]], WHITE); 
			}
			*/
			
        EndMode3D();


        if (pSteps > maxPsteps) DrawText("WARNING CPU overloaded lagging real time", 10, 0, 20, RED);
        DrawText(TextFormat("%2i FPS", GetFPS()), 10, 20, 20, WHITE);

        DrawText(TextFormat("Phys steps per frame %i",pSteps), 10, 120, 20, WHITE);
        DrawText(TextFormat("Phys time per frame %f",physTime), 10, 140, 20, WHITE);
        DrawText(TextFormat("total time per frame %f",GetFrameTime()), 10, 160, 20, WHITE);

        EndDrawing();

    }

    
    // De-Initialization
    //--------------------------------------------------------------------------------------
    
    
    // TODO release cars
    
    for (int i=0; i<MAXCAR; i++) {
		FreeVehicle(physCtx, cars[i]);
	}
    UnloadModel(ground);
    UnloadModel(carBody);

    CleanupPhysics(physCtx);
    
    CleanupGraphics(&graphics);

    CloseWindow();              // Close window and OpenGL context
    
    return 0;
}
