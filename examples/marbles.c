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

typedef enum { SLOPE, STRAIGHT, CORNER } MeshType;

typedef struct LevelElement {
    MeshType type;
    float gridX;    // Will be multiplied by 2.0
    float gridY;    // Will be multiplied by 0.55
    float gridZ;    // Will be multiplied by 2.0
    float angle;  // Rotation in radians (e.g., M_PI, -M_PI/2)
} LevelElement;


// Define the level layout
LevelElement levelData[] = {
    { SLOPE,    0, 8, 0, 0.0f },      
    { SLOPE,    1, 7, 0, 0.0f },      
    { STRAIGHT, 2, 6, 0, 0.0f },      
    { STRAIGHT, 3, 6, 0, 0.0f },
    { CORNER,   4, 6, 0, M_PI },
    { SLOPE,    4, 6, 1, -M_PI/2 },   
    { CORNER,   4, 5, 2, 0.0f },
    { SLOPE,    5, 5, 2, 0.0f },      
    { CORNER,   6, 4, 2, M_PI/2 },
    { SLOPE,    6, 4, 1, M_PI/2 },      
    { SLOPE,    6, 3, 0, M_PI/2 },      
    { CORNER,   6, 2, -1, M_PI },
    { CORNER,   5, 2, -1, -M_PI/2},
    { SLOPE,    5, 2, 0, -M_PI/2 },      
    { SLOPE,    5, 1, 1, -M_PI/2 },
	{ SLOPE, 	5, 0, 2, -M_PI/2 },        
	{ SLOPE, 	5, -1, 3, -M_PI/2 },        

	{ STRAIGHT, 	5, 6, 9, -M_PI/2 },  
	{ CORNER,   5, 5.5, 10, M_PI/2},
	{ STRAIGHT, 4, 5.5, 10, 0.0f },       
	{ SLOPE, 	3, 5, 10, M_PI },  
	{ STRAIGHT, 2, 4, 10, 0.0f },       
	{ STRAIGHT, 1, 4, 10, 0.0f },       
	{ STRAIGHT, 0, 4, 10, 0.0f },       
	{ STRAIGHT, -1, 4, 10, 0.0f },       
	{ CORNER,   -2, 4, 10, 0},
	{ SLOPE, 	-2, 4, 9, M_PI/2 },  
	{ STRAIGHT, -2, 3, 8, M_PI/2 },       
	{ STRAIGHT, -2, 3, 7, M_PI/2 },       
	{ STRAIGHT, -2, 3, 6, M_PI/2 },       
	{ SLOPE, -2, 3, 5, M_PI/2 },       
	{ STRAIGHT, -2, 2, 4, M_PI/2 },       
	{ STRAIGHT, -2, 2, 3, M_PI/2 },       
	{ STRAIGHT, -2, 2, 2, M_PI/2 },       
	{ SLOPE, -2, 2, 1, M_PI/2 },   
	
	{ STRAIGHT, -2, 9, -5, M_PI/2 },       
	{ SLOPE, -2, 9, -6, M_PI/2 },       
	{ CORNER, -2, 8, -7, -M_PI/2 },  
	{ STRAIGHT, -1, 8, -7, 0},   
	{ CORNER, 0, 8, -7, M_PI},   
	{ SLOPE, 0, 8, -6, -M_PI/2 },   
	 
	{ STRAIGHT, 0, 14, 0, M_PI/2},   
	{ STRAIGHT, 0, 14, 1, M_PI/2},   
	{ CORNER, 0, 14, 2, M_PI/2},   
	{ SLOPE, -1, 14, 2, M_PI },   
	{ CORNER, -2, 13, 2, -M_PI/2},   
	{ CORNER, -2, 13, 3, 0}, 
	{ SLOPE, -1, 13, 3, 0 },  
	{ CORNER, 0, 12, 3, M_PI/2 },  
	{ CORNER, 0, 12, 2, M_PI },  
	{ SLOPE, -1, 12, 2, M_PI },  
	{ CORNER, -2, 11, 2, 0},  
	{ SLOPE, -2, 11, 1, M_PI/2 },  
	{ CORNER, -2, 10, 0, -M_PI/2},  
	{ SLOPE, -1, 10, 0,0},  
	
};


void LoadLevel(PhysicsContext* physCtx, GraphicsContext* graphics) {
    int numElements = sizeof(levelData) / sizeof(LevelElement);

	const float layerHeight = 1.17;

    for (int i = 0; i < numElements; i++) {
        LevelElement e = levelData[i];
		float ea = e.angle + M_PI_2;
		
        switch(e.type) 
        {
            case SLOPE:
				(void)e; // workaround for c99 decl after label
				float pAngle = 30.0f * (M_PI / 180.0f); 
				
				if (ea > M_PI * 2.0f) ea -= M_PI * 2.0f;
				dQuaternion q,q2,q3;
				dQFromAxisAndAngle (q3, 1, 0, 0, pAngle);
				dQFromAxisAndAngle (q2, 0, 1, 0, ea);
				dQMultiply0 (q, q2, q3);
				dMatrix4 R;
				dQtoR(q, R);
				
				float offX = 0.5f * cosf(ea);
				float offZ = 0.5f * sinf(ea);

				// Rail 1
				Vector3 pos = (Vector3) {
					(float)e.gridX * 2.0f + offX,
					(float)e.gridY * layerHeight -.2f,
					(float)e.gridZ * 2.0f - offZ};
				dGeomID dg = CreateCylinderGeom(physCtx, graphics, 0.1f, 2.4f, pos);
				dGeomSetRotation(dg, R);
		
				// Rail 2
				pos.x = (float)e.gridX * 2.0f - offX;
				pos.z = (float)e.gridZ * 2.0f + offZ;
				dGeomID dg2 = CreateCylinderGeom(physCtx, graphics, 0.1f, 2.4f, pos);
				dGeomSetRotation(dg2, R);

				clistAddNode(physCtx->statics, dg);
				clistAddNode(physCtx->statics, dg2);
				break;
            
            case STRAIGHT: 
            
				offX = 0.5f * cosf(ea);
				offZ = 0.5f * sinf(ea);
				
				pos = (Vector3) {
					(float)e.gridX * 2.0f + offX,
					(float)e.gridY * layerHeight +0.4f,
					(float)e.gridZ * 2.0f - offZ};
            
				dg = CreateCylinderGeom(physCtx, graphics, 0.1f, 2, pos);
				
				pos.x = (float)e.gridX * 2.0f - offX;
				pos.z = (float)e.gridZ * 2.0f + offZ;
				
				dg2 = CreateCylinderGeom(physCtx, graphics, 0.1f, 2, pos);
				
				// always rotate as Z axis cylinders need extra half pi rotation
				dQFromAxisAndAngle(q, 0, 1, 0, e.angle+M_PI_2);
				dGeomSetQuaternion(dg, q);
				dGeomSetQuaternion(dg2, q);
				
				clistAddNode(physCtx->statics, dg);
				clistAddNode(physCtx->statics, dg2);
				break;
            
            case CORNER: 
				// this was just a brain melt !
				offX = (float)e.gridX * 2.0f;
				offZ = (float)e.gridZ * 2.0f;

				// Stick with the rot + M_PI_2 that gave us consistency
				float rot = e.angle + M_PI_2;
				float s = sinf(rot);
				float c = cosf(rot);

				for (int j = 0; j < 3; j++) {
					float pAngle = (j * (M_PI_2 / 3.0f)) + (M_PI_2 / 6.0f);
					
					float lx1 = -1.0f + 0.5f * cosf(pAngle);
					float lz1 = -1.0f + 0.5f * sinf(pAngle);
					float lx2 = -1.0f + 1.55f * cosf(pAngle);
					float lz2 = -1.0f + 1.55f * sinf(pAngle);

					pos.x = offX - (lx1 * c + lz1 * s); 
					pos.y = (float)e.gridY * layerHeight + 0.4f;
					pos.z = offZ - (lz1 * c - lx1 * s); 

					dQFromAxisAndAngle(q, 0, 1, 0, rot - pAngle);
					dQtoR(q, R);

					// Inner Rail
					dg = CreateCylinderGeom(physCtx, graphics, 0.1f, 0.261f, pos);
					dGeomSetRotation(dg, R);
					clistAddNode(physCtx->statics, dg);

					pos.x = offX - (lx2 * c + lz2 * s);
					pos.y += 0.5f;
					pos.z = offZ - (lz2 * c - lx2 * s);
					
					// Outer Rail
					dg2 = CreateCylinderGeom(physCtx, graphics, 0.1f, 0.776f, pos);
					dGeomSetRotation(dg2, R);
					clistAddNode(physCtx->statics, dg2);
				}
				break;
        }

    }
}

#define MAX_PISTON 6

MultiPiston** CreateLift(PhysicsContext* physCtx,
                         GraphicsContext* graphics,
                         Vector3 position,
                         float yaw,
                         float strength)
{
    Vector3 upAxis = (Vector3){0.f, 1.f, 0.f};

    // Allocate multipiston array using raylib allocator
    MultiPiston** mp = (MultiPiston**)MemAlloc(sizeof(MultiPiston*) * MAX_PISTON);

    dMatrix3 R_yaw;
    dRFromAxisAndAngle(R_yaw, 0, 1, 0, yaw);

    // Pistons 
    for (int i = 0; i < MAX_PISTON; i++)
    {
        Vector3 localPos = (Vector3){10.f, -3.8f + i*1.4f, 7.6f + i*1.6f};

        Vector3 worldPos = Vector3Add(
            position,
            Vector3RotateByAxisAngle(localPos, upAxis, yaw)
        );

        Vector3 axis = Vector3RotateByAxisAngle(
            (Vector3){1.f,0.f,0.f},
            upAxis,
            yaw
        );

        mp[i] = CreateMultiPiston(physCtx, graphics,
                                  worldPos,
                                  axis,
                                  3,
                                  2.0f,
                                  1.2f,
                                  strength);

        dMatrix3 R_local;
        dMatrix3 R_final;

        dRFromEulerAngles(R_local,
                          0,
                          M_PI_2,
                          (-M_PI/16.f) * 7.f);

        dMultiply0(R_final, R_yaw, R_local, 3, 3, 3);
        dBodySetRotation(mp[i]->sections[0]->body, R_final);
        PinEntityToWorld(physCtx, mp[i]->sections[0]);
    }

    // -------------------------------------------------
    // Lift End
    // -------------------------------------------------
    {
        Vector3 localPos = (Vector3){9.2f, 5.f, 16.7f};

        Vector3 worldPos = Vector3Add(
            position,
            Vector3RotateByAxisAngle(localPos, upAxis, yaw)
        );

        dGeomID liftEnd = CreateBoxGeom(
            physCtx,
            graphics,
            (Vector3){2.f,.1f,4.f},
            worldPos
        );

        dMatrix3 R_tilt;
        dMatrix3 R_final;

        dRFromAxisAndAngle(R_tilt, 1, 0, 0, -(M_PI/16.f) * 7.f);
        dMultiply0(R_final, R_yaw, R_tilt, 3, 3, 3);
        dGeomSetRotation(liftEnd, R_final);
        clistAddNode(physCtx->statics, liftEnd);
    }

    // -------------------------------------------------
    // Rail 1
    // -------------------------------------------------
    {
        Vector3 localPos = (Vector3){9.2f, 2.f, 12.f};
        Vector3 worldPos = Vector3Add(
            position,
            Vector3RotateByAxisAngle(localPos, upAxis, yaw)
        );

        dGeomID liftRail1 = CreateBoxGeom(
            physCtx,
            graphics,
            (Vector3){.1f,.8f,13.f},
            worldPos
        );

        dMatrix3 R_tilt;
        dMatrix3 R_final;

        dRFromAxisAndAngle(R_tilt, 1, 0, 0, -(M_PI/16.f) * 3.5f);
        dMultiply0(R_final, R_yaw, R_tilt, 3, 3, 3);
        dGeomSetRotation(liftRail1, R_final);
        clistAddNode(physCtx->statics, liftRail1);
    }

    // -------------------------------------------------
    // Rail 2
    // -------------------------------------------------
    {
        Vector3 localPos = (Vector3){10.8f, 2.f, 12.f};

        Vector3 worldPos = Vector3Add(
            position,
            Vector3RotateByAxisAngle(localPos, upAxis, yaw)
        );

        dGeomID liftRail2 = CreateBoxGeom(
            physCtx,
            graphics,
            (Vector3){.1f,.8f,13.f},
            worldPos
        );

        dMatrix3 R_tilt;
        dMatrix3 R_final;

        dRFromAxisAndAngle(R_tilt, 1, 0, 0, -(M_PI/16.f) * 3.5f);

        dMultiply0(R_final, R_yaw, R_tilt, 3, 3, 3);

        dGeomSetRotation(liftRail2, R_final);

        clistAddNode(physCtx->statics, liftRail2);
    }

    return mp;
}


int main(void)
{
	// init
    PhysicsContext* physCtx = CreatePhysics();
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
    SetupCamera(graphics);
    graphics->camera.position = (Vector3){0.f,5.f,8.f};
    SetCameraYaw(M_PI/4);

	const Vector3 dropPoint = (Vector3){0.f,10.f,0.f};
	
	LoadLevel(physCtx,graphics);
	

	
	MultiPiston** lift1 = CreateLift(physCtx,
							graphics,
							(Vector3){0.f, 0.f, 0.f},
							0,
							400.0f);

	
	MultiPiston** lift2 = CreateLift(physCtx,
                                graphics,
                                (Vector3){6.f, 4.f, 8.f},
                                PI,
                                400.0f);

	MultiPiston** lift3 = CreateLift(physCtx,
                                graphics,
                                (Vector3){-10.f, 10.f, -18.f},
                                0,
                                400.0f);
                                                                	
	int frameCount = 0;
	int released = 0;
	const int maxRelesed = 16;
	
    while (!WindowShouldClose())
    {
		for(int i=0; i < MAX_PISTON; i++) {
			float offset = 0.f;
			// every other piston is 180 degrees out of phase
			if (i % 2 == 0) offset = M_PI;
			SetMultiPistonVelocity(lift1[i], sin(((float)frameCount)/64.0f+offset)*8.0f);
			SetMultiPistonVelocity(lift2[i], sin(((float)frameCount)/64.0f+offset)*8.0f);
			SetMultiPistonVelocity(lift3[i], sin(((float)frameCount)/64.0f+offset)*8.0f);
		}
		
		
		if (released < maxRelesed) {
			if (frameCount % 420 == 0) {
				entity* e = CreateSphere(physCtx, graphics, 0.55, dropPoint, Vector3Zero(), 10);
				dBodySetAutoDisableFlag(e->body, 0);
				released++;
			}
		}
        frameCount++;
		
		UpdateExampleCamera(graphics);
        StepPhysics(physCtx);
        
		cnode_t* node = physCtx->objList->head;

        while (node != NULL) {
			entity* ent = node->data;
            dBodyID bdy = ent->body;
            cnode_t* next = node->next; // get the next node now in case we delete this one
            const dReal* pos = dBodyGetPosition(bdy);

            if(pos[1]<-10) {
                FreeEntity(physCtx, ent); // warning deletes global entity list entry, get your next node before doing this!
                CreateSphere(physCtx, graphics, 0.55, dropPoint, Vector3Zero(), 10);
            }
            
            node = next;
        }

        // drawing
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(graphics->camera);
                DrawBodies(graphics, physCtx);
                DrawStatics(graphics, physCtx);
            EndMode3D();

            DrawText("Marbles!", 10, 40, 20, RAYWHITE);

        EndDrawing();
    }

    FreePhysics(physCtx);
    FreeGraphics(graphics);
    
    // TODO make this easier !
    for (int i = 0; i < MAX_PISTON; i++)
    {
		FreeMultiPiston(lift1[i]);
		FreeMultiPiston(lift2[i]);
		FreeMultiPiston(lift3[i]);
	}
	free(lift1);
	free(lift2);
	free(lift3);
	
    CloseWindow();
    return 0;
}
