/*
 * Copyright (c) 2026 Chris Camacho (codifies -  http://bedroomcoders.co.uk/)
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

/**
 * @file init.c
 * @brief Initialization and cleanup functions for graphics and physics
 *
 * This file provides functions to initialize and clean up the graphics
 * and physics subsystems. It handles resource loading, memory allocation,
 * and proper shutdown of all framework components.
 *
 * @author Chris Camacho (codifies - http://bedroomcoders.co.uk/)
 * @date 2026
 *
 * @section initialization_order Initialization Order
 *
 * Typical initialization sequence:
 * 1. InitGraphics() - Set up window, models, textures, shaders
 * 2. InitPhysics() - Create ODE world, space, contact groups
 *
 * @section cleanup_order Cleanup Order
 *
 * Typical cleanup sequence:
 * 1. CleanupPhysics() - Destroy ODE world, free bodies
 * 2. CleanupGraphics() - Unload textures, shaders, close window
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "raylibODE.h"



// a lot of this stuff doesn't change too often 
// so no point polluting main.c with it...

/**
 * @brief Initialize graphics context and window
 *
 * Sets up the RayLib window, loads all models and textures, compiles
 * shaders, and creates lighting. This must be called before any
 * rendering can occur.
 *
 * @param width Window width in pixels
 * @param height Window height in pixels
 * @param title Window title string
 *
 * @note Enables VSync and 4x MSAA by default
 * @note Hides and locks cursor for FPS-style camera control
 * @note Loads default textures from data/ directory
 *
 * @see GraphicsContext
 * @see FreeGraphics
 */
GraphicsContext* CreateGraphics(int width, int height, const char* title)
{
    InitWindow(width, height, title);
    SetWindowState(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    
    // work around for mouse delta and initial camera direction
    SetMousePosition(width/2, height/2); 
    BeginDrawing();
    EndDrawing();

    DisableCursor();  // Hide and lock cursor
    
    GraphicsContext* ctx = MemAlloc(sizeof(GraphicsContext));
    // Load models
    ctx->box = LoadModelFromMesh(GenMeshCube(1, 1, 1));
    ctx->ball = LoadModelFromMesh(GenMeshSphere(.5, 32, 32));
    ctx->cylinder = LoadModel("data/cylinder.obj");

    // Load sphere textures
    ctx->sphereTextures[0] = LoadTexture("data/ball.png");
    ctx->sphereTextures[1] = LoadTexture("data/beach-ball.png");
    ctx->sphereTextures[2] = LoadTexture("data/earth.png");

    // Load box textures
    ctx->boxTextures[0] = LoadTexture("data/crate.png");
    ctx->boxTextures[1] = LoadTexture("data/grid.png");

    // Load cylinder textures
    ctx->cylinderTextures[0] = LoadTexture("data/drum.png");
    ctx->cylinderTextures[1] = LoadTexture("data/cylinder2.png");

    // Load ground texture
    ctx->groundTexture = LoadTexture("data/grass.png");

    // Apply default textures to models (overridden by per-instance textures)
    ctx->box.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = ctx->boxTextures[0];
    ctx->ball.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = ctx->sphereTextures[0];
    ctx->cylinder.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = ctx->cylinderTextures[0];

    // Load shader and set up uniforms
    ctx->shader = LoadShader("data/simpleLight.vs", "data/simpleLight.fs");
    ctx->shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(ctx->shader, "matModel");
    ctx->shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(ctx->shader, "viewPos");

    // Set ambient light
    int amb = GetShaderLocation(ctx->shader, "ambient");
    SetShaderValue(ctx->shader, amb, (float[4]){0.2, 0.2, 0.2, 1.0}, SHADER_UNIFORM_VEC4);

    // Apply shader to models
    ctx->box.materials[0].shader = ctx->shader;
    ctx->ball.materials[0].shader = ctx->shader;
    ctx->cylinder.materials[0].shader = ctx->shader;

    // Create lights
    ctx->lights[0] = CreateLight(LIGHT_POINT, (Vector3){-25, 25, 25}, Vector3Zero(),
                                (Color){128, 128, 128, 255}, ctx->shader);
    ctx->lights[1] = CreateLight(LIGHT_POINT, (Vector3){-25, 25, -25}, Vector3Zero(),
                                (Color){64, 64, 64, 255}, ctx->shader);
                                
	return ctx;
}



/**
 * @brief Initialize physics world and context
 *
 * Creates and configures the ODE physics world, collision space,
 * and contact group. Also seeds the random number generator.
 *
 * @return Pointer to newly created PhysicsContext
 *
 * @note Checks for single-precision ODE build (required)
 * @note Sets gravity to -9.8 m/s^2 (Earth gravity)
 * @note Enables auto-disable for resting bodies
 *
 * @warning Exits with error message if precision mismatch detected
 *
 * @see CleanupPhysics
 * @see PhysicsContext
 */
PhysicsContext* CreatePhysics()//dSpaceID* space)//, GraphicsContext* gfxCtx)
{
	if (sizeof(dReal) != sizeof(float)) {
        fprintf(stderr, "\n[SIM ERROR] Precision Mismatch Detected!\n");
        fprintf(stderr, "Expected: %zu bytes (float), Got: %zu bytes (dReal).\n", sizeof(float), sizeof(dReal));
        fprintf(stderr, "Please re-link with the single-precision version of ODE.\n\n");
        exit(EXIT_FAILURE); 
    }
    srand ( time(NULL) );
    
    // Allocate physics context
    PhysicsContext* ctx = RL_MALLOC(sizeof(PhysicsContext));
    if (!ctx) return NULL;
    
	ctx->objList = clistCreateList();
	ctx->statics = clistCreateList();

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    ctx->world = dWorldCreate();
    printf("phys iterations per step %i\n", dWorldGetQuickStepNumIterations(ctx->world));
    ctx->space = dHashSpaceCreate(NULL);
    //ctx->space = space;  // Store space pointer for cleanup
    ctx->contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(ctx->world, 0, -9.8, 0);

    dWorldSetAutoDisableFlag(ctx->world, 1);
    dWorldSetAutoDisableLinearThreshold(ctx->world, 0.05);
    dWorldSetAutoDisableAngularThreshold(ctx->world, 0.05);
    dWorldSetAutoDisableSteps(ctx->world, 4);



    return ctx;
}

void FreePhysics(PhysicsContext* ctx)
{
    if (!ctx) return;

	cnode_t* node = ctx->objList->head;

	while (node != NULL) {
		entity* ent = (entity*)node->data;
		if (ent) {
			FreeBodyAndGeoms(ent->body);
			free(ent);
		}
		node = node->next;
	}

	clistFreeList(&ctx->objList);
	
	
	node = ctx->statics->head;

	while (node != NULL) {
		dGeomID geom = node->data;
		if (geom) {
			geomInfo* gi = dGeomGetData(geom);
			if (gi) {
				if (gi->indices) RL_FREE(gi->indices);
				if (gi->triData) dGeomTriMeshDataDestroy(gi->triData);
				free(gi);
			}
			dGeomSetBody(geom, 0);
			dGeomDestroy(geom);
		}
		node = node->next;
	}
	
	clistFreeList(&ctx->statics);

	
	// dJointGroupEmpty clears the joints; dJointGroupDestroy frees the group memory
    if (ctx->contactgroup) {
        dJointGroupEmpty(ctx->contactgroup);
        dJointGroupDestroy(ctx->contactgroup);
    }

	dSpaceDestroy(ctx->space);
	dWorldDestroy(ctx->world);
	dCloseODE();

    RL_FREE(ctx);
}

void FreeGraphics(GraphicsContext* ctx)
{

    // Clean up graphics resources
    UnloadModel(ctx->box);
    UnloadModel(ctx->ball);
    UnloadModel(ctx->cylinder);
    
    // Unload all textures
    UnloadTexture(ctx->sphereTextures[0]);
    UnloadTexture(ctx->sphereTextures[1]);
    UnloadTexture(ctx->sphereTextures[2]);
    UnloadTexture(ctx->boxTextures[0]);
    UnloadTexture(ctx->boxTextures[1]);
    UnloadTexture(ctx->cylinderTextures[0]);
    UnloadTexture(ctx->cylinderTextures[1]);
    UnloadTexture(ctx->groundTexture);
    
    UnloadShader(ctx->shader);
    
    RL_FREE(ctx);
}
