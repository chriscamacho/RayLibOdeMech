/*
 * Copyright (c) 2021-26 Chris Camacho (codifies -  http://bedroomcoders.co.uk/)
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
 
#ifndef RAYLIBODE_H
#define RAYLIBODE_H

#include "raylib.h"
#include "raymath.h"
#include "rlights.h"

#include <ode/ode.h>
#include "clist.h"

#define WORLD_GROUP		0x0001
#define PISTON_GROUP	0x0002

// Object counts
#define NUM_OBJ 300

// Plane configuration
#define PLANE_SIZE 20.0f
#define PLANE_THICKNESS 1.0f

#define maxPsteps 6
    
typedef struct entity {
	dBodyID body;/**< ODE physics body for this entity */
	cnode_t* node; /**< all entities are in a global list, this is its list node */
	void* data; /**< user data pointer tag on extra meta data to a geom. */
} entity;

/**
 * @brief Function pointer type for physics trigger events
 * * This callback is triggered when a geometry designated as a "ghost" or sensor 
 * overlaps with another geometry. When this callback is set in geomInfo, 
 * the physics engine will skip standard contact resolution, allowing 
 * objects to pass through while still notifying game logic.
 * * @param trigger  The dGeomID of the geometry that has the callback assigned.
 * @param intruder The dGeomID of the geometry that entered or is inside the trigger.
 * * @note If attached to a dBodyID, the trigger will follow the body's transform.
 * @see geomInfo
 */
typedef void (*TriggerCallback)(dGeomID trigger, dGeomID intruder);

/**
 * @brief Stores geometry metadata for collision, texturing, and trimesh data.
 * * This structure encapsulates everything needed to render and handle physics 
 * for a specific piece of geometry, including UV scaling and raw mesh indices.
 */
typedef struct geomInfo {
    bool collidable;        /**< Toggle for physics engine interaction. */
    Texture* texture;       /**< Pointer to the associated diffuse/albedo map. set to null for invisible geom*/
    float uvScaleU;         /**< Horizontal texture tiling factor. */
    float uvScaleV;         /**< Vertical texture tiling factor. */
    Model visual;           /**< Model override used for custom static trimeshes. */
    Color hew;				/**< white for normal, used to tint a geom */
    /** @name Trimesh Encapsulation
     * Members used specifically for raw triangle mesh data.
     * @{ */
    int* indices;           /**< Pointer to the array of vertex indices. */
    dTriMeshDataID triData; /**< ODE/Physics-specific trimesh data identifier. */
    /** @} */

    TriggerCallback triggerOnCollide;  /**< If this is non-NULL, the geom acts as a ghost/trigger */
    void* data; /**< user data pointer tag on extra meta data to a geom. */
} geomInfo;


// Struct to track the results of our raycast
typedef struct RayHit {
    dGeomID geom;
    dReal depth; // Distance from ray origin
    Vector3 pos; // Hit position in world space
} RayHit;

// Graphics context - holds all rendering resources
typedef struct GraphicsContext {
    Model box;
    Model ball;
    Model cylinder;
    
    
    // TODO capsules really need their own textures not cylinder texutres
    // Texture arrays for different geometry types
    Texture sphereTextures[3];      // ball.png, beach-ball.png, earth.png
    Texture boxTextures[2];         // crate.png, grid.png
    Texture cylinderTextures[2];    // drum.png, cylinder2.png
    Texture groundTexture;          // grass.png
    
    Camera camera;
    Shader shader;
    Light lights[MAX_LIGHTS];
} GraphicsContext;

// Physics context - holds all physics state
typedef struct PhysicsContext {
    dWorldID world;
    dSpaceID space;             
    dJointGroupID contactgroup;
    float frameTime; // cumlative frame time
	clist_t* objList;
	clist_t* statics; // list of static ode geoms
} PhysicsContext;

typedef struct MultiPiston {
    entity** sections;
    dJointID* joints;
    int count;
    Vector3 direction;
} MultiPiston;

entity* CreateBaseEntity(PhysicsContext* ctx);

// Helper to allocate geomInfo with collision flag, optional texture, and UV scale
geomInfo* CreateGeomInfo(bool collidable, Texture* texture, float uvScaleU, float uvScaleV);

// create a geom only but with geomInfo attched
dGeomID CreateSphereGeom(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, Vector3 pos);

// helper to create a static collision geom from a model
cnode_t* CreateStaticTrimesh(PhysicsContext* physCtx, GraphicsContext* gfxCtx, Model model, Texture* tex, float uvScale);

// add a physics visual to the world
entity* CreateBox(PhysicsContext* ctx, GraphicsContext* gfxCtx, Vector3 size, Vector3 pos, Vector3 rot, float mass);
entity* CreateSphere(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, Vector3 pos, Vector3 rot, float mass); 
entity* CreateCylinder(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, float length, Vector3 pos, Vector3 rot, float mass);
entity* CreateCapsule(PhysicsContext* ctx, GraphicsContext* gfxCtx, float radius, float length, Vector3 pos, Vector3 rot, float mass);
entity* CreateDumbbell(PhysicsContext* ctx, GraphicsContext* gfxCtx, float shaftRad, float shaftLen, float endRad, Vector3 pos, Vector3 rot, float mass);

// add a random simple physics object to the world
entity* CreateRandomEntity(PhysicsContext* ctx, GraphicsContext* gfxCtx, Vector3 pos);

// return a reference to an entity the mouse is pointing to...
entity* PickEntity(PhysicsContext* physCtx, GraphicsContext* gfxCtx, Vector3* hitPoint);

void SetEntityHew(entity* ent, Color c);

dJointID CreateRotor(PhysicsContext* physCtx, entity* from, entity* to, Vector3 axis);

// free a body, freeing its geoms first
void FreeBodyAndGeoms(dBodyID bdy);

//void drawAllSpaceGeoms(dSpaceID space, struct GraphicsContext* ctx);
void DrawGeom(dGeomID geom, struct GraphicsContext* ctx);

// draw all the bodies in the object list
void DrawBodies(struct GraphicsContext* ctx, PhysicsContext* pctx);

// draw geoms attached to a body
void DrawBodyGeoms(dBodyID bdy, struct GraphicsContext* ctx);

// draw static geoms
void DrawStatics(struct GraphicsContext* ctx, PhysicsContext* pctx);

// Random float in range [min, max]
float rndf(float min, float max);

// step the physics world enough times to keep up with realtime
int StepPhysics(PhysicsContext* physCtx);

void FreeEntity(PhysicsContext* physCtx, entity* ent);

void SetPistonLimits(dJointID joint, float min, float max);
dJointID CreatePiston(PhysicsContext* physCtx, entity* entA, entity* entB, float strength);

void RayToOdeMat(Matrix* mat, dReal* R);
void OdeToRayMat(const dReal* R, Matrix* matrix);

MultiPiston* CreateMultiPiston(PhysicsContext* physCtx, GraphicsContext* graphics, 
                               Vector3 pos, Vector3 direction, int count, 
                               float sectionLen, float baseWidth, float strength);
void SetMultiPistonVelocity(MultiPiston* mp, float velocity);         

void SetBodyOrientation(dBodyID body, Vector3 direction);

dJointID PinEntityToWorld(PhysicsContext* physCtx, entity* ent);                      

#endif // RAYLIBODE_H


