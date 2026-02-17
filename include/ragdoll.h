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

#ifndef RAYLIBODERAGDOLL_H
#define RAYLIBODERAGDOLL_H

#include "raylib.h"

#include <ode/ode.h>


// Rag doll structure - generic enough for neural network muscle control
// Uses motors on joints for future neural network control
typedef struct RagDoll {
    dBodyID *bodies;           // Array of bodies (head, torso, arms, legs)
    dGeomID *geoms;            // Array of geometries
    dJointID *joints;           // Array of joints connecting bodies
    dJointID *motors;           // Array of motor joints for muscle control
    int bodyCount;              // Number of bodies
    int jointCount;             // Number of joints
    int motorCount;             // Number of motors
} RagDoll;

// Predefined rag doll body parts for easy access
typedef enum {
    RAGDOLL_HEAD = 0,
    RAGDOLL_TORSO,
    RAGDOLL_LEFT_UPPER_ARM,
    RAGDOLL_LEFT_LOWER_ARM,
    RAGDOLL_RIGHT_UPPER_ARM,
    RAGDOLL_RIGHT_LOWER_ARM,
    RAGDOLL_LEFT_UPPER_LEG,
    RAGDOLL_LEFT_LOWER_LEG,
    RAGDOLL_RIGHT_UPPER_LEG,
    RAGDOLL_RIGHT_LOWER_LEG,
    RAGDOLL_BODY_COUNT         // Total count
} RagdollBodyPart;


// Forward declaration - GraphicsContext is defined in init.h
struct GraphicsContext;

// Rag doll functions - generic for neural network muscle control
//RagDoll* CreateRagdoll(dSpaceID space, dWorldID world, Vector3 position, struct GraphicsContext* ctx);
RagDoll* CreateRagdoll(struct PhysicsContext* pctx, struct GraphicsContext* ctx, Vector3 position);

void UpdateRagdollMotors(RagDoll *ragdoll, float *motorForces);
void DrawRagdoll(RagDoll *ragdoll, struct GraphicsContext* ctx);
void FreeRagdoll(PhysicsContext* ctx, RagDoll *ragdoll);

// Ragdoll spawn configuration
#define RAGDOLL_SPAWN_CENTER_X 0.0f
#define RAGDOLL_SPAWN_CENTER_Z 0.0f
#define RAGDOLL_SPAWN_HALF_EXTENT 3.0f
#define RAGDOLL_SPAWN_MIN_Y 1.6f
#define RAGDOLL_SPAWN_MAX_Y 2.6f

// Get a random spawn position within the defined ragdoll spawn volume
Vector3 GetRagdollSpawnPosition(void);

#endif // RAYLIBODERAGDOLL_H
