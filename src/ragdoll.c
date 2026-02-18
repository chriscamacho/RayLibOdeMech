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
 * @file ragdoll.c
 * @brief Ragdoll physics simulation implementation
 *
 * This file provides a complete ragdoll (humanoid) physics simulation
 * using ODE's joint system. The ragdoll consists of multiple body parts
 * connected by hinge and universal joints, allowing for realistic
 * human-like movement and posing.
 *
 * @author Chris Camacho (codifies - http://bedroomcoders.co.uk/)
 * @date 2026
 *
 * @section ragdoll_structure Ragdoll Structure
 *
 * Body parts (10 total):
 * - Head (sphere)
 * - Torso (box)
 * - Left/Right Upper Arm (capsule)
 * - Left/Right Lower Arm (capsule)
 * - Left/Right Upper Leg (capsule)
 * - Left/Right Lower Leg (capsule)
 *
 * Joints (9 total):
 * - Neck (hinge)
 * - Left/Right Shoulder (universal)
 * - Left/Right Elbow (hinge)
 * - Left/Right Hip (universal)
 * - Left/Right Knee (hinge)
 *
 * @section motor_control Motor Control
 *
 * The ragdoll supports motor control on all joints, making it
 * suitable for neural network or reinforcement learning control.
 * Use UpdateRagdollMotors() to apply forces to joints.
 */

#include "raylibODE.h"
#include "ragdoll.h"

// Get a spawn position within the defined ragdoll spawn volume

/**
 * @brief Get a random spawn position for ragdolls
 *
 * Generates a random position within the configured spawn volume
 * for ragdoll objects. This helps spread out multiple ragdolls
 * when they are created.
 *
 * @return Random Vector3 position within spawn bounds
 *
 * @note Spawn bounds are defined by RAGDOLL_SPAWN_* constants
 * @see RAGDOLL_SPAWN_CENTER_X
 * @see RAGDOLL_SPAWN_CENTER_Z
 * @see RAGDOLL_SPAWN_HALF_EXTENT
 * @see RAGDOLL_SPAWN_MIN_Y
 * @see RAGDOLL_SPAWN_MAX_Y
 */
Vector3 GetRagdollSpawnPosition(void)
{
    Vector3 pos;
    pos.x = rndf(RAGDOLL_SPAWN_CENTER_X - RAGDOLL_SPAWN_HALF_EXTENT, RAGDOLL_SPAWN_CENTER_X + RAGDOLL_SPAWN_HALF_EXTENT);
    pos.y = rndf(RAGDOLL_SPAWN_MIN_Y, RAGDOLL_SPAWN_MAX_Y);
    pos.z = rndf(RAGDOLL_SPAWN_CENTER_Z - RAGDOLL_SPAWN_HALF_EXTENT, RAGDOLL_SPAWN_CENTER_Z + RAGDOLL_SPAWN_HALF_EXTENT);
    return pos;
}


// Rag doll creation 
// Creates a humanoid rag doll with configurable joint motors
// actually way more complex than the vehicle stuff !

/**
 * @brief Create a ragdoll physics body
 *
 * Creates a complete humanoid ragdoll with articulated joints.
 * The ragdoll is fully physics-simulated and can be used for
 * character physics, death animations, or as a basis for
 * reinforcement learning control.
 *
 * @param pctx Pointer to physics context
 * @param ctx Pointer to graphics context
 * @param position Initial position for the ragdoll
 * @return Pointer to newly created RagDoll structure
 *
 * @note Bodies: 10 (head, torso, 2 upper arms, 2 lower arms, 2 upper legs, 2 lower legs)
 * @note Joints: 9 (neck, shoulders, elbows, hips, knees)
 * @note All bodies are registered with framework for rendering
 *
 * @see RagDoll
 * @see UpdateRagdollMotors
 * @see DrawRagdoll
 * @see FreeRagdoll
 */
RagDoll* CreateRagdoll(struct PhysicsContext* pctx, struct GraphicsContext* ctx, Vector3 position)
{
    RagDoll *ragdoll = RL_MALLOC(sizeof(RagDoll));
    ragdoll->bodyCount = RAGDOLL_BODY_COUNT;
    ragdoll->jointCount = 9;  // neck, shoulders, elbows, hips, knees
    ragdoll->motorCount = 0;  // No motors initially, can be added for neural network control

    // Allocate arrays for bodies, geoms, joints, and motors
    ragdoll->bodies = RL_MALLOC(ragdoll->bodyCount * sizeof(dBodyID));
    ragdoll->geoms = RL_MALLOC(ragdoll->bodyCount * sizeof(dGeomID));
    ragdoll->joints = RL_MALLOC(ragdoll->jointCount * sizeof(dJointID));
    ragdoll->motors = RL_MALLOC(ragdoll->jointCount * sizeof(dJointID));  // Potential motors

    dMass m;

    // Body dimensions
    // TODO should be header defines
    float headRadius = 0.25f;
    float torsoWidth = 0.4f;
    float torsoHeight = 0.6f;
    float torsoDepth = 0.25f;
    float armLength = 0.35f;
    float armRadius = 0.1f;
    float legLength = 0.45f;
    float legRadius = 0.12f;

    // Ragdoll specific textures (consistent across all ragdolls)
    Texture* headTex = &ctx->sphereTextures[1];        // beach-ball.png
    Texture* torsoTex = &ctx->boxTextures[0];         // crate.png
    Texture* limbTex = &ctx->cylinderTextures[1];     // cylinder2.png

    // Create head
    dMassSetSphere(&m, 1, headRadius);
    dMassAdjust(&m, 5.0f);
    ragdoll->bodies[RAGDOLL_HEAD] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_HEAD], &m);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_HEAD],
                     position.x, position.y + 1.6f, position.z);
    ragdoll->geoms[RAGDOLL_HEAD] = dCreateSphere(pctx->space, headRadius);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_HEAD], ragdoll->bodies[RAGDOLL_HEAD]);
    dGeomSetData(ragdoll->geoms[RAGDOLL_HEAD], CreateGeomInfo(true, headTex, 1.0f, 1.0f));

    // Create torso
    dMassSetBox(&m, 1, torsoWidth, torsoHeight, torsoDepth);
    dMassAdjust(&m, 30.0f);
    ragdoll->bodies[RAGDOLL_TORSO] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_TORSO], &m);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_TORSO],
                     position.x, position.y + 0.9f, position.z);
    ragdoll->geoms[RAGDOLL_TORSO] = dCreateBox(pctx->space, torsoWidth, torsoHeight, torsoDepth);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_TORSO], ragdoll->bodies[RAGDOLL_TORSO]);
    dGeomSetData(ragdoll->geoms[RAGDOLL_TORSO], CreateGeomInfo(true, torsoTex, 1.0f, 1.0f));

    // Create arms - initialize mass for each individually
    // ODE cylinders are along Z-axis by default
    // Rotate geoms 90° around Y to orient along X-axis
    dMatrix3 R_arm;
    dRFromAxisAndAngle(R_arm, 0, 1, 0, M_PI * 0.5);

    // Left upper arm
    dMassSetCylinder(&m, 1, 3, armRadius, armLength);
    dMassAdjust(&m, 3.0f);
    ragdoll->bodies[RAGDOLL_LEFT_UPPER_ARM] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_LEFT_UPPER_ARM], &m);
    //ragdoll->geoms[RAGDOLL_LEFT_UPPER_ARM] = dCreateCylinder(space, armRadius, armLength);
    ragdoll->geoms[RAGDOLL_LEFT_UPPER_ARM] = dCreateCapsule(pctx->space, armRadius, armLength);
    
    dGeomSetBody(ragdoll->geoms[RAGDOLL_LEFT_UPPER_ARM], ragdoll->bodies[RAGDOLL_LEFT_UPPER_ARM]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_LEFT_UPPER_ARM], R_arm);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_LEFT_UPPER_ARM],
                     position.x - 0.35f, position.y + 1.1f, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_LEFT_UPPER_ARM], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Left lower arm
    dMassSetCylinder(&m, 1, 3, armRadius, armLength);
    dMassAdjust(&m, 3.0f);
    ragdoll->bodies[RAGDOLL_LEFT_LOWER_ARM] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_LEFT_LOWER_ARM], &m);
    //ragdoll->geoms[RAGDOLL_LEFT_LOWER_ARM] = dCreateCylinder(space, armRadius, armLength);
    ragdoll->geoms[RAGDOLL_LEFT_LOWER_ARM] = dCreateCapsule(pctx->space, armRadius, armLength);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_LEFT_LOWER_ARM], ragdoll->bodies[RAGDOLL_LEFT_LOWER_ARM]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_LEFT_LOWER_ARM], R_arm);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_LEFT_LOWER_ARM],
                     position.x - 0.35f - armLength, position.y + 1.1f, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_LEFT_LOWER_ARM], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Right upper arm
    dMassSetCylinder(&m, 1, 3, armRadius, armLength);
    dMassAdjust(&m, 3.0f);
    ragdoll->bodies[RAGDOLL_RIGHT_UPPER_ARM] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_RIGHT_UPPER_ARM], &m);
    //ragdoll->geoms[RAGDOLL_RIGHT_UPPER_ARM] = dCreateCylinder(space, armRadius, armLength);
    ragdoll->geoms[RAGDOLL_RIGHT_UPPER_ARM] = dCreateCapsule(pctx->space, armRadius, armLength);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_RIGHT_UPPER_ARM], ragdoll->bodies[RAGDOLL_RIGHT_UPPER_ARM]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_RIGHT_UPPER_ARM], R_arm);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_RIGHT_UPPER_ARM],
                     position.x + 0.35f, position.y + 1.1f, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_RIGHT_UPPER_ARM], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Right lower arm
    dMassSetCylinder(&m, 1, 3, armRadius, armLength);
    dMassAdjust(&m, 3.0f);
    ragdoll->bodies[RAGDOLL_RIGHT_LOWER_ARM] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_RIGHT_LOWER_ARM], &m);
    //ragdoll->geoms[RAGDOLL_RIGHT_LOWER_ARM] = dCreateCylinder(space, armRadius, armLength);
    ragdoll->geoms[RAGDOLL_RIGHT_LOWER_ARM] = dCreateCapsule(pctx->space, armRadius, armLength);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_RIGHT_LOWER_ARM], ragdoll->bodies[RAGDOLL_RIGHT_LOWER_ARM]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_RIGHT_LOWER_ARM], R_arm);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_RIGHT_LOWER_ARM],
                     position.x + 0.35f + armLength, position.y + 1.1f, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_RIGHT_LOWER_ARM], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Create legs - initialize mass for each individually
    // ODE cylinders are along Z-axis by default
    // Rotate geoms 90° around X to orient along Y-axis
    dMatrix3 R_leg;
    dRFromAxisAndAngle(R_leg, 1, 0, 0, M_PI * 0.5);

    // Left upper leg
    dMassSetCylinder(&m, 1, 3, legRadius, legLength);
    dMassAdjust(&m, 8.0f);
    ragdoll->bodies[RAGDOLL_LEFT_UPPER_LEG] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_LEFT_UPPER_LEG], &m);
    //ragdoll->geoms[RAGDOLL_LEFT_UPPER_LEG] = dCreateCylinder(space, legRadius, legLength);
    ragdoll->geoms[RAGDOLL_LEFT_UPPER_LEG] = dCreateCapsule(pctx->space, legRadius, legLength);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_LEFT_UPPER_LEG], ragdoll->bodies[RAGDOLL_LEFT_UPPER_LEG]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_LEFT_UPPER_LEG], R_leg);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_LEFT_UPPER_LEG],
                     position.x - 0.15f, position.y + 0.45f, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_LEFT_UPPER_LEG], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Left lower leg
    dMassSetCylinder(&m, 1, 3, legRadius, legLength);
    dMassAdjust(&m, 8.0f);
    ragdoll->bodies[RAGDOLL_LEFT_LOWER_LEG] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_LEFT_LOWER_LEG], &m);
    //ragdoll->geoms[RAGDOLL_LEFT_LOWER_LEG] = dCreateCylinder(space, legRadius, legLength);
    ragdoll->geoms[RAGDOLL_LEFT_LOWER_LEG] = dCreateCapsule(pctx->space, legRadius, legLength);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_LEFT_LOWER_LEG], ragdoll->bodies[RAGDOLL_LEFT_LOWER_LEG]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_LEFT_LOWER_LEG], R_leg);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_LEFT_LOWER_LEG],
                     position.x - 0.15f, position.y, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_LEFT_LOWER_LEG], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Right upper leg
    dMassSetCylinder(&m, 1, 3, legRadius, legLength);
    dMassAdjust(&m, 8.0f);
    ragdoll->bodies[RAGDOLL_RIGHT_UPPER_LEG] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_RIGHT_UPPER_LEG], &m);
    //ragdoll->geoms[RAGDOLL_RIGHT_UPPER_LEG] = dCreateCylinder(space, legRadius, legLength);
    ragdoll->geoms[RAGDOLL_RIGHT_UPPER_LEG] = dCreateCapsule(pctx->space, legRadius, legLength);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_RIGHT_UPPER_LEG], ragdoll->bodies[RAGDOLL_RIGHT_UPPER_LEG]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_RIGHT_UPPER_LEG], R_leg);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_RIGHT_UPPER_LEG],
                     position.x + 0.15f, position.y + 0.45f, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_RIGHT_UPPER_LEG], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Right lower leg
    dMassSetCylinder(&m, 1, 3, legRadius, legLength);
    dMassAdjust(&m, 8.0f);
    ragdoll->bodies[RAGDOLL_RIGHT_LOWER_LEG] = dBodyCreate(pctx->world);
    dBodySetMass(ragdoll->bodies[RAGDOLL_RIGHT_LOWER_LEG], &m);
    //ragdoll->geoms[RAGDOLL_RIGHT_LOWER_LEG] = dCreateCylinder(space, legRadius, legLength);
    ragdoll->geoms[RAGDOLL_RIGHT_LOWER_LEG] = dCreateCapsule(pctx->space, legRadius, legLength);
    dGeomSetBody(ragdoll->geoms[RAGDOLL_RIGHT_LOWER_LEG], ragdoll->bodies[RAGDOLL_RIGHT_LOWER_LEG]);
    dGeomSetOffsetWorldRotation(ragdoll->geoms[RAGDOLL_RIGHT_LOWER_LEG], R_leg);
    dBodySetPosition(ragdoll->bodies[RAGDOLL_RIGHT_LOWER_LEG],
                     position.x + 0.15f, position.y, position.z);
    dGeomSetData(ragdoll->geoms[RAGDOLL_RIGHT_LOWER_LEG], CreateGeomInfo(true, limbTex, 1.0f, 1.0f));

    // Create joints connecting body parts

    // Neck joint (Hinge between head and torso)
    ragdoll->joints[0] = dJointCreateHinge(pctx->world, 0);
    dJointAttach(ragdoll->joints[0], ragdoll->bodies[RAGDOLL_HEAD], ragdoll->bodies[RAGDOLL_TORSO]);
    dJointSetHingeAnchor(ragdoll->joints[0], position.x, position.y + 1.35f, position.z);
    dJointSetHingeAxis(ragdoll->joints[0], 1, 0, 0);
    dJointSetHingeParam(ragdoll->joints[0], dParamLoStop, -0.5f);
    dJointSetHingeParam(ragdoll->joints[0], dParamHiStop, 0.5f);

    // Left shoulder joint (Universal)
    ragdoll->joints[1] = dJointCreateUniversal(pctx->world, 0);
    dJointAttach(ragdoll->joints[1], ragdoll->bodies[RAGDOLL_TORSO], ragdoll->bodies[RAGDOLL_LEFT_UPPER_ARM]);
    dJointSetUniversalAnchor(ragdoll->joints[1], position.x - 0.3f, position.y + 1.2f, position.z);
    dJointSetUniversalAxis1(ragdoll->joints[1], 0, 0, 1);
    dJointSetUniversalAxis2(ragdoll->joints[1], 1, 0, 0);
    // Axis 1 (Z): limit forward/backward swing
    dJointSetUniversalParam(ragdoll->joints[1], dParamLoStop, -2.0f);
    dJointSetUniversalParam(ragdoll->joints[1], dParamHiStop, 1.5f);
    // Axis 2 (X): limit side-to-side movement
    dJointSetUniversalParam(ragdoll->joints[1], dParamLoStop2, -1.5f);
    dJointSetUniversalParam(ragdoll->joints[1], dParamHiStop2, 1.5f);

    // Left elbow joint (Hinge) - only bends backward
    ragdoll->joints[2] = dJointCreateHinge(pctx->world, 0);
    dJointAttach(ragdoll->joints[2], ragdoll->bodies[RAGDOLL_LEFT_UPPER_ARM], ragdoll->bodies[RAGDOLL_LEFT_LOWER_ARM]);
    const dReal* leftElbowPos = dBodyGetPosition(ragdoll->bodies[RAGDOLL_LEFT_LOWER_ARM]);
    dJointSetHingeAnchor(ragdoll->joints[2], leftElbowPos[0] + armLength/2, leftElbowPos[1], leftElbowPos[2]);
    dJointSetHingeAxis(ragdoll->joints[2], 0, 0, 1);
    dJointSetHingeParam(ragdoll->joints[2], dParamLoStop, 0.0f);      // Can't bend forward
    dJointSetHingeParam(ragdoll->joints[2], dParamHiStop, 2.5f);      // Max bend ~143 degrees

    // Right shoulder joint (Universal)
    ragdoll->joints[3] = dJointCreateUniversal(pctx->world, 0);
    dJointAttach(ragdoll->joints[3], ragdoll->bodies[RAGDOLL_TORSO], ragdoll->bodies[RAGDOLL_RIGHT_UPPER_ARM]);
    dJointSetUniversalAnchor(ragdoll->joints[3], position.x + 0.3f, position.y + 1.2f, position.z);
    dJointSetUniversalAxis1(ragdoll->joints[3], 0, 0, 1);
    dJointSetUniversalAxis2(ragdoll->joints[3], 1, 0, 0);
    // Axis 1 (Z): limit forward/backward swing
    dJointSetUniversalParam(ragdoll->joints[3], dParamLoStop, -2.0f);
    dJointSetUniversalParam(ragdoll->joints[3], dParamHiStop, 1.5f);
    // Axis 2 (X): limit side-to-side movement
    dJointSetUniversalParam(ragdoll->joints[3], dParamLoStop2, -1.5f);
    dJointSetUniversalParam(ragdoll->joints[3], dParamHiStop2, 1.5f);

    // Right elbow joint (Hinge) - only bends backward
    ragdoll->joints[4] = dJointCreateHinge(pctx->world, 0);
    dJointAttach(ragdoll->joints[4], ragdoll->bodies[RAGDOLL_RIGHT_UPPER_ARM], ragdoll->bodies[RAGDOLL_RIGHT_LOWER_ARM]);
    const dReal* rightElbowPos = dBodyGetPosition(ragdoll->bodies[RAGDOLL_RIGHT_LOWER_ARM]);
    dJointSetHingeAnchor(ragdoll->joints[4], rightElbowPos[0] - armLength/2, rightElbowPos[1], rightElbowPos[2]);
    dJointSetHingeAxis(ragdoll->joints[4], 0, 0, 1);
    dJointSetHingeParam(ragdoll->joints[4], dParamLoStop, 0.0f);      // Can't bend forward
    dJointSetHingeParam(ragdoll->joints[4], dParamHiStop, 2.5f);      // Max bend ~143 degrees

    // Left hip joint (Universal)
    ragdoll->joints[5] = dJointCreateUniversal(pctx->world, 0);
    dJointAttach(ragdoll->joints[5], ragdoll->bodies[RAGDOLL_TORSO], ragdoll->bodies[RAGDOLL_LEFT_UPPER_LEG]);
    dJointSetUniversalAnchor(ragdoll->joints[5], position.x - 0.15f, position.y + 0.6f, position.z);
    dJointSetUniversalAxis1(ragdoll->joints[5], 1, 0, 0);
    dJointSetUniversalAxis2(ragdoll->joints[5], 0, 0, 1);
    // Axis 1 (X): limit leg swing forward/back
    dJointSetUniversalParam(ragdoll->joints[5], dParamLoStop, -1.5f);
    dJointSetUniversalParam(ragdoll->joints[5], dParamHiStop, 2.0f);
    // Axis 2 (Z): limit leg rotation
    dJointSetUniversalParam(ragdoll->joints[5], dParamLoStop2, -1.0f);
    dJointSetUniversalParam(ragdoll->joints[5], dParamHiStop2, 1.0f);

    // Left knee joint (Hinge) - only bends backward
    ragdoll->joints[6] = dJointCreateHinge(pctx->world, 0);
    dJointAttach(ragdoll->joints[6], ragdoll->bodies[RAGDOLL_LEFT_UPPER_LEG], ragdoll->bodies[RAGDOLL_LEFT_LOWER_LEG]);
    const dReal* leftKneePos = dBodyGetPosition(ragdoll->bodies[RAGDOLL_LEFT_LOWER_LEG]);
    dJointSetHingeAnchor(ragdoll->joints[6], leftKneePos[0], leftKneePos[1] + legLength/2, leftKneePos[2]);
    dJointSetHingeAxis(ragdoll->joints[6], 1, 0, 0);
    dJointSetHingeParam(ragdoll->joints[6], dParamLoStop, 0.0f);       // Can't bend forward
    dJointSetHingeParam(ragdoll->joints[6], dParamHiStop, 2.5f);       // Max bend ~143 degrees

    // Right hip joint (Universal)
    ragdoll->joints[7] = dJointCreateUniversal(pctx->world, 0);
    dJointAttach(ragdoll->joints[7], ragdoll->bodies[RAGDOLL_TORSO], ragdoll->bodies[RAGDOLL_RIGHT_UPPER_LEG]);
    dJointSetUniversalAnchor(ragdoll->joints[7], position.x + 0.15f, position.y + 0.6f, position.z);
    dJointSetUniversalAxis1(ragdoll->joints[7], 1, 0, 0);
    dJointSetUniversalAxis2(ragdoll->joints[7], 0, 0, 1);
    // Axis 1 (X): limit leg swing forward/back
    dJointSetUniversalParam(ragdoll->joints[7], dParamLoStop, -1.5f);
    dJointSetUniversalParam(ragdoll->joints[7], dParamHiStop, 2.0f);
    // Axis 2 (Z): limit leg rotation
    dJointSetUniversalParam(ragdoll->joints[7], dParamLoStop2, -1.0f);
    dJointSetUniversalParam(ragdoll->joints[7], dParamHiStop2, 1.0f);

    // Right knee joint (Hinge) - only bends backward
    ragdoll->joints[8] = dJointCreateHinge(pctx->world, 0);
    dJointAttach(ragdoll->joints[8], ragdoll->bodies[RAGDOLL_RIGHT_UPPER_LEG], ragdoll->bodies[RAGDOLL_RIGHT_LOWER_LEG]);
    const dReal* rightKneePos = dBodyGetPosition(ragdoll->bodies[RAGDOLL_RIGHT_LOWER_LEG]);
    dJointSetHingeAnchor(ragdoll->joints[8], rightKneePos[0], rightKneePos[1] + legLength/2, rightKneePos[2]);
    dJointSetHingeAxis(ragdoll->joints[8], 1, 0, 0);
    dJointSetHingeParam(ragdoll->joints[8], dParamLoStop, 0.0f);        // Can't bend forward
    dJointSetHingeParam(ragdoll->joints[8], dParamHiStop, 2.5f);        // Max bend ~143 degrees
    
    // register the bodies with the framework 
    for (int i=0; i<ragdoll->bodyCount; i++)
    {
		entity* ent = RL_MALLOC(sizeof(entity));
		ent->body = ragdoll->bodies[i];
		dBodySetData(ragdoll->bodies[i], ent);
		ent->node = clistAddNode(pctx->objList, ent );
	}

    return ragdoll;
}

// Update rag doll motors 
// motorForces array should have one value per joint for control

/**
 * @brief Update ragdoll joint motors
 *
 * Applies motor forces to the ragdoll's joints for control.
 * This is useful for neural network or reinforcement learning
 * control of the ragdoll.
 *
 * @param ragdoll Pointer to the ragdoll structure
 * @param motorForces Array of motor force values (one per joint)
 *
 * @note Array should have ragdoll->jointCount elements for hinge joints
 * @note For universal joints, uses ragdoll->jointCount * 2 elements
 * @note Force of 0 disables the motor for that joint
 *
 * @see CreateRagdoll
 */
void UpdateRagdollMotors(RagDoll *ragdoll, float *motorForces)
{
    if (!ragdoll || !motorForces) return;

    for (int i = 0; i < ragdoll->jointCount; i++) {
        dJointID joint = ragdoll->joints[i];
        int jointType = dJointGetType(joint);

        // For hinge joints, set velocity and max force for motor control
        if (jointType == dJointTypeHinge) {
            dJointSetHingeParam(joint, dParamVel, motorForces[i]);
            dJointSetHingeParam(joint, dParamFMax, fabsf(motorForces[i]) > 0.001f ? 50.0f : 0.0f);
        }
        // For universal joints, control both axes
        else if (jointType == dJointTypeUniversal) {
            // motorForces[i] controls axis 1, motorForces[i + jointCount] would control axis 2
            dJointSetUniversalParam(joint, dParamVel, motorForces[i]);
            dJointSetUniversalParam(joint, dParamVel2, motorForces[i + ragdoll->jointCount]);
            dJointSetUniversalParam(joint, dParamFMax, fabsf(motorForces[i]) > 0.001f ? 50.0f : 0.0f);
            dJointSetUniversalParam(joint, dParamFMax2, fabsf(motorForces[i + ragdoll->jointCount]) > 0.001f ? 50.0f : 0.0f);
        }
    }
}

// Draw rag doll using the existing drawGeom system

/**
 * @brief Render a ragdoll to the screen
 *
 * Renders all body parts of the ragdoll using the framework's
 * existing rendering system.
 *
 * @param ragdoll Pointer to the ragdoll structure
 * @param ctx Pointer to the graphics context
 *
 * @note Uses the same drawGeom system as other physics bodies
 * @see CreateRagdoll
 */
void DrawRagdoll(RagDoll *ragdoll, struct GraphicsContext* ctx)
{
    if (!ragdoll) return;

    for (int i = 0; i < ragdoll->bodyCount; i++) {
        if (ragdoll->geoms[i]) {
            DrawGeom(ragdoll->geoms[i], ctx);
        }
    }
}


// Free rag doll resources - properly destroys ODE objects before freeing wrapper arrays

/**
 * @brief Free a ragdoll and all its resources
 *
 * Properly destroys all ODE bodies, geoms, joints, and motors
 * associated with the ragdoll, and frees the wrapper structure.
 *
 * @param ctx Pointer to physics context
 * @param ragdoll Pointer to the ragdoll structure to free
 *
 * @note Removes ragdoll from framework's object list
 * @note Frees all allocated memory for bodies, geoms, joints
 *
 * @see CreateRagdoll
 */
void FreeRagdoll(PhysicsContext* ctx, RagDoll *ragdoll)
{
    if (!ragdoll) return;

    // Destroy ODE joints
    if (ragdoll->joints) {
        for (int i = 0; i < ragdoll->jointCount; i++) {
            if (ragdoll->joints[i]) {
                dJointDestroy(ragdoll->joints[i]);
            }
        }
    }

    // Destroy ODE motors
    if (ragdoll->motors) {
        for (int i = 0; i < ragdoll->motorCount; i++) {
            if (ragdoll->motors[i]) {
                dJointDestroy(ragdoll->motors[i]);
            }
        }
    }
    
    // Destroy ODE bodies and geoms (remove from framework tracking too)
    for (int i=0; i<ragdoll->bodyCount; i++)
    {
		entity* ent = dBodyGetData(ragdoll->bodies[i]);
		FreeBodyAndGeoms(ragdoll->bodies[i]);
		clistDeleteNode(ctx->objList, &ent->node);
		RL_FREE(ent);
	}

    // Free wrapper arrays
    if (ragdoll->bodies) RL_FREE(ragdoll->bodies);
    if (ragdoll->geoms) RL_FREE(ragdoll->geoms);
    if (ragdoll->joints) RL_FREE(ragdoll->joints);
    if (ragdoll->motors) RL_FREE(ragdoll->motors);
    
    

    RL_FREE(ragdoll);
}
