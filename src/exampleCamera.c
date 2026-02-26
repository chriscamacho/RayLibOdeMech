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
 * @file exampleCamera.c
 * @brief Camera control implementations
 *
 * This file provides camera control functions for the examples.
 * It includes both a free-fly camera and a vehicle-following camera.
 *
 * @author Chris Camacho (codifies - http://bedroomcoders.co.uk/)
 * @date 2026
 *
 * @section camera_controls Camera Controls
 *
 * Free camera (updateCamera):
 * - W/S: Forward/backward
 * - A/D: Strafe left/right
 * - Q/E: Down/up
 * - Mouse: Look around
 * - Shift: Speed boost
 *
 * Vehicle camera (updateVehicleCamera):
 * - Mouse: Look around relative to vehicle
 * - Camera follows vehicle from behind
 */


// example camera intended for the examples, probably more useful
// to make something more specific for individual projects 


#include "raylibODE.h"
static float cameraYaw; 
static float cameraPitch; 


/**
 * @brief Get current camera yaw angle
 * @return Current yaw angle in radians
 */
float GetCameraYaw() { return cameraYaw; }
void SetCameraYaw(float y) { cameraYaw = y; }

/**
 * @brief Get current camera pitch angle
 * @return Current pitch angle in radians
 */
float GetCameraPitch() { return cameraPitch; }
void SetCameraPitch(float p) { cameraPitch = p; }

/**
 * @brief Initialize the camera
 *
 * Sets up the camera position and orientation for the scene.
 * Should be called once at startup.
 *
 * @param ctx Pointer to graphics context containing camera
 *
 * @note Camera looks at origin by default
 * @note Initializes yaw/pitch based on initial look direction
 */
void SetupCamera(struct GraphicsContext* ctx)
{
	// Define the camera to look into our 3d world
	Vector3 cameraTarget = { 4.0f, 2.0f, 1.0f };
	ctx->camera = (Camera){(Vector3){ 12.0f, 8.0f, 12.0f }, cameraTarget,
						  (Vector3){ 0.0f, 1.0f, 0.0f }, 45.0f, CAMERA_PERSPECTIVE};
    
    // Calculate initial yaw/pitch to look at target
    Vector3 toTarget = Vector3Subtract(cameraTarget, ctx->camera.position);
    float dist = Vector3Length(toTarget);
    cameraYaw = atan2f(toTarget.z, toTarget.x);
    cameraPitch = asinf(toTarget.y / dist);
}

/** @brief update a camera but attach it to a vehicle
 * 
 * if you call this instead of the usual update camera
 * you will be attached to a car, you won't be able to 
 * free roam, but you will be able to mouse look
 * 
 * @param ctx graphics context used for the camera info
 * @param car the vehicle you want to be attached to
 */
void UpdateVehicleCamera(struct GraphicsContext* ctx, vehicle* car)
{
    // Static variables for vehicle camera state (relative to car)
    static float relativeYaw = 0.0f;    // Yaw relative to car's forward
    static float relativePitch = 0.0f;  // Pitch relative to car
    
    // Mouse look (relative to car)
    relativeYaw += GetMouseDelta().x * 0.003f;
    relativePitch += GetMouseDelta().y * 0.003f;
    relativePitch = Clamp(relativePitch, -1.5f, 1.5f);
    
    // Get car position and rotation
    const dReal* carPos = dBodyGetPosition(car->bodies[0]);
    const dReal* carRot = dBodyGetRotation(car->bodies[0]);
    
    // Extract car's local axes from rotation matrix
    Vector3 carForward = { carRot[0], carRot[4], carRot[8] };   // X-axis (forward)
    Vector3 carRight = { carRot[2], carRot[6], carRot[10] };    // Z-axis (right)
    Vector3 carUp = { carRot[1], carRot[5], carRot[9] };        // Y-axis (up)
    
    Vector3 carPosition = { carPos[0], carPos[1], carPos[2] };
    
    // Position camera at car position (offset slightly up and back for better view)
    ctx->camera.position = carPosition;
    ctx->camera.position.y += 4.0f;  // Raise camera to "driver's eye level"
    ctx->camera.position = Vector3Add(ctx->camera.position, Vector3Scale(carForward, -8.0f)); // Slightly back

    
    // Calculate view direction in car's local space
    // Start with car's forward direction, then apply relative yaw and pitch
    Vector3 viewDir = carForward;
    
    // Apply yaw rotation (around car's up axis)
    float cosYaw = cosf(relativeYaw);
    float sinYaw = sinf(relativeYaw);
    viewDir = Vector3Add(
        Vector3Scale(carForward, cosYaw),
        Vector3Scale(carRight, sinYaw)
    );
    
    // Apply pitch (tilt up/down)
    viewDir = Vector3Add(
        Vector3Scale(viewDir, cosf(relativePitch)),
        Vector3Scale(carUp, sinf(relativePitch))
    );
    
    // Normalize view direction
    viewDir = Vector3Normalize(viewDir);
    
    // Set camera target
    ctx->camera.target = Vector3Add(ctx->camera.position, viewDir);
    
    // Update light shader
    SetShaderValue(ctx->shader, ctx->shader.locs[SHADER_LOC_VECTOR_VIEW], &ctx->camera.position.x, SHADER_UNIFORM_VEC3);
}

/** @brief update the camera
 * 
 * This will also move the camera depending on the control keys
 * WASD as expected relative to the view direction,
 * QE move up and down the world axis and shift moves faster
 * 
 * @param ctx pointer to the graphics context
 */
void UpdateCameraControl(struct GraphicsContext* ctx)
{    

	// Update camera with mouse look (first-person style)
	cameraYaw += GetMouseDelta().x * 0.003f;
	cameraPitch += GetMouseDelta().y * 0.003f;
	cameraPitch = Clamp(cameraPitch, -1.5f, 1.5f);
	
	// Calculate forward, right, and up vectors from yaw/pitch
	Vector3 forward = { cosf(cameraYaw) * cosf(cameraPitch), sinf(cameraPitch), sinf(cameraYaw) * cosf(cameraPitch) };
	Vector3 up = { 0.0f, 1.0f, 0.0f };
	Vector3 right = Vector3CrossProduct(forward, up);
	
	// Camera movement speed
	float moveSpeed = 0.1f * GetFrameTime() * 60.0f;
	
	if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
		moveSpeed *= 4.0;
	} 
	
	// W/S - forward/backward in camera direction
	if (IsKeyDown(KEY_W)) {
		ctx->camera.position = Vector3Add(ctx->camera.position, Vector3Scale(forward, moveSpeed));
	}
	if (IsKeyDown(KEY_S)) {
		ctx->camera.position = Vector3Subtract(ctx->camera.position, Vector3Scale(forward, moveSpeed));
	}
	
	// A/D - strafe left/right
	if (IsKeyDown(KEY_D)) {
		ctx->camera.position = Vector3Add(ctx->camera.position, Vector3Scale(right, moveSpeed));
	}
	if (IsKeyDown(KEY_A)) {
		ctx->camera.position = Vector3Subtract(ctx->camera.position, Vector3Scale(right, moveSpeed));
	}
	
	// Q/E - up/down (global Y)
	if (IsKeyDown(KEY_E)) {
		ctx->camera.position.y += moveSpeed;
	}
	if (IsKeyDown(KEY_Q)) {
		ctx->camera.position.y -= moveSpeed;
	}
	
	// Update target based on new position
	ctx->camera.target = Vector3Add(ctx->camera.position, forward);

	// update the light shader with the camera view position
	SetShaderValue(ctx->shader, ctx->shader.locs[SHADER_LOC_VECTOR_VIEW], &ctx->camera.position.x, SHADER_UNIFORM_VEC3);
	
}
