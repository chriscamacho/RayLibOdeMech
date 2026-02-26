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

#include "raylibODE.h"

#define screenWidth 1920/1.2
#define screenHeight 1080/1.2



int main(void)
{
	// init
    PhysicsContext* physCtx = CreatePhysics();
    GraphicsContext* graphics = CreateGraphics(screenWidth, screenHeight, "Raylib and OpenDE Sandbox");
    SetupCamera(graphics);

    // Create ground plane
    dGeomID planeGeom = dCreateBox(physCtx->space, PLANE_SIZE, PLANE_THICKNESS, PLANE_SIZE);
    dGeomSetPosition(planeGeom, 0, -PLANE_THICKNESS / 2.0, 0);
    dGeomSetData(planeGeom, CreateGeomInfo(true, &graphics->groundTexture, 25.0f, 25.0f));
    clistAddNode(physCtx->statics, planeGeom);



    while (!WindowShouldClose())
    {
		UpdateCameraControl(graphics);
        StepPhysics(physCtx);

        // drawing
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(graphics->camera);
                DrawBodies(graphics, physCtx);
                DrawStatics(graphics, physCtx);
            EndMode3D();

            DrawText("This does nothing use it for your own experiments!", 10, 40, 20, RAYWHITE);

        EndDrawing();
    }

    FreePhysics(physCtx);
    FreeGraphics(graphics);
    CloseWindow();
    return 0;
}
