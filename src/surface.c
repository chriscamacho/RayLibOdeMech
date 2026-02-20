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

#include "surface.h"

SurfaceMaterial gSurfaces[SURFACE_COUNT] =
{
    // WOOD
    {
        .friction = 2.60f,
        .bounce = 0.02f,
        .bounce_vel = 0.1f,
        .slip1 = 0.001f,
        .slip2 = 0.001f
    },

    // METAL
    {
        .friction = 2.8f,
        .bounce = 0.005f,
        .bounce_vel = 0.05f,
        .slip1 = 0.001f,
        .slip2 = 0.001f
    },

    // ICE
    {
        .friction = .4f,
        .bounce = 0.0f,
        .bounce_vel = 0.0f,
        .slip1 = 0.05f,
        .slip2 = 0.05f
    },

    // RUBBER
    {
        .friction = 2.80f,
        .bounce = 0.85f,
        .bounce_vel = 0.1f,
        .slip1 = 0.0005f,
        .slip2 = 0.0005f
    },

    // EARTH (dirt / ground)
    {
        .friction = 2.9f,
        .bounce = 0.05f,
        .bounce_vel = 0.1f,
        .slip1 = 0.0005f,
        .slip2 = 0.0005f
    }
};
