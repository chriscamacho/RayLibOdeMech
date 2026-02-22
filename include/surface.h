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
 * @file surface.h
 * @brief Physics surface material definitions and lookup table.
 * * Defines the physical properties for different world surfaces to be 
 * used in collision and friction calculations.
 */

#ifndef SURFACE_H
#define SURFACE_H

/**
 * @enum SurfaceType
 * @brief Unique identifiers for different physical materials.
 * * These constants are used as indices into the gSurfaces array.
 * @note SURFACE_COUNT must always remain the last element to allow for loop iteration.
 */
typedef enum
{
    SURFACE_WOOD = 0,   /**< Hard organic surface with moderate friction. */
    SURFACE_METAL,      /**< Smooth metallic surface with high density. */
    SURFACE_ICE,        /**< Low-friction crystalline surface. */
    SURFACE_RUBBER,     /**< High-friction elastic surface. */
    SURFACE_EARTH,      /**< Granular, high-damping surface (soil/dirt). */
    SURFACE_COUNT       /**< The total number of defined surfaces. */
} SurfaceType;

/**
 * @struct SurfaceMaterial
 * @brief Physical coefficients defining how an object interacts with a surface.
 * * This structure holds constants used by the physics engine to resolve 
 * collisions and sliding movement.
 * * * @par Example Usage:
 * @code
 * entity* testBox = CreateBox(physCtx, graphics, (Vector3){1,1,1}, pos, Vector3Zero(), 20.f);
 * dGeomID testGeom = dBodyGetFirstGeom(testBox->body);
 * geomInfo* boxInfo = dGeomGetData(testGeom);
 * * // Assign the specific surface properties using the loop index
 * boxInfo->surface = &gSurfaces[i];
 * @endcode
 * @note the surface should be set on every geom attached to a body you expect to
 * collide, it defaults to SURFACE_EARTH and is not needed for things like triggers
 * of geom's maked as not colliding
 */
typedef struct SurfaceMaterial
{
    float friction;     /**< Coefficient of friction (mu). */
    float bounce;       /**< Coefficient of restitution. Defines energy loss on impact. */
    float bounce_vel;   /**< bounce velocity factor. */
    float slip1;        /**< Primary slip coefficient. */
    float slip2;        /**< Secondary slip coefficient. */
} SurfaceMaterial;

/**
 * @brief Global lookup table for surface properties.
 * This array is indexed by @ref SurfaceType. and contains default
 * values for each surface type
 */
extern SurfaceMaterial gSurfaces[SURFACE_COUNT];

#endif // SURFACE_H
