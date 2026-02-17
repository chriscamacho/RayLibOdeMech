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
 * @file collision.c
 * @brief Collision detection and response for the physics framework
 *
 * This file handles all collision detection between physics bodies
 * using ODE's collision system. It provides the nearCallback function
 * that is called by ODE during collision detection.
 *
 * @author Chris Camacho (codifies - http://bedroomcoders.co.uk/)
 * @date 2026
 *
 * @note This module uses dSpaceCollide for broad-phase collision detection
 * @note Maximum contacts per collision pair is limited to 8
 */

#include "raylibODE.h"

#define MAX_CONTACTS 8

void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

        
    geomInfo* gi1 = (geomInfo*)dGeomGetData(o1);
    geomInfo* gi2 = (geomInfo*)dGeomGetData(o2);	// Check for trigger callbacks
    
    if (gi1 && gi1->triggerOnCollide) {
        gi1->triggerOnCollide(o1, o2);
        return; // Skip physical resolution
    }
    if (gi2 && gi2->triggerOnCollide) {
        gi2->triggerOnCollide(o2, o1);
        return; // Skip physical resolution
    }

    if (gi1 && !gi1->collidable) return;
    if (gi2 && !gi2->collidable) return;
    
    dContact contact[MAX_CONTACTS]; 
    int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    if (numc > 0) {
        struct PhysicsContext* ctx = (struct PhysicsContext*)data;

        for (int i = 0; i < numc; i++) {
			contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
										dContactSoftERP | dContactSoftCFM | dContactApprox1;
			contact[i].surface.mu = 1000;
			contact[i].surface.slip1 = 0.0001;
			contact[i].surface.slip2 = 0.0001;
			contact[i].surface.soft_erp = 0.1;
			contact[i].surface.soft_cfm = 0.001;
		  
			contact[i].surface.bounce = 0.001;
			contact[i].surface.bounce_vel = 0.001;

            dJointID c = dJointCreateContact(ctx->world, ctx->contactgroup, &contact[i]);
            dJointAttach(c, b1, b2);
        }
    }
}
