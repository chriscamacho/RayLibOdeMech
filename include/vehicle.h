#ifndef VEHICLE_H
#define VEHICLE_H

#include "raylibODE.h"


vehicle* CreateVehicle(PhysicsContext* pctx, struct GraphicsContext* ctx, Vector3 pos, Vector3 carScale, float wheelRadius, float wheelWidth);
void UpdateVehicle(vehicle* car, float accel, float steer);
void FreeVehicle(PhysicsContext* pctx, vehicle* car);
void UnflipVehicle (vehicle *car);
#endif
