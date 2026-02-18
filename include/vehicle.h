#ifndef VEHICLE_H
#define VEHICLE_H

#include "raylibODE.h"

#define VEH_PART_COUNT 7 // Chassis + 4 wheels + front marker sway
#define WHEEL_COUNT 5

typedef struct vehicle {
    dBodyID bodies[VEH_PART_COUNT];
    dGeomID geoms[VEH_PART_COUNT];
    dJointID joints[WHEEL_COUNT];
    int bodyCount;
} vehicle;

vehicle* CreateVehicle(PhysicsContext* pctx, struct GraphicsContext* ctx, Vector3 pos, Vector3 carScale, float wheelRadius, float wheelWidth);
void UpdateVehicle(vehicle* car, float accel, float steer);
void FreeVehicle(PhysicsContext* pctx, vehicle* car);
void UnflipVehicle (vehicle *car);
#endif
