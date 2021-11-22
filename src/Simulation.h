#pragma once

#include <vector>
#include "DroneState.h"
#include "CraftProperties.h"
#include "EigenTypes.h"

#define G_ACC 9.80665
const Eigen::Vector3f G_VEC(0, 0, G_ACC);

typedef Vector4f (*thrust_3d_fn)(DroneState3d, float);
RealtimeDroneState realtimeTransition(RealtimeDroneState currentState, Eigen::Vector3f a, Eigen::Vector3f w, float dt);
std::vector<DroneState3d> simulate(CraftProperties properties, DroneState3d initialState, thrust_3d_fn P, float dt, float tFinal);
