# pragma once

#include <vector>
#include "DroneState.h"
#include "CraftProperties.h"
#include "EigenTypes.h"

typedef Vector4f (*thrust_3d_fn)(DroneState3d, float);
std::vector<DroneState3d> simulate(CraftProperties properties, DroneState3d initialState, thrust_3d_fn P, float dt, float tFinal);
