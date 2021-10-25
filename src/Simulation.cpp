#include "Simulation.h"
#include "CraftProperties.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

std::vector<DroneState3d> simulate(CraftProperties properties, DroneState3d initialState, thrust_3d_fn P, float dt, float tFinal) {
    std::vector<DroneState3d> states;
    states.push_back(initialState);

    DroneState3d s = initialState;
    for(float t = dt; t <= tFinal; t += dt) {
        Vector4f T = P(s, t);
        Vector3f T_dir = (s.q * Quaternion<float>(0, 0, 0, T.norm())  * s.q.inverse()).vec();

        // linear dynamics
        Vector3f a = T_dir / properties.mass + properties.g;
        Vector3f p = s.p + s.v * dt;
        Vector3f v = s.v + a * dt;

        // rotational dynamics
        Vector3f torque1 = properties.radius * (T[1] - T[3]) * Vector3f(0, 1, 0);
        Vector3f torque2 = properties.radius * (T[2] - T[0]) * Vector3f(1, 0, 0);
        Vector3f netTorque = torque1 + torque2;
        Vector3f angular_a = netTorque / properties.rotationalInertia;

        Vector3f new_w = s.w * s.a + angular_a * dt;
        float w = new_w.norm();
        Vector3f newAxis = new_w.norm() != 0 ? new_w / new_w.norm() : Vector3f(0, 0, 0);

        Quaternion<float> q = s.q * angleAxisQuaternion<float>(s.w * dt, newAxis);


        // next state
        s = DroneState3d(p, q, v, w, newAxis);
        states.push_back(s);
    }

    return states;
}
