#include "Simulation.h"
#include "CraftProperties.h"
#include "EigenTypes.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>


// transform state from global sensor data
DroneState3d stateTransition(DroneState3d s, Vector3f a, Vector3f w, float dt) {
    // apply the current orientation quaternion backwards (to rotate from local to global instead of global to local)
    Vector3f globalAcceleration = (s.q.inverse() * realImaginaryQuaternion<float>(0., a) * s.q).vec();
    // Vector3f globalAcceleration(a);
    // subtract expected value from it
    globalAcceleration -= G_VEC;
    // compute the new global position and velocity based on global acceleration
    Vector3f p = s.p + s.v * dt + 0.5 * globalAcceleration * dt * dt;
    Vector3f v = s.v + globalAcceleration * dt;

    /*
    new_ω = s.ω .* s.a + α .*  Δt
	new_axis = norm(new_ω) != 0 ? new_ω ./ norm(new_ω) : [0., 0., 0.]
    */
    Vector3f new_w = w;
    Vector3f new_axis = new_w.norm() != 0 ? new_w / new_w.norm() : Vector3f(0, 0, 0);
    Quaternionf new_q = s.q * angleAxisQuaternion<float>(new_w.norm() * dt, new_axis); // s.q * q(s.ω * Δt, new_axis)

    // return DroneState3d(s.p, s.q, s.v, s.w, a);
    return DroneState3d(p, new_q, v, new_w.norm(), new_axis);
}

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
