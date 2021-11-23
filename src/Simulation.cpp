#include "Simulation.h"
#include "CraftProperties.h"
#include "EigenTypes.h"
#include "math/utils.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>


using namespace Eigen;


// weight given to gyroscope for orientation
// inverse (1-mu) weight given to accelerometer for orientation
#define mu 0.98


float a = 1/sqrt(2);
Matrix<float, 2, 2> Ainv {
    { a, a},
    {-a, a}
};
Matrix<float, 3, 3> Ainv3 {
    { a, a, 0},
    {-a, a, 0},
    { 0, 0, 1}
};


// transform state from global sensor data
RealtimeDroneState realtimeTransition(RealtimeDroneState s, Vector3f a, Vector3f w, float dt) {
    // estimate orientation from acceleration vector
    Vector3f an = a / a.norm() * G_VEC.norm();
    an = Ainv3 * an;  // transform into our more convenient diagonal space
    Vector2f a_xz(an.x(), an.z());
    Vector2f a_yz(an.y(), an.z());
    Vector2f qa(_sign(an.y()) * acos(a_xz.norm() / G_VEC.norm()), -_sign(an.x()) * acos(a_yz.norm() / G_VEC.norm()));

    // transformed angular velocity
    Vector2f wxy(w.x(), w.y());
    Vector2f w45 = Ainv * wxy;
    Vector2f dq(w45.x() * dt, w45.y() * dt);

    return RealtimeDroneState(mu * (s.q + dq) + (1 - mu) * qa, Vector3f(w45.x(), w45.y(), w.z()));
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
