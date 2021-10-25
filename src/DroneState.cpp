#include "DroneState.h"
#include <iostream>

DroneState3d::DroneState3d() {
    this->p = Vector3d(0., 0., 0.);
    this->q = Quaternion<float>(1., 0., 0., 0.);
    this->v = Vector3d(0., 0., 0.);
    this->w = 0.;
    this->a = Vector3d(0., 0., 0.);
}
DroneState3d::DroneState3d(Vector3d p, Quaternion<float> q, Vector3d v, float w, Vector3d a) {
    this->p = p;
    this->q = q;
    this->v = v;
    this->w = w;
    this->a = a;
}

std::string vector3_string(const Vector3d& v) {
    return std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " + std::to_string(v.z());
}

std::ostream& operator<<(std::ostream& os, const DroneState3d& s) {
    os << "DroneState3d(" << std::endl;
    os << "\tp = " << vector3_string(s.p) << std::endl;
    os << "\tq = " << s.q << std::endl;
    os << "\tv = " << vector3_string(s.v) << std::endl;
    os << "\tw = " << std::to_string(s.w) << std::endl;
    os << "\ta = " << vector3_string(s.a) << std::endl;
    os << ")" << std::endl;
    return os;
}
