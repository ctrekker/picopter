#include <iostream>
#include <vector>


#include "DroneState.h"
#include "VariadicTable.h"

DroneState3d::DroneState3d() {
    this->p = Vector3f(0., 0., 0.);
    this->q = Quaternion<float>(1., 0., 0., 0.);
    this->v = Vector3f(0., 0., 0.);
    this->w = 0.;
    this->a = Vector3f(0., 0., 0.);
}
DroneState3d::DroneState3d(Vector3f p, Quaternion<float> q, Vector3f v, float w, Vector3f a) {
    this->p = p;
    this->q = q;
    this->v = v;
    this->w = w;
    this->a = a;
}

std::string vector3_string(const Vector3f& v) {
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

std::string _prettyVector(Vector3f v) {
    std::stringstream ss;

    ss << v.x() << " " << v.y() << " " << v.z();
    return ss.str();
}
std::string _prettyQuaternion(Quaternionf q) {
    std::stringstream ss;

    ss << q.w() << " + " << q.x() << "i + " << q.y() << "j + " << q.z() << "k";
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const std::vector<DroneState3d>& ss) {
    VariadicTable<int, std::string, std::string, std::string, float, std::string> vt({"i", "p", "q", "v", "w", "a"}, 20);
    for(int i=0; i<ss.size(); i++) {
        DroneState3d s = ss[i];
        vt.addRow(i, _prettyVector(s.p), _prettyQuaternion(s.q), _prettyVector(s.v), s.w, _prettyVector(s.a));
    }

    vt.print(os);

    return os;
}
