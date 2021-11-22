#include <iostream>
#include <vector>


#include "DroneState.h"
#include "VariadicTable.h"


using namespace Eigen;


std::string _prettyVector(Vector3f v) {
    std::stringstream ss;

    ss << v.x() << " " << v.y() << " " << v.z();
    return ss.str();
}
std::string _prettyVector2(Vector2f v) {
    std::stringstream ss;

    ss << v.x() << " " << v.y();
    return ss.str();
}
std::string _prettyQuaternion(Quaternionf q) {
    std::stringstream ss;

    ss << q.w() << " + " << q.x() << "i + " << q.y() << "j + " << q.z() << "k";
    return ss.str();
}



/* == DroneState3d == */
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

std::ostream& operator<<(std::ostream& os, const DroneState3d& s) {
    os << "DroneState3d(" << std::endl;
    os << "\tp = " << _prettyVector(s.p) << std::endl;
    os << "\tq = " << s.q << std::endl;
    os << "\tv = " << _prettyVector(s.v) << std::endl;
    os << "\tw = " << std::to_string(s.w) << std::endl;
    os << "\ta = " << _prettyVector(s.a) << std::endl;
    os << ")" << std::endl;
    return os;
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


/* 
== DroneStateRealtime ==
Just like DroneState3d but only contains relevant information for control (for now)
*/
RealtimeDroneState::RealtimeDroneState() {
    this->q = Vector2f(0., 0.);
    this->w = Vector3f(0., 0., 0.);
}
RealtimeDroneState::RealtimeDroneState(Vector2f q, Vector3f w) {
    this->q = q;
    this->w = w;
}

std::ostream& operator<<(std::ostream& os, const RealtimeDroneState& s) {
    os << "RealtimeDroneState(" << std::endl;
    os << "\tq = " << _prettyVector2(s.q) << std::endl;
    os << "\tw = " << _prettyVector(s.w) << std::endl;
    os << ")" << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<RealtimeDroneState>& ss) {
    VariadicTable<int, std::string, std::string> vt({"i", "q", "w"}, 20);
    for(int i=0; i<ss.size(); i++) {
        RealtimeDroneState s = ss[i];
        vt.addRow(i, _prettyVector2(s.q), _prettyVector(s.w));
    }

    vt.print(os);

    return os;
}
