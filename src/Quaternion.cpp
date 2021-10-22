#include <iostream>
#include "Quaternion.h"

Quaternion::Quaternion(float r, float i, float j, float k) {
    this->r = r;
    this->i = i;
    this->j = j;
    this->k = k;
}

std::ostream& operator<<(std::ostream &s, const Quaternion &q) {
    s << "Quaternion(" << q.r << " + " << q.i << "i + " << q.j << "j + " << q.k << "k" << ")";
    return s;
}
Quaternion operator+(Quaternion &q1, Quaternion &q2) {
    Quaternion q(q1.r + q2.r, q1.i + q2.i, q1.j + q2.j, q1.k + q2.k);
    return q;
}
