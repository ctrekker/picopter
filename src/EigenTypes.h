#include <math.h>
#include <Eigen/Geometry>

typedef Matrix<float, 4, 1> Vector4f;

template <typename T>
Quaternion<T> realImaginaryQuaternion(T real, Vector3<T> imaginary) {
    return Quaternion<T>(real, imaginary.x(), imaginary.y(), imaginary.z());
}
template <typename T>
Quaternion<T> angleAxisQuaternion(float angle, Vector3f axis) {
    Vector3<T> im = sin(angle/2) * axis;
    return realImaginaryQuaternion<T>(cos(angle/2), im);
}
