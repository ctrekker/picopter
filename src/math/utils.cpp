#include "utils.h"

#include <vector>
#include <Eigen/Dense>
#include <math.h>


using namespace Eigen;


Vector3f _sum(std::vector<Vector3f> vecs) {
    Vector3f agg(0., 0., 0.);

    for(int i=0; i<vecs.size(); i++) {
        agg += vecs[i];
    }

    return agg;
}
Vector3f _mean(std::vector<Vector3f> vecs) {
    return _sum(vecs) / vecs.size();
}
float _dot(Vector3f u, Vector3f v) {
    return u.transpose() * v;
}
float _angleBetween(Vector3f vec1, Vector3f vec2) {
    float dotProduct = _dot(vec1, vec2);
    float normProduct = vec1.norm() * vec2.norm();
    float in = dotProduct / normProduct;
    return acos(in);
}
Vector3f _proj(Vector3f u, Vector3f v) {
    return (_dot(u, v) / _dot(v, v)) * v;
}
