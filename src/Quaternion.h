#ifndef H_QUATERNION
#define H_QUATERNION

class Quaternion {
public:
    float r;
    float i;
    float j;
    float k;

    Quaternion(float r, float i, float j, float k);
};

std::ostream& operator<<(std::ostream &s, const Quaternion &q);
Quaternion operator+(Quaternion &q1, Quaternion &q2);

#endif