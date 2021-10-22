#include <iostream>
#include <vector>
#include <string>

#include "src/Quaternion.h"

int main()
{
    Quaternion q1(1., 0., 0., 0.);
    Quaternion q2(0., 1., 0., 0.);

    std::cout << (q1 + q2) << std::endl;
}
