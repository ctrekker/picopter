#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "src/Quaternion.h"

using Eigen::MatrixXd;

int main()
{
    Quaternion q1(1., 0., 0., 0.);
    Quaternion q2(0., 1., 0., 0.);

    std::cout << (q1 + q2) << std::endl;

MatrixXd m(2,2);
  m(0,0) = 3;
    m(1,0) = 2.5;
      m(0,1) = -1;
        m(1,1) = m(1,0) + m(0,1);
	  std::cout << m << std::endl;
}
