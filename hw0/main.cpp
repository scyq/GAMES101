#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

#define pi acos(-1)

using namespace std;

int main()
{

    Eigen::Vector3f v(2.0f, 1.0f, 1.0f);
    Eigen::Matrix3f trans;
    
    float alpha = 45 * pi / 180;

    trans << 
        cos(alpha), -sin(alpha), 1,
        sin(alpha), cos(alpha), 2,
        0, 0, 1;

    cout << trans * v << endl;

    return 0;
}