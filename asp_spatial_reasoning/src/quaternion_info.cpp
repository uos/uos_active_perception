#include <iostream>
#include <cmath>

#include <tf/tf.h>

int main(int argc, char** argv)
{
    double pi = 4.0 * std::atan(1.0);
    tf::Quaternion q(0.8925, -4.664, -0.451, -9.2295);
    tf::Vector3 xr = tf::Transform(q, tf::Vector3(0,0,0))(tf::Vector3(1,0,0));
    double pitch = (pi / 2.0) - tf::tfAngle(xr, tf::Vector3(0,0,1));
    std::cout << "Pitch: " << pitch << " rad or " << pitch * 360.0 / (2.0 * pi) << " deg" << std::endl;
}
