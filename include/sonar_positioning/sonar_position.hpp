#ifndef __SONAR_POSITIONING__
#define __SONAR_POSITIONING__
#include "ros/ros.h"

namespace sonar {

class sonar_position {
public:
    sonar_position(ros::Nodehandle);
    ~sonar_position();
private:
    ros::Nodehandle nh_;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_sonar;
    ros::Publisher pub_position;

    double yaw, pitch , roll;

};

}
#endif