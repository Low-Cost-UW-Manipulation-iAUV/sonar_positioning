#ifndef __SONAR_POSITIONING__
#define __SONAR_POSITIONING__
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Odometry.h"

namespace sonar {

class sonar_position {
public:
    sonar_position(ros::NodeHandle);
    ~sonar_position();

private:
    void sub_callback_imu(const nav_msgs::Odometry::ConstPtr& );
    void sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& );

    double sonar2Distance(const std_msgs::Int32MultiArray::ConstPtr&);
    double getOdomDistance(int, double, double);
    void publish_position(void);

    ros::NodeHandle nh_;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_sonar;
    ros::Publisher pub_position;

    double yaw, pitch , roll;
    double x_position, y_position;
    double x_position_sum, y_position_sum;
    int x_position_counter, y_position_counter;
    bool ready_x, ready_y;
    ros::Time imu_timestamp;
    unsigned int current_sonar_state;
};

}
#endif