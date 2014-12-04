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
    double getOdomDistance(float, double, double);
    void publish_position_x(void);
    void publish_position_y(void);

    ros::NodeHandle nh_;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_sonar;
    ros::Publisher pub_position_x;
    ros::Publisher pub_position_y;

    double yaw, pitch, roll;
    double last_distance;
    
    double x_position, y_position;
    double x_position_sum, y_position_sum;
    int x_position_counter, y_position_counter;
    bool variance_x_found, variance_y_found;
    ros::Time imu_timestamp;
    int samples_per_direction;

    int calibration_length;
    double variance_x, variance_y;
    std::vector<double> x_variance_data;
    std::vector<double> y_variance_data;

    // The width of what we call forwards in RADIANS. i.e. from +10deg to -10deg. View is top down with right hand frame
    std::vector<double> beam_width_x;
    std::vector<double> beam_width_y;


    double mean(const std::vector<double>);
    double std2(const std::vector<double>, const double mean);    
};

}
#endif
