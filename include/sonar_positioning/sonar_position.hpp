#ifndef __SONAR_POSITIONING__
#define __SONAR_POSITIONING__
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"

namespace sonar {

class sonar_position {
public:
    sonar_position(ros::NodeHandle);
    ~sonar_position();
private:
    void sub_callback_imu(const sensor_msgs::Imu::ConstPtr& );
    void sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& );

    double sonar2Distance(const std_msgs::Int32MultiArray::ConstPtr&);
    double getOdomDistance(float, double, double);
    void publish_position_x(void);
    void publish_position_y(void);
    void configure_sonar (void);

    ros::NodeHandle nh_;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_sonar;
    ros::Publisher pub_position_x;
    ros::Publisher pub_position_y;
    ros::Publisher pub_sonar_command;
    double yaw, pitch, roll;
    double last_distance;
    
    double x_position, y_position;
    bool variance_x_found, variance_y_found;
    bool not_yet_configured;
    ros::Time imu_timestamp;

    int calibration_length;
    int consecutive_bin_value_threshold;
    int wall_threshold;
    double variance_x, variance_y;
    std::vector<double> x_variance_data;
    std::vector<double> y_variance_data;
    // The width of what we call forwards in RADIANS. i.e. from +10deg to -10deg. View is top down with right hand frame
    std::vector<double> beam_width_x;
    std::vector<double> beam_width_y;

    double steps2rad(int );
    int rad2steps(double );
    int deg2steps(double );



    double mean(const std::vector<double>);
    double std2(const std::vector<double>, const double mean);    
};

}
#endif
