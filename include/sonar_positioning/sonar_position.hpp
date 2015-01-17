#ifndef __SONAR_POSITIONING__
#define __SONAR_POSITIONING__
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

namespace sonar {

class sonar_position {
public:
    sonar_position(ros::NodeHandle, std::string);
    ~sonar_position();
    void do_subs_pubs(void);

private:
    void sub_callback_imu(const nav_msgs::Odometry::ConstPtr&);
    void sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& );

    double sonar2Distance(const std_msgs::Int32MultiArray::ConstPtr&);
    double getOdomDistance(float, double, double);
    void publish_position(std::string);
    void send_limits_sonar(double, double);
    void publish_sonar_beam_transform(double,std::string,std::string);
    double wrapRad(double);
    double wrapDeg(double);

    std::string sonar_name_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_sonar;
    ros::Publisher pub_position;
    ros::Publisher pub_sonar_command;
    double yaw, pitch, roll;
    double last_distance;
    double position;
    double angle;
    double offset_angle;
    bool variance_x_found, variance_y_found;
    ros::Time imu_timestamp;

    int calibration_length;
    int consecutive_bin_value_threshold;
    int wall_threshold;
    double variance_x, variance_y;
    std::vector<double> x_variance_data;
    std::vector<double> y_variance_data;
    // The width of what we call forwards in RADIANS. i.e. from +10deg to -10deg. View is top down with right hand frame
    std::vector<double> beam_target;
    std::string axis;


    // transform broadcaster that will publish the 'virtual' beam heading of the sonars
    tf::TransformBroadcaster transformer;
    std::string child_frame_id;
    std::string parent_frame_id;

    double steps2rad(int );
    int rad2steps(double );
    int deg2steps(double );
    int process_sonar(const std_msgs::Int32MultiArray::ConstPtr&);
    void get_sonar_calibration_data(void);
    void get_transform_parameters(void);
    void get_ENU_beam_targets(void);
    void get_angular_offset(void);
    void get_processing_parameters(void);
    int store_variance(double , std::string);
    void track_wall(void);


    double mean(const std::vector<double>);
    double std2(const std::vector<double>, const double mean);    
};

}
#endif
