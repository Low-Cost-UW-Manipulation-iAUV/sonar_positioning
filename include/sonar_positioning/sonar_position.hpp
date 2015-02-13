#ifndef __SONAR_POSITIONING__
#define __SONAR_POSITIONING__
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <boost/circular_buffer.hpp>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>


namespace sonar {

class sonar_position {
public:
    sonar_position(ros::NodeHandle, std::string);
    ~sonar_position();
    void do_subs_pubs(void);

private:
    void sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& );

    // Publish the position
    void publish_position(std::string);
    std::string child_frame_id;

    std::string sonar_name_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_sonar;
    ros::Publisher pub_position;
    ros::ServiceClient sonar_command_service_client;
    std::string sonar_commands_server;

    // Parameter Server stuff
    void get_sonar_calibration_data(void);
    void get_frame_id(void);
    void get_ENU_beam_targets(void);
    void get_processing_parameters(void);
    void get_transform_search_update_rate(void);

    double last_distance;
    double position;
    double angle;
    bool variance_x_found, variance_y_found;
    int calibration_length;
    double variance_x, variance_y;
    std::vector<double> x_variance_data;
    std::vector<double> y_variance_data;
    // The width of what we call forwards in RADIANS. i.e. from +10deg to -10deg. View is top down with right hand frame
    std::vector<double> beam_target;
    double beam_goal;
    std::string axis;
    double update_rate;
    ros::Timer timer_update;

    // find the sonar head yaw in pool frame
    double transform_search_update_rate;
    tf::TransformListener listener;
    void tf_pool_sonar_head(const ros::TimerEvent& event );
    double pool_sh_roll, pool_sh_pitch, pool_sh_yaw;
    ros::Timer pool_sonar_head_timer;

    // broadcast the sonarhead in sonar mount
    void broadcast_sonar_head(double);
    tf::TransformBroadcaster br_sonar_mount_head;
    
    // Sonar specific 
    double steps2rad(int );
    bool sonar_configured;
    int process_sonar(const std_msgs::Int32MultiArray::ConstPtr&);
    double hyp2ad(double);

    // Sonar signal processing
    int squared_rolling(const std_msgs::Int32MultiArray::ConstPtr& , double * );
    double skip_bins;
    double max_bins;
    double old_yaw;
    double threshold;
    boost::circular_buffer<std::vector<double> > sonar_summer;
    std::vector<double> sum;
    bool squared_rolling_setup;

    int store_variance(double , std::string);

    // Track the wall    
    int track_wall(void);
    void timed_wall_tracking(const ros::TimerEvent & );
    double heading_threshold;
    int send_limits_sonar(double, double);
    double get_sonar_mount_heading_pool(void);


    // Math
    double mean(const std::vector<double>);
    double mean(const std::vector<double> , typename std::vector<double>::const_iterator , typename std::vector<double>::const_iterator );
    double std2(const std::vector<double>, const double mean);
    double wrapRad(double);
    double wrapDeg(double);
    int rad2steps(double );
    int deg2steps(double );  
};

} // end of namespace
#endif
