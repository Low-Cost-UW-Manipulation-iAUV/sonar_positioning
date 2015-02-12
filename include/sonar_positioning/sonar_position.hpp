#ifndef __SONAR_POSITIONING__
#define __SONAR_POSITIONING__
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <boost/circular_buffer.hpp>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"


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
    int consecutive_bin_value_threshold;
    int wall_threshold;
    double variance_x, variance_y;
    std::vector<double> x_variance_data;
    std::vector<double> y_variance_data;
    // The width of what we call forwards in RADIANS. i.e. from +10deg to -10deg. View is top down with right hand frame
    std::vector<double> beam_target;
    std::string axis;
    double update_rate;
    ros::Timer timer_update;

    // find the yaw in pool frame
    void tf_pool_sonar(const ros::TimerEvent& );    
    ros::Timer pool_base_link_timer;
    double transform_search_update_rate;
    double pool_bl_yaw, pool_bl_pitch, pool_bl_roll;
    tf::TransformListener listener;


    boost::circular_buffer<std::vector<double> > bvm_data;
    bool bvm_data_setup;
    double valley_limit;
    double mountain_minimum;
    
    double skip_bins;
    double max_bins;
    double old_yaw;
    double threshold;
    boost::circular_buffer<std::vector<double> > sonar_summer;
    std::vector<double> sum;
    bool squared_rolling_setup;

    // Sonar specific 
    double steps2rad(int );
    bool sonar_configured;


    int process_sonar(const std_msgs::Int32MultiArray::ConstPtr&);

    // Sonar signal processing
    int squared_rolling(const std_msgs::Int32MultiArray::ConstPtr& , double * );
   // old sonar processing
    double sonar2Distance(const std_msgs::Int32MultiArray::ConstPtr&, double *);
    int blurred_valleys_mountains(const std_msgs::Int32MultiArray::ConstPtr&, double * );

    int store_variance(double , std::string);

    double hyp2ad(double , double );

    // Track the wall    
    int track_wall(void);
    void timed_wall_tracking(const ros::TimerEvent & );
    double heading_threshold;
    int send_limits_sonar(double, double);

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
