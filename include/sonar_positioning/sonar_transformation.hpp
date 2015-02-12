#ifndef __SONAR_TRANSFORMATION__
#define __SONAR_TRANSFORMATION__
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <tf/transform_broadcaster.h>

namespace sonar_transform {

class sonar_transformer {
public:
    sonar_transformer(ros::NodeHandle);
    ~sonar_transformer();

private:
    void bc_pool_sonar(const ros::TimerEvent&);
    void bc_odom_pool(const ros::TimerEvent&);
    void find_dx(double , double , double, double &);
    void find_dy(double , double , double, double &);
    void get_odom_pool_offset(void);
    void get_sonar_offset(void);
    void get_broadcast_rate(void);

    ros::NodeHandle nh_;
    tf::TransformListener listener;
    tf::TransformBroadcaster br_odom_pool;
    tf::TransformBroadcaster br_pool_sonar;
    ros::Timer pool_sonar_timer;
    ros::Timer odom_pool_timer;

    double broadcast_rate;
    double pool_bl_roll, pool_bl_pitch, pool_bl_yaw;
    double pool_yaw;
    // sonar mounting offsets in base_link
    double bmt_Sx, bmt_Sy, uwe_Sx, uwe_Sy;
};

} // end of namespace

#endif