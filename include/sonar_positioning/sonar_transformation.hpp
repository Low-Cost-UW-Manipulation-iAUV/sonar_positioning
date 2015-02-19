#ifndef __SONAR_TRANSFORMATION__
#define __SONAR_TRANSFORMATION__
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
namespace sonar_transform {

class sonar_transformer {
public:
    sonar_transformer(ros::NodeHandle);
    ~sonar_transformer();

private:
    void bc_pool_sonar(const ros::TimerEvent&);
    void bc_sonar_mount_bl(const ros::TimerEvent& event);

    void find_dx(double , double , double, double &);
    void find_dy(double , double , double, double &);
    void get_sonar_offset(void);
    void get_broadcast_rate(void);

    ros::NodeHandle nh_;
    tf::TransformListener listener;
    tf::TransformBroadcaster br_pool_sonar;
    ros::Timer pool_sonar_timer;
    ros::Publisher pub_sonars;
    double broadcast_rate;
    double pool_bl_roll, pool_bl_pitch, pool_bl_yaw;
    // sonar mounting offsets in base_link
    double bmt_Sx, bmt_Sy, uwe_Sx, uwe_Sy, svs_Sx, svs_Sz;
    double last_sonar_x, last_sonar_y;
    // Message Filtering
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_sonarBMT;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_sonarUWE;
    typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> * sync;

    void sonar_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& , const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );
    int pub_sequence;
};

} // end of namespace

#endif