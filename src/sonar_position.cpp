
#include "sonar_positioning/sonar_position.hpp"

#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

namespace sonar {

sonar_position::sonar_position(ros::NodeHandle nh) {
    nh_ = nh;
    sub_imu = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", &sonar_position::sub_callback_imu, this);
    sub_sonar = nh_.subscribe<std_msgs::Int32MultiArray>("/sonar", &sonar_position::sub_callback_sonar, this);
    pub_position = nh_.advertise<nav_msgs::Odometry>("sonar/position", 100);
    yaw = 0;
    pitch = 0;
    roll = 0;

}
/** sub_callback_imu(): store the current attitude solution of the system.
        This data will (for the time being) be from the sensor fusion system.
*/
sonar_position::sub_callback_imu(const nav_msgs::Odometry::ConstPtr& message) {

    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, yaw, pitch);    
    imu_timestamp = message->header.stamp; 
}

/** sub_callback_sonar(): transforms the sonar data and publishes it as a position update for the sensor fusion.
*/
sonar_position::sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {



    //publish the datas
    nav_msgs::Odometry sonar_position;
    sonar_position.pose.pose.x = 0;
    sonar_position.pose.pose.y = 1:
    sonar_position.pose.pose.z = 2;

    pub_position.publish(sonar_position);

}

}  //end of namespace




int main(int argc, char **argv) {
    ros::init(argc, argv, "sonar_positioning");
    ros::NodeHandle nh;
    /// An Async spinner creates another thread which will handle the event of this node being executed.

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // create the instance of the class
    sonar::sonar_position orange_box(nh);

    // register the 
    ros::spin();

    ROS_INFO("imu_positioning: Shutting down ");

}