#include <tf/transform_datatypes.h>
#include "sonar_positioning/sonar_transformation.hpp"
namespace sonar_transform {


sonar_transformer::sonar_transformer(ros::NodeHandle nh) {
    nh_ = nh;
    
    get_odom_pool_offset();
    get_sonar_offset();
    get_broadcast_rate();
    ros::Duration update_freq = ros::Duration(1.0/broadcast_rate);
    pool_sonar_timer = nh_.createTimer(update_freq, &sonar_transformer::bc_pool_sonar, this);
    odom_pool_timer = nh_.createTimer(update_freq, &sonar_transformer::bc_odom_pool, this);

}
sonar_transformer::~sonar_transformer(void) {

}

/** get_odom_pool_offset(): get the sonar offset from the parameter server
*/
void sonar_transformer::get_odom_pool_offset(void) {

    std::vector<double> temp;
    if (!nh_.getParam("/sonar/odom_pool_rotation/quat", temp)) {
        ROS_ERROR("Sonar Transform: Could not find pool_yaw, exiting");
        ros::shutdown();
    }
    tf::Quaternion q(temp.at(0), temp.at(1), temp.at(2), temp.at(3));
    tf::Matrix3x3 m(q);
    double rubbish;
    m.getRPY(rubbish, rubbish, pool_yaw);
    nh_.setParam("/sonar/odom_pool_rotation/deg", pool_yaw * 180/M_PI);
}

/** get_sonar_offsets(): get the sonar offset from the parameter server
*/
void sonar_transformer::get_sonar_offset(void) {

    std::vector<double> temp;
    if (!nh_.getParam("/sonar/sonar_positioning_BMT/position_base_link_frame", temp)) {
        ROS_ERROR("Sonar Transform: Could not find SONAR_BMT position, exiting");
        ros::shutdown();
    }
    bmt_Sx = temp.at(0);
    bmt_Sy = temp.at(1);

    temp.clear();
    if (!nh_.getParam("/sonar/sonar_positioning_UWE/position_base_link_frame", temp)) {
        ROS_ERROR("Sonar Transform: Could not find SONAR_BMT position, exiting");
        ros::shutdown();
    }
    uwe_Sx = temp.at(0);
    uwe_Sy = temp.at(1);      
}

void sonar_transformer::get_broadcast_rate(void) {
    if (!nh_.getParam("/sonar/broadcast_rate", broadcast_rate)) {
        ROS_ERROR("Sonar Transform: Could not find broadcast_rate, exiting");
        ros::shutdown();
    }
}

/** bc_pool_sonar(): will acctually do the work of broadcasting the transforms
*/
void sonar_transformer::bc_pool_sonar(const ros::TimerEvent& event) {
    tf::StampedTransform found_transform;
    try{
      listener.lookupTransform("/pool", "/base_link", ros::Time(0), found_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // get the rotation from pool to base_link
    tf::Quaternion quat = found_transform.getRotation();
    tf::Matrix3x3 m(quat);
    m.getRPY(pool_bl_roll, pool_bl_pitch, pool_bl_yaw);
    ROS_INFO("sonar_transformer: yaw of pool->base_link : %f", pool_bl_yaw);

    
    // calculate the offset for sonarBMT
    double bmt_dx;    
    find_dx(pool_bl_yaw, bmt_Sx, bmt_Sy, bmt_dx);

    // fill and broadcast the transform
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0.0,0.0,0.0);
    transform.setRotation(q);
    transform.setOrigin( tf::Vector3(bmt_dx,0.0,0.0) );

    // broadcast it
    br_pool_sonar.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pool", "SONAR_BMT"));

    // Repeat the same for sonarUWE
    double uwe_dy;    
    find_dy(pool_bl_yaw, uwe_Sx, uwe_Sy, uwe_dy);

    // fill and broadcast the transform
    q.setRPY(0.0,0.0,0.0);
    transform.setRotation(q);
    transform.setOrigin( tf::Vector3(0.0,uwe_dy,0.0) );

    // broadcast it
    br_pool_sonar.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pool", "SONAR_UWE"));
}

/** bc_odom_pool(): will acctually do the work of broadcasting the transforms
*/
void sonar_transformer::bc_odom_pool(const ros::TimerEvent& event) {
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0,0.0,0.0) );

    tf::Quaternion q;
    q.setRPY(0.0,0.0,pool_yaw);

    transform.setRotation(q);
    br_odom_pool.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "pool"));
}



/** find_dx(): calculates the offset of the sonar in frame 'pool'
*/
void sonar_transformer::find_dx(double alpha, double sx, double sy, double &dx) {
    dx = cos(alpha - atan2(sy,sx)) * sqrt( pow(sx,2) + pow(sy,2) );
}

/** find_dx(): calculates the offset of the sonar in frame 'pool'
*/
void sonar_transformer::find_dy(double alpha, double sx, double sy, double &dy) {
    dy = sin(alpha - atan2(sy,sx)) * sqrt( pow(sx,2) + pow(sy,2) );
}

} // end of namespace


int main(int argc, char **argv){
    ros::init(argc, argv, "sonar_transformer");
    ros::NodeHandle nh;

    sonar_transform::sonar_transformer sonner(nh);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    ROS_INFO("sonar_transformer: Shutting down ");
}