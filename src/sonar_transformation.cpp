#include <tf/transform_datatypes.h>
#include "sonar_positioning/sonar_transformation.hpp"
using namespace message_filters;
namespace sonar_transform {


sonar_transformer::sonar_transformer(ros::NodeHandle nh) {
    nh_ = nh;
    pub_sequence = 0;
    get_sonar_offset();
    get_broadcast_rate();

    ros::Duration update_freq = ros::Duration(1.0/broadcast_rate);
    pool_sonar_timer = nh_.createTimer(update_freq, &sonar_transformer::bc_pool_sonar, this);

    pub_sonars = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sonar/position", 10);

    sub_sonarBMT.subscribe(nh_, "/sonar/positionBMT", 10);
    sub_sonarUWE.subscribe(nh_, "/sonar/positionUWE", 10);
    sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(2), sub_sonarBMT, sub_sonarUWE);
    sync->registerCallback(boost::bind(&sonar_transformer::sonar_callback, this, _1, _2));
}
sonar_transformer::~sonar_transformer(void) {

}

void sonar_transformer::sonar_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sonarBMT, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sonarUWE ) {
    geometry_msgs::PoseWithCovarianceStamped output;  
    geometry_msgs::PoseStamped temp_in;
    geometry_msgs::PoseStamped temp_out;
    ROS_INFO("repacking the sonar_data");
    // Transform the sonarBMT into the pool frame
    temp_in.header = sonarBMT->header;
    temp_in.pose = sonarBMT->pose.pose;
    try {
        listener.transformPose("pool", temp_in, temp_out);
    } catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
    // store the new position of the sonarBMT and its covariance (x axis)
    output.pose.pose.position.x = temp_out.pose.position.x;
    output.pose.covariance.fill(0.0);
    output.pose.covariance.at(0) = sonarBMT->pose.covariance.at(0);

    // Transform the sonarUWE into the pool frame
    temp_in.header = sonarUWE->header;
    temp_in.pose = sonarUWE->pose.pose;
    try {    
        listener.transformPose("pool", temp_in, temp_out);
    } catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
    // store the new position of the sonarUWE and its covariance (x axis)
    output.pose.pose.position.y = temp_out.pose.position.y;
    output.pose.covariance.at(7) = sonarUWE->pose.covariance.at(7);
    output.pose.pose.orientation.w = 1;

    // prepare the outgoing message
    output.header.seq = pub_sequence;
    pub_sequence ++;

    output.header.stamp = ros::Time::now();
    output.header.frame_id = "pool";
    if(output.pose.pose.position.x != 0 && output.pose.pose.position.y != 0) {
        pub_sonars.publish(output);
    }

}



/** get_sonar_offsets(): get the sonar offset from the parameter server
*/
void sonar_transformer::get_sonar_offset(void) {

    std::vector<double> temp;
    if (!nh_.getParam("/sonar/sonarBMT/position_base_link_frame", temp)) {
        ROS_ERROR("Sonar Transform: Could not find SONAR_BMT position, exiting");
        ros::shutdown();
    }
    bmt_Sx = temp.at(0);
    bmt_Sy = temp.at(1);

    temp.clear();
    if (!nh_.getParam("/sonar/sonarUWE/position_base_link_frame", temp)) {
        ROS_ERROR("Sonar Transform: Could not find SONAR_BMT position, exiting");
        ros::shutdown();
    }
    uwe_Sx = temp.at(0);
    uwe_Sy = temp.at(1);

    temp.clear();
    if (!nh_.getParam("/sonar/svs/position_base_link_frame", temp)) {
        ROS_ERROR("Sonar Transform: Could not find SVS position, exiting");
        ros::shutdown();
    }
    svs_Sz = temp.at(2);
    svs_Sx = temp.at(0);
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
    catch (tf::TransformException &ex){
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
    transform.setOrigin( tf::Vector3(bmt_dx * -1,0.0,0.0) );

    // broadcast it
    br_pool_sonar.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pool", "SONAR_BMT"));

    // Repeat the same for sonarUWE
    double uwe_dy;    
    find_dy(pool_bl_yaw, uwe_Sx, uwe_Sy, uwe_dy);

    // fill and broadcast the transform
    q.setRPY(0.0,0.0,0.0);
    transform.setRotation(q);
    transform.setOrigin( tf::Vector3(0.0,uwe_dy * -1,0.0) );

    // broadcast it
    br_pool_sonar.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pool", "SONAR_UWE"));

   // And finally for the SVS aka dz
    double svs_dz;    
    find_dx(pool_bl_pitch, svs_Sz, svs_Sx, svs_dz);

    // fill and broadcast the transform
    q.setRPY(0.0,0.0,0.0);
    transform.setRotation(q);
    transform.setOrigin( tf::Vector3(0.0,0.0,svs_dz * -1) );

    // broadcast it
    br_pool_sonar.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pool", "SVS"));

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