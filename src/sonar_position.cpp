
#include "sonar_positioning/sonar_position.hpp"


#include <tf/transform_datatypes.h>

#define SURGE 1
#define SWAY 2
namespace sonar {

sonar_position::sonar_position(ros::NodeHandle nh) {
    nh_ = nh;
    sub_imu = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &sonar_position::sub_callback_imu, this);
    sub_sonar = nh_.subscribe<std_msgs::Int32MultiArray>("/sonar", 1, &sonar_position::sub_callback_sonar, this);

    pub_position = nh_.advertise<nav_msgs::Odometry>("sonar/position", 100);
    yaw = 0;
    pitch = 0;
    roll = 0;

    x_position = 0;
    y_position = 0;

    x_position_sum = 0;
    y_position_sum = 0;

    x_position_counter = 0;
    y_position_counter = 0;

    ready_x = false;
    ready_y = false;
    current_sonar_state = 0;


}

sonar_position::~sonar_position() {

}
/** sub_callback_imu(): store the current attitude solution of the system.
        This data will (for the time being) be from the sensor fusion system.
*/
void sonar_position::sub_callback_imu(const nav_msgs::Odometry::ConstPtr& message) {

    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, yaw, pitch);    
    imu_timestamp = message->header.stamp; 
}

/** sub_callback_sonar(): transforms the sonar data and publishes it as a position update for the sensor fusion.
*/
void sonar_position::sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {

    // extract the important data from the sonar data
    int d_hypot = sonar2Distance(message);

    // if we are looking ahead along x
    if(message->data[0] <= 45 && message->data[0] >= -45) {
        double odom_dist = getOdomDistance(d_hypot, (yaw + message->data[0]), pitch);
        
        // are we starting a new set of measurements along the x axis? --> this means the y axis was done and we should publish the data.
        if(current_sonar_state != SURGE) {
            current_sonar_state = SURGE;
            x_position_counter = 1;
            x_position_sum = odom_dist;

            // the y axis must be finished and we should publish its data.
            if(ready_y == true) {
                publish_position();
            }
        } else { // no, we are still working on the x axis.
            x_position_counter ++;
            x_position_sum += odom_dist;

            // this should never happen
            if(x_position_counter == 0) {
                x_position_counter = 1;
                ROS_ERROR("sonar_position: the position counter x was somehow set to 0... This should NEVER happen.");
            } 
            x_position = x_position_sum / x_position_counter;
            // we have stored a real measurement in the x_position 
            ready_x = true;            
        }
    } else if (message->data[0] <= 135 && message->data[0] >45 ) {
        double odom_dist = getOdomDistance(d_hypot, (yaw + message->data[0]), roll);
        
        // are we starting a new set of measurements along the x axis? --> this means the y axis was done and we should publish the data.
        if(current_sonar_state != SWAY) {
            current_sonar_state = SWAY;
            y_position_counter = 1;
            y_position_sum = odom_dist;

            // the y axis must be finished and we should publish its data.
            if(ready_x == true) {
                publish_position();
            }
        } else { // no, we are still working on the x axis.
            y_position_counter ++;
            y_position_sum += odom_dist;

            // this should never happen
            if(y_position_counter == 0) {
                y_position_counter = 1;
                ROS_ERROR("sonar_position: the position counter y was somehow set to 0... This should NEVER happen.");
            } 
            y_position = y_position_sum / y_position_counter;
            // we have stored a real measurement in the x_position 
            ready_y = true;            
        }
    }


}

/** sonar2Distance(): Find the strongest signal in the sonar array.
*/
double sonar_position::sonar2Distance(const std_msgs::Int32MultiArray::ConstPtr& message) {
    // Find the number of data bins
    int numBins = message->data[1];
    int range = message->data[2];

    int max = 0;
    int strongest_bin = 0;
    for (int x=3; x<(range + 3); x++) {
        if (message->data[x] > max) {
            max = message->data[x];
            strongest_bin = x;
        }
    }
    return strongest_bin * range/numBins;
}

/** getOdomDistance(): find the distance to the assumed vertical wall 
            in the odom frame given the distance to the same wall in body frame.
*/
double sonar_position::getOdomDistance(int body_position, double angle_a, double angle_b) {

    // rotate the body frame sonar measurement along the x axis into the odom frame
    double undo_pitch = cos(angle_a) * (double )body_position ;
    double odom_position = cos(angle_b) * undo_pitch;


    return odom_position;

}

/** publish_position(): DOes what is says
*/
void sonar_position::publish_position(void) {
    nav_msgs::Odometry sonar_position;
    sonar_position.pose.pose.position.x = x_position;
    sonar_position.pose.pose.position.y = y_position;
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