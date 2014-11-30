
#include "sonar_positioning/sonar_position.hpp"


#include <tf/transform_datatypes.h>

#define SURGE 1
#define SWAY 2
namespace sonar {

sonar_position::sonar_position(ros::NodeHandle nh) {
    nh_ = nh;
    sub_imu = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &sonar_position::sub_callback_imu, this);
    sub_sonar = nh_.subscribe<std_msgs::Int32MultiArray>("/sonarData", 1, &sonar_position::sub_callback_sonar, this);

    pub_position_x = nh_.advertise<nav_msgs::Odometry>("/sonar/position/x", 100);
    pub_position_y = nh_.advertise<nav_msgs::Odometry>("/sonar/position/y", 100);


    if (!nh_.getParam("/sonar/calibration_length", calibration_length)) {

        ROS_ERROR("Sonar Position: Could not find calibration_length, assuming 1000. \n");
        calibration_length = 1000;
        nh_.setParam("/sonar/calibration_length", calibration_length);
    }  


    x_variance_data.clear();
    x_variance_data.reserve(calibration_length);
    y_variance_data.clear();
    y_variance_data.reserve(calibration_length);

    yaw = 0;
    pitch = 0;
    roll = 0;

    x_position_sum = 0;
    y_position_sum = 0;

    x_position_counter = 1;
    y_position_counter = 1;

    // check if the variance for x is available on the parameter server
    if (!nh_.getParam("/sonar/variance/x", variance_x)) {

        ROS_ERROR("Sonar Position: Could not find variance of x, will determine it now.");
        variance_x = 0;
        variance_x_found = false;
    } else { // if it is available skip the determination
        variance_x_found = true;
    }

    // check for y as well.
    if (!nh_.getParam("/sonar/variance/y", variance_y)) {

        ROS_ERROR("Sonar Position: Could not find variance of y, will determine it now.");
        variance_y = 0;
        variance_y_found = false;
    } else { // if it is available skip the determination
        variance_y_found = true;
    }    


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

    int binsperaxis = message->data[1];
    double angle = message->data[0]/6392*360; ///THIS WILL NBEED CHANGING
    ROS_INFO("Angle is: %f - change the angle value soon!!!", angle);
    // extract the important data from the sonar data
    int d_hypot = sonar2Distance(message);

    // if we are looking ahead (along x)
    if(angle <= 45 && angle >= -45) {
        double odom_dist = getOdomDistance(d_hypot, (yaw + message->data[0]), pitch);
        x_position_counter ++;
        x_position_sum += odom_dist;

        // this should never happen
        if(x_position_counter == 0) {
            x_position_counter = 1;
            ROS_ERROR("sonar_position: the position counter x was somehow set to 0... This should NEVER happen.");
        } 
        // if we know the variance of the sonar positioning data
        if(variance_x_found == true) {
            // is the x axis measurement done? 
            if(x_position_counter >= binsperaxis) {
                // calculate the mean of the data.
                x_position = x_position_sum / x_position_counter;  

                // publish it              
                publish_position_x();
                x_position_counter = 1;
                x_position_sum = odom_dist;
            }
        } else { // we haven't got any variance data.
            if(x_variance_data.size() < calibration_length) {
                x_variance_data.push_back(x_position);
                ROS_INFO("sonar_position: Determining Sonar variance on x - Sample %lu of %d",x_variance_data.size(), calibration_length);
            } else {
                variance_x = pow( std2(x_variance_data, mean(x_variance_data) ), 2);
                ROS_INFO("sonar_position: Variance of x is %f, stored on param server", variance_x);
                nh_.setParam("/sonar/variance/x", variance_x);

                variance_x_found = true;
            }

        } // We are looking at the y axis
    } else if (angle <= 135 && angle >45 ) {
        // find the adjascent (distance along y in 'odom' frame) if the d_hypot is the hypothenuse.
            // We first look top down: undo the yaw and sonar angle. And then look at it from the side to counteract the current roll.
        double odom_dist = getOdomDistance(d_hypot, (yaw + message->data[0]), roll);
        y_position_counter ++;
        y_position_sum += odom_dist;

        // this should never happen
        if(y_position_counter == 0) {
            y_position_counter = 1;
            ROS_ERROR("sonar_position: the position counter y was somehow set to 0... This should NEVER happen.");
        } 
        // if we know the variance of the sonar positioning data
        if(variance_y_found == true) {
            // is the x axis measurement done? 
            if(y_position_counter >= binsperaxis) {
                // calculate the mean of the data.
                y_position = y_position_sum / y_position_counter;  

                // publish it              
                publish_position_y();
                y_position_counter = 1;
                y_position_sum = odom_dist;
            }
        } else { // we haven't got any variance data.
            if(y_variance_data.size() < calibration_length) {
                y_variance_data.push_back(y_position);
                ROS_INFO("sonar_position: Determining Sonar variance on y - Sample %lu of %d",y_variance_data.size(), calibration_length);                
            } else {
                variance_y = pow( std2(y_variance_data, mean(y_variance_data) ), 2);
                ROS_INFO("sonar_position: Variance of y is %f, stored on param server", variance_y);
                nh_.setParam("/sonar/variance/y", variance_y);

                variance_y_found = true;
            }
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
void sonar_position::publish_position_x(void) {
    nav_msgs::Odometry sonar_position;
    sonar_position.header.stamp = ros::Time::now(); 
    sonar_position.header.frame_id = "odom";
    sonar_position.pose.pose.position.x = x_position;
    sonar_position.pose.covariance.fill(0.0);
    // set the variance data for x
    sonar_position.pose.covariance[0] = variance_x;

    pub_position_x.publish(sonar_position);
}
/** publish_position(): DOes what is says
*/
void sonar_position::publish_position_y(void) {
    nav_msgs::Odometry sonar_position;
    sonar_position.header.stamp = ros::Time::now(); 
    sonar_position.header.frame_id = "odom";
    sonar_position.pose.pose.position.y = y_position;
    sonar_position.pose.covariance.fill(0.0);
    // set the variance data for y
    sonar_position.pose.covariance[7] = variance_y;

    pub_position_y.publish(sonar_position);
}

/** mean(): calculates the mean
*/
double sonar_position::mean(const std::vector<double> vec) {
    double sum = 0;
    unsigned int size = 0;
    for (typename std::vector<double>::const_iterator it = vec.begin(); it!=vec.end(); ++it, ++size) { 
        sum += (*it);
        // if size is not == 0, divide by size, else return 0
    }
    return (size)?(sum/size):0;
}

/** std2(): find the std_deviation across our current_range
*/
double sonar_position::std2(const std::vector<double> vec, const double mean) {
    double sum = 0;
    unsigned int size = 0;
    for(typename std::vector<double>::const_iterator it = vec.begin(); it != vec.end(); ++it) {
        sum+=std::pow(((*it) - mean),2);
         ++size;
    }
    // if size is not == 0, divide by size, else return 0            
    return size?std::sqrt(sum/size):-1;
    
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

    ROS_INFO("sonar_positioning: Shutting down ");

}