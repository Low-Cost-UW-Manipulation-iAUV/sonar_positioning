
#include "sonar_positioning/sonar_position.hpp"


#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include <math.h>
#include <sstream>

#define SURGE 1
#define SWAY 2

#define STEPSPERROTATION 6400     // The sonar outputs stepper position in 1/16 gradians or gon (400 gon /rotation).
                                    // Thus a full rotation is 6400 steps.

#define DEGREESPERROTATION 360
#define STD_ANGLE_PLUS 0.1745329252  //10°
#define STD_ANGLE_MINUS 6.108652382  //350°
#define DEG2RAD (M_PI/180)
namespace sonar {

sonar_position::sonar_position(ros::NodeHandle nh) {
    nh_ = nh;
    sub_imu = nh_.subscribe<nav_msgs::Odometry>("/imu/data", 1, &sonar_position::sub_callback_imu, this);
    sub_sonar = nh_.subscribe<std_msgs::Int32MultiArray>("/sonarData", 1, &sonar_position::sub_callback_sonar, this);

    pub_position_x = nh_.advertise<nav_msgs::Odometry>("/sonar/position/x", 100);
    pub_position_y = nh_.advertise<nav_msgs::Odometry>("/sonar/position/y", 100);
    pub_sonar_command = nh_.advertise<std_msgs::String>("/sonarRequest", 5);
    not_yet_configured = true;

    if (!nh_.getParam("/sonar/calibration_length", calibration_length)) {

        ROS_ERROR("Sonar Position: Could not find calibration_length, assuming 1000. \n");
        calibration_length = 200;
        nh_.setParam("/sonar/calibration_length", calibration_length);
    }  

    if (!nh_.getParamCached("/sonar/samples_per_direction", samples_per_direction)) {

        ROS_ERROR("Sonar Position: Could not find samples_per_direction, assuming 5. \n");
        samples_per_direction = 20;
        nh_.setParam("/sonar/samples_per_direction", samples_per_direction);
    }

    beam_width_x.clear();
    beam_width_x.resize(2,0);

    if (!nh_.getParamCached("/sonar/beam_width/x", beam_width_x)) {

        ROS_ERROR("Sonar Position: Could not find beam_width_x, assuming +-10Deg either side of the x axis. \n");
        
        beam_width_x[0] = STD_ANGLE_PLUS ; // = +10deg = 3022.2 steps -  left side of the range we want to consider
        beam_width_x[1] = STD_ANGLE_MINUS; // = 350° = 3377.7 steps -  right side of what we want to consider
        nh_.setParam("/sonar/samples_per_direction", beam_width_x);
    } 
    
    beam_width_y.clear();
    beam_width_y.resize(2,0);
    if (!nh_.getParamCached("/sonar/beam_width/y", beam_width_y)) {

        ROS_ERROR("Sonar Position: Could not find beam_width_y, assuming +-10Deg either side of the y axis. \n");
        
        beam_width_y[0] = (M_PI/2 + STD_ANGLE_PLUS); // = 100° = 1777.7 steps -  left side of the range we want to consider
        beam_width_y[1] = (M_PI/2 - STD_ANGLE_PLUS); // = 80° = 1422.2 steps -  right side of what we want to consider
        nh_.setParam("/sonar/samples_per_direction", beam_width_y);
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

    last_distance = 0;
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

/**configure_sonar():
*/
void sonar_position::configure_sonar(void) {
    // send the sonar the command to only sweep our required area.
    std_msgs::String sonar_command;    

    std::ostringstream temp;
    temp << "leftlim="<< rad2steps(beam_width_y[0]) <<",rightlim="<< rad2steps(beam_width_x[1]) <<"";
    sonar_command.data = temp.str();
    ROS_INFO("beam_width_y[0]: %f, beam_width_x[1]: %f", beam_width_y[0], beam_width_x[1]);
    ROS_INFO(sonar_command.data.c_str());
    pub_sonar_command.publish(sonar_command);  
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
    double angle_rad = steps2rad(message->data[0]);


    // extract the imortant data from the sonar data
    float d_hypot = sonar2Distance(message);
    //ROS_INFO("hypo distance is %f", d_hypot);
    ROS_INFO("angle_deg: %f, steps: %d, hypot_dist: %f", angle_rad/DEG2RAD, message->data[0], d_hypot);

    if(not_yet_configured==true) {
        configure_sonar();
        not_yet_configured = false;
    }


    // if we are looking ahead (along x). i.e.
    //                  <= 10°             &&                >= 0°  OR                  <= 360°                       >=350°
    if( (angle_rad <= beam_width_x[0] && angle_rad >= 0 ) ||  (angle_rad <= (2*M_PI) && angle_rad >= beam_width_x[1] ) ) {
        double odom_dist = getOdomDistance(d_hypot, (yaw + angle_rad), pitch);

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
            if(x_position_counter >= samples_per_direction) {
                // calculate the mean of the data.
                x_position = x_position_sum / (double) x_position_counter;  

                // publish it              
                publish_position_x();
                x_position_counter = 1;
                x_position_sum = odom_dist;
            }
        } else { // we haven't got any variance data.
            if(x_variance_data.size() < calibration_length) {
                x_variance_data.push_back(odom_dist);
                ROS_INFO("sonar_position: Determining Sonar variance on x - Sample %lu of %d",x_variance_data.size(), calibration_length);
            } else {
                variance_x = pow( std2(x_variance_data, mean(x_variance_data) ), 2);
                ROS_INFO("sonar_position: Variance of x is %f, stored on param server", variance_x);
                nh_.setParam("/sonar/variance/x", variance_x);

                variance_x_found = true;
            }

        } // We are looking at the y axis. i.e.
    //                        <= 100°            &&                >= 80°
    } else if (angle_rad <= beam_width_y[0] && angle_rad >= beam_width_y[1] ) {
        // find the adjascent (distance along y in 'odom' frame) if the d_hypot is the hypothenuse.
            // We first look top down: undo the yaw and sonar angle. Here we have to take into consideration that we are offset by +90deg.... And then look at it from the side to counteract the current roll.
        double odom_dist = getOdomDistance(d_hypot, (M_PI/2 -(yaw + angle_rad)), roll);

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
            if(y_position_counter >= samples_per_direction) {
                // calculate the mean of the data.
                y_position = y_position_sum / (double) y_position_counter;  

                // publish it              
                publish_position_y();
                y_position_counter = 1;
                y_position_sum = odom_dist;
            }
        } else { // we haven't got any variance data.
            if(y_variance_data.size() < calibration_length) {
                y_variance_data.push_back(odom_dist);
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
    // the maximum distance to be found in dM
    double range = message->data[2]/10.0;
    // ensure we are staying within the message and that neither of these numbers are NaN
    if(( message->data.size() >= (numBins + 3) ) && (isnan(numBins) == false)  && (isnan(range) == false) && (numBins != 0) ) {

        int max = 0;
        int strongest_bin = 0;
        // find the strongest reflection
        for (int x=3; x < (numBins+3); x++) {

            if (message->data[x] > max) {
                max = message->data[x];
                strongest_bin = (x-2);
            }
        }

        // make sure the stronges reflection is also non NaN
        if(isnan(strongest_bin) == false) {
                    // turn range back into meters from dM and return the strongest reflection distance
            last_distance = (double) strongest_bin * (range / (double) numBins);
            return last_distance;
        } else {
            ROS_ERROR("Message did not contain a proper distance info. - returning last_distance");

            return last_distance;
        }
        
    } else {
        ROS_ERROR("Message did not contain proper info. - returning last_distance");
        ROS_ERROR("debug info: data.size(): %lu, numBins: %d, range: %f",message->data.size(),numBins,range );
        return last_distance;
    }
}


/** getOdomDistance(): find the distance to the assumed vertical wall 
            in the odom frame given the distance to the same wall in body frame.
*/
double sonar_position::getOdomDistance(float body_position, double angle_a, double angle_b) {

    // rotate the body frame sonar measurement along the x axis into the odom frame
    double undo_pitch = fabs(cos(angle_a)) * body_position ;
    double odom_position = fabs(cos(angle_b)) * undo_pitch;


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
    for (typename std::vector<double>::const_iterator it = vec.begin(); it!=vec.end(); ++it, ++size) 
    { 
        sum += (*it);
        // if size is not == 0, divide by size, else return 0
    }

    return (size)?(sum/(double) size):0;
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
    return size?std::sqrt(sum/(double)size):-1;
    
}


/**steps2rad(): conversts the micron steps into rad
*/
double sonar_position::steps2rad(int steps) {
    if (steps <=3200) {
        return (-1 *( (double) steps ) + 3200) * ((2*M_PI)/STEPSPERROTATION);
    } else { // so if (message->data[1] > 3200)
        return(-1*( (double) steps ) + 9600) * ((2*M_PI)/STEPSPERROTATION);
    }
}

/**rad2steps(): converts rad into the micron steps
*/
int sonar_position::rad2steps(double rad) {
    if (rad <=M_PI) {
        return (int) (3200 - (rad * (3200/M_PI) ) );
    } else { // so if (message->data[1] > 3200)
        return (int) (9600 - (rad * (3200/M_PI) ) );
    }
}

/**deg2steps(): converts degrees into the micron steps
*/
int sonar_position::deg2steps(double deg) {
    if (deg <=180) {
        return (int) (3200 - (deg * (STEPSPERROTATION/DEGREESPERROTATION) ) );
    } else { // so if (message->data[1] > 3200)
        return (int) (9600 - (deg * (STEPSPERROTATION/DEGREESPERROTATION) ) );
    }
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
