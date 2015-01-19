
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
#define STD_ANGLE_LEFT 4  
#define STD_ANGLE_RIGHT 0  
#define DEG2RAD (M_PI/180)
namespace sonar {

sonar_position::sonar_position(ros::NodeHandle nh, std::string sonar_name) {
    nh_ = nh;
    sonar_name_ = sonar_name;

// Setup the publications and subscription
    do_subs_pubs();

// Prepare the sonar calibration
    get_sonar_calibration_data();

// Get transform frame_ids from parameter server
    get_transform_parameters();

// Get the (initial) target headings for the sonar
    get_ENU_beam_targets();

// Get the angular offset for the transformation
    get_angular_offset();


// Prep variables    
    yaw = 0;
    pitch = 0;
    roll = 0;
    last_distance = 0;

// Prep the processing parameters
    get_processing_parameters();
}

sonar_position::~sonar_position() {

}

void sonar_position::do_subs_pubs(void) {
    sub_imu = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &sonar_position::sub_callback_imu, this );

    std::string temp_string;
    std::ostringstream param_address;
    
    // Subscribe to the raw Sonar data
    temp_string.clear();
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/listen_to" ;
    if (!nh_.getParam(param_address.str(), temp_string)) {

        ROS_ERROR("Sonar Position: cant find raw data topic to listen to, exiting \n");
        ros::shutdown();
    }
    sub_sonar = nh_.subscribe<std_msgs::Int32MultiArray>(temp_string, 1, &sonar_position::sub_callback_sonar, this);


    // Publish the finished data
    temp_string.clear();
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/publish_data_to" ;

    if (!nh_.getParam(param_address.str(), temp_string)) {

        ROS_ERROR("Sonar Position: cant find which rostopic to publish to,error \n");
        ros::shutdown();
    }
    pub_position = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(temp_string, 10);


    // Publish sonar commands here
    temp_string.clear();
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/publish_commands_to" ;

    if (!nh_.getParam(param_address.str(), temp_string)) {

        ROS_ERROR("Sonar Position: cant find which rostopic to publish commands to,error \n");
        ros::shutdown();
    } 
    pub_sonar_command = nh_.advertise<std_msgs::String>(temp_string, 5);   
}



void sonar_position::get_sonar_calibration_data(void) {
    std::ostringstream param_address;

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/calibration_length";
    if (!nh_.getParam(param_address.str(), calibration_length)) {

        ROS_ERROR("Sonar Position: Could not find calibration_length, assuming 20. \n");
        calibration_length = 20;
        nh_.setParam(param_address.str(), calibration_length);
    }  

    variance_x_found = false;
    variance_y_found = false;

    x_variance_data.clear();
    x_variance_data.reserve(calibration_length);
    y_variance_data.clear();
    y_variance_data.reserve(calibration_length);

    // check if the variance for x is available on the parameter server
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/variance/x";
    if (!nh_.getParam(param_address.str(), variance_x)) {

        ROS_ERROR("Sonar Position: Could not find variance of x, will determine it now.");
        variance_x = 0;
        variance_x_found = false;
    } else { // if it is available skip the determination
        variance_x_found = true;
    }

    // check for y as well.
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/variance/y";
    if (!nh_.getParam(param_address.str(), variance_y)) {

        ROS_ERROR("Sonar Position: Could not find variance of y, will determine it now.");
        variance_y = 0;
        variance_y_found = false;
    } else { // if it is available skip the determination
        variance_y_found = true;
    }
}

/** get_transform_parameters(): 
*/
void sonar_position::get_transform_parameters(void) {
    std::ostringstream param_address;
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/parent_frame";  
    if (!nh_.getParam(param_address.str(), parent_frame_id)) {

        ROS_ERROR("Sonar Position: couldn't find parent_frame_id, assuming odom\n");
        parent_frame_id = "odom";
        nh_.setParam(param_address.str(), parent_frame_id);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/child_frame";  
    if (!nh_.getParam(param_address.str(), child_frame_id)) {

        ROS_ERROR("Sonar Position: couldn't find child_frame_id, assuming sonar_undefined\n");
        child_frame_id = "sonar_undefined";
        nh_.setParam(param_address.str(), child_frame_id);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/position_base_link_frame";
    std::vector<double> temp;
    
    temp.clear();    
    if (!nh_.getParam(param_address.str(), temp)) {

        ROS_ERROR("Sonar Position: couldn't find position_base_link_frame, assuming 0\n");
        temp[0] = 0.0;
        temp[1] = 0.0;
        temp[2] = 0.0;
        nh_.setParam(param_address.str(), temp);
    }
    transform_x = temp[0];
    transform_y = temp[1];
    transform_z = temp[2];


    temp.clear();
    if (!nh_.getParam("/sonar/svs/position_base_link_frame", temp)) {

        ROS_ERROR("Sonar Position: couldn't find SVS position_base_link_frame, assuming 0\n");
        temp[0] = 0.0;
        temp[1] = 0.0;
        temp[2] = 0.0;
        nh_.setParam(param_address.str(), temp);
    }
    svs_transform_x = temp[0];
    svs_transform_y = temp[1];
    svs_transform_z = temp[2];   

    if (!nh_.getParam("/sonar/svs/child_frame_id", svs_child_frame_id)) {

        ROS_ERROR("Sonar Position: couldn't find SVS child_frame_id_id, assuming SVS\n");
        svs_child_frame_id = "SVS";
        nh_.setParam(param_address.str(), svs_child_frame_id);
    }
}

void sonar_position::get_angular_offset(void) {
    std::ostringstream param_address;
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/offset_angle";  
    if (!nh_.getParam(param_address.str(), offset_angle)) {

        ROS_ERROR("Sonar Position: Could not find offset_angle, assuming 0\n");
        offset_angle = 0;
        nh_.setParam(param_address.str(), offset_angle);
    }    
}


/** get_ENU_beam_targets(): Get the initial heading (ENU) we want the sonar to look at.
*/
void sonar_position::get_ENU_beam_targets(void) {
    beam_target.clear();
    beam_target.resize(2,0);
    
    std::ostringstream param_address;
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/beam_target";
    if (!nh_.getParamCached(param_address.str(), beam_target)) {

        ROS_ERROR("Sonar Position: Could not find beam_target assuming 0->4Deg\n");
        
        beam_target[0] = STD_ANGLE_LEFT * M_PI/180; // 4°
        beam_target[1] = STD_ANGLE_RIGHT * M_PI/180; // 0°
        nh_.setParam(param_address.str(), beam_target);
    } else {
        beam_target[0] = beam_target[0] * M_PI / 180;
        beam_target[1] = beam_target[1] * M_PI / 180;
    }
    
    axis = "";
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/axis";
    if (!nh_.getParamCached(param_address.str(), axis)) {
        ROS_ERROR("Sonar Position: Could not find axis, assuming x\n");
        axis = "x";
        nh_.setParam(param_address.str(), axis);        
    }

}


void sonar_position::get_processing_parameters(void) {

    std::ostringstream param_address;
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/consecutive_bin_value_threshold";
    if (!nh_.getParamCached(param_address.str(), consecutive_bin_value_threshold)) {

        ROS_ERROR("Sonar Position: Could not find consecutive_bin_value_threshold, assuming 100. \n");
        consecutive_bin_value_threshold = 100;
        nh_.setParam(param_address.str(), consecutive_bin_value_threshold);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/wall_threshold";
    if (!nh_.getParamCached(param_address.str(), wall_threshold)) {

        ROS_ERROR("Sonar Position: Could not find threshold, assuming 3. \n");
        wall_threshold = 3;
        nh_.setParam(param_address.str(), wall_threshold);
    }
}

/** send_limits_sonar():
*/
void sonar_position::send_limits_sonar(double left_limit, double right_limit) {
    // send the sonar the command to only sweep our required area.
    std_msgs::String sonar_command;    

    std::ostringstream temp;
    temp << "leftlim="<< rad2steps( wrapRad(left_limit) ) <<",rightlim="<< rad2steps( wrapRad(right_limit) )<<"";
    sonar_command.data = temp.str();
    ROS_INFO("sonar_positioning - left_limit: %f, right_limit: %f", wrapRad(left_limit), wrapRad(right_limit) );
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

    // Publish the odom->SVS transform as good brothers do.
    publish_sonar_beam_transform(svs_transform_x, svs_transform_y, svs_transform_z,  yaw,pitch,roll, parent_frame_id,svs_child_frame_id);
}


/** sub_callback_sonar(): transforms the sonar data and publishes it as a position update for the sensor fusion.
*/
void sonar_position::sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {
    // let the sonar track the wall.
    track_wall();

    // THe sonar points now wherever we want it.
    process_sonar(message);

    if ( (axis == "x" && variance_x_found) || (axis == "y" && variance_y_found) ){
        
        // Dynamic transform publish which allows for 
        publish_sonar_beam_transform(transform_x,transform_y,transform_z, yaw,pitch,roll, parent_frame_id, child_frame_id);    
        publish_position(axis);
    }
}


/** track_wall(): calculate the correct angle to face the wall and let the sonar face that way
        Maybe we need to limit the number of updates we send here... 
*/
void sonar_position::track_wall(void) {
    
    // Update the beam target
    get_ENU_beam_targets();
    double left = beam_target[0] - yaw;
    double right = beam_target[1] - yaw;
    send_limits_sonar(left, right);
}



/** process_sonar(): Does the actual sonar processing
*/
int sonar_position::process_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {
    double angle_rad = steps2rad(message->data[0]);

    // Find the wall in the 
    double d_hypot = sonar2Distance(message);
    ROS_INFO("sonar_position - angle: %3f, d_hypot: %f",angle_rad/DEG2RAD, d_hypot);
    // If we are working for the x axis
    if(axis == "x") {

        if(variance_x_found == true) {
            position = d_hypot;
            angle = angle_rad;
            return EXIT_SUCCESS;
        } else { // we haven't got any variance data.

            if(x_variance_data.size() >= calibration_length) {
                variance_x = pow( std2(x_variance_data, mean(x_variance_data) ), 2);
                if(store_variance(variance_x, axis) == EXIT_SUCCESS) {
                    variance_x_found = true;
                }
                return EXIT_SUCCESS;
            }
            x_variance_data.push_back(d_hypot);
            ROS_INFO("sonar_position: Determining Sonar variance on x - Sample %lu of %d", x_variance_data.size(), calibration_length);
        }

    } else if(axis == "y") {
       if(variance_y_found == true) {
            position = d_hypot;
            angle = angle_rad;
            return EXIT_SUCCESS;
        } else { // we haven't got any variance data.

            if(y_variance_data.size() >= calibration_length) {
                variance_y = pow( std2(y_variance_data, mean(y_variance_data) ), 2);
                if(store_variance(variance_y, axis) == EXIT_SUCCESS) {
                    variance_y_found = true;
                }
                return EXIT_SUCCESS;
            }
            y_variance_data.push_back(d_hypot);
            ROS_INFO("sonar_position: Determining Sonar variance on y - Sample %lu of %d", y_variance_data.size(), calibration_length);
        }

    }
}

int sonar_position::store_variance(double variance, std::string variable_name) {
    std::ostringstream param_address;

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/variance/" << variable_name;
    nh_.setParam(param_address.str(), variance);

    double temp = 0;
    if (!nh_.getParam(param_address.str(), temp)) {
        ROS_ERROR("Sonar Position: Could not store variance... \n");
        return EXIT_FAILURE;
    } else if (temp == variance) {
        ROS_INFO("sonar_position: Variance of %s is %f, stored on param server", variable_name.c_str(), variance);
        return EXIT_SUCCESS;
    } else{
        ROS_ERROR("Sonar Position: Could not store variance, did not match when checking");
        return EXIT_FAILURE;
    }
}


/** sonar2Distance(): Find the strongest signal in the sonar array.
*/
double sonar_position::sonar2Distance(const std_msgs::Int32MultiArray::ConstPtr& message) {
    get_processing_parameters();

    // Find the number of data bins
    int numBins = message->data[1];
    // the maximum distance to be found in dM
    double range = message->data[2]/10;
    // ensure we are staying within the message and that neither of these numbers are NaN
    if(( message->data.size() >= (numBins + 3) ) && (isnan(numBins) == false)  && (isnan(range) == false) && (numBins != 0) ) {

        double strongest_bin = 0;
        // find the strongest reflection
        int counter_peaks = 0;
        for (int x=3; x < (numBins+3); x++) {

            // count consecutive values higher than the threshold
            if (message->data[x] >= consecutive_bin_value_threshold) {
                counter_peaks ++;
            } else {
                counter_peaks = 0;
            }

            // find the average value once we've got enough consecutive values
            if (counter_peaks >= wall_threshold) {
                int start = x - counter_peaks + 1;
                strongest_bin = ( ((double)(x + start) ) / 2.0 );
                //and shift it back so the number of bins is correct
                strongest_bin = strongest_bin - 3;
                // make sure we don't read any reflections which arrive later than the first reflection.
                break;
            }
        }

        // make sure the stronges reflection is also non NaN
        if(isnan(strongest_bin) == false) {
                    // turn range back into meters from dM and return the strongest reflection distance
            last_distance = strongest_bin * (range / (double) numBins);
            return last_distance;
        } else {
            ROS_ERROR("Strongest_bin was broken. - returning last_distance");

            return last_distance;
        }
        
    } else {
        ROS_ERROR("Message did not contain proper info. - returning last_distance");
        //ROS_ERROR("debug info: data.size(): %lu, numBins: %d, range: %f", message->data.size(), numBins, range );
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
void sonar_position::publish_position(std::string axis) {

    geometry_msgs::PoseWithCovarianceStamped sonar_position;
    sonar_position.header.stamp = ros::Time::now(); 
    sonar_position.header.frame_id = child_frame_id;
    sonar_position.pose.covariance.fill(0.0);

    if(axis == "x") {
        sonar_position.pose.pose.position.x = position;
        // set the variance data for x
        sonar_position.pose.covariance[0] = variance_x;
    } else if(axis == "y") {
        sonar_position.pose.pose.position.y = position;
        // set the variance data for y
        sonar_position.pose.covariance[1] = variance_y;
    }


    pub_position.publish(sonar_position);
}

/** publish_sonar_beam_transform: rotates the sonar_beam frame to the sonar_base frame
        publishes the transformation that will rotate the x,y position in the local sonar frame to the sonar base frame.
*/
void sonar_position::publish_sonar_beam_transform(double x, double y, double z, double yaw, double pitch, double roll, std::string parent_frame_id, std::string child_frame_id) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
    transformer.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id) );
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

double sonar_position::wrapRad(double angle) {
    angle = fmod(angle,2*M_PI);
    if (angle > M_PI) return -2*M_PI + angle;
    if (angle <= -M_PI) return 2*M_PI + angle;
    return angle;
}
  /**
  * The function wraps any value into the [-180,180> range.
  *
  * \param angle The arbitrary value.
  * \return The wrapped value in the [-pi,pi> interval.
  */
double sonar_position::wrapDeg(double angle) {
    angle = fmod(angle,360.);
    if (angle > 180) return -360. + angle;
    if (angle <= -180) return 360. + angle;
    return angle;
}

}  //end of namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "sonar_positioning");
    ros::NodeHandle nh;
    /// An Async spinner creates another thread which will handle the event of this node being executed.
 
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // create the instance of the class
    sonar::sonar_position orange_box_0(nh, "sonar0");
    sonar::sonar_position orange_box_1(nh, "sonar1");

    // register the 
    ros::spin();

    ROS_INFO("sonar_positioning: Shutting down ");

}
