
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
    sonar_configured = false;
    bvm_data_setup = false;

// Prep the processing parameters
    get_processing_parameters();
}

sonar_position::~sonar_position() {

}

void sonar_position::do_subs_pubs(void) {
    sub_imu = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 1, &sonar_position::sub_callback_imu, this );
   // sub_imu = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &sonar_position::sub_callback_imu, this );

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

        ROS_INFO("Sonar Position: Could not find variance of x, will determine it now.");
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

        ROS_INFO("Sonar Position: Could not find variance of y, will determine it now.");
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
    axis = "";
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/axis";
    if (!nh_.getParamCached(param_address.str(), axis)) {
        ROS_ERROR("Sonar Position: Could not find axis, assuming x\n");
        axis = "x";
        nh_.setParam(param_address.str(), axis);        
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/beam_target";
    if (!nh_.getParamCached(param_address.str(), beam_target)) {

        ROS_ERROR("Sonar Position: Could not find beam_target assuming 0->4Deg\n");
        
        beam_target[0] = STD_ANGLE_LEFT ; // 4°
        beam_target[1] = STD_ANGLE_RIGHT ; // 0°
        nh_.setParam(param_address.str(), beam_target);
    } 
    // we want the local beam target in rad
    beam_target[0] = beam_target[0] * M_PI / 180;
    beam_target[1] = beam_target[1] * M_PI / 180;
    ROS_INFO("beam_targets - %s, [%f, %f]",axis.c_str(), beam_target[0], beam_target[1] );

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

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/valley_limit";
    if (!nh_.getParamCached(param_address.str(), valley_limit)) {

        ROS_ERROR("Sonar Position: Could not find valley_limit, assuming 10. \n");
        valley_limit = 10;
        nh_.setParam(param_address.str(), valley_limit);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/mountain_minimum";
    if (!nh_.getParamCached(param_address.str(), mountain_minimum)) {

        ROS_ERROR("Sonar Position: Could not find threshold, assuming 70. \n");
        mountain_minimum = 70;
        nh_.setParam(param_address.str(), mountain_minimum);
    }
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/skip_bins";
    if (!nh_.getParamCached(param_address.str(), skip_bins)) {

        ROS_ERROR("Sonar Position: Could not find skip_bins, assuming 0. \n");
        skip_bins = 0.0;
        nh_.setParam(param_address.str(), skip_bins);
    }

}

/** send_limits_sonar():
*/
int sonar_position::send_limits_sonar(double left_limit, double right_limit) {
    // send the sonar the command to only sweep our required area.
    std_msgs::String sonar_command;    

    std::ostringstream temp;
    temp << "leftlim="<< rad2steps( wrapRad(left_limit) ) <<",rightlim="<< rad2steps( wrapRad(right_limit) )<<"";
    sonar_command.data = temp.str();
    ROS_INFO("sonar_positioning - %s -left_limit: %f, right_limit: %f", axis.c_str(), wrapRad(left_limit), wrapRad(right_limit) );
    pub_sonar_command.publish(sonar_command);
    return EXIT_SUCCESS;
}


/** sub_callback_imu(): store the current attitude solution of the system.
        This data will (for the time being) be from the sensor fusion system.
*/
void sonar_position::sub_callback_imu(const sensor_msgs::Imu::ConstPtr& message ) {//const nav_msgs::Odometry::ConstPtr& message) {

//    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Quaternion q(message->orientation.x, message->orientation.y, message->orientation.z, message->orientation.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    imu_timestamp = message->header.stamp;

    // Publish the odom->SVS transform as good brothers do.

    publish_transform(svs_transform_x, svs_transform_y, svs_transform_z,  yaw,pitch,roll, parent_frame_id,svs_child_frame_id);
    publish_transform(transform_x,transform_y,transform_z, yaw,pitch,roll, parent_frame_id, child_frame_id);

}


/** sub_callback_sonar(): transforms the sonar data and publishes it as a position update for the sensor fusion.
*/
void sonar_position::sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {
    // let the sonar track the wall.
    while (sonar_configured == false) {
        if(track_wall(0.0, 0.0) == EXIT_SUCCESS) {
            sonar_configured = true;
        }

    }

    // THe sonar points now wherever we want it.
    if (process_sonar(message) == EXIT_SUCCESS) {

        if ( (axis == "x" && variance_x_found == true) || (axis == "y" && variance_y_found == true) ){
            
            // Dynamic transform publish which allows for 
            publish_position(axis);
        }
    }
}


/** track_wall(): calculate the correct angle to face the wall and let the sonar face that way
        Maybe we need to limit the number of updates we send here... 
*/
int sonar_position::track_wall(double left_subtrahend, double right_subtrahend) {
    
    // Update the beam target
    get_ENU_beam_targets();
    double left = beam_target[0] - left_subtrahend;
    double right = beam_target[1] - right_subtrahend;
    if (send_limits_sonar(left, right) == EXIT_SUCCESS) {
        return EXIT_SUCCESS;
    } else {
        return EXIT_FAILURE;
    }
}



/** process_sonar(): Does the actual sonar processing
*/
int sonar_position::process_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {
    double angle_rad = steps2rad(message->data[0]);

    // if our beam_target is not crossing 0
    if(beam_target[0] >= beam_target[1]) {
        if(angle_rad >= beam_target[1] && angle_rad <= beam_target[0]) {// we are withing the beam targets which do not wrap
        } else {
            ROS_ERROR("sonar_position - %s - sonar scans at %f outside of demanded area (continous area)", axis.c_str(), angle_rad*180/M_PI);
            return EXIT_FAILURE;
        }
    } else {
        if((angle_rad >= beam_target[1] && angle_rad <= 2*M_PI) || (angle_rad <= beam_target[0] && angle_rad >= 0) ) { //as above but crossing 360/0 discontinuity
        } else {
            ROS_ERROR("sonar_position - %s - sonar scans at %f outside of demanded area (discontinous area)", axis.c_str(), angle_rad*180/M_PI);
            return EXIT_FAILURE;
        }
    }

    // Find the wall in the noisy data
/* 
   double d_hypot = 0;
    if( sonar2Distance(message, &d_hypot) == EXIT_FAILURE) {
        return EXIT_FAILURE;
    }
*/
    double d_hypot = 0;
    if( blurred_valleys_mountains(message, &d_hypot) == EXIT_FAILURE) {
        return EXIT_FAILURE;
    }    

    ROS_INFO("sonar_position - %s - angle: %3f, d_hypot: %f", axis.c_str() , angle_rad/DEG2RAD, d_hypot);

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
double sonar_position::sonar2Distance(const std_msgs::Int32MultiArray::ConstPtr& message, double *d_hypot) {
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
        for (int x=3+skip_bins; x < (numBins+3); x++) {

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
            *d_hypot = last_distance;
            return EXIT_SUCCESS;
        } else {
            ROS_ERROR("Strongest_bin was broken. - returning last_distance");

            return EXIT_FAILURE;
        }
        
    } else {
        ROS_ERROR("Message did not contain proper info. - returning last_distance");
        //ROS_ERROR("debug info: data.size(): %lu, numBins: %d, range: %f", message->data.size(), numBins, range );
        return EXIT_FAILURE;
    }
}

int sonar_position::blurred_valleys_mountains(const std_msgs::Int32MultiArray::ConstPtr& message, double * d_hypot) {
    get_processing_parameters();

    int numBins = message->data[1];
    // the maximum distance to be found in dM
    double range = message->data[2]/10;
    // ensure we are staying within the message and that neither of these numbers are NaN
    if(( message->data.size() >= (numBins + 3) ) && (isnan(numBins) == false)  && (isnan(range) == false) && (numBins >= 10) ) {
        std::vector<double> temp;

        if(bvm_data_setup == false) {
            temp.resize(numBins,0.1);
            bvm_data.clear();
            bvm_data.resize(3);
            bvm_data_setup = true;
            bvm_data.at(0) = temp;
            bvm_data.at(1) = temp;
            bvm_data.at(2) = temp;
        }
        // store the data in the circular buffer
        std::vector<double> blurred;
        temp.assign(message->data.begin()+3, message->data.end());
        bvm_data.push_back(temp);
        // Convolution with a kernel = ones(3), kernel(2,2) = 0;
        blurred.reserve(numBins);
        // assign the first value,  convolution cant do this
        blurred.push_back(bvm_data[1][0] );

        for (int x = 1; x < numBins-1; x++) {

            blurred.push_back(bvm_data[1][x] + (bvm_data[0][x] + bvm_data[2][x] + bvm_data[0][x-1] + bvm_data[1][x-1] + bvm_data[2][x-1] + bvm_data[0][x+1] + bvm_data[1][x+1] + bvm_data[2][x+2])/8 );
        }
        // assign the first value,  convolution cant do this
        blurred.push_back(*bvm_data[2].end() );

        // Go through all datapoints after the crappy sensor area
        for (std::vector<double>::iterator itx = blurred.begin()+skip_bins; itx != (blurred.end()-20); ++itx) {
           
            // is the valley low enough
            if ( mean(blurred, itx, itx+10) <= valley_limit) {
           
                // is the mountain high enough find the peak                
                if (mean(blurred,itx+10, itx+20) >= mountain_minimum) {

                    std::vector<double>::iterator it = std::max_element(itx+10, itx+20);
                    
                    // and return the peaks position
                    *d_hypot = ( (double)(it - blurred.begin()) ) * (range / (double) numBins);
                    
                    return EXIT_SUCCESS;
                } 
            }
        }
        ROS_ERROR("sonar_position - %s - BVM Could not find wall", axis.c_str());
        return EXIT_FAILURE;
    }
    ROS_ERROR("sonar_position - %s - data broken", axis.c_str());

    return EXIT_FAILURE;
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

/** publish_transform: rotates the sonar_beam frame to the sonar_base frame
        publishes the transformation that will rotate the x,y position in the local sonar frame to the sonar base frame.
*/
void sonar_position::publish_transform(double x, double y, double z, double yaw, double pitch, double roll, std::string parent_frame_id, std::string child_frame_id) {
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

/** mean(): calculates the mean
*/
double sonar_position::mean(const std::vector<double> vec, typename std::vector<double>::const_iterator start, typename std::vector<double>::const_iterator finish) {
    double sum = 0;
    unsigned int size = 0;
    for (typename std::vector<double>::const_iterator it = start; it!=finish; ++it, ++size) 
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
