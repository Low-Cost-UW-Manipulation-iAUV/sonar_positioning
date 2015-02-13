
#include "sonar_positioning/sonar_position.hpp"


#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include <math.h>
#include <sstream>
#include <algorithm> 
#include <functional> 
#include "micron_driver/sonarRequest.h"

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


// Prep variables    
    old_yaw = 0;
    pool_sh_yaw = 0;    
    pool_sh_pitch = 0;
    pool_sh_roll = 0;
    last_distance = 0;
    update_rate = 1;
    sonar_configured = false;
    squared_rolling_setup = false;

// Setup the publications and subscription
    do_subs_pubs();

// Prepare the sonar calibration
    get_sonar_calibration_data();

// get the transform search update_rate
    get_transform_search_update_rate();

// Get frame_id from parameter server
    get_frame_id();

// Get the (initial) target headings for the sonar
    get_ENU_beam_targets();

// Prep the processing parameters
    get_processing_parameters();

// Find the sonar head heading in the pool frame
    pool_sonar_head_timer = nh_.createTimer(transform_search_update_rate, &sonar_position::tf_pool_sonar_head, this);


// Track the wall at the specified 
    if(update_rate != 0) {
        ros::Duration update_freq = ros::Duration(1.0/update_rate);
        timer_update = nh_.createTimer(update_freq, &sonar_position::timed_wall_tracking, this);
    }
    ROS_INFO("sonar_position - %s: Init finished", sonar_name_.c_str());
}

sonar_position::~sonar_position() {

}


void sonar_position::do_subs_pubs(void) {

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
    sonar_commands_server.clear();
    param_address << "/sonar/" << sonar_name_ << "/sonar_commands_server" ;

    if (!nh_.getParam(param_address.str(), sonar_commands_server)) {

        ROS_ERROR("Sonar Position: cant find which ServiceServer to send commands to,error \n");
        ros::shutdown();
    } 
    sonar_command_service_client = nh_.serviceClient<micron_driver::sonarRequest>(sonar_commands_server);
    
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

        ROS_INFO("Sonar Position - %s: Could not find variance of x, will determine it now.", sonar_name_.c_str());
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

        ROS_INFO("Sonar Position - %s: Could not find variance of y, will determine it now.", sonar_name_.c_str());
        variance_y = 0;
        variance_y_found = false;
    } else { // if it is available skip the determination
        variance_y_found = true;
    }

}

/** get_frame_id(): 
*/
void sonar_position::get_frame_id(void) {

    std::ostringstream param_address;
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/child_frame";  
    if (!nh_.getParam(param_address.str(), child_frame_id)) {

        ROS_ERROR("Sonar Position - %s: couldn't find child_frame_id, exiting", sonar_name_.c_str());
        ros::shutdown();
    }  
}

/** transform_search_update_rate(): 
*/
void sonar_position::get_transform_search_update_rate(void) {

    if (!nh_.getParam("/sonar/transform_search_update_rate", transform_search_update_rate)) {

        ROS_ERROR("Sonar Position - %s: couldn't find transform_search_update_rate, exiting", sonar_name_.c_str());
        ros::shutdown();
    }  
}

/** get_ENU_beam_targets(): Get the direction we want the sonar to look towards in the pool frame
*/
void sonar_position::get_ENU_beam_targets(void) {
 
    // Which axis is this sonar measuring?  
    std::ostringstream param_address;
    axis = "";
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/axis";
    if (!nh_.getParamCached(param_address.str(), axis)) {
        ROS_ERROR("Sonar Position - %s: Could not find axis, assuming x", sonar_name_.c_str());
        axis = "x";
        nh_.setParam(param_address.str(), axis);
    }

    // what direction in the pool frame to we want to look at? What are the beamwidth limits?
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/beam_target";
    std::vector<double> temp_vector;
    if (!nh_.getParamCached(param_address.str(), temp_vector)) {

        ROS_ERROR("Sonar Position - %s: Could not find beam_target, exiting", sonar_name_.c_str());
        ros::shutdown();
    } 
    // beam_width targets
    beam_target.at(0) = temp_vector.at(0) * M_PI / 180;
    beam_target.at(1) = temp_vector.at(1) * M_PI / 180;
    // direction of the wall
    beam_goal = temp_vector.at(2) * M_PI / 180;

    // how often do we want to update the sonar head heading
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/update_rate";
    if (!nh_.getParamCached(param_address.str(), update_rate)) {
        ROS_ERROR("Sonar Position - %s: Could not find update_rate, assuming 1 Hz", sonar_name_.c_str());
        update_rate = 1;
        nh_.setParam(param_address.str(), update_rate);        
    }

    // how far is it allowed to deviate from the set course?
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/heading_threshold";
    double temp;
    if (!nh_.getParamCached(param_address.str(), temp)) {
        ROS_ERROR("Sonar Position - %s: Could not find heading_threshold, assuming 10°", sonar_name_.c_str());
        heading_threshold = 10;
        nh_.setParam(param_address.str(), temp);
    }
            heading_threshold = temp * M_PI/180;
}


void sonar_position::get_processing_parameters(void) {

    std::ostringstream param_address;

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/skip_bins";
    if (!nh_.getParamCached(param_address.str(), skip_bins)) {

        ROS_ERROR("Sonar Position - %s: Could not find skip_bins, assuming 0.", sonar_name_.c_str());
        skip_bins = 0.0;
        nh_.setParam(param_address.str(), skip_bins);
    }
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/max_bins";
    if (!nh_.getParamCached(param_address.str(), max_bins)) {

        ROS_ERROR("Sonar Position - %s: Could not find max_bins, assuming 210.", sonar_name_.c_str());
        max_bins = 210;
        nh_.setParam(param_address.str(), max_bins);
    }
    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/threshold";
    if (!nh_.getParamCached(param_address.str(), threshold)) {

        ROS_ERROR("Sonar Position - %s: Could not find threshold, assuming 100000.", sonar_name_.c_str());
        threshold = 100000;
        nh_.setParam(param_address.str(), threshold);
    }
}

/** send_limits_sonar(): produce a nice string of the limits we want to send and call the ros service running in the sonar driver node
*/
int sonar_position::send_limits_sonar(double left_limit, double right_limit) {
    // send the sonar the command to only sweep our required area.
    micron_driver::sonarRequest sonar_command;    

    std::ostringstream temp;
    temp << "leftlim="<< rad2steps( wrapRad(left_limit) ) << ",rightlim=" << rad2steps( wrapRad(right_limit) )<<"";
    sonar_command.request.request = temp.str();
    ROS_INFO("sonar_positioning - %s -left_limit: %f, right_limit: %f", axis.c_str(), wrapRad(left_limit), wrapRad(right_limit) );
    
    ROS_INFO("sonar_position - %s: waiting for sonarRequest Service to come online", sonar_name_.c_str());
    ros::service::waitForService(sonar_commands_server, -1);

    if(sonar_command_service_client.call(sonar_command)) {
        return EXIT_SUCCESS;
    } else {
        return EXIT_FAILURE;
    }
}

/** tf_pool_sonar_head(): gets the current heading of the sonarhead in reference to the pool
                        Timer based callback
*/
void sonar_position::tf_pool_sonar_head(const ros::TimerEvent& event ) {
    tf::StampedTransform found_transform;

    std::ostringstream sonar_name_head_string;
    sonar_name_head_string.clear();
    sonar_name_head_string.str("");
    sonar_name_head_string <<  sonar_name_ << "_head";
    try{
      listener.lookupTransform("/pool", sonar_name_head_string.str(), ros::Time(0), found_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // get the rotation from pool to base_link
    tf::Quaternion quat = found_transform.getRotation();
    tf::Matrix3x3 m(quat);
    m.getRPY(pool_sh_roll, pool_sh_pitch, pool_sh_yaw);

}


/** sub_callback_sonar(): transforms the sonar data and publishes it as a position update for the sensor fusion.
*/
void sonar_position::sub_callback_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {
    if (process_sonar(message) == EXIT_SUCCESS) {

        if ( (axis == "x" && variance_x_found == true) || (axis == "y" && variance_y_found == true) ){
            
            // Dynamic transform publish which allows for 
            publish_position(axis);
        }
    }
}

/** timed_wall_tracking(): Decides if we want to update the sonar head direction
                            Runs every x seconds and updates the direction if the 
                            sonar head is outside of our allowed zone around beam_goal
*/
void sonar_position::timed_wall_tracking(const ros::TimerEvent & event) {
    if(sonar_configured == false) {
        track_wall();
        sonar_configured = true;
    }
    // of we are outside of our allowed zone around beam_goal
    if ( fabs(wrapRad(beam_goal) - wrapRad(pool_sh_yaw) ) > heading_threshold) {

        // double check we haven't set the beam_width bigger than our allowed range
        if ( (wrapRad(beam_target.at(0) - wrapRad(beam_goal)) > heading_threshold) || ((wrapRad(beam_goal) - wrapRad(beam_target.at(1))) > heading_threshold) ){
            ROS_ERROR("sonar_positioning - %s: the requested sonar beam does not fit into your heading threshold", sonar_name_.c_str());
        } else {
            ROS_INFO("sonar_positioning - %s: Updating the sonar head heading", sonar_name_.c_str());
            track_wall();
        }

    }
}


/** track_wall(): Find the angle between beam_target and yaw (both in pool frame) and set the sonar head to it
*/
int sonar_position::track_wall(void) {

    // Update the beam target
    get_ENU_beam_targets();
    double left, right;
   
    // Find the sonar_mount heading in the pool frame
    double sonar_mount_heading_pool_frame = get_sonar_mount_heading_pool();

    left  =  wrapRad(wrapRad(beam_target.at(0)) - wrapRad(sonar_mount_heading_pool_frame));
    right =  wrapRad(wrapRad(beam_target.at(1)) - wrapRad(sonar_mount_heading_pool_frame));

    ROS_INFO("%s: left : %f = %f - %f", sonar_name_.c_str(), left , beam_target.at(0), sonar_mount_heading_pool_frame);
    if (send_limits_sonar(left, right) == EXIT_SUCCESS) {
        return EXIT_SUCCESS;
    } else {
        return EXIT_FAILURE;
    }
}


/** get_sonar_mount_heading_pool(): Finds the heading of the sonar mount in the pool frame
*/
double sonar_position::get_sonar_mount_heading_pool(void) {
    tf::StampedTransform found_transform;

    std::ostringstream sonar_name_head_string;
    sonar_name_head_string.clear();
    sonar_name_head_string.str("");
    sonar_name_head_string <<  sonar_name_ << "_mount";
    try{
      listener.lookupTransform("/pool", sonar_name_head_string.str(), ros::Time(0), found_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // get the rotation from pool to base_link
    tf::Quaternion quat = found_transform.getRotation();
    tf::Matrix3x3 m(quat);
    double rubbish;
    double angle;
    m.getRPY(rubbish, rubbish, angle);
    return angle;

}
/** broadcast_sonar_head(): Broadcast the sonarhead position in the sonar mount frame
*/
void sonar_position::broadcast_sonar_head(double angle_rad) {
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0,0.0,0.0) );

    tf::Quaternion q;
    q.setRPY(0.0,0.0,angle_rad);

    transform.setRotation(q);
    std::ostringstream sonar_name_mount_string;
    sonar_name_mount_string.clear();
    sonar_name_mount_string.str("");
    sonar_name_mount_string <<  sonar_name_ << "_mount";

    std::ostringstream sonar_name_head_string;
    sonar_name_head_string.clear();
    sonar_name_head_string.str("");
    sonar_name_head_string <<  sonar_name_ << "_head";

    br_sonar_mount_head.sendTransform( tf::StampedTransform(transform, ros::Time::now(), sonar_name_mount_string.str(), sonar_name_head_string.str() ) );

}


/** process_sonar(): Does the actual sonar processing
*/
int sonar_position::process_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {
    
    // get the angle in rad from the sonar steps
    double angle_rad = steps2rad(message->data[0]);
    broadcast_sonar_head(angle_rad);
    // find the wall and the  distance (hypotenuse) to it
    double d_hypot = 0;
    if( squared_rolling(message, &d_hypot) == EXIT_FAILURE) {
        return EXIT_FAILURE;
    }    
    ROS_INFO("sonar_position %s -  %s - angle: %3f, d_hypot: %f",sonar_name_.c_str(), axis.c_str() , angle_rad/DEG2RAD, d_hypot);
    
    // Due to the pool frame attachment our position can only be negative.
    d_hypot = -1 * d_hypot; 

    // get the shortest distance from the hypothenuse.     
    double adjascent = hyp2ad(d_hypot);

    if(axis == "x") {

        if(variance_x_found == true) {
            position = adjascent;
            angle = angle_rad;
            return EXIT_SUCCESS;
        } else { // we haven't got any variance data.

            if(x_variance_data.size() >= calibration_length) {
                ROS_INFO("sonar_position - %s: Determining Sonar variance on x - Sample %lu of %d",sonar_name_.c_str(), x_variance_data.size(), calibration_length);

                variance_x = pow( std2(x_variance_data, mean(x_variance_data) ), 2);
                if(store_variance(variance_x, axis) == EXIT_SUCCESS) {
                    variance_x_found = true;
                }
                return EXIT_SUCCESS;
            }
            x_variance_data.push_back(adjascent);
            ROS_INFO("sonar_position - %s: Determining Sonar variance on x - Sample %lu of %d", sonar_name_.c_str(), x_variance_data.size(), calibration_length);
            return EXIT_FAILURE;
        }

    } else if(axis == "y") {
       if(variance_y_found == true) {
            position = adjascent;
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
            y_variance_data.push_back(adjascent);
            ROS_INFO("sonar_position - %s: Determining Sonar variance on y - Sample %lu of %d", sonar_name_.c_str(), y_variance_data.size(), calibration_length);
            return EXIT_FAILURE;            
        }

    }
}

double sonar_position::hyp2ad(double hypothenuse) {
    
    return cos(wrapRad(beam_goal) - wrapRad(pool_sh_yaw)) * hypothenuse;
}

int sonar_position::store_variance(double variance, std::string variable_name) {
    std::ostringstream param_address;

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/variance/" << variable_name;
    nh_.setParam(param_address.str(), variance);

    double temp = 0;
    if (!nh_.getParam(param_address.str(), temp)) {
        ROS_ERROR("Sonar Position - %s: Could not store variance...", sonar_name_.c_str());
        return EXIT_FAILURE;
    } else if (temp == variance) {
        ROS_INFO("sonar_position - %s: Variance of %s is %f, stored on param server", sonar_name_.c_str(), variable_name.c_str(), variance);
        return EXIT_SUCCESS;
    } else{
        ROS_ERROR("Sonar Position - %s: Could not store variance, did not match when checking", sonar_name_.c_str());
        return EXIT_FAILURE;
    }
}




int sonar_position::squared_rolling(const std_msgs::Int32MultiArray::ConstPtr& message, double * d_hypot) {
    get_processing_parameters();

    int numBins = message->data[1];
    // the maximum distance to be found in dM
    double range = message->data[2]/10;
    

    // ensure we are staying within the message and that neither of these numbers are NaN
    if(( message->data.size() >= (numBins + 3) ) && (isnan(numBins) == false)  && (isnan(range) == false) && (numBins >= 10) ) {
        
        // initialise the empty sum vector and put it into the summer to initialize that, too
        if(squared_rolling_setup == false) {
            sum.resize(numBins,0.0);
            
            sonar_summer.resize(10);
            //  fill the buffer
            for (int x = 0; x<10; x++) {
                sonar_summer.push_front(sum);                
            }
            squared_rolling_setup = true;
        }
        std::vector<double> temp;
        //  get the data from the message
        temp.assign(message->data.begin()+3, message->data.end());

        // square it
        std::transform(temp.begin(), temp.end(), temp.begin(), temp.begin(), std::multiplies<double>());

        // remove the last item from our sum
        std::transform(sum.begin(), sum.end(), sonar_summer.back().begin(), sum.begin(), std::minus<double>());

        // add the new item to the buffer
        sonar_summer.push_front(temp);

        // add the newest item to the sum
        std::transform(sum.begin(), sum.end(), sonar_summer.front().begin(), sum.begin(), std::plus<double>());

        // go through array starting after the skipped bins and finishing before the max_bins
        for(std::vector<double>::iterator itx = sum.begin()+skip_bins; itx != sum.begin()+max_bins; ++itx ) {
            // if the bin is above the threshold calculate the distance and return
            if(*itx >= threshold) {
                *d_hypot = ( (double)(itx - sum.begin()) ) * (range / (double) numBins);

                return EXIT_SUCCESS;
            }        
        }
        ROS_ERROR("sonar_position - %s - %s - BVM Could not find wall", sonar_name_.c_str(), axis.c_str());
        return EXIT_FAILURE;
    }
    ROS_ERROR("sonar_position - %s - %s - data broken", sonar_name_.c_str(), axis.c_str());

    return EXIT_FAILURE;
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
        sonar_position.pose.covariance[7] = variance_y;
    } else {
        ROS_ERROR("sonar_position - %s: no axis found...", sonar_name_.c_str());
    }


    pub_position.publish(sonar_position);
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
 
    ros::AsyncSpinner spinner(8);
    spinner.start();
    // create the instance of the class
    sonar::sonar_position orange_box_0(nh, "sonarUWE");
    sonar::sonar_position orange_box_1(nh, "sonarBMT");

    // register the 
    ros::spin();

    ROS_INFO("sonar_positioning: Shutting down ");

}
