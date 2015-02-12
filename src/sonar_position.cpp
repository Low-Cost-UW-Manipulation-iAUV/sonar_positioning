
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
    pool_bl_yaw = 0;
    old_yaw = 0;
    pool_bl_pitch = 0;
    pool_bl_roll = 0;
    last_distance = 0;
    bvm_data_setup = false;
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

// Find the yaw angle in the pool frame
    pool_base_link_timer = nh_.createTimer(transform_search_update_rate, &sonar_position::tf_pool_sonar, this);


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

/** get_ENU_beam_targets(): Get the initial heading (ENU) we want the sonar to look at.
        THis might be in reference to 
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
        ROS_ERROR("Sonar Position - %s: Could not find axis, assuming x", sonar_name_.c_str());
        axis = "x";
        nh_.setParam(param_address.str(), axis);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/beam_target";
    if (!nh_.getParamCached(param_address.str(), beam_target)) {

        ROS_ERROR("Sonar Position - %s: Could not find beam_target assuming 0->4Deg", sonar_name_.c_str());
        
        beam_target[0] = STD_ANGLE_LEFT ; // 4°
        beam_target[1] = STD_ANGLE_RIGHT ; // 0°
        nh_.setParam(param_address.str(), beam_target);
    } 
    // we want the local beam target in rad
    beam_target[0] = beam_target[0] * M_PI / 180;
    beam_target[1] = beam_target[1] * M_PI / 180;
    ROS_INFO("beam_targets - %s, [%f, %f]",axis.c_str(), beam_target[0], beam_target[1] );
  

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/track_wall/update_rate";
    if (!nh_.getParamCached(param_address.str(), update_rate)) {
        ROS_ERROR("Sonar Position - %s: Could not find update_rate, assuming 1 Hz", sonar_name_.c_str());
        update_rate = 1;
        nh_.setParam(param_address.str(), update_rate);        
    }

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
    param_address << "/sonar/" << sonar_name_ << "/processing_params/consecutive_bin_value_threshold";
    if (!nh_.getParamCached(param_address.str(), consecutive_bin_value_threshold)) {

        ROS_ERROR("Sonar Position - %s: Could not find consecutive_bin_value_threshold, assuming 100.", sonar_name_.c_str());
        consecutive_bin_value_threshold = 100;
        nh_.setParam(param_address.str(), consecutive_bin_value_threshold);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/wall_threshold";
    if (!nh_.getParamCached(param_address.str(), wall_threshold)) {

        ROS_ERROR("Sonar Position - %s: Could not find threshold, assuming 3.", sonar_name_.c_str());
        wall_threshold = 3;
        nh_.setParam(param_address.str(), wall_threshold);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/valley_limit";
    if (!nh_.getParamCached(param_address.str(), valley_limit)) {

        ROS_ERROR("Sonar Position - %s: Could not find valley_limit, assuming 10.", sonar_name_.c_str());
        valley_limit = 10;
        nh_.setParam(param_address.str(), valley_limit);
    }

    param_address.clear();
    param_address.str("");
    param_address << "/sonar/" << sonar_name_ << "/processing_params/mountain_minimum";
    if (!nh_.getParamCached(param_address.str(), mountain_minimum)) {

        ROS_ERROR("Sonar Position - %s: Could not find threshold, assuming 70.", sonar_name_.c_str());
        mountain_minimum = 70;
        nh_.setParam(param_address.str(), mountain_minimum);
    }
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

/** send_limits_sonar():
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


/** tf_pool_sonar(): gets the current heading of the sub in reference to the pool
*/
void sonar_position::tf_pool_sonar(const ros::TimerEvent& event ) {
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
    ROS_INFO("sonar_position: yaw of pool->base_link : %f", pool_bl_yaw);

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
                            heading has changed more than heading_threshold
*/
void sonar_position::timed_wall_tracking(const ros::TimerEvent & event) {
    if(sonar_configured == false) {
        track_wall();
        sonar_configured = true;
    }
    if (fabs(pool_bl_yaw - old_yaw) >= heading_threshold )  { //
        old_yaw = pool_bl_yaw;
        track_wall();
    }

}


/** track_wall(): Find the angle between beam_target and yaw (both in pool frame) and set the sonar head to it
*/
int sonar_position::track_wall(void) {

    // Update the beam target
    get_ENU_beam_targets();
   double left, right;
   

    left  =  wrapRad(beam_target[0] - pool_bl_yaw);
    right =  wrapRad(beam_target[1] - pool_bl_yaw);


    if (send_limits_sonar(left, right) == EXIT_SUCCESS) {
        return EXIT_SUCCESS;
    } else {
        return EXIT_FAILURE;
    }
}


/** process_sonar(): Does the actual sonar processing
*/
int sonar_position::process_sonar(const std_msgs::Int32MultiArray::ConstPtr& message) {
    
    // get the angle in rad from the sonar steps
    double angle_rad = steps2rad(message->data[0]);

    // find the wall and the  distance (hypotenuse) to it
    double d_hypot = 0;
    if( squared_rolling(message, &d_hypot) == EXIT_FAILURE) {
        return EXIT_FAILURE;
    }    
    ROS_INFO("sonar_position %s -  %s - angle: %3f, d_hypot: %f",sonar_name_.c_str(), axis.c_str() , angle_rad/DEG2RAD, d_hypot);
    
    // Due to the pool frame attachment our position can only be negative.
    d_hypot = -1 * d_hypot; 

    // get the shortest distance from the hypothenuse.     
    double adjascent = hyp2ad(d_hypot, angle_rad);

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

double sonar_position::hyp2ad(double hypothenuse, double sonar_head_angle) {
    double xy_angle;
    if (axis == "y") {
        return sin(wrapRad(sonar_head_angle) + wrapRad(pool_bl_yaw) ) * hypothenuse;
    } else {
        return cos(wrapRad(sonar_head_angle) + wrapRad(pool_bl_yaw) ) * hypothenuse;
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
            ROS_ERROR("Sonar Positioning - %s: Strongest_bin was broken. - returning last_distance", sonar_name_.c_str());

            return EXIT_FAILURE;
        }
        
    } else {
        ROS_ERROR("Sonar_Positioning - %s: Message did not contain proper info. - returning last_distance", sonar_name_.c_str());
        //ROS_ERROR("debug info: data.size(): %lu, numBins: %d, range: %f", message->data.size(), numBins, range );
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
    sonar::sonar_position orange_box_0(nh, "sonar_positioning_UWE");
    sonar::sonar_position orange_box_1(nh, "sonar_positioning_BMT");

    // register the 
    ros::spin();

    ROS_INFO("sonar_positioning: Shutting down ");

}
