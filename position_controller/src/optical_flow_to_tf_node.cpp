#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "px_comm/OpticalFlowRad.h"
#include "px_comm/OpticalFlow.h"
#include <sstream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "position_controller/SetCurrentPos.h"

using namespace std;
using namespace std_msgs;
using namespace angles;

namespace position_controller
{

/***
 * OpticalFlowToTF integrates and corrects optical flow data to an odometry pose (TF)
 * author: hrabia
 */
class OpticalFlowToTF
{
public:

  OpticalFlowToTF(ros::NodeHandle& n) :
      handle_(n),last_time_odometry_(ros::Time::now()), last_time_rotation_(ros::Time::now()) ,last_time_external_altitude_(ros::Time::now()) ,
      px4flow_quality_threshold_(0), current_alititude_relative_(0), current_alititude_absolute_(0), private_handle_("~"), current_pitch_pos_integrated_(0), external_altitude_(0.0),
      current_roll_pos_integrated_(0), currentOrientation(0, 0, 0, 1),  external_distance_sensor_(false), external_distance_sensor_interpolation_(true),
      simulation_mode_(false), external_altitude_velocity_(0), external_imu_(false)
{
    
    //init topic subscription and advertisement
    string height_range_topic;
    private_handle_.param<string>("height_range_input_topic", height_range_topic, height_range_topic);

    string flow_input_topic;
    private_handle_.param<string>("flow_input_topic", flow_input_topic, flow_input_topic);

    string flow_rot_input_topic;
    private_handle_.param<string>("flow_rot_input_topic", flow_rot_input_topic, flow_rot_input_topic);

    string odometry_output_topic;
    private_handle_.param<string>("odometry_output_topic", odometry_output_topic, odometry_output_topic);

    string imu_topic;
    private_handle_.param<string>("imu_topic", imu_topic, imu_topic);

    if(flow_rot_input_topic.empty()){
       ROS_ERROR_STREAM("Topic flow_rot_input_topic is not properly configured");
    }

    if(height_range_topic.empty() && flow_input_topic.empty() ){
      ROS_ERROR_STREAM("Topic height_range_input_topic and flow_input_topic are not properly configured");
      throw -1;
    }else{
      ROS_INFO_STREAM("Subscribing to height_range_input_topic: " << height_range_topic);
      ROS_INFO_STREAM("Subscribing to flow_input_topic: " << flow_input_topic);
    }

    private_handle_.param<bool>("simulation_mode", simulation_mode_, simulation_mode_);

    private_handle_.param<bool>("external_distance_sensor", external_distance_sensor_, external_distance_sensor_);

    private_handle_.param<bool>("external_imu", external_imu_, external_imu_);

    private_handle_.param<bool>("external_distance_interpolation", external_distance_sensor_interpolation_, external_distance_sensor_interpolation_);

    private_handle_.param<int>("px4flow_quality_threshold", px4flow_quality_threshold_, px4flow_quality_threshold_);

    odometry_pub_ = handle_.advertise<nav_msgs::Odometry>(odometry_output_topic, 10);

    height_range_subscriber_ = handle_.subscribe(height_range_topic, 10, &OpticalFlowToTF::updateAltidude, this);

    if(external_imu_){
      ROS_INFO_STREAM("Using external IMU for orientation estimation with topic: " << imu_topic);
      imu_subscriber_ = handle_.subscribe(imu_topic, 10, &OpticalFlowToTF::updateIMU, this);
    }else{
      ROS_INFO_STREAM("Using PX4Flow for orientation estimation.");
      flow__rot_subscriber_ = handle_.subscribe(flow_rot_input_topic, 10, &OpticalFlowToTF::updateRotOdometry, this);
    }
    flow_subscriber_ = handle_.subscribe(flow_input_topic, 10, &OpticalFlowToTF::updateOdometry, this);


    servicePos_ = handle_.advertiseService("set_current_pos", &OpticalFlowToTF::setCurrentPos, this);

  }

protected:

  /**
   * Publish current odometry
   *
   * @param timeStamp
   */
  void publishOdometry(const ros::Time& timeStamp)
  {
    //XXX use configuration for frame names
    odometry_msgs_.header.stamp = timeStamp;
    odometry_msgs_.header.seq++;
    odometry_msgs_.header.frame_id = "odom";
    odometry_msgs_.child_frame_id = "base_link";
    odometry_msgs_.pose.pose.position.x = current_pitch_pos_integrated_; //x to the front
    odometry_msgs_.pose.pose.position.y = current_roll_pos_integrated_; // y to the left
    odometry_msgs_.pose.pose.position.z = current_alititude_relative_; //z axis looks up
    odometry_pub_.publish(odometry_msgs_);

    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odometry_msgs_.pose.pose.position.x , odometry_msgs_.pose.pose.position.y, odometry_msgs_.pose.pose.position.z) );
    transform.setRotation(currentOrientation);
    br.sendTransform(tf::StampedTransform(transform, timeStamp, odometry_msgs_.header.frame_id, odometry_msgs_.child_frame_id));
  }

  /**
   * Integrate odometry pose based on current translation data
   * @param x_velocity current velocity in x direction
   * @param y_velocity current velocity in y direction
   * @param z_alitude current altitude
   * @param duration duration since last update
   * @param timeStamp current time stamp
   */
  void integrateOdometryPos(
      const double x_velocity,
      const double y_velocity,
      const double z_alititude,
      const ros::Duration& duration
  )
  {

    /* Velocity coordinates in PX4Flow and simulated MORSE sensor with front facing x
     *         x
     *        (+)
     *         ^
     *         |
     *       --|--> (+)y
     *         |
     */

    //transfer change into start frame
    //XXX rotate altitude as well?
    //flip y in order to match our/ROS coordinate frame
    //XXX this conversion assumes x directed to the back, think about how this setting could be generalised
    tf::Vector3 delta (-x_velocity * duration.toSec(), y_velocity * duration.toSec(), 0);

    delta = tf::quatRotate(currentOrientation, delta);

    current_pitch_pos_integrated_ += delta.x();
    current_roll_pos_integrated_  += delta.y();
    current_alititude_absolute_ += current_alititude_relative_ - z_alititude; //integrate altitude delta
    current_alititude_relative_ = z_alititude;

  }

  /**
   * Integrate odometry pose based on current rotation data
   * @param x_rot_velocity current rotation around x axis
   * @param y_rot_velocity current rotation around y axis
   * @param z_rot_velocity current rotation around z axis
   * @param duration since last update
   * @param timeStamp current time stamp
   */
  void integratedOdometryRad(
       const double x_rot_velocity,
       const double y_rot_velocity,
       const double z_rot_velocity,
       const ros::Duration& duration
    )
   {
     /*   Rotation coordinate System
      *     x
      *     ^
      *     |
      * y<--|--
      *     |
      *
      *     z axis is up
      */
      //XXX this conversion assumes x directed to the back, think about how this setting could be generalised
     double x_orient_integrated_ = -x_rot_velocity;
     double y_orient_integrated_ = y_rot_velocity;
     double z_orient_integrated_ = -z_rot_velocity;

     //ROS_DEBUG_STREAM("Current Yaw Change: " << to_degrees(z_orient_integrated_));


     tf::Quaternion newQuad = tf::createQuaternionFromRPY( x_orient_integrated_, y_orient_integrated_, z_orient_integrated_);

     currentOrientation = newQuad * currentOrientation;

     currentOrientation = currentOrientation.normalize();

     //tfScalar pitch, roll, yaw;
     //tf::Matrix3x3(currentOrientation).getRPY( roll, pitch, yaw);
     //ROS_DEBUG_STREAM("Current Yaw: " << to_degrees(yaw));

     tf::quaternionTFToMsg(currentOrientation, odometry_msgs_.pose.pose.orientation);
   }

  void updateRotOdometry(const px_comm::OpticalFlowRadConstPtr& px_flow_rad)
  {

    ros::Time now = ros::Time::now();

    ros::Duration duration = now.operator -(last_time_rotation_);

    if(duration.toSec() > 1){
      duration =  ros::Duration(1);
    }

    if(px_flow_rad->quality > px4flow_quality_threshold_){

      integratedOdometryRad(px_flow_rad->integrated_xgyro, px_flow_rad->integrated_ygyro, px_flow_rad->integrated_zgyro, duration);

    }

    publishOdometry(now);

    last_time_rotation_ = now;

  }

  void updateOdometry(const px_comm::OpticalFlowConstPtr& px_flow)
  {
    //ROS_DEBUG_STREAM("Current Distance: " << px_flow->ground_distance);

    ros::Time now = ros::Time::now();

    ros::Duration duration = now.operator - (last_time_odometry_);

    //limit duration if update rate goes below one Hertz, this is also the case on initialization
    if(duration.toSec() > 1){
      duration =  ros::Duration(1);
    }

    double velocity_x = 0;
    double velocity_y = 0;
    double ground_distance = 0;

    if(px_flow->quality > px4flow_quality_threshold_){

      if(external_distance_sensor_){

        double external_altitude = getExternalAltitude(now);
        if(!simulation_mode_)
        {
          velocity_x = px_flow->velocity_x_raw * external_altitude;
          velocity_y = px_flow->velocity_y_raw * external_altitude;
        }else{
          velocity_x = px_flow->velocity_x;
          velocity_y = px_flow->velocity_y;
        }
        ground_distance = external_altitude;
      }else{
        velocity_x = px_flow->velocity_x;
        velocity_y = px_flow->velocity_y;
        ground_distance = px_flow->ground_distance;
      }
    }else{ // distance is not considered in quality attribute
      if(external_distance_sensor_){
        ground_distance = getExternalAltitude(now);
      }else{
        ground_distance = px_flow->ground_distance;
      }

    }
    integrateOdometryPos(velocity_x, velocity_y, ground_distance, duration);

    //only publish after rad update since the updates are always coming in sequence with first translation and second rotation
    if(external_imu_){
      publishOdometry(now);
    }

    last_time_odometry_ = now;

  }

  double getExternalAltitude(ros::Time now)
  {
    //linear interpolate the altitude based on the last velocity and last update timestamp
    //in order to match the current optical flow update
    if(external_distance_sensor_interpolation_ && now > last_time_external_altitude_){
        ros::Duration duration = now.operator - (last_time_external_altitude_);
        return external_altitude_ + external_altitude_velocity_ * duration.toSec();
    }else{
        return external_altitude_;
    }
  }

  void updateAltidude(const sensor_msgs::RangeConstPtr& range)
  {
    ros::Time now = ros::Time::now();
    ros::Duration duration = now.operator - (last_time_external_altitude_);

    external_altitude_velocity_ = (range->range - external_altitude_) / duration.toSec();

    external_altitude_ = range->range;

    last_time_external_altitude_ = now; 
  }

  void updateIMU(const sensor_msgs::ImuConstPtr& imu)
   {

    tf::Quaternion currentImuOrientation;

    tf::quaternionMsgToTF(imu->orientation, currentImuOrientation);

     static tf::Quaternion lastImuOrientation = currentImuOrientation;

     tf::Quaternion imuOrientationDelta = lastImuOrientation.inverse() * currentImuOrientation;

     currentOrientation = imuOrientationDelta * currentOrientation;

     currentOrientation = currentOrientation.normalize();

     tf::quaternionTFToMsg(currentOrientation, odometry_msgs_.pose.pose.orientation);

     publishOdometry(ros::Time::now());

     lastImuOrientation = currentImuOrientation;
   }

  /**
   * (Re)sets/Initializes the current integrated position
   * @param req
   * @param res
   * @return
   */
  bool setCurrentPos(SetCurrentPos::Request &req, SetCurrentPos::Response &res)
  {
    current_pitch_pos_integrated_ = req.xPos;
    current_roll_pos_integrated_ = req.yPos;
    current_alititude_relative_ = req.zPos;
    current_alititude_absolute_ = req.zPos;

    currentOrientation = tf::createQuaternionFromYaw(req.zRot);

    return true;
  }

  ros::NodeHandle handle_;

  ros::NodeHandle private_handle_;

  ros::Time last_time_odometry_;
  ros::Time last_time_rotation_;
  ros::Time last_time_external_altitude_;

  /**
   * Position integrated over time
   */
  double current_pitch_pos_integrated_;
  double current_roll_pos_integrated_;
  double current_alititude_relative_;
  double current_alititude_absolute_;

  /**
   * Orientation integrated over time
   */

  tf::Quaternion currentOrientation;

  int px4flow_quality_threshold_;

  nav_msgs::Odometry odometry_msgs_;

  ros::Publisher odometry_pub_;

  ros::Subscriber height_range_subscriber_;

  ros::Subscriber flow_subscriber_;
  ros::Subscriber flow__rot_subscriber_;

  ros::Subscriber imu_subscriber_;

  ros::ServiceServer servicePos_;

  double external_altitude_;
  double external_altitude_velocity_;

  bool external_distance_sensor_interpolation_;
  bool external_distance_sensor_;

  bool external_imu_;

  //special flag to deal with special simulation situation
  bool simulation_mode_;

};

}

using namespace position_controller;

/**
 * Topic advertising and publishing is done in the main method
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "optical_flow_to_tf");

  ros::NodeHandle n;

  try
  {
    OpticalFlowToTF flow2TF(n);

    ros::spin();

  }
  catch (...)
  {
    ROS_ERROR("optical_flow_to_tf failed");
    return -1;
  }

  return 0;
}
