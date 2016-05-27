#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "px_comm/OpticalFlowRad.h"
#include "px_comm/OpticalFlow.h"
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <string>
#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "mikrokopter_node/Control.h"
#include "mikrokopter_node/FCStatus.h"
#include "position_controller/SetAltitude.h"
#include "position_controller/SetEnabled.h"
#include "position_controller/SetPos.h"

using namespace std;
using namespace std_msgs;
using namespace angles;
using namespace tf;

namespace position_controller
{

/***
 * Position controller
 * THIS CLASS IS DEPRECATED plese use the tf based version instead
 * author: hrabia
 */
class PositionController
{
public:

  PositionController(ros::NodeHandle& n) :
      handle_(n), last_time_altitude_(ros::Time::now()),last_time_odometry_(ros::Time::now()), last_time_rotation_(ros::Time::now()) ,
      enabled_altitude_control_(false), enabled_pitch_roll_control_(false), enabled_yaw_control_(false),
      yaw_current_(0), max_yaw(20), control_yaw_factor(1), px4flow_quality_threshold_(0), current_alititude_(0),
      private_handle_("~"), target_altitude_(0), thrust_hover_(0), target_pos_x_(0), enabled_external_flight_control_(false),
      target_pos_y_(0), target_rot_z_(0), max_yaw_velocity_(0), control_pitch_roll_factor(0.01), current_pitch_pos_integrated_(0),
      current_roll_pos_integrated_(0), max_velocity(2), max_pitch_roll(50), currentOrientation(0, 0, 0, 1), max_thrust_(UINT8_MAX)
      , thrust_max_change_(2), external_distance_sensor_(false), external_altitude_(0.0), simulation_mode_(false)
{
    //init pid controllers
    pid_altitude_acceleration_.initParam("altitude_pid");
    pid_pitch_.initParam("pitch_pid");
    pid_roll_.initParam("pitch_pid");
    //pid_roll_.initParam("roll_pid"); //TODO
    pid_yaw_.initParam("yaw_pid");

    pid_pitch_pos_.initParam("pitch_pos_pid");
    pid_roll_pos_.initParam("pitch_pos_pid");
    //pid_roll_pos_.initParam("roll_pos_pid"); //TODO
    pid_yaw_angle_.initParam("yaw_angle_pid");

    private_handle_.param<bool>("external_distance_sensor", external_distance_sensor_, external_distance_sensor_);

    //init topic subscription and advertisement
    string height_range_topic;
    private_handle_.param<string>("height_range_input_topic", height_range_topic, height_range_topic);

    string flow_input_topic;
    private_handle_.param<string>("flow_input_topic", flow_input_topic, flow_input_topic);

    string flow_rot_input_topic;
    private_handle_.param<string>("flow_rot_input_topic", flow_rot_input_topic, flow_rot_input_topic);

    string motor_controller_output_topic;
    private_handle_.param<string>("motor_controller_output_topic", motor_controller_output_topic, motor_controller_output_topic);

    string motor_controller_input_topic;
    private_handle_.param<string>("motor_controller_input_topic", motor_controller_input_topic, motor_controller_input_topic);

    string odometry_output_topic;
    private_handle_.param<string>("odometry_output_topic", odometry_output_topic, odometry_output_topic);

    string input_pose_topic;
    private_handle_.param<string>("input_pose_topic", input_pose_topic, input_pose_topic);

    string fc_status_input_topic;
    private_handle_.param<string>("fc_status_input_topic", fc_status_input_topic, "/mikrokopter/fc_status");

    if(input_pose_topic.empty()){
      ROS_ERROR_STREAM("Topic input_pose_topic is not properly configured");
    }

    if(flow_rot_input_topic.empty()){
       ROS_ERROR_STREAM("Topic flow_rot_input_topic is not properly configured");
    }

    if(height_range_topic.empty() && flow_input_topic.empty() ){
      ROS_ERROR_STREAM("Topic height_range_input_topic and flow_input_topic are not properly configured");
      throw -1;
    }else if( motor_controller_output_topic.empty()){
      ROS_ERROR_STREAM("Topic motor_controller_output_topic is not properly configured");
      throw -1;
    } else if( motor_controller_input_topic.empty()){
      ROS_ERROR_STREAM("Topic motor_controller_input_topic is not properly configured");
      throw -1;
    }else{
      ROS_INFO_STREAM("Subscribing to height_range_input_topic: " << height_range_topic);
      ROS_INFO_STREAM("Subscribing to flow_input_topic: " << flow_input_topic);
      ROS_INFO_STREAM("Subscribing to motor_controller_input_topic: " << motor_controller_input_topic);
      ROS_INFO_STREAM("Subscribing to input_pose_topic: " << input_pose_topic);
      ROS_INFO_STREAM("Advertising motor_controller_output_topic: " << motor_controller_output_topic);
    }

    private_handle_.param<int>("thrust_hover", thrust_hover_, thrust_hover_);

    //init altitude error limits
    pid_altitude_max_incline_error_ = 1;
    pid_altitude_max_decline_error_ = -1;

    private_handle_.param<double>("altitude_max_incline_error", pid_altitude_max_incline_error_, pid_altitude_max_incline_error_);
    private_handle_.param<double>("altitude_max_decline_error", pid_altitude_max_decline_error_, pid_altitude_max_decline_error_);

    private_handle_.param<double>("yaw_max_velocity", max_yaw_velocity_, max_yaw_velocity_);
    private_handle_.param<double>("control_pitch_roll_factor", control_pitch_roll_factor, control_pitch_roll_factor);
    private_handle_.param<double>("control_yaw_factor", control_yaw_factor, control_yaw_factor);

    private_handle_.param<double>("max_velocity", max_velocity, max_velocity);
    private_handle_.param<double>("max_pitch_roll", max_pitch_roll, max_pitch_roll);
    private_handle_.param<double>("max_yaw", max_yaw, max_yaw);

    private_handle_.param<double>("max_thrust", max_thrust_, max_thrust_);
    private_handle_.param<int>("thrust_max_change", thrust_max_change_, thrust_max_change_);

    private_handle_.param<int>("px4flow_quality_threshold", px4flow_quality_threshold_, px4flow_quality_threshold_);

    private_handle_.param<bool>("simulation_mode", simulation_mode_, simulation_mode_);

    //set initial message to 0
    mikrokopterMsg_.pitch = 0;
    mikrokopterMsg_.roll = 0;
    mikrokopterMsg_.yaw = 0;

    //topic providers and subscribers
    mikrokopterPub_ = handle_.advertise<mikrokopter_node::Control>(motor_controller_output_topic, 10);

    odometry_pub_ = handle_.advertise<nav_msgs::Odometry>(odometry_output_topic, 10);

    height_range_subscriber_ = handle_.subscribe(height_range_topic, 10, &PositionController::updateAltidude, this);

    flow_subscriber_ = handle_.subscribe(flow_input_topic, 10, &PositionController::updateOdometry, this);

    flow__rot_subscriber_ = handle_.subscribe(flow_rot_input_topic, 10, &PositionController::updateRotOdometry, this);

    flightcontrol_status_subscriber_ = handle_.subscribe(fc_status_input_topic, 10, &PositionController::updateFCStatus, this);

    motor_contol_input_subscriber_ = handle_.subscribe(motor_controller_input_topic, 10, &PositionController::updateMotorControls, this);

    pose_subscriber_ = handle_.subscribe(input_pose_topic, 10, &PositionController::updateTargetPose, this);

    //advertise provided services
    serviceAltitude = handle_.advertiseService("set_altitude", &PositionController::setAltitude, this);
    serviceEnable = handle_.advertiseService("set_enabled", &PositionController::setEnabled, this);
    servicePos = handle_.advertiseService("set_target_pos", &PositionController::setPos, this);

    // advertise pid gain setter services
    pid_gains_setter_altitude_.add(&pid_altitude_acceleration_);
    pid_gains_setter_pitch_.add(&pid_pitch_);
    pid_gains_setter_roll_.add(&pid_roll_);
    pid_gains_setter_yaw_.add(&pid_yaw_);

    pid_gains_setter_pitch_pos_.add(&pid_pitch_pos_);
    pid_gains_setter_roll_pos_.add(&pid_roll_pos_);
    pid_gains_setter_yaw_angle_.add(&pid_yaw_angle_);

    pid_gains_setter_altitude_.advertise("altitude", private_handle_);
    pid_gains_setter_pitch_.advertise("pitch", private_handle_);
    pid_gains_setter_roll_.advertise("roll", private_handle_);
    pid_gains_setter_yaw_.advertise("yaw", private_handle_);

    pid_gains_setter_pitch_pos_.advertise("pitch_pos", private_handle_);
    pid_gains_setter_roll_pos_.advertise("roll_pos", private_handle_);
    pid_gains_setter_yaw_angle_.advertise("yaw_angle", private_handle_);

    if(simulation_mode_){
      enabled_external_flight_control_ = true;
    }

  }

  void setEnabled(bool enableAltitude, bool enablePitchRoll, bool enableYaw)
  {
    this->enabled_altitude_control_ = enableAltitude;
    this->enabled_pitch_roll_control_ = enablePitchRoll;
    this->enabled_yaw_control_ = enableYaw;
  }

  void setTargetAltidude(double altidude)
  {
    ROS_DEBUG_STREAM("New Target Altitude: " << altidude);
    this->target_altitude_ = altidude;
  }

  void setTargetPos(double x, double y, double zRot)
  {
    this->target_pos_x_ = x;
    this->target_pos_y_ = y;
    this->target_rot_z_ = normalize_angle_positive(zRot);
  }

protected:

  bool setEnabled(SetEnabled::Request &req, SetEnabled::Response &res)
  {
    setEnabled(req.enableAltitudeControl, req.enablePitchRollControl, req.enableYawControl);
    return true;
  }

  bool setAltitude(SetAltitude::Request &req, SetAltitude::Response &res)
  {
    setTargetAltidude(req.altitude);
    return true;
  }

  bool setPos(SetPos::Request &req, SetPos::Response &res)
  {
    setTargetPos(req.xPos, req.yPos, req.zRot);
    return true;
  }


  /**
   * Publish current odometry
   *
   * @param timeStamp
   */
  void publishOdometry(const ros::Time& timeStamp)
  {
    odometry_msgs_.header.stamp = timeStamp;
    odometry_msgs_.header.seq++;
    odometry_msgs_.header.frame_id = "odom";
    odometry_msgs_.child_frame_id = "base_link";
    odometry_msgs_.pose.pose.position.x = current_pitch_pos_integrated_; //x to the front
    odometry_msgs_.pose.pose.position.y = current_roll_pos_integrated_; // y to the right
    odometry_msgs_.pose.pose.position.z = - current_alititude_; //z axis looks down
    odometry_pub_.publish(odometry_msgs_);

    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odometry_msgs_.pose.pose.position.x , odometry_msgs_.pose.pose.position.y, odometry_msgs_.pose.pose.position.z) );
    transform.setRotation(currentOrientation);
    //TODO use configuration for frame name
    br.sendTransform(tf::StampedTransform(transform, timeStamp, "odom", "base_link"));
  }


 /**
  * Update altitude control based on new sensor data
  * @param current_altitude the current relative altitude to ground
  * @param duration duration since last update
  */
  void updateAltitudeController(const double current_altitude, const ros::Duration& duration)
  {
      if(duration == ros::Duration(0.0)){
        ROS_ERROR("Duration is 0");
      }
      if (enabled_altitude_control_ && enabled_external_flight_control_)
      {
        double altitude_error = target_altitude_ - current_altitude;

        //limit error
        altitude_error = min(altitude_error, pid_altitude_max_incline_error_);
        altitude_error = max(altitude_error, pid_altitude_max_decline_error_);

        ROS_DEBUG_STREAM("Current Altitude: " << current_altitude << " Error: "<< altitude_error);

        double thrust_pid = pid_altitude_acceleration_.computeCommand(altitude_error, duration);

        double thrust = thrust_hover_ + thrust_pid;

        double thrust_difference = thrust - (double) mikrokopterMsg_.thrust; //calculated thrust - current thrust
        double thrust_difference_abs = abs(thrust_difference);
        if (thrust_difference_abs > thrust_max_change_ && current_altitude > 0.6) //TODO altitude offset to config
        {
          //calculate direction of change
          //double thrust_change = (thrust_difference / thrust_difference_abs) * thrust_max_change_;
          //ROS_DEBUG_STREAM("Limited Altitude Change: " << thrust_change  << " diff: " << thrust_difference);
          //thrust = mikrokopterMsg_.thrust + thrust_change;
          if(thrust_difference > 0){
            thrust = mikrokopterMsg_.thrust + thrust_max_change_;
          }else{
            thrust = mikrokopterMsg_.thrust - thrust_max_change_;
          }
          ROS_DEBUG_STREAM("Limited Altitude Change ");
        }

        thrust = min(max_thrust_, thrust);
        thrust = max(thrust, 0.0);

        mikrokopterMsg_.thrust = thrust;

        double pe = 0;
        double ie = 0;
        double de = 0;
        pid_altitude_acceleration_.getCurrentPIDErrors(&pe, &ie, &de);
//
        ROS_DEBUG_STREAM("Thrust:" << thrust <<" Current errors - Alt:" << altitude_error);
        ROS_DEBUG_STREAM("P:" << pe << " I:" << ie << " D:"<<de);

      }
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

    //TODO integrate absolute altitude from start

    //transfer change into start frame
    //TODO rotate altitude as well?
    tf::Vector3 delta (x_velocity * duration.toSec(), y_velocity * duration.toSec(), 0);

    delta = tf::quatRotate(currentOrientation, delta);

    current_pitch_pos_integrated_ += delta.x();
    current_roll_pos_integrated_  += delta.y();
    current_alititude_ = z_alititude;

    /* Velocity coordinates in PX4Flow and simulated MORSE sensor
     *         x
     *        (+)
     *         ^
     *         |
     *       --|--> (+)y
     *         |
     */

    //ROS_DEBUG_STREAM("Current Pos X: " << pitch_pos_integrated_ << " Y: " << roll_pos_integrated_);

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
     double x_orient_integrated_ = x_rot_velocity;
     double y_orient_integrated_ = y_rot_velocity;
     double z_orient_integrated_ = z_rot_velocity;

     /*   Rotation coordinate System
      *     x
      *     ^
      *     |
      *     |-->y
      *     |
      *
      *     z axis is down
      */
     tf::Quaternion newQuad = tf::createQuaternionFromRPY( y_orient_integrated_, x_orient_integrated_, z_orient_integrated_).normalize();

     currentOrientation = newQuad * currentOrientation;

     currentOrientation = currentOrientation.normalize();

//   tfScalar pitch, roll, yaw;
//   tf::Matrix3x3(currentOrientation).getRPY( roll, pitch, yaw);
//   ROS_DEBUG_STREAM("Current Yaw: " << to_degrees(yaw));

     tf::quaternionTFToMsg(currentOrientation, odometry_msgs_.pose.pose.orientation);
   }

  void updateYawController(
        const double x_rot_velocity,
        const double y_rot_velocity,
        const double z_rot_velocity,
        const ros::Duration& duration)
    {

      if (enabled_yaw_control_ && enabled_external_flight_control_)
      {
        double yaw_angle_error = shortest_angular_distance(tf::getYaw(currentOrientation), target_rot_z_);

        //ROS_DEBUG_STREAM("Yaw error: " << to_degrees(yaw_angle_error));

        double target_velocity_zrot = pid_yaw_angle_.computeCommand(yaw_angle_error, duration);

        target_velocity_zrot = min(target_velocity_zrot, max_yaw_velocity_);
        target_velocity_zrot = max(target_velocity_zrot, -max_yaw_velocity_);

        //ROS_DEBUG_STREAM("Current Velocity zRot: " << z_rot_velocity << " Target: " << target_velocity_zrot);

        double yaw_vel_error = (target_velocity_zrot - z_rot_velocity);

        /////////////////////////////////////////////////////////////////////////////////////
        // yaw -> z Rotation
        double yaw = pid_yaw_.computeCommand(yaw_vel_error, duration);

        yaw = min(max_yaw,  yaw);
        yaw = max(-max_yaw, yaw);

        mikrokopterMsg_.yaw = yaw;

        //ROS_DEBUG_STREAM("Yaw Output:" << yaw);

      }
    }

  void updateRollPitchController(
      const double x_velocity,
      const double y_velocity,
      const ros::Duration& duration)
  {

    if (enabled_pitch_roll_control_ && enabled_external_flight_control_)
    {

      double pitch_pos_error = target_pos_x_ - current_pitch_pos_integrated_;
      double roll_pos_error = target_pos_y_ - current_roll_pos_integrated_;

      //ROS_DEBUG_STREAM("Target Pos X: " << target_pos_x_ << " Y: " << target_pos_y_);

      ROS_DEBUG_STREAM("---");
      ROS_DEBUG_STREAM("Current Pos X: " << current_pitch_pos_integrated_ << " Y: " << current_roll_pos_integrated_);

      //ROS_DEBUG_STREAM("Current Pos Error X: " << pitch_pos_error << " Y: " << roll_pos_error);

      double target_velocity_x = pid_pitch_pos_.computeCommand(pitch_pos_error, duration);
      double target_velocity_y = pid_roll_pos_.computeCommand(roll_pos_error, duration);

      target_velocity_x = min(target_velocity_x, max_velocity);
      target_velocity_x = max(target_velocity_x, -max_velocity);
      target_velocity_y = min(target_velocity_y, max_velocity);
      target_velocity_y = max(target_velocity_y, -max_velocity);

      //ROS_DEBUG_STREAM("Current Velocity X: " << x_velocity << " Target: " << target_velocity_x);
      //ROS_DEBUG_STREAM("Current Velocity Y: " << y_velocity << " Target: " << target_velocity_y);

      double pitch_vel_error = (-1) * (target_velocity_x - x_velocity);
      double roll_vel_error = target_velocity_y - y_velocity;

      //ROS_DEBUG_STREAM("Current Vel Error X: " << pitch_vel_error << " Y: " << roll_vel_error);

      /* Roll/Pitch value position coordinate influences in MORSE simulation
       * Neutral value is 0, conversation is done in mikrokopter node
       *
       *        |
       *      --|-->(+) Roll
       *        v
       *       (+)
       *      Pitch
       */

      /////////////////////////////////////////////////////////////////////////////////////
      // PITCH -> X
      double pitch = pid_pitch_.computeCommand(pitch_vel_error, duration);

      pitch = min(max_pitch_roll,  pitch);
      pitch = max(-max_pitch_roll, pitch);

      mikrokopterMsg_.pitch = pitch;

      /////////////////////////////////////////////////////////////////////////////////////
      // ROLL -> y
      double roll = pid_roll_.computeCommand(roll_vel_error, duration);

      roll = min(max_pitch_roll,  roll);
      roll = max(-max_pitch_roll, roll);

      mikrokopterMsg_.roll = roll;

      ROS_DEBUG_STREAM("Pitch:" << pitch << " Roll:" << roll);


      double pe = 0;
      double ie = 0;
      double de = 0;

      pid_pitch_.getCurrentPIDErrors(&pe, &ie, &de);

      ROS_DEBUG_STREAM("Pitch:" << pitch <<" Current errors - Error:" << pitch_vel_error);
      ROS_DEBUG_STREAM("P:" << pe << " I:" << ie << " D:"<<de);

      pid_roll_.getCurrentPIDErrors(&pe, &ie, &de);

      ROS_DEBUG_STREAM("Roll:" << roll <<" Current errors - Error:" << roll_vel_error);
      ROS_DEBUG_STREAM("P:" << pe << " I:" << ie << " D:"<<de);

    }
  }

  void updateRotOdometry(const px_comm::OpticalFlowRadConstPtr& px_flow_rad)
  {

    ros::Time now = ros::Time::now();

    ros::Duration duration = now.operator -(last_time_rotation_);

    if(duration.toSec() > 1){
      duration =  ros::Duration(1);
    }

    if(px_flow_rad->quality >= px4flow_quality_threshold_){

      integratedOdometryRad(px_flow_rad->integrated_xgyro, px_flow_rad->integrated_ygyro, px_flow_rad->integrated_zgyro, duration);

      updateYawController(px_flow_rad->integrated_xgyro, px_flow_rad->integrated_ygyro, px_flow_rad->integrated_zgyro, duration);

    }else{
      mikrokopterMsg_.yaw = 0;
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

    if(px_flow->quality >= px4flow_quality_threshold_){

      double velocity_x;
      double velocity_y;
      double ground_distance;

      if(external_distance_sensor_){
        if(!simulation_mode_)
        {
          velocity_x = px_flow->velocity_x_raw * external_altitude_;
          velocity_y = px_flow->velocity_y_raw * external_altitude_;
        }else
        {
          velocity_x = px_flow->velocity_x;
          velocity_y = px_flow->velocity_y;
        }
        ground_distance = external_altitude_;
      }else{
        velocity_x = px_flow->velocity_x;
        velocity_y = px_flow->velocity_y;
        ground_distance = px_flow->ground_distance;
      }

      integrateOdometryPos(velocity_x, velocity_y, ground_distance, duration);

      updateRollPitchController(velocity_x, velocity_y, duration);

    }else{
      mikrokopterMsg_.roll = 0;
      mikrokopterMsg_.pitch = 0;
    }

    publishOdometry(now);

    last_time_odometry_ = now;

    ros::Duration duration_alt = now.operator - (last_time_altitude_);

    //limit duration if update rate goes below one Hertz, this is also the case on initialization
    if(duration_alt.toSec() > 1){
      duration_alt =  ros::Duration(1);
    }

    //TODO disabled because odometry distance is absolute and not relative in simulation
    if(!external_distance_sensor_){
      updateAltitudeController(px_flow->ground_distance, duration_alt);

      last_time_altitude_ = now; //in case we use odometry and another altitude sensor in parallel
    }

    publishMotorOutput(now);
  }

  void updateAltidude(const sensor_msgs::RangeConstPtr& range)
  {

    ros::Time now = ros::Time::now();

    ros::Duration duration = now.operator -(last_time_altitude_);

    if(duration.toSec() > 1){
      duration =  ros::Duration(1);
    }
    external_altitude_ = range->range;

    updateAltitudeController(external_altitude_, duration);

    publishMotorOutput(now);

    last_time_altitude_ = now;
  }

  void updateTargetPose(const geometry_msgs::Pose& newTargetPose){
    target_pos_x_ = newTargetPose.position.x;
    target_pos_y_ = newTargetPose.position.y;
    //TODO may switch sign
    target_altitude_ = newTargetPose.position.z;
    target_rot_z_ = tf::getYaw(newTargetPose.orientation);
  }

  void updateFCStatus(const mikrokopter_node::FCStatus& status)
  {
    bool armedAndExternalControl = status.externalControl && status.motorsArmed;

    if( this->enabled_external_flight_control_ != armedAndExternalControl){
      if(armedAndExternalControl ){ //switched on
        ROS_DEBUG("External control activated");
      }else{ //switched off
        ROS_DEBUG("External control deactivated");
      }
      //reset pids
      pid_altitude_acceleration_.reset();
      pid_pitch_.reset();
      pid_roll_.reset();
      pid_yaw_.reset();
      pid_pitch_pos_.reset();
      pid_roll_pos_.reset();
      pid_yaw_angle_.reset();

      this->enabled_external_flight_control_ = armedAndExternalControl;
    }
  }


  /**
   * Merge external motor controls into position controller depending on current state
   */
  void updateMotorControls(const mikrokopter_node::Control& controls)
  {

    if (!enabled_altitude_control_)
    {
      //override pid controller with manual thrust
      mikrokopterMsg_.thrust = controls.thrust;
    }

    double x = target_pos_x_;
    double y = target_pos_y_;
    double zRot = target_rot_z_;

    if (enabled_pitch_roll_control_)
    {
      //rotate commands into position base_frame
      tf::Vector3 delta (controls.pitch * control_pitch_roll_factor, - controls.roll * control_pitch_roll_factor, 0);

      delta = tf::quatRotate(currentOrientation.inverse(), delta);

      //calculate relative movement based on given pitch roll
      x -= delta.x();
      y -= delta.y();

    }else{
      mikrokopterMsg_.pitch = controls.pitch;
      mikrokopterMsg_.roll = controls.roll;
    }

    if(enabled_yaw_control_){
      zRot += controls.yaw * control_yaw_factor;
    }else{
      mikrokopterMsg_.yaw = controls.yaw;
    }

    setTargetPos(x, y, zRot);

    if(!enabled_yaw_control_ || !enabled_pitch_roll_control_ || !enabled_altitude_control_){
      publishMotorOutput();
    }
  }

  /**
   * Publish calculated/updated motor output date. Since this node is handling the callbacks in a single thread
   * no mutex is needed here
   */
  void publishMotorOutput(ros::Time timeStamp = ros::Time::now()){
    mikrokopterMsg_.header.stamp = timeStamp;
    mikrokopterMsg_.header.seq++;
    mikrokopterPub_.publish(mikrokopterMsg_);
  }

  // services

  ros::ServiceServer serviceAltitude;
  ros::ServiceServer serviceEnable;
  ros::ServiceServer servicePos;

  ros::NodeHandle handle_;

  ros::NodeHandle private_handle_;

  //velocity control
  control_toolbox::PidGainsSetter pid_gains_setter_altitude_;
  control_toolbox::PidGainsSetter pid_gains_setter_pitch_;
  control_toolbox::PidGainsSetter pid_gains_setter_roll_;
  control_toolbox::PidGainsSetter pid_gains_setter_yaw_;

  control_toolbox::PidGainsSetter pid_gains_setter_pitch_pos_;
  control_toolbox::PidGainsSetter pid_gains_setter_roll_pos_;
  control_toolbox::PidGainsSetter pid_gains_setter_yaw_angle_;

  control_toolbox::Pid pid_altitude_acceleration_;
  control_toolbox::Pid pid_pitch_;
  control_toolbox::Pid pid_roll_;
  control_toolbox::Pid pid_yaw_;

  control_toolbox::Pid pid_pitch_pos_;
  control_toolbox::Pid pid_roll_pos_;
  control_toolbox::Pid pid_yaw_angle_;

//limit the error
  double pid_altitude_max_incline_error_; //positive value
  double pid_altitude_max_decline_error_; // negative value

  ros::Time last_time_altitude_;
  ros::Time last_time_odometry_;
  ros::Time last_time_rotation_;

// in meters
  double target_altitude_;

  //required thrust for staying in hover
  int thrust_hover_;

  int yaw_current_;

  int thrust_max_change_; //max thrust adjustment by the controller per update //TODO check if necessary at all

  /**
   * Position integrated over time
   */
  double current_pitch_pos_integrated_;
  double current_roll_pos_integrated_;
  double current_alititude_;

  /**
   * Orientation integrated over time
   */

  tf::Quaternion currentOrientation;

  int px4flow_quality_threshold_;

  mikrokopter_node::Control mikrokopterMsg_;

  nav_msgs::Odometry odometry_msgs_;

  ros::Publisher mikrokopterPub_;
  ros::Publisher odometry_pub_;

  ros::Subscriber height_range_subscriber_;

  ros::Subscriber flow_subscriber_;
  ros::Subscriber flow__rot_subscriber_;

  ros::Subscriber pose_subscriber_;

  ros::Subscriber flightcontrol_status_subscriber_;
  ros::Subscriber motor_contol_input_subscriber_;


  double target_pos_x_;
  double target_pos_y_;
  double target_rot_z_; //radian


  double max_thrust_;
  double max_velocity;
  double max_pitch_roll;
  double max_yaw;
  double control_pitch_roll_factor;
  double control_yaw_factor;
  double max_yaw_velocity_;

  double external_altitude_;

  bool external_distance_sensor_;

  // indicates if the position controller is able to influence the flight controller
  bool enabled_external_flight_control_;

  //enable altitude control
  bool enabled_altitude_control_;

  //enable pitch/roll loiter control (relative movement based on odometry)
  bool enabled_pitch_roll_control_;

  //enable yaw control (relative rotation based on odometry)
  bool enabled_yaw_control_;

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

  ros::init(argc, argv, "position_controller");

  ros::NodeHandle n;

  try
  {
    PositionController controller(n);

    controller.setTargetAltidude(1.00);
    controller.setTargetPos(0.0, 0.0, 0.0);
    controller.setEnabled(true, true, true);

    ros::spin();

  }
  catch (...)
  {
    ROS_ERROR("position_controller failed");
    return -1;
  }

  return 0;
}
