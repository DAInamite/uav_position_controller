#include "ros/ros.h"
#include <sstream>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>
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
 * The class controls UAV motion with PID controllers in a
 * configurable transformation frame including x,y,z + yaw control
 * as well as special landing routines
 * author: hrabia
 */
class PositionControllerTF
{
public:

  /**
   * Constructor and initialization
   * @param n
   */
  PositionControllerTF(ros::NodeHandle& n) :
      handle_(n),last_time_update_(ros::Time::now()) ,
      enabled_altitude_control_(false), enabled_pitch_roll_control_(false), enabled_yaw_control_(false)
      , max_yaw(20), control_yaw_factor(1),
      private_handle_("~"), target_altitude_(0), thrust_hover_(0), target_pos_x_(0), enabled_external_flight_control_(false),
      target_pos_y_(0), target_rot_z_(0), max_yaw_velocity_(0), control_pitch_roll_factor_(0.01),
      max_pitch_roll_(50), max_thrust_(UINT8_MAX), thrust_max_change_(2), max_pitch_roll_velocity_(0.2), simulation_mode_(false),
      landing_turn_off_altitude_(0.25)
{
    //init pid controllers
    pid_altitude_accel_.initParam("altitude_pid");
    pid_pitch_accel_.initParam("pitch_pid");
    pid_roll_accel_.initParam("roll_pid");
    pid_yaw_accel_.initParam("yaw_pid");

    pid_altitude_pos_.initParam("altitude_pos_pid");
    pid_pitch_pos_.initParam("pitch_pos_pid");
    pid_roll_pos_.initParam("roll_pos_pid");
    pid_yaw_angle_.initParam("yaw_angle_pid");

    //init topic subscription and advertisement

    private_handle_.param<string>("robot_frame", robot_frame_, "base_link");

    private_handle_.param<string>("navigation_frame", navigation_frame_, "world");

    string motor_controller_output_topic;
    private_handle_.param<string>("motor_controller_output_topic", motor_controller_output_topic, motor_controller_output_topic);

    string motor_controller_input_topic;
    private_handle_.param<string>("motor_controller_input_topic", motor_controller_input_topic, motor_controller_input_topic);

    string input_pose_topic;
    private_handle_.param<string>("input_pose_topic", input_pose_topic, input_pose_topic);

    string fc_status_input_topic;
    private_handle_.param<string>("fc_status_input_topic", fc_status_input_topic, "/mikrokopter/fc_status");

    if(input_pose_topic.empty()){
      ROS_ERROR_STREAM("Topic input_pose_topic is not properly configured");
    }

    if( motor_controller_output_topic.empty()){
      ROS_ERROR_STREAM("Topic motor_controller_output_topic is not properly configured");
      throw -1;
    } else if( motor_controller_input_topic.empty()){
      ROS_ERROR_STREAM("Topic motor_controller_input_topic is not properly configured");
      throw -1;
    }else{
      ROS_INFO_STREAM("Subscribing to motor_controller_input_topic: " << motor_controller_input_topic);
      ROS_INFO_STREAM("Subscribing to input_pose_topic: " << input_pose_topic);
      ROS_INFO_STREAM("Advertising motor_controller_output_topic: " << motor_controller_output_topic);
    }

    private_handle_.param<int>("thrust_hover", thrust_hover_, thrust_hover_);

    //init altitude error limits
    pid_altitude_max_incline_velocity_ = 1;
    pid_altitude_max_decline_velocity_ = -1;

    private_handle_.param<double>("altitude_max_incline_velocity", pid_altitude_max_incline_velocity_, pid_altitude_max_incline_velocity_);
    private_handle_.param<double>("altitude_max_decline_velocity", pid_altitude_max_decline_velocity_, pid_altitude_max_decline_velocity_);

    private_handle_.param<double>("yaw_max_velocity", max_yaw_velocity_, max_yaw_velocity_);
    private_handle_.param<double>("control_pitch_roll_factor_", control_pitch_roll_factor_, control_pitch_roll_factor_);
    private_handle_.param<double>("control_yaw_factor", control_yaw_factor, control_yaw_factor);

    private_handle_.param<double>("max_pitch_roll_velocity", max_pitch_roll_velocity_, max_pitch_roll_velocity_);
    private_handle_.param<double>("max_pitch_roll", max_pitch_roll_, max_pitch_roll_);
    private_handle_.param<double>("max_yaw", max_yaw, max_yaw);


    private_handle_.param<double>("max_thrust", max_thrust_, max_thrust_);
    private_handle_.param<int>("thrust_max_change", thrust_max_change_, thrust_max_change_);
    private_handle_.param<double>("landing_turn_off_altitude", landing_turn_off_altitude_, landing_turn_off_altitude_);


    private_handle_.param<bool>("simulation_mode", simulation_mode_, simulation_mode_);

    //set initial message to 0
    mikrokopterMsg_.pitch = 0;
    mikrokopterMsg_.roll = 0;
    mikrokopterMsg_.yaw = 0;

    //topic providers and subscribers
    mikrokopterPub_ = handle_.advertise<mikrokopter_node::Control>(motor_controller_output_topic, 10);

    flightcontrol_status_subscriber_ = handle_.subscribe(fc_status_input_topic, 10, &PositionControllerTF::updateFCStatus, this);

    motor_contol_input_subscriber_ = handle_.subscribe(motor_controller_input_topic, 10, &PositionControllerTF::updateMotorControls, this);

    pose_subscriber_ = handle_.subscribe(input_pose_topic, 10, &PositionControllerTF::updateTargetPose, this);

    //advertise provided services
    serviceAltitude_ = handle_.advertiseService("set_altitude", &PositionControllerTF::setAltitude, this);
    serviceEnable_ = handle_.advertiseService("set_enabled", &PositionControllerTF::setEnabled, this);
    servicePos_ = handle_.advertiseService("set_target_pos", &PositionControllerTF::setTargetPos, this);

    // advertise pid gain setter services
    pid_gains_setter_altitude_.add(&pid_altitude_accel_);
    pid_gains_setter_pitch_.add(&pid_pitch_accel_);
    pid_gains_setter_roll_.add(&pid_roll_accel_);
    pid_gains_setter_yaw_.add(&pid_yaw_accel_);

    pid_gains_setter_altitude_pos_.add(&pid_altitude_pos_);
    pid_gains_setter_pitch_pos_.add(&pid_pitch_pos_);
    pid_gains_setter_roll_pos_.add(&pid_roll_pos_);
    pid_gains_setter_yaw_angle_.add(&pid_yaw_angle_);

    pid_gains_setter_altitude_.advertise("altitude", private_handle_);
    pid_gains_setter_pitch_.advertise("pitch", private_handle_);
    pid_gains_setter_roll_.advertise("roll", private_handle_);
    pid_gains_setter_yaw_.advertise("yaw", private_handle_);

    pid_gains_setter_altitude_pos_.advertise("altitude_pos", private_handle_);
    pid_gains_setter_pitch_pos_.advertise("pitch_pos", private_handle_);
    pid_gains_setter_roll_pos_.advertise("roll_pos", private_handle_);
    pid_gains_setter_yaw_angle_.advertise("yaw_angle", private_handle_);

    if (simulation_mode_){
      enabled_external_flight_control_ = true;
    }

  }

  /**
   * enable position controller features
   * @param enableAltitude
   * @param enablePitchRoll
   * @param enableYaw
   */
  void setEnabled(bool enableAltitude, bool enablePitchRoll, bool enableYaw)
  {
    this->enabled_altitude_control_ = enableAltitude;
    this->enabled_pitch_roll_control_ = enablePitchRoll;
    this->enabled_yaw_control_ = enableYaw;
  }

  /**
   * Set new target distance from ground
   * @param altitude
   */
  void setTargetAltidude(double altitude)
  {
    ROS_DEBUG_STREAM("New Target Altitude: " << altitude);
    this->target_altitude_ = altitude;
  }

  /**
   * Set new target x,y and rotation
   * @param x
   * @param y
   * @param zRot
   */
  void setTargetPos(double x, double y, double zRot)
  {
    //ROS_DEBUG_STREAM("New Target X: " << x << " Y: " << y << " Z-Rot: " << zRot);
    this->target_pos_x_ = x;
    this->target_pos_y_ = y;
    this->target_rot_z_ = normalize_angle_positive(zRot);
  }

  /**
   *
   */
  void run()
  {
    ros::Rate r(40); // 40 hz
    while (ros::ok())
    {
      StampedTransform transform_in_target_frame;
      try
      {
        ros::Time now = ros::Time(0);

        //publish tf of current target
        static tf::TransformBroadcaster br;

        tf::Transform target_transform;

        target_transform.setOrigin( tf::Vector3(target_pos_x_, target_pos_y_, target_altitude_) );
        target_transform.setRotation(tf::createQuaternionFromRPY( 0, 0, target_rot_z_));
        br.sendTransform(tf::StampedTransform(target_transform, now, navigation_frame_, "position_control_target"));

        if(tf_listener_.waitForTransform(navigation_frame_, robot_frame_,  now, ros::Duration(1.0))){

          tf_listener_.lookupTransform(navigation_frame_, robot_frame_,  now , transform_in_target_frame);

          Transform current_pos_transform = transform_in_target_frame;

          //ROS_INFO_STREAM("Transform X: " << current_pos_transform.getOrigin().getX() << " Y: " << current_pos_transform.getOrigin().getY() << " Z: " << current_pos_transform.getOrigin().getZ());

          updateFromTF(current_pos_transform, transform_in_target_frame.stamp_);
        }
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      ros::spinOnce();
      r.sleep();
    }
  }

protected:

  /**
   * Enable service handler
   * @param req
   * @param res
   * @return
   */
  bool setEnabled(SetEnabled::Request &req, SetEnabled::Response &res)
  {
    setEnabled(req.enableAltitudeControl, req.enablePitchRollControl, req.enableYawControl);
    return true;
  }

  /**
   * Set Altitude service handler
   * @param req
   * @param res
   * @return
   */
  bool setAltitude(SetAltitude::Request &req, SetAltitude::Response &res)
  {
    setTargetAltidude(req.altitude);
    return true;
  }

  /**
   * Set target position service handler
   * @param req
   * @param res
   * @return
   */
  bool setTargetPos(SetPos::Request &req, SetPos::Response &res)
  {
    setTargetPos(req.xPos, req.yPos, req.zRot);
    return true;
  }

  /**
   * Update routine for altitude landing controller
   * @param velocity_z current decline/incline velocity
   * @param current_altitude current distance to ground
   * @param duration duration since last update
   */
  void updateAltitudeLandingController(const double velocity_z, const double current_altitude, const ros::Duration& duration){

//    Uses the formulas from:
//    (2012), Izzo, D., and de Croon, G.C.H.E., "Landing with time-to-contact and ventral optic flow estimates",
//    in Journal of Guidance, Control, and Dynamics, Volume 35, Issue 4, pages 1362-1367
//    to calculate the target velocity for a soft landing

    if (current_altitude > landing_turn_off_altitude_)
    {
        double altitude_target_velocity = -0.1; //default declination rate

        //only apply this formula for negative velocities in order to realise soft landing
        //because it will result in inclination instead of declenation
        // use default declenation rate if the copter is inclining
        if( velocity_z < 0) {

            double c2 = 0.5; //constant value

            double t = 0; //0 seems to be suificient approximation

            double e_term_1 = exp((c2 * t)) -1 ;

            double e_term_2 = exp((velocity_z / (current_altitude * c2)) * e_term_1 + c2 * t);

            altitude_target_velocity = velocity_z * e_term_2;

            //ROS_DEBUG_STREAM("Landing Control target z velocity: " << altitude_target_velocity);
        }

        double thrust = thrustAccelerationController(altitude_target_velocity, velocity_z, duration);

        mikrokopterMsg_.thrust = thrust;
    }else{
        mikrokopterMsg_.thrust = 0;
    }
  }

 /**
  * Update altitude control based on new sensor data
  * @param current_altitude the current relative altitude to ground
  * @param duration duration since last update
  */
  void updateAltitudeController(const double velocity_z, const double current_altitude, const ros::Duration& duration)
  {
      if(duration == ros::Duration(0.0)){
        ROS_ERROR("Duration is 0");
      }
      if (enabled_altitude_control_ && enabled_external_flight_control_)
      {
        if(target_altitude_ == 0.0){ // use special landing controller for target altitude of 0.0
          updateAltitudeLandingController(velocity_z, current_altitude, duration);
        }else{

          double altitude_pos_error = target_altitude_ - current_altitude;

          //ROS_DEBUG_STREAM("Altitude Current: " << current_altitude << " Error: "<< altitude_pos_error);

          double altitude_target_velocity = pid_altitude_pos_.computeCommand(altitude_pos_error, duration);

          //limit error
          altitude_target_velocity = min(altitude_target_velocity, pid_altitude_max_incline_velocity_);
          altitude_target_velocity = max(altitude_target_velocity, pid_altitude_max_decline_velocity_);

          //ROS_DEBUG_STREAM("Current Altitude: " << current_altitude << " Error: "<< altitude_pos_error);

          double thrust = thrustAccelerationController(altitude_target_velocity, velocity_z, duration);

          mikrokopterMsg_.thrust = thrust;

  //        double pe = 0;
  //        double ie = 0;
  //        double de = 0;
  //        pid_altitude_accel_.getCurrentPIDErrors(&pe, &ie, &de);
  //
  //        ROS_DEBUG_STREAM("Thrust:" << thrust <<" Current errors - Alt:" << altitude_pos_error);
  //        ROS_DEBUG_STREAM("P:" << pe << " I:" << ie << " D:"<<de);
        }
      }
    }

/**
 * thrust velocity controller
 * @param altitude_target_velocity target z velocity
 * @param velocity_z current z velocity
 * @param duration duration since last update
 * @return thrust value
 */
  double thrustAccelerationController(double altitude_target_velocity, const double velocity_z,
                                           const ros::Duration& duration)
  {
    double altitude_velocity_error = altitude_target_velocity - velocity_z;

    ROS_DEBUG_STREAM("Altitude Velo: " << velocity_z << " Target: "<< altitude_target_velocity << " Error:" << altitude_velocity_error);

    double thrust_pid = pid_altitude_accel_.computeCommand(altitude_velocity_error, duration);

    //ROS_DEBUG_STREAM("Thrust PID: " << thrust_pid);

    double thrust = thrust_hover_ + thrust_pid;

    double thrust_difference = thrust - (double) mikrokopterMsg_.thrust; //calculated thrust - current thrust
    double thrust_difference_abs = abs(thrust_difference);

    //limit acceleration
    if (thrust_difference_abs > thrust_max_change_)
    {
      if(thrust_difference > 0){
        thrust = mikrokopterMsg_.thrust + thrust_max_change_;
      }else{
        thrust = mikrokopterMsg_.thrust - thrust_max_change_;
      }
      ROS_DEBUG_STREAM("Limited Alt delta: " << thrust_difference << " to all: " << thrust);
    }

    //limit general thrust
    thrust = min(max_thrust_, thrust);
    thrust = max(thrust, 0.0);

    return thrust;
  }

  /**
   * Yaw rotation controller
   * @param current_yaw current yaw in radians
   * @param z_rot_velocity yaw rotation veloctiy
   * @param duration duration since last update
   */
  void updateYawController(
        const double current_yaw,
        const double z_rot_velocity,
        const ros::Duration& duration)
    {

      if (enabled_yaw_control_ && enabled_external_flight_control_)
      {
        double yaw_angle_error = shortest_angular_distance(current_yaw, target_rot_z_);

        ROS_DEBUG_STREAM("Yaw error: " << to_degrees(yaw_angle_error));

        double target_velocity_zrot = pid_yaw_angle_.computeCommand(yaw_angle_error, duration);

        target_velocity_zrot = min(target_velocity_zrot, max_yaw_velocity_);
        target_velocity_zrot = max(target_velocity_zrot, -max_yaw_velocity_);

        ROS_DEBUG_STREAM("Current Velocity zRot: " << z_rot_velocity << " Target: " << target_velocity_zrot);

        double yaw_vel_error = (target_velocity_zrot - z_rot_velocity);

        /////////////////////////////////////////////////////////////////////////////////////
        // yaw -> z Rotation
        double yaw = pid_yaw_accel_.computeCommand(yaw_vel_error, duration);

        yaw = min(max_yaw,  yaw);
        yaw = max(-max_yaw, yaw);

        mikrokopterMsg_.yaw = -yaw;

        ROS_DEBUG_STREAM("Yaw Output:" << -yaw);

      }
    }

  /**
   * Update routine of the pitch roll controller
   * @param x_pos current x position
   * @param y_pos current y position
   * @param x_velocity current x velocity
   * @param y_velocity current y velocity
   * @param duration duration since last update
   */
  void updateRollPitchController(
      const double x_pos,
      const double y_pos,
      const Quaternion current_rot,
      const double x_velocity,
      const double y_velocity,
      const ros::Duration& duration)
  {

    if (enabled_pitch_roll_control_ && enabled_external_flight_control_)
    {

      double pitch_pos_error = target_pos_x_ - x_pos;
      double roll_pos_error = target_pos_y_ - y_pos;

      ROS_DEBUG_STREAM("Current Pos Error X: " << pitch_pos_error << " Y: " << roll_pos_error);

      double target_velocity_x = pid_pitch_pos_.computeCommand(pitch_pos_error, duration);
      double target_velocity_y = pid_roll_pos_.computeCommand(roll_pos_error, duration);

      //limit target velocity
      target_velocity_x = min(target_velocity_x, max_pitch_roll_velocity_);
      target_velocity_x = max(target_velocity_x, -max_pitch_roll_velocity_);
      target_velocity_y = min(target_velocity_y, max_pitch_roll_velocity_);
      target_velocity_y = max(target_velocity_y, -max_pitch_roll_velocity_);


      ROS_DEBUG_STREAM("Current Velocity X: " << x_velocity << " Target: " << target_velocity_x);
      ROS_DEBUG_STREAM("Current Velocity Y: " << y_velocity << " Target: " << target_velocity_y);

      double pitch_vel_error    = target_velocity_x - x_velocity;
      double roll_vel_error     = target_velocity_y - y_velocity;

      //ROS_DEBUG_STREAM("Current Vel Error X: " << pitch_vel_error << " Y: " << roll_vel_error);

      // PITCH -> X
      double pitch = pid_pitch_accel_.computeCommand(pitch_vel_error, duration);

      pitch = min(max_pitch_roll_,  pitch);
      pitch = max(-max_pitch_roll_, pitch);

      // ROLL -> y
      double roll = pid_roll_accel_.computeCommand(roll_vel_error, duration);

      roll = min(max_pitch_roll_,  roll);
      roll = max(-max_pitch_roll_, roll);

      /* Roll/Pitch value position coordinate influences in MORSE simulation
       * Neutral value is 0, further conversation is done in mikrokopter node
       *
       *        |
       *      --|-->(+) Roll
       *        v
       *       (+)
       *      Pitch
       */

      /*
       * Current orientation  has to be taken into account
       *
       */
      tf::Vector3 pitch_roll (-pitch, -roll, 0);

      pitch_roll = tf::quatRotate(current_rot.inverse(), pitch_roll);


      mikrokopterMsg_.pitch = pitch_roll.getX();
      mikrokopterMsg_.roll  = pitch_roll.getY();

      ROS_DEBUG_STREAM("Pitch:" << pitch << " Roll:" << roll);
//
//      double pe = 0;
//      double ie = 0;
//      double de = 0;
//
//      pid_pitch_.getCurrentPIDErrors(&pe, &ie, &de);
//
//      ROS_DEBUG_STREAM("Pitch:" << pitch <<" Current errors - Error:" << pitch_vel_error);
//      ROS_DEBUG_STREAM("P:" << pe << " I:" << ie << " D:"<<de);
//
//      pid_roll_.getCurrentPIDErrors(&pe, &ie, &de);
//
//      ROS_DEBUG_STREAM("Roll:" << roll <<" Current errors - Error:" << roll_vel_error);
//      ROS_DEBUG_STREAM("P:" << pe << " I:" << ie << " D:"<<de);

    }
  }

  /**
   * Main control loop that calculates all required data
   * from current and last transformation
   * @param currentTransform current transformation of the robot in the navigation frame
   */
  void updateFromTF(Transform currentTransform, ros::Time now)
  {
    //skip update if we do not have new tf data
    if(now <= last_time_update_){
      return;
    }

    ros::Duration duration = now.operator -(last_time_update_);

    if (duration.toSec() > 1)
    {
      duration = ros::Duration(1);
    }

    if(last_time_update_ != now){

      double ground_distance = 0;
      double pos_x = 0;
      double pos_y = 0;
      double velocity_x = 0;
      double velocity_y = 0;
      double velocity_z = 0;

      pos_x =  currentTransform.getOrigin().getX();
      pos_y =  currentTransform.getOrigin().getY();
      ground_distance = currentTransform.getOrigin().getZ();

      //ROS_DEBUG_STREAM("Pos x: " << pos_x << "pos y:" << pos_y << "Ground distance " << ground_distance);

      Transform t_diff = last_transform_.inverseTimes(currentTransform); //the position displacement
      Vector3 translational_velocity = t_diff.getOrigin() / duration.toSec();

      Quaternion rotational_velocity = t_diff.getRotation() / duration.toSec();

      velocity_x = translational_velocity.x();
      velocity_y = translational_velocity.y();
      velocity_z = translational_velocity.z();

      //ROS_DEBUG_STREAM("Velocity x: " << velocity_x << " y:" << velocity_y << " z:" << velocity_z);

      //disable all controllers if the robot is landed
      if(!isLanded(ground_distance))
      {
        updateAltitudeController(velocity_z, ground_distance, duration);

        Quaternion currentOrientation = currentTransform.getRotation();

        updateRollPitchController(pos_x, pos_y, currentOrientation, velocity_x, velocity_y, duration);

        updateYawController(tf::getYaw(currentOrientation), getYaw(rotational_velocity), duration);
      }else{
        mikrokopterMsg_.pitch = 0;
        mikrokopterMsg_.roll = 0;
        mikrokopterMsg_.yaw = 0;
        mikrokopterMsg_.thrust = 0;
        resetPIDs();
      }

      publishMotorOutput(now);

    }

    last_time_update_ = now;

    last_transform_ = currentTransform;
  }

  /**
   * Target pose topic subscribtion handler
   * @param newTargetPose
   */
  void updateTargetPose(const geometry_msgs::Pose& newTargetPose){
    target_pos_x_ = newTargetPose.position.x;
    target_pos_y_ = newTargetPose.position.y;
    target_altitude_ = newTargetPose.position.z;
    target_rot_z_ = tf::getYaw(newTargetPose.orientation);
  }

  /**
   * FlightControl topic subscription handler
   * reset pid controllers if the system can not
   * be controlled anymore (disabled motors, or external
   * control switch)
   * @param status
   */
  void updateFCStatus(const mikrokopter_node::FCStatus& status)
  {
    bool armedAndExternalControl = status.externalControl && status.motorsArmed;

    if( this->enabled_external_flight_control_ != armedAndExternalControl){
      if(armedAndExternalControl ){ //switched on
        ROS_DEBUG("External control activated");
      }else{ //switched off
        ROS_DEBUG("External control deactivated");
      }
      resetPIDs();

      this->enabled_external_flight_control_ = armedAndExternalControl;
    }
  }


  /**
   * Subscription handler for merging external motor controls into
   * position controller depending on current state
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
      tf::Vector3 delta (controls.pitch * control_pitch_roll_factor_, - controls.roll * control_pitch_roll_factor_, 0);

      delta = tf::quatRotate(last_transform_.getRotation().inverse(), delta);

      //calculate relative movement based on given pitch roll
      x -= delta.x();
      y = delta.y();

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

  void resetPIDs()
  {
    pid_altitude_accel_.reset();
    pid_pitch_accel_.reset();
    pid_roll_accel_.reset();
    pid_yaw_accel_.reset();
    pid_pitch_pos_.reset();
    pid_roll_pos_.reset();
    pid_yaw_angle_.reset();
  }

  /**
   * Checks if the robot is currently in landing state
   * @param current_altitude
   * @return return true if landed
   */
  bool isLanded(const double current_altitude)
  {
    return (current_altitude < landing_turn_off_altitude_) && (target_altitude_ == 0.0);
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

  //frames
  string robot_frame_; //frame of the controlled robot
  string navigation_frame_; //world/map frame the robot is located inside

  // services
  ros::ServiceServer serviceAltitude_;
  ros::ServiceServer serviceEnable_;
  ros::ServiceServer servicePos_;

  // Node handles
  ros::NodeHandle handle_;
  ros::NodeHandle private_handle_;

  //acceleration controllers
  control_toolbox::PidGainsSetter pid_gains_setter_altitude_;
  control_toolbox::PidGainsSetter pid_gains_setter_pitch_;
  control_toolbox::PidGainsSetter pid_gains_setter_roll_;
  control_toolbox::PidGainsSetter pid_gains_setter_yaw_;

  //velocity controllers
  control_toolbox::PidGainsSetter pid_gains_setter_altitude_pos_;
  control_toolbox::PidGainsSetter pid_gains_setter_pitch_pos_;
  control_toolbox::PidGainsSetter pid_gains_setter_roll_pos_;
  control_toolbox::PidGainsSetter pid_gains_setter_yaw_angle_;

  control_toolbox::Pid pid_altitude_accel_;
  control_toolbox::Pid pid_pitch_accel_;
  control_toolbox::Pid pid_roll_accel_;
  control_toolbox::Pid pid_yaw_accel_;

  control_toolbox::Pid pid_altitude_pos_;
  control_toolbox::Pid pid_pitch_pos_;
  control_toolbox::Pid pid_roll_pos_;
  control_toolbox::Pid pid_yaw_angle_;

  /**
   * Publishers
   */
  mikrokopter_node::Control mikrokopterMsg_;
  ros::Publisher mikrokopterPub_;
  ros::Publisher odometry_pub_;

  /**
   * Subscribers
   */
  tf::TransformListener tf_listener_;
  ros::Subscriber height_range_subscriber_;
  ros::Subscriber flow_subscriber_;
  ros::Subscriber flow__rot_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber flightcontrol_status_subscriber_;
  ros::Subscriber motor_contol_input_subscriber_;

  //stored transform of the last update
  Transform last_transform_;

  //time stamp of the last update cycle
  ros::Time last_time_update_;

  /**
   * Controller targets
   */

  // target distance from ground in meters
  double target_altitude_;

  /**
   * target position in the configured transformation
   */
  double target_pos_x_;
  double target_pos_y_;
  double target_rot_z_; //radian

  /**
   * Limits for the controllers
   */

  //limit inclination/declenation velocity
  double pid_altitude_max_incline_velocity_; //positive value
  double pid_altitude_max_decline_velocity_; // negative value

  // distance to ground when motors are turned off during landing, in meters
  double landing_turn_off_altitude_;

  int thrust_max_change_; //max thrust adjustment by the controller per update, limits acceleration

  double max_thrust_;
  double max_pitch_roll_;
  double max_pitch_roll_velocity_; //m/s
  double max_yaw;
  double max_yaw_velocity_; //radian/s

  /**
   * manual control configuration
   */
  double control_pitch_roll_factor_;
  double control_yaw_factor;

  /**
   * General controller configuration parameters
   */

  //required thrust for staying in hover
  int thrust_hover_;

  // indicates if the position controller is able to influence the flight controller
  bool enabled_external_flight_control_;

  //enable altitude control
  bool enabled_altitude_control_;

  //enable pitch/roll loiter control (relative movement based on localization)
  bool enabled_pitch_roll_control_;

  //enable yaw control (relative rotation based on localization)
  bool enabled_yaw_control_;

  //special flag to deal with special simulation situation
  bool simulation_mode_;
};

}

using namespace position_controller;

/**
 * Main method that initializes all required objects
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "position_controller");

  ros::NodeHandle n;

  try
  {
    PositionControllerTF controller(n);

    //controller.setTargetAltidude(0.75);
    controller.setTargetPos(0.0, 0.0, 0.0);
    controller.setEnabled(true, true, true);

    controller.run();

  }
  catch (...)
  {
    ROS_ERROR("position_controller failed");
    return -1;
  }

  return 0;
}
