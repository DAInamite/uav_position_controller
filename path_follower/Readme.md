#path_follower

The path_follower sequentially executes trajectories (array of PoseStamped) it receives on its action /follow_path
by instructing the position_controller. Therefore it converts the given PoseStampeds into the target coordinate frame based on the frame_id of the message.

The frame representing the robot (robot_frame) as well as the navigation map (position_controller_nav_frame) can be configured in the launch configuration, as well as the thresholds that determine if the robot has reached a position/orientation(pose_threshold_d/pose_threshold_r). 
The robot frame and navigation frame should match the configuration of the position_controller


