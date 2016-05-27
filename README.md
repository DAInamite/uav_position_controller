# uav_position_controller
ROS packages for controlling position, velocity and accelaration of an UAV by interfacing a lower level controller with pitch, roll, yaw and thrust commands

Package includes two sub packages
* position_controller is the actual controller node
* rqt_position_controller is a rqt plugin for external configuration (e.g PIDs)

The position_controller includes 3 nodes
* optical_flow_to_tf_node: A node that is integrating odometry information from the PX4Flow sensor to a tf
* position_controller_tf_node: A generic position controller based on a controlled tf 
* position_controller_node: An old implementation that is directly using the information of the PX4Flow sensor, should not be used anymore (deprecated)

The position_controller dependes on a forked version of 
* control_toolbox: https://github.com/cehberlin/control_toolbox
* px-ros-pkg for the optical_flow_to_tf_node: https://github.com/cehberlin/px-ros-pkg
* mikrokopter_node for the control message type, this is probably going to be changed in the future: https://github.com/DAInamite/mikrokopter_node


