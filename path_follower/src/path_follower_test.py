#! /usr/bin/env python

import roslib
roslib.load_manifest('path_follower')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import path_follower.msg

import math

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Quaternion

from geometry_msgs.msg import PoseStamped

# Convert from given degree of yaw rotation to geometry_msgs.msg.Quaternion
def quaternion_from_yaw_degree(yaw_degree):
    q = quaternion_from_euler(0, 0, math.radians(yaw_degree))
    
    return  Quaternion(*q)

# Define a fixed example path
def fixed_path():

    p00 = PoseStamped()
    p00.pose.position.x = 0.0
    p00.pose.orientation = quaternion_from_yaw_degree(0)

    p0 = PoseStamped()
    p0.pose.position.x = 2.0
    p0.pose.orientation = quaternion_from_yaw_degree(0)

    p1 = PoseStamped()
    p1.pose.position.x = 2.0
    p1.pose.position.y = 2.0
    p1.pose.orientation = quaternion_from_yaw_degree(0)

    p2 = PoseStamped()
    p2.pose.position.x = 0.0
    p2.pose.position.y = 2.0
    p2.pose.orientation = quaternion_from_yaw_degree(0)

    p3 = PoseStamped()
    p3.pose.position.x = 0.0
    p3.pose.position.y = 0.0
    p3.pose.orientation = quaternion_from_yaw_degree(0)

    return [p00,p0, p1, p2, p3]


def test():
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('/follow_path', path_follower.msg.FollowPathAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = path_follower.msg.FollowPathGoal()
    path = goal.path
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'odom'
    path.poses = fixed_path()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    print client.get_state(), client.get_goal_status_text()

    # Prints out the result of executing the action
    return client.get_result()  # final pose

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('path_follower_test')
        result = test()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
