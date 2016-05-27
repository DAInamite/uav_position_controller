#!/usr/bin/env python2

import roslib
roslib.load_manifest('path_follower')
import rospy
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from exploration.srv import GetWaypoint 

# Convert from given degree of yaw rotation to geometry_msgs.msg.Quaternion
def quaternion_from_yaw_degree(yaw_degree):
    q = quaternion_from_euler(0, 0, math.radians(yaw_degree))
    return  Quaternion(*q)

# Define a path to explore
def fixed_path():

    p00 = PoseStamped()
    p00.pose.position.x = 0.0
    p00.pose.orientation = quaternion_from_yaw_degree(0)

    p0 = PoseStamped()
    p0.pose.position.x = 1.0
    p0.pose.orientation = quaternion_from_yaw_degree(0)

    p1 = PoseStamped()
    p1.pose.position.x = -1.0
    p1.pose.orientation = quaternion_from_yaw_degree(0)

    p2 = PoseStamped()
    p2.pose.position.x = 0.0
    p2.pose.orientation = quaternion_from_yaw_degree(0)

    return [p00, p0, p1, p2]


def handle_get_waypoint(request):
    path = fixed_path()
    if request.index < len(path): 
        return fixed_path()[request.index]
    else:
        return None

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('simple_exploration_node')
        rospy.Service('get_waypoint', GetWaypoint, handle_get_waypoint)
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
