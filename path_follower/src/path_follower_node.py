#! /usr/bin/env python

import roslib
roslib.load_manifest('path_follower')
import rospy
import actionlib

from path_follower.msg import FollowPathAction, FollowPathFeedback, FollowPathResult
import tf
from position_controller.srv import SetPos
from nav_msgs.msg import Path, Odometry


class PathFollower:
    def __init__(self):
        self._position_controller_nav_frame = rospy.get_param('~position_controller_nav_frame', 'odom')

        self._current_pose = None
        self._robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self._next_target_tf_name = rospy.get_param('~target_tf_name', 'path_follower_next_target')
        self._pose_threshold_d = rospy.get_param('~pose_threshold_d', 0.2)
        self._pose_threshold_r = rospy.get_param('~pose_threshold_r', 0.2)

        rospy.loginfo('create TransformListener')
        self._tf_listener = tf.TransformListener()
        rospy.loginfo('create TransformBroadcaster')
        self._tf_broadcaster = tf.TransformBroadcaster()

        set_pose_name = rospy.get_param('~set_pose', '/position_controller/set_target_pos')
        rospy.loginfo('waiting for %s...' % set_pose_name)
        rospy.wait_for_service(set_pose_name)
        self._set_pose = rospy.ServiceProxy(set_pose_name, SetPos)
        self._set_pose_thread = None

        # for debug
        self._path_pub = rospy.Publisher('path_follower_path', Path, queue_size=10)

        action_name = rospy.get_param('~action_name', '/follow_path')
        rospy.loginfo('create action server %s', action_name)
        self.server = actionlib.SimpleActionServer(action_name, FollowPathAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        self._path_pub.publish(goal.path)
        feedback = FollowPathFeedback()
        feedback.number_of_poses_reached = 0
        self.server.publish_feedback(feedback)
        for pose in goal.path.poses:
            p = pose.pose
            try:
                res = self.go_to_pose(p, goal.path.header.frame_id)
                if not res:
                    self.server.set_aborted(text="can't reach target: %s" % str(p))
                    return
                feedback.number_of_poses_reached += 1
                self.server.publish_feedback(feedback)
            except (tf.LookupException, tf.ConnectivityException) as e:
                rospy.logerr("ServiceException: %s", e)
                self.server.set_aborted(text=str(e))
                return

        result = FollowPathResult()
        result.pose = goal.path.poses[-1]
        result.pose.header = goal.path.header
        self.server.set_succeeded(result)

    def go_to_pose(self, target_pose, path_frame):
        
        while not self.server.is_preempt_requested():
            # Calculate goal into local robot path_frame
            pose = self._get_tf_in_target_frame(target_pose, path_frame, self._robot_frame)

            d2 = pose[0] ** 2 + pose[1] ** 2
            r2 = pose[2] ** 2
            # Check if goal is getting 0,0 in local path_frame
            if d2 < self._pose_threshold_d ** 2 and r2 < self._pose_threshold_r ** 2:
                return True
            
            pose = self._get_tf_in_target_frame(target_pose, path_frame, self._position_controller_nav_frame)
            #call position controller service with new target
            self._set_pose(pose[0], pose[1], pose[2])
            #print "Target:", pose[0], pose[1], pose[2]
            rospy.sleep(0.5)
        return False

    def _get_tf_in_target_frame(self, pose, frame, target_frame):
        position = pose.position
        position = (position.x, position.y, position.z)
        orientation = pose.orientation
        orientation = (orientation.x, orientation.y, orientation.z, orientation.w)
        #broadcast the next target position
        self._tf_broadcaster.sendTransform(position, orientation,
                                           rospy.Time.now(), self._next_target_tf_name, frame)
        
        self._tf_listener.waitForTransform(target_frame, frame,
                                           rospy.Time(0), rospy.Duration(10.0))
        (trans, rot) = self._tf_listener.lookupTransform(target_frame, frame, rospy.Time(0))

        m0 = tf.transformations.translation_matrix(trans).dot(tf.transformations.quaternion_matrix(rot))
        m1 = tf.transformations.translation_matrix(position).dot(tf.transformations.quaternion_matrix(orientation))
        m = m0.dot(m1)
        angle = tf.transformations.euler_from_matrix(m)
        p = tf.transformations.translation_from_matrix(m)
        return [p[0], p[1], angle[-1]]

if __name__ == '__main__':
    rospy.init_node('path_follower')
    server = PathFollower()
    rospy.spin()
