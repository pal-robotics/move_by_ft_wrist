#!/usr/bin/python
"""
Created on 10/05/16

@author: Sammy Pfeiffer
@email: sam.pfeiffer@pal-robotics.com

"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf import TransformListener, ExtrapolationException
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from control_msgs.msg import PointHeadActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamic_reconfigure.server import Server
from move_by_ft_wrist.cfg import ArucoFilterConfig
from math import radians
from copy import deepcopy

class ArucoFilter(object):
    def __init__(self):
        rospy.loginfo("Initializing...")

        # Node Hz rate
        self.rate = 10
        self.rate_changed = False
        self.global_frame_id = '/world'
        self.topic_name = '/aruco_filtered'
        self.initial_pose_head_x = 1.0
        self.initial_pose_head_y = 0.0
        self.initial_pose_head_z = 0.6

        self.initial_pose = Pose()

        self.tf_l = TransformListener()
        rospy.sleep(3.0)  # Let the subscriber to its job

        self.dyn_rec_srv = Server(ArucoFilterConfig, self.dyn_rec_callback)

        self.last_pose = None
        self.new_pose = False
        self.pose_sub = rospy.Subscriber('/aruco_single/pose',
                                         PoseStamped,
                                         self.pose_cb,
                                         queue_size=1)
        rospy.loginfo("Subscribed to: " + str(self.pose_sub.resolved_name))

        self.last_goal_timestamp = rospy.Time.now()
        self.pose_pub = rospy.Publisher(self.topic_name,
                                        PoseStamped,
                                        queue_size=1)

        self.head_pub = rospy.Publisher('/whole_body_kinematic_controler/gaze_objective_stereo_optical_frame_goal',
                                        PoseStamped,
                                        queue_size=1)

        self.point_head_pub = rospy.Publisher('/head_controller/point_head_action/goal',
                                              PointHeadActionGoal,
                                              queue_size=1)

        self.wrist_pub = rospy.Publisher('/whole_body_kinematic_controler/wrist_right_ft_link_goal',
                                         PoseStamped,
                                         queue_size=1)

        self.hand_pub = rospy.Publisher(
            '/right_hand_controller/command', JointTrajectory, queue_size=1)

        rospy.sleep(1.0)
        self.run()

    def pose_cb(self, data):
        self.last_pose = data
        self.new_pose = True

    def run(self):
        r = rospy.Rate(self.rate)
        rospy.loginfo("Node running!")
        while not rospy.is_shutdown():
            # To change the node rate
            if self.rate_changed:
                r = rospy.Rate(self.rate)
                self.rate_changed = False
            self.publish_goal()
            r.sleep()

    def transform_pose_to_world(self, pose, from_frame="arm_right_tool_link"):
        ps = PoseStamped()
        ps.header.stamp = self.tf_l.getLatestCommonTime(
            self.global_frame_id, from_frame)
        if from_frame[0] == '/':
            ps.header.frame_id = from_frame[1:]
        else:
            ps.header.frame_id = from_frame

        # TODO: check pose being PoseStamped or Pose
        ps.pose = pose
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                world_ps = self.tf_l.transformPose(self.global_frame_id, ps)
                transform_ok = True
                return world_ps
            except ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming pose... trying again \n(" + str(e) + ")")
                rospy.sleep(0.05)
                ps.header.stamp = self.tf_l.getLatestCommonTime(
                    self.global_frame_id, from_frame)

    def send_head_point_goal(self, pose):
        g = PointHeadActionGoal()
        t = g.goal.target
        t.header.frame_id = pose.header.frame_id
        t.point = pose.pose.position
        g.goal.pointing_axis.z = 1.0
        g.goal.pointing_frame = 'stereo_optical_frame'
        g.goal.min_duration = rospy.Duration(0.5)
        g.goal.max_velocity = 1.0
        self.point_head_pub.publish(g)


    def publish_goal(self):
        if not self.new_pose and rospy.Time.now() - self.last_goal_timestamp > rospy.Duration(self.goal_timeout):
            # Send home goal
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame_id
            ps.pose = deepcopy(self.initial_pose)
            self.pose_pub.publish(ps)
            if self.send_wrist_goals:
                self.wrist_pub.publish(ps)
            if self.send_head_goals:
                ps_head = PoseStamped()
                ps_head.header.frame_id = self.global_frame_id
                ps_head.pose.position.x = self.initial_pose_head_x
                ps_head.pose.position.y = self.initial_pose_head_y
                ps_head.pose.position.z = self.initial_pose_head_z
                ps_head.pose.orientation.w = 1.0
                self.head_pub.publish(ps_head)
                self.send_head_point_goal(ps_head)
            self.thumbs_up_hand()

        else:
            if self.new_pose:
                self.new_pose = False
                if self.last_pose.header.frame_id != self.global_frame_id:
                    rospy.loginfo("Goal in different frame, (" +
                                  self.last_pose.header.frame_id +
                                  ") transforming to " + self.global_frame_id)
                    goal_pose = self.transform_pose_to_world(
                        self.last_pose.pose, self.last_pose.header.frame_id)
                else:
                    goal_pose = deepcopy(self.last_pose)

                goal_pose.header.stamp = rospy.Time.now()


                goal_pose.pose.position.x = self.sanitize(goal_pose.pose.position.x,
                                                          self.min_x,
                                                          self.max_x)
                goal_pose.pose.position.y = self.sanitize(goal_pose.pose.position.y,
                                                          self.min_y,
                                                          self.max_y)
                goal_pose.pose.position.z = self.sanitize(goal_pose.pose.position.z,
                                                          self.min_z,
                                                          self.max_z)

                # TODO: deal with orientation
                goal_pose.pose.orientation = deepcopy(self.initial_pose.orientation)

                if self.use_marker_orientation:
                    # Add roll to the hand, which is yaw in the marker, which will
                    # be in the wrist frame Z, so yaw too.
                    o1 = self.initial_pose.orientation
                    o2 = self.last_pose.pose.orientation
                    r1, p1, y1 = euler_from_quaternion(
                        [o1.x, o1.y, o1.z, o1.w])
                    r2, p2, y2 = euler_from_quaternion(
                        [o2.x, o2.y, o2.z, o2.w])
                    # rospy.loginfo("Marker rpy: " + str( (r2, p2, y2) ))
                    # rospy.loginfo("initialrpy: " + str( (r1, p1, y1) ))
                    q = quaternion_from_euler(r1 + (r2 + 1.57), p1 + (-y2), y1 + p2)# y1 + y2)
                    goal_pose.pose.orientation = Quaternion(*q)


                self.pose_pub.publish(goal_pose)
                if self.send_head_goals:
                    self.head_pub.publish(goal_pose)
                    self.send_head_point_goal(goal_pose)
                if self.send_wrist_goals:
                    # Set offset for not covering the marker with the hand
                    # And not push the marker itself
                    goal_pose.pose.position.x -= 0.15
                    goal_pose.pose.position.z -= 0.1
                    self.wrist_pub.publish(goal_pose)
                # Put hand in pointing pose
                self.pointing_hand()

                self.last_goal_timestamp = rospy.Time.now()

    def sanitize(self, data, min_v, max_v):
        if data > max_v:
            return max_v
        if data < min_v:
            return min_v
        return data

    def dyn_rec_callback(self, config, level):
        """
        :param config:
        :param level:
        :return:
        """
        rospy.loginfo("Received reconf call: " + str(config))

        # Node Hz rate
        if self.rate != config['rate']:
            self.rate_changed = True
            self.rate = config['rate']

        self.send_head_goals = config['send_head_goals']
        self.send_wrist_goals = config['send_wrist_goals']

        self.use_marker_orientation = config['use_marker_orientation']

        # Limits
        self.min_x = config['min_x']
        self.max_x = config['max_x']
        self.min_y = config['min_y']
        self.max_y = config['max_y']
        self.min_z = config['min_z']
        self.max_z = config['max_z']

        self.initial_pose_x = config['initial_pose_x']
        self.initial_pose_y = config['initial_pose_y']
        self.initial_pose_z = config['initial_pose_z']
        self.initial_pose.position.x = self.initial_pose_x
        self.initial_pose.position.y = self.initial_pose_y
        self.initial_pose.position.z = self.initial_pose_z

        self.initial_pose_roll_degrees = config['initial_pose_roll_degrees']
        self.initial_pose_pitch_degrees = config['initial_pose_pitch_degrees']
        self.initial_pose_yaw_degrees = config['initial_pose_yaw_degrees']

        q = quaternion_from_euler(radians(self.initial_pose_roll_degrees),
                                  radians(self.initial_pose_pitch_degrees),
                                  radians(self.initial_pose_yaw_degrees))
        self.initial_pose.orientation = Quaternion(*q)

        self.initial_pose_head_x = config['initial_pose_head_x']
        self.initial_pose_head_y = config['initial_pose_head_y']
        self.initial_pose_head_z = config['initial_pose_head_z']

        if config['global_frame_id'][0] != '/':
            config['global_frame_id'] = '/' + config['global_frame_id']
        if config['global_frame_id'] != self.global_frame_id:
            self.global_frame_id = config['global_frame_id']

        if self.topic_name != config["topic_name"]:
            self.topic_name = config["topic_name"]
            self.follow_sub = rospy.Subscriber(self.topic_name,
                                               PoseStamped,
                                               self.follow_pose_cb,
                                               queue_size=1)

        self.goal_timeout = config['goal_timeout']

        return config

    def pointing_hand(self):
        jt = JointTrajectory()
        jt.joint_names = [
            'hand_right_thumb_joint', 'hand_right_index_joint', 'hand_right_mrl_joint']
        jtp = JointTrajectoryPoint()
        # Hardcoded joint limits
        jtp.positions = [5.5, 0.0, 6.0]
        jtp.time_from_start = rospy.Duration(0.5)
        jt.points.append(jtp)

        # Send goal
        self.hand_pub.publish(jt)

    def open_hand(self):
        jt = JointTrajectory()
        jt.joint_names = [
            'hand_right_thumb_joint', 'hand_right_index_joint', 'hand_right_mrl_joint']
        jtp = JointTrajectoryPoint()
        # Hardcoded joint limits
        jtp.positions = [0.0, 0.0, 0.0]
        jtp.time_from_start = rospy.Duration(0.5)
        jt.points.append(jtp)

        # Send goal
        self.hand_pub.publish(jt)


    def thumbs_up_hand(self):
        jt = JointTrajectory()
        jt.joint_names = [
            'hand_right_thumb_joint', 'hand_right_index_joint', 'hand_right_mrl_joint']
        jtp = JointTrajectoryPoint()
        # Hardcoded joint limits
        jtp.positions = [0.0, 5.5, 6.0]
        jtp.time_from_start = rospy.Duration(0.5)
        jt.points.append(jtp)

        # Send goal
        self.hand_pub.publish(jt)

if __name__ == '__main__':
    rospy.init_node('aruco_filter')
    af = ArucoFilter()
