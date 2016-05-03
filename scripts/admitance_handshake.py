#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 02/05/16
@author: sampfeiffer
"""
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import WrenchStamped, PoseStamped, Quaternion, Pose
from tf.transformations import quaternion_from_euler
from math import radians
from copy import deepcopy


class AdmitanceHandshake(object):
    def __init__(self):
        rospy.loginfo("Initializing AdmitanceHandshake")
        self.activate_pub = rospy.Publisher(
            '/activate_hand_by_ft', Empty, queue_size=1)
        self.deactivate_pub = rospy.Publisher(
            '/deactivate_hand_by_ft', Empty, queue_size=1)

        self.min_avg_force_to_trigger = rospy.get_param(
            '~min_avg_force_to_trigger', 15.0)
        self.handshake_poses_time_step = rospy.get_param(
            '~handshake_poses_time_step', 3.0)
        self.initial_pose = Pose()
        # REEMC values
        # self.initial_pose.position.x = 0.25
        # self.initial_pose.position.y = -0.271
        # self.initial_pose.position.z = 0.23
        # TIAGO values
        self.initial_pose.position.x = rospy.get_param('~initial_pose_x', 0.4)
        self.initial_pose.position.y = rospy.get_param(
            '~initial_pose_y', -0.2)
        self.initial_pose.position.z = rospy.get_param('~initial_pose_z', 1.1)

        # REEMC
        #self.global_frame_id = '/world'
        # TIAGO
        self.global_frame_id = rospy.get_param(
            '~global_frame_id', 'base_footprint')

        self.initial_pose_roll_degrees = rospy.get_param(
            '~initial_pose_roll_degrees', 0.0)
        self.initial_pose_pitch_degrees = rospy.get_param(
            '~initial_pose_pitch_degrees', 21.0)
        self.initial_pose_yaw_degrees = rospy.get_param(
            '~initial_pose_yaw_degrees', 17.0)

        q = quaternion_from_euler(radians(self.initial_pose_roll_degrees),
                                  radians(self.initial_pose_pitch_degrees),
                                  radians(self.initial_pose_yaw_degrees))  # hand a bit inclined forward and inside
        self.initial_pose.orientation = Quaternion(*q)

        self.last_wrench = None
        self.wrench_topic = rospy.get_param('~wrench_topic', '/right_wrist_ft')
        self.wrench_sub = rospy.Subscriber(self.wrench_topic,
                                           WrenchStamped,
                                           self.wrench_cb,
                                           queue_size=1)
        rospy.loginfo("Listening to wrench at: " + str(self.wrench_sub.resolved_name))

        self.executing_handshake = False
        self.trigger_sub = rospy.Subscriber('/admitance_handshake_trigger',
                                            Empty,
                                            self.trigger_cb,
                                            queue_size=1)

        self.pose_to_follow_topic = rospy.get_param('~pose_to_follow_topic', '/pose_to_follow')
        self.follow_pose_pub = rospy.Publisher(self.pose_to_follow_topic,
                                               PoseStamped,
                                               queue_size=1)

        self.num_times_shake = rospy.get_param('~num_times_shake', 3)
        self.shake_amount_z = rospy.get_param('~shake_amount_z', 0.15)

        rospy.loginfo("Node initialized.")

    def wrench_cb(self, data):
        self.last_wrench = data

    def trigger_cb(self, data):
        if not self.executing_handshake:
            self.executing_handshake = True

    def avg_last_force(self):
        f = self.last_wrench.wrench.force
        return abs(f.x) + abs(f.y) + abs(f.z)

    def wait_for_disturbance(self):
        rospy.loginfo("Waiting for disturbance...")
        # wait for disturbance
        while not rospy.is_shutdown() and self.avg_last_force() < self.min_avg_force_to_trigger:
            rospy.sleep(0.1)

        rospy.loginfo("Disturbance found! Activating finger interaction.")
        self.activate_fingers_interaction()

    def do_handshake(self):
        rospy.loginfo("Doing handshake")
        for i in range(self.num_times_shake):
            # go down and wait
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = self.global_frame_id

            ps.pose = deepcopy(self.initial_pose)
            ps.pose.position.z += -self.shake_amount_z / 2.0

            self.follow_pose_pub.publish(ps)
            rospy.sleep(self.handshake_poses_time_step / 2.0)

            # go up and wait
            rospy.loginfo("Going up for s")
            ps.header.stamp = rospy.Time.now()
            ps.pose = deepcopy(self.initial_pose)
            ps.pose.position.z += self.shake_amount_z / 2.0

            self.follow_pose_pub.publish(ps)
            rospy.sleep(self.handshake_poses_time_step)

        # go to initial pose and wait
        rospy.loginfo("Going back to initial but more back for s")
        ps.header.stamp = rospy.Time.now()
        ps.pose = deepcopy(self.initial_pose)
        ps.pose.position.x += -0.10

        self.follow_pose_pub.publish(ps)
        rospy.sleep(self.handshake_poses_time_step)

        self.deactivate_fingers_interaction()

    def activate_fingers_interaction(self):
        self.activate_pub.publish(Empty())

    def deactivate_fingers_interaction(self):
        self.deactivate_pub.publish(Empty())

    def go_to_initial_pose(self):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.global_frame_id
        ps.pose = deepcopy(self.initial_pose)
        self.follow_pose_pub.publish(ps)

    def run(self):
        while not rospy.is_shutdown():
            if self.executing_handshake:
                self.go_to_initial_pose()
                self.wait_for_disturbance()
                self.do_handshake()
                self.executing_handshake = False
            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('admitance_handshake')
    ah = AdmitanceHandshake()
    ah.run()
