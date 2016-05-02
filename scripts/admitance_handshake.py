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

        self.min_avg_force_to_trigger = 15.0
        self.handshake_poses_time_step = 3.0

        self.initial_pose = Pose()
        self.initial_pose.position.x = 0.25
        self.initial_pose.position.y = -0.271
        self.initial_pose.position.z = 0.23
        q = quaternion_from_euler(radians(0.0), radians(21.0), radians(17.0))  # hand a bit inclined forward and inside
        self.initial_pose.orientation = Quaternion(*q)


        self.last_wrench = None
        self.wrench_sub = rospy.Subscriber('/right_wrist_ft',
                                           WrenchStamped,
                                           self.wrench_cb,
                                           queue_size=1)

        self.executing_handshake = False
        self.trigger_sub = rospy.Subscriber('/admitance_handshake_trigger',
                                            Empty,
                                            self.trigger_cb,
                                            queue_size=1)

        self.follow_pose_pub = rospy.Publisher('/pose_to_follow',
                                               PoseStamped,
                                               queue_size=1)

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
        # go down and wait
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = '/world'

        ps.pose = deepcopy(self.initial_pose)
        ps.pose.position.z += -0.05

        self.follow_pose_pub.publish(ps)
        rospy.sleep(self.handshake_poses_time_step / 2.0)

        # go up and wait
        rospy.loginfo("Going up for s")
        ps.header.stamp = rospy.Time.now()
        ps.pose = deepcopy(self.initial_pose)
        ps.pose.position.z += 0.05

        self.follow_pose_pub.publish(ps)
        rospy.sleep(self.handshake_poses_time_step)

        # go down and wait
        rospy.loginfo("Going down for s")
        ps.header.stamp = rospy.Time.now()
        ps.pose = deepcopy(self.initial_pose)
        ps.pose.position.z += -0.05

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
        ps.header.frame_id = '/world'
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
