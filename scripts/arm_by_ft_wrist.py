#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 22/01/16
@author: sampfeiffer
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import WrenchStamped


class ArmByFtWrist(object):
    def __init__(self):
        self.initial_pose = self.get_initial_pose()
        self.limit_min_x = 0.4
        self.limit_max_x = 0.8
        self.limit_min_y = -0.4
        self.limit_max_y = 0.4
        self.limit_min_z = 0.6
        self.limit_max_z = 1.2
        self.min_force_change = 2.0
        self.force_to_dist_divisor = 500.0 # 1000.0
        self.current_pose = self.initial_pose
        self.pose_pub = rospy.Publisher('/joystick_pose_goal',
                                        PoseStamped,
                                        queue_size=1)
        self.last_wrench = None
        self.wrench_sub = rospy.Subscriber('/wrist_ft',
                                           WrenchStamped,
                                           self.wrench_cb,
                                           queue_size=1)
        self.offset = None
        self.learn_offset()

    def wrench_cb(self, data):
        self.last_wrench = data

    def get_initial_pose(self):
        ps = PoseStamped()
        ps.header.frame_id = 'base_footprint'
        ps.pose.position.x = 0.5
        ps.pose.position.y = -0.3
        ps.pose.position.z = 1.0

        ps.pose.orientation.w = 1.0
        return ps

    def learn_offset(self):
        rospy.loginfo("Going to learn offset... Don't touch the end effector")
        while not rospy.is_shutdown() and self.last_wrench is None:
            rospy.sleep(0.5)

        # TODO: actually do something smart
        self.offset = self.last_wrench

        rospy.loginfo("Offset learnt!")

    def run(self):
        r = rospy.Rate(4)
        while not rospy.is_shutdown():
            self.move_towards_force()
            r.sleep()

    def move_towards_force(self):
        fx, fy, fz = self.get_force_movement()
        rospy.loginfo("fx, fy, fz: " + str((fx, fy, fz)))
        if abs(fx) > self.min_force_change:
            self.current_pose.pose.position.z += (fx / self.force_to_dist_divisor) * -1.0
            self.current_pose.pose.position.z = self.sanitize(self.current_pose.pose.position.z,
                                                              self.limit_min_z,
                                                              self.limit_max_z)
        if abs(fy) > self.min_force_change:
            self.current_pose.pose.position.y += (fy / self.force_to_dist_divisor) * -1.0
            self.current_pose.pose.position.y = self.sanitize(self.current_pose.pose.position.y,
                                                              self.limit_min_y,
                                                              self.limit_max_y)
        if abs(fz) > self.min_force_change:
            self.current_pose.pose.position.x += (fz / self.force_to_dist_divisor) * -1.0
            self.current_pose.pose.position.x = self.sanitize(self.current_pose.pose.position.x,
                                                              self.limit_min_x,
                                                              self.limit_max_x)
        rospy.loginfo("Pose:\n" + str(self.current_pose.pose))
        self.pose_pub.publish(self.current_pose)

    def get_force_movement(self):
        f_x = self.last_wrench.wrench.force.x
        f_y = self.last_wrench.wrench.force.y
        f_z = self.last_wrench.wrench.force.z

        off_x = self.offset.wrench.force.x
        off_y = self.offset.wrench.force.y
        off_z = self.offset.wrench.force.z

        real_x = f_x - off_x
        real_y = f_y - off_y
        real_z = f_z - off_z
        return real_x, real_y, real_z

    def sanitize(self, data, min_v, max_v):
        if data > max_v:
            return max_v
        if data < min_v:
            return min_v
        return data

if __name__ == '__main__':
    rospy.init_node('arm_by_ft_wrist')
    abftw = ArmByFtWrist()
    abftw.run()
