#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 02/05/16
@author: sampfeiffer
"""

import rospy
from geometry_msgs.msg import WrenchStamped
from dynamic_reconfigure.server import Server
from move_by_ft_wrist.cfg import ForceTorqueConfig


class ForceTorquePublisher(object):
    def __init__(self):
        rospy.loginfo("Initializing...")
        # Node Hz rate
        self.rate = 10
        self.rate_changed = False

        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        self.frame_id = "arm_right_tool_link"
        self.topic_name = "/right_wrist_ft_fake"

        self.dyn_rec_srv = Server(ForceTorqueConfig, self.dyn_rec_callback)

        self.pub = rospy.Publisher(self.topic_name,
                                   WrenchStamped,
                                   queue_size=1)

        rospy.loginfo("Done initializing.")

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

        self.fx = config['fx']
        self.fy = config['fy']
        self.fz = config['fz']
        self.tx = config['tx']
        self.ty = config['ty']
        self.tz = config['tz']
        self.frame_id = config['frame_id']
        if config['topic_name'] != self.topic_name:
            self.topic_name = config['topic_name']
            self.pub = rospy.Publisher(self.topic_name,
                                       WrenchStamped,
                                       queue_size=1)

        return config

    def pub_ft(self):
        ws = WrenchStamped()
        ws.header.frame_id = self.frame_id
        ws.header.stamp = rospy.Time.now()

        ws.wrench.force.x = self.fx
        ws.wrench.force.y = self.fy
        ws.wrench.force.z = self.fz

        ws.wrench.torque.x = self.tx
        ws.wrench.torque.y = self.ty
        ws.wrench.torque.z = self.tz

        self.pub.publish(ws)

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.rate_changed:
                r = rospy.Rate(self.rate)
                self.rate_changed = False
            self.pub_ft()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('force_torque_publisher', anonymous=True)
    ft_pub = ForceTorquePublisher()
    ft_pub.run()
