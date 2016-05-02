#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, QuaternionStamped
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformListener, ExtrapolationException
from math import cos, degrees, radians

class AvgForce(object):
    def __init__(self):
        self.tf_l = TransformListener()
        self.last_wrench = None
        self.wrench_sub = rospy.Subscriber('/left_wrist_ft',
                                           WrenchStamped,
                                           self.wrench_cb,
                                           queue_size=1)
    def wrench_cb(self, data):
        self.last_wrench = data

    def get_last_forces(self):
        f = self.last_wrench.wrench.force
        return f.x, f.y, f.z


    def run(self):
        while self.last_wrench is None:
            rospy.sleep(0.2)
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            fx, fy, fz = self.get_last_forces()
            f_total = abs(fx) + abs(fy) + abs(fz)
            total = fx + fy + fz
            rospy.loginfo("Real Forces, abs total, total, x y z:")
            rospy.loginfo(str(round(f_total, 3)) + " " + str(round(total, 3)) +  "\n" + str( (round(fx, 3), round(fy, 3), round(fz, 3) )))
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('avgforce')
    ffg = AvgForce()
    ffg.run()
