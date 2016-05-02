#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, QuaternionStamped
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformListener, ExtrapolationException
from math import cos, degrees, radians

class ForceFromGravity(object):
    def __init__(self):
        self.tf_l = TransformListener()
        self.last_wrench = None
        self.wrench_sub = rospy.Subscriber('/left_wrist_ft',
                                           WrenchStamped,
                                           self.wrench_cb,
                                           queue_size=1)
    def wrench_cb(self, data):
        self.last_wrench = data

    def compute_forces(self):
        qs = self.get_orientation()
        o = qs.quaternion
        r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
        rospy.loginfo("wrist_left_ft_link rpy vs world: " + str( (round(r, 3), round(p, 3), round(y, 3)) ))
        rospy.loginfo("in degrees: " + str( (round(degrees(r), 3), round(degrees(p), 3), round(degrees(y), 3)) ))
        hand_total_force = 10 # 10 Newton, near to a Kg
        fx = hand_total_force * cos(r) * -1.0 # hack
        fy = hand_total_force * cos(p)
        fz = hand_total_force * cos(y)
        #total = abs(fx) + abs(fy) + abs(fz)
        #rospy.loginfo("fx, fy, fz, total:")
        #rospy.loginfo( str( (round(fx, 3), round(fy, 3), round(fz, 3), round(total, 3)) ) )
        return fx, fy, fz

    def get_last_forces(self):
        f = self.last_wrench.wrench.force
        return f.x, f.y, f.z

    def get_orientation(self, from_frame="wrist_left_ft_link"):
        qs = QuaternionStamped()
        qs.quaternion.w = 1.0
        qs.header.stamp = self.tf_l.getLatestCommonTime("world", from_frame)
        qs.header.frame_id = "/" + from_frame
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                world_q = self.tf_l.transformQuaternion("/world", qs)
                transform_ok = True
                return world_q
            except ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming pose... trying again \n(" + str(e) + ")")
                rospy.sleep(0.05)
                qs.header.stamp = self.tf_l.getLatestCommonTime(
                    "world", from_frame)

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            cx, cy, cz = self.compute_forces()
            c_total = abs(cx) + abs(cy) + abs(cz)
            fx, fy, fz = self.get_last_forces()
            f_total = abs(fx) + abs(fy) + abs(fz)
            rospy.loginfo("Computed Forces:" + str(round(c_total, 3)) + "\n" + str( (round(cx, 3), round(cy, 3), round(cz, 3) )))
            rospy.loginfo("Real Forces    :" + str(round(f_total, 3)) + "\n" + str( (round(fx, 3), round(fy, 3), round(fz, 3) )))
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('forcefromgravity')
    ffg = ForceFromGravity()
    rospy.sleep(3.0)
    ffg.run()
