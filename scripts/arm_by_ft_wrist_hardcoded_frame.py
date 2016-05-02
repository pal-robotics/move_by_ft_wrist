#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 26/04/16
@author: sampfeiffer
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformListener, ExtrapolationException
from copy import deepcopy

from dynamic_reconfigure.server import Server
from move_by_ft_wrist.cfg import HandshakeConfig


class WrenchFixer(object):
    """Just a class to more comfortably deal with wrench sign
    multiplier (*-1.0 when needed, or stay as 1.0)
    and to switch frames, when the force torque X axis does
    not correspond with the X axis of the link you wanna move"""

    def __init__(self, fx, fy, fz,
                 tx, ty, tz,
                 x_axis='x', y_axis='y', z_axis='z',
                 roll_axis='x', pitch_axis='y', yaw_axis='z'):
        self.fx = fx
        self.fy = fy
        self.fz = fz
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.roll_axis = roll_axis
        self.pitch_axis = pitch_axis
        self.yaw_axis = yaw_axis


class ArmByFtWrist(object):
    def __init__(self):
        rospy.loginfo("Initializing...")
        # Node Hz rate
        self.rate = 10
        self.rate_changed = False
        self.send_goals = False

        # Limits
        self.min_x = 0.0
        self.max_x = 0.6
        self.min_y = -0.5
        self.max_y = -0.05
        self.min_z = -0.2
        self.max_z = 0.2

        # Force stuff
        self.fx_scaling = 0.0
        self.fy_scaling = 0.0
        self.fz_scaling = 0.0

        self.fx_deadband = 0.0
        self.fy_deadband = 0.0
        self.fz_deadband = 0.0

        # Torque stuff
        self.tx_scaling = 0.0
        self.ty_scaling = 0.0
        self.tz_scaling = 0.0

        self.tx_deadband = 0.0
        self.ty_deadband = 0.0
        self.tz_deadband = 0.0

        self.axis_force = "zxy"
        self.sign_force = "+++"
        self.axis_torque = "zxy"
        self.sign_torque = "+++"
        # Signs adjusting fx, fy, fz, tx(roll), ty(pitch), tz(yaw)
        # for the left hand of reemc right now
        # And axis flipping
        self.frame_fixer = WrenchFixer(1.0, 1.0, 1.0,
                                       1.0, 1.0, 1.0,
                                       'z', 'x', 'y',
                                       'z', 'x', 'y')

        self.goal_frame_id = "arm_right_tool_link"

        self.dyn_rec_srv = Server(HandshakeConfig, self.dyn_rec_callback)

        # Goal to send to WBC
        self.tf_l = TransformListener()
        rospy.sleep(3.0)  # Let the subscriber to its job
        self.initial_pose = self.get_initial_pose()
        self.tf_l = None  # Let the garbage collector kill it
        self.current_pose = self.initial_pose
        self.last_pose_to_follow = deepcopy(self.current_pose)
        if self.goal_frame_id[0] == '/':
            self.goal_frame_id = self.goal_frame_id[1:]
        self.pose_pub = rospy.Publisher('/whole_body_kinematic_controler/' + self.goal_frame_id + '_goal',
                                        PoseStamped,
                                        queue_size=1)

        # Safe debugging goal
        self.debug_pose_pub = rospy.Publisher('/force_torqued_pose',
                                              PoseStamped,
                                              queue_size=1)

        # Goal to follow
        self.follow_sub = rospy.Subscriber('/pose_to_follow',
                                           PoseStamped,
                                           self.follow_pose_cb,
                                           queue_size=1)

        # Sensor input
        self.last_wrench = None
        self.wrench_sub = rospy.Subscriber('/right_wrist_ft',
                                           WrenchStamped,
                                           self.wrench_cb,
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

        self.send_goals = config['send_goals']

        # Limits
        self.min_x = config['min_x']
        self.max_x = config['max_x']
        self.min_y = config['min_y']
        self.max_y = config['max_y']
        self.min_z = config['min_z']
        self.max_z = config['max_z']

        # Force stuff
        self.fx_scaling = config['fx_scaling']
        self.fy_scaling = config['fy_scaling']
        self.fz_scaling = config['fz_scaling']

        self.fx_deadband = config['fx_deadband']
        self.fy_deadband = config['fy_deadband']
        self.fz_deadband = config['fz_deadband']

        # Torque stuff
        self.tx_scaling = config['tx_scaling']
        self.ty_scaling = config['ty_scaling']
        self.tz_scaling = config['tz_scaling']

        self.tx_deadband = config['tx_deadband']
        self.ty_deadband = config['ty_deadband']
        self.tz_deadband = config['tz_deadband']

        # Fixing transform stuff
        self.axis_force = config['axis_force']
        self.sign_force = config['sign_force']
        self.axis_torque = config['axis_torque']
        self.sign_torque = config['sign_torque']

        if config['goal_frame_id'][0] == '/':
            config['goal_frame_id'] = config['goal_frame_id'][1:]
        if config['goal_frame_id'] != self.goal_frame_id:
            self.goal_frame_id = config['goal_frame_id']
            self.pose_pub = rospy.Publisher('/whole_body_kinematic_controler/' + self.goal_frame_id + '_goal',
                                            PoseStamped,
                                            queue_size=1)

        args = []
        for sign in self.sign_force:
            if sign == '+':
                args.append(1.0)
            else:
                args.append(-1.0)

        for sign in self.sign_torque:
            if sign == '+':
                args.append(1.0)
            else:
                args.append(-1.0)

        args.extend([self.axis_force[0], self.axis_force[1], self.axis_force[2],
                     self.axis_torque[0], self.axis_torque[1], self.axis_torque[2]])

        self.frame_fixer = WrenchFixer(*args)

        return config

    def follow_pose_cb(self, data):
        if data.header.frame_id != '/world' and data.header.frame_id != 'world':
            rospy.logwarn(
                "Poses to follow should be in frame /world, transforming into /world...")
            world_ps = self.transform_pose_to_world(
                data.pose, from_frame=data.header.frame_id)
            ps = PoseStamped()
            ps.header.stamp = data.header.stamp
            ps.header.frame_id = '/world'
            ps.pose = world_ps
            self.last_pose_to_follow = ps
        else:
            self.last_pose_to_follow = data

    def transform_pose_to_world(self, pose, from_frame="arm_right_tool_link"):
        ps = PoseStamped()
        ps.header.stamp = self.tf_l.getLatestCommonTime("world", from_frame)
        ps.header.frame_id = "/" + from_frame
        # TODO: check pose being PoseStamped or Pose
        ps.pose = pose
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                world_ps = self.tf_l.transformPose("/world", ps)
                transform_ok = True
                return world_ps
            except ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming pose... trying again \n(" + str(e) + ")")
                rospy.sleep(0.05)
                ps.header.stamp = self.tf_l.getLatestCommonTime(
                    "world", from_frame)

    def wrench_cb(self, data):
        self.last_wrench = data

    def get_initial_pose(self):
        zero_pose = Pose()
        zero_pose.orientation.w = 1.0

        current_pose = self.transform_pose_to_world(
            zero_pose, from_frame=self.goal_frame_id)
        return current_pose

    def get_abs_total_force(self, wrench_msg):
        f = wrench_msg.wrench.force
        return abs(f.x) + abs(f.y) + abs(f.z)

    def get_abs_total_torque(self, wrench_msg):
        t = wrench_msg.wrench.torque
        return abs(t.x) + abs(t.y) + abs(t.z)

    def run(self):
        rospy.loginfo(
            "Waiting for first wrench message on: " + str(self.wrench_sub.resolved_name))
        while not rospy.is_shutdown() and self.last_wrench is None:
            rospy.sleep(0.2)
        r = rospy.Rate(self.rate)
        rospy.loginfo("Node running!")
        while not rospy.is_shutdown():
            # To change the node rate
            if self.rate_changed:
                r = rospy.Rate(self.rate)
                self.rate_changed = False
            self.follow_pose_with_admitance_by_ft()
            r.sleep()

    def follow_pose_with_admitance_by_ft(self):
        fx, fy, fz = self.get_force_movement()
        rospy.loginfo(
            "fx, fy, fz: " + str((round(fx, 3), round(fy, 3), round(fz, 3))))
        ps = Pose()
        if abs(fx) > self.fx_deadband:
            ps.position.x = (fx * self.fx_scaling) * self.frame_fixer.fx
        if abs(fy) > self.fy_deadband:
            ps.position.y = (fy * self.fy_scaling) * self.frame_fixer.fy
        if abs(fz) > self.fz_deadband:
            ps.position.z = (fz * self.fz_scaling) * self.frame_fixer.fz

        tx, ty, tz = self.get_torque_movement()
        rospy.loginfo(
            "tx, ty, tz: " + str((round(tx, 3), round(ty, 3), round(tz, 3))))

        roll = pitch = yaw = 0.0
        if abs(tx) > self.tx_deadband:
            roll += (tx * self.tx_scaling) * self.frame_fixer.tx
        if abs(ty) > self.ty_deadband:
            pitch += (ty * self.ty_scaling) * self.frame_fixer.ty
        if abs(tz) > self.tz_deadband:
            yaw += (tz * self.tz_scaling) * self.frame_fixer.tz

        q = quaternion_from_euler(roll, pitch, yaw)
        ps.orientation = Quaternion(*q)

        o = self.last_pose_to_follow.pose.orientation
        r_lastp, p_lastp, y_lastp = euler_from_quaternion([o.x, o.y, o.z, o.w])

        final_roll = r_lastp + roll
        final_pitch = p_lastp + pitch
        final_yaw = y_lastp + yaw
        self.current_pose.pose.orientation = Quaternion(*quaternion_from_euler(final_roll,
                                                                               final_pitch,
                                                                               final_yaw))

        self.current_pose.pose.position.x = self.last_pose_to_follow.pose.position.x + \
            ps.position.x
        self.current_pose.pose.position.y = self.last_pose_to_follow.pose.position.y + \
            ps.position.y
        self.current_pose.pose.position.z = self.last_pose_to_follow.pose.position.z + \
            ps.position.z

        self.current_pose.pose.position.x = self.sanitize(self.current_pose.pose.position.x,
                                                          self.min_x,
                                                          self.max_x)
        self.current_pose.pose.position.y = self.sanitize(self.current_pose.pose.position.y,
                                                          self.min_y,
                                                          self.max_y)
        self.current_pose.pose.position.z = self.sanitize(self.current_pose.pose.position.z,
                                                          self.min_z,
                                                          self.max_z)
        self.current_pose.header.stamp = rospy.Time.now()
        if self.send_goals: # send MODIFIED GOALS
            self.pose_pub.publish(self.current_pose)
        else:
            self.last_pose_to_follow.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.last_pose_to_follow)
        self.debug_pose_pub.publish(self.current_pose)

    def get_force_movement(self):
        f_x = self.last_wrench.wrench.force.__getattribute__(
            self.frame_fixer.x_axis)
        f_y = self.last_wrench.wrench.force.__getattribute__(
            self.frame_fixer.y_axis)
        f_z = self.last_wrench.wrench.force.__getattribute__(
            self.frame_fixer.z_axis)
        return f_x, f_y, f_z

    def get_torque_movement(self):
        t_x = self.last_wrench.wrench.torque.__getattribute__(
            self.frame_fixer.roll_axis)
        t_y = self.last_wrench.wrench.torque.__getattribute__(
            self.frame_fixer.pitch_axis)
        t_z = self.last_wrench.wrench.torque.__getattribute__(
            self.frame_fixer.yaw_axis)
        return t_x, t_y, t_z

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
