#! /usr/bin/env python

from copy import deepcopy
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HandByFT(object):
    def __init__(self):
        self.last_ft_msg = None
        # First message will be used as offset of the force torque sensor for what is considered 0
        self.initial_ft_msg = None
        self.ft_sub = rospy.Subscriber('/right_wrist_ft', WrenchStamped, self.ft_cb, queue_size=1)
        self.hand_pub = rospy.Publisher('/right_hand_controller/command', JointTrajectory, queue_size=1)
        self.active = False
        self.activate_sub = rospy.Subscriber('/activate_hand_by_ft', Empty, self.activate_cb, queue_size=1)
        self.deactivate_sub = rospy.Subscriber('/deactivate_hand_by_ft', Empty, self.deactivate_cb, queue_size=1)
        # Some meaningful values for ignoring noise and not needing to do too much force on the wrist to close it
        self.min_force_amount = 15.0
        self.max_force_amount = 50.0

    def activate_cb(self, data):
        self.active = True
        self.open_hand()

    def deactivate_cb(self, data):
        self.active = False
        self.open_hand()

    def ft_cb(self, data):
        self.last_ft_msg = data

    def wait_for_first_ft(self):
        rospy.loginfo('Waiting for FT data...')
        while not rospy.is_shutdown() and self.last_ft_msg is None:
            rospy.sleep(0.5)
        self.initial_ft_msg = deepcopy(self.last_ft_msg)
        rospy.loginfo('Got initial FT value (used as offset)')

    def run(self):
        # Initialize getting an offset (as the FT will report != 0's)
        self.wait_for_first_ft()
        # Send goals at 5Hz
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.active:
                self.command_hand()
            r.sleep()

    def close_amount_by_ft(self):
        # Simple computation of a unit to decide how much to close
        x = self.last_ft_msg.wrench.force.x - self.initial_ft_msg.wrench.force.x
        y = self.last_ft_msg.wrench.force.y - self.initial_ft_msg.wrench.force.y
        z = self.last_ft_msg.wrench.force.z - self.initial_ft_msg.wrench.force.z
        amount = abs(x) + abs(y) + abs(z)
        if amount > self.min_force_amount and amount < self.max_force_amount:
            return amount / self.max_force_amount
        elif amount <= self.min_force_amount:
            return 0.0
        elif amount >= self.max_force_amount:
            return 1.0
            

    def command_hand(self):
        # Compute how much to close
        amount = self.close_amount_by_ft()
        rospy.loginfo("Closing hand amount: " + str(amount * 100) + "%")
        # Create a goal based on that amount
        goal = self.create_hand_goal(amount)
        # Send goal
        self.hand_pub.publish(goal)
               

    def create_hand_goal(self, closing_amount):
        jt = JointTrajectory()
        jt.joint_names = ['hand_right_thumb_joint', 'hand_right_index_joint', 'hand_right_mrl_joint']
        jtp = JointTrajectoryPoint()
        # Hardcoded joint limits
        jtp.positions = [closing_amount * 6.0, closing_amount * 6.0, closing_amount * 9.0]
        jtp.time_from_start = rospy.Duration(0.5)
        jt.points.append(jtp)
        return jt
 
    def close_hand(self):
        # Create a goal based on that amount
        goal = self.create_hand_goal(1.0)
        # Send goal
        self.hand_pub.publish(goal)

    def open_hand(self):
        rospy.loginfo("Opening hand")
        # Create a goal based on that amount
        goal = self.create_hand_goal(0.0)
        # Send goal
        self.hand_pub.publish(goal)

        
if __name__ == '__main__':
    rospy.init_node('handbyft')
    hbft = HandByFT()
    hbft.run()