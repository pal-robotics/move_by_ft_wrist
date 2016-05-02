#! /usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import WrenchStamped, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians


class WrenchMarker(object):
    def __init__(self, wrench_topic, rate=None):
        # force can be [-120, 120] approx...
        # but pushing with real forces I haven't seen more than 60
        self.force_to_length = 120
        self.pub = rospy.Publisher(wrench_topic + "_marker_array",
                                   MarkerArray,
                                   queue_size=1)
        self.last_wrench = None
        self.sub = rospy.Subscriber(wrench_topic,
                                    WrenchStamped,
                                    self.wrench_cb,
                                    queue_size=1)
        rospy.loginfo("Subscribed to: '" + self.sub.resolved_name + "'.")
        rospy.loginfo("Publishing to: '" + self.pub.resolved_name + "'.")

        self.rate = None
        if rate is not None:
            self.rate = rate
            self.run()


    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.last_wrench is not None:
                ma = self.create_marker_array_from_wrench_stamped(self.last_wrench)
                self.pub.publish(ma)
            r.sleep()


    def wrench_cb(self, data):
        if self.rate is None:
            ma = self.create_marker_array_from_wrench_stamped(data)
            self.pub.publish(ma)
        else:
            self.last_wrench = data

    def create_marker_array_from_wrench_stamped(self, ws):
        ma = MarkerArray()
        # Create a cylinder for each force
        ma.markers.append(
            self.create_force_marker(ws.wrench.force.x, 'x', ws.header.frame_id))
        ma.markers.append(
            self.create_force_marker(ws.wrench.force.y, 'y', ws.header.frame_id))
        ma.markers.append(
            self.create_force_marker(ws.wrench.force.z, 'z', ws.header.frame_id))
        # TODO: figure how to represent nicely torques
        return ma

    def create_force_marker(self, force, axis, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.type = m.CYLINDER

        m.scale.x = 0.02  # diameter
        m.scale.y = 0.02  # also diameter
        m.scale.z = abs(force) / self.force_to_length  # length
        # MarkerArray visualizer in Rviz complains if a scale is 0
        if m.scale.z == 0.0:
            m.scale.z = 0.0001
        # namespaced so we can disable axis
        m.ns = axis
        if axis == 'x':
            m.id = 555
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.pose.position.x = m.scale.z / 2.0
            # Aligned with the X axis of the frame
            if force > 0:
                m.pose.orientation = Quaternion(
                    *quaternion_from_euler(0, radians(90), 0))
            else:  # if the force is negative the cylinder goes to the other side
                m.pose.orientation = Quaternion(
                    *quaternion_from_euler(0, radians(-90), 0))
                m.pose.position.x *= -1.0
        elif axis == 'y':
            m.id = 666
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.pose.position.y = m.scale.z / 2.0
            # Aligned with the Y axis of the frame
            if force > 0:
                m.pose.orientation = Quaternion(
                    *quaternion_from_euler(radians(90), 0, 0))
            else:
                m.pose.orientation = Quaternion(
                    *quaternion_from_euler(radians(-90), 0, 0))
                m.pose.position.y *= -1.0
        elif axis == 'z':
            m.id = 777
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.pose.position.z = m.scale.z / 2.0
            # Aligned with the Z axis of the frame
            if force > 0:
                m.pose.orientation = Quaternion(
                    *quaternion_from_euler(0, 0, 0))
            else:
                m.pose.orientation = Quaternion(
                    *quaternion_from_euler(radians(180), 0, 0))
                m.pose.position.z *= -1.0
        m.color.a = 1.0
        return m

if __name__ == '__main__':
    if len(sys.argv) == 2 or len(sys.argv) == 3:
        rospy.init_node('ft_marker', anonymous=True)
        if len(sys.argv) == 2:
            wm = WrenchMarker(sys.argv[1])
        else:
            wm = WrenchMarker(sys.argv[1], int(sys.argv[2]))
    else:
        print "Usage:"
        print sys.argv[0] + " /my_wrench_stamped_topic"
        exit(0)
    rospy.spin()
