#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy


class TeleopJoyNode:
    """Node that transform joy messages to twist messages for robot control"""
    def __init__(self):
        self.linear_axis = rospy.get_param("linear_axis", 1)
        self.angular_axis = rospy.get_param("angular_axis", 0)
        self.linear_scale = rospy.get_param("linear_scale", 1)
        self.angular_scale = rospy.get_param("angular_scale", 1)

        self.twist_topic = rospy.get_param("~twist_topic", "cmd_vel")
        self.joy_topic = rospy.get_param("~joy_topic", "joy")
        self.rate = rospy.get_param("~rate", 10.0)
        self.is_stamped = rospy.get_param("~is_stamped", False)
        if self.is_stamped:
            self.twist_cls = TwistStamped
            self.frame_id = rospy.get_param("~frame_id", "base_link")
        else:
            self.twist_cls = Twist

        self.twist_msg = self.twist_cls()

        self.twist_publisher = rospy.Publisher(self.twist_topic, self.twist_cls, queue_size=10)
        self.joy_subscriber = rospy.Subscriber(self.joy_topic, Joy, self.joy_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_callback)

    def publish_callback(self, event):
        rospy.loginfo("Publishing twist!")
        self.twist_publisher.publish(self.twist_msg)

    def joy_callback(self, msg: Joy):
        self.twist_msg.angular.z = self.angular_scale * msg.axes[self.angular_axis]
        self.twist_msg.linear.x = self.linear_scale * msg.axes[self.linear_axis]


def main():
    rospy.init_node("joy_teleop")
    teleop_joy = TeleopJoyNode()

    rospy.spin()


if __name__ == '__main__':
    main()
