#!/usr/bin/env python3

from dataclasses import dataclass, asdict
import math
import time
import rospy
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


@dataclass
class SerialCommand:
    linear: float
    angular: float
    base: float
    elbow: float
    magnet: bool
    alarm: bool


@dataclass
class RobotState:
    mine: bool


class RobotControlNode:
    """Node for controlling minesweeper robot using the serial port"""
    def __init__(self):
        self.twist_topic = rospy.get_param("twist_topic", "cmd_vel")
        self.joy_topic = rospy.get_param("joy_topic", "joy")
        self.pico_port = rospy.get_param("pico_port", "/dev/ttyACM0")
        self.up_button = rospy.get_param("up_button", 2)
        self.down_button = rospy.get_param("down_button", 0)
        self.left_button = rospy.get_param("left_button", 3)
        self.right_button = rospy.get_param("right_button", 1)
        self.magnet_button = rospy.get_param("magnet_button", 5)
        self.alarm_button = rospy.get_param("alarm_button", 4)
        self.arm_vel = 0.5

        self.twist_subscriber = rospy.Subscriber(self.twist_topic, Twist, self.twist_callback, queue_size=10)
        self.joy_subscriber = rospy.Subscriber(self.joy_topic, Joy, self.joy_callback, queue_size=10)

        self.serial_conn = serial.Serial(self.pico_port)
        rospy.loginfo(f"using serial port {self.serial_conn.name}")
        self.command = SerialCommand(linear=0, angular=0, base=0, elbow=0, magnet=False, alarm=False)

    def joy_callback(self, msg: Joy):
        self.command.base = self.arm_vel * (msg.buttons[self.left_button] - msg.buttons[self.right_button])
        self.command.elbow = self.arm_vel * (msg.buttons[self.up_button] - msg.buttons[self.down_button])
        self.command.magnet = bool(msg.buttons[self.magnet_button])
        self.command.alarm = bool(msg.buttons[self.alarm_button])

    def twist_callback(self, msg: Twist):
        self.command.linear = msg.linear.x
        self.command.angular = msg.angular.z
        self.send_command(self.command)

    def send_command(self, command: SerialCommand) -> RobotState:
        rospy.loginfo(f"Data to send: {asdict(command)}")
        command = f"{command.linear:.3f},{command.angular:.3f},{command.base:.3f},{command.elbow:.3f},{int(command.magnet)},{int(command.alarm)}/".encode("UTF-8")
        rospy.loginfo(f"Sending command: '{command}'")
        self.serial_conn.write(command)
        while self.serial_conn.in_waiting == 0:
            pass

        res = self.serial_conn.read(self.serial_conn.in_waiting).decode("UTF-8")
        rospy.loginfo(f"bytes: {len(res)}; data: '{res}'")

        if res == "0" or len(res) < 79 or len(res) > (79 + 13):
            rospy.logwarn(f"Bad data1: '{res}'")
            return None

        raw_list = res.strip().split(",")

        try:
            values_list = [float(value) for value in raw_list]
        except ValueError as e:
            rospy.logwarn(f"Bad data2: '{res}'")
            return None

        return RobotState(*values_list)


def main():
    rospy.init_node("robot_control")
    robot_control = RobotControlNode()

    rospy.spin()


if __name__ == '__main__':
    main()
