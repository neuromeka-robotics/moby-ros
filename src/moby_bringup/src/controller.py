#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Author: Nguyen Pham
"""
import os
import yaml
import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time
import rospkg

class TimeCount:
    def __init__(self, count=3, timeout=0.5):
        self.count_out = count
        self.timeout = timeout
        self.clear()

    def clear(self):
        self.time_pre = time.time()
        self.count = 0

    def __call__(self):
        if time.time() - self.time_pre > self.timeout:
            self.clear()
        self.count += 1
        return self.count >= self.count_out

class Controller:
    MAX_SPEED = 0.8
    SPEED_CHANGE_STEP = 0.1

    def __init__(self):
        rospy.init_node('moby_controller', anonymous=True)

        rospack = rospkg.RosPack()
        joy_config_file = os.path.join(rospack.get_path('moby_bringup'), 'param', 'joy_config.yaml')
                
        with open(joy_config_file, 'r') as stream:
            stream.seek(0)
            joy_config = yaml.safe_load(stream)
        self.buttons = joy_config['buttons']
        self.axes = joy_config['axes']
        self.negatives = joy_config['negatives']
        
        # Set up subscriptions
        rospy.Subscriber('joy', Joy, self.joy_callback)

        # Set up publishers
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel = Twist()

        self.cam_pub = rospy.Publisher('agri_cam_vel', Twist, queue_size=10)
        self.cam_vel = Twist()

        self.stop_pub = rospy.Publisher('cmd_stop', Bool, queue_size=10)
        self.zero_pub = rospy.Publisher('cmd_set_zero', Bool, queue_size=10)
        self.zero_time = None
        self.zero_count = TimeCount(count=3, timeout=0.5)

        self.speed_level = self.SPEED_CHANGE_STEP
        self.speed_up = False
        self.speed_down = False

        # 2 types of robot
        self.moby_type = rospy.get_param('~moby_type', 'moby_rp')

        # - To move: Press **L2 + Left joystick** for moving (non-holonomic)
        # - To move: Press **L2 + Right joystick** for moving (holonomic)
        # - To change speed: Press **R, R2** to change speed. Maximum 0.8 m/s (linear), 0.8 rad/s (angular)
        rospy.loginfo('Press L2 + Right joystick for moving, turning (holonomic)')
        rospy.loginfo(f'Press Y or B for speed level, maximum {self.MAX_SPEED}m/s (linear), '
                      f'{self.MAX_SPEED}rad/s (angular)')

    def get_value(self, msg, key):
        if key in self.buttons:
            value = msg.buttons[self.buttons[key]]
        elif key in self.axes:
            value = msg.axes[self.axes[key]]
        else:
            value = 0
        if key in self.negatives:
            return -value
        else:
            return value

    def joy_callback(self, msg):
        A, B = self.get_value(msg, 'A'), self.get_value(msg, 'B')
        X, Y = self.get_value(msg, 'X'), self.get_value(msg, 'Y')
        if A and B and X and Y:  # L - SET ZERO
            if self.zero_count():
                self.zero_pub.publish(Bool(data=True))
        else:
            self.zero_count.clear()

        L, L2 = self.get_value(msg, 'L'), self.get_value(msg, 'L2')
        if L:  # L - STOP
            self.stop_pub.publish(Bool(data=True))

        elif L2 > 0:  # L2 - enable motion

            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.linear.z = 1.0  # use linear.z as decaying priority, last for 1 second
            self.vel.angular.z = 0.0

            LX, LY, RX = self.get_value(msg, 'LX'), self.get_value(msg, 'LY'), self.get_value(msg, 'RX')
            if LX or LY:
                self.vel.linear.x = self.speed_level * LY  # Left stick up/down
                self.vel.linear.y = self.speed_level * LX  # Left stick left/right
            if RX:  # TODO: backward
                self.vel.angular.z = self.speed_level * RX  # Right stick left/right
            if self.moby_type == 'moby_agri':
                self.vel.linear.y = 0.0
                TX, TY = self.get_value(msg, 'TX'), self.get_value(msg, 'TY')
                if TX or TY:  # TODO: backward
                    self.cam_vel.linear.z = self.speed_level * TY
                    self.cam_vel.angular.z = self.speed_level * TX
                    self.cam_pub.publish(self.cam_vel)
            self.vel_pub.publish(self.vel)

        R = self.get_value(msg, 'R')
        if (Y > 0 or R > 0) and not self.speed_up:
            self.speed_level = round((self.speed_level + self.SPEED_CHANGE_STEP), 1)
            if self.speed_level > self.MAX_SPEED:
                self.speed_level = self.MAX_SPEED
            self.speed_up = True
            rospy.loginfo(f'MAX SPEED: {self.speed_level}')
        elif not (Y > 0 or R > 0) and self.speed_up:
            self.speed_up = False

        R2 = self.get_value(msg, 'R2')
        if (B > 0 or R2 > 0) and not self.speed_down:
            self.speed_level = round((self.speed_level - self.SPEED_CHANGE_STEP), 1)
            if self.speed_level < self.SPEED_CHANGE_STEP:
                self.speed_level = 0.0
            self.speed_down = True
            rospy.loginfo(f'MAX SPEED: {self.speed_level}')
        elif not (B > 0 or R2 > 0) and self.speed_down:
            self.speed_down = False

def main():
    controller = Controller()
    rospy.spin()

if __name__ == '__main__':
    main()
