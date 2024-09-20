#!/usr/bin/python3
#-*- coding: utf-8 -*-

import json
import math
import time

import rospy

from std_msgs.msg import Bool, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, LaserScan, Range
from geometry_msgs.msg import Twist, Quaternion, TransformStamped

from moby_bringup.msg import RailSensor, CameraAngle, CameraHeight, OdomRatio

from tf import TransformBroadcaster

from utils.motordriver_utils import *
from utils.wrap_utils import *

from neuromeka import MobyClient
from neuromeka import EtherCAT

import os
import numpy as np

from threading import Lock
from collections import deque
from collections import namedtuple
import datetime

LogDat = namedtuple("LogDat", ["t", "x", "y", "yaw"])

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class SetStepControlBurstOnStuck:
    def __init__(self, moby_client, timeout_burst=1.0, count_burst=20):
        self.moby_client = moby_client
        self.set_step_control_raw = moby_client.set_target_vel
        self.burst_tic = time.time()
        self.count = 0
        self.timeout_burst = timeout_burst
        self.count_burst = count_burst

    def __call__(self, vx, vy, vw):
        vel = self.moby_client.get_moby_vel()
        if (np.sum(np.abs([vx, vy, vw])) > 0.01
                and np.sum(np.abs(vel)) < 0.001):
            self.count += 1
        else:
            self.burst_tic = time.time()
            self.count = 0
        if self.burst_tic - time.time() > self.timeout_burst \
                and self.count > self.count_burst:
            vx = vx * 3
            vy = vy * 3
            vw = vw * 3
        self.set_step_control_raw(vx, vy, vw)

class TimedPriority:
    def __init__(self, value=0):
        self.set(value)

    def set(self, value):
        self.time0 = time.time()
        self.value = value

    def __call__(self):
        return max(0, self.value - (time.time() - self.time0))

class MobyROSConnector:
    PUBLISH_RATE = 60  # Hz

    def __init__(self):
        rospy.init_node('moby_driver', anonymous=True)

        # Initialize parameters with default values
        self.step_ip = rospy.get_param('~step_ip', "192.168.1.12")
        self.use_gyro = rospy.get_param('~use_gyro', True)
        self.moby_type = rospy.get_param('~moby_type', "moby_rp")
        self.body_length = rospy.get_param('~body_length', 0.9)
        self.body_width = rospy.get_param('~body_width', 0.6)
        self.lidar_margin = rospy.get_param('~lidar_margin', 0.2)
        self.ir_margin = rospy.get_param('~ir_margin', 0.2)
        self.flag_save_log = rospy.get_param('~flag_save_log', False)
        self.duration_log = rospy.get_param('~duration_log', 30.0)
        self.reset_log()

        self.vx_max_ir, self.vx_min_ir = np.inf, -np.inf
        self.vy_max_ir, self.vy_min_ir = np.inf, -np.inf
        self.vx_max_lidar, self.vx_min_lidar = np.inf, -np.inf
        self.vy_max_lidar, self.vy_min_lidar = np.inf, -np.inf
        self.scan_ranges_stack = deque(maxlen=2)
        self.priority_saved = TimedPriority()

        self.twist_subscriber = rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
        self.stop_subscriber = rospy.Subscriber('cmd_stop', Bool, self.stop_callback)
        self.zero_subscriber = rospy.Subscriber('cmd_set_zero', Bool, self.zero_callback)
        self.lidar_subscriber = rospy.Subscriber('scan', LaserScan, self.lidar_callback)

        if self.moby_type == 'moby_agri':
            self.cam_subscriber = rospy.Subscriber('agri_cam_vel', Twist, self.cam_callback)
            self.camera_module_angle_subscriber = rospy.Subscriber('camera_angle', CameraAngle, self.camera_angle_callback)
            self.camera_module_height_subscriber = rospy.Subscriber('camera_height', CameraHeight, self.camera_elevator_callback)

        self.odom_ratio_subscriber = rospy.Subscriber('odom_ratio', OdomRatio, self.odom_ratio_callback)

        self.odom_publisher = rospy.Publisher('odom_encoders', Odometry, queue_size=10)
        self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=10)
        self.rail_sensor_publisher = rospy.Publisher('rail_sensor', RailSensor, queue_size=10)
        self.joint_state_pub = rospy.Publisher('joint_states',JointState, queue_size=10)
        
        self.imu_msg = Imu()
        self.odom_msg = Odometry()
        self.rail_msg = RailSensor()

        # Initialize topics
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.PUBLISH_RATE), self.timer_callback)

        # Initialize variables
        self.moby = None
        self.control_timeout = 0
        self.stop_send_cmd_vel = True

        self.ecat = None
        self.cam_timeout = 0
        self.stop_send_cam_vel = True
        self.cam_lock = Lock()

        self.odom_ratio = 1
        # self.ir_pub_dict = None
        
        # Robot-specific
        self.wheel_base = 0.786  # Distance between front and rear wheels
        self.track_width = 0.4108  # Distance between left and right wheels
        self.wheel_radius = 0.1  # Wheel radius (in meters)

    def reset_log(self):
        if self.flag_save_log:
            self.save_log()
        self.cmd_log = []
        self.vel_log = []
        self.pos_log = []

    def save_log(self):
        home_dir = os.environ["HOME"]
        log_dir = os.path.join(home_dir, "LOG", datetime.datetime.now().strftime('%Y%m%d%H%M%S'))
        os.makedirs(log_dir)
        if hasattr(self, "cmd_log") and len(self.cmd_log) > 0:
            np.savetxt(os.path.join(log_dir, "cmd_log.csv"), self.cmd_log, delimiter=",")
        if hasattr(self, "vel_log") and len(self.vel_log) > 0:
            np.savetxt(os.path.join(log_dir, "vel_log.csv"), self.vel_log, delimiter=",")
        if hasattr(self, "pos_log") and len(self.pos_log) > 0:
            np.savetxt(os.path.join(log_dir, "pos_log.csv"), self.pos_log, delimiter=",")

    def connect(self):
        rospy.loginfo(f"STEP IP: {self.step_ip}")
        rospy.loginfo(f"Use gyro: {self.use_gyro}")
        rospy.loginfo(f"Wait Moby on : {self.step_ip}")

        self.moby = None
        while self.moby is None:
            time.sleep(1)
            try:
                self.moby = MobyClient(self.step_ip)
                self.moby.use_gyro_for_odom(self.use_gyro)
                self.moby.reset_gyro()
                # ir_data = self.moby.get_ir_data()
                # if self.ir_pub_dict is None:
                #     self.ir_pub_dict = {
                #         ir_key: rospy.Publisher(f"{ir_key}_range", Range, queue_size=10)
                #         for ir_key in ir_data.keys()
                #     }
            except Exception as e:
                rospy.logerr(f"CANNOT CONNECT TO STEP ON {self.step_ip}. TRY RECONNECT EVERY SECOND")
                rospy.logerr(str(e))
                self.moby = None

        self.ecat = None
        while self.ecat is None:
            time.sleep(1)
            try:
                self.ecat = EtherCAT(self.step_ip)
                if self.moby_type == "moby_agri":
                    slave_num = len(self.ecat.is_system_ready())
                    if slave_num > 5:
                        self.ecat.set_servo(4, False)
                        self.ecat.set_servo(3, True)
                        self.ecat.set_servo_rx(2, 15, OP_MODE_CYCLIC_SYNC_TORQUE, 0, 0, 0)
                        self.ecat.set_maxTorque(2, 4000)
                        self.ecat.set_max_motor_speed(2, 5000)
                        self.ecat.set_maxTorque(3, 65000)
                        self.ecat.set_max_motor_speed(3, 10000000)
                        assert self.ecat.is_system_ready()[4] == 0
                    else:
                        rospy.logwarn(f"Slave number {slave_num} is not expected for MOBY-AGRI. "
                                      f"Maybe Camera module is not available")
            except Exception as e:
                rospy.logerr(f"CANNOT CONNECT TO ECAT CLIENT ON {self.step_ip}")
                rospy.logerr(str(e))
                self.ecat = None
        rospy.loginfo(f"Moby Connected to : {self.step_ip}")

    # def publish_ir(self): #IR WAS REMOVE
    #     try:
    #         ir_data = self.moby.get_ir_data()
    #         vx_max_ir, vx_min_ir, vy_max_ir, vy_min_ir = np.inf, -np.inf, np.inf, -np.inf
    #         for ir_key, ir_val in ir_data.items():
    #             self.ir_pub_dict[ir_key].publish(
    #                 Range(header=Header(frame_id=f"{ir_key}_link", stamp=rospy.Time.now()),
    #                       radiation_type=Range.INFRARED,
    #                       field_of_view=0.436,  # deg2rad(25)
    #                       min_range=0.0,
    #                       max_range=0.3,
    #                       range=ir_val / 1000
    #                       )
    #             )
    #             if ir_val / 1000 < self.ir_margin:
    #                 if 'front' in ir_key:
    #                     vx_max_ir = 0
    #                 if 'rear' in ir_key:
    #                     vx_min_ir = 0
    #                 if 'left' in ir_key:
    #                     vy_max_ir = 0
    #                 if 'right' in ir_key:
    #                     vy_min_ir = 0
    #         self.vx_max_ir, self.vx_min_ir = vx_max_ir, vx_min_ir
    #         self.vy_max_ir, self.vy_min_ir = vy_max_ir, vy_min_ir
    #     except Exception as e:
    #         rospy.logerr(f"Error in publish_ir: {e}")

    def lidar_callback(self, scan):
        try:
            self.scan = scan
            scan_angles = np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment)[:-1]
            self.scan_ranges_stack.append(np.array(self.scan.ranges))
            self.scan_ranges = np.min(self.scan_ranges_stack, axis=0)
            self.scan_xy = np.multiply([np.cos(scan_angles), np.sin(scan_angles)],
                                       self.scan_ranges).transpose()
            idx_fr = np.abs(self.scan_xy[:, 1]) < self.body_width / 2
            idx_lr = np.abs(self.scan_xy[:, 0]) < self.body_length / 2
            idx_lr = np.abs(self.scan_xy[:, 0]) < self.body_length / 2
            idx_front = np.logical_and(idx_fr, self.scan_xy[:, 0] > 0)
            idx_rear = np.logical_and(idx_fr, self.scan_xy[:, 0] < 0)
            idx_left = np.logical_and(idx_lr, self.scan_xy[:, 1] > 0)
            idx_right = np.logical_and(idx_lr, self.scan_xy[:, 1] < 0)

            if np.sum(np.abs(self.scan_xy[idx_front, 0]) < self.body_length / 2 + self.lidar_margin) > 1:
                self.vx_max_lidar = 0
            else:
                self.vx_max_lidar = np.inf

            if np.sum(np.abs(self.scan_xy[idx_rear, 0]) < self.body_length / 2 + self.lidar_margin) > 1:
                self.vx_min_lidar = 0
            else:
                self.vx_min_lidar = -np.inf

            if np.sum(np.abs(self.scan_xy[idx_left, 1]) < self.body_width / 2 + self.lidar_margin) > 1:
                self.vy_max_lidar = 0
            else:
                self.vy_max_lidar = np.inf

            if np.sum(np.abs(self.scan_xy[idx_right, 1]) < self.body_width / 2 + self.lidar_margin) > 1:
                self.vy_min_lidar = 0
            else:
                self.vy_min_lidar = -np.inf
        except Exception as e:
            rospy.logerr(f"Error in lidar_callback: {e}")

    def stop_callback(self, req):
        try:
            if self.moby is not None and req.data:
                rospy.loginfo("STOP MOTION (TORQUE ZERO)")
                self.moby.stop_motion()
                self.reset_log()
        except Exception as e:
            rospy.logerr(f"Error in stop_callback: {e}")

    def zero_callback(self, req):
        try:
            if self.moby is not None and req.data:
                rospy.loginfo("SET ZERO AS CURRENT")
                self.moby.set_zero_as_current()
        except Exception as e:
            rospy.logerr(f"Error in zero_callback: {e}")

    def twist_callback(self, twist):
        try:
            if self.moby is not None:
                vx = np.clip(twist.linear.x, max(self.vx_min_ir, self.vx_min_lidar), min(self.vx_max_ir, self.vx_max_lidar))
                vy = np.clip(twist.linear.y, max(self.vy_min_ir, self.vy_min_lidar), min(self.vy_max_ir, self.vy_max_lidar))
                priority = twist.linear.z
                vw = twist.angular.z
                priority_saved = self.priority_saved()

                rospy.logdebug(f"Set Moby Velocity {[vx, vy, vw]} ({priority:.2f}/{priority_saved:.2f})")
                if priority >= priority_saved:
                    rospy.loginfo(f"Set Moby Velocity {[vx, vy, vw]} ({priority:.2f})")
                    self.priority_saved.set(priority)
                    self.moby.set_target_vel(vx, vy, vw)
                    
                self.control_timeout = time.time()
                self.stop_send_cmd_vel = False
                
                if self.moby is not None and self.flag_save_log:
                    time_sec = rospy.Time.now().to_sec()
                    self.cmd_log.append(LogDat(time_sec, vx, vy, vw))
        except Exception as e:
            rospy.logerr(f"Error in twist_callback: {e}")

    def elev_by(self, delta_cm):
        try:
            slave_idx = 3
            if self.ecat.is_system_ready()[slave_idx + 1] == 0:
                self.ecat.set_servo(slave_idx + 1, True)
                self.ecat.set_servo_rx(slave_idx, 15, OP_MODE_CYCLIC_SYNC_TORQUE, 0, 0, 0)
            elev_cur = self.ecat.get_servo_tx(3)[2] / 4000000
            target_pos_cm = elev_cur + delta_cm
            target_pos_cnt = int(target_pos_cm * 4000000)
            self.ecat.set_servo_rx(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
            time.sleep(0.01)
            self.ecat.set_servo_rx(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)
        except Exception as e:
            rospy.logerr(f"Error in elev_by: {e}")

    def rotate_by(self, delta_deg):
        try:
            slave_idx = 2
            rot_cur = self.ecat.get_servo_tx(slave_idx)[2] / 1000
            target_pos_deg = rot_cur + delta_deg
            target_pos_cnt = int(target_pos_deg * 1000)
            self.ecat.set_servo_rx(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
            time.sleep(0.01)
            self.ecat.set_servo_rx(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)
        except Exception as e:
            rospy.logerr(f"Error in rotate_by: {e}")

    def cam_callback(self, twist):
        try:
            if self.moby_type == 'moby_agri':
                with self.cam_lock:
                    if self.ecat is not None:
                        rospy.loginfo(f"Move Camera Elev: {twist.linear.z:.2f}, Rot: {twist.angular.z:.2f}")
                        if abs(twist.linear.z) > 0.05:
                            self.elev_by(twist.linear.z * 20)
                        if abs(twist.angular.z) > 0.05:
                            self.rotate_by(-twist.angular.z * 45)
                    self.cam_timeout = time.time()
                    self.stop_send_cam_vel = False
        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")

    def odom_ratio_callback(self, msg):
        try:
            self.odom_ratio = msg.odom_ratio
            rospy.loginfo(f'Odom Ratio: "{self.odom_ratio}"')
        except Exception as e:
            rospy.logerr(f"Error in odom_ratio_callback: {e}")

    def camera_angle_callback(self, msg):
        try:
            rospy.loginfo(f'Publishing: "{msg.camera_angle}"')
            target_pos_deg = msg.camera_angle
            slave_idx = 2
            target_pos_cnt = int(target_pos_deg * 1000)
            self.ecat.set_servo_rx(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
            time.sleep(0.01)
            self.ecat.set_servo_rx(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)
        except Exception as e:
            rospy.logerr(f"Error in camera_angle_callback: {e}")

    def camera_elevator_callback(self, msg):
        try:
            rospy.loginfo(f'Publishing: "{msg.camera_height}"')
            target_pos_cm = msg.camera_height
            if target_pos_cm > 50:
                rospy.logwarn('TARGET POSITION IS TOO HIGH')
                return
            slave_idx = 3
            target_pos_cnt = int(target_pos_cm * 4000000)
            self.ecat.set_servo_rx(slave_idx, 0x3f, 1, target_pos_cnt, 0, 0)
            time.sleep(0.01)
            self.ecat.set_servo_rx(slave_idx, 0x2f, 1, target_pos_cnt, 0, 0)
        except Exception as e:
            rospy.logerr(f"Error in camera_elevator_callback: {e}")

    def odom_publish_callback(self):
        try:
            if self.moby is not None:
                moby_pose = self.moby.get_moby_pose()
                moby_vel = self.moby.get_moby_vel()
            else:
                moby_pose = [0.0, 0.0, 0.0]
                moby_vel = [0.0, 0.0, 0.0]

            self.odom_msg.header.frame_id = 'odom'
            self.odom_msg.child_frame_id = 'base_footprint'
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.pose.pose.position.x = moby_pose[0]
            self.odom_msg.pose.pose.position.y = moby_pose[1]
            self.odom_msg.pose.pose.position.z = 0.0
            self.odom_msg.pose.pose.orientation = quaternion_from_euler(0, 0, moby_pose[2])
            self.odom_msg.twist.twist.linear.x = moby_vel[0] * self.odom_ratio
            self.odom_msg.twist.twist.linear.y = moby_vel[1] * self.odom_ratio
            self.odom_msg.twist.twist.angular.z = moby_vel[2]

            if self.flag_save_log:
                time_sec = self.odom_msg.header.stamp.to_sec()
                self.vel_log.append(LogDat(time_sec, *moby_vel))
                self.pos_log.append(LogDat(time_sec, *moby_pose))

            self.odom_publisher.publish(self.odom_msg)
        except Exception as e:
            rospy.logerr(f"Error in odom_publish_callback: {e}")

    def imu_publish_callback(self):
        try:
            if self.moby is not None:
                if self.use_gyro:
                    self.moby.use_gyro_for_odom(True)
                    moby_gyro = self.moby.get_gyro_data()
                else:
                    self.moby.use_gyro_for_odom(False)
                    moby_gyro = [self.moby.get_moby_pose()[2], 0]
            else:
                moby_gyro = [0.0, 0.0]

            self.imu_msg.header.frame_id = 'imu_link'
            self.imu_msg.header.stamp = rospy.Time.now()
            self.imu_msg.orientation = quaternion_from_euler(0, 0, moby_gyro[0])

            self.imu_publisher.publish(self.imu_msg)
        except Exception as e:
            rospy.logerr(f"Error in imu_publish_callback: {e}")

    def rail_sensor_publish_callback(self):
        try:
            if self.ecat is not None:
                di = self.ecat.get_di(0)
                left_sensor = di[2]
                right_sensor = di[3]
            else:
                left_sensor = 0
                right_sensor = 0

            self.rail_msg.rail_sensor = 1 if left_sensor or right_sensor else 0
            self.rail_sensor_publisher.publish(self.rail_msg)
        except Exception as e:
            rospy.logerr(f"Error in rail_sensor_publish_callback: {e}")

    def joint_state_publisher(self):
        try:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            
            if self.moby_type == 'moby_rp' or self.moby_type == 'moby_rp_v3':
                joint_state_msg.name = ['fl_rot_joint', 'fr_rot_joint', 'rl_rot_joint', 'rr_rot_joint', 
                                        'fl_tract_joint', 'fr_tract_joint', 'rl_tract_joint', 'rr_tract_joint']
                # joint_state_msg.position = [0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0]
                # joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0]             
                moby_wheel_angle = self.moby.get_rotation_angle()
                moby_wheel_vel = self.moby.get_drive_speed()

                joint_state_msg.position = [math.radians(float(moby_wheel_angle['fl'])), 
                                            math.radians(float(moby_wheel_angle['fr'])), 
                                            math.radians(float(moby_wheel_angle['bl'])),
                                            math.radians(float(moby_wheel_angle['br'])),
                                            0.0, 0.0, 0.0, 0.0]
                joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0,
                                            float(moby_wheel_vel['fl'])/self.wheel_radius, 
                                            float(moby_wheel_vel['fr'])/self.wheel_radius, 
                                            float(moby_wheel_vel['bl'])/self.wheel_radius,
                                            float(moby_wheel_vel['br'])/self.wheel_radius]
                
                self.joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            rospy.error(str(e))
            
    def timer_callback(self, event):
        try:
            self.joint_state_publisher()
            self.odom_publish_callback()
            self.imu_publish_callback()
            self.rail_sensor_publish_callback()
            if self.moby is not None and time.time() - self.control_timeout >= 0.15 and not self.stop_send_cmd_vel:
                rospy.loginfo(f"Set Moby Velocity {[0, 0, 0]}")
                self.moby.set_target_vel(0.0, 0.0, 0.0)
                self.stop_send_cmd_vel = True
            if self.ecat is not None and self.moby_type == "moby_agri":
                if time.time() - self.cam_timeout >= 0.15 and not self.stop_send_cam_vel:
                    self.elev_by(0)
                    self.rotate_by(0)
                    self.ecat.set_servo(4, False)
                    self.ecat.set_servo(3, True)
                    self.ecat.set_servo_rx(2, 15, OP_MODE_CYCLIC_SYNC_TORQUE, 0, 0, 0)
                    self.stop_send_cam_vel = True
            # # self.publish_ir()
        except Exception as e:
            rospy.logerr(f"Error in timer_callback: {e}")

def main():
    moby_driver = MobyROSConnector()
    moby_driver.connect()
    rospy.spin()

if __name__ == '__main__':
    main()
