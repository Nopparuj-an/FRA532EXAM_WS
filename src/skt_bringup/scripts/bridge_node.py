#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion,
                               Twist, TransformStamped,TwistWithCovariance, Vector3)
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
import math
import numpy as np
import time  # Importing the time library
from typing import NamedTuple
from sensor_msgs.msg import Imu 
from ament_index_python.packages import get_package_share_directory
import yaml
import os,sys
from std_msgs.msg import Header, Float32, Float32MultiArray
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class bridge_node(Node):
    def __init__(self):
        super().__init__('PubOdomNode')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        queqe_size = 10

        pkg_path = get_package_share_directory('skt_bringup')
        yaml_file = os.path.join(pkg_path, 'config', ('feedback_config.yaml'))

        # Read parameters from YAML file
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)

        self.accel_offset = config['accel_offset']
        self.accel_cov = config['accel_cov']
        self.gyro_offset = config['gyro_offset']
        self.gyro_cov = config['gyro_cov']
        self.mag_cov = config['mag_cov']

        self.odom_pose_cov = config['odom_pose_cov']
        self.odom_twist_cov = config['odom_twist_cov']

        # Publisher
        self.publish_odom = self.create_publisher(Odometry, 'odom', queqe_size)
        self.publish_imu = self.create_publisher(Imu, 'example/imu', qos_profile=qos_profile)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Subscribers
        self.subscribe_wheel = self.create_subscription(
            Float32MultiArray,
            'wheel_speeds',
            self.feedback_wheel,
            qos_profile=qos_profile)
        
        self.subscribe_imu = self.create_subscription(
            Float32MultiArray,
            'imu',
            self.feedback_imu,
            qos_profile=qos_profile
            )

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_th = 0.0

        self.rightwheel_speed = 0.0
        self.leftwheel_speed = 0.0

        self.relative_yaw = 0.0
        self.initial_orientation = None

        self.SigP = np.array([[1e-6, 0, 0], 
                              [0, 1e-6, 0], 
                              [0, 0, 1e-6]])
        # # Initialize the transform broadcaster
        self.tf_br = TransformBroadcaster(self)

    def feedback_imu(self, msg):
        # Subtract offset from linear acceleration and angular velocity
        accel_x = (msg.data[2] - self.accel_offset[0]) * 9.81
        accel_y = (msg.data[3] - self.accel_offset[1]) * 9.81
        accel_z = (msg.data[4] - self.accel_offset[2]) * 9.81
        
        gyro_x = (msg.data[5] - self.gyro_offset[0]) * math.pi/180
        gyro_y = (msg.data[6] - self.gyro_offset[1]) * math.pi/180
        gyro_z = (msg.data[7] - self.gyro_offset[2]) * math.pi/180

        # Create IMU message and fill in the data
        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.header.stamp = self.get_clock().now().to_msg() 

        if abs(accel_x) < 0.3:
            accel_x = 0.0
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        quaternion = tf_transformations.quaternion_from_euler(msg.data[8]*math.pi/180, -1*msg.data[9]*math.pi/180, -1*msg.data[10]*math.pi/180)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        flat_accel_cov = [item for sublist in self.accel_cov for item in sublist]
        flat_gyro_cov = [item for sublist in self.gyro_cov for item in sublist]
        flat_mag_cov = [item for sublist in self.mag_cov for item in sublist]

        imu_msg.linear_acceleration_covariance = [float(value) for value in flat_accel_cov]
        imu_msg.angular_velocity_covariance = [float(value) for value in flat_gyro_cov]
        imu_msg.orientation_covariance = [float(value) for value in flat_mag_cov]

        self.publish_imu.publish(imu_msg)

    def feedback_wheel(self, msg):
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_callback_time'):
            self.last_callback_time = current_time
            return
        dt = (current_time - self.last_callback_time).to_msg().nanosec * 1e-9

        self.last_callback_time = current_time
        self.leftwheel_speed = msg.data[0] * (0.0675 / 2)
        self.rightwheel_speed = msg.data[1] * (0.0675 / 2)

        self.delta_x = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.cos(self.th)
        self.delta_y = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.sin(self.th)
        self.delta_th = (self.rightwheel_speed - self.leftwheel_speed) / 0.162

        self.x += self.delta_x * dt
        self.y += self.delta_y * dt
        self.th += self.delta_th * dt

        # Error propagation
        kr = 1.0e-5
        kl = 1.0e-5

        ds = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * dt
        b = 0.1625
        dth = (self.rightwheel_speed - self.leftwheel_speed) * dt / b
        p13 = -ds * math.sin(self.th + dth/2)
        p23 = ds * math.cos(self.th + dth/2)

        rl11 = 0.5 * math.cos(self.th + dth * 0.5) - (ds * 0.5 * math.sin(self.th + dth * 0.5))/ b
        rl12 = 0.5 * math.cos(self.th + dth * 0.5) + (ds * 0.5 * math.sin(self.th + dth * 0.5))/ b
        rl21 = 0.5 * math.sin(self.th + dth * 0.5) + (ds * 0.5 * math.cos(self.th + dth * 0.5))/ b
        rl22 = 0.5 * math.sin(self.th + dth * 0.5) - (ds * 0.5 * math.cos(self.th + dth * 0.5))/ b

        Fp = np.array([[1, 0 , p13], 
                       [0, 1, p23],
                       [0, 0, 1]])

        Frl = np.array([[rl11, rl12], 
                        [rl21, rl22], 
                        [1/b, -1/b]])

        rl_cov = np.array([[kr * abs(self.rightwheel_speed * dt), 0],
                           [0, kl * abs(self.rightwheel_speed * dt)]])

        A = (Fp @ self.SigP @ Fp.transpose())

        B = (Frl @ rl_cov @ Frl.transpose())
        self.SigP = A + B
        print(self.SigP)

    def timer_callback(self):
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.th)
        # Create Odometry message and fill in the data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()  # Update time stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        )
        odom_msg.twist.twist.linear = Vector3(x=round((self.rightwheel_speed + self.leftwheel_speed), 5)* 0.5, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.delta_th)

        flat_odom_pose_cov = [item for sublist in self.odom_pose_cov for item in sublist]
        flat_odom_twist_cov = [item for sublist in self.odom_twist_cov for item in sublist]

        odom_msg.pose.covariance = [float(value) for value in flat_odom_pose_cov]
        odom_msg.twist.covariance = [float(value) for value in flat_odom_twist_cov]

        # Publish the Odometry message
        self.publish_odom.publish(odom_msg)

         # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'  # Make sure it matches the child frame ID in odom_output
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)
    
def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = bridge_node()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()