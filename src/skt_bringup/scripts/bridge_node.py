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
import os
from std_msgs.msg import Header, Float32, Float32MultiArray

class bridge_node(Node):
    def __init__(self):
        super().__init__('PubOdomNode')
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

        self.odom_pose_cov = config['odom_pose_cov']
        self.odom_twist_cov = config['odom_twist_cov']
        # Publisher
        self.publish_odom = self.create_publisher(Odometry, 'odom', queqe_size)
        self.publish_imu = self.create_publisher(Imu, 'Imu', queqe_size)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Subscribers
        self.subscribe_wheel = self.create_subscription(
            Float32MultiArray,
            'wheel_feedback',
            self.feedback_wheel,
            queqe_size)
        
        self.subscribe_imu = self.create_subscription(
            Imu,
            'calc_imu',
            self.feedback_imu,
            queqe_size)


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

        # # Initialize the transform broadcaster
        self.tf_br = TransformBroadcaster(self)

    def feedback_imu(self, msg):
        # Subtract offset from linear acceleration and angular velocity
        accel_x = msg.linear_acceleration.x - self.accel_offset[0]
        accel_y = msg.linear_acceleration.y - self.accel_offset[1]
        accel_z = msg.linear_acceleration.z - self.accel_offset[2]
        
        gyro_x = msg.angular_velocity.x - self.gyro_offset[0]
        gyro_y = msg.angular_velocity.y - self.gyro_offset[1]
        gyro_z = msg.angular_velocity.z - self.gyro_offset[2]

        # Create IMU message and fill in the data
        imu_msg = Imu()
        imu_msg.header = msg.header

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.orientation.x = msg.orientation.x
        imu_msg.orientation.x = msg.orientation.y
        imu_msg.orientation.x = msg.orientation.z
        
        self.publish_imu.publish(imu_msg)

    def feedback_wheel(self, msg):
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_callback_time'):
            self.last_callback_time = current_time
            return
        dt = (current_time - self.last_callback_time).to_msg().nanosec * 1e-9

        self.last_callback_time = current_time
        self.leftwheel_speed = msg.data[0] * 0.0675
        self.rightwheel_speed = msg.data[1] * 0.0675

        self.delta_x = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.cos(self.th)
        self.delta_y = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.sin(self.th)
        self.delta_th = (self.rightwheel_speed - self.leftwheel_speed) / 0.169

        self.x += self.delta_x * dt
        self.y += self.delta_y * dt
        self.th += self.delta_th * dt

        print(self.th)
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
        odom_msg.twist.twist.linear = Vector3(x=self.delta_x, y=self.delta_y, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.delta_th)

        flat_odom_pose_cov = [item for sublist in self.odom_pose_cov for item in sublist]
        flat_odom_twist_cov = [item for sublist in self.odom_twist_cov for item in sublist]

        # odom_msg.pose.covariance = [float(value) for value in flat_odom_pose_cov]
        # odom_msg.twist.covariance = [float(value) for value in flat_odom_twist_cov]

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