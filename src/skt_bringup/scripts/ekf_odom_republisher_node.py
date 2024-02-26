#!/usr/bin/python3

import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion,
                               Twist, TransformStamped,TwistWithCovariance, Vector3)


class ekfOdomRepublisher(Node):
    def __init__(self):
        super().__init__('ekf_odom_republisher_node')
        for i in range(7):
            self.get_logger().info(f"Waiting for ekf node: {7-i}")
            time.sleep(1)
        self.create_subscription(Odometry, 'odometry/filtered', self.ekf_odom_callback, 10)
        self.publish_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_br = TransformBroadcaster(self)

        self.margin = 0.0002
        self.dt = 0.01
        self.amount = 20  # amount of data to calibrate

        self.state = 0  # 0: calibrating, 1: transition, 2: running
        self.vx_sum = 0.0
        self.wz_sum = 0.0
        self.count = 0
        self.vx_avg = 0.0
        self.wz_avg = 0.0
        
        self.vx_max = 0.0
        self.vx_min = 0.0
        self.wz_max = 0.0
        self.wz_min = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def timer_callback(self):
        if self.vx_min < self.vx < self.vx_max:
            self.vx = 0.0
        if self.wz_min < self.wz < self.wz_max:
            self.wz = 0.0

        # integrate linear position
        if abs(self.vx) <= 0.5:
            self.x += self.vx * math.cos(self.theta) * self.dt
            self.y += self.vx * math.sin(self.theta) * self.dt

        # publish odometry
        # print(self.x, self.y)
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()  # Update time stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        )
        self.publish_odom.publish(odom_msg)

         # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'  # Make sure it matches the child frame ID in odom_output
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)

    def ekf_odom_callback(self, msg):
        if self.state == 2:
            self.theta = tf_transformations.euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])[2]
            self.vx = msg.twist.twist.linear.x - self.vx_avg
            self.wz = msg.twist.twist.angular.z - self.wz_avg
        elif self.state == 0:
            self.vx_sum += msg.twist.twist.linear.x
            self.wz_sum += msg.twist.twist.angular.z
            self.vx_max = max(self.vx_max, msg.twist.twist.linear.x)
            self.vx_min = min(self.vx_min, msg.twist.twist.linear.x)
            self.wz_max = max(self.wz_max, msg.twist.twist.angular.z)
            self.wz_min = min(self.wz_min, msg.twist.twist.angular.z)
            self.count += 1
            if self.count % 10 == 0:
                self.get_logger().info(f"Calibrating: {self.count}/{self.amount}")
            if self.count >= self.amount:
                self.state = 1
        elif self.state == 1:
            self.vx_avg = self.vx_sum / self.amount
            self.wz_avg = self.wz_sum / self.amount
            self.vx_max -= self.vx_avg - self.margin
            self.vx_min -= self.vx_avg + self.margin
            self.wz_max -= self.wz_avg - self.margin
            self.wz_min -= self.wz_avg + self.margin
            self.get_logger().info(f'Calibration finished\nAverage vx: {self.vx_avg}\nAverage wz: {self.wz_avg}')
            del(self.amount)
            del(self.vx_sum)
            del(self.wz_sum)
            del(self.count)
            self.create_timer(self.dt, self.timer_callback)
            self.state = 2


def main(args=None):
    rclpy.init(args=args)
    node = ekfOdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
