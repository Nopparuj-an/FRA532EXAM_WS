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


class bridge_node(Node):
    def __init__(self):
        super().__init__('PubOdomNode')
        queqe_size = 10
        # Publisher
        self.publish_odom = self.create_publisher(Odometry, 'odom', queqe_size)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Subscribers
        self.subscribe_wheel = self.create_subscription(
            Twist,
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

        # Initialize the transform broadcaster
        self.tf_br = TransformBroadcaster(self)
        self.isOdomUpdate = False
        self.odom_output = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='odom'
            ),
            child_frame_id='base_link',
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    angular=Vector3(
                        z=0.0
                    )
                )
            )
        )
        
    def feedback_imu(self, msg):
        pass

    def feedback_wheel(self, msg):
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_callback_time'):
            self.last_callback_time = current_time
            return
        dt = (current_time - self.last_callback_time).to_msg().nanosec * 1e-9

        self.last_callback_time = current_time
        self.leftwheel_speed = msg.data[0] * 0.075
        self.rightwheel_speed = msg.data[1] * 0.075

        self.delta_x = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.cos(self.th)
        self.delta_y = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.sin(self.th)
        self.delta_th = (self.rightwheel_speed - self.leftwheel_speed) / 0.4

        self.x += np.round(self.delta_x * dt, 3)
        self.y += np.round(self.delta_y * dt, 3)
        self.th += np.round(self.delta_th * dt, 3)

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
        odom_msg.twist.twist.linear = Vector3(x=self.vx, y=self.vy, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.wz)

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