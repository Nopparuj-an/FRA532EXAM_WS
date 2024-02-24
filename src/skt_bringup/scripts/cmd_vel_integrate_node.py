#!/usr/bin/python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class cmd_vel_integrate_node(Node):
    def __init__(self):
        super().__init__('cmd_vel_integrate_node')
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.dt = 0.01
        self.create_timer(self.dt, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        # if self.first_run:
        #     self.last_time = self.get_clock().now().nanoseconds * 1e-9
        #     self.first_run = False
        #     return
        
        # current_time = self.get_clock().now().nanoseconds * 1e-9
        # dt = current_time - self.last_time
        # self.last_time = current_time

        # # integrate angular position
        # self.theta += msg.angular.z * dt
        # self.theta %= 2 * math.pi

        # # integrate linear position
        # self.x += msg.linear.x * math.cos(self.theta) * dt
        # self.y += msg.linear.x * math.sin(self.theta) * dt

    def timer_callback(self):
        # integrate linear position
        self.x += self.cmd_linear_x * math.cos(self.theta) * self.dt
        self.y += self.cmd_linear_x * math.sin(self.theta) * self.dt

        # integrate angular position
        self.theta += self.cmd_angular_z * self.dt
        self.theta %= 2 * math.pi

        print(self.x, self.y)


def main(args=None):
    rclpy.init(args=args)
    node = cmd_vel_integrate_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
