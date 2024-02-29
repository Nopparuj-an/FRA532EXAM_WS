#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PlayCmdVel(Node):
    def __init__(self):
        super().__init__('play_cmd_vel')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.01, self.timer_callback)

        self.linear_speed = 0.1
        self.angular_speed = 0.8
        # self.cmd = ["X 0.5", "S 0.5", "W 3.14/2", "S 0.5", "X 0.5", "S 0.5", "W 3.14/2", "S 0.5", "X 0.5", "S 0.5", "W 3.14/2", "S 0.5", "X 0.5", "S 0.5", "W 3.14/2"]
        self.cmd = ["X 0.46", "S 0.5", "W 3.14/2", "S 0.5", "X 1.525", "S 0.5", "W 3.14/2", "S 0.5", "X 0.46", "S 0.5", "W 3.14/2", "S 0.5", "X 1.525", "S 0.5", "W 3.14/2"]
        self.cmd_index = -1
        self.next_cmd_time = 0

    def timer_callback(self):
        if self.get_clock().now().nanoseconds > self.next_cmd_time:
            self.cmd_index += 1
            if self.cmd_index >= len(self.cmd):
                self.get_logger().info('Finished')
                self.publish_cmd_vel(0, 0)
                exit()
            self.get_logger().info(f"Publishing: {self.cmd[self.cmd_index]}")
            cmd = self.cmd[self.cmd_index].split()
            cmd[1] = eval(cmd[1])

            if cmd[0] == 'X':
                self.publish_cmd_vel(self.linear_speed, 0)
                self.next_cmd_time = self.get_clock().now().nanoseconds + (cmd[1] / self.linear_speed) * 1e9
            elif cmd[0] == 'W':
                self.publish_cmd_vel(0, self.angular_speed)
                self.next_cmd_time = self.get_clock().now().nanoseconds + (cmd[1] / self.angular_speed) * 1e9 * 1.057
            elif cmd[0] == 'S':
                self.publish_cmd_vel(0, 0)
                self.next_cmd_time = self.get_clock().now().nanoseconds + cmd[1] * 1e9

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlayCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
