#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from csv import writer
from datetime import datetime
import os
from std_msgs.msg import Float32MultiArray


x = 0.0
y = 0.0
t = 0.0

now = datetime.now()
date_time = now.strftime("%m-%d-%Y_%H-%M-%S")
file_name = f"odom_{date_time}.csv"
path = "/home/nopparuj/FRA532EXAM_WS/odom_record"

file_path = os.path.join(path, file_name)


class odom_record(Node):
    def __init__(self):
        super().__init__("odom_record")

        self.create_subscription(Odometry, "/example/odom", self.odom_callback, 10)
        self.create_subscription(Odometry, "/odom", self.ekf_callback, 10)
        self.create_subscription(Float32MultiArray, "/cmd_vel_integrated", self.cmd_vel_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.integrated_x = 0.0
        self.integrated_y = 0.0
        self.ekf_x = 0.0
        self.ekf_y = 0.0

        with open(file_path, "a") as f:
            csv_writer = writer(f)
            csv_writer.writerow(["time", "odom_y", "odom_x", "integrated_y", "integrated_x", "ekf_y", "ekf_x"])
            print("Saving to: ", file_name)

    def cmd_vel_callback(self, msg):
        self.integrated_x = msg.data[0]
        self.integrated_y = msg.data[1]

    def ekf_callback(self, msg):
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y

    def odom_callback(self, msg):
        global x, y, t
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def timer_callback(self):
        global x, y, t
        self.get_logger().info(f"\nOdom: {y}, {x}\nIntegrated: {self.integrated_y}, {self.integrated_x}\nEKF: {self.ekf_y}, {self.ekf_x}")
        with open(file_path, "a") as f:
            csv_writer = writer(f)
            csv_writer.writerow([t, y, x, self.integrated_y, self.integrated_x, self.ekf_y, self.ekf_x])


def main(args=None):
    rclpy.init(args=args)
    node = odom_record()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Saved to: ", file_name)
        exit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
