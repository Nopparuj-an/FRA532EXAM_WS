#!/usr/bin/python3

import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np
from sensor_msgs.msg import Imu


class Calibrate_imu(Node):
    def __init__(self):
        super().__init__('Calibrate_imu')
        # establish timer
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        self.create_subscription(Imu, '/imu_data', self.msg_callback, 10)
        self.calibation_flag = True

        self.count = 0
        self.offset_gyro_x = []
        self.offset_gyro_y = []
        self.offset_gyro_z = []

        self.offset_accel_x = []
        self.offset_accel_y = []
        self.offset_accel_z = []

    def timer_callback(self):
        pass
    
    def msg_callback(self, msg):
        if self.calibation_flag:
            gyro_x = msg.angular_velocity.x
            gyro_y = msg.angular_velocity.y
            gyro_z = msg.angular_velocity.z
            
            accel_x = msg.linear_acceleration.x
            accel_y = msg.linear_acceleration.y
            accel_z = msg.linear_acceleration.z

            self.count += 1
            self.offset_gyro_x.append(gyro_x)
            self.offset_gyro_y.append(gyro_y)
            self.offset_gyro_z.append(gyro_z)
            
            self.offset_accel_x.append(accel_x)
            self.offset_accel_y.append(accel_y)
            self.offset_accel_z.append(accel_z)

            if self.count >= 100:
                stack_gyro = np.stack((self.offset_gyro_x, self.offset_gyro_y, self.offset_gyro_z),axis=0)
                stack_accel = np.stack((self.offset_accel_x, self.offset_accel_y, self.offset_accel_z),axis=0)

                self.offset_gyro_x = np.mean(self.offset_gyro_x)
                self.offset_gyro_y = np.mean(self.offset_gyro_y)
                self.offset_gyro_z = np.mean(self.offset_gyro_z)

                self.offset_accel_x = np.mean(self.offset_accel_x)
                self.offset_accel_y = np.mean(self.offset_accel_y)
                self.offset_accel_z = np.mean(self.offset_accel_z)

                self.cov_gyro = np.cov(stack_gyro)
                self.cov_accel = np.cov(stack_accel)

                self.write_config_to_yaml()
                self.calibation_flag = False
                print("Calibration", self.count*100/100, "%")
                self.get_logger().info('Calibrated {} / 100'.format(self.count*100))

    def write_config_to_yaml(self):
        pkg_path = get_package_share_directory('skt_bringup')
        yaml_file = os.path.join(pkg_path, 'config', 'feedback_config.yaml')

        # Read parameters from YAML file
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)

        # Update accel_offset values
        config['accel_offset'] = [self.offset_accel_x, self.offset_accel_y, self.offset_accel_z]
        config['gyro_offset'] = [self.offset_gyro_x, self.offset_gyro_y, self.offset_gyro_z]
        config['accel_cov'] = self.cov_accel.tolist()
        config['gyro_cov'] = self.cov_gyro.tolist()

        # Write updated parameters back to YAML file
        with open(yaml_file, 'w') as file:
            yaml.dump(config, file)

        self.get_logger().info('Updated config written to {}'.format(yaml_file))
        sys.exit()
def main(args=None):
    rclpy.init(args=args)
    node = Calibrate_imu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
