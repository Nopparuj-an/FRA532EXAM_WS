#!/usr/bin/python3

import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Calibrate_imu(Node):
    def __init__(self):
        super().__init__('Calibrate_imu')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        self.create_subscription(Float32MultiArray, 'imu', self.msg_callback, qos_profile=qos_profile)
        self.calibation_flag = True

        self.count = 0
        self.offset_gyro_x = []
        self.offset_gyro_y = []
        self.offset_gyro_z = []

        self.offset_accel_x = []
        self.offset_accel_y = []
        self.offset_accel_z = []

        self.mag_x = []
        self.mag_y = []
        self.mag_z = []
        print("Initialize")
    def timer_callback(self):
        pass
    
    def msg_callback(self, msg):
        if self.calibation_flag:

            accel_x = msg.data[2]
            accel_y = msg.data[3]
            accel_z = msg.data[4]
        
            gyro_x = msg.data[5]
            gyro_y = msg.data[6]
            gyro_z = msg.data[7]
        

            mag_x = msg.data[8]
            mag_y = msg.data[9]
            mag_z = msg.data[10]

            self.count += 1
            self.offset_gyro_x.append(gyro_x)
            self.offset_gyro_y.append(gyro_y)
            self.offset_gyro_z.append(gyro_z)
            
            self.offset_accel_x.append(accel_x)
            self.offset_accel_y.append(accel_y)
            self.offset_accel_z.append(accel_z)

            self.mag_x.append(mag_x)
            self.mag_y.append(mag_y)
            self.mag_z.append(mag_z)

            self.get_logger().info('Calibrated {} / 100'.format(self.count))
            if self.count >= 100:
                stack_gyro = np.stack((self.offset_gyro_x, self.offset_gyro_y, self.offset_gyro_z),axis=0)
                stack_accel = np.stack((self.offset_accel_x, self.offset_accel_y, self.offset_accel_z),axis=0)
                stack_mag = np.stack((self.mag_x, self.mag_y, self.mag_z),axis = 0)

                self.offset_gyro_x = np.mean(self.offset_gyro_x)
                self.offset_gyro_y = np.mean(self.offset_gyro_y)
                self.offset_gyro_z = np.mean(self.offset_gyro_z)

                self.offset_accel_x = np.mean(self.offset_accel_x)
                self.offset_accel_y = np.mean(self.offset_accel_y)
                self.offset_accel_z = np.mean(self.offset_accel_z)

                self.cov_gyro = np.cov(stack_gyro)
                self.cov_accel = np.cov(stack_accel)
                self.cov_mag = np.cov(stack_mag)

                self.write_config_to_yaml()
                self.calibation_flag = False
                sys.exit()
    
    def write_config_to_yaml(self):
        pkg_path = get_package_share_directory('skt_bringup')
        yaml_file = os.path.join(pkg_path, 'config', 'feedback_config.yaml')

        # Read parameters from YAML file
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)

        # Update accel_offset values
        config['accel_offset'] = [float(self.offset_accel_x), float(self.offset_accel_y), float(self.offset_accel_z)]
        config['gyro_offset'] = [float(self.offset_gyro_x), float(self.offset_gyro_y), float(self.offset_gyro_z)]
        config['accel_cov'] = self.cov_accel.tolist()
        config['gyro_cov'] = self.cov_gyro.tolist()
        config['mag_cov'] = self.cov_mag.tolist()
        # Write updated parameters back to YAML file
        with open(yaml_file, 'w') as file:
            yaml.dump(config, file)

        self.get_logger().info('Updated config written to {}'.format(yaml_file))
        
def main(args=None):
    rclpy.init(args=args)
    node = Calibrate_imu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
