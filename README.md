# FRA532 Mobile Robot: Exam I

PLACEHOLDER

<br>

## System Diagram

![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/4d0b79ab-859c-44ab-9d02-f8867203b138)

<br>

## Microcontroller Programming

You can see the code here: [ESP32 Micro-ROS code](https://github.com/Nopparuj-an/FRA532EXAM_WS/blob/master/src/micro-ros_publisher/src/main.cpp)

- **Micro-ROS Wi-Fi Connection**

  Using the following code

  ```cpp
  // Initialize micro-ROS
  IPAddress ip(192, 168, 12, 1);
  set_microros_wifi_transports("TrashX", "00000000", ip, 8888);
  ```

  The microcontroller is capable of connecting to the Micro-ROS agent by starting Micro-ROS with the following command

  ```sh
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  ```

  ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/bb2bea8a-add6-4df5-b29c-450aaf60f8c7)

- **IMU Read and Publish**

  The code used to read and publish IMU data is split in two parts and run on different CPU core.

  Core 1: Read IMU and store to a variable

  ```cpp
  if (mpu.update()) {
    if (xSemaphoreTake(sem_imu, 0) == pdTRUE) {
      ax = mpu.getAccX();
      ay = mpu.getAccY();
      az = mpu.getAccZ();
      gx = mpu.getGyroX();
      gy = mpu.getGyroY();
      gz = mpu.getGyroZ();
      mx = mpu.getRoll();
      my = mpu.getPitch();
      mz = mpu.getYaw();
      xSemaphoreGive(sem_imu);
    }
  }
  ```

  Core 0: Publish data to Micro-ROS

  ```cpp
  void publish_imu() {
      static float _ax, _ay, _az, _gx, _gy, _gz, _mx, _my, _mz;
      if (xSemaphoreTake(sem_imu, 0) == pdTRUE) {
          _ax = ax;
          _ay = ay;
          _az = az;
          _gx = gx;
          _gy = gy;
          _gz = gz;
          _mx = mx;
          _my = my;
          _mz = mz;
          xSemaphoreGive(sem_imu);
      }

      static bool first_run = true;
      if (first_run) {
          imu_msg.data.data = (float *)malloc(11 * sizeof(float));
          first_run = false;
      }

      int64_t time_ms = rmw_uros_epoch_millis();

      imu_msg.data.size = 11;
      imu_msg.data.data[0] = time_ms >> 32; // Time MSB
      imu_msg.data.data[1] = time_ms & 0xFFFFFFFF; // Time LSB
      imu_msg.data.data[2] = _ax;
      imu_msg.data.data[3] = _ay;
      imu_msg.data.data[4] = _az;
      imu_msg.data.data[5] = _gx;
      imu_msg.data.data[6] = _gy;
      imu_msg.data.data[7] = _gz;
      imu_msg.data.data[8] = _mx;
      imu_msg.data.data[9] = _my;
      imu_msg.data.data[10] = _mz;

      // Publish the IMU data
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
  ```

  | Index | Size |         Data          |
  | :---: | :--: | :-------------------: |
  |   0   |  32  |    Time (ms) << 32    |
  |   1   |  32  |     Time (ms) LSB     |
  |   2   |  32  | Linear Acceleration X |
  |   3   |  32  | Linear Acceleration Y |
  |   4   |  32  | Linear Acceleration Z |
  |   5   |  32  |  Angular Velocity X   |
  |   6   |  32  |  Angular Velocity Y   |
  |   7   |  32  |  Angular Velocity Z   |
  |   8   |  32  |         Roll          |
  |   9   |  32  |         Pitch         |
  |  10   |  32  |          Yaw          |

  The performance of the data transmission is approximately 45Hz.

  ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/88512286-c456-41a0-87cb-81f2a6026298)

- **Motor Control by `/cmd_vel`**

  The code for `/cmd_vel` subscription and motor control is split in two parts and run on different CPU core.

  Core 0: Subscribe data from Micro-ROS

  ```cpp
  void cmd_vel_subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    speed_linear = msg->linear.x;
    speed_angular = msg->angular.z;
    xSemaphoreGive(sem_motor_run);
  }
  ```

  Core 1: Control the motors

  ```cpp
  if (xSemaphoreTake(sem_motor_run, 0) == pdTRUE) {
    if(speed_linear == 0 && speed_angular == 0) {
      Motor.turnWheel(1, LEFT, 0);
      Motor.turnWheel(2, RIGHT, 0);
    } else {
      float meter2rad = 1.0 / 0.03375; // wheel radius
      float wheel_separation = 0.162; // distance between wheels

      // convert to wheel speeds of differential drive robot (rev/s)
      float left_wheel_speed = speed_linear * meter2rad - (speed_angular * wheel_separation / 2) * meter2rad;
      float right_wheel_speed = speed_linear * meter2rad + (speed_angular * wheel_separation / 2) * meter2rad;

      // convert to motor speeds
      left_wheel_speed = 9.48202984517541 * left_wheel_speed + 0.908799073391677;
      right_wheel_speed = 9.48202984517541 * right_wheel_speed + 0.908799073391677;

      // send to motors
      Motor.setSpeed(1, left_wheel_speed);
      Motor.setSpeed(2, -right_wheel_speed);
    }
  }
  ```

- **Motor Speed Read and Publish**

  The code used to read and publish IMU data is split in two parts and run on different CPU core.

  Core 1: Read motor speed and store to a variable

  ```cpp
  if (xSemaphoreTake(sem_motor_speed, 0) == pdTRUE) {
    float left_wheel_speed = Motor.readSpeed(1);
    float right_wheel_speed = Motor.readSpeed(2);

    if(left_wheel_speed > 1024){
      left_wheel_speed = 1024 - left_wheel_speed;
    }
    if(right_wheel_speed > 1024){
      right_wheel_speed = right_wheel_speed - 1024;
    } else {
      right_wheel_speed = -right_wheel_speed;
    }

    speed_left = 0.0956657612073996 * left_wheel_speed;
    speed_right = 0.0956657612073996 * right_wheel_speed;
    xSemaphoreGive(sem_motor_speed);
  }
  ```

  Core 0: Publish data to Micro-ROS

  ```cpp
  void publish_wheelspeed() {
    static float _speed_left, _speed_right;
    if (xSemaphoreTake(sem_motor_speed, 0) == pdTRUE) {
      _speed_left = speed_left;
      _speed_right = speed_right;
      xSemaphoreGive(sem_motor_speed);
    }
  
    static bool first_run = true;
    if (first_run) {
      wheel_speeds_msg.data.data = (float *)malloc(2 * sizeof(float));
      first_run = false;
    }
  
    wheel_speeds_msg.data.size = 2;
    wheel_speeds_msg.data.data[0] = _speed_left;
    wheel_speeds_msg.data.data[1] = _speed_right;
    RCCHECK(rcl_publish(&wheel_speeds_publisher, &wheel_speeds_msg, NULL));
  }
  ```

  | Index | Size |     Data    |
  |:-----:|:----:|:-----------:|
  |   0   |  32  |  speed_left |
  |   1   |  32  | speed_right |

  The performance of the data transmission is approximately 45Hz.

  ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/501083bb-af9e-4f38-a9fa-e2e5fd199c4f)

- **Wheel Speed Kinematics**

  Inverse Kinematics for differential drive robot is used to convert task-space command (linear velocity and angular velocity) into configuration-space command (angular speed of each wheel)

  ```cpp
  float meter2rad = 1.0 / 0.03375; // wheel radius
  float wheel_separation = 0.162; // distance between wheels

  // convert to wheel speeds of differential drive robot (rev/s)
  float left_wheel_speed = speed_linear * meter2rad - (speed_angular * wheel_separation / 2) * meter2rad;
  float right_wheel_speed = speed_linear * meter2rad + (speed_angular * wheel_separation / 2) * meter2rad;
  ```

  An equation acquired from testing is used to convert angular velocity to motor command.

  ```cpp
  // convert to motor speeds
  left_wheel_speed = 9.48202984517541 * left_wheel_speed + 0.908799073391677;
  right_wheel_speed = 9.48202984517541 * right_wheel_speed + 0.908799073391677;
  ```

<br>

## Computer Programming

- **IMU Calibration Node Explained**
  
  This section clarifies the purpose and usage of the `calibrate_node.py` script, which performs IMU calibration.
  
  **What it does:**
  
  * The script, named `calibrate_node.py`, acts as a ROS 2 node responsible for calibrating an Inertial Measurement Unit (IMU).
  * During the calibration process, the node calculates:
      * **IMU bias:** This value represents systematic errors inherent in the IMU's sensors, such as slight offsets from their ideal readings.
      * **IMU covariance:** This matrix describes the uncertainties associated with the IMU's measurements, providing an indication of the data's reliability.
  * Once calculated, the node saves these values to a YAML file, which can be used later to compensate for the IMU's biases and incorporate the covariance information into your application.
  
  **How to run it:**
  
  1. **Open a terminal window.**
  2. **Ensure ROS 2 is properly sourced and running.**
  3. **Execute the following command:**

  ```bash
  ros2 run skt_bringup calibrate_node.py
  ```
   Config will save to this [path](https://github.com/Nopparuj-an/FRA532EXAM_WS/blob/master/config/feedback_config.yaml)
- **Robot Bridge Node**
    The main function of bridge node have 3 funtion first is calculated odometry and pose covarience odometry name  `example/odom` and other is to transform raw value of IMU to calibrated value name `example/imu`.

  - **Calculating odometry**
    
    Generally the pose (position) of a robot is represented by the vector:
    
    ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/9c4c62f6-ef0c-4454-90ea-f786307010e6)

    for differential robot we can write (Δx, Δy, Δθ) as.

    ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/a97da59f-f2f7-4b8b-9c31-bef980a679a9)

    where:

    (Δx; Δy; Δθ) = path traveled in the last sampling interval. (Δs_r; Δs_l) = traveled distances for the right and left wheel respectively. b = distance between the two wheels of differential-drive robot

    then position updated can write as

    ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/d242eb59-a04b-4eaf-81dc-bcc4e89fa6e6)
    
    
  - **Calculate odom pose covarience**  
  There are many sources of odometric error, some of them are:
    - Limited resolution during integration (time increments, measurement resolution, etc.);
    - Misalignment of the wheels (deterministic);
    - Uncertainty in the wheel diameter and in particular unequal wheel diameter (deterministic);
    - Variation in the contact point of the wheel;
    - Unequal floor contact (slipping, nonplanar surface, etc.).
    
      So we use error propagation to estimated covarience of the odom pose
    
      ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/fa01ff4c-7d7d-480c-a772-22caa73f85d1)
    
      where:
      
      C_x = covariance matrix representing the input uncertainties
      
      C_y = covariance matrix representing the propagated uncertainties for the outputs
      
      F_x = is the Jacobian matrix of the vehicle pose model f

      Giving us the pose covarience estimate:

      ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/3d8dff40-e171-4ce8-99a5-6c94b0df415a)

      where:

      ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/bd0ce4da-91bd-463c-a673-f9b46b7e429d)

      ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/2cd37ea6-8239-443f-94e8-f9ce03e5e4f5)

      ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/46ff8440-9b3c-4bb0-927a-1a9768d6d0bf)

       The values for the errors konstant k_r and k_l depend on the robot and the environment and should be exprerimentally established by performing and analyzing representative movements.

       Fully explaination about error propagation see in this [link](https://www.youtube.com/watch?v=ubg_AAM7Zd8) 

  - **Reading covariance and offset from the file and publish IMU topic**

- **Robot Description**
  The primary purpose of the `skt_description` package is to provide a comprehensive description of the robot's physical structure, including its base, caster frame, sensor frames, and any additional components required for visualization and simulation within the ROS environment.
  
    To view robot_frame:
    ```cpp
    ros2 run tf2_tools view_frames 
    ```
   ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/b503afc6-1b7a-4f30-8080-b16d1cc28ea2)


- **robot_localization configuration**
    In configuring [robot\_localization](http://docs.ros.org/en/melodic/api/robot\_localization/html/configuring\_robot\_localization.html), we strategically reduce the information fused by eliminating redundant data such as $\( X \)$ and $\( Y \)$ positions and using only $\( X \)$ and $\( Y \)$ velocities $\( v_x \)$ and $\( v_y \)$, derived from the same source. We enable the fusion of $\( v_y \)$ to allow for error estimation in the $\( Y \)$-axis. Additionally, we incorporate absolute yaw orientation from odometry data to complement $\( w_z \)$ (yaw rate) from the IMU, acknowledging the inherent uncertainties in mechanical systems and IMU yaw readings affected by hard iron and soft iron effects. We include $\( a_x \)$ to estimate system slip. The configuration parameters are as follows:
    
    ```yaml
    odom0: example/odom
    odom0_config: [false,  false,  false,
                   false, false, true,
                   true, true, false,
                   false, false, true,
                   false, false, false]
    
    imu0: example/imu
    imu0_config: [false, false, false,
                  false,  false,  true,
                  false, false, false,
                  false,  false,  true,
                  true,  false,  false]
    ```

## Testing and Laboratories

- **Getting absolute position of the robot**

  Using a calibration stick the same height as the robot tracking point, we place it at various position on X and Y axis and take the picture. Then using the Tracker software, we find out the top position and the base position of the stick
  
  ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/83bad1e8-6a14-48f5-bfd0-e081711f1324)

  Then using a spreadsheet to plot and calculate the correction equation, we can now use the formular to compensate for the height of the robot.

  ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/0a9d7b48-c997-4915-9857-c76cef382d67)

  Compare with the Perspective projection <-> Orthographic Projection [see more](https://github.com/Kangonaut/simple-perspective-projection)
  
  ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/122732439/510b9612-62b2-4aee-ba41-69ff62c7cc1e)

  We can see that the ratio form ideal and real world experiment are close.
- **Finding relationship between command and velocity of Dynamixel motors**

  A simple code that detects motor position wrap-around and record the amount of counts and time is used to find the frequency of the motor.

  ```cpp
  int M1_pos_last = 0;
  int trig_count = 0;

  void loop()
  {
    int M1_pos = Motor.readPosition(1);
    if (abs(M1_pos - M1_pos_last) > 150) {
      trig_count++;
      sprintf(buffer, "M1 Triggered count[%d] millis[%lu] speed[%i]", trig_count, millis(), Motor.readSpeed(1));
      Serial.println(buffer);
    }
    M1_pos_last = M1_pos;
  }
  ```

  This allows us to record and calculate the formular for the motor speed as follow:

  ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/2b8b7c23-bc8f-4e88-9a38-10a52e235492)

  <br>

- **Finding relationship between actual position and wheel odometry (Unit in meters)**

  | Position | Error |
  |:-:|:-:|
  | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/06c11942-fff8-4c76-b18b-1049c45fb6b0) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/666da6ff-ffc3-4557-953d-b2a5ebfe408b) |
  | Cumulative Error: | X: -1.5370 m Y: -0.5319 m |

  <br>

- **Finding relationship between actual position `/cmd_vel` (Unit in meters)**

  | Position | Error |
  |:-:|:-:|
  | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/d33fb052-adb8-42aa-8ea6-181b282ccf5e) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/f3031894-f629-4837-b90d-b73ae2c78699) |
  | Cumulative Error: | X: 0.8861 m Y: 1.0993 m |

- **Adjusting covariance of wheel odometry and IMU & EKF configurations (Unit in meters)**

  | Values | Line | Circle | C |
  | :-: | :-: | :-: | :-: |
  | odom_vx_cov : 1.0e-6 odom_wz_cov : 1.0e-6 kr,kl : 1.0 e-10| ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/433212f1-d448-44cd-8283-0c4a7c65e056) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/fdd2dd0e-4677-4e1a-a05d-0cd0a51b0e62) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/c0c2a8b8-c535-429d-9efd-2ee210c10fd5) |
  | odom_vx_cov : 1.0e-9 odom_wz_cov : 1.0e-6 kr,kl : 1.0 e-9| ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/32db5ea5-cf63-4798-b6af-742b2e79466a) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/0153dafa-ad13-4f16-8508-32f1762064a5) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/2da30337-6e15-4c26-8bd9-e2c32dca831d) |
  | odom_vx_cov : 1.0e-12 odom_wz_cov : 1.0e-6 kr,kl : 1.0 e-8| ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/f3cb8918-ae76-4e6c-99a8-d38ef9ca3924) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/da4d3c4f-cdc2-48a6-b379-96575124cb50) | ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/07a004b8-f41b-4644-8d8e-00954eb2b5d4) |
