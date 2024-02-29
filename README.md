# FRA532 Mobile Robot: Exam I

PLACEHOLDER

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

- **IMU Calibration Node**

  PLACEHOLDER

  - **Calculating covariance**

    PLACEHOLDER

  - **Saving covariance and offsets to `.yaml` file**

    PLACEHOLDER

- **Robot Bridge Node**

  PLACEHOLDER

  - **Calculating odometry and covariance from wheel angular velocity**

    Odom pose covarience  

  - **Reading covariance and offset from the file and publish IMU topic**

    PLACEHOLDER

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


<br>

## Testing and Laboratories

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

- **Finding relationship between actual position and wheel odometry**

  PLACEHOLDER

- **Finding relationship between actual position `/cmd_vel`**

  PLACEHOLDER

- **Adjusting covariance of wheel odometry and IMU**

  PLACEHOLDER

- **Adjusting EKF configurations**

  PLACEHOLDER
