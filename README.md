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
    // Read IMU
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

    | Index | Size | Data |
    |:---:|:---:|:---:|
    | 0 | 32 | Time (ms) << 32 |
    | 1 | 32 | Time (ms) LSB |
    | 2 | 32 | Linear Acceleration X |
    | 3 | 32 | Linear Acceleration Y |
    | 4 | 32 | Linear Acceleration Z |
    | 5 | 32 | Angular Velocity X |
    | 6 | 32 | Angular Velocity Y |
    | 7 | 32 | Angular Velocity Z |
    | 8 | 32 | Roll |
    | 9 | 32 | Pitch |
    | 10 | 32 | Yaw |

    The performance of the data transmission is over 46Hz

    ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/88512286-c456-41a0-87cb-81f2a6026298)

- **Motor Control by `/cmd_vel`**

    PLACEHOLDER

- **Motor Speed Read and Publish**

    PLACEHOLDER

- **Wheel Speed Kinematics**

    PLACEHOLDER

<br>

## Computer Programming

- **Sensor Calibration Node**

    PLACEHOLDER
  
    - **Calculating covariance**
 
        PLACEHOLDER
      
    - **Saving covariance and offsets to `.yaml` file**
 
        PLACEHOLDER

- **Robot Bridge Node**

    PLACEHOLDER
  
    - **Calculating odometry and covariance from wheel angular velocity**
 
        PLACEHOLDER
      
    - **Reading covariance and offset from the file and publish IMU topic**
 
        PLACEHOLDER

- **Robot Description**

    PLACEHOLDER

- **EKF Configuration**

    PLACEHOLDER

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
