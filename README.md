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
    
    The microcontroller is capable of connecting to the Micro-ROS agent.
  
    ![image](https://github.com/Nopparuj-an/FRA532EXAM_WS/assets/47713359/bb2bea8a-add6-4df5-b29c-450aaf60f8c7)

- **IMU Read and Publish**

    PLACEHOLDER

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
