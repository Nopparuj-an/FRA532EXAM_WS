#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "MPU9250.h"
#include "Motor.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// ======================================== FUNCTION DECLARATIONS ========================================

void TaskOther(void *pvParameters);
void TaskMicroROS(void *pvParameters);
void error_loop();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void publish_imu();
void cmd_vel_subscription_callback(const void * msgin);
void publish_wheelspeed();

// ============================================== MICRO ROS ==============================================

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t imu_publisher;
std_msgs__msg__Float32MultiArray imu_msg;

rcl_publisher_t wheel_speeds_publisher;
std_msgs__msg__Float32MultiArray wheel_speeds_msg;

// micro ROS objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t defaultTimer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ================================================ OTHER ================================================

TaskHandle_t Task0;
TaskHandle_t Task1;

SemaphoreHandle_t sem_imu;
SemaphoreHandle_t sem_motor_run;
SemaphoreHandle_t sem_motor_speed;

float speed_linear = 0;
float speed_angular = 0;
float speed_left = 0;
float speed_right = 0;

MPU9250 mpu;
float ax, ay, az, gx, gy, gz, mx, my, mz;

#define DirectionPin 4
#define MotorBaudRate 115200

// Error handle loop
void error_loop() {
    pinMode(16, OUTPUT);
    while(1) {
        digitalWrite(16, !digitalRead(16)); // Toggle GPIO 16
        delay(100);
    }
}

// ================================================ TASKS ================================================

// Other Task
void TaskOther(void *pvParameters) {
  while (1) {
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

    // Run Motor
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

    // Read Motor Speed
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

    // Yeild to other core
    vTaskDelay(1);
  }
}

// Micro-ROS Task
void TaskMicroROS(void *pvParameters) {
  // Initialize micro-ROS
  // set_microros_serial_transports(Serial);
  IPAddress ip(192, 168, 12, 1);
  set_microros_wifi_transports("TrashX", "00000000", ip, 8888);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // sync time
  rmw_uros_sync_session(1000);

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
  &imu_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/imu"));

  RCCHECK(rclc_publisher_init_best_effort(
  &wheel_speeds_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/wheel_speeds"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
  &cmd_vel_subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(
  &defaultTimer,
  &support,
  RCL_MS_TO_NS(timer_timeout),
  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); // DON'T FOTGET: Increase the number of handles
  RCCHECK(rclc_executor_add_timer(&executor, &defaultTimer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_subscription_callback, ON_NEW_DATA));

  while (1) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    delay(1);
  }
}

// ================================================ CALLBACKS ================================================

// Timer callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    publish_imu();
    publish_wheelspeed();
  }
}

void cmd_vel_subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  speed_linear = msg->linear.x;
  speed_angular = msg->angular.z;
  xSemaphoreGive(sem_motor_run);
}

// ================================================ PUBLISHERS ================================================

void publish_imu() {
    // read 9 dof of MPU9250
    // transmit to micro-ROS as 11 values in a Float32MultiArray
    // [time_MSB, time_LSB, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z]

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

// ================================================ MAIN ================================================

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Motor.begin(MotorBaudRate, DirectionPin, &Serial2);

  if (!mpu.setup(0x68)) {
    // MPU connection failed
    error_loop();
  }
  mpu.setMagneticDeclination(-0.53);

  sem_imu = xSemaphoreCreateMutex();
  sem_motor_run = xSemaphoreCreateBinary();
  sem_motor_speed = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(TaskMicroROS, "MicroROS Task", 30000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(TaskOther, "Other Task", 10000, NULL, 1, &Task0, 1);
}

void loop() {
  // Do nothing
  delay(INT32_MAX);
}
