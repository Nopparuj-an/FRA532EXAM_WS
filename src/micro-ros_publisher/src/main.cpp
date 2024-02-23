#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "MPU9250.h"
#include "Motor.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

// micro ROS objects
rcl_publisher_t defaultPublisher;
std_msgs__msg__Int32 defaultMsg;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// micro ROS objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t defaultTimer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// other objects
MPU9250 mpu;

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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&defaultPublisher, &defaultMsg, NULL));
    defaultMsg.data++;
  }
}

void cmd_vel_subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // Process the incoming message here
}

void setup() {
    Serial.begin(115200);
    // set_microros_serial_transports(Serial);
    IPAddress ip(192, 168, 12, 1);
    set_microros_wifi_transports("TrashX", "00000000", ip, 8888);
    
    Wire.begin();
    Motor.begin(MotorBaudRate, DirectionPin, &Serial2);
    delay(2000);

    if (!mpu.setup(0x68)) {
        // MPU connection failed
        error_loop();
    }

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
    &defaultPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
    &defaultTimer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // DON'T FOTGET: Increase the number of handles
    RCCHECK(rclc_executor_add_timer(&executor, &defaultTimer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_subscription_callback, ON_NEW_DATA));

    defaultMsg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
