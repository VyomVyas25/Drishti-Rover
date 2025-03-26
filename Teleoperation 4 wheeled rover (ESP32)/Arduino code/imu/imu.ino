#include <micro_ros_arduino.h>
#include <Wire.h>
#include <BNO055_support.h>  // Library for BNO055
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>  // Importing the Twist message type

rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg_twist;  // Create Twist message
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2
#define BNO055_I2C_ADDR2 0x29  // BNO055 I2C Address

struct bno055_t myBNO;
struct bno055_euler myEulerData;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // Read Euler angles from the BNO055
    bno055_read_euler_hrp(&myEulerData);

    // Convert the yaw value to degrees (since BNO055 returns data in Q16 format)
    float yaw = float(myEulerData.h) / 16.0;

    // Assign the yaw value to the linear.x field of the Twist message
    msg_twist.angular.x = yaw;

    // Publish the Twist message
    RCSOFTCHECK(rcl_publish(&publisher, &msg_twist, NULL));  // Publishing Twist message
  }
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize I2C and BNO055
  Wire.begin();
  BNO_Init(&myBNO);  // Initialize BNO055
  bno055_set_operation_mode(OPERATION_MODE_NDOF);  // Set BNO055 to NDoF mode

  // Create init options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_bno055_node", "", &support));

  // Create publisher for Twist message
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "bno055_twist"));

  // Create timer for periodic publishing
  const unsigned int timer_timeout = 1000;  // 1-second interval
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
