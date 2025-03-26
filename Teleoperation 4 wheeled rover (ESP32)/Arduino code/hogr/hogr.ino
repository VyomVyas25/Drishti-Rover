#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

#define MOTOR1_PWM_PIN 12
#define MOTOR1_DIR_PIN 13
#define MOTOR2_PWM_PIN 14
#define MOTOR2_DIR_PIN 27

#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
#define MAX_PWM 255

rcl_subscription_t subscriber;
sensor_msgs__msg__Joy msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void setMotor(int pwmChannel, int dirPin, bool direction, int speed) {
  digitalWrite(dirPin, direction);
  ledcWrite(pwmChannel, speed);
}

void stopMotor(int pwmChannel) {
  ledcWrite(pwmChannel, 0);
}

void subscription_callback(const void * msgin) {  
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;

  // Motor 1 Control
  if (joy_msg->buttons.data[0] == 1) { // X button: Motor 1 CW
    setMotor(MOTOR1_CHANNEL, MOTOR1_DIR_PIN, HIGH, MAX_PWM);
  } else if (joy_msg->buttons.data[1] == 1) { // O button: Motor 1 CCW
    setMotor(MOTOR1_CHANNEL, MOTOR1_DIR_PIN, LOW, MAX_PWM);
  } else {
    stopMotor(MOTOR1_CHANNEL); // Stop Motor 1
  }

  // Motor 2 Control
  if (joy_msg->buttons.data[2] == 1) { // Square button: Motor 2 CCW
    setMotor(MOTOR2_CHANNEL, MOTOR2_DIR_PIN, LOW, MAX_PWM);
  } else if (joy_msg->buttons.data[3] == 1) { // Triangle button: Motor 2 CW
    setMotor(MOTOR2_CHANNEL, MOTOR2_DIR_PIN, HIGH, MAX_PWM);
  } else {
    stopMotor(MOTOR2_CHANNEL); // Stop Motor 2
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  
  // Configure PWM for motors using ledcSetup
  ledcSetup(MOTOR1_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_PWM_PIN, MOTOR1_CHANNEL);
  ledcSetup(MOTOR2_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_PWM_PIN, MOTOR2_CHANNEL);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "joy"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}