#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Define LED_BUILTIN manually for ESP32
#define LED_BUILTIN 4  // Update to your board's LED pin if necessary

#define MOTOR_A_PWM_PIN 12
#define MOTOR_A_DIR_PIN 13
#define MOTOR_B_PWM_PIN 27
#define MOTOR_B_DIR_PIN 14




rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setupMotors() {
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_A_DIR_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_DIR_PIN, OUTPUT);
}

void controlMotors(float linear_x, float angular_z) {
  int speedA = (int)(255 * constrain(linear_x - angular_z, -1.0, 1.0));
  int speedB = (int)(255 * constrain(linear_x + angular_z, -1.0, 1.0));
  
  if (speedA > 0) {
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    speedA = -speedA;
  }
  
  if (speedB > 0) {
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    speedB = -speedB;
  }
  
  analogWrite(MOTOR_A_PWM_PIN, speedA);
  analogWrite(MOTOR_B_PWM_PIN, speedB);
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  controlMotors(msg->linear.x, msg->angular.z);
}
  
void setup() {
  set_microros_transports();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  setupMotors();
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
