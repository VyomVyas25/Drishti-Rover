#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// Motor pins for the first set of motors
#define MOTOR1_PWM_PIN 32 // drill
#define MOTOR1_DIR_PIN 33 
#define MOTOR2_PWM_PIN 25 // auger
#define MOTOR2_DIR_PIN 26
#define MOTOR3_PWM_PIN 19 // act
#define MOTOR3_DIR_PIN 18
#define STEPPER_DIR_PIN 22
#define STEPPER_STEP_PIN 4
#define enpin 23

// Motor pins for the second set of motors
#define MOTOR_A_PWM_PIN 12
#define MOTOR_A_DIR_PIN 13
#define MOTOR_B_PWM_PIN 27
#define MOTOR_B_DIR_PIN 14

// ROS objects
rcl_subscription_t subscriber1, subscriber2;
rcl_publisher_t motor_pwm_publisher;
geometry_msgs__msg__Twist msg1, msg2;
std_msgs__msg__Float32MultiArray motor_pwm_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Macros for error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Error handling loop
void error_loop() {
    while (1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// Setup motors for the first set
void setupMotorsSet1() {
    pinMode(MOTOR1_PWM_PIN, OUTPUT);
    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_PWM_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR3_PWM_PIN, OUTPUT);
    pinMode(MOTOR3_DIR_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(enpin, OUTPUT);

    ledcSetup(0, 5000, 8);
    ledcSetup(1, 5000, 8);
    ledcSetup(2, 5000, 8);

    ledcAttachPin(MOTOR1_PWM_PIN, 0);
    ledcAttachPin(MOTOR2_PWM_PIN, 1);
    ledcAttachPin(MOTOR3_PWM_PIN, 2);
}

// Setup motors for the second set
void setupMotorsSet2() {
    pinMode(MOTOR_A_PWM_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_PWM_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);
}

// Control logic for the first set of motors
void controlMotorsSet1(const geometry_msgs__msg__Twist *cmd) {
    int motor1_speed = abs(cmd->linear.x);
    int motor2_speed = abs(cmd->linear.y);
    int motor3_speed = abs(cmd->linear.z);
    int stepper_speed = abs(cmd->angular.x);

    digitalWrite(MOTOR1_DIR_PIN, cmd->linear.x >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR2_DIR_PIN, cmd->linear.y >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR3_DIR_PIN, cmd->linear.z >= 0 ? HIGH : LOW);
    digitalWrite(STEPPER_DIR_PIN, cmd->angular.x >= 0 ? HIGH : LOW);
    digitalWrite(enpin, LOW);

    ledcWrite(0, motor1_speed);
    ledcWrite(1, motor2_speed);
    ledcWrite(2, motor3_speed);

    // Stepper motor control
    for (int i = 0; i < stepper_speed/8; ++i) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(500);
    }

    motor_pwm_msg.data.data[0] = motor1_speed;
    motor_pwm_msg.data.data[1] = motor2_speed;
    motor_pwm_msg.data.data[2] = motor3_speed;
    motor_pwm_msg.data.data[3] = stepper_speed;
    motor_pwm_msg.data.size = 4;

    RCSOFTCHECK(rcl_publish(&motor_pwm_publisher, &motor_pwm_msg, NULL));
}

// Control logic for the second set of motors
void controlMotorsSet2(float linear_x, float angular_z) {
    int speedA = (int)(255 * constrain(linear_x - angular_z, -1.0, 1.0));
    int speedB = (int)(255 * constrain(linear_x + angular_z, -1.0, 1.0));

    digitalWrite(MOTOR_A_DIR_PIN, speedA >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_B_DIR_PIN, speedB >= 0 ? HIGH : LOW);

    analogWrite(MOTOR_A_PWM_PIN, abs(speedA));
    analogWrite(MOTOR_B_PWM_PIN, abs(speedB));
}

// Subscription callback for the first set
void subscription_callback_set1(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    controlMotorsSet1(msg);
}

// Subscription callback for the second set
void subscription_callback_set2(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    controlMotorsSet2(msg->linear.x, msg->angular.z);
}

void setup() {
    Serial.begin(115200);
    set_microros_transports();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    RCCHECK(rclc_subscription_init_default(&subscriber1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel_1"));
    RCCHECK(rclc_subscription_init_default(&subscriber2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

    RCCHECK(rclc_publisher_init_default(&motor_pwm_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_pwm"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg1, &subscription_callback_set1, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg2, &subscription_callback_set2, ON_NEW_DATA));

    motor_pwm_msg.data.capacity = 4;
    motor_pwm_msg.data.size = 0;
    motor_pwm_msg.data.data = (float *)malloc(4 * sizeof(float));
    if (motor_pwm_msg.data.data == NULL) {
        error_loop();
    }

    setupMotorsSet1();
    setupMotorsSet2();
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
