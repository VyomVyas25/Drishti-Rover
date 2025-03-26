#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// Motor and Stepper Pin Definitions
#define MOTOR1_PWM_PIN 19
#define MOTOR1_DIR_PIN 18
#define MOTOR2_PWM_PIN 5
#define MOTOR2_DIR_PIN 21
#define MOTOR3_PWM_PIN 5
#define MOTOR3_DIR_PIN 21 // TX pin repurposed
#define MOTOR4_PWM_PIN 2
#define MOTOR4_DIR_PIN 15
#define MOTOR5_PWM_PIN 5 // RX pin repurposed
#define MOTOR5_DIR_PIN 4
#define STEPPER_DIR_PIN 22
#define STEPPER_STEP_PIN 23

// ROS 2-related Variables
rcl_subscription_t subscriber;
rcl_publisher_t motor_pwm_publisher;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Float32MultiArray motor_pwm_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Error Handling Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Error Loop
void error_loop() {
    while (1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// Function to Configure Motors and Stepper Pins
void setupMotors() {
    pinMode(MOTOR1_PWM_PIN, OUTPUT);
    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_PWM_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR3_PWM_PIN, OUTPUT);
    pinMode(MOTOR3_DIR_PIN, OUTPUT);
    pinMode(MOTOR4_PWM_PIN, OUTPUT);
    pinMode(MOTOR4_DIR_PIN, OUTPUT);
    pinMode(MOTOR5_PWM_PIN, OUTPUT);
    pinMode(MOTOR5_DIR_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);

    // PWM Setup
    ledcSetup(0, 5000, 8);
    ledcSetup(1, 5000, 8);
    ledcSetup(2, 5000, 8);
    ledcSetup(3, 5000, 8);
    ledcSetup(4, 5000, 8);

    ledcAttachPin(MOTOR1_PWM_PIN, 0);
    ledcAttachPin(MOTOR2_PWM_PIN, 1);
    ledcAttachPin(MOTOR3_PWM_PIN, 2);
    ledcAttachPin(MOTOR4_PWM_PIN, 3);
    ledcAttachPin(MOTOR5_PWM_PIN, 4);
}

// Function to Control Motors and Publish PWM Data
void controlMotors(const geometry_msgs__msg__Twist *cmd) {
    // Constrain motor speeds (-125 to 125) to prevent invalid values
    int motor1_speed = constrain(cmd->linear.x, -125, 125);
    int motor2_speed = constrain(cmd->linear.y, -125, 125);
    int motor3_speed = constrain(cmd->linear.z, -125, 125);
    int motor4_speed = constrain(cmd->angular.x, -125, 125);
    int motor5_speed = constrain(cmd->angular.y, -125, 125);
    int stepper_speed = constrain(cmd->angular.z, -125, 125);

    // Set motor directions and speeds
    digitalWrite(MOTOR1_DIR_PIN, motor1_speed >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR2_DIR_PIN, motor2_speed >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR3_DIR_PIN, motor3_speed >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR4_DIR_PIN, motor4_speed >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR5_DIR_PIN, motor5_speed >= 0 ? HIGH : LOW);

    ledcWrite(0, abs(motor1_speed));
    ledcWrite(1, abs(motor2_speed));
    ledcWrite(2, abs(motor3_speed));
    ledcWrite(3, abs(motor4_speed));
    ledcWrite(4, abs(motor5_speed));

    // Publish motor PWM values
    motor_pwm_msg.data.data[0] = motor1_speed;
    motor_pwm_msg.data.data[1] = motor2_speed;
    motor_pwm_msg.data.data[2] = motor3_speed;
    motor_pwm_msg.data.data[3] = motor4_speed;
    motor_pwm_msg.data.data[4] = motor5_speed;
    motor_pwm_msg.data.size = 5;

    RCSOFTCHECK(rcl_publish(&motor_pwm_publisher, &motor_pwm_msg, NULL));

    // Stepper Motor Control (non-blocking)
    static unsigned long last_step_time = 0;
    if (stepper_speed > 0 && millis() - last_step_time > (100000 / stepper_speed)) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(10); // Minimal step duration
        digitalWrite(STEPPER_STEP_PIN, LOW);
        last_step_time = millis();
    }
}

// Callback for Subscriber
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    Serial.print("Received Twist: linear.x = ");
    Serial.println(msg->linear.x);
    controlMotors(msg);
}

// Setup Function
void setup() {
    Serial.begin(115200);
    set_microros_transports(); // Initialize micro-ROS transport
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // Initialize micro-ROS
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel_1"));
    RCCHECK(rclc_publisher_init_default(&motor_pwm_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_pwm"));

    // Initialize executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // Allocate memory for motor PWM message
    motor_pwm_msg.data.capacity = 5;
    motor_pwm_msg.data.size = 0;
    motor_pwm_msg.data.data = (float *)malloc(5 * sizeof(float));
    if (motor_pwm_msg.data.data == NULL) {
        error_loop();
    }

    setupMotors();
}

// Main Loop
void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
