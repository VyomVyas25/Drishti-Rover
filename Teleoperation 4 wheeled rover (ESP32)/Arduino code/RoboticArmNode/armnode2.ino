#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#define MOTOR1_PWM_PIN 19
#define MOTOR1_DIR_PIN 18
#define MOTOR2_PWM_PIN 32// tx/rx
#define MOTOR2_DIR_PIN 33
#define MOTOR3_PWM_PIN 27
#define MOTOR3_DIR_PIN 14 // tx/rx
#define MOTOR4_PWM_PIN 25
#define MOTOR4_DIR_PIN 26
#define MOTOR5_PWM_PIN 12 //  tx / rx
#define MOTOR5_DIR_PIN 13
#define STEPPER_DIR_PIN 22
#define STEPPER_STEP_PIN 4
#define enpin 23

rcl_subscription_t subscriber;
rcl_publisher_t motor_pwm_publisher;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Float32MultiArray motor_pwm_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
    while (1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

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
    pinMode(enpin, OUTPUT);

    ledcSetup(0, 5000, 8); // Channel 0, 5 kHz PWM, 8-bit resolution
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

void controlMotors(const geometry_msgs__msg__Twist *cmd) {
    // Map velocity inputs (-125 to 125) to motor controls
    int motor1_speed = abs(cmd->linear.x);
    int motor2_speed = abs(cmd->linear.y);
    int motor3_speed = abs(cmd->linear.z);
    int motor4_speed = abs(cmd->angular.x);
    int motor5_speed = abs(cmd->angular.y);
    int stepper_speed = abs(cmd->angular.z);

    digitalWrite(MOTOR1_DIR_PIN, cmd->linear.x >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR2_DIR_PIN, cmd->linear.y >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR3_DIR_PIN, cmd->linear.z >= 0 ? LOW : HIGH);
    digitalWrite(MOTOR4_DIR_PIN, cmd->angular.x >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR5_DIR_PIN, cmd->angular.y >= 0 ? LOW : HIGH);
    digitalWrite(STEPPER_DIR_PIN, cmd->angular.z >= 0 ? HIGH : LOW);
    digitalWrite(enpin, LOW);

    ledcWrite(0, motor1_speed);
    ledcWrite(1, motor2_speed);
    ledcWrite(2, motor3_speed);
    ledcWrite(3, motor4_speed);
    ledcWrite(4, motor5_speed);

    // Publish motor PWM values
    motor_pwm_msg.data.data[0] = motor1_speed;
    motor_pwm_msg.data.data[1] = motor2_speed;
    motor_pwm_msg.data.data[2] = motor3_speed;
    motor_pwm_msg.data.data[3] = motor4_speed;
    motor_pwm_msg.data.data[4] = motor5_speed;
    motor_pwm_msg.data.size = 5;
    RCSOFTCHECK(rcl_publish(&motor_pwm_publisher, &motor_pwm_msg, NULL));

    // Control stepper motor speed using a delay
    for (int i = 0; i < stepper_speed/4; ++i) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(150);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(150);
    }
}

void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    controlMotors(msg);
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
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel_1"));

    // Initialize motor PWM publisher
    RCCHECK(rclc_publisher_init_default(&motor_pwm_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_pwm"));

    // Initialize executor and add subscription
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    // Initialize motor PWM message
    motor_pwm_msg.data.capacity = 5;
    motor_pwm_msg.data.size = 0;
    motor_pwm_msg.data.data = (float *)malloc(5 * sizeof(float));
    if (motor_pwm_msg.data.data == NULL) {
        error_loop();
    }

    setupMotors();
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
