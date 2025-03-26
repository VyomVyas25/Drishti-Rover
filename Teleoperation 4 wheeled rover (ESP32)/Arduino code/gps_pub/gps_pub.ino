#include <micro_ros_arduino.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// Define pin numbers for ESP32
#define LED_PIN 2 // GPIO 2
#define RX_PIN 16 // GPIO 16
#define TX_PIN 17 // GPIO 17

TinyGPSPlus gps;
SoftwareSerial ss(RX_PIN, TX_PIN);

rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (gps.location.isUpdated()) {
      char buffer[100];
      snprintf(buffer, sizeof(buffer), "LAT=%.6f LON=%.6f", gps.location.lat(), gps.location.lng());
      msg.data.data = buffer;
      msg.data.size = strlen(buffer);
      msg.data.capacity = sizeof(buffer);
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  ss.begin(9600);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));
  RCCHECK(rclc_publisher_init_default(&publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"gps_data"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data.data = (char *)malloc(100 * sizeof(char));
  msg.data.size = 0;
  msg.data.capacity = 100;
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(100);
}
