#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>

// create servo object to control a servo
Servo servo_1, servo_2, servo_3, servo_4, servo_5, servo_6;

// connect servo to ESP32 ( 23, 19, 18, 05, 17, 16 ) <-- verfügbare Pins ohne SCL(22)/ SDA(21) mit rücksicht auf Wlan (dont use ADC2 Pins by using wlan)
// pin 5 is schlecht weil  pwm impuls zum start
const int servoPin1 = 16;
const int servoPin2 = 17;
const int servoPin3 = 5;

const int servoPin4 = 18;
const int servoPin5 = 19;
const int servoPin6 = 23;



rcl_subscription_t servo_sub;
std_msgs__msg__UInt8MultiArray servo_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


void error_loop() {
  while (1) {
    delay(100);
  }
}


void servo_callback(const void* msgin) {
  const std_msgs__msg__UInt8MultiArray* msg = (const std_msgs__msg__UInt8MultiArray*)msgin;

  if (msg->data.size != 6) {
    printf("Fehler: Erwarte 6 Elemente in UInt8MultiArray, aber % zu erhalten.\n", msg->data.size);
    return;
  }

  u_int8_t angle1 = msg->data.data[0];
  u_int8_t angle2 = msg->data.data[1];
  u_int8_t angle3 = msg->data.data[2];
  u_int8_t angle4 = msg->data.data[3];
  u_int8_t angle5 = msg->data.data[4];
  u_int8_t angle6 = msg->data.data[5];

  servo_1.write(angle1);
  servo_2.write(angle2);
  servo_3.write(angle3);
  servo_4.write(angle4);
  servo_5.write(angle5);
  servo_6.write(angle6);
}



void setup() {
  set_microros_transports();
  // set_microros_wifi_transports("FRITZ!Box 7490", "54908635459129454475", "192.168.178.32", 8888);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo_1.setPeriodHertz(50);  // standard 50 hz servo
  servo_2.setPeriodHertz(50);
  servo_3.setPeriodHertz(50);
  servo_4.setPeriodHertz(50);
  servo_5.setPeriodHertz(50);
  servo_6.setPeriodHertz(50);
  servo_1.attach(servoPin1, 500, 2400);  // attaches the servo on pin 18 to the servo object
  servo_2.attach(servoPin2, 500, 2400);
  servo_3.attach(servoPin3, 500, 2400);
  servo_4.attach(servoPin4, 500, 2400);
  servo_5.attach(servoPin5, 500, 2400);
  servo_6.attach(servoPin6, 500, 2400);
  delay(2000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "servo_esp", "", &support));


  //servo_1 subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "/servos"));

  servo_msg.data.capacity = 6;
  servo_msg.data.size = 0;
  servo_msg.data.data = (u_int8_t*)malloc(servo_msg.data.capacity * sizeof(u_int8_t));

  servo_msg.layout.dim.capacity = 6;
  servo_msg.layout.dim.size = 0;
  servo_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(servo_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < servo_msg.layout.dim.capacity; i++) {
    servo_msg.layout.dim.data[i].label.capacity = 6;
    servo_msg.layout.dim.data[i].label.size = 0;
    servo_msg.layout.dim.data[i].label.data = (char*)malloc(servo_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA));
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
