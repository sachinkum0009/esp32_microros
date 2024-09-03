#include <Arduino.h>
#include <micro_ros_platformio.h>
// #include <micro_ros_transport.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#define NANO_SECS 1000000000

// #include <MPU6050.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665 // m/s2
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gg;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

/**
 * motor pins code
 */
#define LEFT_MOTOR_PIN1 18
#define LEFT_MOTOR_PIN2 17
#define RIGHT_MOTOR_PIN1 5
#define RIGHT_MOTOR_PIN2 19
// #define LEFT_ENABLE_PIN 14
// #define RIGHT_ENABLE_PIN 15

#define STBY_PIN 22
#define MODE_PIN 23

// Setting PWM properties
// const int frequencies = 30000;
// const int pwmChannelLeft = 0;
// const int pwmChannelRight = 1;
// const int resolution = 8;
// int dutycycle = 200;

/**
 * Encoders
 *
 */

#define LEFT_ENCODER_PIN1 13
#define LEFT_ENCODER_PIN2 15
#define RIGHT_ENCODER_PIN1 34
#define RIGHT_ENCODER_PIN2 35

int32_t left_encoder_value, right_encoder_value;

rcl_publisher_t publisher, imu_publisher, encoder_publisher;
rcl_subscription_t twist_subscriber;
std_msgs__msg__Int32 msg;

sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32MultiArray encoder_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    /**
     * IMU Sensor Values
     */
    if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);

      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);

      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetGyro(&gg, fifoBuffer);
      mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
      // Serial.print("ggWorld\t");
      // Serial.print(ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD);
      // Serial.print("\t");
      // Serial.print(ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD);
      // Serial.print("\t");
      // Serial.println(ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD);

      // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    imu_msg.header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / NANO_SECS);
    imu_msg.header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % NANO_SECS);
    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;
    imu_msg.orientation.w = q.w;

    imu_msg.angular_velocity.x = ggWorld.x;
    imu_msg.angular_velocity.y = ggWorld.y;
    imu_msg.angular_velocity.z = ggWorld.z;

    imu_msg.linear_acceleration.x = aaWorld.x;
    imu_msg.linear_acceleration.y = aaWorld.y;
    imu_msg.linear_acceleration.z = aaWorld.z;

    // encoder_msg.data.data[0] += 1;
    // encoder_msg.data.data[1] += 1;
    encoder_msg.data.data[0] = left_encoder_value;

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
  }
}

// subscription callback
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  // printf("Received linear x: %f, angular z: %f \n",  (float)  msg->linear.x, (float) msg->angular.z);
  // Map linear.x to motor speed and angular.z to motor direction
  int speed = map((int)(msg->linear.x * 100), -100, 100, -255, 255);
  int turn = map((int)(msg->angular.z * 100), -100, 100, -255, 255);

  // Determine motor control based on received messages
  if (speed > 0)
  {
    // Move forward
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  }
  else if (speed < 0)
  {
    // Move backward
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  }
  else
  {
    // Stop
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  }

  // Apply turn
  if (turn > 0)
  {
    // Turn right
    analogWrite(LEFT_MOTOR_PIN1, abs(speed) - turn);
    analogWrite(RIGHT_MOTOR_PIN1, abs(speed) + turn);
  }
  else if (turn < 0)
  {
    // Turn left
    analogWrite(LEFT_MOTOR_PIN1, abs(speed) + abs(turn));
    analogWrite(RIGHT_MOTOR_PIN1, abs(speed) - abs(turn));
  }
  else
  {
    // No turn, same speed for both motors
    analogWrite(LEFT_MOTOR_PIN1, abs(speed));
    analogWrite(RIGHT_MOTOR_PIN1, abs(speed));
  }
}

void initialize_ros_msgs()
{
  char *frame_id = new char[strlen("imu_link") + 1];
  strcpy(frame_id, "imu_link");
  imu_msg.header.frame_id.data = frame_id;

  float accel_cov_ = 0.00001;
  float gyro_cov_ = 0.00001;

  imu_msg.angular_velocity_covariance[0] = gyro_cov_;
  imu_msg.angular_velocity_covariance[4] = gyro_cov_;
  imu_msg.angular_velocity_covariance[8] = gyro_cov_;

  imu_msg.linear_acceleration_covariance[0] = accel_cov_;
  imu_msg.linear_acceleration_covariance[4] = accel_cov_;
  imu_msg.linear_acceleration_covariance[8] = accel_cov_;

  left_encoder_value = 0;
  right_encoder_value = 0;
  int32_t data[] = {left_encoder_value, right_encoder_value};
  encoder_msg.data.capacity = 2;
  encoder_msg.data.size = 2;
  encoder_msg.data.data = data;
}

static inline void doEncoderA()
{

  // look for a low-to-high on channel A
  if (digitalRead(LEFT_ENCODER_PIN1) == HIGH)
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(LEFT_ENCODER_PIN2) == LOW)
    {
      left_encoder_value = left_encoder_value + 1; // CW
    }
    else
    {
      left_encoder_value = left_encoder_value - 1; // CCW
    }
  }
  else // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(LEFT_ENCODER_PIN2) == HIGH)
    {
      left_encoder_value = left_encoder_value + 1; // CW
    }
    else
    {
      left_encoder_value = left_encoder_value - 1; // CCW
    }
  }
}

static inline void doEncoderB()
{
  // look for a low-to-high on channel B
  if (digitalRead(LEFT_ENCODER_PIN2) == HIGH)
  {
    // check channel A to see which way encoder is turning
    if (digitalRead(LEFT_ENCODER_PIN1) == HIGH)
    {
      left_encoder_value = left_encoder_value + 1; // CW
    }
    else
    {
      left_encoder_value = left_encoder_value - 1; // CCW
    }
  }
  // Look for a high-to-low on channel B
  else
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(LEFT_ENCODER_PIN1) == LOW)
    {
      left_encoder_value = left_encoder_value + 1; // CW
    }
    else
    {
      left_encoder_value = left_encoder_value - 1; // CCW
    }
  }
}

static inline void doEncoderC()
{

  // look for a low-to-high on channel A
  if (digitalRead(RIGHT_ENCODER_PIN1) == HIGH)
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(RIGHT_ENCODER_PIN2) == LOW)
    {
      right_encoder_value = right_encoder_value + 1; // CW
    }
    else
    {
      right_encoder_value = right_encoder_value - 1; // CCW
    }
  }
  else // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(RIGHT_ENCODER_PIN2) == HIGH)
    {
      right_encoder_value = right_encoder_value + 1; // CW
    }
    else
    {
      right_encoder_value = right_encoder_value - 1; // CCW
    }
  }
}

static inline void doEncoderD()
{
  // look for a low-to-high on channel B
  if (digitalRead(RIGHT_ENCODER_PIN2) == HIGH)
  {
    // check channel A to see which way encoder is turning
    if (digitalRead(RIGHT_ENCODER_PIN1) == HIGH)
    {
      right_encoder_value = right_encoder_value + 1; // CW
    }
    else
    {
      right_encoder_value = right_encoder_value - 1; // CCW
    }
  }
  // Look for a high-to-low on channel B
  else
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(RIGHT_ENCODER_PIN1) == LOW)
    {
      right_encoder_value = right_encoder_value + 1; // CW
    }
    else
    {
      right_encoder_value = right_encoder_value - 1; // CCW
    }
  }
}

void setup()
{

  pinMode(MODE_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  digitalWrite(MODE_PIN, HIGH); // Set MODE pin to L (closed) for IN input mode, H (opened) Phase input mode
  delay(10);
  digitalWrite(STBY_PIN, HIGH); // HIGH to enable the motor driver

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
                                // Wire.begin();
  Wire.begin(27, 26);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-156);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(-14);
  mpu.setXAccelOffset(-3699);
  mpu.setYAccelOffset(-2519);
  mpu.setZAccelOffset(1391); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  // Motor Driver
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);

  /**
   * encoders
   */

  pinMode(LEFT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN1), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN2), doEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN1), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN2), doEncoderD, CHANGE);

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  // 192.168.117.185
  // 172.22.72.27
  //   IPAddress agent_ip(172, 22, 72, 27);
  //   size_t agent_port = 8888;
  //   char ssid[] = "Xiaomi 13 Lite";
  //   char psk[]= "1234567890";

  //   set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  delay(2000);
  initialize_ros_msgs();

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));

  // create imu publisher
  RCCHECK(rclc_publisher_init_default(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu"));

  // create encoder publisher
  RCCHECK(rclc_publisher_init_default(
      &encoder_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "encoders"));

  // Create subscriber.
  RCCHECK(rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // add subscriber to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));

  msg.data = 0;
}

void loop()
{
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}