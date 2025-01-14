#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <mutex>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define NANO_SECS 1000000000

#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define WHEEL_RADIUS 0.035        // m
#define WHEEL_SEPARATION_BY_2 0.5 // m

using std::mutex;

// Motor Control
float leftWheelSpeed = 0;  // rad / s
float rightWheelSpeed = 0; // rad / s
int leftMotorPWM;          // pulse width in microseconds (us)
int rightMotorPWM;         // pulse width in microseconds (us)
float angularComponent;

// MPU control/status vars
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aaWorld, ggWorld;
VectorFloat gravity;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int32MultiArray encoder_msg;

// Encoder and motor pins
#define LEFT_ENCODER_PIN1 15
#define LEFT_ENCODER_PIN2 13
#define RIGHT_ENCODER_PIN1 34
#define RIGHT_ENCODER_PIN2 35

#define LEFT_MOTOR_PIN1 18
#define LEFT_MOTOR_PIN2 17
#define RIGHT_MOTOR_PIN1 5
#define RIGHT_MOTOR_PIN2 19

#define STBY_PIN 22
#define MODE_PIN 23

int32_t left_encoder_value = 0, right_encoder_value = 0;

rcl_publisher_t imu_publisher, encoder_publisher;
rcl_subscription_t twist_subscriber;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

void publish_imu_task(void *param)
{
  while (1)
  {
    if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetAccel(&aaWorld, fifoBuffer);
      mpu.dmpGetGyro(&ggWorld, fifoBuffer);

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

      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void publish_encoder_task(void *param)
{
  while (1)
  {
    encoder_msg.data.data[0] = left_encoder_value;
    encoder_msg.data.data[1] = right_encoder_value;

    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void cmd_vel_task(void *param)
{
  geometry_msgs__msg__Twist twist_msg;
  while (1)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    angularComponent = WHEEL_SEPARATION_BY_2 * twist_msg.angular.z;

    if (twist_msg.linear.x == 0 && twist_msg.angular.z == 0)
    {
      analogWrite(LEFT_MOTOR_PIN1, 0);
      analogWrite(RIGHT_MOTOR_PIN1, 0);
    }
    else
    {
      leftWheelSpeed = (twist_msg.linear.x + angularComponent) / WHEEL_RADIUS;
      rightWheelSpeed = (twist_msg.linear.x - angularComponent) / WHEEL_RADIUS;

      leftMotorPWM = map((int)leftWheelSpeed, -45, 45, -249, 249);
      rightMotorPWM = map((int)rightWheelSpeed, -45, 45, -249, 249);

      digitalWrite(LEFT_MOTOR_PIN2, leftWheelSpeed > 0 ? LOW : HIGH);
      digitalWrite(RIGHT_MOTOR_PIN2, rightWheelSpeed > 0 ? LOW : HIGH);
      analogWrite(LEFT_MOTOR_PIN1, abs(leftMotorPWM));
      analogWrite(RIGHT_MOTOR_PIN1, abs(rightMotorPWM));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup()
{
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(MODE_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);
  digitalWrite(STBY_PIN, HIGH);

  Wire.begin(27, 26);
  Wire.setClock(400000);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0)
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  pinMode(LEFT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN1), []() { left_encoder_value++; }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN1), []() { right_encoder_value++; }, CHANGE);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support);
  rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
  rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoders");
  rclc_subscription_init_default(&twist_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  xTaskCreate(publish_imu_task, "IMU Publisher", 4096, NULL, 1, NULL);
  xTaskCreate(publish_encoder_task, "Encoder Publisher", 2048, NULL, 1, NULL);
  xTaskCreate(cmd_vel_task, "Cmd Vel Subscriber", 2048, NULL, 1, NULL);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000)); // FreeRTOS handles the tasks, no need for loop execution
}
