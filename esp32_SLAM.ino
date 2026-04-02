#ifndef ESP32
#error This example runs on ESP32
#endif

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <SCServo.h>
#include <LDS_LDROBOT_LD14P.h>

// ── WiFi ──────────────────────────────────────────────────────────────────────
char ssid[]         = "SSID";  //Put your laptop's WIFI's SSID
char password[]     = "PASS";  //Put your laptop's WIFI's password
char agent_ip[]     = "0.0.0.0"; //Put your laptop's IP address
uint32_t agent_port = 8888;

// ── Servo UART (Serial1) ──────────────────────────────────────────────────────
#define SERVO_UART_RX  16
#define SERVO_UART_TX  17
#define SERVO_BAUD     1000000
#define LEFT_ID        12
#define RIGHT_ID       11
#define MAX_SPEED      4000

// ── LIDAR UART (Serial2) ──────────────────────────────────────────────────────
#define LIDAR_RX_PIN   14
#define LIDAR_TX_PIN   15

// ── Servo ─────────────────────────────────────────────────────────────────────
SMS_STS servo;
HardwareSerial ServoSerial(1);

// ── LIDAR ─────────────────────────────────────────────────────────────────────
LDS_LDROBOT_LD14P lidar;
HardwareSerial LidarSerial(2);

#define SCAN_SIZE 360
float scan_ranges[SCAN_SIZE];
float scan_intensities[SCAN_SIZE];

// ── micro-ROS entities ────────────────────────────────────────────────────────
rcl_publisher_t    joint_state_pub;
rcl_publisher_t    scan_pub;
rcl_subscription_t joint_cmd_sub;

sensor_msgs__msg__JointState         joint_state_msg;
sensor_msgs__msg__LaserScan          laser_scan_msg;
std_msgs__msg__Float32MultiArray     joint_cmd_msg;

rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rclc_executor_t executor;

// ── Joint state storage ───────────────────────────────────────────────────────
double positions[2]  = {0.0, 0.0};
double velocities[2] = {0.0, 0.0};
double efforts[2]    = {0.0, 0.0};
float  cmd_data[2]   = {0.0, 0.0};

char left_name[]  = "left_wheel_joint";
char right_name[] = "right_wheel_joint";
rosidl_runtime_c__String joint_names[2];

// ── Timing ────────────────────────────────────────────────────────────────────
unsigned long last_joint_pub  = 0;
unsigned long last_ping_check = 0;
#define JOINT_PUB_INTERVAL 50
#define PING_INTERVAL      500   // ← increased from 2000 to 500ms

// ── Connection state ──────────────────────────────────────────────────────────
bool micro_ros_connected = false;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) return false; }

// ── Servo helpers ─────────────────────────────────────────────────────────────
void setWheelMode(int id) {
  servo.unLockEprom(id);
  delay(100);
  servo.WheelMode(id);
  delay(100);
  servo.LockEprom(id);
  delay(100);
  servo.EnableTorque(id, 0);
  delay(100);
  servo.EnableTorque(id, 1);
  delay(100);
}

int16_t radps_to_servo_speed(float radps) {
  const float steps_per_rad = 4096.0f / (2.0f * M_PI);
  int16_t spd = (int16_t)(radps * steps_per_rad);
  return constrain(spd, -MAX_SPEED, MAX_SPEED);
}

// ── Command callback ──────────────────────────────────────────────────────────
void joint_cmd_callback(const void * msg_in) {
  const std_msgs__msg__Float32MultiArray * cmd =
    (const std_msgs__msg__Float32MultiArray *)msg_in;
  if (cmd->data.size < 2) return;

  servo.WriteSpe(LEFT_ID,  -radps_to_servo_speed( cmd->data.data[0]));
  servo.WriteSpe(RIGHT_ID,  radps_to_servo_speed( cmd->data.data[1]));
}

// ── LIDAR callbacks ───────────────────────────────────────────────────────────
void publish_laser_scan() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  laser_scan_msg.header.stamp.sec     = ts.tv_sec;
  laser_scan_msg.header.stamp.nanosec = ts.tv_nsec;

  for (int i = 0; i < SCAN_SIZE; i++) {
    laser_scan_msg.ranges.data[i]      = scan_ranges[i];
    laser_scan_msg.intensities.data[i] = scan_intensities[i];
  }
  laser_scan_msg.ranges.size      = SCAN_SIZE;
  laser_scan_msg.intensities.size = SCAN_SIZE;

  rcl_publish(&scan_pub, &laser_scan_msg, NULL);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  if (distance_mm > 0 && distance_mm < 12000) {
    float dist_m = distance_mm / 1000.0;

    while (angle_deg >= 360.0) angle_deg -= 360.0;
    while (angle_deg < 0.0)    angle_deg += 360.0;

    int index = ((int)(360.0 - angle_deg)) % 360;
    if (index >= 0 && index < SCAN_SIZE) {
      scan_ranges[index]      = dist_m;
      scan_intensities[index] = quality;
    }
  }

  if (scan_completed) {
    if (micro_ros_connected) {
      publish_laser_scan();
    }
    for (int i = 0; i < SCAN_SIZE; i++) {
      scan_ranges[i]      = 12.0;
      scan_intensities[i] = 0;
    }
  }
}

void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {}

int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

// ── Joint state publisher ─────────────────────────────────────────────────────
void publish_joint_states() {
  if (millis() - last_joint_pub < JOINT_PUB_INTERVAL) return;
  last_joint_pub = millis();

  static int16_t prev_left_raw  = -1;
  static int16_t prev_right_raw = -1;
  static double  left_total     = 0.0;
  static double  right_total    = 0.0;

  int left_fb  = servo.FeedBack(LEFT_ID);
  int right_fb = servo.FeedBack(RIGHT_ID);

  if (left_fb != -1) {
    int16_t left_raw = servo.ReadPos(LEFT_ID);
    if (prev_left_raw == -1) prev_left_raw = left_raw;

    int16_t left_delta = left_raw - prev_left_raw;
    if (left_delta >  2048) left_delta -= 4096;
    if (left_delta < -2048) left_delta += 4096;

    left_total -= (left_delta / 4096.0) * 2.0 * M_PI;
    prev_left_raw = left_raw;

    positions[0]  = left_total;
    velocities[0] = -(servo.ReadSpeed(LEFT_ID) / (4096.0 / (2.0 * M_PI)));
  }

  if (right_fb != -1) {
    int16_t right_raw = servo.ReadPos(RIGHT_ID);
    if (prev_right_raw == -1) prev_right_raw = right_raw;

    int16_t right_delta = right_raw - prev_right_raw;
    if (right_delta >  2048) right_delta -= 4096;
    if (right_delta < -2048) right_delta += 4096;

    right_total += (right_delta / 4096.0) * 2.0 * M_PI;
    prev_right_raw = right_raw;

    positions[1]  = right_total;
    velocities[1] = -(servo.ReadSpeed(RIGHT_ID) / (4096.0 / (2.0 * M_PI)));
  }

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  joint_state_msg.header.stamp.sec     = ts.tv_sec;
  joint_state_msg.header.stamp.nanosec = ts.tv_nsec;

  rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
}

// ── micro-ROS entity management ───────────────────────────────────────────────
bool create_ros_entities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    return false;
  if (rclc_node_init_default(&node, "robot_node", "", &support) != RCL_RET_OK)
    return false;

  RCCHECK(rclc_publisher_init_default(
    &joint_state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));

  RCCHECK(rclc_publisher_init_default(
    &scan_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"));

  joint_cmd_msg.data.data     = cmd_data;
  joint_cmd_msg.data.size     = 2;
  joint_cmd_msg.data.capacity = 2;

  RCCHECK(rclc_subscription_init_default(
    &joint_cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "joint_commands"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &joint_cmd_sub, &joint_cmd_msg,
    &joint_cmd_callback, ON_NEW_DATA));

  // Sync ESP32 clock with agent before any publishing
  RCCHECK(rmw_uros_sync_session(1000));

  return true;
}

void destroy_ros_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&joint_state_pub, &node);
  rcl_publisher_fini(&scan_pub, &node);
  rcl_subscription_fini(&joint_cmd_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  // Servo init
  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_UART_RX, SERVO_UART_TX);
  servo.pSerial = &ServoSerial;
  delay(1000);
  setWheelMode(LEFT_ID);
  setWheelMode(RIGHT_ID);

  // LIDAR init
  LidarSerial.setRxBufferSize(1024);
  LidarSerial.begin(lidar.getSerialBaudRate(), SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setPacketCallback(lidar_packet_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.init();
  lidar.start();

  // Message metadata setup
  joint_names[0].data     = left_name;
  joint_names[0].size     = strlen(left_name);
  joint_names[0].capacity = strlen(left_name) + 1;
  joint_names[1].data     = right_name;
  joint_names[1].size     = strlen(right_name);
  joint_names[1].capacity = strlen(right_name) + 1;

  joint_state_msg.name.data     = joint_names;
  joint_state_msg.name.size     = 2;
  joint_state_msg.name.capacity = 2;
  joint_state_msg.position.data     = positions;
  joint_state_msg.position.size     = 2;
  joint_state_msg.position.capacity = 2;
  joint_state_msg.velocity.data     = velocities;
  joint_state_msg.velocity.size     = 2;
  joint_state_msg.velocity.capacity = 2;
  joint_state_msg.effort.data     = efforts;
  joint_state_msg.effort.size     = 2;
  joint_state_msg.effort.capacity = 2;

  joint_state_msg.header.frame_id.data     = "base_link";
  joint_state_msg.header.frame_id.size     = strlen("base_link");
  joint_state_msg.header.frame_id.capacity = strlen("base_link") + 1;

  laser_scan_msg.header.frame_id.data     = "base_link";
  laser_scan_msg.header.frame_id.size     = strlen("base_link");
  laser_scan_msg.header.frame_id.capacity = strlen("base_link") + 1;

  laser_scan_msg.angle_min       = 0.0;
  laser_scan_msg.angle_max       = 2 * PI;
  laser_scan_msg.angle_increment = (2 * PI) / SCAN_SIZE;
  laser_scan_msg.time_increment  = 0.0;
  laser_scan_msg.scan_time       = 0.2;
  laser_scan_msg.range_min       = 0.02;
  laser_scan_msg.range_max       = 12.0;

  laser_scan_msg.ranges.data          = scan_ranges;
  laser_scan_msg.ranges.capacity      = SCAN_SIZE;
  laser_scan_msg.intensities.data     = scan_intensities;
  laser_scan_msg.intensities.capacity = SCAN_SIZE;

  for (int i = 0; i < SCAN_SIZE; i++) {
    scan_ranges[i]      = 12.0;
    scan_intensities[i] = 0;
  }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  lidar.loop();
  lidar.loop();
  lidar.loop();

  unsigned long now = millis();

  switch (micro_ros_connected) {
    case false:
      if (now - last_ping_check >= PING_INTERVAL) {
        last_ping_check = now;
        if (rmw_uros_ping_agent(200, 3) == RMW_RET_OK) {
          if (create_ros_entities()) {
            micro_ros_connected = true;
          } else {
            destroy_ros_entities();
          }
        }
      }
      break;

    case true:
      if (now - last_ping_check >= PING_INTERVAL) {
        last_ping_check = now;
        if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) {
          destroy_ros_entities();
          micro_ros_connected = false;
          servo.WriteSpe(LEFT_ID,  0);
          servo.WriteSpe(RIGHT_ID, 0);
          break;
        }
        rmw_uros_sync_session(200);  // ← increased from 100 to 200
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      publish_joint_states();
      break;
  }
}
