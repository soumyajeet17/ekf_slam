#include <Arduino.h>
#include <WiFi.h>
#include <ros.h>

// ROS Message Headers
#include <my_slam_interfaces/Ticks.h>
#include <my_slam_interfaces/LandmarkArray.h>
#include <my_slam_interfaces/Landmark.h>
#include <std_msgs/String.h>

// LiDAR Library
#include "LDS_YDLIDAR_X2_X2L.h"
#include <vector>
#include <cmath>

// =================================================================
// ===           >>>  CONFIGURE YOUR ROBOT HERE  <<<             ===
// =================================================================

// --- Wi-Fi & ROS Server Configuration ---
const char* ssid = "*******";
const char* password = "*********";

//IPAddress server(192, 168, 0, 103);
IPAddress server(10, 42, 0, 1);
uint16_t serverPort = 11411;

// --- Hardware Pin Definitions ---
const int AIN1 = 14; const int AIN2 = 27;
const int BIN1 = 26; const int BIN2 = 25;
#define LEFT_ENC_A 34
#define RIGHT_ENC_A 32
const uint8_t LIDAR_EN_PIN  = 19;
const uint8_t LIDAR_PWM_PIN = 15;
const uint8_t LIDAR_TX_PIN  = 17;
const uint8_t LIDAR_RX_PIN  = 16;

// --- Landmark Detection Parameters ---
const float CLUSTER_MAX_DISTANCE_M = 0.10f;
const int MIN_POINTS_FOR_FIT = 5;

// Circle Validation Parameters
const float MAX_FIT_RMSE_M = 0.02f; // Increased for motion tolerance
const float MIN_CIRCLE_RADIUS_M = 0.02f;
const float MAX_CIRCLE_RADIUS_M = 0.55f;
// MODIFIED: Corrected to be less than the 30-degree FOV restriction
const float MIN_ARC_ANGLE_DEG = 45.0f;

// Set the desired range to focus landmark detection.
const float MIN_DETECTION_DISTANCE_M = 0.3f;
const float MAX_DETECTION_DISTANCE_M = 1.0f;

const int MAX_LANDMARKS_PER_SCAN = 10; // Max circles to send per message

// =================================================================

// --- ROS Handles & Globals ---
ros::NodeHandle nh;
my_slam_interfaces::Ticks ticks_msg;
ros::Publisher pub_ticks("robot_ticks", &ticks_msg);
my_slam_interfaces::LandmarkArray landmark_array_msg;
ros::Publisher pub_landmarks("landmarks", &landmark_array_msg);

// --- Encoder and Motor State Variables ---
long left_ticks = 0;
long right_ticks = 0;
int left_direction = 1;
int right_direction = 1;

HardwareSerial LidarSerial(1);
LDS_YDLIDAR_X2_X2L lidar;
struct Point {
  float x;
  float y;
};
struct Circle {
  Point center;
  float radius;
  bool isValid = false;
};
std::vector<Point> g_scan_points;
volatile bool g_scan_ready_to_process = false;


// --- Motor Control Functions ---
void stopbot() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}
void moveForward() {
  left_direction = 1;
  right_direction = 1;
  ledcWrite(0, 230);
  ledcWrite(1, 0);
  ledcWrite(2, 230);
  ledcWrite(3, 0);
}
void moveBackward() {
  left_direction = -1;
  right_direction = -1;
  ledcWrite(0, 0);
  ledcWrite(1, 230);
  ledcWrite(2, 0);
  ledcWrite(3, 230);
}
void moveLeft() {
  left_direction = -1;
  right_direction = 1;
  ledcWrite(0, 0);
  ledcWrite(1, 230);
  ledcWrite(2, 230);
  ledcWrite(3, 0);
}
void moveRight() {
  left_direction = 1;
  right_direction = -1;
  ledcWrite(0, 230);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 230);
}
void commandCallback(const std_msgs::String& cmd) {
  String c = cmd.data;
  if (c == "forward") moveForward();
  else if (c == "backward") moveBackward();
  else if (c == "left") moveLeft();
  else if (c == "right") moveRight();
  else stopbot();
}
ros::Subscriber<std_msgs::String> sub_cmd("cmd_move", &commandCallback);

// --- LiDAR Callback with 30-degree Field of View Restriction ---
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  float dist_m = distance_mm / 1000.0f;

  // Angle check has been restored to process only the front 30-degree scan
  if (dist_m >= MIN_DETECTION_DISTANCE_M && dist_m <= MAX_DETECTION_DISTANCE_M &&
      (angle_deg <= 25.0f || angle_deg >= 335.0f))
  {
    float angle_rad = angle_deg * M_PI / 180.0f;
    g_scan_points.push_back({dist_m * cos(angle_rad), dist_m * sin(angle_rad)});
  }

  if (scan_completed) {
    g_scan_ready_to_process = true;
  }
}
int lidar_serial_read_callback() {
  return LidarSerial.read();
}
size_t lidar_serial_write_callback(const uint8_t *buf, size_t len) {
  return LidarSerial.write(buf, len);
}
void lidar_motor_pin_callback(float value, LDS::lds_pin_t pin);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(pub_ticks);
  nh.advertise(pub_landmarks);
  nh.subscribe(sub_cmd);

  // --- MODIFIED: Robust Motor Initialization ---
  // Explicitly set motor driver pins as outputs and set them to LOW
  // before attaching PWM. This prevents floating inputs from causing
  // arbitrary motor movement on boot.
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  // Now attach the pins to the PWM controller
  ledcAttachPin(AIN1, 0); ledcAttachPin(AIN2, 1);
  ledcAttachPin(BIN1, 2); ledcAttachPin(BIN2, 3);
  for (int i = 0; i < 4; i++) {
    ledcSetup(i, 1000, 8);
  }

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);


  LidarSerial.begin(lidar.getSerialBaudRate(), SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);
  lidar.init();
  lidar.start();

  // Explicitly stop motors after setup to prevent arbitrary movement
  stopbot();
}

unsigned long last_ticks_pub = 0;

// New state variables for polling-based encoder reading
bool last_left_enc_state = LOW;
bool last_right_enc_state = LOW;

// --- Main Loop ---
void loop() {
  // --- Polling-based Encoder Logic ---
  bool current_left_state = digitalRead(LEFT_ENC_A);
  if (last_left_enc_state == LOW && current_left_state == HIGH) {
    left_ticks += left_direction;
  }
  last_left_enc_state = current_left_state;

  bool current_right_state = digitalRead(RIGHT_ENC_A);
  if (last_right_enc_state == LOW && current_right_state == HIGH) {
    right_ticks += right_direction;
  }
  last_right_enc_state = current_right_state;

  if (g_scan_ready_to_process) {
    process_and_publish_landmarks();
    g_scan_points.clear();
    g_scan_ready_to_process = false;
  }

  if (millis() - last_ticks_pub > 100) {
    ticks_msg.header.stamp = nh.now();
    ticks_msg.left_ticks = left_ticks;
    ticks_msg.right_ticks = right_ticks;
    pub_ticks.publish(&ticks_msg);
    last_ticks_pub = millis();
  }

  lidar.loop();
  nh.spinOnce();
  delay(10);
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t pin) {
  int p = (pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_EN_PIN : LIDAR_PWM_PIN;
  if (value <= (float)LDS::DIR_INPUT) {
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
      ledcSetup(4, 10000, 11);
      ledcAttachPin(p, 4);
    } else {
      pinMode(p, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }
  if (pin == LDS::LDS_MOTOR_PWM_PIN) {
    ledcWrite(4, value);
  } else {
    digitalWrite(p, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
  }
}


// --- Landmark Processing Functions (Helper functions remain) ---
std::vector<std::vector<Point>> findClusters(const std::vector<Point>& points);
Circle fitCircle(const std::vector<Point>& cluster);
bool isGoodCircleFit(const Circle& circle, const std::vector<Point>& cluster);


// --- REWRITTEN: process_and_publish_landmarks ---
void process_and_publish_landmarks() {
  if (g_scan_points.size() < MIN_POINTS_FOR_FIT) return;

  my_slam_interfaces::Landmark landmark_buffer[MAX_LANDMARKS_PER_SCAN];
  landmark_array_msg.landmarks = landmark_buffer;

  std::vector<std::vector<Point>> clusters = findClusters(g_scan_points);
  int landmark_count = 0;

  for (const auto& cluster : clusters) {
    if (cluster.size() < MIN_POINTS_FOR_FIT) continue;
    if (landmark_count >= MAX_LANDMARKS_PER_SCAN) break; // Don't exceed buffer

    Circle circle = fitCircle(cluster);

    if (circle.isValid && isGoodCircleFit(circle, cluster)) {
      landmark_buffer[landmark_count].x = circle.center.x;
      landmark_buffer[landmark_count].y = circle.center.y;
      landmark_buffer[landmark_count].radius = circle.radius;
      landmark_buffer[landmark_count].id = 0;
      landmark_count++;
    }
  }

  if (landmark_count > 0) {
    landmark_array_msg.header.stamp = nh.now();
    landmark_array_msg.header.frame_id = "base_scan";
    landmark_array_msg.landmarks_length = landmark_count;
    pub_landmarks.publish(&landmark_array_msg);
  }
}

bool isGoodCircleFit(const Circle& circle, const std::vector<Point>& cluster) {
  if (circle.radius < MIN_CIRCLE_RADIUS_M || circle.radius > MAX_CIRCLE_RADIUS_M) return false;

  float sum_sq_err = 0.0f;
  for (const auto& p : cluster) {
    float dx = p.x - circle.center.x; float dy = p.y - circle.center.y;
    float error = std::sqrt(dx * dx + dy * dy) - circle.radius;
    sum_sq_err += error * error;
  }
  if (std::sqrt(sum_sq_err / cluster.size()) > MAX_FIT_RMSE_M) return false;

  const Point& p_start = cluster.front(); const Point& p_end = cluster.back();
  float angle_start = atan2(p_start.y - circle.center.y, p_start.x - circle.center.x);
  float angle_end = atan2(p_end.y - circle.center.y, p_end.x - circle.center.x);
  float angle_diff = angle_end - angle_start;
  if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  if (angle_diff < -M_PI) angle_diff += 2 * M_PI;
  if (std::abs(angle_diff) * 180.0f / M_PI < MIN_ARC_ANGLE_DEG) return false;

  return true;
}

std::vector<std::vector<Point>> findClusters(const std::vector<Point>& points) {
  std::vector<std::vector<Point>> clusters;
  if (points.empty()) return clusters;

  clusters.emplace_back();
  clusters.back().push_back(points[0]);

  for (size_t i = 1; i < points.size(); ++i) {
    const Point& p1 = points[i - 1]; const Point& p2 = points[i];
    float dist_sq = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
    if (dist_sq > (CLUSTER_MAX_DISTANCE_M * CLUSTER_MAX_DISTANCE_M)) {
      clusters.emplace_back();
    }
    clusters.back().push_back(p2);
  }
  return clusters;
}

Circle fitCircle(const std::vector<Point>& cluster) {
  Circle result;
  result.isValid = false;
  if (cluster.size() < 3) return result;

  double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0, sum_xy = 0;
  double sum_x3 = 0, sum_y3 = 0, sum_xy2 = 0, sum_x2y = 0;

  for (const auto& p : cluster) {
    double x = p.x; double y = p.y; double x2 = x * x; double y2 = y * y;
    sum_x += x; sum_y += y; sum_x2 += x2; sum_y2 += y2; sum_xy += x * y;
    sum_x3 += x2 * x; sum_y3 += y2 * y; sum_xy2 += x * y2; sum_x2y += x2 * y;
  }

  double N = cluster.size();
  double A11 = N * sum_x2 - sum_x * sum_x;
  double A12 = N * sum_xy - sum_x * sum_y;
  double A22 = N * sum_y2 - sum_y * sum_y;
  double BB1 = 0.5 * (N * sum_x3 + N * sum_xy2 - sum_x * sum_x2 - sum_x * sum_y2);
  double BB2 = 0.5 * (N * sum_y3 + N * sum_x2y - sum_y * sum_x2 - sum_y * sum_y2);

  double detA = A11 * A22 - A12 * A12;
  if (std::abs(detA) < 1e-6) return result;

  double cx = (BB1 * A22 - BB2 * A12) / detA;
  double cy = (BB2 * A11 - BB1 * A12) / detA;

  double C = -N * (cx * cx + cy * cy) + 2 * cx * sum_x + 2 * cy * sum_y - (sum_x2 + sum_y2);
  double radius_sq = -C / N;

  if (radius_sq > 0) {
    result.center.x = cx;
    result.center.y = cy;
    result.radius = std::sqrt(radius_sq);
    result.isValid = true;
  }
  return result;
}
