#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
using namespace websockets;

// ===================== WiFi =====================
const char* ssid     = "<your-ssid>";
const char* password = "<your-password>";

// ===================== WebSocket =====================
const char* ws_host = "<your-address>"; // Computer IP running the server
const uint16_t ws_port = 8765;
const char* ws_path = "/esp/1";       // Bot ID 1 path
WebsocketsClient client;

// ===================== Ultrasonic (e.g., HC-SR04) =====================
// CHANGE these to your wiring
const int ULTRASONIC_TRIG = 5;   // output
const int ULTRASONIC_ECHO = 4;   // input
const unsigned long PULSE_TIMEOUT_US = 30000; // 30ms timeout
const float EDGE_THRESH_CM = 12.0;

// ===================== TB6612FNG Pins =====================
// CHANGE these to your wiring
const int STBY = 23; // enable

const int AIN1 = 19;
const int AIN2 = 18;
const int PWMA = 21; // PWM channel 0

// Motor B (Right)
const int BIN1 = 16;
const int BIN2 = 17;
const int PWMB = 25; // PWM channel 1

// Movement config
int driveSpeed = 180; // 0-255 PWM default speed
bool isMoving = false;

// ===================== Helpers =====================
void standby(bool enable) {
  digitalWrite(STBY, enable ? HIGH : LOW);
}

void setMotor(int in1, int in2, int pwmPin, int speed) {
  // speed: -255..255 (negative = reverse)
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    // Brake
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopMotors() {
  setMotor(AIN1, AIN2, PWMA, 0);
  setMotor(BIN1, BIN2, PWMB, 0);
  isMoving = false;
}

void moveForward() {
  standby(true);
  setMotor(AIN1, AIN2, PWMA, driveSpeed);
  setMotor(BIN1, BIN2, PWMB, driveSpeed);
  isMoving = true;
}

void moveBackward() {
  standby(true);
  setMotor(AIN1, AIN2, PWMA, -driveSpeed);
  setMotor(BIN1, BIN2, PWMB, -driveSpeed);
  isMoving = true;
}

void turnLeft() {
  standby(true);
  // Left motor backward, right motor forward (in-place turn)
  setMotor(AIN1, AIN2, PWMA, -driveSpeed);
  setMotor(BIN1, BIN2, PWMB, driveSpeed);
  isMoving = true;
}

void turnRight() {
  standby(true);
  // Left motor forward, right motor backward
  setMotor(AIN1, AIN2, PWMA, driveSpeed);
  setMotor(BIN1, BIN2, PWMB, -driveSpeed);
  isMoving = true;
}

float readDistanceCm() {
  // Trigger pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // Measure echo
  unsigned long dur = pulseIn(ULTRASONIC_ECHO, HIGH, PULSE_TIMEOUT_US);
  if (dur == 0) {
    return NAN; // timeout/no reading
  }
  // HC-SR04: distance cm = duration / 58.2
  return dur / 58.2f;
}

// ===================== WebSocket Callbacks =====================
void handleSimpleCommand(const String& cmd) {
  String c = cmd;
  c.toLowerCase();

  if (c == "forward" || c == "front") {
    moveForward();
  } else if (c == "backward" || c == "back") {
    moveBackward();
  } else if (c == "left") {
    turnLeft();
  } else if (c == "right") {
    turnRight();
  } else if (c == "stop") {
    stopMotors();
  } else if (c.startsWith("speed:")) {
    // Example: "speed:200" to change base speed
    int sp = c.substring(6).toInt();
    driveSpeed = constrain(sp, 0, 255);
  } else {
    // Unknown command
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void onMessage(WebsocketsMessage message) {
  String data = message.data();
  Serial.print("WS message: ");
  Serial.println(data);

  // If your server sends plain strings like "forward"/"stop", this works.
  // If it sends JSON, you can add a tiny parser for {"cmd":"forward"}.
  handleSimpleCommand(data);
}

void onEvent(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("WebSocket connection opened.");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("WebSocket connection closed.");
    stopMotors();
  }
}

// ===================== Setup/Loop =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Pins
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  standby(false);
  stopMotors();

  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // WebSocket
  client.onMessage(onMessage);
  client.onEvent(onEvent);
  client.connect(ws_host, ws_port, ws_path);
  Serial.println("Connecting to WebSocket server...");
}

void loop() {
  client.poll();

  // Periodically send status (distance + moving flag)
  static uint32_t last_status_ms = 0;
  if (millis() - last_status_ms > 1500) {
    last_status_ms = millis();
    if (client.available()) {
      float dist = readDistanceCm();
      bool edge = (!isnan(dist) && dist > 0 && dist < EDGE_THRESH_CM);

      // Safety: stop if too close to an obstacle/edge
      if (edge) {
        Serial.println("EDGE DETECTED -> STOP");
        stopMotors();
      }

      String status = "{\"bot_id\":1,";
      status += "\"status\":\"";
      status += (isMoving ? "moving" : "idle");
      status += "\",\"cleaning\":";
      status += (isMoving ? "true" : "false");
      status += ",\"distance_cm\":";
      if (isnan(dist)) status += "null"; else status += String(dist, 1);
      status += ",\"edge\":";
      status += (edge ? "true" : "false");
      status += "}";

      Serial.print("Sending status: ");
      Serial.println(status);
      client.send(status);
    }
  }
}