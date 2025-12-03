#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
using namespace websockets;

// ===================== WiFi =====================
const char* ssid     = "Password";
const char* password = "passwordnotfound";

// ===================== WebSocket =====================
const char* ws_host = "10.171.9.72";
const uint16_t ws_port = 8765;
const char* ws_path = "/esp/1";
WebsocketsClient client;

// ===================== Ultrasonic =====================
const int ULTRASONIC_VCC_PIN = 27;
const int ULTRASONIC_TRIG = 13;
const int ULTRASONIC_ECHO = 12;
const unsigned long PULSE_TIMEOUT_US = 30000;
const float EDGE_THRESH_CM = 12.0;

// ===================== TB6612FNG Pins =====================
const int PWMA = 23;
const int AIN1 = 21;
const int AIN2 = 22;
const int PWMB = 19;
const int BIN1 = 5;
const int BIN2 = 18;
const int STBY = 4;

// ===================== Variables =====================
int driveSpeed = 190;
bool isMoving = false;
String currentDirection = "stop";
String edgeState = "unknown";
bool safetyStopActive = false;
bool edgeMessageSent = false; // prevent spam when edge lost

// ===================== Motor Helpers =====================
void standby(bool enable) { digitalWrite(STBY, enable ? HIGH : LOW); }

void setMotor(int in1, int in2, int pwmPin, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(pwmPin, 0);
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
  currentDirection = "forward";
}

void moveBackward() {
  standby(true);
  setMotor(AIN1, AIN2, PWMA, -driveSpeed);
  setMotor(BIN1, BIN2, PWMB, -driveSpeed);
  isMoving = true;
  currentDirection = "backward";
}

void turnLeft() {
  standby(true);
  setMotor(AIN1, AIN2, PWMA, -driveSpeed);
  setMotor(BIN1, BIN2, PWMB, driveSpeed);
  isMoving = true;
  currentDirection = "left";
}

void turnRight() {
  standby(true);
  setMotor(AIN1, AIN2, PWMA, driveSpeed);
  setMotor(BIN1, BIN2, PWMB, -driveSpeed);
  isMoving = true;
  currentDirection = "right";
}

// ===================== Ultrasonic =====================
float readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  unsigned long dur = pulseIn(ULTRASONIC_ECHO, HIGH, PULSE_TIMEOUT_US);
  if (dur == 0) return NAN;
  return dur / 58.2f;
}

// ===================== Status Builder =====================
String buildStatusJson(float dist) {
  String status = "{\"bot_id\":1,";
  status += "\"status\":\"" + String(isMoving ? "moving" : "idle") + "\",";
  status += "\"direction\":\"" + currentDirection + "\",";
  status += "\"distance_cm\":";
  if (isnan(dist)) status += "null"; else status += String(dist, 1);
  status += ",\"edge\":\"" + edgeState + "\"}";
  return status;
}

// ===================== WebSocket Command Handler =====================
void handleSimpleCommand(const String& cmd) {
  String c = cmd; c.toLowerCase();

  if (c == "forward" || c == "front") moveForward();
  else if (c == "backward" || c == "back") moveBackward();
  else if (c == "left") turnLeft();
  else if (c == "right") turnRight();
  else if (c == "stop") { stopMotors(); currentDirection = "stop"; }
  else if (c.startsWith("speed:")) {
    int sp = c.substring(6).toInt();
    driveSpeed = constrain(sp, 0, 255);
  } else if (c == "status") {
    float d = readDistanceCm();
    String status = buildStatusJson(d);
    Serial.print("Sending status: "); Serial.println(status);
    client.send(status);
  } else Serial.print("Unknown command: "), Serial.println(cmd);
}

// ===================== WebSocket Callbacks =====================
void onMessage(WebsocketsMessage message) {
  String data = message.data();
  Serial.print("WS message: "); Serial.println(data);
  handleSimpleCommand(data);
}

void onEvent(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened)
    Serial.println("WebSocket connection opened.");
  else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("WebSocket connection closed."); stopMotors();
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(ULTRASONIC_VCC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_VCC_PIN, HIGH);
  delay(100);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  standby(false); stopMotors();

  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT_PULLDOWN);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.println("\nWiFi connected!"); Serial.print("IP: "); Serial.println(WiFi.localIP());

  client.onMessage(onMessage);
  client.onEvent(onEvent);
  Serial.println("Connecting to WebSocket server...");
  if (client.connect(ws_host, ws_port, ws_path))
    Serial.println("WebSocket connection established");
  else Serial.println("WebSocket connection failed");
}

// ===================== Loop =====================
void loop() {
  client.poll();

  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 300) {
    lastCheck = millis();
    float dist = readDistanceCm();

    if (isnan(dist)) {
      edgeState = "no_reading";
      if (!safetyStopActive) {
        Serial.println("NO ULTRASONIC READING -> STOP");
        stopMotors();
        safetyStopActive = true;
        edgeMessageSent = false;
      }
    }
    else if (dist >= EDGE_THRESH_CM) {
      edgeState = "none";
      if (!safetyStopActive) {
        Serial.print("EDGE FAR ("); Serial.print(dist); Serial.println(" cm) -> STOP (safety)");
        stopMotors();
        safetyStopActive = true;
        // Send one-time message to WebSocket
        if (!edgeMessageSent) {
          String msg = buildStatusJson(dist);
          Serial.println("Sending edge-not-detected message to WebSocket:");
          Serial.println(msg);
          client.send(msg);
          edgeMessageSent = true;
        }
      }
    }
    else {
      edgeState = "detected";
      if (safetyStopActive) {
        Serial.println("EDGE DETECTED AGAIN -> SAFE TO MOVE");
        safetyStopActive = false;
        edgeMessageSent = false;
        // Resume movement
        if (currentDirection == "forward") moveForward();
        else if (currentDirection == "backward") moveBackward();
        else if (currentDirection == "left") turnLeft();
        else if (currentDirection == "right") turnRight();
      }
    }
  }
}
