// swarm_bot.ino
// ESP32 WebSocket client for Swarm Aquarium bots
// Features:
// - Connects to a WebSocket server (the Swarm Manager)
// - Receives textual commands: start, stop, left, right
// - Sends status updates: alive, edge
// - Uses an ultrasonic sensor to detect edges/obstacles and reports 'edge'
// - Waits for decision reply from manager to turn left/right

#include <WiFi.h>
#include <WebSocketsClient.h>

// WiFi credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// WebSocket server
const char* ws_host = "192.168.1.100"; // manager IP
const uint16_t ws_port = 9002;
const char* ws_path = "/";

WebSocketsClient webSocket;

// Ultrasonic (HC-SR04) pins
const int trigPin = 13; // example
const int echoPin = 12; // example

// Motor control pins (placeholder)
const int motorLeftFwd = 25;
const int motorLeftRev = 26;
const int motorRightFwd = 27;
const int motorRightRev = 14;

bool running = false;
unsigned long lastHeartbeat = 0;

long readUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  if (duration == 0) return 9999;
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

void stopMotors() {
  digitalWrite(motorLeftFwd, LOW);
  digitalWrite(motorLeftRev, LOW);
  digitalWrite(motorRightFwd, LOW);
  digitalWrite(motorRightRev, LOW);
}

void moveForward() {
  digitalWrite(motorLeftFwd, HIGH);
  digitalWrite(motorLeftRev, LOW);
  digitalWrite(motorRightFwd, HIGH);
  digitalWrite(motorRightRev, LOW);
}

void turnLeft() {
  // simple differential turn
  digitalWrite(motorLeftFwd, LOW);
  digitalWrite(motorLeftRev, HIGH);
  digitalWrite(motorRightFwd, HIGH);
  digitalWrite(motorRightRev, LOW);
}

void turnRight() {
  digitalWrite(motorLeftFwd, HIGH);
  digitalWrite(motorLeftRev, LOW);
  digitalWrite(motorRightFwd, LOW);
  digitalWrite(motorRightRev, HIGH);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("Websocket disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("Websocket connected");
      webSocket.sendTXT("register:swarm_bot");
      break;
    case WStype_TEXT: {
      String msg = String((char*)payload);
      msg.trim();
      Serial.print("WS msg: "); Serial.println(msg);
      if (msg == "start") {
        running = true;
        moveForward();
      } else if (msg == "stop") {
        running = false;
        stopMotors();
      } else if (msg == "left") {
        turnLeft();
        delay(400);
        moveForward();
      } else if (msg == "right") {
        turnRight();
        delay(400);
        moveForward();
      } else if (msg == "decision:left") {
        turnLeft();
        delay(400);
        moveForward();
      } else if (msg == "decision:right") {
        turnRight();
        delay(400);
        moveForward();
      }
      break;
    }
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorLeftFwd, OUTPUT);
  pinMode(motorLeftRev, OUTPUT);
  pinMode(motorRightFwd, OUTPUT);
  pinMode(motorRightRev, OUTPUT);
  stopMotors();

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  webSocket.begin(ws_host, ws_port, ws_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();

  // Heartbeat
  if (millis() - lastHeartbeat > 5000) {
    webSocket.sendTXT("alive");
    lastHeartbeat = millis();
  }

  long dist = readUltrasonicDistance();
  // If distance less than threshold, treat as edge
  if (dist < 8 && dist > 0 && running) {
    stopMotors();
    webSocket.sendTXT("edge");
    // wait for decision from manager (decision:left/right)
    unsigned long waitStart = millis();
    while (millis() - waitStart < 5000) {
      webSocket.loop();
      delay(50);
    }
    // After waiting, resume forward (manager should have sent a decision)
    moveForward();
  }

  delay(50);
}
