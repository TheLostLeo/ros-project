#include <WiFi.h>
#include <WebSocketsClient.h>

const char* ssid = "Test";
const char* password = "12345678";

const char* ws_host = "10.88.81.72"; // ROS server IP   
const uint16_t ws_port = 8765;
const char* ws_path = "/esp/1";        // 👈 Unique ID for this ESP

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to server");
      webSocket.sendTXT("hello_from_esp");
      break;
    case WStype_TEXT:
      Serial.printf("[WS] Message from server: %s\n", payload);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  webSocket.begin(ws_host, ws_port, ws_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();
}
