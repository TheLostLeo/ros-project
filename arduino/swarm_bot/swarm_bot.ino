#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>

// --- Your WiFi Credentials ---
const char* ssid = "Password";
const char* password = "passwordnotfound";

// --- WebSocket Server ---
const char* ws_host = "10.63.209.72"; // <-- IMPORTANT: Change to your computer's IP
const uint16_t ws_port = 8765;
const char* ws_path = "/esp/2"; // Path for bot ID 1

using namespace websockets;
WebsocketsClient client;

// This function is called when a message is received from the server
void onMessage(WebsocketsMessage message) {
    Serial.print("Got message: ");
    Serial.println(message.data());

    // You can add logic here to control the bot based on the message
    if (message.data() == "front") {
        Serial.println("Received 'front' command! Moving forward.");
        // Add your motor control code here, e.g., moveForward();
    }
}

// This function is called for different connection events
void onEvent(WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Websocket connection opened.");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Websocket connection closed.");
    } else if (event == WebsocketsEvent::GotPing) {
        Serial.println("Got a ping!");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- Connect to WiFi ---
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // --- Setup WebSocket Callbacks ---
    client.onMessage(onMessage);
    client.onEvent(onEvent);

    // --- Connect to WebSocket Server ---
    // The "true" at the end enables auto-reconnect
    client.connect(ws_host, ws_port, ws_path);
    Serial.println("Attempting to connect to WebSocket server...");
}

void loop() {
    // This is required to keep the connection alive and check for messages
    client.poll();

    // Example: send a status message back to the server every 5 seconds
    static uint32_t last_send_time = 0;
    if (millis() - last_send_time > 5000) {
        last_send_time = millis();
        if (client.available()) {
            String status_msg = "{\"bot_id\": 1, \"status\": \"idle\", \"battery_level\": 0.95}";
            Serial.print("Sending status: ");
            Serial.println(status_msg);
            client.send(status_msg);
        }
    }
}