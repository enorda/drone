#include <WiFi.h>
#include <esp_now.h>

const char TERMINATOR = '\n';

// Variable to hold the incoming command from Jetson
String commandFromJetson = "";

// ESP-NOW peer address (replace with the MAC address of the receiver ESP32)
uint8_t peerAddress[] = {0x88, 0x13, 0xBF, 0x07, 0xAD, 0xC0}; // Example MAC, update with actual

void setup() {
  Serial.begin(115200);

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized");
}

// Send command to peer via ESP-NOW
void sendCommand(const String &command) {
  // Convert the command to a character array
  char commandArray[command.length() + 1];
  command.toCharArray(commandArray, command.length() + 1);

  // Send data to peer
  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)commandArray, sizeof(commandArray));
  
  if (result == ESP_OK) {
    Serial.println("Sent command via ESP-NOW: " + command);
  } else {
    Serial.println("Error sending command");
  }
}

void loop() {
  // Check for command from Jetson
  if (Serial.available() > 0) {
    commandFromJetson = Serial.readStringUntil(TERMINATOR);
    sendCommand(commandFromJetson);
  }

  delay(500);
}
