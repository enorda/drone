#include <WiFi.h>
#include <esp_now.h>

String dataFromHost = "";

// Updated callback function with esp_now_recv_info structure
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  char receivedData[len + 1];
  memcpy(receivedData, incomingData, len);
  receivedData[len] = '\0';  // Null-terminate the string
  dataFromHost = String(receivedData);
  Serial.print("Received: ");
  Serial.println(dataFromHost);
}

void setup() {
  Serial.begin(115200);

  // Initialize WiFi in Station mode for ESP-NOW
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register for a callback function to handle incoming data
  esp_now_register_recv_cb(onDataReceive);

  Serial.println("ESP-NOW client initialized");
}

void loop() {
  // Continuously check for incoming data (handled in onDataReceive callback)
  delay(500);  // Adjust the delay as needed
}
