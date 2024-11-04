#include <WiFi.h>

const char* ssid = "ESP32_Host";
const char* password = "12345678";

WiFiServer server(80);

unsigned long timeout = 30000;
unsigned long lastActivityTime = 0;

void setup() {
  Serial.begin(115200);

  // Set up WiFi Access Point
  WiFi.softAP(ssid, password);

  server.begin();
  Serial.println("Server started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  // Check for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");

    // Reset the timer when the client connects
    lastActivityTime = millis();

    while (client.connected()) {
      if (client.available()) {
        String message = client.readStringUntil('\n');
        message.trim();
        Serial.println("Server received: " + message);

        int number = 2;
        float coordinates[2] = {35.1589, -112.45698};
        bool isMarkerFound = true;
        bool moveUp = true;

        char arrayMessage[150]; 
        snprintf(arrayMessage, sizeof(arrayMessage), "[%d,[%.6f,%.6f],%s,%s]", 
                 number, coordinates[0], coordinates[1], 
                 isMarkerFound ? "true" : "false", 
                 moveUp ? "true" : "false");

        client.println("Array: " + String(arrayMessage)); 
        Serial.println("Sent array: " + String(arrayMessage));

        // Reset the activity timer 
        lastActivityTime = millis();
      }

      if (Serial.available()) {
        char serialInput[150];
        int serialBytes = Serial.readBytesUntil('\n', serialInput, sizeof(serialInput) - 1);  
        serialInput[serialBytes] = '\0';  // Null-terminate the string

        Serial.print("Server received data from user keyboard: ");
        Serial.println(serialInput);  // Print for debugging

        // Optionally send this serial input to the client
        client.println("Received from server: " + String(serialInput));
        Serial.println("Sent array: " + String(serialInput));

        // Reset activity timer if serial input is processed
        lastActivityTime = millis();
      }

      if (millis() - lastActivityTime > timeout) {
        Serial.println("Timeout reached, disconnecting client");
        client.stop();
        break;
      }
    }

    if (!client.connected()) {
      Serial.println("Client disconnected");
    }
  }
}
