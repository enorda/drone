#include <WiFi.h>

const char* ssid = "ESP32_Host";     
const char* password = "12345678";   

const char* host_ip = "192.168.4.1"; 
const uint16_t port = 80;

const char defaultMessage[] = "Default message: ESP32 communication established!";
unsigned long timeout = 30000;  //close connection after 30 sec inactive
unsigned long startTime = 0;
unsigned long retryDelay = 2000; 
unsigned long lastRetryTime = 0;
unsigned long lastActivityTime = 0;
bool reminderSent = false;

void setup() {
  Serial.begin(115200);

  // Connect to WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(defaultMessage);  
}

WiFiClient client;

void loop() {
  if (!client.connected() && millis() - lastRetryTime > retryDelay) {
    if (client.connect(host_ip, port)) {
      Serial.println("Connected to server");
      client.println(defaultMessage);    //send default message to the server
      startTime = millis();     // Start timeout timer
    } 
    else {
      Serial.println("Failed to connect to server");
      lastRetryTime = millis();  // Update the last retry time
    }
  }
  
  // No need to writing input from the Client serial monitor, because I only check sending String from server

  // Wait for response from server
  while (client.connected()) {
    if (client.available()) {
      char response[150];
      int arrayRead = client.readBytesUntil('\n', response, sizeof(response)-1);
      response[(arrayRead)] = '\0';  // Null-terminate the string

      Serial.print("Received from server: ");
      Serial.println(response);

      if (strncmp(response, "Array: ", 7) == 0) {
        char arrayData[150];
        strncpy(arrayData, response + 7, sizeof(arrayData)-1);  // Extract the array data part
        arrayData[sizeof(arrayData) - 1] = '\0';
        processArrayMessage(arrayData);  // Call function to process the array
      } else {
        Serial.println("Received nothing from server");
      }

      //Serial.println("Connection will close after 30 seconds of inactive");
      startTime = millis();      //reset timeout
    }

    if ((millis() - startTime >= timeout / 2) && !reminderSent) {
        Serial.println("Client: Waiting for data from server...");
        reminderSent = true;  // Set the flag to avoid repeated reminders
    }

    if (millis() - startTime >= timeout) {
      Serial.println("Timeout reached, closing connection due to inactivity");
      client.stop(); 
      break;
    }
  }

  if (!client.connected()) {
    Serial.println("Client disconnected");
  }
}

// Function to process the array message from the server
void processArrayMessage(char* arrayData) {
  // Variables to store extracted values
  int packageNumber;
  float latitude, longitude;
  bool isMarkerFound = false;
  bool moveUp = false;

  char temp[10]; // Temporary buffer for parsing true/false values
  int result = sscanf(arrayData, "[%d,[%f,%f],%[^,],%[^]]]", &packageNumber, &latitude, &longitude, temp, temp + 5);

  if (result < 5) {
    Serial.println("Error parsing the array data");
    return;
  }

  // Convert the true/false strings to boolean values
  if (strcmp(temp, "true") == 0) {
    isMarkerFound = true;
  }
  if (strcmp(temp + 5, "true") == 0) {
    moveUp = true;
  }

  // Print the extracted values
  Serial.print("Package number: ");
  Serial.println(packageNumber);

  Serial.print("Coordinates: [");
  Serial.print(latitude, 6);
  Serial.print(", ");
  Serial.print(longitude, 6);
  Serial.println("]");

  Serial.print("isMarkerFound: ");
  Serial.println(isMarkerFound ? "true" : "false");

  Serial.print("moveUp: ");
  Serial.println(moveUp ? "true" : "false");
}

