#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include "secrets.h" // Include your secrets file for WiFi credentials

const int port = 3333;

WiFiServer server(port);
WiFiClient client;

void setup() {
  Serial.begin(115200, SERIAL_8N1, 16, 17); // Initialize serial communication
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.println("WiFi connected");
  }

  //Serial.println("WiFi connected");
  server.begin();
  //Serial.print("Server listening on IP: ");
  //Serial.println(WiFi.localIP());
}

void loop() {
  if (client.connected()) {
    // Handle data from client
    if (client.available()) {
      char c = client.read();
      Serial.write(c); // Forward to serial
    }
    // Handle data from serial
    if (Serial.available()) {
      char c = Serial.read();
      client.write(c); // Forward to client
    }
  } else {
    // Wait for client connection
    client = server.available();
  }
}
