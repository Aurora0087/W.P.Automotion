// Corrected code - full program with the loop() function modified to work in the Arduino IDE
#include <ESP8266WiFi.h>
#include <espnow.h>

// Define the pins for the ultrasonic sensor
#define TRIG_PIN 4  // D2 on many NodeMCU boards (GPIO4)
#define ECHO_PIN 5  // D1 on many NodeMCU boards (GPIO5)

// Define the LED pin
#define LED_PIN 16  // Example: GPIO16 (D0 on some NodeMCU boards)

// Define the speed of sound in cm/us (microseconds)
#define SOUND_SPEED 0.034

// Define a timeout for the ultrasonic sensor
#define MAX_DISTANCE 400  // Maximum distance in cm we want to measure
#define TIMEOUT 30000     // Timeout in microseconds (30 ms, corresponds to ~510cm)

// Pump on of trigger
#define WATER_DISTANCE_TURN_ON_PUMP 65
#define WATER_DISTANCE_TURN_OFF_PUMP 10

// Structure to send data
typedef struct struct_message {
  int id;
  String msg;
} struct_message;

struct_message myData;

// MAC Address of the receiver ESP8266 (the one controlling the pump)
uint8_t broadcastAddress[] = { 0x48, 0x3F, 0xDA, 0x47, 0xC4, 0x28 };

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
}

// Function to measure the distance
float measureDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Set the trigger pin HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT);  // Added timeout

  // If the pulsein timed out it returns 0
  if (duration == 0) {
    return MAX_DISTANCE;  // Or some other error value
  }


  // Calculate the distance in centimeters
  float distance = (duration * SOUND_SPEED) / 2;

  // Limit the distance to the maximum range
  if (distance > MAX_DISTANCE) {
    distance = MAX_DISTANCE;
  }

  return distance;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set the pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Ultrasonic Sensor Test");

  // Set ESP8266 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Register peer

  // Add peer
  if (esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register callback function for data sent
  esp_now_register_send_cb(OnDataSent);
}

int maxDistanceCount = 0;  // Counter for consecutive MAX_DISTANCE readings

void loop() {
  // Measure the distance
  float distance = measureDistance();

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Control the LED (example: blink it every 2 seconds)
  digitalWrite(LED_PIN, HIGH);  // Turn LED on

  if (distance == MAX_DISTANCE) {
    delay(1000);
    digitalWrite(LED_PIN, LOW);  // Turn LED off
    delay(1000);
    maxDistanceCount++; // Increment counter

  } else {
    maxDistanceCount = 0;  // Reset counter if distance is not MAX_DISTANCE
  }

  // Check if we've seen MAX_DISTANCE enough times in a row
  if (maxDistanceCount >= 5) {
    Serial.println("MAX_DISTANCE reached 5 times in a row. Turning OFF pump.");
    // Send a message to another esp8266(mac address: 48:3F:DA:47:C4:28) using esp now that going to turn off pump after residing that message
    myData.id = 2;  // ID to identify the message
    myData.msg = "Turn OFF pump";
    uint8_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == 0) {
      Serial.println("Sent with success");
    } else {
      Serial.print("Error sending the data. Result code: ");
      Serial.println(result);  // Print the error code
    }
    maxDistanceCount = 0;  // Reset counter after sending turn off message
  }
  else if (distance >= WATER_DISTANCE_TURN_ON_PUMP && distance < MAX_DISTANCE) {
    // Send a message to another esp8266(mac address: 48:3F:DA:47:C4:28) using esp now that going to turn on pump after residing that message
    myData.id = 1;  // ID to identify the message
    myData.msg = "Turn ON pump";
    uint8_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == 0) {
      Serial.println("Sent with success");
    } else {
      Serial.print("Error sending the data. Result code: ");
      Serial.println(result);  // Print the error code
    }
  } else if (distance <= WATER_DISTANCE_TURN_OFF_PUMP) {
    // Send a message to another esp8266(mac address: 48:3F:DA:47:C4:28) using esp now that going to turn off pump after residing that message
    myData.id = 2;  // ID to identify the message
    myData.msg = "Turn OFF pump";
    uint8_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == 0) {
      Serial.println("Sent with success");
    } else {
      Serial.print("Error sending the data. Result code: ");
      Serial.println(result);  // Print the error code
    }
  }

  // Wait for a short period
  delay(250);  // Reduced delay for faster readings
}