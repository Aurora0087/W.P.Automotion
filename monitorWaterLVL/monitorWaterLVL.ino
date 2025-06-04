// Corrected code - full program with the loop() function modified to work in the Arduino IDE
#include <ESP8266WiFi.h>
#include <espnow.h>

// Define the pins for the ultrasonic sensor
#define TRIG_PIN 4  // D2 on many NodeMCU boards (GPIO4)
#define ECHO_PIN 5  // D1 on many NodeMCU boards (GPIO5)

// Define the LED pin
#define LED_PIN 16  // Example: GPIO16 (D0 on some NodeMCU boards)

// Define the button pin
#define OVERRIDE_BUTTON_PIN 0  // D3 on many NodeMCU boards (GPIO0)  **IMPORTANT: Requires Pull-Up Resistor**
// Define the button pin (for Manual ON/OFF)
#define MANUAL_ON_OFF_BUTTON_PIN 14  // D5 on many NodeMCU boards (GPIO14) **IMPORTANT: Requires Pull-Up Resistor**

#define OVERRIDE_LED_PIN 12
#define MANUAL_ON_OFF_LED_PIN 13

// Define the speed of sound in cm/us (microseconds)
#define SOUND_SPEED 0.034

// Define a timeout for the ultrasonic sensor
#define MAX_DISTANCE 450  // Maximum distance in cm we want to measure
#define TIMEOUT 30000     // Timeout in microseconds (30 ms, corresponds to ~510cm)

// Pump on of trigger
#define WATER_DISTANCE_TURN_ON_PUMP 70
#define WATER_DISTANCE_TURN_OFF_PUMP 25

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

// Global variable to track the button state
bool pumpManuallyOverridden = false;  // Start with automatic control
bool pumpManuallyOn = false;          // Manual pump ON/OFF state, default OFF

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set the pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OVERRIDE_BUTTON_PIN, INPUT_PULLUP);       // Enable internal pull-up resistor
  pinMode(MANUAL_ON_OFF_BUTTON_PIN, INPUT_PULLUP);  //Enable pull up button

  pinMode(OVERRIDE_LED_PIN, OUTPUT);
  pinMode(MANUAL_ON_OFF_LED_PIN, OUTPUT);

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

int maxDistanceCount = 0;                       // Counter for consecutive MAX_DISTANCE readings
unsigned long lastOverrideButtonPressTime = 0;  // To debounce the override button
unsigned long lastManualButtonPressTime = 0;    // To debounce the on/off button
const unsigned long DEBOUNCE_DELAY = 50;        // Milliseconds

void loop() {
  // Measure the distance
  float distance = measureDistance();

  // Read the Override button state
  int overrideButtonState = digitalRead(OVERRIDE_BUTTON_PIN);

  // Read the ON/OFF button state
  int manualOnOffButtonState = digitalRead(MANUAL_ON_OFF_BUTTON_PIN);

  // Check for button press (debounce)
  if (overrideButtonState == LOW && (millis() - lastOverrideButtonPressTime > DEBOUNCE_DELAY)) {

    pumpManuallyOverridden = !pumpManuallyOverridden;  // Toggle the override state
    lastOverrideButtonPressTime = millis();

    Serial.print("Pump Override: ");
    if (pumpManuallyOverridden) {
      digitalWrite(OVERRIDE_LED_PIN, HIGH);  // Turn LED on for override state
      Serial.println("ON");
      delay(1500);
    } else {
      digitalWrite(OVERRIDE_LED_PIN, LOW);  // Turn LED off for override state
      Serial.println("OFF");
      delay(1500);
    }
  }


  // Check for Manual ON/OFF button press (debounce)
  if (manualOnOffButtonState == LOW && (millis() - lastManualButtonPressTime > DEBOUNCE_DELAY)) {

    pumpManuallyOn = !pumpManuallyOn;  // Toggle the manual on/off state
    lastManualButtonPressTime = millis();

    Serial.print("Pump Manual ON/OFF: ");
    if (pumpManuallyOn) {
      digitalWrite(MANUAL_ON_OFF_LED_PIN, HIGH);  // Turn LED on for override state
      Serial.println("ON");
    } else {
      digitalWrite(MANUAL_ON_OFF_LED_PIN, LOW);  // Turn LED off for override state
      Serial.println("OFF");
    }
  }

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Control the LED (example: blink it every 2 seconds)
  digitalWrite(LED_PIN, HIGH);  // Turn LED on

  if (distance == MAX_DISTANCE) {
    delay(250);
    digitalWrite(LED_PIN, LOW);  // Turn LED off
    delay(250);
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW); 
    delay(1000);
    maxDistanceCount++;  // Increment counter
  } else {
    maxDistanceCount = 0;  // Reset counter if distance is not MAX_DISTANCE
  }

  // Check if we've seen MAX_DISTANCE enough times in a row
  if (maxDistanceCount >= 5) {
    Serial.println("MAX_DISTANCE reached 5 times in a row. Turning OFF pump.");
    if (!pumpManuallyOverridden) {  //Only send if not manually overridden
      sendPumpMessage(2, "Turn OFF pump");
    }
    maxDistanceCount = 0;  // Reset counter after sending turn off message
  } else if (distance >= WATER_DISTANCE_TURN_ON_PUMP && distance < MAX_DISTANCE && !pumpManuallyOverridden) {
    sendPumpMessage(1, "Turn ON pump");
  } else if (distance <= WATER_DISTANCE_TURN_OFF_PUMP && !pumpManuallyOverridden) {
    sendPumpMessage(2, "Turn OFF pump");
  } else if (pumpManuallyOverridden) {
    if (pumpManuallyOn) {
      sendPumpMessage(1, "Turn ON pump");
      Serial.println("Pump is manually Overridden sending turn on pump");
      delay(1000);
    } else {
      sendPumpMessage(2, "Turn OFF pump");
      Serial.println("Pump is manually Overridden sending turn off pump");
      delay(1000);
    }
  }

  // Wait for a short period
  delay(250);  // Reduced delay for faster readings
}

// Function to send pump message
void sendPumpMessage(int id, String msg) {
  myData.id = id;
  myData.msg = msg;
  uint8_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == 0) {
    Serial.println("Sent with success");
  } else {
    Serial.print("Error sending the data. Result code: ");
    Serial.println(result);  // Print the error code
  }
}