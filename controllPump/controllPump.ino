#include <ESP8266WiFi.h>
#include <espnow.h>

// Define the pump control pin (replace with the actual pin you're using)
#define PUMP_PIN  4  // D2 on many NodeMCU boards (GPIO4)

// Structure to receive data (must match sender's structure)
typedef struct struct_message {
  int id;
  String msg;
} struct_message;

struct_message myData;

// Callback function when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("ID: ");
  Serial.println(myData.id);
  Serial.print("Message: ");
  Serial.println(myData.msg);
  Serial.println();

  // Control the pump based on the received ID
  if (myData.id == 1) {  // Turn ON pump
    Serial.println("Turning ON pump");
    digitalWrite(PUMP_PIN, HIGH);  // Or whatever logic turns your pump on (active high)
  } else if (myData.id == 2) {  // Turn OFF pump
    Serial.println("Turning OFF pump");
    digitalWrite(PUMP_PIN, LOW);   // Or whatever logic turns your pump off (active low)
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Receiver Code Started");

  // Set the pump pin as an output
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW); // Ensure the pump is initially off

  // Set ESP8266 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.print("Receiver ESP8266 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());   //Display device MAC adddress on serial monitor

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW role to slave (receiver)
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

  // Register callback function for data received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing to do in the main loop; ESP-NOW handles the communication
}