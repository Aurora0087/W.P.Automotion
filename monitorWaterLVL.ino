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
  pinMode(LED_PIN, OUTPUT);  // Set LED pin as output

  Serial.println("Ultrasonic Sensor Test");
}

void loop() {
  // Measure the distance
  float distance = measureDistance();

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Control the LED (example: blink it every 2 seconds)
  digitalWrite(LED_PIN, HIGH);  // Turn LED on

  if (distance >= MAX_DISTANCE) {
    delay(500);
    digitalWrite(LED_PIN, LOW);  // Turn LED off
    delay(500);
  }

  // Wait for a short period
  delay(250);  // Reduced delay for faster readings
}