#define BLYNK_TEMPLATE_ID           "TMPL6Q2cymGGo"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "3gJ0ZxdVyNMh4N-F9OJoaiWs4Vt2t6z8"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Wi-Fi credentials
char ssid[] = "AbuMalik_2.4G";
char pass[] = "0500056388";

// Servo motor parameters
const int servoPin = 2;
const int pwmChannel = 0;
const int frequency = 50; // 50 Hz for standard servos
const int resolution = 16;
const int minPulseWidth = 500;  // Minimum pulse width in microseconds
const int maxPulseWidth = 2500; // Maximum pulse width in microseconds

// Touch sensor pin
const int touchPin = 15;

// Servo state variables
bool isServoOn = false;      // Track whether the servo is "on" (90 degrees)
bool lastTouchState = LOW;   // Store the last state of the touch sensor
int currentAngle = 0;        // Current angle of the servo (0 or 90 degrees)

// Function to map angle to PWM duty cycle
int angleToDutyCycle(int angle) {
  return map(angle, 0, 180, (minPulseWidth * pow(2, resolution)) / 20000,
             (maxPulseWidth * pow(2, resolution)) / 20000);
}

// Blynk Virtual Button Handler (V2)
BLYNK_WRITE(V2) {
  // Toggle the servo state
  isServoOn = !isServoOn;

  // Update the servo position
  int targetAngle = isServoOn ? 90 : 0; // 90 degrees for "on", 0 degrees for "off"
  currentAngle = targetAngle;          // Synchronize with the current angle
  int dutyCycle = angleToDutyCycle(targetAngle);
  ledcWrite(pwmChannel, dutyCycle);

  // Send updated state to Blynk
  sendServoStateToBlynk();

  // Debug output
  Serial.print("Button toggled. Servo is now: ");
  Serial.println(isServoOn ? "ON (90 degrees)" : "OFF (0 degrees)");
}

// Function to send the servo state to Blynk (V1)
void sendServoStateToBlynk() {
  Blynk.virtualWrite(V1, isServoOn ? 1 : 0); // Send 1 for ON, 0 for OFF
}

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // Connect to Wi-Fi and Blynk
  Serial.println("Connecting to WiFi...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Configure servo PWM channel
  ledcSetup(pwmChannel, frequency, resolution);
  ledcAttachPin(servoPin, pwmChannel);

  // Initialize servo to 0 degrees
  int dutyCycle = angleToDutyCycle(0);
  ledcWrite(pwmChannel, dutyCycle);

  // Configure touch sensor
  pinMode(touchPin, INPUT_PULLDOWN);

  Serial.println("Setup complete.");
}

void loop() {
  // Keep Blynk running
  Blynk.run();

  // Read the current state of the touch sensor
  bool touchState = digitalRead(touchPin);

  // Check if the touch sensor state has changed
  if (touchState == HIGH && lastTouchState == LOW) {
    // Toggle the servo state
    isServoOn = !isServoOn;

    // Update the servo position
    int targetAngle = isServoOn ? 90 : 0; // 90 degrees for "on", 0 degrees for "off"
    currentAngle = targetAngle;          // Synchronize with the current angle
    int dutyCycle = angleToDutyCycle(targetAngle);
    ledcWrite(pwmChannel, dutyCycle);

    // Send the updated state to Blynk
    sendServoStateToBlynk();

    // Debug output
    Serial.print("Touch sensor toggled. Servo is now: ");
    Serial.println(isServoOn ? "ON (90 degrees)" : "OFF (0 degrees)");

    // Debounce delay
    delay(50);
  }

  // Update the last touch state
  lastTouchState = touchState;

  // Small delay to reduce processing load
  delay(10);
}