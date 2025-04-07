#include <AccelStepper.h>

// Stepper Motor Definitions
#define STEP_PIN 3  // Pin for Step signal
#define DIR_PIN 4   // Pin for Direction signal

// TCRT5000 Sensor Definitions
#define SENSOR_LED_PIN 6  // LED pin for TCRT5000
#define SENSOR_PIN A3     // Photodiode input pin

// Stepper motor instance
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Variables
int pegsPerRow = 5;            // Number of pegs in each row
int currentPegCount = 0;       // Number of pegs detected in the current row
int totalRows = 4;             // Total number of rows to knit
int currentRow = 0;            // Current row count
unsigned long lastPegTime = 0; // Timestamp of the last detected peg
int debounceTime = 200;        // Debounce time in milliseconds
int detectionThreshold = 50;   // Signal difference threshold for peg detection
bool motorRunning = false;     // Flag to indicate motor is running
bool direction = true;         // Direction: true = forward, false = backward

// Function to read sensor and detect pegs
bool detectPeg() {
  digitalWrite(SENSOR_LED_PIN, HIGH);  // Turn on LED
  delayMicroseconds(500);
  int readingWithSignal = analogRead(SENSOR_PIN);  // Noise + signal
  digitalWrite(SENSOR_LED_PIN, LOW);   // Turn off LED
  delayMicroseconds(500);
  int readingWithoutSignal = analogRead(SENSOR_PIN);  // Noise
  int difference = readingWithoutSignal - readingWithSignal;  // Signal strength

  unsigned long currentTime = millis();

  // Peg detection logic
  if (difference > detectionThreshold && (currentTime - lastPegTime) > debounceTime) {
    lastPegTime = currentTime;  // Update last detection time
    return true;  // Peg detected
  }
  return false;  // No peg detected
}

// Function to start the motor for a new row
void startNewRow() {
  if (currentRow < totalRows) {
    currentPegCount = 0;          // Reset peg count
    direction = !direction;       // Reverse direction
    stepper.setCurrentPosition(0); // Reset stepper position
    int targetSteps = pegsPerRow * 200; // Adjust '200' to match your motor's steps per peg
    stepper.moveTo(direction ? targetSteps : -targetSteps); // Move to target based on direction
    motorRunning = true;
    currentRow++;
    Serial.print("Starting row ");
    Serial.print(currentRow);
    Serial.println(direction ? " (Forward)" : " (Backward)");
  } else {
    // Stop motor after the total number of rows has been reached
    Serial.println("Knitting complete! Stopping motor.");
    stepper.stop();
    motorRunning = false; // Stop motor and end process
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize sensor
  pinMode(SENSOR_LED_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);

  // Initialize stepper motor
  stepper.setMaxSpeed(70);      // Set max speed (steps per second)
  stepper.setAcceleration(40);  // Set acceleration (steps per second^2)

  // Start the first row
  startNewRow();
}

void loop() {
  // Run the stepper motor if it’s still running
  if (motorRunning) {
    stepper.run();
  }

  // Check for peg detection
  if (detectPeg()) {
    currentPegCount++;
    Serial.print("Peg detected! Count: ");
    Serial.println(currentPegCount);

    // Stop motor if the target number of pegs for the row is reached
    if (currentPegCount >= pegsPerRow) {
      stepper.stop();           // Stop the motor
      motorRunning = false;     // Clear motor running flag
      Serial.println("Row complete.");
      delay(1000);              // Pause before starting the next row
      startNewRow();            // Start the next row or stop if rows are complete
    }
  }
}