#include <AccelStepper.h>

// Stepper Motor Definitions
#define STEP_PIN 2    // Pin for Step signal
#define DIR_PIN 4     // Pin for Direction signal
#define ENABLE_PIN 8  // PWM pin for Enable signal

// A4988 Microstepping Mode Pins
#define MS1_PIN 13    // MS1 pin for microstepping
#define MS2_PIN 12    // MS2 pin for microstepping
#define MS3_PIN 11    // MS3 pin for microstepping

// TCRT5000 Sensor Definitions
#define SENSOR_LED_PIN 6  // LED pin for TCRT5000
#define SENSOR_PIN A3     // Photodiode input pin

// Stepper motor instance with microstepping support
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Variables
int pegsPerRow = 20;              // Number of pegs in each row
int currentPegCount = 0;          // Number of pegs detected in the current row
int totalRows =30;               // Total number of rows to knit
int currentRow = 0;               // Current row count
unsigned long lastPegTime = 0;    // Timestamp of the last detected peg
int debounceTime = 500;           // Debounce time in milliseconds
int detectionThreshold = 50;      // Signal difference threshold for peg detection
bool motorRunning = false;        // Flag to indicate motor is running
bool direction = true;            // Direction: true = forward, false = backward
int forwardExtraSteps = 145;        // Extra steps for forward direction (even rows)
int backwardExtraSteps = 145;      // Extra steps for backward direction (odd rows)
int maxSpeed = 800;               // Increased max speed for microstepping
bool programStarted = false;      // Flag to indicate if program has been started

// Microstepping configuration for A4988
#define FULL_STEP 1
#define HALF_STEP 2
#define QUARTER_STEP 4
#define EIGHTH_STEP 8
#define SIXTEENTH_STEP 16

int currentMicrostepMode = EIGHTH_STEP;

// Additional variables for edge detection
bool previousPegState = false;

// Set microstepping mode for A4988
void setMicrosteppingMode(int mode) {
    switch(mode) {
        case FULL_STEP:
            digitalWrite(MS1_PIN, LOW);
            digitalWrite(MS2_PIN, LOW);
            digitalWrite(MS3_PIN, LOW);
            break;
        case HALF_STEP:
            digitalWrite(MS1_PIN, HIGH);
            digitalWrite(MS2_PIN, LOW);
            digitalWrite(MS3_PIN, LOW);
            break;
        case QUARTER_STEP:
            digitalWrite(MS1_PIN, LOW);
            digitalWrite(MS2_PIN, HIGH);
            digitalWrite(MS3_PIN, LOW);
            break;
        case EIGHTH_STEP:
            digitalWrite(MS1_PIN, HIGH);
            digitalWrite(MS2_PIN, HIGH);
            digitalWrite(MS3_PIN, LOW);
            break;
        case SIXTEENTH_STEP:
            digitalWrite(MS1_PIN, HIGH);
            digitalWrite(MS2_PIN, HIGH);
            digitalWrite(MS3_PIN, HIGH);
            break;
        default:
            digitalWrite(MS1_PIN, HIGH);
            digitalWrite(MS2_PIN, HIGH);
            digitalWrite(MS3_PIN, LOW);  // Default to 1/8 step
            break;
    }
    
    currentMicrostepMode = mode;
    stepper.setMaxSpeed(maxSpeed * mode);
}

// Enable/disable the stepper motor
void setMotorEnabled(bool enabled) {
    digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH);  // Most drivers use LOW for enable
}

// Move stepper a specific number of steps
void moveSteps(long numberOfSteps, bool moveDirection) {
    setMotorEnabled(true);  // Enable motor before moving
    motorRunning = false;
    
    // Set direction
    digitalWrite(DIR_PIN, moveDirection ? HIGH : LOW);
    
    // Configure stepper for the move with microstepping consideration
    long adjustedSteps = numberOfSteps * currentMicrostepMode;
    stepper.setSpeed(moveDirection ? maxSpeed : -maxSpeed);
    stepper.move(adjustedSteps * (moveDirection ? 1 : -1));
    
    // Execute the move
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    
    if (!motorRunning) {  // If we're not continuing to run after this move
        setMotorEnabled(false);  // Disable motor after movement
    }
}

// Helper function to move extra steps without peg detection
void moveExtraStepsWithoutCounting(int steps, bool moveDirection) {
    setMotorEnabled(true);
    motorRunning = false;
    
    // Set direction
    digitalWrite(DIR_PIN, moveDirection ? HIGH : LOW);
    
    // Configure stepper for the move with microstepping consideration
    long adjustedSteps = steps * currentMicrostepMode;
    stepper.setSpeed(moveDirection ? maxSpeed : -maxSpeed);
    stepper.move(adjustedSteps * (moveDirection ? 1 : -1));
    
    // Execute the move
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}

bool detectPeg() {
    digitalWrite(SENSOR_LED_PIN, HIGH);                         // Turn on LED
    delayMicroseconds(50);
    int readingWithSignal = analogRead(SENSOR_PIN);             // Noise + signal
    digitalWrite(SENSOR_LED_PIN, LOW);                          // Turn off LED
    delayMicroseconds(50);
    int readingWithoutSignal = analogRead(SENSOR_PIN);          // Noise
    int difference = readingWithoutSignal - readingWithSignal;  // Signal strength
    unsigned long currentTime = millis();

    // Peg detection logic with edge detection
    bool currentPegState = difference > detectionThreshold;
    if (currentPegState && !previousPegState && (currentTime - lastPegTime) > debounceTime) {
        lastPegTime = currentTime;          // Update last detection time
        previousPegState = currentPegState; // Update previous state
        return true;                        // Peg detected
    }
    previousPegState = currentPegState;     // Update previous state
    return false;                           // No peg detected
}

void startNewRow() {
    if (currentRow < totalRows) {
        currentPegCount = 0;                            // Reset peg count
        direction = !direction;                         // Reverse direction
        
        // Only move extra steps if it's not the first row
        if (currentRow > 0) {
            if (direction) {  // Moving backward (odd rows)
                // Move forward first
                moveExtraStepsWithoutCounting(backwardExtraSteps, false);
                delay(200);  // Small delay between movements
                // Then move back
                moveExtraStepsWithoutCounting(backwardExtraSteps, true);
            } else {  // Moving forward (even rows)
                // Move forward first
                moveExtraStepsWithoutCounting(forwardExtraSteps, true);
                delay(200);  // Small delay between movements
                // Then move back
                moveExtraStepsWithoutCounting(forwardExtraSteps, false);
            }
        }
        
        digitalWrite(DIR_PIN, direction ? HIGH : LOW);  // Update direction pin for next row
        stepper.setSpeed(direction ? maxSpeed : -maxSpeed);  // Set speed based on direction
        setMotorEnabled(true);  // Enable motor for the new row
        motorRunning = true;
        currentRow++;
        Serial.print("Starting row ");
        Serial.println(currentRow);
    } else {
        Serial.println("Knitting complete! Motor stopping.");
        stepper.stop();    // Stop the motor completely
        motorRunning = false;
        setMotorEnabled(false);  // Disable motor when complete
        programStarted = false;  // Reset program state
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Initialize pins
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(SENSOR_LED_PIN, OUTPUT);
    pinMode(SENSOR_PIN, INPUT);
    
    // Initialize A4988 microstepping pins
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);
    
    // Initialize stepper motor with microstepping
    stepper.setMaxSpeed(maxSpeed * currentMicrostepMode);
    stepper.setAcceleration(100 * currentMicrostepMode);
    setMotorEnabled(false);             // Start with motor disabled
    
    // Set initial microstepping mode to 1/8 step
    setMicrosteppingMode(EIGHTH_STEP);
    
    Serial.println("Press 's' to start knitting...");
}

void loop() {
    // Check for serial command to start
    if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 's' && !programStarted) {
            programStarted = true;
            currentRow = 0;
            currentPegCount = 0;
            Serial.println("Starting knitting program...");
            startNewRow();
        }
    }

    // Only run the main program logic if it has been started
    if (programStarted) {
        // Run the stepper motor if it's still running
        if (motorRunning) {
            stepper.runSpeed(); // Use runSpeed for indefinite movement
        }
        
        // Check for peg detection
        if (detectPeg()) {
            currentPegCount++;
            Serial.print("Peg detected! Count: ");
            Serial.println(currentPegCount);
            
            // Move to next row if the target number of pegs plus extraPegs is reached
            if (currentPegCount >= pegsPerRow) {
                motorRunning = false;             // Clear motor running flag
                setMotorEnabled(false);           // Disable motor between rows
                Serial.println("Row complete.");
                delay(400);                       // Pause before starting the next row
                startNewRow();                    // Start the next row or stop motor
            }
        }
    }
}