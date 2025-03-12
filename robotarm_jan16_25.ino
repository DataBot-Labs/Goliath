#include <AccelStepper.h>

// Pin Definitions for Stepper Motors and Limit Switches
const int NUM_MOTORS = 6;
const int limitSwitchPins[NUM_MOTORS] = {52, 50, 48, 46, 44, 42}; // Limit switch pins for each motor
const int aEncoderPins[NUM_MOTORS] = {53, 49, 45, 41, 37, 33};  
const int bEncoderPins[NUM_MOTORS] = {51, 47, 43, 39, 35, 31};  
const int stepPins[NUM_MOTORS] = {7, 6, 3, 4, 2, 5};            // Step pins for each motor
const int dirPins[NUM_MOTORS] = {30, 32, 34, 40, 38, 36};             // Direction pins for each motor

// Homing speed for each motor
const int homingSpeed[NUM_MOTORS] = {100, 300, 300, 50, 50, 200};

// Maximum speed for each motor after homing
const int maxSpeed[NUM_MOTORS] = {100, 400, 400, 100, 70, 400};

// Steps to move back after hitting the limit switch
const int stepsToReverse[NUM_MOTORS] = {11450, 6000, 10000, 6000, 6000, 6000};

//definitions of some varibles
int battery_state = 0; //TODO REAL READINGS OVER ANALOG INPUT
bool homingDone = false; //TODO  when not home we dont take imputs yes instead (error home arm first)
static int sliderX = 0, sliderY = 0, sliderZ = 0; // To store slider values
static int lastTouchedComponentID = -1; // Tracks the last touched slider

// Stepper Motor Configuration
AccelStepper steppers[NUM_MOTORS] = {
    AccelStepper(AccelStepper::DRIVER, stepPins[0], dirPins[0]),
    AccelStepper(AccelStepper::DRIVER, stepPins[1], dirPins[1]),
    AccelStepper(AccelStepper::DRIVER, stepPins[2], dirPins[2]),
    AccelStepper(AccelStepper::DRIVER, stepPins[3], dirPins[3]),
    AccelStepper(AccelStepper::DRIVER, stepPins[4], dirPins[4]),
    AccelStepper(AccelStepper::DRIVER, stepPins[5], dirPins[5])
};


// Buffer to store received command
String command = "";

void setup() {
  Serial.begin(115200);  // Initialize serial communication USB
  Serial1.begin(9600); // Initialize serial communication with Nextion Display : Match baud rate with Nextion

  // Initialize each motor
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].setMaxSpeed(maxSpeed[i]);
    steppers[i].setAcceleration(100);
    pinMode(limitSwitchPins[i], INPUT_PULLUP); // Set limit switches as input (NC)


  }
}

void loop() {
  // Read commands from serial input
  listenOnUSB();
  
  // Read commands from Nextion Display 
  listenOnDisplay();

  // Run steppers if moving
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].run();
  }
}

void listenOnUSB(){
  if (Serial.available() > 0) {
    char received = Serial.read();

    // Check for command termination
    if (received == '>') {
      processCommand(command);
      command = "";  // Clear the command buffer
    } else {
      command += received;  // Append to command buffer
    }
    }
}

void listenOnDisplay() {
    while (Serial1.available()) {
    uint8_t startByte = Serial1.read(); // Read the start byte

    if (startByte == 0x65) { // Touch event
      delay(10); // Wait for the full packet

      if (Serial1.available() >= 6) {
        int pageID = Serial1.read();      // Read Page ID (not used here)
        int componentID = Serial1.read(); // Read Component ID
        int eventType = Serial1.read();   // Read Event Type (0=release, 1=press)
        Serial1.read(); // Skip the stupid 0xFF bytes 3x
        Serial1.read();
        Serial1.read();

        // Handle touch event
        //Serial.print("Touch Event - Component ID: ");
        //Serial.println(componentID);

        if (pageID == 1 && componentID == 4 && eventType == 0) {
          homeFromDisplay(); // Call the function
        }
        else if (pageID == 17 && componentID == 3 && eventType == 1) {
          ikFromDisplay(); // Call the function
        }

        else if (pageID == 11 && componentID == 3 && eventType == 1){
        mm1_cw();
        }
        else if (pageID == 11 && componentID == 4 && eventType == 1){
        mm1_ccw();
        }
        else if ((pageID == 11 && componentID == 3 && eventType == 0) || (pageID == 11 && componentID == 4 && eventType == 0)) {
        steppers[0].stop();
        }

        else if (pageID == 12 && componentID == 3 && eventType == 1) {
        mm2_ccw();
        }
        else if (pageID == 12 && componentID == 4 && eventType == 1){
        mm2_cw();
        }
        else if ((pageID == 12 && componentID == 3 && eventType == 0) || (pageID == 12 && componentID == 4 && eventType == 0)) {
        steppers[1].stop();
        }

        else if (pageID == 13 && componentID == 3 && eventType == 1){
        mm3_cw();
        }
        else if (pageID == 13 && componentID == 2 && eventType == 1){
        mm3_ccw();
        }
        else if ((pageID == 13 && componentID == 2 && eventType == 0) || (pageID == 13 && componentID == 3 && eventType == 0)) {
        steppers[2].stop();
        }

        else if (pageID == 14 && componentID == 3 && eventType == 1){
        mm4_cw();
        }
        else if (pageID == 14 && componentID == 2 && eventType == 1){
        mm4_ccw();
        }
        else if ((pageID == 14 && componentID == 2 && eventType == 0) || (pageID == 14 && componentID == 3 && eventType == 0)) {
        steppers[3].stop();
        }

        else if (pageID == 15 && componentID == 3 && eventType == 1){
        mm5_cw();
        }
        else if (pageID == 15 && componentID == 2 && eventType == 1){
        mm5_ccw();
        }
        else if ((pageID == 15 && componentID == 2 && eventType == 0) || (pageID == 15 && componentID == 3 && eventType == 0)) {
        steppers[4].stop();

        }
        else if (pageID == 16 && componentID == 2 && eventType == 1){
        mm6_cw();
        }
        else if (pageID == 16 && componentID == 3 && eventType == 1){
        mm6_ccw();
        }
        else if ((pageID == 16 && componentID == 2 && eventType == 0) || (pageID == 16 && componentID == 3 && eventType == 0)) {
        steppers[5].stop();
        }

        // Update the last touched slider
        lastTouchedComponentID = componentID;
      }
    } else if (startByte == 0x71) { // Release event (value update)
      delay(10); // Wait for the full packet

      if (Serial1.available() >= 6) {
        int sliderValue = Serial1.read(); // Read the slider value
        Serial1.read(); // Skip unused bytes
        Serial1.read();
        Serial1.read(); // Skip 0xFF
        Serial1.read();
        Serial1.read();

        // Map the value to the last known touched slider
        if (lastTouchedComponentID == 4) { // Slider Y
          sliderY = sliderValue;
          Serial.print("Slider Y Updated: ");
          Serial.println(sliderY);
        } else if (lastTouchedComponentID == 5) { // Slider X
          sliderX = sliderValue;
          Serial.print("Slider X Updated: ");
          Serial.println(sliderX);
        } else if (lastTouchedComponentID == 6) { // Slider Z
          sliderZ = sliderValue;
          Serial.print("Slider Z Updated: ");
          Serial.println(sliderZ);
        }

        // Reset lastTouchedComponentID after processing
        lastTouchedComponentID = -1;
      }
    }
  }
}

 
void homeFromDisplay(){
  // Update number field if the correct button is pressed
            // Example: Button with ID 0 released
                homeSteppers();
                Serial1.print("jcalpro.val=");
                Serial1.print(20); // Example value to send
                Serial1.write(0xff);
                Serial1.write(0xff);
                Serial1.write(0xff);
                
                
                Serial1.print("tcal.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
                Serial1.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
                Serial1.print("arm calibrated!");  // This is the text you want to send to that object and atribute mentioned before.
                Serial1.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
                Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
                Serial1.write(0xff);
                Serial1.write(0xff);
                homingDone = 1;

            }
            


void processCommand(String cmd) {
  if (cmd.startsWith("<ik:")) {
    handleInverseKinematics(cmd);
  } else if (cmd.startsWith("<po:")) {
    handlePositionControl(cmd);
  } else if (cmd.startsWith("<ho:")) {
    homeSteppers();
  } else if (cmd.startsWith("<sp:")) {
    handleSetSpeeds(cmd);  
  } else {
    Serial.println("Unknown command");
  }
 
}

void handleInverseKinematics(String cmd) {
  // Extract XYZ values
  cmd.remove(0, 4);  // Remove '<ik:'

  
  float x = cmd.substring(0, cmd.indexOf(',')).toFloat();
  cmd = cmd.substring(cmd.indexOf(',') + 1);
  float y = cmd.substring(0, cmd.indexOf(',')).toFloat();
  float z = cmd.substring(cmd.indexOf(',') + 1).toFloat();

  // Calculate motor positions based on IK (Placeholder)
  // This is where your IK logic should go
  int motorPositions[NUM_MOTORS] = {100, 200, 300, 400, 500, 600};  // Example positions

  // Move motors to calculated positions
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].moveTo(motorPositions[i]);
  }
  Serial.println("End Effector Has Reached Position"); 
  Serial.println(x);
  Serial.println(y);
  Serial.println(z); 
  
}

void ikFromDisplay() {
  // Print the slider values
  Serial.println("Executing ik solver");
  Serial.print("x: ");
  Serial.println(sliderX);
  Serial.print("y: ");
  Serial.println(sliderY);
  Serial.print("z: ");
  Serial.println(sliderZ);
}

void handlePositionControl(String cmd) {
  // Extract motor positions
  cmd.remove(0, 4);  // Remove '<po:'
  
  
  int positions[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    int index = cmd.indexOf(',');
    if (index == -1) {
      // If no comma is found, this is the last position
      positions[i] = cmd.toInt();
      break;
    }

    positions[i] = cmd.substring(0, index).toInt();
    cmd = cmd.substring(index + 1);
  }

  // Move motors to specified positions
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].moveTo(positions[i]);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" has moved to: ");
    Serial.println(positions[i]);
  }
}

void handleSetSpeeds(String cmd) {
  // Extract motor speeds
  cmd.remove(0, 4);  // Remove '<sp:'

  int speeds[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    int index = cmd.indexOf(',');
    if (index == -1) {
      speeds[i] = cmd.toInt();
      break;
    }
    speeds[i] = cmd.substring(0, index).toInt();
    cmd = cmd.substring(index + 1);
  }

  // Set speeds for each motor
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].setMaxSpeed(speeds[i]);
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" speed set to: ");
    Serial.println(speeds[i]);
  }
}

void homeSteppers() {
  // Homing sequence for all motors
    homeAllSteppers(); // look for limitswitches
    moveAllToHome();  // Set current position as "home"
    delay(40);
    Serial.println("All motors homed.");
  }
  
  
// Function to home all stepper motors
void homeAllSteppers() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("Homing motor ");
    Serial.println(i + 1);

    // Move the motor until the limit switch is pressed
    while (digitalRead(limitSwitchPins[i]) == HIGH) {
      steppers[i].setSpeed(-homingSpeed[i]);
      steppers[i].runSpeed();
    }

    // Once the limit switch is pressed, stop the motor
    steppers[1].setCurrentPosition(-stepsToReverse[i]);
    steppers[2].setCurrentPosition(-stepsToReverse[i]);
    steppers[3].setCurrentPosition(-stepsToReverse[i]);
    steppers[4].setCurrentPosition(-stepsToReverse[i]);
    steppers[5].setCurrentPosition(-stepsToReverse[i]);
    steppers[6].setCurrentPosition(-stepsToReverse[i]);
  
    steppers[i].setSpeed(0);
    steppers[i].runSpeedToPosition(); // Ensure precise stopping

    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.println(" homed.");
  }

  Serial.println("All motors homed.");
  homingDone = true;
}

// Function to move all motors to the zero (home) position
void moveAllToHome() {
  Serial.println("Moving all motors to start position...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].setMaxSpeed(maxSpeed[i]);
    steppers[i].setAcceleration(620);
    steppers[i].moveTo(0);
  }

  // Ensure all motors reach home position
  bool allDone = false;
  while (!allDone) {
    allDone = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (steppers[i].distanceToGo() != 0) {
        steppers[i].run();
        }
    }
  }

  Serial.println("All motors at start position.");
}

// functions for manually moving one motor at a time 
void mm1_cw() {
  steppers[0].setSpeed(maxSpeed[0]);
  steppers[0].move(-1000);
  steppers[0].run(); 
  }

void mm1_ccw() {
  steppers[0].setSpeed(maxSpeed[0]);
  steppers[0].move(1000);
  steppers[0].run(); 
  }

void mm2_cw() {
  steppers[1].setSpeed(maxSpeed[1]);
  steppers[1].move(-4000);
  steppers[1].run(); 
  }

void mm2_ccw() {
  steppers[1].setSpeed(maxSpeed[1]);
  steppers[1].move(4000);
  steppers[1].run(); 
  }

void mm3_cw() {
  steppers[2].setSpeed(maxSpeed[2]);
  steppers[2].move(-3000);
  steppers[2].run(); 
  }

void mm3_ccw() {
  steppers[2].setSpeed(maxSpeed[2]);
  steppers[2].move(3000);
  steppers[2].run(); 
  }

void mm4_cw() {
  steppers[3].setSpeed(maxSpeed[3]);
  steppers[3].move(1000);
  steppers[3].run(); 
  }

void mm4_ccw() {
  steppers[3].setSpeed(maxSpeed[3]);
  steppers[3].move(-1000);
  steppers[3].run(); 
  }

void mm5_cw() {
  steppers[4].setSpeed(maxSpeed[4]);
  steppers[4].move(1000);
  steppers[4].run(); 
  }

void mm5_ccw() {
  steppers[4].setSpeed(maxSpeed[4]);
  steppers[4].move(-1000);
  steppers[4].run(); 
  }

void mm6_cw() {
  steppers[5].setSpeed(maxSpeed[5]);
  steppers[5].move(-1000);
  steppers[5].run(); 
  }

void mm6_ccw() {
  steppers[5].setSpeed(maxSpeed[5]);
  steppers[5].move(1000);
  steppers[5].run(); 
  }