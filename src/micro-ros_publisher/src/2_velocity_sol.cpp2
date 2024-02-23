#include <Arduino.h>
#include "Motor.h" // Include Arduino and Motor library headers

#define DirectionPin 4 // Define pin for motor direction
#define BaudRate 115200 // Define baud rate for serial communication

// Declare global variables for motor velocity, encoder counts, etc.
int velocity = 20;
int count_left = 0;
int count_right = 0;
long preMilliseconds = 0;
float dt = 0;

// Variables for phase, offset and position calculations
float lastPhase_left = 0;
int offset_left = 0;

float lastPhase_right = 0;
int offset_right = 0;



void setup() 
{
  // Initialize motor control and serial communication
  Motor.begin(BaudRate, DirectionPin, &Serial2);
  Serial.begin(115200);
  // Read initial motor positions to set offsets
  offset_left = Motor.readPosition(1);
  offset_right = -Motor.readPosition(2);
  lastPhase_left = offset_left;
  lastPhase_right = offset_right;
}


void loop() 
{
  // Execute code block every 30 milliseconds
  if (millis() - preMilliseconds >= 30)
  {
    // Calculate time elapsed since last loop iteration
    dt = (millis() - preMilliseconds) / 1000.0;
    preMilliseconds = millis();

    // Read current motor positions
    count_left = Motor.readSpeed(1);
    count_right = -Motor.readSpeed(2);

    // Send command to Dynamixel
    // Motor.turnWheel(1, LEFT, 10);
    // Motor.turnWheel(2, RIGHT, 10);
    

  }
}

