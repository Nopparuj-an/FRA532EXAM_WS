#include <Arduino.h>
#include "Motor.h"

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
  // offset_left = Motor.readPosition(1);
  // offset_right = -Motor.readPosition(2);
  Motor.turnWheel(1, LEFT, 500);
  delay(500);
  // Motor.turnWheel(2, RIGHT, 100);
}

int M1_pos_last = 0;
int trig_count = 0;

void loop() 
{
  // Serial.print(Motor.readSpeed(1));
  // Serial.print(" ");
  // Serial.print(Motor.readSpeed(2));
  // Serial.print(" ");
  // Serial.print(Motor.readPosition(1));
  // Serial.print(" ");
  // Serial.println(Motor.readPosition(2));
  // Serial.println();

  int M1_pos = Motor.readPosition(1);
  if (abs(M1_pos - M1_pos_last) > 150) {
    trig_count++;
    char buffer[100]; // Adjust size as needed
    sprintf(buffer, "M1 Triggered count[%d] millis[%lu] speed[%i]", trig_count, millis(), Motor.readSpeed(1));
    Serial.println(buffer);
  }
  M1_pos_last = M1_pos;
}

