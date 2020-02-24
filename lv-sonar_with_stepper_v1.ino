/*
  Ultrasonic sensor with stepper motor [test]
  Emilostuff – 2020
*/

// sensor library
#include "Maxbotix.h"

// system parameters
const int currentPos = 800;
const int range = 4000;
const int stepInterval = 40;
const int delayLimit = 3000;

// Proportional control
int error;
int kp = 6;

// Sensor
float value = 0;
Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV, Maxbotix::BEST, 5);

// IO
const int stepX = 2;
const int dirX  = 5;
const int enPin = 8;



// Go to the position that correlates to the specific distance measured
int posFromDist(int dist) {
  if (dist > 400) {
    dist = 400;
  }
  int pos = map(dist, 0, 400, 0, range);
  return pos;
}


void step(float speed) {

  // speed is steps pr second

  // calculate direction
  bool direction;

  if (speed >= 0) {
    direction = HIGH;
  } else {
    direction = LOW;
  }

  digitalWrite(dirX, !direction);

  //calculate steps to take here
  int steps = (abs(speed) / 1000) * float(stepInterval);

  // limit steps to be equal to or less than the error (prevents overshoot and oscillations)
  if (steps > abs(error)) {
    steps = -error;
  }

  // calculate delay between steps in micros
  double delay = ((float(stepInterval) / float(steps)) * 1000) / 2;


  // limit delay to prevent steppers from moving too slow...
  if (delay > delayLimit) {
    delay = delayLimit + (delay - delayLimit) / 20;
  }

  // Limit delay to prevent moving too fast
  if (delay < 200) {
    delay = 200;
  }

  // The move loop
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepX, HIGH);
    delayMicroseconds(delay);
    digitalWrite(stepX, LOW);
    delayMicroseconds(delay);
    // update position
    currentPos = (direction) ? currentPos + 1 : currentPos - 1;
  }
}


void setup()
{
  Serial.begin(9600);

  // Set the delay between AD readings to 10ms
  rangeSensorAD.setADSampleDelay(2);

  // Sets pins
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  digitalWrite(dirX, HIGH);
}

void loop()
{
  // get reading
  value = value * 0 + rangeSensorAD.getRange() * 1;

  // print it – optional
  Serial.println(value);

  //Calculate error
  error = currentPos - posFromDist(value);

  // move
  step(-error * kp);

}
