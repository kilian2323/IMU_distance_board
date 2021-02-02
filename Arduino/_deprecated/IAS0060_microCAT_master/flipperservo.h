#ifndef FLIPPERSERVO_H
#define FLIPPERSERVO_H

#include <Arduino.h>
#include "minimoto.h"
#include "as5040.h"
#include "config.h"



class FlipperServo
{
  AS5040 feedback;
  MiniMoto hbridge;
  void move(int movement);
public:
  float frequency;
  int zero_direction;
  int calibrated_zero;
  int reversed, reversed_sense;
  int amplitude;
  int phase_offset;
  int servo_position;
  void initialize(byte motor_num);
  void update();
};

#endif //FLIPPERSERVO_H

