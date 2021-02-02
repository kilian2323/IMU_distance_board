#ifndef PRESSURE_H
#define PRESSURE_H
#include <Arduino.h>
#include "config.h"

class Pressure {
  union SPI_data {
    byte packet[4];
    long value;
  }data;
public:
  volatile long airpressure;
  void init();
  byte cm();
};

#endif //PRESSURE_H
