#include "pressure.h"
#include <SPI.h>

void Pressure::init() {
  pinMode(PRESSURE_CS, OUTPUT);
  digitalWrite(PRESSURE_CS, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  // Read and output pressure in case calibration is needed
  // trigger first conversion
  delay(1); // FIXME: I don't know if this is necessary
  digitalWrite(PRESSURE_CS, LOW);
  delay(1);
  digitalWrite(PRESSURE_CS, HIGH);
  delay(20);
  // first value is air pressure, read it now
  digitalWrite(PRESSURE_CS, LOW);
  data.packet[2] = SPI.transfer(0xAA);
  data.packet[1] = SPI.transfer(0x55);
  data.packet[0] = SPI.transfer(0xAA);
  digitalWrite(PRESSURE_CS, HIGH);
  // move the sign bit and clear overflow bits
  if (data.value & (1L << 21)) {
    data.value |= 0xFFC00000;
  }
  else {
    data.value &= ~0xFFC00000;
  }

  // convert to Pascals
  airpressure = (data.value * PRESSURE_SLOPE) + PRESSURE_OFFSET; // Pa
  Serial.print("Air pressure is ");
  Serial.println(airpressure, DEC);
#ifndef CALIBRATE_PRESSURE
  // This needs to be measured
  airpressure = AIR_PRESSURE;
#endif
  // trigger next conversion
  delay(1); // FIXME: I don't know if this is necessary
  digitalWrite(PRESSURE_CS, LOW);
  delay(1);
  digitalWrite(PRESSURE_CS, HIGH);
  delay(20);

}

byte Pressure::cm()
{
  byte depth;
  SPI.setDataMode(SPI_MODE3);
  digitalWrite(PRESSURE_CS, LOW);
  data.packet[2] = SPI.transfer(0xAA);
  data.packet[1] = SPI.transfer(0x55);
  data.packet[0] = SPI.transfer(0xAA);
  digitalWrite(PRESSURE_CS, HIGH);
  // move the sign bit and clear overflow bits
  if (data.value & (1L << 21))
    data.value |= 0xFFC00000;
  else
    data.value &= ~0xFFC00000;

  // convert to Pascals
  data.value = (data.value * PRESSURE_SLOPE) + PRESSURE_OFFSET;

  // and to cm
  float depth_f = float(data.value - airpressure) / 101.325;

  if (depth_f < 0)
    depth = 0;
  else if (depth_f > 255)
    depth = 255;
  else
    depth = depth_f;

  // trigger next conversion
  delay(1); // FIXME: I don't know if this is necessary
  digitalWrite(PRESSURE_CS, LOW);
  delay(1);
  digitalWrite(PRESSURE_CS, HIGH);
  return depth;
}
