#include <Arduino.h>
#include <BLEServer.h>
#include <Wire.h>
#include <SPI.h>

/* SPI Variables */
int spi_clk_rate = 100*10^6;
// SPI Chip Select Pin
int spi_cs = D10;

void setup() {
  // Setup I2C
  Wire.begin();
  // Setup SPI
  SPI.begin();
  pinMode(spi_cs, OUTPUT);
}

void loop() {
  
}