#include <Arduino.h>

#include <BLEServer.h>
#include <BLEAdvertising.h>

#include <Wire.h>
#include <SPI.h>

/* SPI Variables */
const u_int spi_clk_rate = 100*10^6; // Clock Rate
const u_int8_t spi_cs = D10; // SPI Chip Select
const SPISettings adc_settings = SPISettings(spi_clk_rate, MSBFIRST, SPI_MODE0); // SPI Settings

/* I2C Variables */
const int display_address = 40; // I2C Device Address

/* LED Pins */
const u_int8_t led_pins[3] = {
	D7,	// LED 1
	D8,	// LED 2
	D9	// LED 3
};

/* Temperature Sensor */
const u_int8_t temp_read_pin = D2; // Temperature Sensor Input Pin

/* Potentiostat Sensor */
const u_int8_t potentiostat_pin = D3; // Potentiostat PWM Pin

/* Power Delivery */
const u_int8_t pd_en_pin = A7; // PD_EN Pin

void setup() {
	// Setup I2C
	Wire.begin();

	// Setup SPI
	SPI.begin();
	pinMode(spi_cs, OUTPUT);
	SPI.beginTransaction(adc_settings);

	// Setup LED
	pinMode(led_pins[0], OUTPUT);
	pinMode(led_pins[1], OUTPUT);
	pinMode(led_pins[2], OUTPUT);

	// Setup Temperature Sensor
	pinMode(temp_read_pin, INPUT);

	// Setup Potentiostat
	pinMode(potentiostat_pin, OUTPUT);

	// Setup PD_EN
	analogWrite(pd_en_pin, 4095);
}

void loop() {

}