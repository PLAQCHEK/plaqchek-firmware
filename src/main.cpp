#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <Wire.h>
#include <SPI.h>

/* SPI */
#define SPI_CS D10	// SPI Chip Select

const u_int spi_clk_rate = 100*10^6;	// Clock Rate
const SPISettings adc_settings = SPISettings(spi_clk_rate, MSBFIRST, SPI_MODE0);	// SPI Settings

/* I2C */
#define DISPLAY_ADDRESS = 40	// I2C Device Address

/* LED Pins */
#define LED_1 D7	// LED 1
#define LED_2 D8	// LED 2
#define LED_3 D9	// LED 3

/* Temperature Sensor */
#define TEMP_PIN D2	// Temperature Sensor Input Pin

/* Potentiostat Sensor */
#define POTENT_PIN D3	// Potentiostat PWM Pin

/* Power Delivery */
#define PD_PIN A7	// PD_EN Pin

/* BLE */
#define SERVICE_UUID "42bce946-ada2-4607-9d2f-71ec54c0cdf4"			// Service UUID
#define CHARACTERISTIC_UUID "820fc77c-e787-4103-a731-3937e965bc95"	// Characteristic UUID

void setup() {
	// Launch MSG
	Serial.begin(9600);
	Serial.println("Starting BLE work!");
	
	// Setup I2C
	Wire.begin();

	// Setup SPI
	SPI.begin();
	pinMode(SPI_CS, OUTPUT);
	SPI.beginTransaction(adc_settings);

	// Setup LED
	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);
	pinMode(LED_3, OUTPUT);

	// Setup Temperature Sensor
	pinMode(TEMP_PIN, INPUT);

	// Setup Potentiostat
	pinMode(POTENT_PIN, OUTPUT);

	// Setup PD_EN
	analogWrite(PD_PIN, 4095);

	// Setup BLE
	BLEDevice::init("PLAQCHEK");
	BLEServer *pServer = BLEDevice::createServer();
	BLEService *pService = pServer->createService(SERVICE_UUID);
	BLECharacteristic *pCharacteristic = pService->createCharacteristic(
											CHARACTERISTIC_UUID,
											BLECharacteristic::PROPERTY_READ |
											BLECharacteristic::PROPERTY_WRITE
										);

	pCharacteristic->setValue("Hello World");
	pService->start();

	// This still is working for backward compatibility
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
	pAdvertising->setMinPreferred(0x12);
	BLEDevice::startAdvertising();
	Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
	delay(2000);
}