#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <Wire.h>
#include <SPI.h>

/* SPI */
#define SPI_CS D10	// SPI Chip Select

const u_int spi_clk_rate = 1*10^6;	// Clock Rate (100 MHz, but test with 1 MHz)
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
#define DEVICE_NAME "PLAQCHEK"	// Device Name to use on BLE connection
#define DEVICE_DESC "PLAQCHEK Heart Risk Detector"

void setup() {
	// Launch MSG
	Serial.begin(9600);
	Serial.println("Starting BLE work!");
	
	// Setup I2C
	Wire.begin();

	// Setup SPI
	pinMode(SPI_CS, OUTPUT);
	digitalWrite(SPI_CS, LOW);
	SPI.begin();

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
	BLEDevice::init(DEVICE_NAME);
	BLEServer *pServer = BLEDevice::createServer();
	BLEService *pService = pServer->createService(SERVICE_UUID);
	BLECharacteristic *pCharacteristic = pService->createCharacteristic(
											CHARACTERISTIC_UUID,
											BLECharacteristic::PROPERTY_READ |
											BLECharacteristic::PROPERTY_WRITE
										);
	pCharacteristic->setValue(DEVICE_DESC);
	pService->start();

	// Start Advertising
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
	pAdvertising->setMinPreferred(0x12);
	BLEDevice::startAdvertising();
	Serial.println("Characteristic defined! Now you can read it in your phone!");
}

// ADC Functions
uint16_t read_adc() {
	digitalWrite(SPI_CS, HIGH);				// Enable the chip
	SPI.beginTransaction(adc_settings); 	// Small delay for stability

	// Send 16 clock cycles to read ADC data
	uint8_t highByte = SPI.transfer(0x00);  // Read MSB
	uint8_t lowByte = SPI.transfer(0x00);   // Read LSB

	SPI.endTransaction();
	digitalWrite(SPI_CS, LOW); // Disable the chip
	
	// Combine MSB and LSB
	return (highByte << 8) | lowByte;
}

void loop() {
	delay(2000);
	uint16_t bit = read_adc();
	Serial.printf("ADC Bit: %d", bit);
}
