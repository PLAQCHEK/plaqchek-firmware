#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* OLED */
#define SCREEN_WIDTH 128	// OLED display width, in pixels
#define SCREEN_HEIGHT 64	// OLED display height, in pixels

#define OLED_RESET -1	// Reset pin; -1 if sharing Arduino reset pin

#define SCREEN_ADDRESS 0x3D	// 0x3C-0x3D, based on jumpers
const Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
#define SERVICE_UUID "00010000-ada2-4607-9d2f-71ec54c0cdf4"			// Service UUID
#define DEVICE_NAME "PLAQCHEK"	// Device Name to use on BLE connection


// BLE Characteristics
BLECharacteristic *lpPLA2Characteristic,	// LpPLA2 Data
				  *statusCharacteristic,	// Status Data
				  *progressCharacteristic,	// Progress Data
				  *startCharacteristic;		// Start Data

#define LPPLA2_UUID "0001RN01-ada2-4607-9d2f-71ec54c0cdf4"	// Lp-PLA2 Characteristic UUID
#define STATUS_UUID "0001R002-ada2-4607-9d2f-71ec54c0cdf4"	// Status Characteristic UUID
#define PROGRESS_UUID "0001RN03-ada2-4607-9d2f-71ec54c0cdf4"	// Progress Characteristic UUID
#define START_UUID "000100W4-ada2-4607-9d2f-71ec54c0cdf4"	// Start Characteristic UUID

bool is_connected = false;	// Connection status

// BLECallback for handling successful connections
class BLECallback : public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		Serial.println("Device Connected!");
		is_connected = true;
	}
	void onDisconnect(BLEServer* pServer) {
		Serial.println("Device Disconnected!");
		is_connected = false;
	}
};

// Data Parameters
float lppla2_value = 0.0f;	// Lp-PLA2 value in ppm
String status_value = "";	// Status of sample
u_int16_t progress_value = 0; 	// Progress value in %

/**
 * Setup
 */
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

	// Connection Ping
	pServer->setCallbacks(new BLECallback());

	// Setup Service
	BLEService *pService = pServer->createService(SERVICE_UUID);

	// Setup Characteristics
	lpPLA2Characteristic = pService->createCharacteristic(
											LPPLA2_UUID,
											BLECharacteristic::PROPERTY_READ |		
                        					BLECharacteristic::PROPERTY_NOTIFY
										);
	lpPLA2Characteristic->setValue(0x00);

	statusCharacteristic = pService->createCharacteristic(
											STATUS_UUID,
											BLECharacteristic::PROPERTY_READ
										);
	statusCharacteristic->setValue("");

	progressCharacteristic = pService->createCharacteristic(
											PROGRESS_UUID,
											BLECharacteristic::PROPERTY_READ |		
                        					BLECharacteristic::PROPERTY_NOTIFY
										);
	progressCharacteristic->setValue(0x00);

	startCharacteristic = pService->createCharacteristic(
											START_UUID,
											BLECharacteristic::PROPERTY_WRITE
										);

	// Start Service
	pService->start();

	// Start Advertising
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
	BLEDevice::startAdvertising();

	// Advertising
	Serial.println("Device BLE started to scan...");
}

/**
 * Reads the ADC using SPI
 * 
 * @return x in x/max_resolution
 */
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

/**
 * Loop
 */
void loop() {
	uint16_t bit = read_adc();
	Serial.printf("ADC Bit: %d\n", bit);
	delay(2000);
}
