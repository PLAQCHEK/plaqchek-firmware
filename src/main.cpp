#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* OLED */
#define DISPLAY_ATTACHED true // Whether to use display or not
#define SCREEN_WIDTH 128	// OLED display width, in pixels
#define SCREEN_HEIGHT 64	// OLED display height, in pixels

#define OLED_RESET -1	// Reset pin; -1 if sharing Arduino reset pin

#define SCREEN_ADDRESS 0x3D	// 0x3C-0x3D, based on jumpers
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// PLAQCHEK Logo : 74x64px
#define LOGO_WIDTH 74
#define LOGO_HEIGHT 64
const unsigned char logo_bitmap [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xe0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x03, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xe3, 0xff, 0xf1, 0xff, 0xff, 
	0xf0, 0x00, 0x03, 0xff, 0xfe, 0x00, 0x7f, 0x80, 0x1f, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xfc, 0x00, 
	0x1e, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xf8, 0x00, 0x0c, 0x00, 0x07, 0xff, 0xf0, 0x00, 
	0x03, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x03, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 
	0x01, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xf0, 0x00, 0x03, 0xff, 
	0xc0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xff, 
	0xf0, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 
	0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xc0, 0x03, 0x00, 0x00, 
	0x00, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xc0, 0x07, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x03, 0xff, 
	0xc0, 0x05, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xe0, 0x0d, 0x80, 0x40, 0x01, 0xff, 
	0xf0, 0x00, 0x03, 0xff, 0xe0, 0x18, 0x80, 0x60, 0x01, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xe0, 0x10, 
	0x80, 0xf0, 0x01, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xf0, 0x30, 0xc1, 0xb1, 0xff, 0xff, 0xf0, 0x00, 
	0x03, 0xff, 0xff, 0xe0, 0x41, 0x1b, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xf8, 0x00, 0x43, 0x0e, 
	0x07, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xf8, 0x00, 0x66, 0x0c, 0x07, 0xff, 0xf0, 0x00, 0x03, 0xff, 
	0xfc, 0x00, 0x26, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xfe, 0x00, 0x2c, 0x00, 0x1f, 0xff, 
	0xf0, 0x00, 0x03, 0xff, 0xfe, 0x00, 0x38, 0x00, 0x1f, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0x00, 
	0x18, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0x80, 0x10, 0x00, 0x7f, 0xff, 0xf0, 0x00, 
	0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 
	0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x01, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 
	0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x07, 0xff, 0xff, 
	0xf0, 0x00, 0x03, 0xff, 0xff, 0xfc, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xfe, 
	0x00, 0x1f, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xf0, 0x00, 
	0x03, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xff, 0xe1, 0xff, 
	0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xff, 0xf3, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 
	0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xfe, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// BLE Logo : 7x7px
#define BLE_LOGO_WIDTH 7
#define BLE_LOGO_HEIGHT 7
const unsigned char ble_logo [] PROGMEM = {
	0x18, 0x94, 0x52, 0x2c, 0x52, 0x94, 0x18
};

/* SPI */
#define SPI_CS D10	// SPI Chip Select

const u_int spi_clk_rate = 1*10^6;	// Clock Rate (100 MHz, but test with 1 MHz)
const SPISettings adc_settings = SPISettings(spi_clk_rate, MSBFIRST, SPI_MODE0);	// SPI Settings

#define VREF 5.0 // reference votlage for ADC
#define ADC_RES 65536.0	// Resolution for the ADC
uint16_t adc_val = 0;	// Latest ADC Value
float exact_adc_val = 0.0f;	// Exact Voltage from ADC

/* LED Pins */
#define LED_1 D7	// LED 1
#define LED_2 D8	// LED 2
#define LED_3 D9	// LED 3

/* SW Pins */
#define SW1 A2
#define SW2 A0
#define SW3 A1
bool sw1_state = LOW;
bool sw2_state = LOW;
bool sw3_state = LOW;

/* Temperature Sensor */
#define TEMP_PIN D2	// Temperature Sensor Input Pin

/* Potentiostat Sensor */
#define POTENT_PIN D3	// Potentiostat PWM Pin
#define PWM_PERIOD_MS 1000	// Period in ms.
unsigned long previous_t_millis = 0;	// For time tracking
bool pwm_state = false;	// Current state of PWM

/* Power Delivery */
#define PD_PIN A7	// PD_EN Pin

/* BLE */
#define DEVICE_NAME "PLAQCHEK"	// Device Name to use on BLE connection
#define MAINSERVICE_UUID "00010000-ada2-4607-9d2f-71ec54c0cdf4"		// Main Service UUID
#define DEVSERVICE_UUID "000D0000-ada2-4607-9d2f-71ec54c0cdf4"		// Dev Service UUID

// Main Service BLE Characteristics
BLECharacteristic *lpPLA2Characteristic,	// LpPLA2 Data
				  *statusCharacteristic,	// Status Data
				  *progressCharacteristic,	// Progress Data
				  *startCharacteristic;		// Start Data

#define LPPLA2_UUID "00010001-ada2-4607-9d2f-71ec54c0cdf4"	// Lp-PLA2 Characteristic UUID
#define STATUS_UUID "00010002-ada2-4607-9d2f-71ec54c0cdf4"	// Status Characteristic UUID
#define PROGRESS_UUID "00010003-ada2-4607-9d2f-71ec54c0cdf4"	// Progress Characteristic UUID
#define START_UUID "00010004-ada2-4607-9d2f-71ec54c0cdf4"	// Start Characteristic UUID

// Main Service BLE Characteristics
BLECharacteristic *adcCharacteristic,	// ADC Data
				  *pwmCharacteristic;	// PWM Data

#define ADC_UUID "000D0001-ada2-4607-9d2f-71ec54c0cdf4"	// ADC Characteristic UUID
#define PWM_UUID "000D0002-ada2-4607-9d2f-71ec54c0cdf4"	// PWM Characteristic UUID

bool is_connected = false;	// Connection status

/* Data Parameters */
float lppla2_value = 0.0f;	// Lp-PLA2 value in ppm
String status_value = "";	// Status of sample
u_int16_t progress_value = 0; 	// Progress value in %

/* UI Parameters */
bool start_ref = false;
bool reference_done = false;
bool started = false;
bool reading_done = false;

/* Start Service Commands */
#define START "start"
#define RESET "reset"
#define HARDRESET "hard"

// BLECallback for handling successful connections
class BLECallback : public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		Serial.println("Device Connected!");
		is_connected = true;
		BLEDevice::stopAdvertising();
	}
	void onDisconnect(BLEServer* pServer) {
		Serial.println("Device Disconnected!");
		is_connected = false;
		BLEDevice::startAdvertising();
	}
};

// BLE Start Service Callback
class BLEStartCallback: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		std::string value = pCharacteristic->getValue();

		// Just print the received msg
		Serial.print("ble start service received: ");
		if (value.length() > 0) {
			for (int i = 0; i < value.length(); i++)
				Serial.print(value[i]);
		}
		Serial.print("\n");

		// Process the command
		if (value == START) {
			// Grab dark reference if not already
			if (!reference_done) {
				start_ref = true;
			// Start reading
			} else {
				started = true;
			}
		// Reset for new reading
		} else if (value == RESET) {
			started = false;
			reading_done = false;
		// Reset to redo dark reference
		} else if (value == HARDRESET) {
			start_ref = false;
			reference_done = false;
			started = false;
			reading_done = false;
		}
	}
};

/**
 * Draw Boot Screen
 */
void drawBoot() {
	// Make sure the display is cleared
	display.clearDisplay(); 

	// Draw the bitmap:
	display.drawBitmap((SCREEN_WIDTH - LOGO_WIDTH)/2, (SCREEN_HEIGHT - LOGO_HEIGHT)/2, logo_bitmap, LOGO_WIDTH, LOGO_HEIGHT, WHITE);
  
	// Update the display
	display.display();
}

/**
 * Draw the current UI
 * 
 * Note: Letters take up 7x5 pixels with 1 pixel spacing and 1 pixel spacing 
 * between lines
 */
void drawUI() {
	// Make sure the display is cleared
	display.clearDisplay();

	display.setTextSize(1);    
	display.setTextColor(SSD1306_WHITE);

	// Display Title
	display.setCursor(0, 0);
	display.println("PLAQCHEK");
	display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);  // Divider line

	// BLE Connection Icon
	if (is_connected) {
		display.drawBitmap(SCREEN_WIDTH - BLE_LOGO_WIDTH, 0, ble_logo, BLE_LOGO_WIDTH, BLE_LOGO_HEIGHT, WHITE);
	}
  
	// Different States
	if (!start_ref) {
		display.setCursor(0, 15);
		display.println("Click REF to read\ndark reference");
	} else if (!reference_done) {
		display.setCursor(0, 15);
		display.println("Reading dark\nreference...");
	} else if (!started) {
		display.setCursor(0, 15);
		display.println("Dark reference set!\nClick START to begin");
	} else if (!reading_done) {
		display.setCursor(0, 15);
		display.println("Reading...");
	} else {
		display.setCursor(0, 15);
		display.printf("Pred Lp-PLA2 Conc: %.2f ng/mL\r\n", lppla2_value);
		display.setCursor(0, 25);
		display.printf("Risk Level: %s\r\n", lppla2_value > 200 ? "POS" : "NEG");
		// Maybe some icon here
		display.setCursor(0, 35);
		display.println("Reset to start again");
	}

	display.display();  // Render to the screen
}

/**
 * Setup
 */
void setup() {
	// Setup LED
	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);
	pinMode(LED_3, OUTPUT);

	// Booting Lights
	digitalWrite(LED_1, HIGH);
	digitalWrite(LED_2, HIGH);
	digitalWrite(LED_3, HIGH);

	// Launch MSG
	Serial.begin(115200);

	// Initialize OLED (I2C)
	if (DISPLAY_ATTACHED) {
		if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
			Serial.println(F("SSD1306 allocation failed"));
			// HLH = OLED Failure
			digitalWrite(LED_1, HIGH);
			digitalWrite(LED_2, LOW);
			digitalWrite(LED_3, HIGH);
			for (;;); // Halt execution
		}
		drawBoot();
	}

	// Setup SPI
	pinMode(SPI_CS, OUTPUT);
	digitalWrite(SPI_CS, HIGH);
	SPI.begin();

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

	// Setup Main Service
	BLEService *mainService = pServer->createService(MAINSERVICE_UUID);

	// Setup Characteristics
	lpPLA2Characteristic = mainService->createCharacteristic(
											LPPLA2_UUID,
											BLECharacteristic::PROPERTY_READ |		
                        					BLECharacteristic::PROPERTY_NOTIFY
										);
	lpPLA2Characteristic->setValue("");

	statusCharacteristic = mainService->createCharacteristic(
											STATUS_UUID,
											BLECharacteristic::PROPERTY_READ
										);
	statusCharacteristic->setValue("");

	progressCharacteristic = mainService->createCharacteristic(
											PROGRESS_UUID,
											BLECharacteristic::PROPERTY_READ |		
                        					BLECharacteristic::PROPERTY_NOTIFY
										);
	progressCharacteristic->setValue("");

	startCharacteristic = mainService->createCharacteristic(
											START_UUID,
											BLECharacteristic::PROPERTY_WRITE
										);
	startCharacteristic->setCallbacks(new BLEStartCallback());

	// Start Main Service
	mainService->start();

	// Setup Dev Service
	BLEService *devService = pServer->createService(DEVSERVICE_UUID);

	adcCharacteristic = devService->createCharacteristic(
											ADC_UUID,
											BLECharacteristic::PROPERTY_READ |		
                        					BLECharacteristic::PROPERTY_NOTIFY
										);
	adcCharacteristic->setValue(adc_val);

	pwmCharacteristic = devService->createCharacteristic(
											PWM_UUID,
											BLECharacteristic::PROPERTY_READ |		
                        					BLECharacteristic::PROPERTY_NOTIFY
										);
	pwmCharacteristic->setValue("0");

	// Start Main Service
	devService->start();

	// Start Advertising
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(MAINSERVICE_UUID);
	pAdvertising->addServiceUUID(DEVSERVICE_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
	pAdvertising->setMinPreferred(0x12);
	BLEDevice::startAdvertising();

	// Setup Buttons
	pinMode(SW1, INPUT);
	pinMode(SW2, INPUT);
	pinMode(SW3, INPUT);

	// Final Message
	Serial.println("device boot ok");

	// Render UI
	if (DISPLAY_ATTACHED) {
		delay(1000); // so you can see boot screen
		drawUI();
	}

	// Reset Lights
	digitalWrite(LED_1, LOW);
	digitalWrite(LED_2, LOW);
	digitalWrite(LED_3, LOW);
}

/**
 * Detect Button Status
 */
void update_states() {
	sw1_state = digitalRead(SW1);
	sw2_state = digitalRead(SW2);
	sw3_state = digitalRead(SW3);

	// Update State
	if (!start_ref && sw1_state == HIGH) { // Started reference reading
		start_ref = true;
	} else if (!started && sw3_state == HIGH) { // Started reading
		started = true;
	} else if (reading_done && sw2_state == HIGH) { // Reset current reading
		started = false;
		reading_done = false;
	} 
}

/**
 * Update the PWM cycle for Potentiostat
 */
void update_pwm() {
	// Get Current Time
	unsigned long current_t_millis = millis();
	// Check if switch is needed
	if (current_t_millis - previous_t_millis >= PWM_PERIOD_MS / 2) {
		previous_t_millis = current_t_millis;
		pwm_state != pwm_state;
		// Update state
		digitalWrite(POTENT_PIN, pwm_state ? HIGH : LOW);
		pwmCharacteristic->setValue(pwm_state ? "1" : "0");
		pwmCharacteristic->notify();
	}
}

/**
 * Reads the ADC using SPI
 */
void read_adc() {
	// Start Transaction
	SPI.beginTransaction(adc_settings);

    // Bring CNVST LOW to enable data output
    digitalWrite(SPI_CS, LOW);
	delayMicroseconds(100);

    // Read ADC
    adc_val = SPI.transfer16(0x00);
	delayMicroseconds(100);

	// Terminate Transaction
	digitalWrite(SPI_CS, HIGH);
    SPI.endTransaction();

	// Update Exact
	adcCharacteristic->setValue(adc_val);
	adcCharacteristic->notify();
	exact_adc_val = (adc_val / ADC_RES) * VREF;
}

/**
 * Loop
 */
void loop() {
	// Update Buttons
	update_states();

	// Update PWM
	update_pwm();

	// Update ADC
	read_adc();

	// Logging
	Serial.printf("%d\n", adc_val);

	// Update UI
	if (DISPLAY_ATTACHED)
		drawUI();
}