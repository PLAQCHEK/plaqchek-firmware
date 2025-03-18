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

#define SCREEN_ADDRESS 0x3D	// 0x3D, based on jumpers
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ICONS */
#define LOGO_WIDTH 74
#define LOGO_HEIGHT 64
#define INDICATOR_WIDTH 7
#define INDICATOR_HEIGHT 7

// PLAQCHEK Logo : 74x64px
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
const unsigned char ble_logo [] PROGMEM = {
	0x18, 0x94, 0x52, 0x2c, 0x52, 0x94, 0x18
};

// Photodiode Logo : 7x7px
const unsigned char diode_logo [] PROGMEM = {
	0x44, 0x64, 0x74, 0xfe, 0x74, 0x64, 0x44
};

// Electrode Logo : 7x7px
const unsigned char electrode_logo [] PROGMEM = {
	0x3e, 0x7c, 0xf0, 0x7e, 0x1c, 0x30, 0x40
};

/* SPI */
#define SPI_CS D10	// SPI Chip Select

const u_int spi_clk_rate = 1*10^6;	// Clock Rate (1 MHz, but test with 1 MHz)
const SPISettings adc_settings = SPISettings(spi_clk_rate, MSBFIRST, SPI_MODE0);	// SPI Settings

#define VREF 5.0 // reference votlage for ADC
#define ADC_RES 65536.0	// Resolution for the ADC
uint16_t adc_val = 0;	// Latest ADC Value

/* LED Pins */
#define LED_1 D7	// LED 1
#define LED_2 D8	// LED 2
#define LED_3 D9	// LED 3

/* SW Pins */
#define SW1 A2	// REF
#define SW2 A0	// RESET
#define SW3 A1	// START
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
bool adc_state = false;	// PD State

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
#define DATA_SAMPLES 2000
float dark_ref_value = 0.0f; // Dark ref value
float lppla2_value = 0.0f;	// Lp-PLA2 value in ng/mL
String status_value = "";	// Status of sample
uint16_t progress_value = 0; 	// Progress value in %
uint16_t counter = 0;	// Tracking Data Aquisition
uint16_t ref_data[DATA_SAMPLES];	// Reference Data
uint16_t reading_data[DATA_SAMPLES];	// Reading Data

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
	void onConnect(BLEServer* pServer) override {
		Serial.println("Device Connected!");
		is_connected = true;
		BLEDevice::stopAdvertising();
	}
	void onDisconnect(BLEServer* pServer) override {
		Serial.println("Device Disconnected!");
		is_connected = false;
		BLEDevice::startAdvertising();
	}
};

// BLE Start Service Callback
class BLEStartCallback: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) override {
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
 * Draw Progress Bar
 */
void drawProgress() {
	// Draw Boundary
	display.drawRect(0, SCREEN_HEIGHT - 7, SCREEN_WIDTH, 7, WHITE);

	// Draw Fill
	display.fillRect(1, SCREEN_HEIGHT - 6, ((SCREEN_WIDTH - 2) * progress_value) / 100, 5, WHITE);
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
	display.setTextColor(WHITE);

	// Display Title
	display.setCursor(0, 0);
	display.println("PLAQCHEK");
	display.drawLine(0, 10, SCREEN_WIDTH, 10, WHITE);  // Divider line

	// Indicators
	// BLE Connection Icon
	if (is_connected) {
		display.drawBitmap(SCREEN_WIDTH - INDICATOR_WIDTH, 0, ble_logo, INDICATOR_WIDTH, INDICATOR_HEIGHT, WHITE);
	}
	// Diode On Icon
	if (adc_state) {
		display.drawBitmap(SCREEN_WIDTH - (2*INDICATOR_WIDTH + 1), 0, diode_logo, INDICATOR_WIDTH, INDICATOR_HEIGHT, WHITE);
	}
	// PWM On Icon
	if (pwm_state) {
		display.drawBitmap(SCREEN_WIDTH - (3*INDICATOR_WIDTH + 2), 0, electrode_logo, INDICATOR_WIDTH, INDICATOR_HEIGHT, WHITE);
	}
  
	// Different States
	if (!start_ref) {
		display.setCursor(0, 15);
		display.println("Click REF to read\ndark reference");
	} else if (!reference_done) {
		display.setCursor(0, 15);
		display.println("Reading dark\nreference...");
		display.setCursor(0, 35);
		display.printf("ADC Value: %u", adc_val);
		drawProgress();
	} else if (!started) {
		display.setCursor(0, 15);
		display.println("Dark reference set!\nClick START to begin");
		display.setCursor(0, 35);
		display.println("Click REF and RESET\nto redo reference");
	} else if (!reading_done) {
		display.setCursor(0, 15);
		display.println("Reading...");
		display.setCursor(0, 35);
		display.printf("ADC Value: %u", adc_val);
		drawProgress();
	} else {
		display.setCursor(0, 15);
		display.printf("LpPLA2 Conc: %.2f ng/mL\r\n", lppla2_value);
		display.setCursor(0, 35);
		display.printf("Risk Level: %s\r\n", lppla2_value > 200 ? "POS" : "NEG");
		// Maybe some icon here
		display.setCursor(0, 45);
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

	// Booting Lights Initial
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

		// Boot Success
		digitalWrite(LED_1, HIGH);
		digitalWrite(LED_2, HIGH);
		digitalWrite(LED_3, LOW);
		drawBoot();
	}

	// Setup Temperature Sensor
	pinMode(TEMP_PIN, INPUT);

	// Setup Potentiostat
	pinMode(POTENT_PIN, OUTPUT);
	digitalWrite(POTENT_PIN, LOW);

	// Setup SPI
	pinMode(SPI_CS, OUTPUT);
	digitalWrite(SPI_CS, HIGH);
	SPI.begin();

	// SPI Success
	digitalWrite(LED_1, LOW);
	digitalWrite(LED_2, HIGH);
	digitalWrite(LED_3, HIGH);

	// Setup PD_EN
	pinMode(PD_PIN, OUTPUT);
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
	lpPLA2Characteristic->setValue(lppla2_value);

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
	progressCharacteristic->setValue(progress_value);

	startCharacteristic = mainService->createCharacteristic(
											START_UUID,
											BLECharacteristic::PROPERTY_WRITE
										);
	startCharacteristic->setCallbacks(new BLEStartCallback());

	// Start Main Service
	mainService->start();

	// BLE Service Start
	digitalWrite(LED_1, LOW);
	digitalWrite(LED_2, LOW);
	digitalWrite(LED_3, HIGH);

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

	// Advertising Start
	digitalWrite(LED_1, HIGH);
	digitalWrite(LED_2, LOW);
	digitalWrite(LED_3, LOW);

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
	} else if (reference_done && !started && sw3_state == HIGH) { // Started reading
		started = true;
	} else if (reading_done && sw2_state == HIGH) { // Reset current reading
		started = false;
		reading_done = false;
	} else if (reference_done && !started && sw2_state == HIGH && sw1_state == HIGH) { // Hard Reset
		start_ref = false;
		reference_done = false;
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
		pwm_state = !pwm_state;

		// Update state
		digitalWrite(POTENT_PIN, pwm_state ? HIGH : LOW);
		pwmCharacteristic->setValue(pwm_state ? "1" : "0");
		pwmCharacteristic->notify();
	}
}

/**
 * Toggle the ADC
 */
void toggle_adc(bool state) {
	adc_state = state;
	analogWrite(PD_PIN, adc_state ? 4095 : 0);
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
	Serial.printf("%u\n", adc_val);
	adcCharacteristic->setValue(adc_val);
	adcCharacteristic->notify();
}

/**
 * Calculate Dark Ref
 */
void calculateDarkRef() {
	uint32_t dark_sum = 0;

	for (uint16_t i = 0; i < DATA_SAMPLES; i++) {
		Serial.printf("%u\n", i);
		dark_sum += ref_data[i];
	}

	dark_ref_value = ((float) dark_sum / DATA_SAMPLES);

	Serial.println("dark ref calculated");
}

/**
 * Calculate Lp-PLA2 Conc
 */
void calculateLPPLA2() {
	uint32_t sum = 0;

	for (uint16_t i = 0; i < DATA_SAMPLES; i++) {
		sum += reading_data[i];
	}

	lppla2_value = -0.0538 * (((float) sum / DATA_SAMPLES) - dark_ref_value) + 295.63; // Calibration Curve

	// Negative check
	if (lppla2_value < 0.0f) {
		lppla2_value = 0.0f;
	}

	// Progress Notify
	lpPLA2Characteristic->setValue(lppla2_value);
	statusCharacteristic->setValue(lppla2_value > 200 ? "POS" : "NEG");
	lpPLA2Characteristic->notify();

	Serial.println("lppla2 conc calculated");
}

/**
 * Runs the reference aquiring step
 */
void run_ref_state() {
	// If start
	if (!adc_state) {
		Serial.println("start ref read");
		toggle_adc(true);
		counter = 0;
		progress_value = 0;
	// If progressing
	} else if (counter < DATA_SAMPLES) {
		read_adc();
		ref_data[counter] = adc_state;
		counter++;
		progress_value = (counter * 100) / DATA_SAMPLES;
	// Finishing
	} else {
		Serial.println("end ref read");
		toggle_adc(false);
		reference_done = true;
		progress_value = 100;
		calculateDarkRef();
	}

	// Progress Notify
	progressCharacteristic->setValue(progress_value);
	progressCharacteristic->notify();
}

/**
 * Runs the sample reading step
 */
void run_sample_state() {
	// Start if needed
	if (!adc_state) {
		Serial.println("start read");
		toggle_adc(true);
		counter = 0;
		progress_value = 0;
		pwm_state = false;
		previous_t_millis = millis();
	}

	// Run PWM
	update_pwm();

	// If finishing
	if (counter >= DATA_SAMPLES) {
		Serial.println("end read");
		toggle_adc(false);
		reading_done = true;
		progress_value = 100;
		calculateLPPLA2();
	// If progressing
	} else {
		read_adc();
		reading_data[counter] = adc_state;
		counter++;
		progress_value = (counter * 100) / DATA_SAMPLES;
	} 

	// Progress Notify
	progressCharacteristic->setValue(progress_value);
	progressCharacteristic->notify();
}

/**
 * Loop
 */
void loop() {
	// Run states
	// Read Dark Reference
	if(start_ref && !reference_done) {
		run_ref_state();
	// Read Sample
	} else if (started && !reading_done) {
		run_sample_state();
	// Update Buttons
	} else {
		update_states();
	}

	// Update UI
	if (DISPLAY_ATTACHED)
		drawUI();
}