#include <SPI.h>

#define VREF 5.0 // reference votlage for ADC

// SPI stuff
#define SPI_CS D10 // CS pin for MCP3313
SPISettings adcsettings(1000000, MSBFIRST, SPI_MODE0);


// PWM stuff
#define PWM D3	// Potentiostat PWM Pin
#define PWM_PERIOD_MS 1000 // period in ms.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(SPI_CS, OUTPUT);
  pinMode(PWM, OUTPUT);

  digitalWrite(SPI_CS, HIGH);  // disable ADC

  SPI.begin();
}

uint8_t highByte, lowByte;
uint16_t adc_val;
float adc_v;

unsigned long previous_t_millis = 0;
bool pwm_state = LOW;

void pwm() {
	unsigned long current_t_millis = millis();
	if (current_t_millis - previous_t_millis >= PWM_PERIOD_MS / 2) {
		previous_t_millis = current_t_millis;
		pwm_state =! pwm_state;
		digitalWrite(PWM, pwm_state);
	}
}


void loop() {

  pwm(); // start PWM. it will always be running for now

  SPI.beginTransaction(adcsettings);
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(100);

  adc_val = SPI.transfer16(0);

  delayMicroseconds(100);

  digitalWrite(SPI_CS, HIGH);
  SPI.endTransaction();

  float adc_v = (adc_val / 65536.0) * VREF;

  Serial.printf("ADC value: %d ", adc_val);
  Serial.print("ADC value (V): ");
  Serial.print(adc_v, 3); 
  Serial.println();
  // Serial.printf("high bit: %d ", highByte);
  // Serial.printf("low bit: %d ", lowByte);
  delay(1000);
}
