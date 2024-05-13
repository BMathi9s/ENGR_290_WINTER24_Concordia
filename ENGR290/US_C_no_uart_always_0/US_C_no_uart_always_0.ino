#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// #define TRIG_PIN 13 // PB5
// #define ECHO_PIN 3 // PD3
// #define LED_PIN 11  // PB3
#define TRIG_PIN PORTB5 // Arduino Nano physical pin 13 (Port B, Pin 5)
#define ECHO_PIN PD3 // Arduino Nano physical pin 3 (Port D, Pin 3)
#define LED_PIN PB3  // Arduino Nano physical pin 11 (Port B, Pin 3)


void pinMode_Cus(uint8_t pin, uint8_t mode) {
  if (mode == OUTPUT) {
    if (pin == TRIG_PIN) {
      DDRB |= (1 << PB5); // Set TRIG_PIN (PB5) as output
    } else if (pin == LED_PIN) {
      DDRB |= (1 << PB3); // Set LED_PIN (PB3) as output
    }
  } else if (mode == INPUT) {
    if (pin == ECHO_PIN) {
      //DDRB &= ~(1 << PB3); // Set ECHO_PIN (PB3) as input
      DDRD &= ~(1 << PD3);
    }
  }
}

void digitalWrite_Cus(uint8_t pin, uint8_t val) {
  if (val == LOW) {
    PORTB &= ~(1 << pin); // Clear the bit in the port register
  } else if (val == HIGH) {
    PORTB |= (1 << pin);  // Set the bit in the port register
  }
}

uint16_t pulseIn_c(uint8_t pin, uint8_t state) {
  uint16_t pulse_width = 0;
  while ((PINB & (1 << pin)) == state) {
    Serial.print("pw: ");
  Serial.println(pulse_width);
    if (pulse_width++ > 3000) // Timeout to avoid infinite loop
      return 0;
    _delay_us(1);
  }
  return pulse_width;
}


void pwm_init() {
  // Timer 2 initialization for PWM on pin PB3 (Arduino Nano D11)
  TCCR2A |= (1 << COM2A1) | (1 << WGM20) | (1 << WGM21); // Fast PWM mode, non-inverting
  TCCR2B |= (1 << CS21); // Prescaler 8
}

void setBrightness(uint8_t brightness) {
  // Set PWM duty cycle
  OCR2A = brightness;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int controlLedBrightness(float distance) {
  int brightness = 0;
  if (distance <= 15) {
    brightness = 255;
    digitalWrite_Cus(TRIG_PIN, HIGH);
  } else if (distance >= 40) {
    brightness = 0;
    digitalWrite_Cus(TRIG_PIN, HIGH);
  } else if (distance > 15 && distance < 40) {
    brightness = 255 - map((int)distance, 15, 40, 0, 255);
    digitalWrite_Cus(TRIG_PIN, LOW);
  }
  return 255 - brightness; // Logic is inverted for no reason
}

void setup() {
  Serial.begin(9600);
  pinMode_Cus(TRIG_PIN, OUTPUT);
  pinMode_Cus(ECHO_PIN, INPUT);
  pinMode_Cus(LED_PIN, OUTPUT);
  pwm_init();
}

void loop() {
  digitalWrite_Cus(TRIG_PIN, HIGH);
  _delay_us(10);
  digitalWrite_Cus(TRIG_PIN, LOW);

  uint16_t duration_us = pulseIn_c(ECHO_PIN, HIGH);

  float distance = 0.017 * duration_us;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  setBrightness(controlLedBrightness(distance));

  _delay_ms(200);
  digitalWrite_Cus(TRIG_PIN, LOW);
  _delay_ms(20);
}
