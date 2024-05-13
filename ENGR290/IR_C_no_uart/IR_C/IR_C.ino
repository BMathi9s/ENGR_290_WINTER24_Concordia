
//Arduino code for Arduino NANO
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>
#define IR_PIN PORTC0 // IR_PIN sensor analog output pin
#define LED_PIN PB3  //led control 

#define TRIG_PIN PORTB5 //led control on the arduino nano


//define for pinMode
#define INPUT 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0

uint8_t brightness = 0;


void pinMode_Cus(uint8_t pin, uint8_t mode) {
  if (pin == IR_PIN) { //if its an ANALOG GPIO
    if (mode == INPUT) {
      DDRC &= ~(1 << pin); // Clear the bit in the data direction register to make it an input
    } else if (mode == OUTPUT) {
      DDRC |= (1 << pin); // Set the bit in the data direction register to make it an output
    }
  } else if (pin == LED_PIN || pin == TRIG_PIN) { //classic gpio
    if (mode == INPUT) {
      DDRB &= ~(1 << pin); // Clear the bit in the data direction register to make it an input
    } else if (mode == OUTPUT) {
      DDRB |= (1 << pin); // Set the bit in the data direction register to make it an output
    }
  }
}

uint16_t analogRead_cus(uint8_t pin) {
    ADMUX = (ADMUX & 0xF8) | (pin & 0x07); // select ADC channel
    ADCSRA |= (1 << ADSC); // start conversion
    while (ADCSRA & (1 << ADSC)); // wait for conversion to complete
    return ADC; // return the ADC value
}



void digitalWrite_cus(uint8_t pin, uint8_t val) {
    if (val == LOW) {
        // If the value is LOW, clear the bit in the port register
        PORTB &= ~(1 << pin);
    } else if (val == HIGH) {
        // If the value is HIGH, set the bit in the port register
        PORTB |= (1 << pin);
    }
}

long map_cus(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int Control_led_brightness(int distance){
   int brightness = 0;
if(distance <= 15){
    brightness = 255;
    digitalWrite_cus(TRIG_PIN, HIGH);
  }
  if(distance >= 40){
     brightness = 0;
     digitalWrite_cus(TRIG_PIN, HIGH);
  }
  if(distance > 15 && distance < 40){
    brightness = 255 - map_cus(distance,15,40,0,255); //not sure if mapping to 40,15 works but we can try
  }
  return 255-brightness; //logic is inverted for no reason
}

void pwm_init() {
    // Timer 2 initialization for PWM on pin PB3 (Arduino Nano D11)
    TCCR2A |= (1 << COM2A1) | (1 << WGM20) | (1 << WGM21); // Fast PWM mode, non-inverting
    TCCR2B |= (1 << CS21); // Prescaler 8
    DDRB |= (1 << LED_PIN); // Set PB3 as output
}

void set_brightness(uint8_t brightness) {
    // Set PWM duty cycle
    OCR2A = brightness;
}


void setup() {
  Serial.begin(9600);
  pinMode_Cus(IR_PIN,INPUT);
  pwm_init();
  //pinMode_Cus(LED,OUTPUT);
  pinMode_Cus(TRIG_PIN,OUTPUT);
  
}
void loop(){
  int raw = analogRead_cus(IR_PIN);
  float volts = (raw/1024.0) * 5.0;
  float distance = 27.728 * pow(volts, -1.2045); // formula from the datasheet

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  set_brightness(Control_led_brightness(distance));
  _delay_ms(100);
  digitalWrite_cus(TRIG_PIN, LOW);
}









