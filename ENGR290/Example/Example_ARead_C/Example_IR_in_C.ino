#include <avr/io.h>
#include <math.h>

#define IR_PIN 0 // Analog pin A0 on Arduino Nano
#define LED 3  //led control
#define TRIG_PIN PORTB5 //led control on the arduino nano

void pinMode_Cus(uint8_t pin, uint8_t mode) {
  if (pin == IR_PIN) { //if its an ANALOG GPIO
    if (mode == INPUT) {
      DDRC &= ~(1 << pin); // Clear the bit in the data direction register to make it an input
    } else if (mode == OUTPUT) {
      DDRC |= (1 << pin); // Set the bit in the data direction register to make it an output
    }
  } 
}

float calculate_distance(int raw_value) {
    float volts = (raw_value / 1024.0) * 5.0;
    float distance = 27.728 * pow(volts, -1.2045); // Formula from the datasheet
    return distance;
}

uint16_t analogRead_cus(uint8_t pin) {
    ADMUX = (ADMUX & 0xF8) | (pin & 0x07); // select ADC channel
    ADCSRA |= (1 << ADSC); // start conversion
    while (ADCSRA & (1 << ADSC)); // wait for conversion to complete
    return ADC; // return the ADC value
}


  void setup(){
    // Setup code
    Serial.begin(9600);
    pinMode_Cus(IR_PIN,INPUT);
 }
    // Main loop
  void loop(){
        // Read analog value from IR sensor
        int raw_value = analogRead_cus(IR_PIN); // Read ADC result

        // Calculate distance
        float distance = calculate_distance(raw_value);
        Serial.println(distance);
        

        // Main code (could do something with the distance value)
    }

