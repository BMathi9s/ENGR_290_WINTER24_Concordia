#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define LED_PIN 3 // Assuming the LED is connected to pin 3 (PB3) on the Arduino Nano

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
uint8_t brightness = 0;
//int main(void) {
  void setup(){
    pwm_init();
    
  }

   // while (1) {
    void loop(){
        // Fade LED in
        for (brightness = 255; brightness > 0; brightness -= 5) {
            set_brightness(brightness);
            _delay_ms(50);
        }
    }
//}



//this translate that example :  

// #define LED 11//PORTB3  //led control

// void setup() {
//   // put your setup code here, to run once:
//   pinMode(LED,OUTPUT);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   for(int i =255;i>=0;i-=5){
//     analogWrite(LED,i);
//     delay(50);
//   }
// }
