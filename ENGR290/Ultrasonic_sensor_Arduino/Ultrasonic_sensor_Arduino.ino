
//using P10
//Arduino code for Arduino NANO
#define TRIG_PIN 13 //pb5// The Arduino pin connected to TRIG pin of ultrasonic sensor
#define ECHO_PIN 3 //pb // The Arduino pin connected to ECHO pin of ultrasonic sensor
#define LED 11 //pb3

float distance; // store the distance from sensor

void setup() {
  // begin serial port
  Serial.begin (9600);
  
  // Configure the trigger and echo pins to output mode
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); //INPUT_PULLUP
  pinMode(LED,OUTPUT);


}

void loop() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance = 0.017 * duration_us;

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  analogWrite(LED,Control_led_brightness(distance));

  delay(200);
  digitalWrite(TRIG_PIN, LOW);
}


int Control_led_brightness(int distance){
   int brightness = 0;
if(distance <= 15){
    brightness = 255;
    digitalWrite(TRIG_PIN, HIGH);
  }
  if(distance >= 40){
     brightness = 0;
     digitalWrite(TRIG_PIN, HIGH);
  }
  if(distance > 15 && distance < 40){
    brightness = 255 - map(distance,15,40,0,255); //not sure if mapping to 40,15 works but we can try
  }
  return 255-brightness; //logic is inverted for no reason
}
