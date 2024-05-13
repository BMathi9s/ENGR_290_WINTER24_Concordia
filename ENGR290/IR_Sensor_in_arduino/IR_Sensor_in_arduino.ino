
//using P8
//Arduino code for Arduino NANO
#define IR A0 // IR sensor analog output pin
#define LED 11 //pb3
#define TRIG_PIN 13

void setup() {
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
  pinMode(TRIG_PIN,OUTPUT);
}

void loop() {
  int raw = analogRead(IR);
  float volts = (raw/1024.0) * 5.0;
  float distance = 27.728 * pow(volts, -1.2045); // formula from the datasheet

  Serial.print("Distance: ");
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