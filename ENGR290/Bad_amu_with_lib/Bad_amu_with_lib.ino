#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
#define LED_L 13
#define LED_D3 11
#define SERVO_PIN 6

float g = 9.8;
int brighness_D3 = 0;

Servo myservo;

MPU6050 mpu(Wire);

unsigned long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  myservo.attach(SERVO_PIN);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

Wire.setWireTimeout(3000, true);
}

void loop() {
  mpu.update();

  if(millis() - timer > 1000){ // print data every second
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    delay(1);
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    delay(1);
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());

    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    delay(1);
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    delay(1);
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
    delay(1);

    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    delay(1);
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    delay(1);

    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    delay(1);
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    delay(1);
    Serial.print("\tYaw: ");Serial.println(mpu.getAngleZ());
    delay(1);
    Serial.println(F("=====================================================\n"));
    delay(1);
    timer = millis();



  }
  delay(1);
  write_servo(mpu.getAngleZ());
  control_D3(mpu.getAccX());



}


 void write_servo(int yaw){//yaw = mpu.getAngleZ();
      if(abs(yaw) <= 85){
    myservo.write(yaw+90);
    delay(50);
    digitalWrite(LED_L, LOW);
    }
    else if(abs(yaw) > 85){
    digitalWrite(LED_L, HIGH);
    }
  }


 void control_D3(int x_acc){ // mpu.getAccX();
      if(abs(x_acc) < (0.08 * g)){
        brighness_D3 = 0;
        Serial.println("in");
      }
      if(abs(x_acc) > (1.08 * g)){
        brighness_D3 = 255;
        Serial.println("out");
      }
      if(abs(x_acc)  <= (1.08 * g) && abs(x_acc)  >= (0.08 * g) ){
        brighness_D3 = map(abs(x_acc),0.08,1.08,0,255);
        Serial.println("mid");
      }
      analogWrite(LED_D3, 255-brighness_D3); // inverted logic
      Serial.print("\Brightness: ");Serial.println(brighness_D3);
  }