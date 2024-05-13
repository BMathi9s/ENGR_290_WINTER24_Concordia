
//Author Mathias Desrochers 2024 16 april
//ENGR 290 - best time, 27 sec

//inspiration https://github.com/maarten-pennings/MPU6050/tree/master/examples/Pid

#include <Servo.h>
#include <Wire.h>
#include <mpu6050.h>
#include <HCSR04.h>
Servo myservo;
#define Fan_with_speed_ctl 5 // Fan speed control pin
#define Skirt_fan 7 // Skirt fan pin
#define servo_pin 6 // Servo control pin
// HC-SR04 (ultrasonisc sensor) pins
HCSR04 hc(13, 3);
// Define turn direction
bool right = true;
bool left = false;

#define FORWARD_ANGLE 90
#define RIGHT_TURN_ANGLE 180
#define LEFT_TURN_ANGLE 0
// Fine tuning

#define FORWARD_ANGLE 90
#define SPEED_FAN_NORMAL_RUNNING 255
#define RIGHT_TURN_ANGLE 180
#define LEFT_TURN_ANGLE 0
#define SERVO_TURN_DELAY_FOR_READ 400
#define SPEED_TURNING 255
#define TURN_DELAY 300
#define RIGHT_TURN_SMALL_ANGLE 130
#define LEFT_TURN_SMALL_ANGLE 40
int X = 3;
// Fine Tuning Turn

int WALL_DISTANCE =40;
int LAST_WALL_DISTANCE = 170;

int YAW_FULL_TURN = 180; //180
int YAW_ORIGIN = 0;  //0

int YAW_SMALL_TURN =35; //45
int YAW_TURN = 180;  //90
int YAW_MINI_TURN =70; //45

#define TIME_PID_SMALL_TURN 30 //idk exactly time
#define TIME_PID_MINI_TURN 3  //idk exactly time
#define TIME_PID_TURN 10
#define TIME_PID_NO_US 50

// PID constants
#define PID_K_p 0.5
#define PID_K_i 0.1
#define PID_K_d 2.0


// PID variables
float i_input = 0;
float d_last = 0;
int target_yaw = 0;

MPU6050 mpu6050;
void setup() {
    // Initialize servo and sensor
    myservo.attach(servo_pin);
    pinMode(Fan_with_speed_ctl, OUTPUT);
    pinMode(Skirt_fan, OUTPUT);
    
    // Initialize serial communication
    Serial.begin(9600);
    // Initialize MPU6050 and I2C
    mpu6050_begin();
    // Initialize PID variables
    pid_begin();
    // Set fan speed and lift skirt fan
    Initial_hover_set_up();
    // Allow time for initialization
    target_yaw = 0.0;
}

void loop() {

    while(hc.dist() >= WALL_DISTANCE){
        go_forward(target_yaw);
        Serial.println("check wall");
    }
    //turn
    make_a_better_decision();  
    //go forawrd without reading sensor
    just_PID(TIME_PID_NO_US);
}

void just_PID(int time){
  for(int i = 0; i<=time; i++){
    go_forward(target_yaw);
    } 
}


void make_a_better_decision() {
    static int count = 0; // Make count static to preserve its value across function calls
    if(count == 0){
      bool i = make_a_decision();
                if(i){
                  //set count to 0  (right > left)
                  count = 0;
                }
                if(!i){
                  //set count to 1 (left > right)
                  count = 1;
                }
                just_PID(20);
      }
      count++;
    if(count % 2){
      //turn right
      //small_turn(right);
       
      full_turn(right); // turn right    
      if(count == X){
      //uptade max wall distance to go out exit
       WALL_DISTANCE = LAST_WALL_DISTANCE;
       //maybe update targeted yaw
    }
    }
    if(!(count % 2)){
      //turn left
      //small_turn(left);
      //mini_turn(right);
      //small_turn(left); 
      full_turn(left); // turn left
      if(count == X+1){
      //uptade max wall distance to go out exit
       WALL_DISTANCE = LAST_WALL_DISTANCE;
       //maybe update targeted yaw
    }
    }

}

bool make_a_decision(){
  int left_dist;
      int right_dist;
   //look left
   //set speed fan 0
   set_fan_speed(0);
        
      turn(left);
      myservo.write(0);
      just_PID(6); 
      left_dist = hc.dist();
      delay(20);
       //look right
      turn(right);
      myservo.write(180);
      just_PID(6); 
      right_dist = hc.dist();
      delay(20);
      //set speed fan max
      set_new_target_yaw(0);
      set_fan_speed(255);
      //just_PID(20); 
       if(left_dist > right_dist){
        Serial.println("left");
         return false;
         
        }
      if(right_dist > left_dist){
        Serial.println("right");
        return true;
        }
}


void mini_turn(bool right) {
    if (right) {
        add_values_to_the_targeted_yaw(-YAW_MINI_TURN);
    } else {
       add_values_to_the_targeted_yaw(YAW_MINI_TURN);
    }
    just_PID(TIME_PID_MINI_TURN);
}

void small_turn(bool right) {
    if (right) {
        add_values_to_the_targeted_yaw(-YAW_SMALL_TURN);
    } else {
       add_values_to_the_targeted_yaw(YAW_SMALL_TURN);
    }
    just_PID(TIME_PID_SMALL_TURN);
}
void full_turn(bool right) {
    if (right) {  
        //add_values_to_the_targeted_yaw(-YAW_FULL_TURN);
        set_new_target_yaw(-YAW_FULL_TURN);
        
    } else {
        //add_values_to_the_targeted_yaw(YAW_FULL_TURN);
        set_new_target_yaw(YAW_ORIGIN);
    }
}

    void turn(bool right) {
    if (right) {
        set_new_target_yaw(-YAW_TURN);
    } else {
        set_new_target_yaw(YAW_TURN);
    }
}
void Initial_hover_set_up(){
    // myservo.write(0);
    // delay(300);
    // myservo.write(90);
    // delay(300);
    // myservo.write(180);
    // delay(300);
    myservo.write(FORWARD_ANGLE);
    lift_skirt(true);
    delay(500);
   set_fan_speed(SPEED_FAN_NORMAL_RUNNING);
  //delay(500);
}
// PID initialization function
void pid_begin() {
    i_input = 0;
    d_last = 0;
    Serial.println("PID control initialized");
}
void add_values_to_the_targeted_yaw(int add){
  // Adjust target yaw for a right turn
    target_yaw += add;
    // if(target_yaw <= -360){
    //   target_yaw = -360;
    // }
    // if(target_yaw >= 360){
    //   target_yaw =360;
    // }
}
void set_new_target_yaw(int target){
  target_yaw = target;
}
// PID control function
int pid(float error) {
    float p_input = error;
    i_input = constrain(i_input + error, -50, 50); // Constrain integral sum
    float d_input = error - d_last; 
    d_last = error;
    
    return p_input * PID_K_p + i_input * PID_K_i + d_input * PID_K_d;
}
// Go forward function with PID control
void go_forward(float target_yaw) {
    // Read current yaw from MPU6050
    float current_yaw = mpu6050_yaw();
    
    // Calculate error between target and current yaw
    float error = target_yaw - current_yaw;
    
    // Calculate correction using PID control
    int correction = pid(error);
    
    // Adjust servo angle based on correction
    int servo_angle = FORWARD_ANGLE - correction;
    servo_angle = constrain(servo_angle, 0, 180);
    myservo.write(servo_angle);
    // Print out the current yaw, target yaw, error, correction, and final servo angle
    delay(20);
    Serial.print("Current yaw: ");
    Serial.print(current_yaw, 2);
    Serial.print(", Target yaw: ");
    Serial.print(target_yaw, 2);
    Serial.print(", Error: ");
    Serial.print(error, 2);
    Serial.print(", Correction: ");
    Serial.print(correction);
    Serial.print(", Servo angle: ");
    Serial.println(servo_angle);
}
// MPU6050 yaw reading function
float mpu6050_yaw() {
    MPU6050_t data = mpu6050.get();
    while (data.dir.error != 0) {
        Serial.println(mpu6050.error_str(data.dir.error));
        // Reset I2C and reread the data
        TWCR = 0;
        Wire.begin();
        data = mpu6050.get();
    }
    return data.dir.yaw;
}
void lift_skirt(bool on) {
    digitalWrite(Skirt_fan, on ? HIGH : LOW);
}
// Function to set the speed of the main fan
void set_fan_speed(int speed) {
    analogWrite(Fan_with_speed_ctl, speed);
}
void mpu6050_begin()  {
  Wire.begin();
  Serial.print("MPU6050: Starting calibration; leave device flat and still ... ");
  int error= mpu6050.begin(); 
  Serial.println(mpu6050.error_str(error));
}
