#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  //set to initial rest position for SAFETY :O
  pwm.setPWM(0, 0, 130); //perpendicular to car
  delay(50);
  pwm.setPWM(1, 0, 380); //shoulder: obtuse angle back(rest position)
  delay(50);
  pwm.setPWM(2, 0, 470); //holds elbow parallel to ground in rest position
  delay(50);
  pwm.setPWM(3, 0, 350); //Wrist in rest position
  delay(50);
  pwm.setPWM(4, 0, 250); //gripper open
  delay(500);
  pwm.setPWM(0, 0, 290); //positioned over car
  delay(3000);
}

void loop() {
  pick_up();
  delay(3000);
  drop_off();
  delay(3000);
}

//declaring position functions
void rest_pos1(){
  for (int i = 290; i >= 130;) {
    pwm.setPWM(0, 0, i); //moving from over car to perpendicular to car
    i = i - 10;
    delay (50);
  }
  for (int i = 350; i <= 460;) {
    pwm.setPWM(3, 0, i); //wrist from rest to reaching
    i = i + 10;
    delay(50);
  }
  for (int i = 380; i >= 220; ) {
    pwm.setPWM(1, 0, i); //shoulder from (obtuse angle) rest to reaching
    i = i - 10;
    delay(50);
  }
}

void pos1_pos2(){
  for (int i = 460; i >= 400;) {
    pwm.setPWM(3, 0, i); //wrist from reaching pos1 to fully extended pos 2
    i = i - 10;
    delay(50);
  }
  for (int i = 220; i >= 210;) {
    pwm.setPWM(1, 0, i); //shoulder from reaching pos 1 to fully extended pos 2
    i = i - 10;
    delay(50);
  }
  for (int j=470; j>=380;){
    pwm.setPWM(2, 0, j); //elbow from horizontal(rest/pos1) to fully extended
    j = j-10;
    delay(50);
  }
  for (int i = 210; i >= 180;) {
    pwm.setPWM(1, 0, i); //shoulder from reaching pos 1 to fully extended pos 2
    i = i - 10;
    delay(50);
  }
  for (int j=380; j>=350;){
    pwm.setPWM(2, 0, j); //elbow from horizontal(rest/pos1) to fully extended
    j = j-10;
    delay(50);
  }
}

void pos2_pos1(){
 //should start at wrist then shoulder then elbow
  for (int i = 400; i <= 460;) {
    pwm.setPWM(3, 0, i); //wrist from fully extended pos 2 to reaching pos1
    i = i + 10;
    delay(50);
  }
  for (int i = 190; i <= 220;) {
    pwm.setPWM(1, 0, i); //shoulder from fully extended pos 2 to reaching pos 1
    i = i + 10;
    delay(50);
  }
  for (int j=400; j<=470;){
    pwm.setPWM(2, 0, j); //elbow from fully extended to horizontal(rest/pos1)
    j = j+10;
    delay(50);
  } 
}

void pos1_rest(){
 for (int i = 220; i <= 380;) {
    pwm.setPWM(1, 0, i); //shoulder from reaching (pos 1) to (obtuse angle) rest
    i = i + 10;
    delay(50);
  }
  for (int i = 130; i <= 290;) {
    pwm.setPWM(0, 0, i); //moving from perpendicular to car to over car
    i = i + 10;
    delay (50);
  }
  for (int i = 460; i >= 350;) {
    pwm.setPWM(3, 0, i); //wrist from position 1 to rest
    i = i - 10;
    delay(50);
  }  
}

void open_gripper(){
  for (int i = 318; i >= 250;) {
    pwm.setPWM(4, 0, i); //gripper from close to open
    i = i - 10;
    delay(100);
  }  
}

void close_gripper(){
  for (int i = 250; i <= 318;) {
    pwm.setPWM(4, 0, i); //gripper from open to close
    i = i + 10;
    delay(100);
  }    
}

//declaring pick up and drop off functions 
void pick_up(){
  rest_pos1();
  delay(500);
  pos1_pos2();
  delay(500);
  close_gripper();
  delay(500);
  pos2_pos1();
  delay(500);
  pos1_rest();
}

void drop_off(){
  rest_pos1();
  delay(500);
  pos1_pos2();
  delay(500);
  open_gripper();
  delay(500);
  pos2_pos1();
  delay(500);
  pos1_rest();
}

