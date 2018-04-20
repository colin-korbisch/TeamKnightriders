#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

int i, j, k;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup() 
{ 
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
  pwm.setPWM(3, 0, 300); //Wrist in rest position
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

void rest_pos1(){
  for (int i = 290; i >= 130;) {
    pwm.setPWM(0, 0, i); //moving from over car to perpendicular to car
    i = i - 10;
    delay (50);
  }
  for (int i = 300; i <= 460;) {
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
    for (i = 0; i= 100; i++){      
      int servo1 = i*((190-220)/100) + 220;
      int servo2 = i*((350-470)/100) + 470; 
      int servo3 = i*((400-460)/100) + 460;

      pwm.setPWM(1, 0 , servo1);
      pwm.setPWM(1, 0 , servo2);
      pwm.setPWM(1, 0 , servo3);         
    }
    
//    while(i >= 190 && j >=350 && k >=400)//pos1 to pos2
//  {
//    i = 220; //servo 1 in pos1
//    j = 470; //servo 2 in pos1
//    k = 460; //servo 3 in pos1
//    
//    i = i - ((220-190)/100);
//    j = j - ((470-350)/100);
//    k = k - ((460-400)/100);
//    
//    pwm.setPWM(1, 0 , i);
//    pwm.setPWM(2, 0 , j);
//    pwm.setPWM(3, 0 , k);    
//  }
}

void pos2_pos1(){
    for (i = 0; i= 100; i++){      
      int servo1 = i*((220-190)/100) + 190; 
      int servo2 = i*((470-350)/100) + 350; 
      int servo3 = i*((460-400)/100) + 400;

      pwm.setPWM(1, 0 , servo1);
      pwm.setPWM(1, 0 , servo2);
      pwm.setPWM(1, 0 , servo3);         
    }
  
//    while(i <= 220 && j <=470 && k <=460)//pos2 to pos1
//  {
//    i = 190; //servo 1 in pos2
//    j = 350; //servo 2 in pos2
//    k = 400; //servo 3 in pos2
//    
//    i = i + ((220-190)/100);
//    j = j + ((470-350)/100);
//    k = k + ((460-400)/100);
//    
//    pwm.setPWM(1, 0 , i);
//    pwm.setPWM(2, 0 , j);
//    pwm.setPWM(3, 0 , k);    
//  }  
}

void pos1_rest(){
 for (int i = 220; i <= 340;) {
    pwm.setPWM(1, 0, i); //shoulder from reaching (pos 1) to (obtuse angle) rest
    i = i + 10;
    delay(50);
  }
  for (int i = 460; i >= 390;) {
    pwm.setPWM(3, 0, i); //wrist from position 1 to rest
    i = i - 10;
    delay(50);
  } 
  for (int i = 340; i <= 380;) {
    pwm.setPWM(1, 0, i); //shoulder from reaching (pos 1) to (obtuse angle) rest
    i = i + 10;
    delay(50);
  }
  for (int i = 460; i >= 300;) {
    pwm.setPWM(3, 0, i); //wrist from position 1 to rest
    i = i - 10;
    delay(50);
  } 
  for (int i = 130; i <= 290;) {
    pwm.setPWM(0, 0, i); //moving from perpendicular to car to over car
    i = i + 10;
    delay (50);
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


