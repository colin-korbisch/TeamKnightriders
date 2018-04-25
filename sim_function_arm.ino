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
  
  //set pixie servo facing forward
   pwm.setPWM(5, 0, 250); 

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
  for (int i = 290; i <= 460;) {
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
    for (i = 0; i <= 100; i++){  
      int p1servo1 = 200;
      int p1servo2 = 460;
      int p1servo3 = 480;
      int p2servo1 = 170;
      int p2servo2 = 330;
      int p2servo3 = 350; 
       
      float conv1 = (p2servo1 - p1servo1)/100.;
      float conv2 = (p2servo2 - p1servo2)/100.; 
      float conv3 = (p2servo3 - p1servo3)/100.;   
      float servo1 = i*conv1 + p1servo1;
      float servo2 = i*conv2 + p1servo2; 
      float servo3 = i*conv3 + p1servo3;

      pwm.setPWM(1, 0 , servo1);
      pwm.setPWM(2, 0 , servo2);
      pwm.setPWM(3, 0 , servo3); 
      delay(75); 
      Serial.print(conv1);       
    }
}

void pos2_pos1(){
    for (j = 0; j<= 100; j++){  
      int p1servo1 = 200;
      int p1servo2 = 460;
      int p1servo3 = 480;
      int p2servo1 = 170;
      int p2servo2 = 330;
      int p2servo3 = 350;  
                 
      float conv1 = (p1servo1-p2servo1)/100.;
      float conv2 = (p1servo2-p2servo2)/100.; 
      float conv3 = (p1servo3-p2servo3)/100.;
      
      float servo1 = j*conv1 + 170; 
      float servo2 = j*conv2 + 330; 
      float servo3 = j*conv3 + 380;

      pwm.setPWM(1, 0 , servo1);
      pwm.setPWM(2, 0 , servo2);
      pwm.setPWM(3, 0 , servo3); 
      delay(75);        
    }
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
  for (int i = 460; i >= 290;) {
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

void pixie_fwd(){
   pwm.setPWM(5, 0, 250);   
}

void pixie_passenger(){
  
   pwm.setPWM(5, 0, 500); 
}

//declaring pick up and drop off functions 
void pick_up(){
  pixie_passenger();
  delay(500);
  rest_pos1();
  delay(500);
  pos1_pos2();
  delay(500);
  close_gripper();
  delay(2000);
  pos2_pos1();
  delay(500);
  pos1_rest();
  pixie_fwd();
}

void drop_off(){
  pixie_passenger();
  delay(500);
  rest_pos1();
  delay(500);
  pos1_pos2();
  delay(500);
  open_gripper();
  delay(2000);
  pos2_pos1();
  delay(500);
  pos1_rest();
  pixie_fwd();
}


