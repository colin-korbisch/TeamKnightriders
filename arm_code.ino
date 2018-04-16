#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Circuit_Playground.h>

#include <Adafruit_PWMServoDriver.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  400 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

uint8_t servonum = 0;
uint16_t pulselen;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop()
{
delay(1000);
//rest position
  for(int i =0; i<=290; )
  {
    pwm.setPWM(1, 0, i); //acute angle forward (mid position)
    i = i +10;
    delay(50);
  }
  
delay(200);

  for (int j =0; j<=550;)
  {
    pwm.setPWM(2, 0, j); //holds elbow parallel to ground in rest position
    j = j+10;
    delay(50);
  }
  
delay(200);

  for (int k = 0; k<=380; )
  {
  pwm.setPWM(1, 0, k); //obtuse angle back(rest position)
  k = k+10;
  delay(50);
  }
  
delay(200);

  for (int l = 0; l<=290; )
  { pwm.setPWM(0, 0, l); //positioned over car
     l = l+10;
     delay(50);
  }
  
delay(200);

  for( int m = 0; m<=350; )
  {
  pwm.setPWM(3, 0, m); //Wrist in rest position
  m = m+10;
  delay(50);
  }

delay(2000);


//pwm.setPWM(3, 0, 400); //Wrist mid
//delay(200);

for( int n =0; n<=530;)
  {
  pwm.setPWM(3, 0, n); //Wrist reaching horizontal to ground
  n = n+10;
  delay(50);
  } 
delay(200);

//into position 1
  for(int o = 0; o<=130; )
  {
  pwm.setPWM(0, 0, o); //perpendicular to car
  o = o+10;
  delay(50);
  }

delay(200);
//pwm.setPWM(2, 0, 510); //not used currently 
//delay(200);

  for(int p = 0; p<=200; )
  { 
  pwm.setPWM(1, 0, p); //acute angle forward (reach position
  p = p+10;
  delay(50);
  }

delay(1000);
pwm.setPWM(4, 0, 240); //gripper open
delay(1000);
pwm.setPWM(4, 0, 280); //gripper mid 
delay(1000);
pwm.setPWM(4, 0, 318); //gripper close
delay(1000);


}



//void rest(){
//
//int rest[] = {120, 380};
//
//for(int i = 0; i > 1; i++){
//  
//pulselen = rest[i];
//pwm.setPWM(i, 0, pulselen);
//delay(10);
//}
//}
//
//void pos1(){
//
//int pos1[] = {290, 120};
//
//for(int j = 0; j > 1; j++){
//pulselen = pos1[j];
//pwm.setPWM(j, 0, pulselen);
//delay(10);
//}
//}
//
//void pos2 (){
//
//}
//
//void passpick(){
//
//}
//
//void passdrop(){
//
//}
//}

