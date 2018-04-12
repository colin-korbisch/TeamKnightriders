



//Specify the links and initial tuning parameters
double Sense1;
double Sense2;
float curSteer; 
double mspeed;
float motorPWM;

float setMotorSpeed(double spM){
  double outPWM;
  outPWM = map(spM,-100,100,6,9);
  return outPWM;
}
float setTurning(float perSteer) {
  double steerPWM;
  steerPWM = map(perSteer,-1,1,6.5,8.5);
  return steerPWM;
}
float controller(float curSteer,double sig1,double sig2) {
  //curSteer is PWM turning signal
  //sig1 is reflectivity sensor 1
  //sig 2 is reflectivity sensor 2
  double dev;
  if (sig1>2.5 and sig2>2.5)
  {  
    //if both sensors are white, start turning back towards neutral
    dev = curSteer - 10;
    curSteer = curSteer - dev*0.06;
  }
  else if (sig1<2.5 and sig2>2.5)
  {
    //if sig1 on black (left side), turn towards left
    // thus decrease turning PWM towards 6.5
    dev = curSteer - 6.5;
    curSteer = curSteer - 0.03*dev;
  }
  else if (sig1>2.5 and sig2<2.5)
  {
    //if sig2 on black (right side), turn towards right
    // increase PWM towards 8.5
    dev = 15 - curSteer;
    curSteer = curSteer + 0.03*dev;
  }
  return curSteer;
}
void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
  TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of 61.04 Hz
  Sense1 = analogRead(0);
  Sense1 = map(Sense1,0,1024,0,5);
  Sense2 = analogRead(1);
  Sense2 = map(Sense2,0,1024,0,5);
  analogWrite(5,7.5);
  curSteer = 7.5;
  mspeed = 15;

}

void loop()
{
  Serial.print(curSteer);
  Serial.print("");
  motorPWM = setMotorSpeed(mspeed);
  analogWrite(5,motorPWM);
  curSteer = controller(curSteer, Sense1, Sense2);
  analogWrite(6,curSteer*2.4);
  Sense1 = analogRead(0);
  Sense1 = map(Sense1,0,1024,0,5);
  Sense2 = analogRead(1);
  Sense2 = map(Sense2,0,1024,0,5);
}
