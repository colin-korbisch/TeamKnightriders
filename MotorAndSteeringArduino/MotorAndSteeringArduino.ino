#include <Adafruit_PWMServoDriver.h>

// Interrupt variables
volatile float fall_Time = 0;                   // Placeholder for microsecond time when last falling edge occured.
volatile float rise_Time = 0;                   // Placeholder for microsecond time when last rising edge occured.
volatile float dutyCycle = 0;                            // Duty Cycle %
volatile float lastRead = 0; 

volatile float fall_Time2 = 0;                   // Placeholder for microsecond time when last falling edge occured.
volatile float rise_Time2 = 0;                   // Placeholder for microsecond time when last rising edge occured.
volatile float dutyCycle2 = 0;                            // Duty Cycle %
volatile float lastRead2 = 0;                    // Last interrupt time (needed to determine interrupt lockup due to 0% and 100% duty cycle)

// Controller Variables
double Setpoint, Input, Output;
double integral;
float old_error;

// Pwm Variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
# define STEER_RIGHT = 500                        // Pulse Length needed for steering right
# define STEER_LEFT = 380                         // PL for steering left

// Other Variables
float oldDuty;
float pwmOut;
int periodOut = 20;



void PinChangeISR0(){                                   // Pin 2 (Interrupt 0) service routine
  lastRead = micros();                                  // Get current time
  if (digitalRead(2) == LOW) {
    // Falling edge
    fall_Time = lastRead;                               // Just store falling edge and calculate on rising edge
  }
  else {
    // Rising edge
    float total_Time = rise_Time - lastRead;    // Get total cycle time
    float on_Time = fall_Time - rise_Time;      // Get on time during this cycle
    total_Time = total_Time / on_Time;                  // Divide it down
    dutyCycle = -1* 100 / total_Time;                       // Convert to a percentage
    rise_Time = lastRead;                               // Store rise time
  }
}

void PinChangeISR1(){                                   // Pin 2 (Interrupt 0) service routine
  lastRead2 = micros();                                  // Get current time
  if (digitalRead(2) == LOW) {
    // Falling edge
    fall_Time2 = lastRead2;                               // Just store falling edge and calculate on rising edge
  }
  else {
    // Rising edge
    float total_Time2 = rise_Time2 - lastRead2;    // Get total cycle time
    float on_Time2 = fall_Time2 - rise_Time2;      // Get on time during this cycle
    total_Time2 = total_Time2 / on_Time2;                  // Divide it down
    dutyCycle2 = -1*100 / total_Time2;                       // Convert to a percentage
    rise_Time2 = lastRead2;                               // Store rise time
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  attachInterrupt(0,PinChangeISR0,CHANGE);
  attachInterrupt(1,PinChangeISR1,CHANGE);
  
  pwm.begin();
  pwm.setPWMFreq(50);
  
  Setpoint = 15 * 2;
  oldDuty = 100;
}

void loop() {
  static float oldLastRead = lastRead;
  static float oldRead2 = lastRead2;
  if (oldLastRead != lastRead) {
    //Serial.println(dutyCycle);
    oldLastRead = lastRead;
  }
  else { // No interrupt since last read so must be 0% or 100%
    if (digitalRead(2) == LOW){
      dutyCycle = 0;
    }
    else {
      dutyCycle = 100;
    }
  }
  if (oldRead2 != lastRead2) {
    oldRead2 = lastRead2;
  }
  else {
    if (digitalRead(3) == LOW){
      dutyCycle2 = 0;
    }
    else {
      dutyCycle2 = 100;
    }
  }
  if (dutyCycle < 0){
    dutyCycle = 0;
  }
  if (dutyCycle2 < 0) {
    dutyCycle2 = 0;
  }
  if (abs(dutyCycle-oldDuty) > 100) {
    dutyCycle = 0;
  }
  
  Input = (dutyCycle + oldDuty)/2;        // Using just the passenger side sensor
  float Output = PIDCont(Input,Setpoint);
  Output = constrain(Output,-20,20);
  pulseLength = map(Output,-20,20,STEER_RIGHT,STEER_LEFT); // Need to set right PWMS
  pwm.setPWM(steerServo,0,pulseLength)
  
  
}

float PIDCont(float Input,double Setp) {
  double kp = 0.75, kd = 0.001, ki = 0.0005;
  double error, de, u;
  error = Setp - Input;
  de = error-old_error;
  integral = integral + error;
  u = kp*error + kd*de + ki*integral;
  return u;
}


