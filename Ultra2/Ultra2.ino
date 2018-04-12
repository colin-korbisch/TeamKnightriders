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



float old5 = 100, old4 = 100, old3 = 100, old2 = 100, old1 = 100;
float output;

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
  attachInterrupt(0,PinChangeISR0,CHANGE);
  attachInterrupt(1,PinChangeISR1,CHANGE);
}

void loop() {
  static float oldLastRead = lastRead;
  if (oldLastRead != lastRead) {
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
  if (dutyCycle-old1 > 100) {
    dutyCycle = 0;
  }
  old5 = old4;
  old4 = old3;
  old3 = old2;
  old2 = old1;
  old1 = dutyCycle;
  output = (dutyCycle+old1+old2+old3+old4+old5)/6;
  Serial.println(dutyCycle);
  delay(100);
}
