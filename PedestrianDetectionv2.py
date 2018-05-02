import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

TrigLeft = 11 
EchoLeft = 13
TrigRight = 15
EchoRight = 16
#ThresholdValue = 

GPIO.setup(TrigLeft,GPIO.OUT)
GPIO.setup(TrigRight,GPIO.OUT)
GPIO.setup(EchoLeft,GPIO.IN)
GPIO.setup(EchoRight,GPIO.IN)

i = 0

GPIO.output(TrigLeft, False) #wait for left sensor to settle
time.sleep(2)
GPIO.output(TrigRight, False) #wait for right sensor to settle
time.sleep(2)

while (1):
	GPIO.output(TrigLeft, True)  #give 10uS pulse to trigger the module 
	time.sleep(0.00001)
	GPIO.output(TrigLeft, False) #go back to low
		
	while GPIO.input(EchoLeft)==0:
		Left_Pulse_Start = time.time()

	while GPIO.input(EchoLeft)==1:
		Left_Pulse_End = time.time()

	Left_Pulse_Duration = Left_Pulse_End - Left_Pulse_Start
	LeftDistance = Left_Pulse_Duration * 17150

	GPIO.output(TrigRight, True)  #give 10uS pulse to trigger the module 
	time.sleep(0.00001)
	GPIO.output(TrigRight, False) #go back to low

	while GPIO.input(EchoRight)==0:
		Right_Pulse_Start = time.time()

	while GPIO.input(EchoRight)==1:
		Right_Pulse_End = time.time()

	Right_Pulse_Duration = Right_Pulse_End - Right_Pulse_Start
	RightDistance = Right_Pulse_Duration * 17150

	LeftDistance = round(LeftDistance,2)
	RightDistance = round(RightDistance,2)

	print "left:", LeftDistance
	print "right:", RightDistance

	i = i+1
	time.sleep(0.1)

GPIO.cleanup()

#if(LeftDistance <= ThresholdValue):

	#while(RightDistance >= ThresholdValue):
	
	 # UpdateMotor(7.5)
	  
	#if(RightDistance <= ThresholdValue) 
	
	 # time.sleep(3)
	 # UpdateMotor(OldDC)
	 

#else if(RightDistance <= ThresholdValue):

	#while(LeftDistance >= ThresholdValue):
	
	 # UpdateMotor(7.5)
	  
	#if(LeftDistance <= ThresholdValue):
	
	 # time.sleep(3)
	 #UpdateMotor(OldDC)
