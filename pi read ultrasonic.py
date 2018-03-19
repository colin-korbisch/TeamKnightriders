import RPi.GPIO as GPIO
import time
import numpy as np



def ConvertDistance(dist):
	max = 50
	frac = dist/50
	return frac * 100

GPIO.setmode(GPIO.BCM)
TRIG1 = 24
TRIG2 = 25
ECHO1 = 26
ECHO2 = 27

GPIO.setup(TRIG1,GPIO.OUT)
GPIO.setup(TRIG2,GPIO.OUT)
GPIO.setup(ECHO1,GPIO.IN)
GPIO.setup(ECHO2,GPIO.IN)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(17,GPIO.OUT)



ultra1 = GPIO.PWM(16, 1000)
ultra2 = GPIO.PWM(17, 1000)
		




GPIO.output(TRIG1, False)
GPIO.output(TRIG2, False)
time.sleep(2)

while True:
	GPIO.output(TRIG1, True)
	GPIO.output(TRIG2, True)
	time.sleep(0.00001)
	GPIO.output(TRIG1, False)
	GPIO.output(TRIG2, False)
	echo_start = time.time()
	time_out = False
	
	start1 = 0
	start2 = 0
	end1 = 0
	end2 = 0
	polling = True

	while polling:
		curpoll = time.time() - echo_start
		if curpoll <= 0.1:
			if GPIO.input(ECHO1) == 0 and start1 == 0:
				start1 = time.time()
			if GPIO.input(ECHO2) == 0 and start2 == 0:
				start2 = time.time()
			if GPIO.input(ECHO1) == 1:
				end1 = time.time()
			if GPIO.input(ECHO2) == 1:
				end2 = time.time()
			if end1 !=0 and end2 != 0:
				polling = False
				goodsig = True
		else:
			polling = False
			goodsig = False

	if goodsig:
		dur1 = end1 - start1
		dur2 = end2 - start2

		distance1 = dur1 * 17150
		distance2 = dur2 * 17150
		distance1 = round(distance1,2)
		distance2 = round(distance2,2)

		readU1 = ConvertDistance(distance1)
		readU2 = ConvertDistance(distance2)

		ultra1.ChangeDutyCycle(readU1)
		ultra2.ChangeDutyCycle(readU2)

	