import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

#Pixie pin 36 to 12
GPIO.setup(36, GPIO.OUT) 

#Arm pin 37 to 11
GPIO.setup(37, GPIO.OUT)

#Turn pixie
GPIO.output(36, 1)
time.sleep(.1)
GPIO.output(36,0)
time.sleep(1)

#pick up passenger
GPIO.output(37,1)
time.sleep(.1)
GPIO.output(37,0)
time.sleep(3)

#drop off passenger
GPIO.output(37,1)
time.sleep(.1)
GPIO.output(37,0)
time.sleep(3)


