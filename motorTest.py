# from BreadthSearch import *
# from PiSkyPixy import *
import RPi.GPIO as GPIO
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import time
# import cv2

# Initialize the pins
steerPin = 32
drivePin = 33
GPIO.setmode(GPIO.BOARD)
GPIO.setup(drivePin,GPIO.OUT)
GPIO.setup(steerPin,GPIO.OUT)
dp = GPIO.PWM(drivePin,50)
sp = GPIO.PWM(steerPin,50)

dp.start(7.5)
sp.start(11)
print "starting neutral"
time.sleep(5)
print "going straight"
dp.ChangeDutyCycle(7.8)
time.sleep(2)
print "going reverse"
dp.ChangeDutyCycle(7)
time.sleep(0.05)
dp.ChangeDutyCycle(7.5)
time.sleep(0.05)
dp.ChangeDutyCycle(7.14)
time.sleep(2)
quit()


# while True:
# 	mdc = mdc+0.01
# 	dp.ChangeDutyCycle(mdc)
# 	print mdc
# 	time.sleep(0.5)