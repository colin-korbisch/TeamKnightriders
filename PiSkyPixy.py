#!/usr/bin/env python2

import RFM69
from RFM69registers import *
import datetime
import time

def radioSetup():
	NODE=2
	NET=10
	KEY="TOPSECRETPASSWD"
	TIMEOUT=5
	TOSLEEP=0.1

	radio = RFM69.RFM69(RF69_915MHZ, NODE, NET, True)
	# print "class initialized"

	# print "reading all registers"
	results = radio.readAllRegs()
	#for result in results:
	#	print result

	# print "Performing rcCalibration"
	radio.rcCalibration()

	# print "setting high power"
	radio.setHighPower(True)

	return radio

def getRadioSignal(radio):

	# print "start recv"
	radio.receiveBegin()
	timedOut=0
	while not radio.receiveDone():
		timedOut+=TOSLEEP
		time.sleep(TOSLEEP)
		if timedOut > TIMEOUT:
			return -1, -1, -1
			break
	

	# print "end recv..."
	# print " *** %s from %s RSSI:%s" % ("".join([chr(letter) for letter in radio.DATA]), radio.SENDERID, radio.RSSI)
	posData = "".join([chr(letter) for letter in radio.DATA])
	# print posData
	robData, psngrData, destData = posData.split("|")
	# print robData
	# print psngrData
	# print destData

	return robData, psngrData, destData

if __name__ == "__main__":
	while True:
		radio = radioSetup()
		data1, data2, data3 = getRadioSignal(radio)
		print data1
