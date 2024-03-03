from __future__ import print_function
import qwiic_led_stick
import qwiic_proximity
import time
import sys
from networktables import NetworkTables

NetworkTables.initialize(server='10.10.21.41')
proximityDataTable = NetworkTables.getTable('SmartDashboard')

def changeColor(proxValue, my_stick):
	if proxValue <=100:
		my_stick.set_all_LED_color(0, 0, 0)
		proximityDataTable.putString("FRC-Note", "Not Found")
	elif proxValue <=1000:
		my_stick.set_all_LED_color(255, 0, 0)
		proximityDataTable.putString("FRC-Note", "Found")
	else:
		my_stick.set_all_LED_color(0, 255, 0)
		proximityDataTable.putString("FRC-Note", "Captured")



def runExample():

	print("\nSparkFun Proximity Sensor VCN4040 Example 1\n")
	oProx = qwiic_proximity.QwiicProximity()
	my_stick = qwiic_led_stick.QwiicLEDStick()
	if oProx.is_connected() == False:
		print("The Qwiic Proximity device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return
		
	if oProx.begin() == False:
		print("\nThe Qwiic Proximity isn't connected to the sytsem. Please check your connection", \
            file=sys.stderr)
		return
	print("\nProximity ready!")
	
	if my_stick.begin() == False:
		print("\nThe Qwiic LED Stick isn't connected to the sytsem. Please check your connection", \
			file=sys.stderr)
		return
	print("\nLED Stick ready!")

	my_stick.set_all_LED_brightness(10)

	while True:
		proxValue = oProx.get_proximity()
		print("Proximity Value: %d" % proxValue)
		proximityDataTable.putNumber("proximityItems", proxvalue)
		changeColor(proxValue, my_stick)
		time.sleep(.4)
    
runExample()

