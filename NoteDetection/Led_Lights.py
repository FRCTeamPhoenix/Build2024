from __future__ import print_function
import time
import sys
from networktables import NetworkTables
import time
import random
import board
import adafruit_dotstar as dotstar
import logging

NetworkTables.initialize(server='10.23.42.2')
proximityDataTable = NetworkTables.getTable('SmartDashboard')

#lights
# Using a DotStar Digital LED Strip with 60*4 (240 leds) LEDs connected to hardware SPI
dots = dotstar.DotStar(board.SCK, board.MOSI, 240, brightness=1)

# Start Loggign for pubsub
logging.basicConfig(level=logging.DEBUG)

# Setup Network Tables
#NetworkTables.initialize(server="192.168.86.34")
#ColorTable = NetworkTables.getTable("SmartDashboard")
#ColorTable.putString("color","team")
#count = 0
#index = 0
uptime = 0

#Colors
Colors = {
"red" : (255,0,0),
"yellow" : (255,255,0),
"purple" : (153,0,153),
"orange" : (255,128,0),
"red_orange" : (255,69,0),
"red_bright" : (255,51,51),
"dark_orange" : (255,140,0),
"blue" : (51,51,255),
"crimson" : (220,20,60),
"gold" : (255,215,0),
"green" : (0,255,0),
"off" : (0,0,0),
"white" : (255,255,255)
}

# HELPERS
# a random color 0 -> 192
#def team_color():
#  n_dots = len(dots)
#  dots[0]=Colors["red"];
#  dots[1]=Colors["off"];
#  dots[59]=Colors["orange"];
#  dots[4]=Colors["off"];
#  dots[5]=Colors["off"];
#  for led in range(6,len(dots)):
#    dots[led-5]=Colors["red"];
#    dots[led-4]=Colors["off"];
#    dots[led-3]=Colors["off"];
#    dots[led-2]=Colors["orange"];
#    dots[led-1]=Colors["off"];
#    dots[led]=Colors["off"];
#    #Bail out if the color changed.
    #nt_color = ColorTable.getString("color","team")
#    if (index != 0):
#      return
#  dots.fill(Colors["off"])

#def amp():
#  n_dots = len(dots)
#  dots[0]=Colors["blue"];
#  dots[1]=Colors["off"];
#  dots[59]=Colors["blue"];
#  dots[4]=Colors["off"];
#  dots[5]=Colors["off"];
#  for led in range(6,len(dots)):
#    dots[led-5]=Colors["blue"];
#    dots[led-4]=Colors["off"];
#    dots[led-3]=Colors["off"];
#    dots[led-2]=Colors["blue"];
#    dots[led-1]=Colors["off"];
#    dots[led]=Colors["off"];
#    #Bail out if the color changed.
#    #nt_color = ColorTable.getString("color","team")
#    if (index != 1):
#      return
#  dots.fill(Colors["off"])

#def cooperation():
#  n_dots = len(dots)
#  dots[0]=Colors["yellow"];
#  dots[1]=Colors["off"];
#  dots[59]=Colors["yellow"];
#  dots[4]=Colors["off"];
#  dots[5]=Colors["off"];
#  for led in range(6,len(dots)):
#    dots[led-5]=Colors["yellow"];
#    dots[led-4]=Colors["off"];
#    dots[led-3]=Colors["off"];
#    dots[led-2]=Colors["yellow"];
#    dots[led-1]=Colors["off"];
#    dots[led]=Colors["off"];
    #Bail out if the color changed.
    #nt_color = ColorTable.getString("color","team")
#    if (index != 2):
#      return
#  dots.fill(Colors["off"])
#
#lights = [team_color(), amp(), cooperation()]

def changeColorAll(colorToSet):
    dots.fill(Colors[colorToSet])

def runExample():
    uptime = 0

    while True:
        if ( not NetworkTables.isConnected()):
            #NetworkTables.initialize(server='10.23.42.2')
            NetworkTables.initialize(server='10.145.41.60')
            proximityDataTable = NetworkTables.getTable('SmartDashboard')
            proximityDataTable.putString("Color","off")
        else:
            currentColor=proximityDataTable.getString("Color", "off")
            print(f"Current Color: {currentColor}")
            uptime += 1
            changeColorAll(currentColor)
            time.sleep(0.1)

runExample()

