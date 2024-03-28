from __future__ import print_function
import time
import sys
from networktables import NetworkTables
import time
import random
import board
import adafruit_dotstar as dotstar
import logging

serverIP = '10.23.42.2'
NetworkTables.initialize(server=serverIP)
proximityDataTable = NetworkTables.getTable('SmartDashboard')
defaultColor = "yellow"
FIRE_COLORS = [(255, 10, 0), (255, 60, 0), (255, 100, 0), (255, 150, 0), (255, 200, 0), (255, 255, 0)]

#lights
# Using a DotStar Digital LED Strip with 60*4 (240 leds) LEDs connected to hardware SPI
no_dots = 119
dots = dotstar.DotStar(board.SCK, board.MOSI, no_dots, brightness=1)

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
"yellow" : (255,100,0),
"purple" : (153,0,153),
"orange" : (255,35,0),
"red_orange" : (255,15,0),
"orange_yellow" : (255, 55, 0),
"red_bright" : (255,51,51),
"dark_orange" : (255,140,0),
"blue" : (0,0,255),
"crimson" : (220,20,60),
"gold" : (255,215,0),
"green" : (0,255,0),
"off" : (0,0,0),
"white" : (255,255,255)
}

# HELPERS
# a random color 0 -> 192
def team_color():
    n_dots = len(dots)
    #for led in range(0, n_dots):
        #dots[led] = Colors["blue"];
    #dots[0]=Colors["red"];
    #dots[1]=Colors["off"];
    #dots[59]=Colors["orange"];
    #dots[4]=Colors["off"];
    #dots[5]=Colors["off"];
    while True:
        for led in range(6,n_dots):
            dots[led-5]=Colors["off"];
            dots[led-4]=Colors["off"];
            dots[led-3]=Colors["off"];
            dots[led-2]=Colors["orange"];
            dots[led-1]=Colors["off"];
            dots[led]=Colors["off"];
            #Bail out if the color changed.
        nt_color = ColorTable.getString("color","TeamColor")
        if (nt_color != "TeamColor"):
            return
#Flicker??
def flicker():
    return random.randint(200, 255)

#Flames
def fire_effect():
    for i in range(240):
        if i < 20:
            dots[i] = FIRE_COLORS[-1]
        else:
            dots[i] = FIRE_COLORS[random.randint(0, len(FIRE_COLORS)-2)]
    for i in range(20, 220):
        r = flicker()
        g = flicker() // 2
        dots[i] = (r, g, 0)
    dots.show()

def fireee():
    # Initialize fire strip
    fire_strip = [0] * 240

    # Randomly choose position and height of fire
    start_pos = random.randint(0, 200)
    fire_height = random.randint(10, 30)

    # Create fire effect
    for i in range(start_pos, start_pos + fire_height):
        if i < 240:
            fire_strip[i] = flicker()

    # Animate fire effect
    for _ in range(5):
        # Grow fire upward
        for i in range(start_pos - 1, max(start_pos - 5, 0), -1):
            if i >= 0:
                fire_strip[i] = flicker()
        # Shrink fire downward
        for i in range(start_pos + fire_height, min(start_pos + fire_height + 5, 240)):
            if i < 240:
                fire_strip[i] = 0
        # Display fire effect
        for i in range(240):
            if fire_strip[i] > 0:
                r = fire_strip[i]
                g = r // 2
                dots[i] = (r, g, 0)
            else:
                dots[i] = (0, 0, 0)
        dots.show()
        time.sleep(0.05)

def pattern(led):
    n_dots = len(dots)
    aled = led % n_dots
    dots[led % n_dots]=Colors["yellow"]
    dots[(led+1) % n_dots]=Colors["orange_yellow"]
    dots[(led+2) % n_dots]=Colors["orange"]
    dots[(led+3) % n_dots]=Colors["orange"]
    dots[(led+4) % n_dots]=Colors["orange"]
    dots[(led+5) % n_dots]=Colors["red_orange"]
    dots[(led+6) % n_dots]=Colors["red_orange"]
    dots[(led+7) % n_dots]=Colors["red"]
    dots[(led+8) % n_dots]=Colors["red"]
    dots[(led+9) % n_dots]=Colors["red"]
    dots[(led+10) % n_dots]=Colors["red"]
    dots[(led+11) % n_dots]=Colors["red"]
    dots[(led+12) % n_dots]=Colors["red"]

#New team color function
def teamColor():
    n_dots = len(dots)
    changeColorAll("yellow")
    while True:
        for led in range(13,n_dots):
            dots[led-13]=Colors["yellow"]
            dots[led-12]=Colors["orange_yellow"]
            dots[led-11]=Colors["orange"]
            dots[led-10]=Colors["orange"]
            dots[led-9]=Colors["orange"]
            dots[led-8]=Colors["red_orange"]
            dots[led-7]=Colors["red_orange"]
            dots[led-6]=Colors["red"]
            dots[led-5]=Colors["red"]
            dots[led-4]=Colors["red"]
            dots[led-3]=Colors["red"]
            dots[led-2]=Colors["red"]
            dots[led-1]=Colors["red"]
            dots[led]=Colors["red"]
        #for led in reversed(range(13, n_dots)):
            #dots[led-13]=Colors["red"]
            #dots[led-12]=Colors["red"]
            #dots[led-11]=Colors["red"]
            #dots[led-10]=Colors["red"]
            #dots[led-9]=Colors["red"]
            #dots[led-8]=Colors["red"]
            #dots[led-7]=Colors["red"]
            #dots[led-6]=Colors["red"]
            #dots[led-5]=Colors["red_orange"]
            #dots[led-4]=Colors["red_orange"]
            #dots[led-3]=Colors["red_orange"]
            #dots[led-2]=Colors["orange"]
            #dots[led-1]=Colors["orange_yellow"]
            #dots[led]=Colors["off"]
            #Bail out if the color changed.
        nt_color = ColorTable.getString("color","TeamColor")
        if (nt_color != "TeamColor"):
            return

#Basic code that loops the fire back and forth
def team_color():
    n_dots = len(dots)
    changeColorAll("yellow")
    while True:
        for led in range(13,n_dots):
            dots[led-13]=Colors["yellow"]
            dots[led-12]=Colors["orange_yellow"]
            dots[led-11]=Colors["orange"]
            dots[led-10]=Colors["orange"]
            dots[led-9]=Colors["orange"]
            dots[led-8]=Colors["red_orange"]
            dots[led-7]=Colors["red_orange"]
            dots[led-6]=Colors["red"]
            dots[led-5]=Colors["red"]
            dots[led-4]=Colors["red"]
            dots[led-3]=Colors["red"]
            dots[led-2]=Colors["red"]
            dots[led-1]=Colors["red"]
            dots[led]=Colors["red"]
            #Bail out if the color changed.
        nt_color = ColorTable.getString("color","TeamColor")
        if (nt_color != "TeamColor"):
            return

def coolFire():
    n_dots = len(dots)
    while True:
        changeColorAll("yellow")
        for led in range(0, n_dots):
            pattern(led)
            led = (led + 10) % n_dots
        nt_color = ColorTable.getString("color","fire")
        if (nt_color != "fire"):
            return

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

#def coopertition():
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
            NetworkTables.initialize(server=serverIP)
            proximityDataTable = NetworkTables.getTable('SmartDashboard')
            proximityDataTable.putString("Color", defaultColor)
        else:
            currentColor=proximityDataTable.getString("Color", defaultColor)
            print(f"Current Color: {currentColor}")
            uptime += 1
            try:
                if (currentColor == "fire"):
                    coolFire();
                elif (currentColor == "teamColor"):
                    team_color()
                elif (currentColor == "noNote"):
                    changeColorAll("red")
                elif (currentColor == "noteFound"):
                    changeColorAll("green")
                else:
                    changeColorAll(currentColor)
            except:
                changeColorAll(defaultColor)
            time.sleep(0.1)

runExample()
