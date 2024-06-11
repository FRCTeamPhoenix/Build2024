import sys
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
from networktables import NetworkTables 

#Constants
model="crescendonote"
api="kifPJfNGf7lywgAW2kzA"
deviceIdList = ["19443010E15EA12E00", "19443010F157992E00", "1944301081ACA12E00"]
cameraAngleList = [0 , 120, 240]
cameraIdNumber = 0
device = deviceIdList[cameraIdNumber]
cameraAngle = cameraAngleList[cameraIdNumber]
cameraAnglePixleH = 95 / 640
cameraAnglePixleV = 65 / 640
server = "10.23.46.2"

if __name__ == '__main__':
    # instantiating an object (rf) with the RoboflowOak module
    checkCamera = RoboflowOak(model=model, confidence=0.05, overlap=0.5,
    version="1", api_key=api, rgb=True,
    depth=True, device=device, blocking=True, advanced_config={"wide_fov":True})
    
    # connects to networktabel 'SmartDashboard'
    # this might not work   V  if it doesn't add an ip address
    NetworkTables.initialize(server = server)
    cameraDataTable = NetworkTables.getTable('oakCamera')
    while True:
        # clear detected notes
        itemPredictionReturn = []
        # The rf.detect() function runs the model inference
        result, frame, raw_frame, depth = checkCamera.detect()
        predictions = result["predictions"]
        # loop through each object the camera detects
        for p in predictions:
             # append information of indavidual object to list
             objectAngleY = -cameraAnglePixleV * (p.json()["y"] - 320)
             # only use object if confedence is high enough
             if p.json()["confidence"] >= 0.72:
                 # create string to contain all info of given object
                 """
                 object x angle relative to robot
                 object y angle relative to camera 
                 object area
                 object distace
                 object type
                 """
                 # add item to object return list
                 itemPredictionReturn.append(f"{( ( -cameraAnglePixleH * ( p.json()['x'] - 320 ) ) + cameraAngle ) % 360}, {objectAngleY}, {p.json()['width'] * p.json()['height']}, {p.json()['depth']}, {p.json()['class']}")
        # print(itemPredictionReturn)
        # send object list to networktable
        cameraDataTable.putStringArray("cameraItems_0", itemPredictionReturn)
    # setting parameters for depth calculation


