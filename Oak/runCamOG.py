import sys
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
from networktables import NetworkTables 

#Constants
model=""
api=""
device_name="OAK-D-PRO-W-"
cameraNumber = 1


if __name__ == '__main__':
    # instantiating an object (rf) with the RoboflowOak module
    rf_1 = RoboflowOak(model="waoidjawiudjoiwaj", confidence=0.05, overlap=0.5,
    version="1", api_key="kifPJfNGf7lywgAW2kzA", rgb=True,
    depth=True, device="19443010E15EA12E00", blocking=True, device_name="OAK-D-PRO-W-1", advanced_config={"wide_fov":True})
    
    rf_2 = RoboflowOak(model="waoidjawiudjoiwaj", confidence=0.05, overlap=0.5,
    version="1", api_key="kifPJfNGf7lywgAW2kzA", rgb=True,
    depth=True, blocking=True, device_name="OAK-D-PRO-W-2")

    rf_3 = RoboflowOak(model="waoidjawiudjoiwaj", confidence=0.05, overlap=0.5,
    version="1", api_key="kifPJfNGf7lywgAW2kzA", rgb=True,
    depth=True, blocking=True, device_name="OAK-D-PRO-W-3")
    # list of cameras to check
    

    cameraList = [rf_1]
    # initalizes networktable from ip of geven server
    # connects to networktabel 'SmartDashboard'
    NetworkTables.initialize(server='10.10.21.9')
    cameraDataTable = NetworkTables.getTable('oakCamera')
    # clears old data from networktables
    itemPredictionReturn = []
    cameraDataTable.putStringArray("cameraItems", itemPredictionReturn)
    # calculates camera's angle/pixle vale
    cameraAnglePixleH = 95 / 640
    cameraAnglePixleV = 65 / 640
    while True:
        # clear detected notes
        itemPredictionReturn = []
        for cameraNumber in range(len(cameraList)):
            t0 = time.time()
            # The rf.detect() function runs the model inference
            result, frame, raw_frame, depth = cameraList[cameraNumber].detect()
            predictions = result["predictions"]
            t = time.time()-t0
            print("INFERENCE TIME IN MS ", 1/t)
            # loop through each object the camera detects
            for p in predictions:
                 # start a list to contain all data from each object
                 itemPredictionList = []
                 # append information of indavidual object to list
                 for item in p.json():
                     itemPredictionList.append(p.json()[item])
                 objectAngleX = -cameraAnglePixleH * (itemPredictionList[0] - 320)
                 objectAngleY = -cameraAnglePixleV * (itemPredictionList[1] - 320)
                 # converts from camera angle to robot angle
                 match cameraNumber:
                     case 0:
                         robotAngleX = (objectAngleX + 0) % 360
                     case 1:
                         robotAngleX = (objectAngleX + 120) % 360
                     case 2:
                         robotAngleX = (objectAngleX + 260) % 360
                 # only use object if confedence is high enough
                 if itemPredictionList[5] >= 0.72:
                     # create string to contain all info of given object
                     """
                     object x angle relative to robot
                     object y angle relative to camera 
                     object area
                     object distace
                     object type
                     """
                     itemPredictionString = f"{robotAngleX}, {objectAngleY}, {itemPredictionList[2] * itemPredictionList[3]}, {itemPredictionList[4]}, {itemPredictionList[6]}"
                     # add item to object return list
                     itemPredictionReturn.append(itemPredictionString)
        print(itemPredictionReturn)
        # send object list to networktable
        cameraDataTable.putStringArray("cameraItems", itemPredictionReturn)
        # setting parameters for depth calculation
        # max_depth = np.amax(depth)
        #cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
        #cv2.imshow("frame", frame)

        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
        #if cv2.waitKey(1) == ord('q'):
        #    break



