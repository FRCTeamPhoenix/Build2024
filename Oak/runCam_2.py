from depthai_sdk import OakCamera, RecordType
from depthai_sdk.classes import DetectionPacket
from depthai_sdk.classes import SpatialBbMappingPacket
from depthai_sdk import frameNorm
import depthai as dai
import numpy as np
import time
from networktables import NetworkTables 

#Constants
DEBUG=False
model='voyager-te2at/1'
api='rf_egvTfEpIlQs31Ul09iCG'
deviceIdList = ["19443010E15EA12E00", "19443010F157992E00", "1944301081ACA12E00"]
cameraAngleList = [0 , 155, 245]
cameraIdNumber = 1
device = deviceIdList[cameraIdNumber]
cameraAngle = cameraAngleList[cameraIdNumber]
Resolution_Horizontal=1280
Resolution_Vertical = 720
cameraAnglePixleH = 166/Resolution_Horizontal  #95 / 640
cameraAnglePixleV = 65/Resolution_Vertical  #65 / 640
server = "10.23.42.2"
cameraDataTable="oakCamera"
confidence_default = 0.72
confidence_percent = confidence_default

def detectionPacketCallback(packet):
    itemPredictionReturn = []

    for detection in packet.detections:
        # only use object if confedence is high enough
        global confidence_percent
        global cameraDataTable

        if NetworkTables.isConnected():
            confidence_percent = cameraDataTable.getNumber("Confidence Percent",confidence_default)
        if detection.confidence >= confidence_percent:
            objectAngleY = -cameraAnglePixleV * ((((detection.bbox.ymax + detection.bbox.ymin) / 2) * Resolution_Vertical) - (Resolution_Vertical/2))
            objectAngleX = -cameraAnglePixleH * ((((detection.bbox.xmax + detection.bbox.xmin) / 2) * Resolution_Horizontal) - (Resolution_Horizontal/2))

            z = detection.img_detection.spatialCoordinates.z

            area = (detection.bbox.xmax - detection.bbox.xmin) * (detection.bbox.ymax - detection.bbox.ymin)

            # create string to contain all info of given object
            """
            object x angle relative to robot
            object y angle relative to camera 
            object area
            object distace
            object type
            """
            # add item to object return list
            itemPredictionReturn.append(f"{( objectAngleX + cameraAngle ) % 360}, {objectAngleY}, {area}, {z}, {detection.confidence}, {detection.label_str}")
            # send object list to networktable
    
    cameraDataTable.putStringArray(f"cameraItems_{cameraIdNumber}", itemPredictionReturn)

    if DEBUG:
        print(f"{itemPredictionReturn}")


while True:
    try:
        NetworkTables.initialize(server = server)
        cameraDataTable = NetworkTables.getTable(cameraDataTable)
        cameraDataTable.putNumber("Confidence Percent",confidence_percent)
        break #must be connected now
    except:
        #Seems we arent connected yet
        time.sleep(1)

with OakCamera(device) as oak:
    color = oak.camera("color")
    model_config = {
        'source': 'roboflow',
        'model':model,
        'key': api
    }

    #if DEBUG:
    #    oak.record(color.out.encoded,'./',RecordType.VIDEO)
    nn = oak.create_nn(model_config, color, spatial=True)
    oak.callback(nn, callback=detectionPacketCallback)
    #if DEBUG:
    #    oak.visualize([nn.out.main],fps=True)
    oak.start(blocking=True)
