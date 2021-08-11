#!/usr/bin/env python 
#from carla_birdeye_view import BirdViewProducer, BirdViewCropType, PixelDimensions
import numpy as np
from matplotlib import pyplot as plt
import math
import xml.etree.ElementTree as ET
import rospy
import random
import utm
from matplotlib.patches import Arc
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from carla_msgs.msg import CarlaWorldInfo
import tf
import geometry_msgs
from std_msgs.msg import Float32



gnssMsg = NavSatFix()
imuMsg = Imu()
WIMsg = CarlaWorldInfo()
currCurvMsg = Float32()
currCurvHeadMsg = Float32()
hdMapCounter = 0
    
class roadStruct:
    roadId = ''
    roadLink = ''
    roadType = ''
    roadPlanview = ''
    roadElevationProfile = ''
    roadLateralProfile = ''
    roadLanes = ''
    roadObjects = ''
    roadSignals = ''
    planViewGeometry = ''

class junctionStruct:
    junctionID = ''
    junctionData = ''
    
class gridStruct:
    gridNo = 0
    xMin = 0
    xMax = 0
    yMin = 0
    yMax = 0
    # gridRoadIDList
    # gridGeoIDList

def gnssCallback(msg):
    global gnssMsg
    gnssMsg = msg

def imuCallback(msg):
    global imuMsg
    imuMsg = msg

def WorldInfoCallback(msg):
    global WIMsg
    global hdMapCounter
    if hdMapCounter == 0:
        WIMsg = msg
        hdMapCounter = hdMapCounter + 1


def plot(x,y):
    plt.plot(x,y, 'o')


def LatLongtoXY(longtiude,latitude,altitude):
    #hayford ----> (WGS84)
    N =  6387435.3985
    h = gnssMsg.altitude #m
    latitude = math.radians(gnssMsg.latitude) #as degree
    #print(latitude)
    longitude = math.radians(gnssMsg.longitude)
    #print(longtiude)
    X = (N+h)*math.sin(latitude)*math.cos(longitude)
    Y = (N+h)*math.cos(latitude)*math.sin(longitude)
    #z value
    return(X,Y)

def findSuccessor(road):
    sucID= -1
    sucType = -1
    contType = ""
    Suc = None
    for suc in road.roadLink.iter("successor"):
        sucID = int(suc.get("elementId"))
        sucType = suc.get("elementType")
        contType = suc.get("contactPoint")
    if sucType == "junction":  
        #print(ET.tostring(junctionList[int(sucID)].junctionData, encoding='utf8', method='xml'))
        for conn in junctionList[int(sucID)].junctionData.iter("connection"):
            if int(conn.attrib["incomingRoad"]) == road.roadId:
                Suc = int(conn.attrib["connectingRoad"])
                contType = conn.attrib["contactPoint"]
    elif sucType == "road":
        Suc = sucID
    else:
        print("unsupported successor type")
    return(Suc, contType)

def findPredecessor(road):
    preID= -1
    preType = -1
    contType = ""
    Pre = None
    for pre in road.roadLink.iter("predecessor"):
        preID = int(pre.get("elementId"))
        preType = pre.get("elementType")
        contType = pre.get("contactPoint")
    if preType == "junction":   
        #print(ET.tostring(junctionList[int(sucID)].junctionData, encoding='utf8', method='xml'))
        for conn in junctionList[int(preID)].junctionData.iter("connection"):
            if int(conn.attrib["incomingRoad"]) == road.roadId:
                Pre = int(conn.attrib["connectingRoad"])
                contType = conn.attrib["contactPoint"]
            else:
                pass       
    elif preType == "road":
        Pre = preID
    else: 
        print("unsupported predecessor type")
    return(Pre, contType)


def latlon(lat,longi):
     u = utm.from_latlon(lat, longi)
     return(u)
    
def gnssPlot(a):
        liste=[a[0],a[1]]
        c = plot(liste[0],liste[1])
        #c.remove()

CoordSysOriginPoiX = latlon(10,20)[0] 
CoordSysOriginPoiY = latlon(10,20)[1]

def egoVehQuat2Euler(imuMsg):
    egoVehQuaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)
    egoVehEuler = tf.transformations.euler_from_quaternion(egoVehQuaternion)
    egoVehEuler_roll = egoVehEuler[0]
    egoVehEuler_pitch = egoVehEuler[1]
    egoVehEuler_yaw = egoVehEuler[2]
    return [egoVehEuler_roll,egoVehEuler_pitch,egoVehEuler_yaw]

def getGeoInRoad(road):
    calc_roadGEO = []
    for geo in road.roadPlanview.iter("geometry"):
        calc_roadGEO.append(geo)
    return calc_roadGEO


def findVehTendency(prevRoad, prevGeoIndex, currRoad, currGeoIndex, nextRoad, nextGeoIndex, egoYaw):
    tendency = 'none'

    prevGeos = getGeoInRoad(prevRoad)
    currGeos = getGeoInRoad(currRoad)
    nextGeos = getGeoInRoad(nextRoad)

    prevGeo = prevGeos[prevGeoIndex]
    currGeo = currGeos[currGeoIndex]
    nextGeo = nextGeos[nextGeoIndex]

    prevX = float(prevGeo.attrib["x"]) + CoordSysOriginPoiX
    prevY = float(prevGeo.attrib["y"]) + CoordSysOriginPoiY
    currX = float(currGeo.attrib["x"]) + CoordSysOriginPoiX
    currY = float(currGeo.attrib["y"]) + CoordSysOriginPoiY
    nextX = float(nextGeo.attrib["x"]) + CoordSysOriginPoiX
    nextY = float(nextGeo.attrib["y"]) + CoordSysOriginPoiY

    orient_curr2prev = math.atan2(prevY-currY, prevX-currX)
    orient_curr2next = math.atan2(nextY-currY, nextX-currX)

    
    print("prev2curr:", orient_curr2prev)
    print("curr2next:", orient_curr2next)
    print("egoYaw: ", egoYaw)

    egoYaw2prev = abs(egoYaw - orient_curr2prev)
    egoYaw2next = abs(egoYaw - orient_curr2next)
    if egoYaw2prev > egoYaw2next:
        print("1")
        tendency = 'toNext'
    else:
        print("2")
        tendency = 'toPrevious'

    return tendency
    



def sketchLine(geo, roadColour, linewidth):
    ax = plt.gca()
    geoX = float(geo.attrib["x"])+ CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"])
    length = float(geo.attrib["length"])
    sucX = np.cos(hdg)*length + float(geoX)
    sucY = np.sin(hdg)*length + float(geoY)
    plt.plot([geoX, sucX], [geoY, sucY], roadColour, linewidth= linewidth)     

def sketchArc(geo, roadColour, linewidth):
    ax = plt.gca()

    geoX = float(geo.attrib["x"])+ CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"])
    length = float(geo.attrib["length"])

    for arc in geo:
        radius = 1/float(arc.get('curvature'))
    
    radiussign = np.sign(radius)
    radius = abs(radius)
    
    if radiussign < 0:
        origX = np.cos(hdg -math.pi/2)*radius + geoX
        origY = np.sin(hdg -math.pi/2)*radius + geoY
    else:
        origX = -np.cos(hdg -math.pi/2)*radius + geoX
        origY = -np.sin(hdg -math.pi/2)*radius + geoY

    alfa = math.degrees(length/radius)
    tht1 = math.degrees(hdg- math.pi/2)

    if radiussign < 0:
        tht2 = tht1 + 180
        tht1 = tht1 - alfa + 180
    else: 
        tht2 = tht1 + alfa

    arc_element = Arc((origX, origY), 2*radius, 2*radius, angle=0 , theta1=tht1, theta2=tht2, linewidth= linewidth, zorder=0,color=roadColour)#, linestyle = "-")
    ax.add_patch(arc_element)

def findGridLine(geo, segLen, printBool, linewidth):
    gridIDList = []
    if printBool:
        ax = plt.gca()
    geoX = float(geo.attrib["x"])+ CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"])
    length = float(geo.attrib["length"])    

    totalStep = int(np.floor(length/segLen))

    gridID = findGrid(gridList, geoX, geoY)
    gridIDList.append(gridID)

    for segIndex in range(totalStep):

        geoX2 = np.cos(hdg)*(segLen) + float(geoX)
        geoY2 = np.sin(hdg)*(segLen) + float(geoY)

        if printBool == True:
            roadColor = 'k'
            if segIndex % 2 == 0:
                roadColor = 'r'
            plt.plot([geoX, geoX2],[geoY, geoY2], roadColor, linewidth = linewidth)

        gridID = findGrid(gridList, geoX2, geoY2)
        if gridID in gridIDList:
            pass
        else:
            gridIDList.append(gridID)

        geoX = geoX2
        geoY = geoY2
    return gridIDList

def findGridArc(geo, segLen, printBool, linewidth):
    gridIDList = []
    if printBool == True:
        ax = plt.gca()
    # 2.1. Define the parameters.
    geoX = float(geo.attrib["x"])+ CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"])
    length = float(geo.attrib["length"])

    for arc in geo:
        radius = 1/float(arc.get('curvature'))
    
    radiussign = np.sign(radius)
    radius = abs(radius)
    
    # 2.2. Find the center.
    if radiussign < 0:
        origX = np.cos(hdg -math.pi/2)*radius + geoX
        origY = np.sin(hdg -math.pi/2)*radius + geoY
    else:
        origX = -np.cos(hdg -math.pi/2)*radius + geoX
        origY = -np.sin(hdg -math.pi/2)*radius + geoY

    # 2.3. Find the interval.
    alfa = math.degrees(length/radius)
    tht1 = math.degrees(hdg- math.pi/2)

    if radiussign < 0:
        tht2 = tht1 + 180
        tht1 = tht1 - alfa + 180
    else: 
        tht2 = tht1 + alfa        

    # 2.4. Divide the interval.
    interval = abs(tht1 - tht2)
    angleStep = (segLen*interval)/length

    # 2.5. Find the related grids.
    totalStep = int(np.floor(abs(tht1 - tht2)/angleStep))
    
    gridID = findGrid(gridList, geoX, geoY)
    gridIDList.append(gridID)

    for angleIndex in range(totalStep):
        if printBool == True:
            roadColor = 'k'
            if angleIndex % 2 == 0:
                roadColor = 'r'
            arc_element = Arc((origX, origY), 2*radius, 2*radius, angle=0 , theta1=tht1+(angleStep*(angleIndex-1)), theta2=tht1+(angleStep*angleIndex), linewidth=linewidth, zorder=0, color=roadColor)#, linestyle = "-")
            ax.add_patch(arc_element)
        
        if radiussign > 0:
            geoX2 = origX + (geoX-origX)*np.cos(np.deg2rad(angleStep*(angleIndex-1))) - np.sin(np.deg2rad(angleStep*(angleIndex-1)))*(geoY-origY)
            geoY2 = origY + (geoX-origX)*np.sin(np.deg2rad(angleStep*(angleIndex-1))) + np.cos(np.deg2rad(angleStep*(angleIndex-1)))*(geoY-origY)

        else:
            geoX2 = origX + (geoX-origX)*np.cos(np.deg2rad(angleStep*(angleIndex-1)-90)) - np.sin(np.deg2rad(angleStep*(angleIndex-1)-90))*(geoY-origY)
            geoY2 = origY + (geoX-origX)*np.sin(np.deg2rad(angleStep*(angleIndex-1)-90)) + np.cos(np.deg2rad(angleStep*(angleIndex-1)-90))*(geoY-origY)

        gridID = findGrid(gridList, geoX2, geoY2)
        if gridID in gridIDList:
            pass
        else:
            gridIDList.append(gridID)

    return gridIDList    

def findGrid(gridList, posX, posY):
    for gridIndex in range(len(gridList)):
        if (gridList[gridIndex].xMin <= posX <= gridList[gridIndex].xMax):
            if (gridList[gridIndex].yMin <= posY <= gridList[gridIndex].yMax):
                return gridList[gridIndex].gridNo
    return -1

def findDistLine(geo, egoVeh_pos, segLen, printBool):
    distList = []
    if printBool:
        ax = plt.gca()
    geoX = float(geo.attrib["x"])+ CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"])
    length = float(geo.attrib["length"])    

    if segLen != -1:

        totalStep = int(np.floor(length/segLen))

        firstDist = np.sqrt(np.square(geoX - egoVeh_pos[0]) + np.square(geoY - egoVeh_pos[1]))            
        distList.append(firstDist)

        for segIndex in range(totalStep):

            geoX2 = np.cos(hdg)*(segLen) + float(geoX)
            geoY2 = np.sin(hdg)*(segLen) + float(geoY)

            if printBool == True:
                roadColor = 'k'
                if segIndex % 2 == 0:
                    roadColor = 'r'
                plt.plot([geoX, geoX2],[geoY, geoY2], roadColor)

            tempDist = np.sqrt(np.square(geoX2 - egoVeh_pos[0]) + np.square(geoY2 - egoVeh_pos[1]))            
            distList.append(tempDist)

            geoX = geoX2
            geoY = geoY2
    else:
        firstDist = np.sqrt(np.square(geoX - egoVeh_pos[0]) + np.square(geoY - egoVeh_pos[1]))
        distList.append(firstDist)
        geoX2 = geoX + np.cos(hdg)*length
        geoY2 = geoY + np.sin(hdg)*length
        tempDist = np.sqrt(np.square(geoX2 - egoVeh_pos[0]) + np.square(geoY2 - egoVeh_pos[1]))
        distList.append(tempDist)
    return min(distList)
        
def findDistArc(geo, egoVeh_pos, segLen, printBool):
    distList = []
    if printBool == True:
        ax = plt.gca()
    # 2.1. Define the parameters.
    geoX = float(geo.attrib["x"])+ CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"])
    length = float(geo.attrib["length"])

    for arc in geo:
        radius = 1/float(arc.get('curvature'))
    
    radiussign = np.sign(radius)
    radius = abs(radius)
    
    # 2.2. Find the center.
    if radiussign < 0:
        origX = np.cos(hdg -math.pi/2)*radius + geoX
        origY = np.sin(hdg -math.pi/2)*radius + geoY
    else:
        origX = -np.cos(hdg -math.pi/2)*radius + geoX
        origY = -np.sin(hdg -math.pi/2)*radius + geoY

    # 2.3. Find the interval.
    alfa = math.degrees(length/radius)
    tht1 = math.degrees(hdg- math.pi/2)

    if radiussign < 0:
        tht2 = tht1 + 180
        tht1 = tht1 - alfa + 180
    else: 
        tht2 = tht1 + alfa        

    if segLen != -1:
        # 2.4. Divide the interval.
        interval = abs(tht1 - tht2)
        angleStep = (segLen*interval)/length

        # 2.5. Find the related grids.
        totalStep = int(np.floor(abs(tht1 - tht2)/angleStep))
        
        firstDist = np.sqrt(np.square(geoX - egoVeh_pos[0]) + np.square(geoY - egoVeh_pos[1]))
        distList.append(firstDist)

        for angleIndex in range(totalStep):
            if printBool == True:
                roadColor = 'k'
                if angleIndex % 2 == 0:
                    roadColor = 'r'
                arc_element = Arc((origX, origY), 2*radius, 2*radius, angle=0 , theta1=tht1+(angleStep*(angleIndex-1)), theta2=tht1+(angleStep*angleIndex), linewidth=1, zorder=0, color=roadColor)#, linestyle = "-")
                ax.add_patch(arc_element)
            
            if radiussign > 0:
                geoX2 = origX + (geoX-origX)*np.cos(np.deg2rad(angleStep*(angleIndex-1))) - np.sin(np.deg2rad(angleStep*(angleIndex-1)))*(geoY-origY)
                geoY2 = origY + (geoX-origX)*np.sin(np.deg2rad(angleStep*(angleIndex-1))) + np.cos(np.deg2rad(angleStep*(angleIndex-1)))*(geoY-origY)

            else:
                geoX2 = origX + (geoX-origX)*np.cos(np.deg2rad(angleStep*(angleIndex-1)-90)) - np.sin(np.deg2rad(angleStep*(angleIndex-1)-90))*(geoY-origY)
                geoY2 = origY + (geoX-origX)*np.sin(np.deg2rad(angleStep*(angleIndex-1)-90)) + np.cos(np.deg2rad(angleStep*(angleIndex-1)-90))*(geoY-origY)

            tempDist = np.sqrt(np.square(geoX2 - egoVeh_pos[0]) + np.square(geoY2 - egoVeh_pos[1]))
            distList.append(tempDist)
    else:
        firstDist = np.sqrt(np.square(geoX - egoVeh_pos[0]) + np.square(geoY - egoVeh_pos[1]))
        distList.append(firstDist)
        geoX2 = origX + (geoX-origX)*np.cos(np.deg2rad(alfa)) - np.sin(np.deg2rad(alfa))*(geoY-origY)
        geoY2 = origY + (geoX-origX)*np.sin(np.deg2rad(alfa)) + np.cos(np.deg2rad(alfa))*(geoY-origY)
        tempDist = np.sqrt(np.square(geoX2 - egoVeh_pos[0]) + np.square(geoY2 - egoVeh_pos[1]))
        distList.append(tempDist)
    return min(distList)

def findDist(road, geoIndex, egoVeh_pos, segLen, printBool):
    roadGEO = []
    
    for geo in road.roadPlanview.iter("geometry"):
        roadGEO.append(geo)    
    
    for child in roadGEO[geoIndex]: 
        if child.tag == "arc":
            dist = findDistArc(roadGEO[geoIndex], egoVeh_pos, segLen, printBool)
            curv = child.attrib["curvature"]
        elif child.tag == "line":
            dist = findDistLine(roadGEO[geoIndex], egoVeh_pos, segLen, printBool)
            curv = 1     
        else: 
            print("unsupported geometry type!!!!")

    return dist, curv

def findCurvature(road, geoIndex):

    preRoad = road
    currRoad = road
    nextRoad = road

    preGeo = geoIndex
    currGeo = geoIndex
    nextGeo = geoIndex

    preCurv = -1
    currCurv = -1
    nextCurv = -1   

    roadGEO = []
    suc_curv = 0
    for geo in road.roadPlanview.iter("geometry"):
        roadGEO.append(geo) 

    if len(roadGEO) == 1:
        # previous geometry
        presRoadAll = findPredecessor(roadList[road.roadId])
        presRoadID = presRoadAll[0]
        presType = presRoadAll[1]        
        if presType == "end":
            presGeo = getGeoInRoad(roadList[presRoadID])
            presTotalGeo = len(presGeo)
            if presGeo[presTotalGeo-1].find("arc") is not None:
                arcTag = presGeo[presTotalGeo-1].find("arc")
                preCurv = arcTag.attrib["curvature"]
            preRoad = roadList[presRoadID]
            preGeo = presTotalGeo-1
        elif presType == "start":                      
            presGeo = getGeoInRoad(roadList[presRoadID])
            if presGeo[0].find("arc") is not None:
                arcTag = presGeo[0].find("arc")
                preCurv = arcTag.attrib["curvature"]
            preRoad = roadList[presRoadID]
            preGeo = 0
        # current geometry
        if roadGEO[geoIndex].find("arc") is not None:
            arcTag = roadGEO[geoIndex].find("arc")
            currCurv = arcTag.attrib["curvature"]
        # next geometry
        sucRoadAll = findSuccessor(roadList[road.roadId])
        sucRoadID = sucRoadAll[0]        
        sucType = sucRoadAll[1]
        #sketchGeoInRoad(roadList[sucRoadID], 0, 'g')
        if sucType == "end":
            sucGeo = getGeoInRoad(roadList[sucRoadID])
            sucTotalGeo = len(sucGeo)
            if sucGeo[sucTotalGeo-1].find("arc") is not None:
                arcTag = sucGeo[sucTotalGeo-1].find("arc")
                nextCurv = arcTag.attrib["curvature"]
            nextRoad = roadList[sucRoadID]
            nextGeo = sucTotalGeo-1            
        elif sucType == "start":
            sucGeo = getGeoInRoad(roadList[sucRoadID])
            if sucGeo[0].find("arc") is not None:
                arcTag = presGeo[0].find("arc")
                nextCurv = arcTag.attrib["curvature"]
            nextRoad = roadList[sucRoadID]
            nextGeo = 0

    else:
        if geoIndex == 0:
            # previous geometry
            presRoadAll = findPredecessor(roadList[road.roadId])
            presRoadID = presRoadAll[0]
            presType = presRoadAll[1]        
            if presType == "end":
                presGeo = getGeoInRoad(roadList[presRoadID])
                presTotalGeo = len(presGeo)
                if presGeo[presTotalGeo-1].find("arc") is not None:
                    arcTag = presGeo[presTotalGeo-1].find("arc")
                    preCurv = arcTag.attrib["curvature"]
                preRoad = roadList[presRoadID]
                preGeo = presTotalGeo-1
            elif presType == "start":                      
                presGeo = getGeoInRoad(roadList[presRoadID])
                if presGeo[0].find("arc") is not None:
                    arcTag = presGeo[0].find("arc")
                    preCurv = arcTag.attrib["curvature"]
                preRoad = roadList[presRoadID]
                preGeo = 0
            # current geometry
            if roadGEO[geoIndex].find("arc") is not None:
                arcTag = roadGEO[geoIndex].find("arc")
                currCurv = arcTag.attrib["curvature"]
            # next geometry
            if roadGEO[geoIndex+1].find("arc") is not None:
                arcTag = roadGEO[geoIndex+1].find("arc")
                nextCurv = arcTag.attrib["curvature"]
            nextRoad = road
            nextGeo = geoIndex+1


        elif geoIndex == len(roadGEO)-1:
            # previous geometry
            if roadGEO[geoIndex-1].find("arc") is not None:
                arcTag = roadGEO[geoIndex-1].find("arc")
                preCurv = arcTag.attrib["curvature"]
            preRoad = road
            preGeo = geoIndex-1
            # current geometry
            if roadGEO[geoIndex].find("arc") is not None:
                arcTag = roadGEO[geoIndex].find("arc")
                currCurv = arcTag.attrib["curvature"]
            # next geometry
            sucRoadAll = findSuccessor(roadList[road.roadId])
            sucRoadID = sucRoadAll[0]
            sucType = sucRoadAll[1]
            if sucType == "end":
                sucGeo = getGeoInRoad(roadList[sucRoadID])
                sucTotalGeo = len(sucGeo)            
                if sucGeo[sucTotalGeo-1].find("arc") is not None:
                    arcTag = sucGeo[sucTotalGeo-1].find("arc")
                    nextCurv = arcTag.attrib["curvature"]
                nextRoad = roadList[sucRoadID]
                nextGeo = sucTotalGeo-1
            elif sucType == "start":
                sucGeo = getGeoInRoad(roadList[sucRoadID])
                if sucGeo[0].find("arc") is not None:
                    arcTag = presGeo[0].find("arc")
                    nextCurv = arcTag.attrib["curvature"]
                nextRoad = roadList[sucRoadID]
                nextGeo = 0       
        else:
            # previous geometry
            if roadGEO[geoIndex-1].find("arc") is not None:
                arcTag = roadGEO[geoIndex-1].find("arc")
                preCurv = arcTag.attrib["curvature"]
            preRoad = road
            preGeo = geoIndex-1
            # current geometry
            if roadGEO[geoIndex].find("arc") is not None:
                arcTag = roadGEO[geoIndex].find("arc")
                currCurv = arcTag.attrib["curvature"]
            # next geometry
            if roadGEO[geoIndex+1].find("arc") is not None:
                arcTag = roadGEO[geoIndex+1].find("arc")
                nextCurv = arcTag.attrib["curvature"]
            nextRoad = road
            nextGeo = geoIndex+1

    return preRoad, preGeo, preCurv, currRoad, currGeo, currCurv, nextRoad, nextGeo, nextCurv

def findCurvSegHeading(road, geoIndex, posX, posY):
    geos = getGeoInRoad(road)
    geo = geos[geoIndex]
    geoX = float(geo.attrib["x"]) + CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    geoCurv = geo.find("arc").attrib["curvature"]
    geoRad = 1 / float(geoCurv)
    geoHead = math.degrees(float(geo.attrib["hdg"])) #degree
    
    if geoHead < 0:
        geoHead = geoHead + math.degrees(math.pi*2) #degree
    
    print("geometry heading: ", geoHead)
    distGeo2Poi = np.sqrt(np.square(geoX-posX)+np.square(geoY-posY)) #m 
    alfa = np.arccos(1-(np.square(distGeo2Poi)/(2*np.square(geoRad)))) #radian 
    headPos = geoHead + math.degrees(alfa)
    print("heading of Position:", headPos)
    return headPos


def sketchGeoInRoad(road, geoIndex, roadColour, linewidth):
    roadGEO = []
    
    for geo in road.roadPlanview.iter("geometry"):
        roadGEO.append(geo)
    
    
    for child in roadGEO[geoIndex]:
        if child.tag == "arc":
            sketchArc(roadGEO[geoIndex], roadColour, linewidth)
        elif child.tag == "line":
            sketchLine(roadGEO[geoIndex], roadColour, linewidth)
        else: 
            print("unsupported geometry type!!!!")

def prepareMap(road, segLen, drawRoad, drawSeg, roadColour, linewidth):
    roadGEO = []
    
    for geo in road.roadPlanview.iter("geometry"):
        roadGEO.append(geo)
    
    for geoIndex in range(len(roadGEO)):
       for child in roadGEO[geoIndex]:
            if child.tag == "arc":
                if drawRoad:
                    sketchArc(roadGEO[geoIndex], roadColour, linewidth)

                gridElement = findGridArc(roadGEO[geoIndex], segLen, drawSeg, linewidth)
                for jj in range(len(gridElement)):
                    gridRoadIDList[gridElement[jj]].append(int(road.roadId))
                    gridGeoIDList[gridElement[jj]].append(geoIndex)


            elif child.tag == "line":
                if drawRoad:
                    sketchLine(roadGEO[geoIndex], roadColour, linewidth)

                gridElement = findGridLine(roadGEO[geoIndex], segLen, drawSeg, linewidth)
                for jj in range(len(gridElement)):
                    gridRoadIDList[gridElement[jj]].append(int(road.roadId))
                    gridGeoIDList[gridElement[jj]].append(geoIndex)
                
            else: 
                print("unsupported geometry type!!!!")            



if __name__ == '__main__': 
    rospy.init_node("hd_service_provider")
    rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, gnssCallback)
    rospy.Subscriber("/carla/ego_vehicle/imu", Imu, imuCallback)
    rospy.Subscriber("/carla/world_info", CarlaWorldInfo, WorldInfoCallback)
    currCurvPub = rospy.Publisher('/carla/hd_map/currCurv', Float32, queue_size=10)
    currCurvHeadPub = rospy.Publisher('/carla/hd_map/currCurvHead', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    plt.show()
    plt.ion()
    printOnce = True
    while len(WIMsg.map_name) == 0 and not rospy.is_shutdown():
        if printOnce:
            print("\n")
            print("Waiting for HD Map...")
            print("\n")
            printOnce = False

    hdMapET = ET.fromstring(WIMsg.opendrive)

    # 1. Read the Roads.
    # 1.1. Create the array that has all the roads in it.
    maxRoadNo = 2500                                            # TODO: Insert it automatically.
    roadList = []
    junctionList = []
    maxJuncNo = 2500
    lowerlimit = 2
    upperlimit = 3
    limitIndex = 0
    for roadIndex in range(0,maxRoadNo):
        tempRoadObj = roadStruct()
        roadList.append(tempRoadObj)
    for junctionIndex in range(0,maxJuncNo):
        junction = junctionStruct()
        junctionList.append(junction)
    # 1.2. Insert the roads in to the array with their id's aligned. 
    for roadIndex in hdMapET.iter('road'):
        tempRoadObj = roadStruct()
        tempRoadObj_roadId = roadIndex.attrib['id']
        tempRoadObj.roadId = int(tempRoadObj_roadId)
        tempRoadObj.roadLink = roadIndex.find('link')
        tempRoadObj.roadType = roadIndex.find('type')
        tempRoadObj.roadPlanview = roadIndex.find('planView')
        tempRoadObj.roadElevationProfile = roadIndex.find('elevationProfile')
        tempRoadObj.roadLateralProfile = roadIndex.find('lateralProfile')
        tempRoadObj.roadLanes = roadIndex.find('lanes')
        tempRoadObj.roadObjects = roadIndex.find('objects')
        tempRoadObj.roadSignals = roadIndex.find('signals')
        roadList[int(tempRoadObj_roadId)] = tempRoadObj
    
    # 2. Read the Junctions.
    for junctionIndex in hdMapET.iter('junction'):
        junction = junctionStruct()
        junction_junctionID = junctionIndex.attrib["id"]
        junction.junctionID = int(junction_junctionID)
        junction.junctionData = junctionIndex
        junctionList[int(junction.junctionID)] = junction

    
    #print(ET.tostring(junctionList[25].junctionData, encoding='utf8', method='xml'))
    # 3. Grid the map 
    for header in hdMapET.iter('header'):
        
        ymin = float(header.attrib["south"]) + CoordSysOriginPoiY
        ymax = float(header.attrib["north"]) + CoordSysOriginPoiY
        xmin = float(header.attrib["west"]) + CoordSysOriginPoiX
        xmax = float(header.attrib["east"]) + CoordSysOriginPoiX
        plt.xlim(xmin-20, xmax+20)
        plt.ylim(ymin-20, ymax+20)

    gridList = []
    gridNumber = 36 #must be rootable
    gridId = 1
    for gridIndexY in range(int(np.sqrt(gridNumber))):
        tempGrid_yMin = ymin + ((ymax-ymin)/np.sqrt(gridNumber))*gridIndexY
        tempGrid_yMax = tempGrid_yMin + ((ymax-ymin)/np.sqrt(gridNumber))
        for gridIndexX in range(int(np.sqrt(gridNumber))):
            tempGrid = gridStruct()  
            tempGrid.gridNo = gridId
            tempGrid.yMin = tempGrid_yMin
            tempGrid.yMax = tempGrid_yMax
            tempGrid.xMin = xmin + ((xmax-xmin)/np.sqrt(gridNumber))*gridIndexX
            tempGrid.xMax = tempGrid.xMin + ((xmax-xmin)/np.sqrt(gridNumber))
            gridList.append(tempGrid)
            gridId = gridId + 1

    # 3.1. Plot the grids.
    #for gridIndex in range(len(gridList)):
    #    plt.plot([gridList[gridIndex].xMin, gridList[gridIndex].xMax], [gridList[gridIndex].yMin, gridList[gridIndex].yMin], 'b')
    #    plt.plot([gridList[gridIndex].xMin, gridList[gridIndex].xMin], [gridList[gridIndex].yMin, gridList[gridIndex].yMax], 'b')
    #    plt.plot([gridList[gridIndex].xMin, gridList[gridIndex].xMax], [gridList[gridIndex].yMax, gridList[gridIndex].yMax], 'b')
    #    plt.plot([gridList[gridIndex].xMax, gridList[gridIndex].xMax], [gridList[gridIndex].yMin, gridList[gridIndex].yMax], 'b')


    # 4. Prepare map (Assign roads into grids and draw the map.)
    gridRoadIDList = []
    gridGeoIDList = []
    for jj in range(gridNumber):
        gridRoadIDList.append([])
    for hh in range(gridNumber):
        gridGeoIDList.append([])

    roadInit = 0
    roadFinal = 2500    
    for roadNumber in range(roadInit, roadFinal):
        if roadList[roadNumber].roadLink != "":
            prepareMap(roadList[roadNumber], 2, True, False, "k", 3)
            pass

    ax = plt.gca()
   
    

    while not rospy.is_shutdown():
        # Get ego-vehicle position.
        egoVeh_pos = latlon(gnssMsg.latitude, gnssMsg.longitude)
        egoVehEuler = egoVehQuat2Euler(imuMsg)
        gnssPlot(egoVeh_pos)
        # Find related grid.
        egoVeh_gridId = findGrid(gridList, egoVeh_pos[0], egoVeh_pos[1])
        # Find the current geometry.
        distArray = []
        for jj in range(len(gridRoadIDList[egoVeh_gridId])):            
            dist = findDist(roadList[gridRoadIDList[egoVeh_gridId][jj]], gridGeoIDList[egoVeh_gridId][jj], egoVeh_pos, 2, False)[0]
            distArray.append(dist)        
        # Divide into segments or calculate with distances."""
        
        if len(distArray) > 0:
            minDistIndex = distArray.index(min(distArray))
            #sketchGeoInRoad(roadList[gridRoadIDList[egoVeh_gridId][minDistIndex]], gridGeoIDList[egoVeh_gridId][minDistIndex], 'r', 3)

            #print("roadID: ", roadList[gridRoadIDList[egoVeh_gridId][minDistIndex]].roadId, " geoID:",  gridGeoIDList[egoVeh_gridId][minDistIndex])
            preRoad, preGeo, preCurv, currRoad, currGeo, currCurv, nextRoad, nextGeo, nextCurv = findCurvature(roadList[gridRoadIDList[egoVeh_gridId][minDistIndex]], gridGeoIDList[egoVeh_gridId][minDistIndex])
            print("PreCurv: ", preCurv, " CurrCurv: ", currCurv, " NextCurv: ", nextCurv)
            #sketchGeoInRoad(preRoad, preGeo, 'm', 3)
            #sketchGeoInRoad(nextRoad, nextGeo, 'g', 3)

            vehTendency = findVehTendency(preRoad, preGeo, currRoad, currGeo, nextRoad, nextGeo, egoVehEuler[2])        # Fix the bug !.
            if vehTendency == 'toNext':
                tendencyRoad = nextRoad
                tendencyGeo = nextGeo
                tendencyCurv = nextCurv
            else:
                tendencyRoad = preRoad
                tendencyGeo = preGeo
                tendencyCurv = preCurv

            #sketchGeoInRoad(tendencyRoad, tendencyGeo, 'y', 3) #"m"
            
            if currCurv != -1:
                headPos = findCurvSegHeading(currRoad, currGeo, egoVeh_pos[0], egoVeh_pos[1])
                currCurvMsg.data = float(currCurv)
                currCurvHeadMsg.data = float(headPos)
                currCurvPub.publish(currCurvMsg)
                currCurvHeadPub.publish(currCurvHeadMsg)
            

        

        

        plt.draw()       
        plt.show()   
        plt.pause(0.00000000001)
        
        rate.sleep()





#xy = utm.from_latlon(gnssMsg.latitude, gnssMsg.longitude)
    
