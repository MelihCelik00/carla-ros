def countLanes(roadLanes): # pull 'lanes' tagged elements
    _count = 0
    for child in roadLanes:
        if child.tag == "laneSection":
            for sideOfSection in child: # get into the laneSection tag and search for right or left sides
                if sideOfSection.tag == "right" or sideOfSection.tag == "left":
                    for laneCount in sideOfSection: # counts all type of lanes but we need only 'driving' type
                        if laneCount.attrib['type'] == "driving":
                            #print("Length: ", laneCount.tag, "ID: ", laneCount.attrib['id'])
                            _count += 1 # for each driveable lane

    return _count # return total count of lanes
  
  def storeLaneInfo(road):
    """  
    -current-       
    laneDict = {road id: 
                    lane id :{
                        [ a_value, lane_type, side]
                        }
                }
    """
    for laneSec in road.roadLanes.iter("laneSection"):
        laneDict[str(road.roadId)] = {}
        # now in the laneSection tag
        for sideOfRoadGeo in laneSec:
            
            for letsGetToTheWidth in sideOfRoadGeo:
                if sideOfRoadGeo.tag != "center": # left ids are positives, rights are negatives
                    whichSide = sideOfRoadGeo.tag
                # now should be in the right or left tag
                    for laneIteration in letsGetToTheWidth.iter("lane"):
                        # lane iteration is the list of lane tags
                        if not laneIteration.attrib["type"] == "none":
                            widthAttrib = laneIteration.find("width")
                            r_id = road.roadId
                            l_id = laneIteration.attrib["id"]
                            l_type = laneIteration.attrib["type"]
                            widthVal = widthAttrib.attrib["a"]
                            
                            laneDict[str(road.roadId)][l_id] = {}
                            laneDict[str(r_id)][l_id] = [widthVal, l_type, whichSide]
    
                            
                            print("roadId: ", r_id, 
                                   "laneId: ", l_id, 
                                   "laneType: ", l_type, 
                                   "widthValue: ", widthVal)

def sketchLanes(road, geo, roadId, roadColour, linewidth):
    ax = plt.gca()

    # pull road geometry and calculate reference line 
    geoX = float(geo.attrib["x"]) + CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"]) # pull orientation (heading)
    length = float(geo.attrib["length"]) # pull roads length
    # finding end coordinates of line by polar coordinate calculations
    sucX = np.cos(hdg)*length + float(geoX)
    sucY = np.sin(hdg)*length + float(geoY) 

    # Georeference by using polar coordinates and find starting points of lanes 1-by-1
    for laneIdCount in range(len(laneDict[str(roadId)])):
        if laneDict[str(roadId)].values()[laneIdCount][1] == "driving":
            if laneDict[str(roadId)].values()[laneIdCount][2] == "left":
                _coeff = int(laneDict[str(roadId)].keys()[laneIdCount]) * -1
                #print("coeff", _coeff)
                dist = _coeff * float(laneDict[str(roadId)].values()[laneIdCount][0])

            elif laneDict[str(roadId)].values()[laneIdCount][2] == "right":
                _coeff = int(laneDict[str(roadId)].keys()[laneIdCount]) * -1
                #print("coeff",_coeff)
                dist = _coeff * float(laneDict[str(roadId)].values()[laneIdCount][0])
            else:
                pass
        laneInitialCoord_x = dist * np.cos(90+hdg) + float(geoX)
        laneInitialCoord_y = dist * np.sin(90+hdg) + float(geoY)
        laneFinalCoord_x = np.cos(hdg)*length + float(laneInitialCoord_x)
        laneFinalCoord_y = np.sin(hdg)*length + float(laneInitialCoord_y) 
        plt.plot([laneInitialCoord_x, laneFinalCoord_x], [laneInitialCoord_y, laneFinalCoord_y], linewidth=0.5, linestyle = 'dashed',color='green')

    pass

def sketchCurvedLanes(road, geo, roadId, roadColour, linewidth):
    ax = plt.gca()

    plotable = False
    laneInitialCoord_x = 0.0
    laneInitialCoord_y = 0.0
    dist = 0.0
    dist = 0.0

    geoX = float(geo.attrib["x"])+ CoordSysOriginPoiX
    geoY = float(geo.attrib["y"]) + CoordSysOriginPoiY
    hdg = float(geo.attrib["hdg"])
    length = float(geo.attrib["length"])

    for laneIdCount in range(len(laneDict[str(roadId)])):
        if laneDict[str(roadId)].values()[laneIdCount][1] == "driving":
            if laneDict[str(roadId)].values()[laneIdCount][2] == "left":
                _coeff = int(laneDict[str(roadId)].keys()[laneIdCount])
                #print("coeff", _coeff)
                dist = _coeff * float(laneDict[str(roadId)].values()[laneIdCount][0])

            elif laneDict[str(roadId)].values()[laneIdCount][2] == "right":
                _coeff = int(laneDict[str(roadId)].keys()[laneIdCount])
                dist = _coeff * float(laneDict[str(roadId)].values()[laneIdCount][0])
            else:
                pass
            
        laneInitialCoord_x = dist * np.cos(90+hdg) + float(geoX)
        laneInitialCoord_y = dist * np.sin(90+hdg) + float(geoY)


        for arc in geo:
            radius = 1/float(arc.get('curvature'))
        
        radiussign = np.sign(radius)
        radius = abs(radius)
        
        if radiussign < 0:
            origX = np.cos(hdg -math.pi/2)*radius + laneInitialCoord_x
            origY = np.sin(hdg -math.pi/2)*radius + laneInitialCoord_y
        else:
            origX = -np.cos(hdg -math.pi/2)*radius + laneInitialCoord_x
            origY = -np.sin(hdg -math.pi/2)*radius + laneInitialCoord_y

        alfa = math.degrees(length/radius)
        tht1 = math.degrees(hdg- math.pi/2)

        if radiussign < 0:
            tht2 = tht1 + 180
            tht1 = tht1 - alfa + 180
        else: 
            tht2 = tht1 + alfa

        arc_element = Arc((origX, origY), 2*radius, 2*radius, angle=0 , theta1=tht1, theta2=tht2, linewidth= 0.5, zorder=0,color='green', linestyle = "dashed")
        ax.add_patch(arc_element)

