#!/usr/bin/env python
import glob
import os
import sys
import time

import rospy

from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleInfo

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

vehInfo = CarlaEgoVehicleInfo()

def vehicleInfoCallback(data):
    global vehInfo
    vehInfo = data
    #print(data.id)

if __name__ == '__main__': 
    rospy.init_node("lane_type_publish")
    rospy.Subscriber("/carla/ego_vehicle/vehicle_info", CarlaEgoVehicleInfo, vehicleInfoCallback)
    pubLaneType = rospy.Publisher("carla/vehicle/current_lane_type", String, queue_size=10)
    pubLaneID = rospy.Publisher("carla/vehicle/current_road_id", String, queue_size=10)
    pubLeftLaneType = rospy.Publisher("carla/vehicle/left_lane_type", String, queue_size=10)
    pubRightLaneType = rospy.Publisher("carla/vehicle/right_lane_type", String, queue_size=10)
    laneChangePub = rospy.Publisher("carla/vehicle/changeable_lane", String, queue_size=10)
    rate = rospy.Rate(10)
    
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    _map = world.get_map()

    vehicle = world.get_actor(vehInfo.id)

    while not rospy.is_shutdown():
        try:
            _current_lane_info = vehicle.get_world().get_map().get_waypoint(vehicle.get_location(), project_to_road = True, lane_type=carla.LaneType.Any)
            print("lane id: ",  _current_lane_info.lane_id)
            print("lane type: ")
            print(_current_lane_info.lane_type)
            #print(_current_lane_info)
            try:
                left_lane = _current_lane_info.get_left_lane().lane_type
                right_lane = _current_lane_info.get_right_lane().lane_type
                print("Left Lane Type: ")
                print(str(left_lane))
                print("Right Lane Type: ")
                print(right_lane)
            except AttributeError:
                print("None type can't have change_lane_type!!")

            lane_change = _current_lane_info.lane_change
            print("lane change: ")
            print(lane_change)

            time.sleep(1.0)

            pubLaneType.publish(str(_current_lane_info.lane_type))
            pubLaneID.publish(str(_current_lane_info.road_id))
            pubLeftLaneType.publish(str(left_lane))
            pubRightLaneType.publish(str(right_lane))
            laneChangePub.publish(str(lane_change))
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
        
