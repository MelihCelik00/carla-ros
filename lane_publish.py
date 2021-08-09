#!/usr/bin/env python
import glob
import os
import sys
import time

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import carla_common.transforms as trans
from carla_msgs.msg import CarlaWorldInfo, CarlaEgoVehicleInfo
from carla_waypoint_types.srv import GetWaypoint, GetActorWaypoint

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
    pub = rospy.Publisher("/vehicle/current_lane_type", String, queue_size=10)
    rate = rospy.Rate(10)
    
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    _map = world.get_map()

    vehicle = world.get_actor(vehInfo.id)

    while not rospy.is_shutdown():
        try:
            _current_lane_info = vehicle.get_world().get_map().get_waypoint(vehicle.get_location())
            print(_current_lane_info.lane_type)
            print("Right Lane Type: ",_current_lane_info.get_right_lane())
            time.sleep(1.0)

            pub.publish(str(_current_lane_info.lane_type))
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
        