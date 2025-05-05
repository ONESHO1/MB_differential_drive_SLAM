#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from mb_navigation.srv import *
import cv2
import numpy as np
from std_msgs.msg import String
import geometry_msgs.msg
import yaml
from nav_msgs.msg import OccupancyGrid
    
def server_init():
    rospy.wait_for_service('genpath')
    try:
        segment_name = input("enter segment name : ")
        
        srvproxy = rospy.ServiceProxy('genpath', genpath)
        
        resp1 = srvproxy(segment_name)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print(server_init())
