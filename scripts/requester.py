#!/usr/bin/env python2
##
# @file 
# @brief Defines and runs node that requests the creation of random polygons
# @author Biraj Parikh
# @date April 24, 2020
#
# - PARAMETERS:
#   - None
# - PUBLISHES:
#   - /polygonRequests
# - SUBSCRIBES:
#   - /builtPolygons
# - SERVICES:
#   - None
# - INTERFACES:
#   - None

import random

import rospy
from doxygen_test.msg import PolygonBuildData, PolygonBuildDataArr

##
# @brief Defines a node to request the creation of different polygons
# 
# Broadcasts a message to publish a certain polygon, which has its parameters chosen randomly.
# Subscribes to a message that creator broadcasts enumerating all previously created polygons.
# Prints this list out
class Requester:
    def __init__(self):
        """!@brief Defines a node to request the creation of different polygons.

        Broadcasts a message to publish a certain polygon, which has its parameters chosen 
        randomly. Subscribes to a message that creator broadcasts enumerating all previously 
        created polygons. This list is printed out. 
        """
        
        ## possible polygon colors
        self.possibleColors = ["blue", "black", "red", "green"]

        ## possible polygon types
        self.possibleTypes = ["triangle", "square"]

        ## publisher for polygon creation requests 
        self.requestPub = rospy.Publisher("/polygonRequests", PolygonBuildData, queue_size=0)

        rospy.Subscriber("/builtPolygons", PolygonBuildDataArr, self.built_polygons_cb)

        ## frequency (hertz) at which requests should be sent
        self.requestFrequency = 1 

        rospy.init_node("Requester", anonymous=True)

    def built_polygons_cb(self, msg):
        built_polygons = []
        for poly in msg.polygons:
            built_polygons.append((poly.type, poly.sideLength, poly.color))
        rospy.loginfo("built polygons: " + str(built_polygons))

    def build_request_msg(self):
        msg = PolygonBuildData()
        msg.type = random.choice(self.possibleTypes)
        msg.sideLength = random.randint(1, 10)
        msg.color = random.choice(self.possibleColors)
        return msg

    def run(self):
        rate = rospy.Rate(1)  # 1hz
        while not rospy.is_shutdown():
            msg = self.build_request_msg()
            info_str = "Requesting " + msg.type + " with side length " + str(msg.sideLength) + " and color " + msg.color
            rospy.loginfo(info_str)
            self.requestPub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        requester = Requester()
        requester.run()
    except rospy.ROSInterruptException:
        pass
