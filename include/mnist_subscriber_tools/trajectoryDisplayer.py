#!/usr/bin/env python


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# this displays the trajectory associated with the current number provide by
# the keyboard input


class trajectoryDisplayer(object):
    def __init__(self):
        self.markerPub = rospy.Publisher('robotMarker', Marker, queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = "/base_link"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.lifetime = rospy.Duration.from_sec(1)
        self.marker.ns = 'testline'

        # marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.points = []

    # This function takes the contour points, and publishes them
    # Markers to Rviz. It also scales those points down to make
    # the trajectory fit in the standard rviz grid

    def addContourPointsAndPublish(self, contours):
        del self.marker.points[:]
        if contours is not None:
            for contour in contours:
                for unpack in contour:
                    for c in unpack:
                        p = Point()
                        p.x = float(c[0]*0.03)
                        p.y = float(c[1]*0.03)
                        p.z = 0
                        self.marker.points.append(p)
            if not rospy.is_shutdown():
                self.markerPub.publish(self.marker)

    # clear all of the previous markers

    def clearPrevious(self):
        self.marker.action = self.marker.DELETEALL
        self.marker.points = []
        self.markerPub.publish(self.marker)
