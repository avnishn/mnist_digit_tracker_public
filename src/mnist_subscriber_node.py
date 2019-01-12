#!/usr/bin/env python

import rospy

# check mnist_digit_tracker/include/mnist_subscriber_tools
from mnist_subscriber_tools import imageSubscriber
from mnist_subscriber_tools import twoDofInverseKinematics


if __name__ == '__main__':
    # link1, link2 Len = 2.2 m
    rospy.init_node('mnist_image_subscriber', anonymous=False)
    ik = twoDofInverseKinematics(2.2, 2.2)
    ic = imageSubscriber()
    rospy.Rate(100)
    while not rospy.is_shutdown():
        # get the current contour and numbers
        contours = ic.currContour
        number = ic.currNumber
        # if the number that is classified by the tf classifier is 0, then
        # we should move to the position
        if number is 0:
          coordinates = ik.move(None, originFlag = True)
          rospy.sleep(3.)
        else:
          coordinates = ik.move(contours)
