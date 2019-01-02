#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def imageRetrieve():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('mnist_image_subscriber', anonymous=False)

    rospy.Subscriber('mnist_image', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("mnist_image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)


    cv2.imshow("Image window", cv_image)
    cv2.waitKey(100)


if __name__ == '__main__':
  ic = image_converter()
  rospy.init_node('mnist_image_subscriber', anonymous=False)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()