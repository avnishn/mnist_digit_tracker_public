#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospkg

import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
from matplotlib import pyplot as plt
from random import randint
import numpy
import math
import sys


class DynamicsAndKinematics(object):
  
  def __init__ (self, lenArm0, lenArm1):
    self.lenArm0 = lenArm0
    self.lenArm1 = lenArm1

  #inverse kinematics equations for 2 link manipulator
  def ik(self, x, y):
    l_0 = self.lenArm0
    l_1 = self.lenArm1
    dist = math.sqrt(x**2 + y**2)
    gamma = math.acos((dist**2 + l_0**2 - l_1**2) // (2*l_0*dist))
    theta1 = math.atan2(y,x) - gamma
    a, b = y - (l_0 * math.sin(theta1)), x - (l_0 * math.cos(theta1))
    theta2 = math.atan2(a,b) - theta1
    return theta1, theta2
  
  # simulate first order dynamics from one joint state to the next
  # return 2 arrays of joint angles that the manipulator should take 
  # on at every iteration, till reaching the proper position.
  def lowPassFilter(self, beginning_orientation, end_point, a = 0.3):
    end_angles = self.ik(end_point[0], end_point[1])
    curr_x = beginning_orientation[0]
    curr_y = beginning_orientation[0]
    x = [curr_x]
    y = [curr_y]

    while( not ( (end_angles[1]-0.01 <= curr_y <= end_angles[1] + 0.01) and \
                  (end_angles[0]-0.01 <= curr_x <= end_angles[0] + 0.01)) ):
      curr_y = a * end_angles[1] + (1-a) * curr_y
      curr_x = a * end_angles[0] + (1-a) * curr_x
      print(curr_x, curr_y)
      x.append(curr_x)
      y.append(curr_y)
    return x, y

# this displays the trajectory associated with the current number
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

  def addContourPointsAndPublish(self, contours):
    del self.marker.points[:]
    for contour in contours:
      for unpack in contour:
        for c in unpack:
          p = Point()
          p.x = float (c[0]*0.03)
          p.y = float (c[1]*0.03)
          p.z = 0
          self.marker.points.append(p)
    self.markerPub.publish(self.marker)
  # clear all of the previous markers
  def clearPrevious(self):
    self.marker.action = self.marker.DELETEALL
    self.marker.points = []
    self.markerPub.publish(self.marker)

# converts image to cv2 format and then an image classifier is run on it
class image_converter:

  def __init__(self):
    # currently used for displaying trajectory
    self.trajectoryDisp = trajectoryDisplayer()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("mnist_image",Image,self.imageConverterCallback)
    self.graph, self.sess, self.tensors = self.constructGraph()
    self.prevContours = numpy.empty(2)

  # reconstruct graph from pretrained mnist classifier model  
  def constructGraph(self):
    graph = tf.Graph()
    sess = tf.Session(graph=graph)
    rospack = rospkg.RosPack()
    model_path = rospack.get_path("mnist_digit_tracker") + "/mnist_trained_network/model"
    tf.saved_model.loader.load(sess,
                              [tag_constants.SERVING],
                              model_path
                              )
    # input tensor, x
    pred2 = graph.get_tensor_by_name("out:0")
    # output tensor, pred
    x = graph.get_tensor_by_name('x:0')
    tensors = (pred2, x)
    return graph, sess, tensors

  def getContoursAndDisplay(self, cv_image):
    _, contours, _ = cv2.findContours(cv_image.copy(), \
      cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image, contours, -1, (255,255,0), 3)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(100)
    return contours

  # rosnode callback, accepts, converts, classifies image, and finding image contours
  def imageConverterCallback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
    resized_image = cv2.resize(cv_image, (28, 28))
    classification = self.sess.run(tf.argmax(self.tensors[0], 1), \
                                        feed_dict={self.tensors[1]:\
                                        [numpy.asarray(resized_image).ravel()]})

    contours = self.getContoursAndDisplay(cv_image)
    self.trajectoryDisp.addContourPointsAndPublish(contours)


if __name__ == '__main__':
  ic = image_converter()
  ik = DynamicsAndKinematics(0.8,0.8)
  rospy.init_node('mnist_image_subscriber', anonymous=False)
  rospy.Rate(50)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()