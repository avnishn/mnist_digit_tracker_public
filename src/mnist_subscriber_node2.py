#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospkg

import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
from matplotlib import pyplot as plt

from random import randint
import numpy as np
import math
import sys
from collections import deque

# an object to calculate the inverse kinematics of a point, 
# and simulate the first order dynamics of a 2 link manipulator
class DynamicsAndKinematics(object):
  def __init__ (self, lenArm0, lenArm1):
    self.lenArm0 = lenArm0
    self.lenArm1 = lenArm1

  #inverse kinematics equations for 2 link manipulator
  def ik(self, x, y):
    l1 = self.lenArm0
    l2 = self.lenArm1

    dist = np.sqrt(np.square(x)+np.square(y))
    gamma = np.arccos( np.divide( (np.square(dist)+np.square(l1)-np.square(l2)), (2*l1*dist) ) )
    theta1 = np.arctan2(y,x) - gamma

    theta2 = np.arctan2((y - (l1 * np.sin(theta1))), (x - (l1 * np.cos(theta1)))) - theta1

    fwdX = l1*np.cos(theta2) + l2*np.cos(theta1+theta2)
    fwdy = l1*np.sin(theta2) + l2*np.sin(theta1+theta2)
    print (x, fwdX)
    print (y, fwdy)

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
      x.append(curr_x)
      y.append(curr_y)
    return x, y


  # publish a single joint state message
  def sendJointState(self, theta1, theta2):
    pub = rospy.Publisher('mnist_joint_publisher', JointState, queue_size=200)
    rate = rospy.Rate(10)
    joint_msg = JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ["joint0", "joint1"]
    joint_msg.position = [theta1, theta2]
    joint_msg.velocity = []
    joint_msg.effort = [] 
    pub.publish(joint_msg)
    rate.sleep()
  
  def move(self,contours):
    coordinates = []
    if contours is not None:
      for contour in contours:
        for unpack in contour:
          for c in unpack:
            x = float(c[0]*0.03)
            y = float(c[1]*0.03)
            theta1, theta2 = self.ik(x,y)
            coordinates.append((theta1,theta2))
            self.sendJointState(theta1, theta2)
    return coordinates

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
    if contours is not None:
      for contour in contours:
        for unpack in contour:
          for c in unpack:
            p = Point()
            p.x = float (c[0]*0.05)
            p.y = float (c[1]*0.05)
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
    # self.trajectoryDisp = trajectoryDisplayer()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("mnist_image",Image,self.imageConverterCallback)
    self.graph, self.sess, self.tensors = self.constructGraph()
    
    self.currContour = None
    self.currNumber = None
    self.changedFlag = False

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
    # cv2.drawContours(cv_image, contours, -1, (255,255,0), 3)
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(100)
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
                                        [np.asarray(resized_image).ravel()]})

    self.currContour = self.getContoursAndDisplay(cv_image)
    self.currNumber = int(classification)
    # self.trajectoryDisp.addContourPointsAndPublish(self.currContour)


if __name__ == '__main__':
  # link1, link2 Len = 2.2 m
  ik = DynamicsAndKinematics(2.2, 2.2)
  ic = image_converter()
  trajDisp = trajectoryDisplayer()
  
  rospy.init_node('mnist_image_subscriber', anonymous=False)
  rospy.Rate(100)
  while not rospy.is_shutdown():
    # get the current contour and numbers
    contours = ic.currContour
    number = ic.currNumber
    #if number is not None:
      #rospy.loginfo("classified as the number %d", number) 
    # publish the trajectory
    #trajDisp.addContourPointsAndPublish(contours)
    # move the robot
    coordinates= ik.move(contours)
    #ik.sendJointState(0.5,0.5)
  cv2.destroyAllWindows()