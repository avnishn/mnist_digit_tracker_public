#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
from matplotlib import pyplot as plt
from random import randint
import numpy
import sys


# converts image to cv2 format and then an image classifier is run on it
class image_converter:
  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("mnist_image",Image,self.callback)
    self.graph = tf.Graph()
    self.sess = tf.Session(graph=self.graph)
    model_path = "./src/mnist_trained_network/model"
    tf.saved_model.loader.load(self.sess,
                              [tag_constants.SERVING],
                              model_path
                              )
    # input tensor, x
    pred2 = self.graph.get_tensor_by_name("out:0")
    # output tensor, pred
    x = self.graph.get_tensor_by_name('x:0')
    self.tensors = (pred2, x)

  # rosnode callback, accepts, converts, classifies image, and finding image contours
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
    resized_image = cv2.resize(cv_image, (28, 28))
    classification = self.sess.run(tf.argmax(self.tensors[0], 1), \
                                        feed_dict={self.tensors[1]:\
                                        [numpy.asarray(resized_image).ravel()]})
    _, contours, hierarchy = cv2.findContours(cv_image.copy(), \
      cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cv2.drawContours(cv_image, contours, -1, (255,255,0), 3)
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