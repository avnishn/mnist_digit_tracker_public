
import numpy as np
import rospkg
import rospy
import tensorflow as tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tensorflow.python.saved_model import tag_constants

import cv2

from . import trajectoryDisplayer


class imageSubscriber(object):

    def __init__(self, subscribedTopicName = "mnist_image"):
        # currently used for displaying trajectory
        self.trajectoryDisp = trajectoryDisplayer()
        self.subscribedTopic = subscribedTopicName
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            self.subscribedTopic, Image, self.imageConverterCallback)
        self.graph, self.sess, self.tensors = self.constructGraph()

        self.currContour = None
        self.currNumber = None
        self.changedFlag = False

    # reconstruct graph from pretrained mnist classifier model

    def constructGraph(self):
        graph = tf.Graph()
        sess = tf.Session(graph=graph)
        rospack = rospkg.RosPack()
        model_path = rospack.get_path(
            "mnist_digit_tracker") + "/mnist_trained_network/model"
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

    # get contours from OpenCv image

    def getContours(self, cv_image):
        _, contours, _ = cv2.findContours(cv_image.copy(),
                                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    # rosnode callback, accepts, converts, classifies image, and finding image contours

    def imageConverterCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
        resized_image = cv2.resize(cv_image, (28, 28))
        classification = self.sess.run(tf.argmax(self.tensors[0], 1),
                                       feed_dict={self.tensors[1]:
                                                  [np.asarray(resized_image).ravel()]})

        _, self.currContour, _ = cv2.findContours(cv_image.copy(),
                                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.currNumber = int(classification)
        self.trajectoryDisp.addContourPointsAndPublish(self.currContour)
