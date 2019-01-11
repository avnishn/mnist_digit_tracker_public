#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher("mnist_joint_publisher", JointState, queue_size=100)
    #pub2 = rospy.Publisher("joint_states", JointState, queue_size=100)
    rospy.init_node('mover_pub')
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['base_link__link_2', 'link_2__link_3']
    hello_str.position = [1.575, 1.575]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)

    while not rospy.is_shutdown():
      hello_str.header.stamp = rospy.Time.now()
      pub.publish(hello_str)
      #pub2.publish(hello_str)
      rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass