import rospy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# an object to calculate the inverse kinematics of a point,
# and simulate the first order dynamics of a 2 link manipulator


class twoDofInverseKinematics(object):
    def __init__(self, lenArm0, lenArm1):
        self.lenArm0 = lenArm0
        self.lenArm1 = lenArm1
        self.lastCoordinate = (0, 0)
    # inverse kinematics equations for 2 link manipulator

    def ik(self, x, y):
        l1 = self.lenArm0
        l2 = self.lenArm1

        dist = np.sqrt(np.square(x)+np.square(y))
        gamma = np.arccos(
            np.divide((np.square(dist)+np.square(l1)-np.square(l2)), (2*l1*dist)))
        theta1 = np.arctan2(y, x) - gamma

        theta2 = np.arctan2((y - (l1 * np.sin(theta1))),
                            (x - (l1 * np.cos(theta1)))) - theta1
        return theta1, theta2

    # simulate first order dynamics from one joint state to the next
    # return 2 arrays of joint angles that the manipulator should take
    # on at every iteration, till reaching the proper position.

    def lowPassFilter(self, beginning_orientation, end_angles, a=0.3):
        # end_angles = self.ik(end_point[0], end_point[1])
        curr_x = beginning_orientation[0]
        curr_y = beginning_orientation[1]
        coordinates = [(curr_x, curr_y)]

        while(not ((end_angles[1]-0.01 <= curr_y <= end_angles[1] + 0.01) and
                   (end_angles[0]-0.01 <= curr_x <= end_angles[0] + 0.01))):
            curr_y = a * end_angles[1] + (1-a) * curr_y
            curr_x = a * end_angles[0] + (1-a) * curr_x
            coordinates.append((curr_x, curr_y))


        return coordinates

    # publish a single joint state message

    def sendJointState(self, theta1, theta2):
        pub = rospy.Publisher('mnist_joint_publisher',
                              JointState, queue_size=200)
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

    # move the arm across the contour points provided

    def move(self, contours, originFlag = False):
        coordinates = [(0, 0)]
        if contours is not None and not originFlag:
            for contour in contours:
                for unpack in contour:
                    for c in unpack:
                        x = float(c[0]*0.03)
                        y = float(c[1]*0.03)
                        theta1, theta2 = self.ik(x, y)
                        coordinates.append((theta1, theta2))
            self.lastCoordinate = coordinates[-1]
        else:
            coordinates = [self.lastCoordinate, (2,2)]
        filteredCoordinates = []
        for i in range(1, len(coordinates)):
            filteredCoordinates.extend(self.lowPassFilter(
                coordinates[i-1], coordinates[i]))
        for c in filteredCoordinates:
            theta1, theta2 = c[0], c[1]
            self.sendJointState(theta1, theta2)
        return coordinates