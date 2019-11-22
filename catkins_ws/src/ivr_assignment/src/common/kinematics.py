
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from numpy import cos, sin

class Kinematics:

    def __init__(self):
        #rospy.init_node('kinematics', anonymous=True)

        self.time_trajectory = rospy.get_time()

        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')

        self.error = np.array([0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0,0.0], dtype='float64')


    def forward_kinematics(self,joints):
        a=joints[0]
        b=joints[1]
        c=joints[2]
        d=joints[3]

        end_effector = np.array([2* cos(d)*(cos(a)*sin(b)*cos(c)-sin(a)*sin(c))-2*sin(d)*cos(a)*cos(b) + 3*cos(a)*sin(b)*cos(c)+2*cos(a)*sin(b)-3*sin(a)*sin(c),
        2*cos(d)*(sin(a)*sin(b)*cos(c)+cos(a)*sin(c))-2*sin(d)*sin(a)*cos(b)+3*sin(a)*sin(b)*cos(c)+3*cos(a)*sin(c)+2*sin(a)*sin(b),
        2*cos(d)*cos(b)*cos(c) + 2*sin(d)*sin(b) +3*cos(b)*cos(c)+2*cos(b)])

        return end_effector

    # Calculate the robot Jacobian
    def calculate_jacobian(self,joints):
        a=joints[0]
        b=joints[1]
        c=joints[2]
        d=joints[3]

        jacobian = np.array([[2*cos(d)*((-1)*sin(a)*sin(b)*cos(c) - cos(a)*sin(c))+ 2*sin(d)*sin(a)*cos(b) -3*sin(a)*sin(b)*cos(c)-2*sin(a)*sin(b)-3*cos(a)*sin(c),
                              2*cos(d)*(cos(a)*cos(b)*cos(c))+2*sin(d)*cos(a)*sin(b)+3*cos(a)*cos(b)*cos(c)+2*cos(a)*cos(b),
                              2*cos(d)*((-1)*cos(a)*sin(b)*sin(c)-sin(a)*cos(c))-3*cos(a)*sin(b)*sin(c)-3*sin(a)*cos(c),
                              (-2)*sin(d)*(cos(a)*sin(b)*cos(c)-sin(a)*sin(c))-2*cos(d)*cos(a)*cos(b)],
                           [2*cos(d)*(cos(a)*sin(b)*cos(c)-sin(a)*sin(c))-cos(a)*cos(b)*2*sin(d)+3*cos(a)*sin(b)*cos(c)-3*sin(a)*sin(c)+2*cos(a)*sin(b),
                           2*cos(d)*(sin(a)*cos(b)*cos(c) + cos(a)*sin(c))+2*sin(d)*sin(a)*sin(b)+3*sin(a)*cos(b)*cos(c)+2*sin(a)*cos(b),
                           2*cos(d)*((-1)*sin(a)*sin(b)*sin(c)+cos(a)*cos(c))-3*sin(a)*sin(b)*sin(c)+3*cos(a)*cos(c),
                           (-2)*sin(d)*(sin(a)*sin(b)*cos(c)+cos(a)*sin(c))-2*cos(d)*sin(a)*cos(b)],
                           [0,
                           (-2)*cos(d)*sin(b)*cos(c)+2*sin(d)*cos(b)-3*sin(b)*cos(c)-2*sin(b),
                           (-2)*cos(d)*cos(b)*sin(c)-3*cos(b)*sin(c),
                           (-2)*sin(d)*cos(b)*cos(c)+2*cos(d)*sin(b)],
                           [0,
                           (-3)*sin(b)*cos(c)-2*sin(b),
                           (-3)*cos(b)*sin(c),
                           0
                           ]])
        return jacobian

    # Estimate control inputs for open-loop control
    def control_open(self,angles):

        # estimate time step
        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous_step2
        self.time_previous_step2 = cur_time

        q = np.array([0,0,0,0])
        J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian

        # desired trajectory
        #pos_d=
        # estimate derivative of desired trajectory
        self.error_d = (pos_d - self.error)/dt
        self.error = pos_d
        q_d = q + (dt * np.dot(J_inv, self.error_d.transpose()))  # desired joint angles to follow the trajectory
        return q_d


    def control_closed(self,position,q =np.array([0,0,0,0])):
        # we assume we start with a vertical arm,  all angles are null and we want
        #to find the angles that would get us to the current state

        K_p = np.array([[10,0],[0,10]])
        K_d = np.array([[0.1,0],[0,0.1]])

        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        # robot end-effector position
        pos = np.array([0,0,7])

        # desired trajectory
        pos_d=position

        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt

        # estimate error
        self.error = pos_d-pos

        J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian

        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)

        return q_d

    def callback(self,data):
        # Recieve the image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        self.joints=Float64MultiArray()
        self.joints.data= np.array([0,0,0,0])

        #self.end_effector.data= position of end effector

        # send control commands to joints (lab 3)
        q_d = self.control_closed(cv_image)
        #q_d = self.control_open(cv_image)
        self.joint1=Float64()
        self.joint1.data= q_d[0]
        self.joint2=Float64()
        self.joint2.data= q_d[1]
        self.joint3=Float64()
        self.joint3.data= q_d[2]
        self.joint4=Float64()
        self.joint4=Float64()

        # Publishing the desired trajectory on a topic named trajectory(lab 3)
        x_d = self.end_effector    # getting the desired trajectory
        # Publish the results
        try:
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)

# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
