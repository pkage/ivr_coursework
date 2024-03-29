
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
import math

class Kinematics:

    def __init__(self):
        #rospy.init_node('kinematics', anonymous=True)

        self.time_trajectory = rospy.get_time()

        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')

        self.error = np.array([0.0,0.0,0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0,0.0,0.0,0.0], dtype='float64')

        self.static_end_effector_pos = None

    def set_end_effector_pos(self, pos, override=False):
        if self.static_end_effector_pos is None and not override:
            self.static_end_effector_pos = pos


    def __gen_A(self, t, a, r, d):
        return np.array([
            [cos(t), -1*cos(a)*sin(t),    sin(a)*sin(t), r*cos(t)],
            [sin(t),    cos(a)*cos(t), -1*sin(a)*cos(t), r*sin(t)],
            [     0,           sin(a),           cos(a),        d],
            [     0,                0,                0,        1]
        ])

    def plugIn(self, t, a, r, d):
        return np.array([
            [cos(t),(-1)*sin(t), 0, r],
            [sin(t)*cos(a),cos(t)*cos(a),(-1)*sin(a),(-1)*sin(a)*d],
            [sin(t)*sin(a),cos(t)*sin(a),cos(a),cos(a)*d],
            [0,0,0,1]
        ])

    def forward_kinematics(self,joints):
        a=joints[0]
        b=joints[1]
        c=joints[2]
        d=joints[3]

        
        '''
        A1 = self.__gen_A(a, math.pi / 2, 0, 0)
        A2 = self.__gen_A(b + (math.pi / 2), math.pi / 2, 2, 0)
        A3 = self.__gen_A(c, math.pi / 2, 3, 0)
        A4 = self.__gen_A(d, 0, 2, 0)

        _90  = math.pi / 2.0
        _180 = math.pi
        A1 = self.plugIn(a +  _90, _90, 0, 0)
        A2 = self.plugIn(b +  _90, _90, 2, 0)
        A3 = self.plugIn(c + _180, -_90, 0, 3)
        A4 = self.plugIn(d       ,   0, -2, 0)
        '''

        # WORKING
        _90  = math.pi / 2.0
        _180 = math.pi
        A1 = self.__gen_A(a +  _90,      _90, 0, 2)
        A2 = self.__gen_A(b +  _90,  1 * _90, 0, 0)
        A3 = self.__gen_A(       c, -1 * _90, 3, 0)
        A4 = self.__gen_A(       d,        0, 2, 0)


        mult = np.matmul(
            np.matmul(
                np.matmul(A1, A2),
                A3
            ),
            A4
        )

        return np.array([ mult[0][3], mult[1][3], mult[2][3] ])
    
        # -- backup

        A1 = np.array([
            [cos(a), 0,      sin(a), 0],
            [sin(a), 0, (-1)*cos(a), 0],
            [0,     -1,           0, 0],
            [0,      0,           0, 1]
        ])
        A2 = np.array([
            [(-1)*sin(b), 0, (-1)*cos(b), (-2)*sin(b)],
            [     cos(b), 0, (-1)*sin(b),    2*cos(b)],
            [0,          -1,           0,           0],
            [0,           0,           0,           1]
        ])
        A3 = np.array([
            [cos(c),  0, (-1)*sin(c), 3*cos(c)],
            [sin(c),  0,      cos(c), 3*sin(c)],
            [     0, -1,           0,        0],
            [     0,  0,           0,        1]
        ])
        A4 = np.array([
            [cos(d), (-1)*sin(d), 0, 2*cos(d)],
            [sin(d),      cos(d), 0, 2*sin(d)],
            [     0,           0, 1,        0],
            [     0,           0, 0,        1]
        ])

        # --- restored

        A1 = np.array([
            [cos(a), 0,      sin(a), 0],
            [sin(a), 0, (-1)*cos(a), 0],
            [0,      1,           0, 0],
            [0,      0,           0, 1]
        ])
        A2 = np.array([
            [(-1)*sin(b),  0, (-1)*cos(b), (-2)*sin(b)],
            [     cos(b),  0, (-1)*sin(b),    2*cos(b)],
            [          0, -1,           0,           0],
            [          0,  0,           0,           1]
        ])
        A3 = np.array([
            [cos(c),0,sin(c),3*cos(c)],
            [sin(c),0,(-1)*cos(c),3*sin(c)],
            [0,1,0,0],
            [0,0,0,1]
        ])
        A4 = np.array([
            [cos(d),(-1)*sin(d),0,2*cos(d)],
            [sin(d),cos(d),0,2*sin(d)],
            [0,0,1,0],
            [0,0,0,1]
        ])

        mult = np.matmul(
            np.matmul(
                np.matmul(A1, A2),
                A3
            ),
            A4
        )

        return np.array([ mult[0][3], mult[1][3], mult[2][3] ])
        '''
        

        end_effector = np.array([2* cos(d)*(cos(a)*sin(b)*cos(c)-sin(a)*sin(c))-2*sin(d)*cos(a)*cos(b) + 3*cos(a)*sin(b)*cos(c)+2*cos(a)*sin(b)-3*sin(a)*sin(c),
        2*cos(d)*(sin(a)*sin(b)*cos(c)+cos(a)*sin(c))-2*sin(d)*sin(a)*cos(b)+3*sin(a)*sin(b)*cos(c)+3*cos(a)*sin(c)+2*sin(a)*sin(b),
        2*cos(d)*cos(b)*cos(c) + 2*sin(d)*sin(b) +3*cos(b)*cos(c)+2*cos(b)])

        return end_effector'''

    # Calculate the robot Jacobian
    def calculate_jacobian(self,joints):
        a=joints[0]
        b=joints[1]
        c=joints[2]
        d=joints[3]

        jacobian = [
            [
                2*cos(d)*((cos(a) * sin(b) * cos(c)) - (sin(a) * sin(c))) - (2*sin(d)*cos(a)*cos(b) + 2*cos(a)*sin(b)), #1.1
                2*cos(d)*(sin(a)*cos(b)*cos(c)) + 2*sin(d)*sin(a)*sin(b) + 2*sin(a)*cos(b), # 1.2
                2*cos(d)*(-1*sin(a)*sin(b)*sin(c) + cos(a)*cos(c)) # 1.3
                -2*sin(d)*(sin(a)*sin(b)*cos(c) + cos(a)*sin(c)) - 2*cos(d)*sin(a)*cos(b) # 1.4
            ],
            [
                2*sin(d)*(cos(a)*sin(c) + sin(a)*sin(b)*cos(c)) + 2*sin(a)*cos(b)*sin(d) + 2*sin(a)*sin(b), # 2.1
                2*sin(d)*(-1*cos(a)*cos(b)*cos(c)) + 2*cos(a)*sin(b)*sin(d) - 2*cos(a)*cos(b), # 2.2
                2*sin(d)*(sin(a)*cos(c) + cos(a)*sin(b)*sin(c)), # 2.3
                2*cos(d)*(sin(a)*sin(c) - cos(a)*sin(b)*cos(c)) - 2*cos(a)*cos(b)*cos(d), # 2.4
            ],
            [
                0, # 3.1
                -2*cos(d)*sin(b)*cos(c) + 2*cos(b)*sin(d) - 2*sin(b), # 3.2
                -2*cos(d)*cos(b)*sin(c), # 3.3
                -2*sin(d)*cos(b)*cos(c) + 2*sin(b)*cos(d), # 3.4
            ],
            [
                0, # 4.1
                -2*sin(b), # 4.2
                0, # 4.3
                0  # 4.4
            ]
        ]

        j_out = np.ndarray((4,4))
        for row in range(0,3):
            for col in range(0,3):
                j_out[row][col] = jacobian[row][col]

        return j_out

        '''
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
        '''
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

        K_p = np.array([
            [10.0, 0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0, 10.0],
        ])
        K_d = np.array([
            [0.1, 0.0, 0.0, 0.0],
            [0.0, 0.1, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.1],
        ])

        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        # robot end-effector position
        if self.static_end_effector_pos is None:
            print('no end effector position set')
            return
        pos = self.static_end_effector_pos

        # desired trajectory
        pos_d=position

        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error)/dt

        # estimate error
        self.error = pos_d-pos


        J_inv = self.calculate_jacobian(np.zeros(4))
        J_inv = np.linalg.pinv(J_inv)  # calculating the psudeo inverse of Jacobian

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
