#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import math

from common.recognize import TrackedObj, resolve_object_stereo_contours, get_circularity
from common.camera    import XZCamera, YZCamera, find_closest_point
from common.stereo    import normalize_to_robot
from common.kinematics import Kinematics
import os
import signal

class StereoVisionController:

    # Defines publisher and subscriber
    def __init__(self, cam_yz, cam_xz, tracked=[], basejoint='YellowJoint'):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)


        self.killpid = os.getpid()

        self.j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        self.j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        self.j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        self.j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
    
        # initialize position vectors
        self.positions = {}
        for track in tracked:
            self.positions[track.name] = [0.0, 0.0, 0.0]

        self.cv_image_yz = None
        self.cv_image_xz = None
        self.tracked_objects = tracked
        self.basejoint = basejoint
        self.camera_yz = cam_yz
        self.camera_xz = cam_xz
        self.kmatics   = Kinematics()

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    # Receive data from camera 1, process it, and publish
    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image_yz = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Uncomment if you want to save the image
        #cv2.imwrite('image1_copy.png', self.cv_image_yz)

        #im1=cv2.imshow('window1', markup_image(self.cv_image_yz))
        #cv2.waitKey(1)
        
        self.locate()
        # Publish the results
        #try: 
        #    self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image_yz, "bgr8"))
        #except CvBridgeError as e:
        #    print(e)

    def callback2(self, data):
        # Receive the image
        try:
            self.cv_image_xz = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Uncomment if you want to save the image
        #cv2.imwrite('image1_copy.png', self.cv_image_xz)

        #im1=cv2.imshow('window2', markup_image(self.cv_image_xz))
        #cv2.waitKey(1)
        # Publish the results
        #try: 
        #    self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image_xz, "bgr8"))
        #except CvBridgeError as e:
        #    print(e)

    def locate(self):
        # check that we have both images
        if self.cv_image_yz is None or self.cv_image_xz is None:
            return

        for tracked_obj in self.tracked_objects:
            try:
                # search for the tracked object
                results = resolve_object_stereo_contours(
                    self.cv_image_yz,
                    self.cv_image_xz,
                    tracked_obj
                )

                # if we can't see it, then this will just be skipped
                for contour_yz, contour_xz in results:
                    # figure out the slopes by using the camera profiles
                    yz_slopes = self.camera_yz.calculate_slopes(contour_yz)
                    xz_slopes = self.camera_xz.calculate_slopes(contour_xz)

                    # calculate the position using our eq solver
                    position, uncertainty = find_closest_point(
                        self.camera_yz.position, yz_slopes,
                        self.camera_xz.position, xz_slopes
                    )

                    # save the position. this has the effect of ensuring
                    # we only ever have one position for an object at a time
                    self.positions[tracked_obj.name] = position
            except Exception as e:
                #print('!!! Object {} is occluded!'.format(tracked_obj.name))
                pass

        # normalize all the positions
        for key in self.positions:
            self.positions[key] = normalize_to_robot(
                self.positions[self.basejoint],
                self.positions[key],
                scale=(7.0/3.88)#np.array([(7.0/3.88), (7.0/3.88), (7.0/3.88)])
            )


        # testing
        #print('red pos: {}, green z: {}'.format(
        #    self.positions['RedJoint'],
        #    self.positions['GreenJoint'][2]  
        #))




    
        # find the IK angles
        pos_arr = np.array([
            self.positions['RedJoint'][0],
            self.positions['RedJoint'][1],
            self.positions['RedJoint'][2],
            self.positions['GreenJoint'][2]
        ])
        self.kmatics.set_end_effector_pos(pos_arr)

        #np.array([
        #    0.3131,
        #    -0.0076,
        #    3.8841,
        #    2.9092
        #]))
        q = self.kmatics.control_closed( pos_arr )

        #self.j1_pub.publish(q[0])
        #self.j2_pub.publish(q[1])
        #self.j3_pub.publish(q[2])
        #self.j4_pub.publish(q[3])
        
        
        # emulate ctrl-c
        #os.kill(self.killpid, signal.SIGINT)
        #return
        print('q_real: {}'.format( list(q) ))
        return


        #q[2] = q[2] * -1 # HACKS
        #q[3] = q[3] * -0.8 # HACKS
        
        #print('IK arrs: {}'.format(q))

        #[
        #    math.degrees(q[0]),
        #    math.degrees(q[1]),
        #    math.degrees(q[2]),
        #    math.degrees(q[3])
        #]))


        real_effector = np.array(self.positions['RedJoint'])
        #effector = self.kmatics.forward_kinematics([0, 1, -1, 1])
        #effector = self.kmatics.forward_kinematics([0, 0, 0, 1.5707])
        ##effector = self.kmatics.forward_kinematics([0.0, 1, 1, 1])

        effector = self.kmatics.forward_kinematics(q)




        #print('a real: {}'.format(np.array([0.0, 0.0, -1.5707, 0.0])))
        #print('a pred: {}'.format(q))
        

        print('pred: {}\nreal: {}'.format( list(effector), list(real_effector )))
        #print('err:  {}'.format( real_effector - effector ))

        

# call the class
def main(args):
    tracked_objects = [
        TrackedObj(
            'BlueJoint',
            np.array([0,0,255]),
        ),
        TrackedObj(
            'GreenJoint',
            np.array([0,255,0]),
        ),
        TrackedObj(
            'RedJoint',
            np.array([255,0,0]),
        ),
        TrackedObj(
            'YellowJoint',
            np.array([255,255,0]),
            tolerance=5
        ),
        TrackedObj(
            'Sphere Floater',
            np.array([255,165,0]),
            tolerance=5,
            contour_filter=lambda c: get_circularity(c) > 0.9
        ),
        TrackedObj(
            'Cyl Floater',
            np.array([255,165,0]),
            tolerance=5,
            contour_filter=lambda c: get_circularity(c) < 0.9
        )
    ]

    camera_yz = YZCamera(
        position=np.array([18, 0, 3]),
        sensor_size=(800, 800), # dimensions from cv bridge
        fov=math.radians(80) # 80 degree FOV in radians
    )
    camera_xz = XZCamera(
        position=np.array([0, -18, 3]),
        sensor_size=(800, 800), # dimensions from cv bridge
        fov=math.radians(80) # 80 degree FOV in radians
    )

    ic = StereoVisionController(
        camera_yz,
        camera_xz,
        tracked_objects
    )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
