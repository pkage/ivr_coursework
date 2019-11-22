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
from copy import deepcopy


class StereoVisionController:

    # Defines publisher and subscriber
    def __init__(self, cam_yz, cam_xz, tracked=[], basejoint='YellowJoint'):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.x_publisher = rospy.Publisher("/tracker/target_x", Float64, queue_size=1)
        self.y_publisher = rospy.Publisher("/tracker/target_y", Float64, queue_size=1)
        self.z_publisher = rospy.Publisher("/tracker/target_z", Float64, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)


        #self.j1_pub = rospy.Publisher('/robot/joint1_position_controller/command')
        #self.j2_pub = rospy.Publisher('/robot/joint2_position_controller/command')
        #self.j3_pub = rospy.Publisher('/robot/joint3_position_controller/command')
        #self.j4_pub = rospy.Publisher('/robot/joint4_position_controller/command')
    
        # initialize position vectors
        self.positions = {}
        for track in tracked:
            self.positions[track.name] = [0.0, 0.0, 0.0]

        self.oldpos = None
        self.measurements_to_skip = 0

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

        #if self.oldpos is not None:
        #    self.oldpos = self.positions['Cyl Floater']


        if self.measurements_to_skip > 0:
            if self.measurements_to_skip == 1:
                self.oldpos = None
            self.measurements_to_skip -= 1
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
                print('!!! Object {} is occluded!'.format(tracked_obj.name))
                pass

        if self.oldpos is not None:
            if abs(np.linalg.norm(
                    np.array(self.positions['Cyl Floater']).copy() - self.oldpos.copy()
                )) > 0.75:
                print('rejecting shitty measurements')
                self.positions['Cyl Floater'] = self.oldpos.copy()
                self.measurements_to_skip += 10
                return
        else:    
            self.oldpos = self.positions['Cyl Floater'].copy()

        # normalize all the positions
        for key in self.positions:
            self.positions[key] = normalize_to_robot(
                self.positions[self.basejoint],
                self.positions[key],
                scale=(7.0/3.88)#np.array([(7.0/3.88), (7.0/3.88), (7.0/3.88)])
            )


        
        key = 'Cyl Floater'
        print('{}: {}'.format(key, self.positions[key]))
        self.x_publisher.publish(self.positions[key][0])
        self.y_publisher.publish(self.positions[key][1])
        self.z_publisher.publish(self.positions[key][2])




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
