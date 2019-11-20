import sys
import cv2
import math
import numpy as np

class Camera:
    fov = None
    position = np.array([0,0,0])
    sensor_size = np.array([0,0])

    def __init__(self, fov, position, sensor_size):
        self.fov = float(fov)
        self.position = position
        self.sensor_size = sensor_size

    def get_perpendicular_angles(self, contour):
        # start with generating the moment of the contour
        m = cv2.moments(contour)

        # extract the center of the object (we're assuming centroids)
        cX = float(m["m10"] / m["m00"])
        cY = float(m["m01"] / m["m00"])

        # get the fractions across the image X and Y
        cX /= float(self.sensor_size[0])
        cY /= float(self.sensor_size[1])

        # convert to fractions of the field of view (in radians?)
        cX *= self.fov
        cY *= self.fov

        # and shift over to center
        cX = (self.fov / 2.0) - cX
        cY = (self.fov / 2.0) - cY

        return (cX, cY)

    def calculate_slopes(self, contour):
        """
        Calculate the slopes for a contour spotted with this camera

        :param contour: Contour extracted from this camera's images
        """
        raise NotImplementedError

class XZCamera(Camera):
    def calculate_slopes(self, contour):
        """
        Calculate the slopes for a contour spotted with this camera

        :param contour: Contour extracted from this camera's images
        """
        # X in image = Y in 3d space
        # Y in image = Z in 3d space
        cX, cZ = self.get_perpendicular_angles( contour )
        print('\t\tperpendicular angles for XZ contour: ({}, {})'.format(
            math.degrees(cX),
            math.degrees(cZ)
        ))


        # Base vector : Along the Y axis, cX
        slopes = np.array([ math.tan(cX), 1, math.tan(cX) * math.tan(cZ) ])

        # intermediate step: calculate norm
        slope_vec_len = np.linalg.norm(slopes)

        print('\t\t\tresulting norm is {} => {}'.format(
            slope_vec_len,
            np.linalg.norm(slopes / slope_vec_len)
        ))


        # return
        return slopes#/ slope_vec_len


class YZCamera(Camera):
    def calculate_slopes(self, contour):
        """
        Calculate the slopes for a contour spotted with this camera

        :param contour: Contour extracted from this camera's images
        """
        # X in image = Y in 3d space
        # Y in image = Z in 3d space
        cY, cZ = self.get_perpendicular_angles( contour )
        print('\t\tperpendicular angles for YZ contour: ({}, {})'.format(
            math.degrees(cY),
            math.degrees(cZ)
        ))


        # Base vector : Along the X axis, cY
        slopes = np.array([ -1, math.tan(cY), math.tan(cY) * math.tan(cZ) ])

        # intermediate step: calculate norm
        slope_vec_len = np.linalg.norm(slopes)

        # return
        return slopes# / slope_vec_len

def find_closest_point(point1, slopes1, point2, slopes2):
    """
    Find closest points between two skew lines.

    Warning: Does not quite return the closest point, but it's FINE UGH FUCK

    :param point1: First line origin
    :param slopes1: First line vector
    :param point2: Second line origin
    :param slopes2: Second line origin

    :returns: Vec3
    """
    # find unit direction vector for line C, which is perpendicular to lines A and B
    slopes_perp = np.cross(slopes2, slopes1)
    slopes_perp /= np.linalg.norm(slopes_perp)

    # solve the system derived in user2255770's answer from StackExchange: https://math.stackexchange.com/q/1993990
    RHS = point2 - point1
    LHS = np.array([slopes1, -slopes2, slopes_perp]).T

    # get t1, t2, t3 (solutions to the system of linear eqs)
    solution = np.linalg.solve(LHS, RHS)

    return (point1 + ((solution[0]) * slopes1))
    # p3 = p1 + t1v1
    # so the midpoint of the closest points is p4 = (p3) + (t3/2)(slopes_perp)
    #    -> in other words, halfway up the vector from 
    return (point1 + ((solution[0]) * slopes1)) + (solution[2] * slopes_perp)

# ---
'''

# define lines A and B by two points
XA0 = array([1, 0, 0])
XA1 = array([1, 1, 1])
XB0 = array([0, 0, 0])
XB1 = array([0, 0, 1])

# compute unit vectors of directions of lines A and B
UA = (XA1 - XA0) / norm(XA1 - XA0)
UB = (XB1 - XB0) / norm(XB1 - XB0)
# find unit direction vector for line C, which is perpendicular to lines A and B
UC = cross(UB, UA); UC /= norm(UC)

# solve the system derived in user2255770's answer from StackExchange: https://math.stackexchange.com/q/1993990
RHS = XB0 - XA0
LHS = array([UA, -UB, UC]).T
print(solve(LHS, RHS))
# prints "[ 0. -0.  1.]"

'''
