import sys
import cv2
import math
import numpy as np

# H: 0-179, S: 0-255, V: 0-255 for opencv color ranges

class TrackedObj:
    __rgb_color = None
    __tolerance = None
    lower_color = None
    upper_color = None
    name        = None
    coords      = None
    

    def __init__(self, name, rgb_color, coords=None, tolerance=10):
        self.name        = name
        self.__rgb_color = rgb_color
        self.__tolerance = tolerance
    
        hsv_color = cv2.cvtColor(np.uint8([[ rgb_color ]]), cv2.COLOR_RGB2HSV)[0][0]

        self.lower_color = hsv_color + np.uint8([ -tolerance, -200, -200 ])
        self.upper_color = hsv_color + np.uint8([  tolerance,    0,    0 ])

        if (self.lower_color[0] > 179):
            self.lower_color[0] = 180 - (255 - self.lower_color[0])
        if (self.upper_color[0] > 179):
            self.upper_color[0] = 180 - (255 - self.upper_color[0])

        if coords is None:
            self.coords = [None, None, None]
        else:
            self.coords = coords

    def copy(self):
        return TrackedObj(
            self.name,
            self.__rgb_color,
            coords=self.coords,
            tolerance=self.__tolerance
        )

    def __repr__(self):
        return '<TrackedObj: name={}, lower={}, upper={}>'.format(self.name, self.lower_color, self.upper_color)

    def add_coords_xz(self, coords):
        self.coords = [coords[0], self.coords[1], coords[2]]

    def add_coords_yz(self, coords):
        self.coords = [self.coords[0], coords[1], coords[2]]


    def add_coords(self, coords):
        self.coords[0], self.coords[1], self.coords[2] = coords

def countour_center(contour):
    m = cv2.moments(contour)
    cX = int(m["m10"] / m["m00"])
    cY = int(m["m01"] / m["m00"])
    return (cX, cY)


def find_contours(image, lower_color, upper_color):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

    if lower_color[0] > upper_color[0]:
        # lower is higher in the HSV space than the upper,
        # so we need to OR together two ranges
        mask_u = cv2.inRange(
            hsv,
            np.uint8([ 0, lower_color[1], lower_color[2] ]),
            upper_color
        )

        mask_l = cv2.inRange(
            hsv,
            lower_color,
            np.uint8([ 179, upper_color[1], upper_color[2] ])
        )

        # combine the masks
        mask = cv2.bitwise_or( mask_u, mask_l )
        
    else:
        mask = cv2.inRange(hsv, lower_color, upper_color) 
    
    contours, hierarchy = cv2.findContours(
            mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
    )

    return contours


def show_contours(image, contours, color=(255,255,255), name=None):
    for c in contours:
        m = cv2.moments(c)
        b = cv2.boundingRect(c)
        cv2.rectangle(
                image,
                (b[0], b[1]),
                (b[0] + b[2], b[1] + b[3]),
                color,
                1,
                0
        )

        cX = int(m["m10"] / m["m00"])
        cY = int(m["m01"] / m["m00"])
        cv2.circle(image, (cX, cY), 1, color, -1)

        if name is not None:
            cv2.putText(
                    image,
                    name,
                    (b[0] + b[2] + 2, b[1] + int(b[2] / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1
            )

    return image


def resolve_objects(xz_image, yz_image, tracked_objects):
    """
    Resolve objects within an image

    :param xz_image: XZ image
    :param yz_image: YZ image
    :param tracked_objects: [TrackedObj] array to track
    """
    # scale objects
    x_scale = xz_image.shape[1] # shape is (rows, cols, [color depth])
    y_scale = yz_image.shape[1]
    z_scale = xz_image.shape[0]

    processed = []
    for tracked in tracked_objects:
        print('scanning for {}'.format(tracked.name))

        # find the contours from the xz and yz images
        contours_xz = find_contours(
            xz_image.copy(),
            tracked.lower_color,
            tracked.upper_color
        )
        contours_yz = find_contours(
            yz_image.copy(),
            tracked.lower_color,
            tracked.upper_color
        )

        # handle edge cases (e.g. more than one tracked object,
        #       obfuscated objects, etc.)
        if len(contours_xz) != len(contours_yz):
            print('!!! obfuscated object')
            # test
        else:
            for contour_xz, contour_yz in zip(contours_xz, contours_yz):
                # copy the target (in case there are multiple)
                current_target = tracked.copy()

                x, z = countour_center(contour_xz)
                y, z = countour_center(contour_yz)

                # convert everything to floats, and convert to relative coords
                x = float(x) / float(x_scale)
                y = float(y) / float(y_scale)
                z = float(z) / float(z_scale)

                print('\textracted {}'.format(
                    (x, y, z)
                ))

                processed.append(current_target)

    return processed



if __name__=="__main__":
    img_yz = cv2.imread('../captures/cam1_yz_pose1.png')
    img_xz = cv2.imread('../captures/cam2_xz_pose1.png')


    #marked = img.copy()

    #print('image dimensions:', img.shape)

    tracked_objects = [
        TrackedObj(
            'BlueJoint',
            [0,0,255],
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
            'Floater',
            np.array([255,165,0]),
            tolerance=5
        )
    ]

    resolve_objects(img_yz, img_xz, tracked_objects)

    sys.exit(0)

    for tracked in tracked_objects:
        #print('resolving %s' % tracked.name)
        print('resolving %s' % tracked)
        moments = find_contours(img.copy(), tracked.lower_color, tracked.upper_color)
        try:
            marked = show_contours(marked, moments, name=tracked.name)
        except Exception as e:
            print('\tsomething went wrong with %s' % tracked.name)

    cv2.imwrite('test.png', marked)

#im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#for c in contours:
# calculate moments for each contour
#M = cv2.moments(c)

# calculate x,y coordinate of center
#cX = int(M["m10"] / M["m00"])
#cY = int(M["m01"] / M["m00"])
#cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
#cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# display the image
#cv2.imshow("Image", img)
#cv2.waitKey(0)
