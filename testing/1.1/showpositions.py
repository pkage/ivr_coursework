import cv2
import math
import numpy as np
from camera import XZCamera, YZCamera, find_closest_point
from recognize import TrackedObj, resolve_object_stereo_contours

if __name__=='__main__':
    img_yz = cv2.imread('../captures/cam1_yz_neutral.png')
    img_xz = cv2.imread('../captures/cam2_xz_neutral.png')


    # camera setup
    camera_yz = YZCamera(
        position=np.array([18, 0, 3]),
        sensor_size=img_yz.shape,
        fov=math.radians(80) # 80 degree FOV in radians
    )
    camera_xz = XZCamera(
        position=np.array([0, -18, 3]),
        sensor_size=img_xz.shape,
        fov=math.radians(80) # 80 degree FOV in radians
    )
    
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
        )
#        TrackedObj(
#            'Floater',
#            np.array([255,165,0]),
#            tolerance=5
#        )
    ]

    for tracked_obj in tracked_objects:
        print('resolving {}'.format(tracked_obj.name))
        results = resolve_object_stereo_contours(img_yz, img_xz, tracked_obj)

        for contour_yz, contour_xz in results:
            yz_slopes = camera_yz.calculate_slopes(contour_yz)
            xz_slopes = camera_xz.calculate_slopes(contour_xz)

            print('\t\tcam 1 pos: {}, ray: {}'.format(
                    camera_yz.position,
                    yz_slopes
            ))
            print('\t\tcam 2 pos: {}, ray: {}'.format(
                    camera_xz.position,
                    xz_slopes
            ))

            position = find_closest_point(
                camera_yz.position, yz_slopes,
                camera_xz.position, xz_slopes
            )

            print('\tresolved 3d position: {}'.format(position))


