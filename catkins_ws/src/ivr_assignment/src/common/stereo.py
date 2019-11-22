import cv2
import math
import numpy as np
from camera import XZCamera, YZCamera, find_closest_point
from recognize import TrackedObj, resolve_object_stereo_contours

global_eq_name = 'a'

def print_eqs(yz_pos, yz_slopes, xz_pos, xz_slopes):
    def format_npy(arr):
        return '({}, {}, {})'.format(arr[0], arr[1], arr[2])

    global global_eq_name 

    print( 'yz: {}(t) = {} + t*{}'.format(
        global_eq_name,
        format_npy(yz_pos),
        format_npy(yz_slopes)
    ))
    
    global_eq_name = chr(ord(global_eq_name) + 1)

    print( 'xz: {}(t) = {} + t*{}'.format(
        global_eq_name,
        format_npy(xz_pos),
        format_npy(xz_slopes)
    ))
    global_eq_name = chr(ord(global_eq_name) + 1)

def normalize_to_robot(basejoint, target):
    return target - basejoint

if __name__=='__main__':
    img_yz = cv2.imread('../captures/cam1_yz_pose1.png')
    img_xz = cv2.imread('../captures/cam2_xz_pose1.png')

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
#        TrackedObj(
#            'YellowJoint',
#            np.array([255,255,0]),
#            tolerance=5
#        ),
#        TrackedObj(
#            'BlueJoint',
#            np.array([0,0,255]),
#        ),
        TrackedObj(
            'GreenJoint',
            np.array([0,255,0]),
        ),
#        TrackedObj(
#            'RedJoint',
#            np.array([255,0,0]),
#        ),
#        TrackedObj(
#            'Floater',
#            np.array([255,165,0]),
#            tolerance=5
#        )
    ]

    finals = {}

    for tracked_obj in tracked_objects:
        print('resolving {}'.format(tracked_obj.name))
        results = resolve_object_stereo_contours(img_yz, img_xz, tracked_obj)


        for contour_yz, contour_xz in results:
            yz_slopes = camera_yz.calculate_slopes(contour_yz)
            xz_slopes = camera_xz.calculate_slopes(contour_xz)

            #print('\t\tcam 1 pos: {}, ray: {}'.format(
            #        camera_yz.position,
            #        yz_slopes
            #))
            #print('\t\tcam 2 pos: {}, ray: {}'.format(
            #        camera_xz.position,
            #        xz_slopes
            #))

            print_eqs(
                    camera_yz.position, yz_slopes,
                    camera_xz.position, xz_slopes
            )

            position, _ = find_closest_point(
                camera_yz.position, yz_slopes,
                camera_xz.position, xz_slopes
            )

            print('\tresolved 3d position: {}'.format(position))

            finals[tracked_obj.name] = position.copy()


    print('differences:')
    print('\tY->B: {}'.format(
        np.linalg.norm(finals['BlueJoint'] - finals['YellowJoint'])
    ))
    print('\tB->G: {}'.format(
        np.linalg.norm(finals['GreenJoint'] - finals['BlueJoint'])
    ))
    print('\tG->R: {}'.format(
        np.linalg.norm(finals['RedJoint'] - finals['GreenJoint'])
    ))

    # figuring out some shit


    normalized = {
        'YellowJoint': normalize_to_robot(
            finals['YellowJoint'],
            finals['YellowJoint']
        ),
        'BlueJoint': normalize_to_robot(
            finals['YellowJoint'],
            finals['BlueJoint']
        ),
        'GreenJoint': normalize_to_robot(
            finals['YellowJoint'],
            finals['GreenJoint']
        ),
        'RedJoint': normalize_to_robot(
            finals['YellowJoint'],
            finals['RedJoint']
        )
    }

    print('normalized joint positions:')
    for key in normalized:
        print('\t{}: {}'.format(key, normalized[key]))


    # running sanity assertions (for neutral case)
    #assert finals['YellowJoint'][2] < finals['BlueJoint'][2]
    #assert finals['BlueJoint'][2] < finals['GreenJoint'][2]
    #assert finals['GreenJoint'][2] < finals['RedJoint'][2]
