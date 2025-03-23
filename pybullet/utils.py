from collections import namedtuple
import pybullet as p
import matplotlib.pyplot as plt
import datetime
import numpy as np
import os
from scipy.spatial.transform import Rotation as R

#new, from Gemini
def analyze_depth_image(depth_image):
    """
    Analyzes the quality of the depth image.

    Args:
        depth_image: A numpy array representing the depth image.

    Returns:
        A dictionary containing quality metrics:
            - mean_depth: Mean depth value.
            - std_dev_depth: Standard deviation of depth values.
            - percentage_valid: Percentage of valid depth values (non-NaN).
            - completeness: Percentage of pixels with valid depth values.
    """
    valid_depth = depth_image[~np.isnan(depth_image)]

    metrics = {
        'mean_depth': np.mean(valid_depth) if len(valid_depth) > 0 else np.nan,
        'std_dev_depth': np.std(valid_depth) if len(valid_depth) > 0 else np.nan,
        'percentage_valid': (len(valid_depth) / (depth_image.shape[0] * depth_image.shape[1])) * 100,
        'completeness': (len(valid_depth) / (depth_image.shape[0] * depth_image.shape[1])) * 100
    }
    #print(metrics)
    print(f'mean_depth: {metrics['mean_depth']:.2f}, std_dev_depth: {metrics['std_dev_depth']:.2f} percentage_valid: {metrics['percentage_valid']:.2f}, completeness: {metrics['completeness']:.2f} ')
    return metrics

#some are from https://github.com/caelan/pybullet-planning/blob/master/pybullet_tools/utils.py

# Organization and housekeeping
def load_floating_robot(pos, euler, fname, scale=1):
    dirName="/Users/antoniahoffman/PycharmProjects/pyBulletProject1/urdfs/"
    filename=os.path.join(dirName, fname)
    orn = p.getQuaternionFromEuler(euler)
    robot_id = p.loadURDF(filename, pos, orn, useFixedBase=False, globalScaling=scale)
    return robot_id

def start_logging():
    dirName = f'logs/logs_{datetime.datetime.now().strftime("%b_%d_%y_%I_%M_%S%p")}'
    os.mkdir(dirName)
    colourFrameDirName=f'{dirName}/colour_frames'
    os.mkdir(colourFrameDirName)
    depthFrameDirName = f'{dirName}/depth_frames'
    os.mkdir(depthFrameDirName)
    log_id=p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, f'{dirName}/stateLog.mp4')
    return log_id, dirName

def imsave(rgb, depth, t, dirName, image_width=224, image_height=224):
    rgb_im = np.reshape(rgb, (image_width, image_height, 4))
    depth_im = np.reshape(depth, (image_width, image_height))
    colourFrameDirName= f'{dirName}/colour_frames'
    depthFrameDirName = f'{dirName}/depth_frames'
    plt.imsave(f'{colourFrameDirName}/frame_{t}.png', rgb_im/255.)
    plt.imsave(f'{depthFrameDirName}/frame_{t}.png', depth_im, cmap='Greys') #

def new_imsave(rgb, depth, t, dirName, image_width=224, image_height=224): #TODO: Fix
    rgb_im = np.reshape(rgb, (image_width, image_height, 4))
    depth_im = np.reshape(depth, (image_width, image_height))
    depth_im=np.float32(depth_im)
    colourFrameDirName = f'{dirName}/colour_frames'
    depthFrameDirName = f'{dirName}/depth_frames'
    far=1.1
    near=0.7
    depth_scaled= far * near / (far - (far - near) * depth_im)
    plt.imsave(f'{colourFrameDirName}/frame_{t}.png', rgb_im / 255.)
    plt.imsave(f'{depthFrameDirName}/frame_{t}.png', depth_scaled, cmap='Greys')  #

# Math functions
def quatFromVector(direction):
    d = np.array(direction)
    if np.linalg.norm(d) == 0:
        r = np.eye(3)
        print('eek! zero norm!')
    else:
        d = d / np.linalg.norm(d)
        z = np.array([-d[1], d[0], 0])
        n = np.cross(d, z)
        r = np.vstack([d, z, n]).T
    return R.from_matrix(r).as_quat()

# skew symmetric matrix
def skew(vec):
  return np.array([[0,-vec[2],vec[1]],[vec[2],0,-vec[0]],[-vec[1],vec[0],0]])


# Camera stuff
def get_camera():
    CameraInfo = namedtuple('CameraInfo',
                            ['width', 'height', 'viewMatrix', 'projectionMatrix', 'cameraUp', 'cameraForward',
                             'horizontal', 'vertical', 'yaw', 'pitch', 'dist', 'target'])
    return CameraInfo(*p.getDebugVisualizerCamera(physicsClientId=p.GUI))

def default_projection_matrix(image_width=224, image_height=224, near = 0.02, far=0.5, fov=60):
# Camera projection matrix settings from ibvsTutorial
# intrinsics (I think)

    aspect = image_width / image_height

    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    return projection_matrix

def default_debug_camera(pos=[0,0,0], camera_distance=5):
# Set the camera position for GUI mode
    #camera_distance = 8
    camera_yaw = -180.0  # deg
    camera_pitch = -10  # deg
    camera_target_position = pos
    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

def camera_matrix(id, idx, debug=False, start_pos=[0,0,0]):
    camera_link_pose = p.getLinkState(id, idx, computeForwardKinematics=True)[0]
    # Orientation of camera. Get the camera's current orientation to compute up and forward vectors.
    camera_orn_quat = p.getLinkState(id, idx, computeForwardKinematics=True)[1]
    camera_orn_matrix = np.reshape(p.getMatrixFromQuaternion(camera_orn_quat), (3, 3))

    up_vec = np.array([0, 0, 1])
    forward_vec = np.array([-1, 0, 0])  # trial and error

    camera_up = camera_orn_matrix @ up_vec
    camera_forward = camera_orn_matrix @ forward_vec

    camera_target = camera_link_pose + np.array(camera_forward) * 0.01
    # line=p.addUserDebugLine(camera_link_pose, camera_target, lineColorRGB=[1,0,0])
    if debug:
        camera_target_position = [start_pos[0], start_pos[1], start_pos[2]]
    else:
        camera_target_position = [camera_target[0], camera_target[1], camera_target[2]]
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[camera_link_pose[0], camera_link_pose[1], camera_link_pose[2]],
        cameraTargetPosition = camera_target_position,
        cameraUpVector=camera_up)

    return view_matrix

#Information about the robot
def get_link_state(body, link, kinematics=True, velocity=True):
    LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                         'localInertialFramePosition', 'localInertialFrameOrientation',
                                         'worldLinkFramePosition', 'worldLinkFrameOrientation'])
    return LinkState(*p.getLinkState(body, link,
                                     computeForwardKinematics=True,
                                     #computeLinkVelocity=velocity,
                                     ))

CollisionInfo = namedtuple('CollisionInfo',
                           '''
                           contactFlag
                           bodyUniqueIdA
                           bodyUniqueIdB
                           linkIndexA
                           linkIndexB
                           positionOnA
                           positionOnB
                           contactNormalOnB
                           contactDistance
                           normalForce
                           lateralFriction1
                           lateralFrictionDir1
                           lateralFriction2
                           lateralFrictionDir2
                           '''.split())
def get_closest_points(body1, body2, link1=None, link2=None, max_distance=1000):

    if (link1 is None) and (link2 is None):
        results = p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance)
        #print(results)
    elif link2 is None:
        results = p.getClosestPoints(bodyA=body1, bodyB=body2, linkIndexA=link1,
                                     distance=max_distance)
    elif link1 is None:
        results = p.getClosestPoints(bodyA=body1, bodyB=body2, linkIndexB=link2,
                                     distance=max_distance)
    else:
        results = p.getClosestPoints(bodyA=body1, bodyB=body2, linkIndexA=link1, linkIndexB=link2, distance=max_distance)
    return [CollisionInfo(*info) for info in results]

