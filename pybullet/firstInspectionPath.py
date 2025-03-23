import pybullet as p
import pybullet_data
import numpy as np
import utils

physics_client = p.connect(p.GUI)
p.resetSimulation() # Reset the simulation space
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Add path to necessary data for pybullet
p.setGravity(0.0, 0.0, 0) # Set gravity to zero

# Load the floor
plane_id = p.loadURDF("plane.urdf")

#Start logging and define save frequency
logging = False
camera_debug = False

if logging:
    log_id, dirName = utils.start_logging()
    imsave_freq = 10

if camera_debug:
    yaw_slider = p.addUserDebugParameter("yaw", 0, 360, 180)
    pitch_slider = p.addUserDebugParameter("pitch", -90,90, 10)
    distance_slider = p.addUserDebugParameter("distance", 0.2, 3, 1)

# Sim parameters
z_sim = 7
step=.2
T=10
freq = 2*np.pi/T
full_period=T/step
radius = 1

# Load the satellite and the robot - they must be in the "urdfs" folder
SS_start_pos = [1, 1, z_sim]  # initial Space Station position
arm_start_pos = [radius, 0, z_sim] # initial arm position

SS_id = utils.load_floating_robot(SS_start_pos, [0,0,0], "ss1/ss1.urdf", scale=2)
arm_id = utils.load_floating_robot(arm_start_pos, [0,0,0], "arm2/arm2.urdf")

# Set up the camera and define image parameters
image_width=224
image_height=224
utils.default_debug_camera(SS_start_pos, camera_distance=2)
depth_debug = False
if depth_debug:
    fov_slider = p.addUserDebugParameter("fov", 20, 70, 50)
    far_slider = p.addUserDebugParameter("far", 0.3, 3, 0.5)
    near_slider = p.addUserDebugParameter("near", 0.04, 0.3, 0.04)

projection_matrix=utils.default_projection_matrix(near = 0.04, far = 0.8, fov = 50)

# Create the constraint
# TODO:  figure out how this will work in space station centric coordinates
CAMERA_IDX = 4
cid = p.createConstraint(arm_id,
                                CAMERA_IDX,
                                -1,
                                -1,
                                p.JOINT_FIXED,
                                [0, 0, 0],
                                [0, 0, 0],
                                arm_start_pos)


for t in range(1000):
    if depth_debug:
        fov = p.readUserDebugParameter(fov_slider)
        far = p.readUserDebugParameter(far_slider)
        near = p.readUserDebugParameter(near_slider)
    if camera_debug:
        yaw = p.readUserDebugParameter(yaw_slider)
        camera_pitch = p.readUserDebugParameter(pitch_slider)
        camera_distance = p.readUserDebugParameter(distance_slider)
        camera_target_position = SS_id
        p.resetDebugVisualizerCamera(camera_distance, yaw, camera_pitch, camera_target_position)


    #projection_matrix = utils.default_projection_matrix(near=near, far=far, fov=fov)
    view_matrix=utils.camera_matrix(arm_id, CAMERA_IDX)

    _, _, rgb_img, depth_img, _ = p.getCameraImage(
        width=image_width,
        height=image_height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER)

    if logging and t % imsave_freq == 0:
            utils.new_imsave(rgb_img, depth_img, t, dirName)


    # generate first attempt at covering trajectory - series of inclined circles, eccentricity increasing by pi/10 after each circle

    a = step*t

    n = t // full_period
    inc=n*np.pi/10

    p_des = np.array([radius * np.cos(freq * a), radius * np.sin(freq * a)*np.cos(inc), radius * np.sin(freq * a)*np.sin(inc)])+np.array(SS_start_pos)
    relative_pos=np.array(p_des)-np.array(SS_start_pos)

    orn_des=utils.quatFromVector(relative_pos)
    p.changeConstraint(cid, p_des, orn_des, maxForce=5000)
    p.stepSimulation()

if logging:
    p.stopStateLogging(log_id)
p.disconnect()