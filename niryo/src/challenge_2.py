from pyniryo2 import *
from pyniryo import uncompress_image, undistort_image, concat_imgs, show_img
import cv2
import numpy as np
adr_IP= "10.89.5.18" # adresse IP de votre robot, utiliser votre IP
robot = NiryoRobot(adr_IP)

observation_pose = PoseObject(
    x=0.0, y=230.0/1000, z=270.0/1000,
    roll=-2.5, pitch=1.5, yaw=-1.0,
)

# Connecting to robot
robot = NiryoRobot(adr_IP)
robot.arm.calibrate_auto()

# Getting calibration param
mtx, dist = robot.vision.get_camera_intrinsics()
# Moving to observation pose
robot.arm.move_pose(observation_pose)

ratio = 0.56 #mm/px

basgauche = [-141.4, 203.0, 29.7]


    



while "User do not press Escape neither Q":
    # Getting image
    img_compressed = robot.vision.get_img_compressed()
    # Uncompressing image
    img_raw = uncompress_image(img_compressed)
    # Undistorting
    img_undistort = undistort_image(img_raw, mtx, dist)

    # - Display
    # Concatenating raw image and undistorted image
    concat_ims = concat_imgs((img_raw, img_undistort))

    image = cv2.cvtColor(img_undistort, cv2.COLOR_BGR2HSV)
    R = 0
    G = 60
    B = 100
    low = np.array([R, 60, 40])
    high = np.array([R+40, 255,255])

    mask = cv2.inRange(image, low, high)

    contours,_ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    (x, y), radius = cv2.minEnclosingCircle(sorted_contours[0])
    

    position_x = ((640-x)*ratio + basgauche[0])/1000
    position_y = ((480-y)*ratio + basgauche[1])/1000
    position_z = (basgauche[2]/1000) +0.1

    print(f'{position_x*1000=}, {position_y*1000=}, {position_z*1000=}')


    position_objet = PoseObject(
        x=position_x, y=position_y ,z=position_z,
        roll=-2.6, pitch=1.57, yaw=-1.57
    )
    robot.arm.move_pose(position_objet)

    cv2.imshow("Camera", mask)

    # Showing images
    key = show_img("Images raw & undistorted", concat_ims, wait_ms=30)
    if key in [27, ord("q")]:  # Will break loop if the user press Escape or Q
        break


