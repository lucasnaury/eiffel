from pyniryo2 import *
from pyniryo import uncompress_image, undistort_image, concat_imgs, show_img
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

class Challenge2(Node):
    def __init__(self, name):
        super().__init__(name)

        # Init variables

        self._adr_IP= "10.89.5.18"
        self._robot = NiryoRobot(self._adr_IP)

        self._observation_pose = PoseObject(
            x=0.0, y=230.0/1000, z=270.0/1000,
            roll=-2.5, pitch=1.5, yaw=-1.0,
        )

        self._ratio = 0.56 #mm/px
        self._basgauche = [-141.4, 203.0, 29.7]

        self._R = 0
        self._G = 70
        self._B = 100 

        self._robot.arm.calibrate_auto()

        # Getting calibration param
        self._mtx, self._dist = self._robot.vision.get_camera_intrinsics()
        # Moving to observation pose
        self._robot.arm.move_pose(self._observation_pose)

        # Initialize publishers and subscribers
        # TODO

    def loop(self):
        # Getting image
        img_compressed = self._robot.vision.get_img_compressed()
        # Uncompressing image
        img_raw = uncompress_image(img_compressed)
        # Undistorting
        img_undistort = undistort_image(img_raw, self._mtx, self._dist)

        image = cv2.cvtColor(img_undistort, cv2.COLOR_BGR2HSV)
        
        low = np.array([R, 80, 40])
        high = np.array([R+20, 255,255])

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
        
        self._robot.arm.move_pose(position_objet)

def main():
    """
    Main loop
    """
    rclpy.init()
    rosNode = Challenge2()
    rclpy.spin(rosNode)

    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
