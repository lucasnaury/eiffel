from pyniryo2 import *
from pyniryo import uncompress_image, undistort_image, concat_imgs, show_img
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Challenge2(Node):
    def __init__(self, name="Challenge2"):
        super().__init__(name)

        # Init variables

        self._adr_IP= "10.89.5.18"
        self._robot = NiryoRobot(self._adr_IP)

        self._observation_pose = PoseObject(
            x=230.0/1000, y=0.0, z=270.0/1000,
            roll=-2.5, pitch=1.5, yaw=-1.0,
        )

        self._place_pose = PoseObject(
            x=249.2/1000, y=-248.9/1000 ,z=60.0,
            roll=2.623 ,pitch=1.44, yaw=2.654
        )

        self._ratio = 0.56 #mm/px
        self._basgauche = [235.2, 137.3, 30.5]

        self._R = 0
        self._G = 70
        self._B = 100 

        # TODO : ordre : rouge, jaune, bleu, vert, petits boulons, grands boulons, pas autre chose

        self._robot.arm.calibrate_auto()

        # Getting calibration param
        self._mtx, self._dist = self._robot.vision.get_camera_intrinsics()
        # Moving to observation pose
        self._robot.arm.move_pose(self._observation_pose)

        # Initialize publishers and subscribers
        # Initialize subscribers
        self.create_subscription(String, '/pibot', self.pibot_callback, 10)

        # Initialize a publisher
        self._string_publisher = self.create_publisher(String, '/ned_2', 10)

    def pibot_callback(self, msg):
        self._pibot_msg = msg
        self.loop()

    def loop(self):

        #TODO : boucle for sur l'ordre des pieces
        # Getting image
        img_compressed = self._robot.vision.get_img_compressed()
        # Uncompressing image
        img_raw = uncompress_image(img_compressed)
        # Undistorting
        img_undistort = undistort_image(img_raw, self._mtx, self._dist)

        image = cv2.cvtColor(img_undistort, cv2.COLOR_BGR2HSV)
        
        low = np.array([self._R, 80, 40])
        high = np.array([self._R+20, 255,255])

        mask = cv2.inRange(image, low, high)

        contours,_ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

        
        rect = cv2.minAreaRect(sorted_contours[0])
        (x, y), (width, height), angle  = rect
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        print(x, y)

        imgSave2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        imgSave = cv2.drawContours(imgSave2,[box],0,(0,0,255),2)

        cv2.imwrite('test.jpg', imgSave)

        if width > height :
            print("coucou")
            angle = angle-90

        angle = angle*np.pi/180
        

        position_y = x*self._ratio/1000
        position_y = -position_y+(130.2/1000) + self._basgauche[1]/1000
        position_x = ((640-y)*self._ratio + self._basgauche[0])/1000
        position_z = (self._basgauche[2]/1000)

        print(f'{position_x*1000=}, {position_y*1000=}, {position_z*1000=}, {angle=}')


        position_objet = PoseObject(
            x=position_x, y=position_y ,z=position_z,
            roll=-2.6, pitch=1.57, yaw=angle
        )
        
        self._robot.arm.move_pose(position_objet)
        self._robot.tool.grasp_with_tool()

        self._robot.arm.move_pose(self._place_pose)
        self._robot.tool.release_with_tool()

        message = String()
        self._string_publisher.publish(message)

def main():
    """
    Main loop
    """
    rclpy.init()
    rosNode = Challenge2()
    rosNode.loop()

    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
