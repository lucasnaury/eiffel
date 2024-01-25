#!/usr/bin/env python3


# Imports
from pyniryo2 import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Challenge1(Node):
    def __init__(self, name="Challenge1"):
        super().__init__(name)
        # Constants
        self._workspace_name = "Plan_piece"  # Robot's Workspace Name
        self._robot_ip_address = "10.89.5.18"
        self._offset_size = 0.05
        self._ORDRE = [
            (ObjectShape.SQUARE, ObjectColor.RED, 0), 
            (ObjectShape.SQUARE, ObjectColor.GREEN, 1), 
            (ObjectShape.SQUARE, ObjectColor.BLUE, 2), 
            (ObjectShape.CIRCLE, ObjectColor.BLUE, 2), 
            (ObjectShape.CIRCLE, ObjectColor.GREEN, 1), 
            (ObjectShape.CIRCLE, ObjectColor.RED, 0)
        ]

        # The pose from where the image processing happens
        self._observation_pose = PoseObject(
            x=20.0/1000, y=286.0/1000, z=302.0/1000,
            roll=2.95, pitch=1.32, yaw=-1.84
        )
        # Place pose
        self._place_pose = PoseObject(
            x=-187.0/1000, y=-189.9/1000 ,z=93.5/1000,
            roll=0.0, pitch=1.57, yaw=-1.57
        )

        # Initialization
        self._robot = NiryoRobot(self._robot_ip_address)
        self._robot.arm.calibrate_auto()
        self._robot.tool.update_tool()

        # Initialize subscribers
        self.create_subscription(Bool, '/pibot', self.pibot_callback, 10)

        # Initialize a publisher
        self._marker_publisher = self.create_publisher(Bool, '/ned_2', 10)

        # Timer for the publisher
        self.create_timer(1/60, self.loop)
    
    def pibot_callback(self, msg):
        self._pibot_msg = msg

    def loop(self):
        # if not published or not at the right position
        if self._pibot_msg is None or not self._pibot_msg:
            return
        
        for objet in self._ORDRE:
            self._robot.arm.move_pose(self._observation_pose)
            obj_found, shape, color = self._robot.vision.vision_pick(self._workspace_name,shape=objet[0], color=objet[1])

            c1 = False
            while not c1:
                if not obj_found:
                    self._robot.wait(0.1)
                    obj_found, shape, color = self._robot.vision.vision_pick(self._workspace_name,shape=objet[0], color=objet[1])
                    continue
                c1 = True
                z_offset = 0 if objet[0] == ObjectShape.SQUARE else 0.01
                next_place_pose = self._place_pose.copy_with_offsets(x_offset=objet[2] * self._offset_size, z_offset=z_offset)
                self._robot.pick_place.place_from_pose(next_place_pose)
        self._robot.end()

def main():
    """
    Main loop
    """
    rclpy.init()
    rosNode = Challenge1()
    rclpy.spin(rosNode)

    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
