#!/usr/bin/env python3


# Imports
from pyniryo2 import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Challenge1(Node):
    def __init__(self, name="Challenge1"):
        super().__init__(name)
        # Constants
        self._workspace_name = "Plan_piece"  # Robot's Workspace Name
        self._robot_ip_address = "10.89.5.18"
        self._offset_size = 0.05
        self._ORDRE = [
            [ObjectShape.SQUARE, ObjectColor.RED, 0], 
            [ObjectShape.SQUARE, ObjectColor.GREEN, 1], 
            [ObjectShape.SQUARE, ObjectColor.BLUE, 2], 
            [ObjectShape.CIRCLE, ObjectColor.BLUE, 2], 
            [ObjectShape.CIRCLE, ObjectColor.GREEN, 1], 
            [ObjectShape.CIRCLE, ObjectColor.RED, 0]
        ]

        self._pibot_msg = None

        # The pose from where the image processing happens
        self._observation_pose = PoseObject(
            x=183.0/1000, y=2.6/1000, z=258.7/1000,
            roll=-0.337, pitch=1.5, yaw=-0.3
        )
        # Place pose
        place_pose1 = PoseObject(
            x=249.2/1000, y=-248.9/1000 ,z=-66.6/1000,
            roll=2.623 ,pitch=1.44, yaw=2.654
        )
        place_pose2 = PoseObject(
            x=186.6/1000, y=-245.5/1000 ,z=-68.1/1000,
            roll=-2.88, pitch=1.45, yaw=2.488
        )
        place_pose3= PoseObject(
            x=179.2/1000, y=-298.8/1000 ,z=-68.4/1000,
            roll=-2.82, pitch=1.4, yaw=-2.73
        )

        self._intermediate_pose = PoseObject(
            x=225.1/1000, y=-233.0/1000, z=120.0/1000,
            roll=-0.4, pitch=1.2, yaw=-0.96
        )

        self._poses = [place_pose1, place_pose2, place_pose3]

        # Initialization
        self._robot = NiryoRobot(self._robot_ip_address)
        self._robot.arm.calibrate_auto()
        self._robot.tool.update_tool()

        # Initialize subscribers
        self.create_subscription(String, '/pibot', self.pibot_callback, 10)

        # Initialize a publisher
        self._string_publisher = self.create_publisher(String, '/ned_2', 10)

    
    def pibot_callback(self, msg):
        self._pibot_msg = msg

    def loop(self):        
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
                self._robot.arm.move_pose(self._intermediate_pose)

                z_offset = 0 if objet[0] == ObjectShape.SQUARE else 0.01
                place_pose = self._poses[objet[2]]
                next_place_pose = place_pose.copy_with_offsets(z_offset=z_offset)
                self._robot.pick_place.place_from_pose(next_place_pose)
                self._robot.arm.move_pose(self._intermediate_pose)
        self._robot.end()

        message = String()
        self._string_publisher.publish(message)

def main():
    """
    Main loop
    """
    rclpy.init()
    rosNode = Challenge1()
    
    rosNode.loop()
    rclpy.spin(rosNode)

    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
