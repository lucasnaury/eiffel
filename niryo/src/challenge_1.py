#!/usr/bin/env python3


# Imports
from pyniryo2 import *

# - Constants
workspace_name = "Plan_piece"  # Robot's Workspace Name
robot_ip_address = "10.89.5.18"

# The pose from where the image processing happens
observation_pose = PoseObject(
    x=20.0/1000, y=286.0/1000, z=302.0/1000,
    roll=2.95, pitch=1.32, yaw=-1.84
)
# Place pose
place_pose = PoseObject(
    x=-187.0/1000, y=-189.9/1000 ,z=93.5/1000,
    roll=0.0, pitch=1.57, yaw=-1.57
)

# - Initialization

# Connect to robot
robot = NiryoRobot(robot_ip_address)
# Calibrate robot if robot needs calibration
robot.arm.calibrate_auto()
# Updating tool
robot.tool.update_tool()

# --- -------------- --- #-----------------------------------------------------------------

# Initializing variables
offset_size = 0.05


#while catch_count < max_catch_count:
    # Moving to observation pose


ORDRE = [(ObjectShape.SQUARE, ObjectColor.RED, 0), (ObjectShape.SQUARE, ObjectColor.GREEN, 1), (ObjectShape.SQUARE, ObjectColor.BLUE, 2), (ObjectShape.CIRCLE, ObjectColor.BLUE, 2), (ObjectShape.CIRCLE, ObjectColor.GREEN, 1), (ObjectShape.CIRCLE, ObjectColor.RED, 0)]


# Trying to get object via Vision Pick

for objet in ORDRE:
    robot.arm.move_pose(observation_pose)
    obj_found, shape, color = robot.vision.vision_pick(workspace_name,shape=objet[0], color=objet[1])

    c1 = False
    while not c1:
        if not obj_found:
            robot.wait(0.1)
            obj_found, shape, color = robot.vision.vision_pick(workspace_name,shape=objet[0], color=objet[1])
            continue
        c1 = True
        z_offset = 0 if objet[0] == ObjectShape.SQUARE else 0.01
        next_place_pose = place_pose.copy_with_offsets(x_offset=objet[2] * offset_size, z_offset=z_offset)
        robot.pick_place.place_from_pose(next_place_pose)



# --- -------------- --- #-------------------------------------------------------------------


robot.end()

