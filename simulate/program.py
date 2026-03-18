# pylint: disable=protected-access
"""Contains the program which controls the robot."""

from math import radians
from voraus_robot_arm import CartesianPose, JointPose, VorausIndustrialRobotArm, Percent

home = JointPose(0, -1.57, 1.57, -1.57, -1.57, 0) # No se, lo cambie y no altero notoriamente el Bewegung

# scan = JointPose(radians(80.61),-radians(49.12), radians(21.57), -radians(81.46), -radians(90.71), radians(133.04))
# pre_pre_pick = JointPose(radians(70.95), -radians(49.95), radians(7.36), -radians(56.94),-radians(93.90), radians(137.29))
# pre_pick = CartesianPose(0.2635, 0.7652, 0.45, -radians(179.95), -radians(0.71), -radians(123.64))
# pick = CartesianPose(0.2635, 0.7652, 0.4, -radians(179.95), -radians(0.71), -radians(123.64))

pre_pick = JointPose().from_list(
    [radians(d) for d in [97.56, -64.87, 57.12, -81.24, -87.37, 0]]
)
pre_pick_i = JointPose().from_list(
    [radians(d) for d in [84.09, -82.30, 95.40, -100.78, -87.37, 0]]
)

pick_i = JointPose().from_list(
    [radians(d) for d in [84.09, -79.74, 102.03, -109.97, -87.37, 0]]
)

pick_ic = CartesianPose(0.2097, 0.5707, 0.211, radians(177.42), radians(2.38), radians(5.91))

pre_place_i = JointPose().from_list(
    [radians(d) for d in [142.23, -93.62, 111.79, -109.74, -87.16, -60.40]]
)

place_i = JointPose().from_list(
    [radians(d) for d in [142.23, -91.80, 115.59, -115.36, -87.16, -60.40]]
)

place_ic = CartesianPose(-0.3093, 0.4304, 0.2165, radians(176.79), radians(0.50), radians(8.19)) 

if __name__ == "__main__":
    robot = VorausIndustrialRobotArm()

    with robot.connect("voraus-core", port=48401):
        robot._driver._objects.robot.commands.tool.set_tool_transformation_offset((0, 0, 0.103, 0, 0, 0))
        tool_control_output = robot.get_digital_output_v(2)
        robot.enable()

        input("Press <enter> to pick.")
        robot.move_ptp(pre_pick, blending=Percent(50))
        robot.move_ptp(pre_pick_i, blending=Percent(50))
        robot.move_linear(pick_ic).result()
        tool_control_output.set()
        robot.move_ptp(pre_pick).result()

        input("Press <enter> to place.")
        robot.move_ptp(pre_place_i, blending= Percent(50))
        robot.move_linear(place_ic).result()
        input("Press <enter> to leave.")
        tool_control_output.clear()
        robot.move_ptp(pre_place_i)

        input("Press <enter> to go back.")
        robot.move_ptp(home)
        while(robot.move_ptp(home).result() == False):
            print("Robot is moving to home position.")
  
