from math import radians
from voraus_robot_arm import CartesianPose, JointPose, VorausIndustrialRobotArm, Percent
home = JointPose(0, -1.57, 1.57, -1.57, -1.57, 0)
pre_pick = JointPose().from_list(
    [radians(d) for d in [80.41, -82.18, 93.56, -97.89, -87.37, 0]]
)
if __name__ == "__main__":
    robot = VorausIndustrialRobotArm()
    with robot.connect("voraus-core", port=48401):
        robot._driver._objects.robot.commands.tool.set_tool_transformation_offset((0, 0, 0.103, 0, 0, 0))
        tool_control_output = robot.get_digital_output_v(2)
        robot.enable()
        input("Press <enter> to go back.")

        robot.move_ptp(home)
        tool_control_output.clear()
        while(robot.move_ptp(home).result() == False):
            print("Robot is moving to home position.")
  
