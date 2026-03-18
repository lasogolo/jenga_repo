"""Moves the robot."""

from math import radians

from asyncua import ua
from asyncua.sync import Client as UAClient
from voraus_robot_arm import JointPose, RobotArm

HOME = JointPose(0, 0, 0, 0, 0, 0)
POSE_1 = JointPose(0, 0, 0, 0, radians(-90), 0)
POSE_2 = JointPose(radians(45), 0, 0, 0, radians(-90), 0)

if __name__ == "__main__":
    robot = RobotArm()
    ua_client = UAClient("opc.tcp://voraus-core:48401/")
    with robot.connection("opc.tcp://voraus-core:48401/"):
        with ua_client:
            control_node = ua_client.get_node("ns=1;i=100240")
            timeoverride_node = ua_client.get_node("ns=1;i=100248")
            control_node.call_method(timeoverride_node, ua.Variant(1.0, ua.VariantType.Double))
            robot.move_ptp(HOME)
            input("Press <enter> to move the robot.")
            robot.move_ptp(POSE_1)
            input("Press <enter> to move the robot again.")
            robot.move_ptp(POSE_2)
