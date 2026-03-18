# pylint: disable=forgotten-debug-statement
"""Contains the program which controls the robot."""

from math import radians

from asyncua import ua
from asyncua.sync import Client as UAClient
from voraus_robot_arm import CartesianPose, JointPose, RobotArm

HOME = JointPose(0, 0, 0, 0, 0, 0)
FIRST_INTERMEDIATE_POSE = JointPose(0, 0, 0, 0, radians(-45), radians(-45))
PRE_PICK_POSE = CartesianPose(0.732, 0, 0.953, radians(180), radians(45), radians(45))
PICK_POSE = CartesianPose(0.732, 0, 0.753, radians(180), radians(45), radians(45))
AFTER_PICK_POSE = CartesianPose(0.732, 0, 0.953, radians(180), radians(45), radians(45))
SECOND_INTERMEDIATE_POSE = CartesianPose(0, 0.8226, 0.953, radians(180), radians(45), radians(45))
PRE_PLACE_POSE = CartesianPose(0, 0.8226, 0.253, radians(180), radians(45), radians(45))
PLACE_POSE = CartesianPose(0, 0.8226, 0.046, radians(180), radians(45), radians(45))
AFTER_PLACE_POSE = CartesianPose(0, 0.8226, 0.253, radians(180), radians(45), radians(45))


if __name__ == "__main__":
    robot = RobotArm()
    ua_client = UAClient("opc.tcp://voraus-core:48401/")
    with robot.connection("opc.tcp://voraus-core:48401/"):
        with ua_client:
            control_node = ua_client.get_node("ns=1;i=100240")
            set_do_node = ua_client.get_node("ns=1;i=100204")
            read_do_node = ua_client.get_node("ns=1;i=202202")
            timeoverride_node = ua_client.get_node("ns=1;i=100248")
            control_node.call_method(timeoverride_node, ua.Variant(1.0, ua.VariantType.Double))
            robot.move_ptp(HOME)
            breakpoint()
            robot.move_ptp(FIRST_INTERMEDIATE_POSE)
            robot.move_linear(PRE_PICK_POSE)
            breakpoint()
            robot.move_linear(PICK_POSE)
            control_node.call_method(
                set_do_node, ua.Variant(2, ua.VariantType.UInt32), ua.Variant(True, ua.VariantType.Boolean)
            )
            robot.move_linear(AFTER_PICK_POSE)
            breakpoint()
            robot.move_linear(SECOND_INTERMEDIATE_POSE)
            robot.move_linear(PRE_PLACE_POSE)
            breakpoint()
            robot.move_linear(PLACE_POSE)
            control_node.call_method(
                set_do_node, ua.Variant(2, ua.VariantType.UInt32), ua.Variant(False, ua.VariantType.Boolean)
            )
            robot.move_linear(AFTER_PLACE_POSE)
            breakpoint()
            robot.move_ptp(HOME)
