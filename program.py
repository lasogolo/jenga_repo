# pylint: disable=forgotten-debug-statement
"""Contains the program which controls the system."""

from math import radians

from asyncua import ua
from asyncua.sync import Client as UAClient
from voraus_robot_arm import CartesianPose, JointPose, RobotArm

HOME = JointPose(0, 0, 0, 0, 0, 0)
PRE_PICK_POSE = CartesianPose(0.7473, -0.8551, 0.0689, radians(-90), radians(45), radians(45))
PICK_POSE = CartesianPose(0.7473, -0.6404, 0.0689, radians(-90), radians(45), radians(45))
AFTER_PICK_POSE = CartesianPose(0.7473, -0.6404, 0.2689, radians(-90), radians(45), radians(45))
PRE_PLACE_POSE = CartesianPose(0.6473, 0.8115, 0.5893, radians(-90), radians(45), radians(45))
PLACE_POSE = CartesianPose(0.6473, 0.8115, 0.3943, radians(-90), radians(45), radians(45))
AFTER_PLACE_POSE = CartesianPose(0.6473, 0.6115, 0.3893, radians(-90), radians(45), radians(45))

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
            robot.move_ptp(PRE_PICK_POSE)
            breakpoint()
            robot.move_linear(PICK_POSE)
            control_node.call_method(
                set_do_node, ua.Variant(2, ua.VariantType.UInt32), ua.Variant(True, ua.VariantType.Boolean)
            )
            robot.move_linear(AFTER_PICK_POSE)
            breakpoint()
            robot.move_linear(PRE_PLACE_POSE)
            breakpoint()
            robot.move_linear(PLACE_POSE)
            control_node.call_method(
                set_do_node, ua.Variant(2, ua.VariantType.UInt32), ua.Variant(False, ua.VariantType.Boolean)
            )
            robot.move_linear(AFTER_PLACE_POSE)
            breakpoint()
            robot.move_ptp(HOME)
