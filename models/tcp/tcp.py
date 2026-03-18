"""Contains a tool center point (TCP) model."""

import sys
from argparse import ArgumentParser
from pathlib import Path

from voraus_3d_visu import Visu
from voraus_simulation import DynamicObject, Pose, Simulation, StaticObject, bullet_types, get_object, transforms

models = Path(__file__).parent


class TCP(DynamicObject):
    """Describes a tool center point (TCP) model."""

    def __init__(
        self,
        position: list[float] | None = None,
        rotation: list[float] | None = None,
        debug: bool = False,
        use_constraints: bool = True,
    ) -> None:
        """Initializes a tool center point (TCP) model.

        Args:
            position: The initial position. Defaults to None.
            rotation: The initial rotation. Defaults to None.
            debug: Show a debug model in the visu. Defaults to False.
            use_constraints: Use constraints if True, else updates the position without physics. Defaults to True.
        """
        self.debug = debug
        self.use_constraints = use_constraints

        glb_path: Path | None = models / "tcp.glb"
        urdf_path = models / "tcp.urdf"

        super().__init__(glb_path, urdf_path, position, rotation)

        self.tcp_constraint: bullet_types.Constraint | None = None
        self.grasp_constraint: bullet_types.Constraint | None = None
        self.grasp_instant: tuple[Pose, DynamicObject] | None = None

    def update_constraint(self, position: list[float], orientation: list[float], grasping: bool = False) -> None:
        """Updates the grasping constraint.

        Args:
            position: The new position of the TCP.
            orientation: The new orientation of the TCP.
            grasping: True, if robot is grasping, else False. Defaults to False.
        """
        # TCP constraint
        if self.tcp_constraint is None:
            self.tcp_constraint = self.create_constraint(
                -1,
                parent_frame_position=[0, 0, 0],
                child_frame_position=position,
                child_frame_orientation=orientation,
            )
        else:
            self.tcp_constraint.change(
                joint_child_pivot=position,
                joint_child_frame_orientation=orientation,
            )

        # grasping constraint
        if grasping:
            contact_points = self.get_contact_points()

            if self.grasp_constraint is None and contact_points:
                contact_id = contact_points[0].body_unique_id_b
                obj = get_object(contact_id, DynamicObject)

                if obj is not None:
                    print("CONSTRAINT CREATE")
                    contact_pose = obj.get_pose()
                    tcp_pose = Pose(position=position, orientation=orientation)
                    tcp_pose_inv = transforms.invert(tcp_pose)
                    restraint_pose = transforms.multiply(tcp_pose_inv, contact_pose)
                    self.grasp_constraint = self.create_constraint(
                        child=contact_id,
                        parent_frame_position=restraint_pose.position,
                        child_frame_position=[0, 0, 0],
                        parent_frame_orientation=restraint_pose.orientation,
                    )

        else:
            if self.grasp_constraint is not None:
                print("CONSTRAINT REMOVE")
                self.grasp_constraint.remove()
                self.grasp_constraint = None

    def update_instant(self, position: list[float], quaternion: list[float], grasping: bool = False) -> None:
        """Updates the TCP pose instantly.

        Args:
            position: The new TCP position.
            quaternion: The new TCP orientation.
            grasping: True, if robot is grasping, else False. Defaults to False.
        """
        # TCP pose
        pose = Pose(position=position, orientation=quaternion)
        self.set_pose(pose)
        # grasping pose
        if grasping:
            contact_points = self.get_contact_points()

            if self.grasp_instant is None and contact_points:
                contact_id = contact_points[0].body_unique_id_b
                obj = get_object(contact_id, DynamicObject)

                if obj is not None:
                    print("INSTANT CREATE")
                    inverted = transforms.invert(pose)
                    offset = transforms.multiply(inverted, obj.get_pose())
                    self.grasp_instant = (offset, obj)

            if self.grasp_instant is not None:
                offset, obj = self.grasp_instant
                grasp_pose = transforms.multiply(pose, offset)
                obj.set_pose(grasp_pose)

        else:
            if self.grasp_instant is not None:
                print("INSTANT REMOVE")
                self.grasp_instant = None

    def update(self, position: list[float], quaternion: list[float], grasping: bool = False) -> None:
        """Updates the TCP pose.

        Args:
            position: The new TCP position.
            quaternion: The new TCP orientation.
            grasping: True, if robot is grasping, else False. Defaults to False.
        """
        if self.use_constraints:
            self.update_constraint(position, quaternion, grasping=grasping)
        else:
            self.update_instant(position, quaternion, grasping=grasping)


if __name__ == "__main__":
    sys.path.append(str(Path(__file__).parent.parent.parent))
    from models.pellet.pellet import Pellet
    from models.robot.robot import Robot
    from models.robot_table.robot_table import RobotTable

    parser = ArgumentParser()
    parser.add_argument("--use-constraints", action="store_true", default=False)
    parser.add_argument("--max-frames", type=int, default=10000)
    args = parser.parse_args()

    simulation = Simulation(frequency=50, visualization=Visu("http://voraus-3d-visu/"))
    robot = Robot("opc.tcp://voraus-core:48401/", position=[0, 0, 0.71], rotation=[0, 0, 0])

    with simulation.run(), robot.connection():
        StaticObject(glb_path=None, urdf_path=Path("plane_transparent.urdf"))

        robot_table = RobotTable(position=[0, 0, 0], rotation=[0, 0, 0])
        pellet = Pellet(position=[0.7320, 0, 1.0162], rotation=[0, 0, 0])

        robot.get_robot_data()
        tcp = TCP(position=robot.robot_data.world_position, debug=False, use_constraints=args.use_constraints)

        FRAME = 0
        while FRAME < args.max_frames:
            robot.get_robot_data()
            grasp = robot.robot_data.digital_outputs[1]
            tcp.update(robot.robot_data.world_position, robot.robot_data.world_quaternion, grasping=grasp)
            simulation.step()
            simulation.sleep()
            FRAME += 1
