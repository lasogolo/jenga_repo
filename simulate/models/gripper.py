"""Contains a gripper model."""

from pathlib import Path

from voraus_simulation import DynamicObject, Pose, bullet_types, get_object, transforms

models = Path(__file__).parents[2] / "assets/gripper"


class Gripper(DynamicObject):
    """Describes a gripper model."""

    def __init__(
        self,
        position: list[float] | None = None,
        rotation: list[float] | None = None,
    ) -> None:
        """Initializes a gripper model.

        Args:
            position: The initial position. Defaults to None.
            rotation: The initial rotation. Defaults to None.
        """
        glb_path = models / "gripper-visu.glb"
        urdf_path = models / "gripper.urdf"
        super().__init__(glb_path, urdf_path, position, rotation)

        self.flange_constraint: bullet_types.Constraint | None = None
        self.grasp_constraint: bullet_types.Constraint | None = None

    def update(self, pose: Pose, grasping: bool = False) -> None:
        # Flange constraint
        if self.flange_constraint is None:
            self.flange_constraint = self.create_constraint(
                -1,
                parent_frame_position=[0, 0, 0],
                child_frame_position=pose.position,
                child_frame_orientation=pose.orientation,
            )
        else:
            self.flange_constraint.change(
                joint_child_pivot=pose.position,
                joint_child_frame_orientation=pose.orientation,
            )

        # Grasping constraint
        if grasping:
            contact_points = self.get_contact_points()

            if self.grasp_constraint is None and contact_points:
                contact_id = contact_points[0].body_unique_id_b
                obj = get_object(contact_id, DynamicObject)

                if obj is not None:
                    print("CONSTRAINT CREATE")
                    contact_pose = obj.get_pose()
                    tcp_pose = Pose(position=pose.position, orientation=pose.orientation)
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
