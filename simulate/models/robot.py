"""Describes a robot visualization model."""

import os
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Generator

from asyncua import ua
from asyncua.sync import Client as UAClient
from voraus_simulation import Pose, traits, transforms

models = Path(__file__).parents[2] / "assets"

VORAUS_CORE_URL = os.getenv("VORAUS_CORE_URL", default="http://voraus-core")
GLB_URL = f"{VORAUS_CORE_URL}/robots/VORAUS_INDUSTRIAL_ROBOT/VORAUS_INDUSTRIAL_ROBOT_002.glb"


@dataclass
class RobotData:
    """Describes the robot data."""

    joint_positions: list[float]
    world_flange_pose: Pose
    digital_outputs: list[bool]


class RobotClient:
    """Describes a robot client."""

    def __init__(self, client: UAClient) -> None:
        """Initializes a robot client.

        Args:
            client: The OPC UA client to use.
        """
        self.client = client
        self.joint_positions = self.client.get_node("ns=1;i=100111")
        self.flange_pose = self.client.get_node("ns=1;i=100719")
        self.tcp_quaternion = self.client.get_node("ns=1;i=100710")
        self.digital_outputs = self.client.get_node("ns=1;i=202202")
        self.commands = self.client.get_node("ns=1;i=100240")
        self.set_digital_output = self.client.get_node("ns=1;i=100204")


class Cube(traits.VisuTraits):
    """Defines a cube visualization model."""

    def __init__(self, position: list[float], orientation: list[float]) -> None:
        """Initializes a cube visualization model.

        Args:
            position: The initial position.
            orientation: The initial orientation.
        """
        super().__init__()

        glb_path = models / "cube/cube.glb"
        self.load_glb(glb_path, Pose(position=position, orientation=orientation))


class Robot(traits.VisuTraits):
    """Defines a robot visualization model."""

    def __init__(self, url: str, position: list[float] | None = None, rotation: list[float] | None = None) -> None:
        """Initializes a robot visualization model.

        Args:
            url: The OPC UA URL.
            position: The initial position. Defaults to None.
            rotation: The initial rotation. Defaults to None.
        """
        super().__init__()

        _position = position or [0, 0, 0]
        _rotation = rotation or [0, 0, 0]

        quaternion = transforms.quaternion_from_euler(_rotation)
        self.load_glb(GLB_URL, Pose(position=_position, orientation=quaternion))

        self.cube = Cube(position=_position, orientation=quaternion)
        self.origin = Pose(position=_position, orientation=quaternion)

        self._robot_data: RobotData | None = None
        self._ua_client = UAClient(url)
        self._robot_client: RobotClient | None = None

    @property
    def robot_data(self) -> RobotData:
        """Makes sure robot data is not None.

        Returns:
            The robot data.
        """
        assert self._robot_data is not None, "Robot data not updated."
        return self._robot_data

    @property
    def robot_client(self) -> RobotClient:
        """Makes sure robot data is not None.

        Returns:
            The robot client.
        """
        assert self._robot_client is not None, "Robot client not connected."
        return self._robot_client

    @contextmanager
    def connection(self) -> Generator[None, None, None]:
        """A robot connection context manager.

        Yields:
            None
        """
        with self._ua_client:
            self._robot_client = RobotClient(self._ua_client)
            self.get_robot_data()
            yield

    def get_robot_data(self) -> RobotData:
        """Reads the robot data via OPC UA.

        Returns:
            The robot data.
        """
        joint_positions: list[float]
        flange_pose: list[float]
        wxyz_quaternion: list[float]
        digital_outputs: list[bool]
        joint_positions, flange_pose, wxyz_quaternion, digital_outputs = self.robot_client.client.read_values(
            [
                self.robot_client.joint_positions,
                self.robot_client.flange_pose,
                self.robot_client.tcp_quaternion,
                self.robot_client.digital_outputs,
            ]
        )
        w, x, y, z = wxyz_quaternion
        xyzw_quaternion = [x, y, z, w]

        world_flange_pose = transforms.multiply(self.origin, Pose(flange_pose[:3], xyzw_quaternion))

        self._robot_data = RobotData(
            joint_positions=joint_positions,
            world_flange_pose=world_flange_pose,
            digital_outputs=digital_outputs,
        )

        return self.robot_data

    def set_digital_output(self, pin: int, value: bool) -> None:
        """Sets a digital output.

        Args:
            pin: The pin number of the output. The first pin is 1.
            value: The value to set.
        """
        self.robot_client.commands.call_method(
            self.robot_client.set_digital_output,
            ua.Variant(pin, ua.VariantType.UInt32),
            ua.Variant(value, ua.VariantType.Boolean),
        )

    def get_visu_data(self) -> list:
        """Returns the visualization data for the robot joint positions.

        Returns:
            The updates for the robot joint positions.
        """
        return [
            self.visu_object.child("CS0").rotation.z(self.robot_data.joint_positions[0]),
            self.visu_object.child("CS1").rotation.z(self.robot_data.joint_positions[1]),
            self.visu_object.child("CS2").rotation.z(self.robot_data.joint_positions[2]),
            self.visu_object.child("CS3").rotation.z(self.robot_data.joint_positions[3]),
            self.visu_object.child("CS4").rotation.z(self.robot_data.joint_positions[4]),
            self.visu_object.child("CS5").rotation.z(self.robot_data.joint_positions[5]),
        ]
