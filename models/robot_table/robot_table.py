"""Describes a robot table simulation model."""

import sys
from math import radians
from pathlib import Path

from voraus_3d_visu import Visu
from voraus_simulation import Simulation, StaticObject

models = Path(__file__).parent


class RobotTable(StaticObject):
    """Defines a robot table simulation model."""

    def __init__(self, position: list[float] | None = None, rotation: list[float] | None = None) -> None:
        """Initializes a robot table simulation model.

        Args:
            position: The initial position. Defaults to None.
            rotation: The initial rotation. Defaults to None.
        """
        glb_path = models / "robot_table.glb"
        urdf_path = models / "robot_table.urdf"
        super().__init__(glb_path, urdf_path, position, rotation)


if __name__ == "__main__":
    sys.path.append(str(Path(__file__).parent.parent.parent))
    from models.pellet.pellet import Pellet

    simulation = Simulation(frequency=50, visualization=Visu("http://voraus-3d-visu/"))
    with simulation.run():
        StaticObject(glb_path=None, urdf_path=Path("plane_transparent.urdf"))

        robot_table = RobotTable(position=[0, 0, 0], rotation=[0, 0, 0])
        pellet = Pellet(position=[0.7213, 0, 0.9788], rotation=[radians(90), 0, radians(90)])

        FRAME = 0
        while FRAME < 100:
            simulation.step()
            simulation.sleep()
            FRAME += 1
