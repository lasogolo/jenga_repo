"""Describes a fence simulation model."""

import sys
from math import radians
from pathlib import Path

from voraus_3d_visu import Visu
from voraus_simulation import Simulation, StaticObject

models = Path(__file__).parent


class Fence(StaticObject):
    """Defines a fence simulation model."""

    def __init__(self, position: list[float] | None = None, rotation: list[float] | None = None) -> None:
        """Initializes a fence simulation model.

        Args:
            position: The initial position. Defaults to None.
            rotation: The initial rotation. Defaults to None.
        """
        glb_path = models / "fence.glb"
        urdf_path = models / "fence.urdf"
        super().__init__(glb_path, urdf_path, position, rotation)


if __name__ == "__main__":
    sys.path.append(str(Path(__file__).parent.parent.parent))
    from models.pellet.pellet import Pellet

    simulation = Simulation(frequency=50, visualization=Visu("http://voraus-3d-visu/"))
    with simulation.run():
        StaticObject(glb_path=None, urdf_path=Path("plane_transparent.urdf"))

        fence = Fence(position=[0, 0, 0], rotation=[0, 0, 0])

        pellet_1 = Pellet(position=[3.4172, -1.3774, 0.3114], rotation=[0, 0, radians(-90)])
        pellet_2 = Pellet(position=[3.7776, -1.3774, 0.3114], rotation=[0, 0, radians(-90)])
        pellet_3 = Pellet(position=[2.7490, 1.1069, 0.3114], rotation=[0, 0, radians(-90)])
        pellet_4 = Pellet(position=[2.3886, 1.1069, 0.3114], rotation=[0, 0, radians(-90)])

        FRAME = 0
        while FRAME < 100:
            simulation.step()
            simulation.sleep()
            FRAME += 1
