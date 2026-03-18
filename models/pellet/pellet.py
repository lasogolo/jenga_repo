"""Contains a pellet simulation model."""

from math import radians
from pathlib import Path

from voraus_3d_visu import Visu
from voraus_simulation import BulletEngine, DynamicObject, Simulation, StaticObject

models = Path(__file__).parent


class Pellet(DynamicObject):
    """Defines a pellet simulation model."""

    def __init__(self, position: list[float] | None = None, rotation: list[float] | None = None) -> None:
        """Initializes a pellet simulation model.

        Args:
            position: The initial position. Defaults to None.
            rotation:  The initial rotation. Defaults to None.
        """
        glb_path = models / "pellet.glb"
        urdf_path = models / "pellet.urdf"
        super().__init__(glb_path, urdf_path, position, rotation)


if __name__ == "__main__":
    simulation = Simulation(frequency=50, visualization=Visu("http://voraus-3d-visu/"), engine=BulletEngine("direct"))
    with simulation.run():
        StaticObject(glb_path=None, urdf_path=Path("plane_transparent.urdf"))

        pellet_1 = Pellet(position=[0, 0, 0.3082], rotation=[0, 0, radians(90)])
        pellet_2 = Pellet(position=[2, 0, 0.3082], rotation=[0, 0, 0])

        FRAME = 0
        while True:
            if FRAME == 100:
                input("Press <enter> to apply force.")
            elif 100 <= FRAME < 125:
                pellet_2.apply_external_force([-80, 0, 0])
            elif FRAME == 125:
                input("Press <enter> to continue.")
            elif FRAME > 300:
                break

            simulation.step()
            simulation.sleep()
            FRAME += 1
