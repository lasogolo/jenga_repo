"""Contains a box simulation model."""

from pathlib import Path

from voraus_simulation import StaticObject

models = Path(__file__).parents[2] / "assets/table"


class table(StaticObject):
    """Defines a table simulation model."""

    def __init__(self, position: list[float] | None = None, rotation: list[float] | None = None) -> None:
        """Initializes a table simulation model.

        Args:
            position: The initial position of the table. Defaults to None.
            rotation:  The initial rotation of the table. Defaults to None.
        """

        print(models)
        glb_path = models / "table_long.glb"
        urdf_path = models / "table_long.urdf"
        super().__init__(glb_path, urdf_path, position, rotation)
