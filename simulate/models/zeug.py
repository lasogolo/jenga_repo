"""Contains a box simulation model."""

from pathlib import Path

from voraus_simulation import DynamicObject

models = Path(__file__).parents[2] / "assets/jenga"


class zeug(DynamicObject):
    """Defines a box simulation model."""

    def __init__(self, position: list[float] | None = None, rotation: list[float] | None = None) -> None:
        """Initializes a box simulation model.

        Args:
            position: The initial position of the box. Defaults to None.
            rotation:  The initial rotation of the box. Defaults to None.
        """
        print(models)
        glb_path = models / "zeug-visualization.glb"
        urdf_path = models / "jenga.urdf"
        super().__init__(glb_path, urdf_path, position, rotation)
