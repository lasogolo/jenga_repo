"""Contains the simulation logic, which represents the real system.."""

import sys
from math import radians
from pathlib import Path

sys.path.append(str(Path(__file__).parents[1]))
if __name__ == "__main__":

    from simulate.models import zeug, Gripper, Robot, table
    from voraus_3d_visu import Visu
    from voraus_simulation import Simulation, StaticObject

    simulation = Simulation(frequency=50, visualization=Visu("http://voraus-3d-visu/", clear_all=True))
    robot = Robot("opc.tcp://voraus-core:48401/", [0, 0, 0.3], rotation=[0, 0, radians(0)])
    zeug_dict = {}
    with simulation.run(), robot.connection():
        StaticObject(glb_file=None, urdf_path=Path("plane_transparent.urdf"))
        x=-0.45
        gripper = Gripper()
        table = table(position=[0, 0.60, 0.04], rotation=[0, 0, radians(90)])
        #zeug0 = zeug(position=[0.4, 0.4, 0.5])
        xg = 0
        for i in range(7):
            yg = 0
            for j in range(3):
                zeug_dict[f"zeug_{i}_{j}"] = zeug(position=[0.4+xg, 0.45+yg, 0.51])
                yg = yg + 0.16 #si me cambias aqui, cambiame en program, kchon!
            xg = xg - 0.07
        while True:
            robot.get_robot_data()

            grasp = robot.robot_data.digital_outputs[1]
            gripper.update(robot.robot_data.world_flange_pose, grasping=grasp)

            simulation.step()
            simulation.sleep()
