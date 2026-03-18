"""Contains the simulation which represents the real system.."""

from argparse import ArgumentParser
from math import radians
from pathlib import Path

from voraus_3d_visu import Visu
from voraus_simulation import Simulation, StaticObject

from models.fence.fence import Fence
from models.pellet.pellet import Pellet
from models.robot.robot import Robot
from models.robot_table.robot_table import RobotTable
from models.shelf.shelf import Shelf
from models.tcp.tcp import TCP

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--use-constraints", action="store_true", default=False)
    parser.add_argument("--max-frames", type=int, default=10000)
    args = parser.parse_args()

    simulation = Simulation(frequency=50, visualization=Visu("http://voraus-3d-visu/"))
    robot = Robot("opc.tcp://voraus-core:48401/", position=[0, 0, 0.71], rotation=[0, 0, 0])

    with simulation.run(), robot.connection():
        StaticObject(glb_path=None, urdf_path=Path("plane_transparent.urdf"))

        robot_table = RobotTable(position=[0, 0, 0], rotation=[0, 0, 0])
        fence = Fence(position=[-0.79, 0.47, 0], rotation=[0, 0, 0])
        shelf = Shelf(position=[0.325, 1.55, 0], rotation=[0, 0, 0])

        # pellet to pick
        pellet_1 = Pellet(position=[0.7473, -0.1348, 0.7798], rotation=[radians(90), 0, radians(90)])

        # pellets to decorate the simulation environment
        pellet_2 = Pellet(position=[2, -0.75, 0.3114], rotation=[0, 0, radians(-90)])
        pellet_3 = Pellet(position=[2.5, -0.75, 0.3114], rotation=[0, 0, radians(-90)])
        pellet_4 = Pellet(position=[2, 1.55, 0.3114], rotation=[0, 0, radians(-90)])
        pellet_5 = Pellet(position=[1.5, 1.55, 0.3114], rotation=[0, 0, radians(-90)])

        robot.get_robot_data()
        tcp = TCP(position=robot.robot_data.tcp_position, debug=False, use_constraints=args.use_constraints)

        FRAME = 0
        while FRAME < args.max_frames:
            robot.get_robot_data()
            grasp = robot.robot_data.digital_outputs[1]
            tcp.update(robot.robot_data.world_position, robot.robot_data.world_quaternion, grasping=grasp)
            simulation.step()
            simulation.sleep()
            FRAME += 1
