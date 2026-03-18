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
    #dx_=0; dy_=0
    tol = 0.041
    with simulation.run(), robot.connection():
        StaticObject(glb_file=None, urdf_path=Path("plane_transparent.urdf"))
        x=-0.45
        gripper = Gripper()
        
        table = table(position=[0, 0.60, 0.04], rotation=[0, 0, radians(90)])
        zeug0 = zeug(position=[0.4, 0.4, 0.52]); zeug1 = zeug(position=[0.4, 0.6, 0.52]); zeug2 = zeug(position=[0.4, 0.8, 0.52])
        zeug0 = zeug(position=[0.3, 0.4, 0.52]); zeug1 = zeug(position=[0.3, 0.6, 0.52]); zeug2 = zeug(position=[0.3, 0.8, 0.52])
        zeug0 = zeug(position=[0.2, 0.4, 0.52]); zeug1 = zeug(position=[0.2, 0.6, 0.52]); zeug2 = zeug(position=[0.2, 0.8, 0.52])
        dr = 0 ; zg = 0
        for k in range(2):
            
            for i in range(3):
                xg = 0; yg = -tol
                for j in range(3):
                    if ((i%2 ==0 and k%2==0) or (i%2 !=0 and k!=0)) :
                        yg = 0
                    else:
                        xg = -tol#*2
                    zeug_dict[f"zeug_{i}_{j}"] = zeug([-0.3093+xg, 0.4304+yg, 0.51+zg], rotation=[0, 0, radians(dr)])
                    if ((i%2 ==0 and k%2==0) or (i%2 !=0 and k!=0)) :
                        xg = xg - tol #si me cambias aqui, cambiame en program, kchon!
                    yg = yg + tol
                zg += 0.0255
                dr += 90

        while True:
            robot.get_robot_data()

            grasp = robot.robot_data.digital_outputs[1]
            gripper.update(robot.robot_data.world_flange_pose, grasping=grasp)

            simulation.step()
            simulation.sleep()
