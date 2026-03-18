# pylint: disable=protected-access
"""Contains the program which controls the robot."""

import time
from time import sleep
import logging
from math import radians
from voraus_robot_arm import CartesianPose, JointPose, VorausIndustrialRobotArm, Percent, configure_logging, MoveCartesianVictorParameters

custom_acel_low = 5
custom_parameters = MoveCartesianVictorParameters(
        acceleration_mps2=custom_acel_low)
 
_logger = logging.getLogger()
home = JointPose(0, -1.57, 1.57, -1.57, -1.57, 0) # No se, lo cambie y no altero notoriamente el Bewegung


pre_pick_vec=[] ; pre_pick_ic_vec=[] ; pick_ic_vec=[]
pre_place_ic_vec=[] ; place_ic_vec=[]
dr=0 ; dz=0 ; dx_place=0
for k in range(2):
    dy=0
    for i in range(3):
        dx=-k*3.5*0.08
        dx_= 0
        dy_= -0.035#-0.039

        for j in range (3):
            if ( (i%2 ==0 and k%2==0) or (i%2 !=0 and k!=0)) :
                dy_ = 0  
            else:
                dx_ = -0.037#*2 

            pre_pick =  JointPose().from_list([radians(d) for d in [80.41, -82.18, 93.56, -97.89, -87.37, 0]])
            pre_pick_k = CartesianPose(0.2456+dx_place, 0.5543, 0.2787, radians(176.12), radians(2.01), radians(9.58))
            pre_pick_i = CartesianPose(0.3964+dx, 0.4487+dy, 0.23, radians(176.13), radians(2.01), radians(9.58))
            pick_ic = CartesianPose(0.3964+dx, 0.4487+dy, 0.21, radians(176.13), radians(2.01), radians(9.58))

            pre_place_ic = CartesianPose(-0.3093+dx_, 0.4304+dy_, 0.25+dz, radians(176.79), radians(0.50), radians(8.19+dr)) 
            place_ic = CartesianPose(-0.3093+dx_, 0.4304+dy_, 0.215+dz, radians(176.79), radians(0.50), radians(8.19+dr)) 

            pre_pick_vec.append(pre_pick_k)
            pre_pick_ic_vec.append(pre_pick_i)
            pick_ic_vec.append(pick_ic)
            pre_place_ic_vec.append(pre_place_ic)
            place_ic_vec.append(place_ic)
            dx = dx - 0.07

            if ((i%2 ==0 and k%2==0) or (i%2 !=0 and k!=0)) :
                dx_ = dx_ - 0.041 #revisar
            dy_ +=0.041 #revisar
        dy += 0.16
        dr += 90
        dz += 0.025
    dx_place = dx_place - 2.5*0.08

pre_place_i = JointPose().from_list(
    [radians(d) for d in [142.23, -93.62, 111.79, -109.74, -87.16, -60.40]]
)

if __name__ == "__main__":
    robot = VorausIndustrialRobotArm()
    input('vamos vamos...?')

    with robot.connect("voraus-core", port=48401):
        robot._driver._objects.robot.commands.tool.set_tool_transformation_offset((0, 0, 0.103, 0, 0, 0))
        tool_control_output = robot.get_digital_output_v(2)
        robot.enable()

        for i in range (9):
            #print("Press <enter> to pick.")
            #print("hola, estoy al inicio")
            inicio = time.time()
            if i<9:
                robot.move_ptp(pre_pick, blending=Percent(100), speed=Percent(100))
            else:
                robot.move_ptp(pre_pick_vec[i], blending=Percent(100) , speed=Percent(100))
            #print("ya casiii voy a pre pick_i")
            robot.move_linear_v(pre_pick_ic_vec[i], velocity_mps=2.0, extra_parameters=custom_parameters)
            #print("ya casiii voy a pick_i")
            robot.move_linear_v(pick_ic_vec[i], velocity_mps=0.2, extra_parameters=custom_parameters).result()
            #tool_control_output.set()
            sleep(1) #pause,stop... docu
            robot.move_linear_v(pre_pick_ic_vec[i], velocity_mps=0.2, extra_parameters=custom_parameters)
            robot.move_ptp(pre_pick_vec[i], blending=Percent(100), speed=Percent(100))
            #print("Press <enter> to place.")
            robot.move_ptp(pre_place_ic_vec[i], blending= Percent(100), speed=Percent(100))
            robot.move_linear_v(place_ic_vec[i], velocity_mps=0.2, extra_parameters=custom_parameters).result()
            sleep(1) #pause,stop... docu
            robot.move_linear_v(pre_place_ic_vec[i], velocity_mps=0.2, extra_parameters=custom_parameters)
            #tcp_pose = robot.get_tcp_pose()
            #_logger.info("The current TCP pose is %s", tcp_pose)
            #print("Press <enter> to leave.")
            #tool_control_output.clear()
            #print("hola, estoy al final")

        fin = time.time()
        tiempo_total = fin - inicio
        print(f"Tiempo de ejecución: {tiempo_total:.4f} segundos")
        input("Press <enter> to go back.")
        robot.move_ptp(home)
        
        #for j in range(18):

        while(robot.move_ptp(home).result() == False):
            print("Robot is moving to home position.")
  
