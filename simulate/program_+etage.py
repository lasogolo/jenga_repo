# pylint: disable=protected-access
"""Contains the program which controls the robot."""
print("Iniciando...")
import logging
from math import radians
from voraus_robot_arm import CartesianPose, JointPose, VorausIndustrialRobotArm, Percent, configure_logging

_logger = logging.getLogger()
home = JointPose(0, -1.57, 1.57, -1.57, -1.57, 0) # No se, lo cambie y no altero notoriamente el Bewegung

pre_pick_vec=[]
pre_pick_ic_vec=[]
pick_ic_vec=[]
pre_place_ic_vec=[]
place_ic_vec=[]
pre_pick = JointPose(80.41, -82.18, 93.56, -97.89, -87.37, 0) #Actualizar y cambiar a cartesian, sumar en x: dx

dz=0
dx_pre=0 #anadir en codigo
for k in range(2):
    #dx=-k*3*0.07
    dy=0
    dr=0
    for i in range(3):
        dx=-k*3.6*0.07
        dx_=0
        dy_= -0.035#-0.039

        for j in range (3):
            if (i%2 ==0) :
                dy_ = 0  
            else:
                dx_ = -0.037#*2 

            
            pre_pick_i = CartesianPose(0.3964+dx, 0.4487+dy, 0.23, radians(176.13), radians(2.01), radians(9.58))
            pick_ic = CartesianPose(0.3964+dx, 0.4487+dy, 0.21, radians(176.13), radians(2.01), radians(9.58))

            pre_place_ic = CartesianPose(-0.3093+dx_, 0.4304+dy_, 0.26+dz, radians(176.79), radians(0.50), radians(8.19+dr)) 
            place_ic = CartesianPose(-0.3093+dx_, 0.4304+dy_, 0.23+dz, radians(176.79), radians(0.50), radians(8.19+dr)) 

            #pre_pick_vec.append(pre_pick) Actualizar y cambiar a cartesian, sumar en x: dx
            pre_pick_ic_vec.append(pre_pick_i)
            pick_ic_vec.append(pick_ic)
            pre_place_ic_vec.append(pre_place_ic)
            place_ic_vec.append(place_ic)
            dx = dx - 0.08

            if (i%2 ==0) :
                dx_ = dx_ - 0.043 #revisar
            dy_ +=0.043
        dy += 0.16
        dr += 90
        dz += 0.025
    #dz += 0.02*4
    dx_pre = dx_pre - (k+1)*3.6*0.07

if __name__ == "__main__":
    robot = VorausIndustrialRobotArm()

    with robot.connect("voraus-core", port=48401):
        robot._driver._objects.robot.commands.tool.set_tool_transformation_offset((0, 0, 0.103, 0, 0, 0))
        tool_control_output = robot.get_digital_output_v(2)
        robot.enable()

        for i in range (18):
            print("Press <enter> to pick.")
            robot.move_ptp(pre_pick, blending=Percent(50), speed=Percent(100))
            robot.move_linear(pre_pick_ic_vec[i], blending=Percent(50), velocity_mps=1.0)
            robot.move_linear(pick_ic_vec[i], velocity_mps=0.2).result()
            tool_control_output.set()

            robot.move_linear(pre_pick_ic_vec[i], velocity_mps=0.2)
            robot.move_ptp(pre_pick).result()

            print("Press <enter> to place.")
            robot.move_ptp(pre_place_ic_vec[i], blending= Percent(50), speed=Percent(100))
            robot.move_linear(place_ic_vec[i], velocity_mps=0.2).result()
            #tcp_pose = robot.get_tcp_pose()
            #_logger.info("The current TCP pose is %s", tcp_pose)
            print("Press <enter> to leave.")
            tool_control_output.clear()

        input("Press <enter> to go back.")
        robot.move_ptp(home)
        while(robot.move_ptp(home).result() == False):
            print("Robot is moving to home position.")
  
