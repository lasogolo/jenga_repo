# pylint: disable=protected-access
"""Contains the program which controls the robot."""

import time
from time import sleep
import logging
from math import radians
from voraus_robot_arm import CartesianPose, JointPose, VorausIndustrialRobotArm, Percent, MoveCartesianVictorParameters

import numpy as np
from math import radians
# Initial variables:

x1=float(-0.3443) ; x2=float(-0.3443)
y1=float(0.4734) ; y2=float(0.3914)
z0=float(0.3300) ; pisos=int(6)
h_slow = 0; h_fast=0; h = 0.02; tol_pick=0; tol_place=0
rx=float(176.79); ry=float(0.50); rz=float(98.19)

# x1=float(input("Geben Sie x1 ein (zb: 0.3443)")) ; y1=float(input("Geben Sie y1 ein (zb: 0.3914 ")) ; z0=float(input("Geben Sie z1-2 ein (zb: 0.330)"))
# x2=float(input("Geben Sie x2 ein (zb: 0.345)")) ; y2=float(input("Geben Sie y2 ein (zb: 0.345)")) ; pisos=int(input("Geben Sie Stock ein "))
# h_slow = 0; h_fast=0; h = 0; tol_pick=0; tol_place=0
# rx=float(input("Geben Sie rx  ein ")); ry=float(input("Geben Sie ry ein ")); rz=float(input("Geben Sie rz ein "))


# x1= , x2=; y1= ;y2=; z0=
# rx=; ry=; rz=; h
dz=0
dx=0.25
do=0.1
# def vert(A,B):
#     if (A%2==0 and B%2==0) or ((A%2!=0 and B%2!=0)):
#         return True

pre_pre_pick = []; pre_pick = []; pick = []; 
pre_pre_place = []; pre_place = []; place = []


new_pre_place=[]; new_place=[]
new_pre_pick=[]; new_pick=[]


for i in range(int(pisos/2)):
    p1=np.array([x1,y1,z0 - dz]); 
    p2=np.array([x2,y2,z0 - dz]); 
    p3=np.array([(x1+x2)/2,(y1+y2)/2,z0 -dz]); 
    # Sig. piso
    p5=np.array([p3[0],p3[1],p3[2]-2*dz]); d32 = np.linalg.norm(p3-p2); d12 = np.linalg.norm(p1-p2)
    p45= d32 * np.cross(p5,np.array([0,0,-1])) / (np.linalg.norm(np.cross(p5,np.array([0,0,-1]))))
    p4=np.array([p5[0]+p45[0],p5[1]+p45[1],p5[2]])
    p65= d12 * np.cross(p5,np.array([0,0,1])) / (np.linalg.norm(np.cross(p5,np.array([0,0,1]))))
    p6=np.array([p5[0]+p65[0],p5[1]+p65[1],p5[2]])


    points = [p1, p2, p3, p4, p5, p6]
    rz_offsets = [0, 0, 0, 90, 90, 90]
    for p, rz_off in zip(points, rz_offsets):
        h_fast += 0.1
        h_slow += 0.05
        pre_pre_pick_i = CartesianPose(p[0], p[1], p[2] + h_fast, radians(rx), radians(ry), radians(rz + rz_off))
        pre_pick_i = CartesianPose(p[0], p[1], p[2] + h_slow, radians(rx), radians(ry), radians(rz + rz_off))
        pick_i = CartesianPose(p[0], p[1], p[2] + tol_pick, radians(rx), radians(ry), radians(rz + rz_off))

        
        pre_pre_place_i = CartesianPose(p[0]+dx, p[1], p[2] + h_fast, radians(rx), radians(ry), radians(rz + rz_off))
        pre_place_i = CartesianPose(p[0]+dx, p[1], p[2] + h_slow-h*pisos, radians(rx), radians(ry), radians(rz + rz_off))
        place_i = CartesianPose(p[0]+dx, p[1], p[2] + tol_place-h*pisos, radians(rx), radians(ry), radians(rz + rz_off))
        
        pre_pre_pick.append(pre_pre_pick_i)
        pre_pick.append(pre_pick_i) 
        pick.append(pick_i)

        pre_pre_place.append(pre_pre_place_i)
        pre_place.append(pre_place_i)
        place.append(place_i)

        new_place_i = CartesianPose(p[0], p[1], p[2] + tol_place, radians(rx), radians(ry), radians(rz + rz_off))
        new_pick_i = CartesianPose(p[0]+dx, p[1], p[2] + tol_pick, radians(rx), radians(ry), radians(rz + rz_off))
        #estos 2 probablemente se recorran al reves

        new_pick.append(new_pick_i)
        new_place.append(new_place_i)
    dz = dz - h

print(pre_pre_pick[0]); print(pre_pre_pick[1])
print(pre_pick[0]) ; print(pre_pick[1])
print(pick[0]) ; print(pick[1])

new_pre_pre_place = pre_pre_pick ; new_pre_place = pre_pick
new_pre_pre_pick = pre_pre_place ; new_pre_pick = pre_place

# pre_pre_place = pre_pre_pick
# pre_place = pre_pick

# Para el codigo main:
"al momento de recorrer los vectores, los pick van con [i], sin embargo place con [(pisos*3-1)-(i)]"

custom_acel_low = 5
custom_parameters = MoveCartesianVictorParameters(
        acceleration_mps2=custom_acel_low)
 
home = JointPose(0, -1.57, 1.57, -1.57, -1.57, 0) # No se, lo cambie y no altero notoriamente el Bewegung
joint =  JointPose().from_list([radians(d) for d in [134.81, -102.52, 104.46, -92.81, -91.44, 0]])

if __name__ == "__main__":
    robot = VorausIndustrialRobotArm()
    input('vamos vamos...?')
 
    with robot.connect("voraus-core", port=48401):
        robot._driver._objects.robot.commands.tool.set_tool_transformation_offset((0, 0, 0.103, 0, 0, 0))
        tool_control_output = robot.get_digital_output_v(2)
        robot.enable()
 
        robot.move_ptp(joint, speed=Percent(100))
        m=0
        f=0
        while (True):
            
            if f != 0:
                ans = input("sigo? :'( ").lower()
                if ans == "nein":
                    print("danke")
                    break
            f+=1
            for j in range(18):
                n = 18-1   #18-1
 
                #new pick
                robot.move_ptp(pre_pre_pick[j], blending=Percent(50) , speed=Percent(100))
                print("en preprepick")
                print(pre_pre_pick[j])
                robot.move_linear_v(pre_pick[j], blending=Percent(50), extra_parameters=custom_parameters)
                print("en prepick")
                print(pre_pick[j])
                robot.move_linear_v(pick[j], velocity_mps=0.2, extra_parameters=custom_parameters).result()
                print(pick[j])
                sleep(1) #pause,stop... docu
                tool_control_output.set()
                robot.move_linear_v(pre_pick[n-j], blending=Percent(50) , velocity_mps=0.1)
                robot.move_ptp(pre_pre_pick[n-j], blending=Percent(50) , speed=Percent(100))

                robot.move_ptp(pre_pre_place[j], blending= Percent(50), speed=Percent(100))
 
                robot.move_ptp(pre_place[j], blending= Percent(50), speed=Percent(100))
                robot.move_linear_v(place[j], velocity_mps=0.2, extra_parameters=custom_parameters).result()
                sleep(1) #pause,stop... docu
                tool_control_output.clear()
                robot.move_linear_v(pre_place[j], blending=Percent(50) , velocity_mps=0.2, extra_parameters=custom_parameters)
                robot.move_linear_v(pre_pre_place[j], blending=Percent(50) , velocity_mps=0.2, extra_parameters=custom_parameters)
                
            sleep(1) #pause,stop... docu
            for j in range(18):
                n = 18-1
                #new pick
                robot.move_ptp(pre_place[n-j], blending=Percent(50) , speed=Percent(100))
                robot.move_linear_v(new_pick[n-j], velocity_mps=0.2, extra_parameters=custom_parameters).result()
                sleep(1) #pause,stop... docu
                tool_control_output.set()
                robot.move_linear_v(pre_place[n-j], blending=Percent(50) , velocity_mps=0.2, extra_parameters=custom_parameters)
 
                robot.move_ptp(pre_pick[j], blending= Percent(50), speed=Percent(100))
                robot.move_linear_v(new_place[j], velocity_mps=0.2, extra_parameters=custom_parameters).result()
                sleep(1) #pause,stop... docu
                tool_control_output.clear()
                robot.move_linear_v(pre_pick[j], blending=Percent(50) , velocity_mps=0.2, extra_parameters=custom_parameters)
 
            m=m+1
            input("Press <enter> to go back.")
            robot.move_ptp(home)
            while(robot.move_ptp(home).result() == False):
                print("Robot is moving to home position.")

        
  
