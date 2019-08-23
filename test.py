
import numpy as np
import time
import sys
import math

try:
    from protobuf_shader.Dbg import Dbg
    from protobuf_shader.Rcv import Rcv 
    from protobuf_shader.Cmd import Cmd    
    from rrt.rrt import PyRRT
    from motion.Pymotion import PyMOTION
except ImportError:
    raise
#    get the image and parse it whenever the object is created by its port and ip
#    .e.g 
# receive from the port
    #rcvport = 23333
    #rcv = Rcv(rcvport,ip)
#    rcv.ball.x to get the x of ball
#    rcv.robot_yellow[1].x to get the yellow robot's x with its robot_id=1

#create a command and send it 
#.e.g
    #cmd = Cmd(cmdport,ip)
    #cmd.addcommand()
    #cmd.addcommand(2,3,3,3)
    #print(cmd.robots_command) #Debug info
    #cmd.sendcommands()

import matplotlib.pyplot as plt
import numpy as np
from random import random

# simulation parameters
Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.01

show_animation = True


def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal,rcv,dbg,cmd):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle

    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """
    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    rho = np.sqrt(x_diff**2 + y_diff**2)
    while rho > 30:
        time.sleep(1)
        x_diff = x_goal - x
        y_diff = y_goal - y

        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.sqrt(x_diff**2 + y_diff**2)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v
        
        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt
        thistime = time.process_time()
        
        pos = dbg.Point(x,y)
        direc = theta
        dbg.drawrobot(pos,direc)
        dbg.sendmsgs()



def rrtInit(rcv):
    rcv.getAllState()
    _locationList = rcv.obstacleList
    #print(_locationList)
    startpointx = rcv.robots_blue[0].x/10
    startpointy = rcv.robots_blue[0].y/10
    goalpointx = 200
    goalpointy = 200
    stepsize = 40
    disTh = 20
    maxAttempts = 10000
    radius = 400
    #print(np.NaN)
    rrt_obj = PyRRT(startpointx,startpointy,goalpointx,goalpointy,_locationList,stepsize,disTh,maxAttempts,radius)
    return rrt_obj


def buildRRT(MAXPOINT,rcv):
    path = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN 
    rrt_obj = rrtInit(rcv)
    #print("rrt_obj initialzed")
    rrt_obj.get_path(path)
    path = np.delete(path,np.where(np.isnan(path))[0],0) #delete the nan values in numpy list 
    return rrt_obj,path


def rrt_debug(MAXPOINT,rrt_obj,dbg):    
    pone = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
    ptwo = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
    pthree = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
    pfour = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN

    rrt_obj.get_rrt(pone,ptwo,pthree,pfour)

    #print("pone:",pone)
    #print("ptwo:",ptwo)
    #print("pthree:",pthree)
    #print("pfour:",pfour)

    #print(np.isnan(pone)[40])
    drawTree(pone,ptwo,pthree,pfour,dbg)


def drawpath(dbg,path):
    index = 0
    PointList = set()
    #print(np.isnan(path)[0])
    while index < len(path):
        newPoint = dbg.Point(path[index],path[index+1])
        PointList.add(newPoint)
        if index>0 :
            dbg.drawline(lastPoint,newPoint,False,7)
        lastPoint= newPoint
        index = index +2
    
    dbg.drawpoints(PointList)  
    #print(dbg.msglist) #Debug
    dbg.sendmsgs()
    #dbg.newmsgs()

ip = "127.0.0.1"    
MAXPOINT = 20
# receive from the port
rcvport = 23333
dbgport = 20001
dbg = Dbg(dbgport,ip) 
rcv = Rcv(rcvport,ip)
cmdport = 50001
cmd = Cmd(cmdport,ip)

rrt_obj,path=buildRRT(MAXPOINT,rcv)
drawpath(dbg,path)

startstate = [path[0],path[1],rcv.mybot.orientation]
#print(startstate)
laststate = startstate
stateList = []
i=2
print(path)
while i<len(path):
    theta = (path[i+1]-path[i-1])/(path[i]-path[i-2])
    thisstate = [path[i],path[i+1],theta] 
    print("thisstate:",thisstate)
    print("laststate:",laststate)
    move_to_pose(laststate[0],laststate[1],laststate[2],thisstate[0],thisstate[1],thisstate[2],rcv,dbg,cmd)
    #rcv.update()
    #rcv.getMyState()
    laststate=[path[i],path[i+1],theta]
    i+=2



    