import numpy as np
import time
import sys
import math

try:
    from protobuf_shader.Dbg import Dbg
    from protobuf_shader.Rcv import Rcv 
    from protobuf_shader.Cmd import Cmd    
    from rrt.rrt import PyRRT
except ImportError:
    raise

def rrtInit(rcv):
    _locationList = rcv.getLocation()
    print(_locationList)
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
    rrt_obj.get_path(path)
    return rrt_obj,path

def drawpath(dbg,path):
    index = 0
    PointList = set()
    #print(np.isnan(path)[0])
    while not np.isnan(path)[index]:
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


def dwa_control(x, u, config, goal, ob):
    """
        Dynamic Window Approach control
    """

    dw = calc_dynamic_window(x, config)

    u, traj = calc_final_input(x, u, dw, config, goal, ob)

    return u, traj


class Config():
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 300  # [cm/s]
        self.min_speed = -50  # [m/s]
        self.max_yawrate = 4.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 50  # [m/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.1  # [m/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.3  # [s] Time tick for motion prediction
        self.predict_time = 2.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_radius = 20.0  # [m] for collision check



def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt   # omega
    x[0] += u[0] * math.cos(x[2]) * dt # x
    x[1] += u[0] * math.sin(x[2]) * dt # y
    x[3] = u[0]  # velocity
    x[4] = u[1]  # yaw

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def calc_final_input(x, u, dw, config, goal, ob):
    """
    calculation final input with dinamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_traj = np.array([x])

    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):

            traj = predict_trajectory(x_init, v, y, config)

            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = config.obstacle_cost_gain*calc_obstacle_cost(traj, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_traj = traj

    return best_u, best_traj


def calc_obstacle_cost(traj, ob, config):
    """
        calc obstacle cost inf: collision
    """

    skip_n = 2 # for speed up
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - traj[-1, 0]
    dy = goal[1] - traj[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost = abs(error_angle - traj[-1, 2])

    return cost



ip = "127.0.0.1"    
MAXPOINT = 100

rcvport = 23333
rcv = Rcv(rcvport,ip)
#print(rcv.me)
obstacleList = rcv.getLocation()
#print(rcv.myposx)
#print(obstacleList)
rrt_obj,path=buildRRT(MAXPOINT,rcv)

print(path)
index = 0
ax = []
ay = []
while not np.isnan(path)[index]:
    ax.insert(0,path[index])
    ay.insert(0,path[index+1])
    index = index+2

x = np.array([rcv.myposx,rcv.myposy,0.0,0.0,0.0])
goal = np.array([ax[1],ay[1]])
#print(goal)
ob = np.reshape(obstacleList,[15,2])
#print(ob)
u = np.array([0.0,0.0])
config = Config()
traj = np.array(x)

while True:
    u, ptraj = dwa_control(x, u, config, goal, ob)

    x = motion(x, u, config.dt) # simulate robot
    traj = np.vstack((traj, x))  # store state history
    print(traj)
    print(x)
    dist_to_goal = math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2)  
    if dist_to_goal <= config.robot_radius:
        print("Goal!!")
        break

print("Done")