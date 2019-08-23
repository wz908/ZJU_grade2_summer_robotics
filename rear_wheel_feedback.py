"""

Path tracking simulation with rear wheel feedback steering control and PID speed control.

author: Atsushi Sakai(@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import bisect

class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B

class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s



Kp = 1.0  # speed propotional gain
# steering control parameter
KTH = 1.0
KE = 0.5

dt = 0.1  # [s]
L = 2.9  # [m]

show_animation = True
#  show_animation = False


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def rear_wheel_feedback_control(state, cx, cy, cyaw, ck, preind):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    omega = v * k * math.cos(th_e) / (1.0 - k * e) - \
        KTH * abs(v) * th_e - KE * v * math.sin(th_e) * e / th_e

    if th_e == 0.0 or omega == 0.0:
        return 0.0, ind

    delta = math.atan2(L * omega / v, 1.0)
    #  print(k, v, e, th_e, omega, delta)

    return delta, ind


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):

    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    goal_flag = False
    target_ind = calc_nearest_index(state, cx, cy, cyaw)

    while T >= time:
        di, target_ind = rear_wheel_feedback_control(
            state, cx, cy, cyaw, ck, target_ind)
        ai = PIDControl(speed_profile[target_ind], state.v)
        state = update(state, ai, di)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            print("Goal")
            goal_flag = True
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2)) +
                      ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v, goal_flag


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = cyaw[i + 1] - cyaw[i]
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    return speed_profile

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

def motionInit(path,rcv,theta,velocity):
    reallocation_x = rcv.myposx
    reallocation_y = rcv.myposy

    #print('reallocation:{},{}'.format(reallocation_x,reallocation_y))
    orientation = theta
    sampling_time = 0.3
    set_v_x = velocity*math.cos(theta)
    set_v_y = velocity*math.sin(theta)
    #print('orientation:{}'.format(orientation))
    begin_x = path[-2]
    begin_y = path[-1]
    #print(path)
    #print('beginpoint:{},{}'.format(begin_x,begin_y))
    motion_obj = PyMOTION(reallocation_x,reallocation_y,orientation,path,sampling_time,set_v_x,set_v_y,begin_x,begin_y)
    #print('the motion class is initialized!')
    return motion_obj

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

def getTestCommand(MAXPOINT):
    pass

def getCommand(MAXPOINT,rrt_obj,cmd):
    leng_time = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
    theta_time = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
    leng = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
    theta = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN

    rrt_obj.get_rrt_time(leng_time,theta_time)
    rrt_obj.get_rrt_length(leng)
    rrt_obj.get_rrt_theta(theta)

    print("leng_time:",leng_time)
    print("theta_time:",theta_time)
    print("leng:",leng)
    print("theta:",theta)

    index = 0
    cmd.addcommand(omega=0)
    cmd.sendcommands()
    time.sleep(leng_time[index])
    cmd.newCommand()
    while not np.isnan(theta_time)[index]:
        if theta_time[index]<0:
            cmd.addcommand(vx=0,omega=-0.1)
            cmd.sendcommands() 
            time.sleep(-theta_time[index])
        else:
            cmd.addcommand(vx=0,omega=0.1)
            cmd.sendcommands() 
            time.sleep(theta_time[index])
        cmd.newCommand()
        cmd.addcommand(omega=0)
        cmd.sendcommands()
        time.sleep(leng_time[index+1])
        cmd.newCommand()

        

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


def main():
    print("rear wheel feedback tracking start!!")

    ip = "127.0.0.1"    
    MAXPOINT = 20
# receive from the port
    rcvport = 23333
    dbgport = 20001
    dbg = Dbg(dbgport,ip) 
    print ("clock2:{}".format(time.process_time_ns()))
    rcv = Rcv(rcvport,ip)
    cmdport = 50001
    cmd = Cmd(cmdport,ip)

    rrt_obj,path=buildRRT(MAXPOINT,rcv)
    drawpath(dbg,path)
    index = 0
    ax = []
    ay = []
    while index < len(path):
        ax.insert(0,(path[index])/10)
        ay.insert(0,(path[index+1])/10)
        index = index+2
    #ax = [0.0, 10.0,20.0]
    #ay = [0.0, 0.5,0.0]
    goal = [ax[-1], ay[-1]]
    print(ax)
    print(ay)
    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=0.1)
    target_speed = 10.0/3.6 
    #print(cx)
    #print(cy)
    startx = rcv.myposx
    starty = rcv.myposy
    sp = calc_speed_profile(cx, cy, cyaw, target_speed,startx,starty)
    t, x, y, yaw, v, goal_flag = closed_loop_prediction(
        cx, cy, cyaw, ck, sp, goal)
    #print(v)
    #print(yaw)
    #print(sp)
    # Test
    assert goal_flag, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[cm]")
        plt.ylabel("y[cm]")
        plt.legend()

        plt.subplots(1)
        plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[cm]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[cm]")
        plt.ylabel("curvature [1/cm]")

        plt.show()


if __name__ == '__main__':
    main()
