import numpy as np
import time
import sys

try:
    from protobuf_shader.Dbg import Dbg
    from protobuf_shader.Rcv import Rcv 
    from protobuf_shader.Cmd import Cmd    
    from rrt.rrt import PyRRT
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

def rrtInit(rcv):
    _locationList = getLocation(rcv)
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

def getTestPath(MAXPOINT):
    path = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
    path[0]=200
    path[1]=200
    path[2]=231.65
    path[3]=-34.2
    path[4]=93
    path[5]=-92.86
    path[6]=-200
    path[7]=-200
    print(path)
    return path

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
    rrt_obj.get_path(path)
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


def main():
    #print ("clock1:{}".format(time.process_time()))
    ip = "127.0.0.1"    
    MAXPOINT = 20
# receive from the port
    rcvport = 23333
    rcv = Rcv(rcvport,ip)
    print(rcv.me)
    obstacleList = rcv.getLocation()
    print(rcv.myposx)
    #print ("clock2:{}".format(time.process_time()))
    #print(len(rcv.robots_yellow))

# draw some debugs
    #dbgport = 20001
    #dbg = Dbg(dbgport,ip)
    #print ("clock2:{}".format(time.process_time()))
    #_locationList = getLocation(rcv)
    #print ("clock3:{}".format(time.process_time()))
    #print(_locationList)
    #print(rcv.robots_yellow[1].x) #Debug info 
    #print(type(rcv.robots_yellow))

# get the path
    #path=getTestPath(MAXPOINT)
    #rrt_obj,path=buildRRT(MAXPOINT,rcv)
    #print ("clock4:{}".format(time.clock()))
    #drawpath(dbg,path)
    #print ("clock5:{}".format(time.clock()))
    #rrt_debug(MAXPOINT,rrt_obj,dbg)
    #print(path)
    #startpointx = rcv.robots_blue[0].x/10
    #startpointy = rcv.robots_blue[0].y/10
    #index = 0
    #ax = []
    #ay = []
    #while not np.isnan(path)[index]:
    #    ax.insert(0,path[index])
    #    ay.insert(0,path[index+1])
    #    index = index+2


    
# send a command
    #cmdport = 50001
    #cmd = Cmd(cmdport,ip)


    #getCommand(MAXPOINT,rrt_obj,cmd)
    #cmd.addcommand()
    #cmd.addcommand(2,3,3,3)
    #print(cmd.robots_command) #Debug info
    #cmd.sendcommands()


    #Point = dbg.Point(3,3) # 3 is alternative number
    #dbg.drawpoint(Point)
    #print(Point) #Debug info

#if you want to draw a robot uncomment the following
    #Point = dbg.Point(3,3) # 3 is alternative number
    #dbg.drawrobot(Point)
    #print(dbg.msglist) #Debug info

#if you want to draw points uncomment the following
    #Point1 = dbg.Point(3,3)
    #Point2 = dbg.Point(40,40)
    #Pointlist = set()
    #Pointlist.add(Point1)
    #Pointlist.add(Point2)
    #print(Pointlist) #Debug info
    #dbg.drawpoints(Pointlist)  

#if you want to draw a line uncomment the following:
    #Point1 = dbg.Point(3,3)
    #Point2 = dbg.Point(40,40)
    #dbg.drawline(Point1,Point2) #ori

#if you want to draw a rectangle, uncomment the following:
#    Point1 = dbg.Point(3,3)
#    Point2 = dbg.Point(40,40)
#    dbg.drawRectangle(Point1,Point2) #ori  

#if you want to draw a polygon, uncomment the following:
    #Point1 = dbg.Point(3,3)
    #Point2 = dbg.Point(40,40)
    #Point3 = dbg.Point(3,40)
    #Point4 = dbg.Point(40,3)
    #Pointlist = set()
    #Pointlist.add(Point1)
    #Pointlist.add(Point2)
    #Pointlist.add(Point3)
    #Pointlist.add(Point4)
    
    #print(Pointlist) #Debug info
    #dbg.drawPolygon(Pointlist)  

#if you want to draw an arc uncomment the following:
    #dbg.drawarc()
    #dbg.drawcurve()

#if you want to input a text uncomment the following:
    #pos = dbg.Point(100,100)
    #dbg.displayText(pos,"NMSL")

# if you want to send the message, uncomment the following
    #print(dbg.msglist) #Debug
    #dbg.sendmsgs()
if __name__ == '__main__':
    main()
