import numpy as np
try:
    from protobuf_shader.Dbg import Dbg
    from protobuf_shader.Rcv import Rcv     
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
#try:
#    from protobuf_shader.Cmd import Cmd
#except ImportError:
#    raise

# create a debug list and send it


def getLocation(rcv):
    location = np.arange(30,dtype=np.float64)
    #control yellow 0
    for i in range(8):
        location[2*i]=rcv.robots_blue[i].x/10
        location[2*i+1]=rcv.robots_blue[i].y/10
    for i in range(1,8):
        #print(i) #Debug info
        location[2*(i+7)]=rcv.robots_yellow[i].x/10
        location[2*(i+7)+1]=rcv.robots_yellow[i].y/10
##    for i in range(15):
        #print(i)
        #location[2*i]=10*(i+1)
        #location[2*i+1]=10*(i+1)
    return location

def rrtInit(rcv):
    _locationList = getLocation(rcv)
    print(_locationList)
    startpointx = -200
    startpointy = -200
    goalpointx = 200
    goalpointy = 200
    stepsize = 40
    disTh = 20
    maxAttempts = 10000
    #print(np.NaN)
    rrt_obj = PyRRT(startpointx,startpointy,goalpointx,goalpointy,_locationList,stepsize,disTh,maxAttempts)
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

def getPath(MAXPOINT,rcv):
    path = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN    
    rrt_obj = rrtInit(rcv)
    rrt_obj.get_path(path)
    return path

def drawpath(dbg,path):
    index = 0
    PointList = set()
    #print(np.isnan(path)[0])
    while not np.isnan(path)[index]:
        newPoint = dbg.Point(path[index],path[index+1])
        PointList.add(newPoint)
        if index>0 :
            dbg.drawline(lastPoint,newPoint)
        lastPoint= newPoint
        index= index +2
    
    dbg.drawpoints(PointList)  
    print(dbg.msglist) #Debug
    dbg.sendmsgs()



def main():
    ip = "127.0.0.1"    
    MAXPOINT = 100
# receive from the port
    rcvport = 23333
    rcv = Rcv(rcvport,ip)
    _locationList = getLocation(rcv)
    #print(_locationList)
    #print(rcv.robots_yellow[1].x) #Debug info 
    #print(type(rcv.robots_yellow))

# get the path
    #path=getTestPath(MAXPOINT)
    path=getPath(MAXPOINT,rcv)
    #print(path)
    

# send a command
    #cmdport = 50001
    #cmd = Cmd(cmdport,ip)
    #cmd.addcommand()
    #cmd.addcommand(2,3,3,3)
    #print(cmd.robots_command) #Debug info
    #cmd.sendcommands()

# draw some debugs
    dbgport = 20001
    dbg = Dbg(dbgport,ip)
    drawpath(dbg,path)


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
