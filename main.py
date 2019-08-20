#    get the image and parse it whenever the object is created by its port and ip
#    .e.g 
# receive from the port
    #rcvport = 23333
    #rcv = Rcv(rcvport,ip)
#    rcv.ball.x to get the x of ball
#    rcv.robot_yellow[1].x to get the yellow robot's x with its robot_id=1
try:
    from Rcv import Rcv     
except ImportError:
    raise

#create a command and send it 
#.e.g
    #cmd = Cmd(cmdport,ip)
    #cmd.addcommand()
    #cmd.addcommand(2,3,3,3)
    #print(cmd.robots_command) #Debug info
    #cmd.sendcommands()
try:
    from Cmd import Cmd
except ImportError:
    raise

# create a debug list and send it

try:
    from Dbg import Dbg
except ImportError:
    raise

def main():
    ip = "127.0.0.1"
# receive from the port
    #rcvport = 23333
    #rcv = Rcv(rcvport,ip)
    #print(rcv.robots_yellow[1].x) #Debug info 
    #print(type(rcv.robots_yellow))
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
    print(dbg.msglist) #Debug
    dbg.sendmsgs()
if __name__ == '__main__':
    main()
