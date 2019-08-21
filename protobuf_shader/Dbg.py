import sys  
import protobuf_shader.zss_debug_pb2 as debug
import socket

class Dbg:
    def __init__(self,port,ip):
        self.msglist = debug.Debug_Msgs()
        self.port = port
        self.ip = ip
    class Point:
        def __init__(self,px=0,py=0):
            self.x = px
            self.y = py

    def drawpoint(self, point,color=2):
        pointmsgs=self.msglist.msgs.add() 
        pointmsg = pointmsgs.points.point.add()
        pointmsg.x = point.x
        pointmsg.y = point.y
        pointmsgs.color = color 
        pointmsgs.type = 6
    
    def drawpoints(self, pointList,color=3):
        pointmsgs=self.msglist.msgs.add()
        for point in pointList:
            pointmsg=pointmsgs.points.point.add()
            pointmsg.x = point.x
            pointmsg.y = point.y
        pointmsgs.color = color 
        pointmsgs.type = 6

    def drawrobot(self,pos,direc=0.0,color=1): #pos in point class
        robotmsg = self.msglist.msgs.add()
        robotmsg.type = 3
        robotmsg.color = color
        robotmsg.robot.pos.x = pos.x
        robotmsg.robot.pos.y = pos.y
        robotmsg.robot.dir = direc

    def drawline(self, point1, point2, ori=True,color=4):
        linemsg = self.msglist.msgs.add()
        linemsg.type = 1
        linemsg.color = color
        linemsg.line.start.x = point1.x
        linemsg.line.start.y = point1.y
        linemsg.line.end.x = point2.x
        linemsg.line.end.y = point2.y
        linemsg.line.FORWARD = ori
        linemsg.line.BACK = not ori
    

###TODO
    def drawPolygon(self, pointList,color=6,fill=1):
        polymsg = self.msglist.msgs.add()
        polymsg.type = 5
        polymsg.color = color
        polymsg.polygon.FILL = fill 
        for point in pointList:
            vertexmsg=polymsg.polygon.vertex.add()
            vertexmsg.x = point.x
            vertexmsg.y = point.y

    def drawRectangle(self,point1,point2,color=5):
        Point1 = self.Point(point1.x,point1.y)
        Point2 = self.Point(point1.x,point2.y)
        Point3 = self.Point(point2.x,point2.y)
        Point4 = self.Point(point2.x,point2.y)
        Pointlist = set()
        Pointlist.add(Point1)
        Pointlist.add(Point2)
        Pointlist.add(Point3)
        Pointlist.add(Point4)
        self.drawPolygon(PointList,7,0)
###END TODO
 
    def displayText(self,pos,text,color=9):
        textmsg = self.msglist.msgs.add()
        textmsg.type = 2
        textmsg.color = color
        textmsg.text.pos.x = pos.x
        textmsg.text.pos.y = pos.y
        textmsg.text.text = text

    def sendmsgs(self):
        msgs_str = self.msglist.SerializeToString()
        send_data = msgs_str
        client = socket.socket(type=socket.SOCK_DGRAM)
        client.sendto(send_data,(self.ip,self.port))
        client.close()

    def newmsgs(self):
        self.msglist = debug.Debug_Msgs()