import sys
import socket
import protobuf_shader.vision_detection_pb2 as vd
import numpy as np

class Rcv:
    def __init__(self,port,ip):
        Rcv.port = port
        Rcv.ip = ip
        self.server = self.buildConnection()
        self.update()
        self.parseWithProto()
        self.me = [0,'b']
        self.obj = []
        self.myposx = 500
        self.myposy = 500
    def update(self):
        self.data,self.address = self.server.recvfrom(10240)
        self.parseWithProto()
    
    def buildConnection(self):
        try:
            server = socket.socket(type=socket.SOCK_DGRAM)
            server.bind((self.ip,self.port))
            #print('the server has open port {}'.format(self.port))
        except :
            print("the connection can't be set")
        return server
    
    # Analyse the data
    def parseWithProto(self):
        detection_proto = vd.Vision_DetectionFrame()
        detection_proto.ParseFromString(self.data)
        self.ball = detection_proto.balls 
        #print(ball.x) //Degub info
        self.robots_yellow = detection_proto.robots_yellow
        #print(self.robots_yellow[1].x) #Debug info
        self.robots_blue = detection_proto.robots_blue 

    def getLocation(self):
        self.obj = []
        location = np.ones(32,dtype=np.float64)*np.NaN
        #control yellow 0
        for i in range(len(self.robots_yellow)):
            self.obj.append([self.robots_yellow[i].robot_id,'y'])
            if self.me == [self.robots_yellow[i].robot_id,'y']:
                self.myposx = self.robots_yellow[i].x/10 # [cm]
                self.myposy = self.robots_yellow[i].y/10 # [cm]
            else:
                location[2*i]=self.robots_yellow[i].x/10
                location[2*i+1]=self.robots_yellow[i].y/10

        for i in range(len(self.robots_blue)):
            #print(i) #Debug info
            self.obj.append([self.robots_blue[i].robot_id,'b'])
            if self.me == [self.robots_blue[i].robot_id,'b']:
                self.myposx = self.robots_blue[i].x/10 # [cm]
                self.myposy = self.robots_blue[i].y/10 # [cm]
            else:    
                location[2*(i+len(self.robots_yellow))]=self.robots_blue[i].x/10
                location[2*(i+len(self.robots_yellow))+1]=self.robots_blue[i].y/10

        #print("{} objects detected!".format(len(self.obj)))
        location = np.delete(location,np.where(np.isnan(location))[0],0) #delete the nan values in numpy list 
        self.checkMyPos()
        ##    for i in range(15):
            #print(i)
            #location[2*i]=10*(i+1)
            #location[2*i+1]=10*(i+1)
        return location

    def checkMyPos(self):
        if self.me in self.obj:
            #print("I am here!")
            return True
        else:
            print("I'm out!")
            return False

    def getMyPos(self):
        if self.checkMyPos():
            return 
#while True:
#    data,address = server.recvfrom(10240)
#    detection_ball_proto = vision_detection_pb2.Vision_DetectionBall()
# detection_ball_proto.ParseFromString(data)
