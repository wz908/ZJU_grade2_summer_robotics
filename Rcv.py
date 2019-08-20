import sys
import socket
import vision_detection_pb2

class Rcv:
    def __init__(self,port,ip):
        Rcv.port = port
        Rcv.ip = ip
        self.server = self.buildConnection()
        self.data,self.address = self.server.recvfrom(10240) 
        self.parseWithProto()
          
    def buildConnection(self):
        try:
            server = socket.socket(type=socket.SOCK_DGRAM)
            server.bind((self.ip,self.port))
            print('the server has open port {}'.format(self.port))
        except :
            print("the connection can't be set")

        return server
    
    # Analyse the data
    def parseWithProto(self):
        detection_proto = vision_detection_pb2.Vision_DetectionFrame()
        detection_proto.ParseFromString(self.data)
        self.ball = detection_proto.balls 
        #print(ball.x) //Degub info
        self.robots_yellow = detection_proto.robots_yellow
        #print(self.robots_yellow[1].x) #Debug info
        self.robots_blue = detection_proto.robots_blue 


#while True:
#    data,address = server.recvfrom(10240)
#    detection_ball_proto = vision_detection_pb2.Vision_DetectionBall()
# detection_ball_proto.ParseFromString(data)


