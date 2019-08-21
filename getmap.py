try:
    from Rcv import Rcv     
except ImportError:
    raise

class Map:
    def __init__(self):
        pass

    def getlocation(self,rcv):
        locationList = []
        for i in range(8):
        #print(i) #Debug info
            locationList.append([rcv.robots_yellow[i].x,rcv.robots_yellow[i].y])
        for i in range(8):
            locationList.append([rcv.robots_blue[i].x,rcv.robots_blue[i].y])
    #print(locationList) #Debug info

        return locationList

    def getmap(self):
        ip = "127.0.0.1"
        rcvport = 23333
        rcv = Rcv(rcvport,ip)
        robot_location = self.getlocation(rcv)
        start = robot_location[15] #the robot 7
        self.robot_location= robot_location
        obstacle = []
        obstacle_range = 2
        for i in range(15):
            obstacle.append((robot_location[i][0],robot_location[i][1],obstacle_range))
        #print(obstacle) #Debug info
        #print(start) #Debug info 
        self.start = start 
        self.obstacle = obstacle
