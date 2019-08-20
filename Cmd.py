import sys  
import socket
import zss_cmd_pb2

class Cmd:
    def __init__(self,port,ip):
        self.robots_command = zss_cmd_pb2.Robots_Command()
        self.robots_status = zss_cmd_pb2.Robot_Status()
        self.port=port
        self.ip = ip
        
    def addcommand(self,index=1, vx=1.5, vy=1.5, omega=7, kick=0,power=60,spin=0,delay=3):
        command = self.robots_command.command.add()   
        command.robot_id = index
        command.velocity_x = vx
        command.velocity_y = vy
        command.velocity_r = omega
        command.kick = kick
        command.power = power
        command.dribbler_spin = spin
        self.robots_command.delay = delay

    def sendcommands(self):
        commands_str = self.robots_command.SerializeToString()
        #print(self.commands) #Debug info
        client = socket.socket(type=socket.SOCK_DGRAM)
        send_data  = commands_str
        while True:
            client.sendto(send_data,(self.ip,self.port))
        client.close()
