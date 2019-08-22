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

print ("start time:{}".format(time.process_time_ns()))
ip = "127.0.0.1"    
MAXPOINT = 20
cmdport = 50001
cmd = Cmd(cmdport,ip)
rcvport = 23333
rcv = Rcv(rcvport,ip)

T = 30
starttime = time.process_time()
nowtime = 0.0
locationx = []
locationy = []
while (nowtime-starttime)<60 :
    currtime = time.process_time()
    while True:        
        cmd.addcommand(0,30,0,0)
        nowtime = time.process_time()
        cmd.sendcommands() 
        if nowtime - currtime > 3:
            rcv.update()
            rcv.getLocation()
            locationx.append(rcv.myposx)
            locationy.append(rcv.myposy)
            break 
    currtime = time.process_time()
    while True:
        cmd.addcommand(0,-30,0,0)  
        nowtime = time.process_time()
        #rcv = Rcv(rcvport,ip)
        cmd.sendcommands() 
        if nowtime - currtime > 3:
            rcv.update()
            rcv.getLocation()
            locationx.append(rcv.myposx)
            locationy.append(rcv.myposy)
            break 

cmd.addcommand(0,0,0,0)  
cmd.sendcommands() 
print(locationx)
print(locationy)