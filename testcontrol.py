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

ip = "127.0.0.1"    
MAXPOINT = 20
cmdport = 50001
cmd = Cmd(cmdport,ip)
rcvport = 23333
rcv = Rcv(rcvport,ip)

T = 30

#print(starttime)
nowtime = 0.0
locationx = []
locationy = []
predlist = []
errx = []
erry = []

rcv.update()
rcv.getLocation()
startpx = rcv.myposx
startpy = rcv.myposy
starttime = time.process_time()
index = 0
nowtime = starttime
samptime = starttime+0.01
targetv = 200
cmdv = targetv
theta = math.pi/4
cmdvx = cmdv*math.cos(theta)
cmdvy = cmdv*math.sin(theta)
targetvx = targetv*math.cos(theta)
targetvy = targetv*math.sin(theta)
#print(targetvx,targetvy)

k = 20
# an important number
while (nowtime-starttime)<1 :    
    cmd.addcommand(0,0,cmdvy,0)
    nowtime = time.process_time()
    duration = nowtime - starttime
    cmd.sendcommands() 
    if nowtime>samptime: 
        rcv.update()
        #predictx = startpx + duration*targetvx
        predicty = startpy + duration*targetvy
        #print(nowtime)
        rcv.getLocation()
        #locationx.append(rcv.myposx)
        locationy.append(rcv.myposy)
        #errx.append(predictx-rcv.myposx)
        erry.append(predicty-rcv.myposy)
        #cmdvx = cmdvx+k*(predictx-rcv.myposx) 
        cmdvy = cmdvy+k*(predicty-rcv.myposy)  
        samptime += 0.01

cmd.addcommand(0,0,0,0)  
cmd.sendcommands() 
nowtime = time.process_time()            
rcv.update()
#print(nowtime)
rcv.getLocation()
#locationx.append(rcv.myposx)
locationy.append(rcv.myposy)

#print(locationx[0])
#print(locationx[-1])
#print(locationy[0])
#print(locationy[-1])
#print("errx:",errx)
#print("erry:",erry)


