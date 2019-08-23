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
erro = []

rcv.update()
rcv.getAllState()
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
k2 = 20
# an important number
omega = 0
while (nowtime-starttime)<1 :    
    cmd.addcommand(0,0,targetvy,omega)
    nowtime = time.process_time()
    duration = nowtime - starttime
    cmd.sendcommands() 
    if nowtime>samptime: 
        rcv.update()
        #predictx = startpx + duration*targetvx
        predicty = startpy + duration*targetvy
        #print(nowtime)
        rcv.getMyState()
        #locationx.append(rcv.myposx)
        locationy.append(rcv.myposy)
        #errx.append(predictx-rcv.myposx)
        erry.append(predicty-rcv.myposy)
        #cmdvx = cmdvx+k*(predictx-rcv.myposx) 
        cmdvy = cmdvy+k*(predicty-rcv.myposy) 
        erro.append(rcv.mybot.orientation)
        omega = -k*rcv.mybot.orientation
        samptime += 0.01

cmd.addcommand(0,0,0,0)  
cmd.sendcommands() 
nowtime = time.process_time()            
rcv.update()
#print(nowtime)
rcv.getMyState()
#locationx.append(rcv.myposx)
locationy.append(rcv.myposy)

print("erro",erro)
print("erry",erry)
#print(locationx[0])
#print(locationx[-1])
print(locationy[0])
print(locationy[-1])
#print("errx:",errx)
#print("erry:",erry)


