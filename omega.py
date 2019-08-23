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

starttime = time.process_time()
while True:
    cmd.addcommand(0,0,-100,0)
    cmd.sendcommands() 
    if time.process_time()-starttime>5:
        break
cmd.addcommand(0,0,0,0)
cmd.sendcommands()