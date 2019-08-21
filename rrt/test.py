import numpy as np
import rrt 

MAXPOINT = 100
def getLocation():
    location = np.arange(30,dtype=np.float64)
    for i in range(15):
        #print(i)
        location[2*i]=10*(i+1)
        location[2*i+1]=10*(i+1)
    return location
_locationList = getLocation()
#print(_locationList)
startpointx = -200
startpointy = -200
goalpointx = 200
goalpointy = 200
stepsize = 40
disTh = 20
maxAttempts = 10000
path = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
#print(np.NaN)
rrt_obj = rrt.PyRRT(startpointx,startpointy,goalpointx,goalpointy,_locationList,stepsize,disTh,maxAttempts)
rrt_obj.get_path(path)
print(path)