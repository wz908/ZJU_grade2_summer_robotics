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
pone = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
ptwo = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
pthree = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
pfour = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN

leng_time = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
theta_time = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
leng = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN
theta = np.ones(MAXPOINT, dtype=np.float64 ) * np.NaN

#print(np.NaN)
rrt_obj = rrt.PyRRT(startpointx,startpointy,goalpointx,goalpointy,_locationList,stepsize,disTh,maxAttempts)
rrt_obj.get_path(path)
rrt_obj.get_rrt(pone,ptwo,pthree,pfour)
rrt_obj.get_rrt_time(leng_time,theta_time)
rrt_obj.get_rrt_length(leng)
rrt_obj.get_rrt_theta(theta)
print(path)
print("pone:",pone)
print("ptwo:",ptwo)
print("pthree:",pthree)
print("pfour:",pfour)
print("leng_time:",leng_time)
print("theta_time:",theta_time)
print("leng:",leng)
print("theta:",theta)