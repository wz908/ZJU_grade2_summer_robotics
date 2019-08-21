# distutils: language = c++
from rrtTest cimport RRT
import numpy as np
cimport numpy as np

cdef class PyRRT:
    cdef RRT c_rrt

    def __cinit__(self,double startpointx, double startpointy, double goalpointx,double goalpointy, np.ndarray[np.double_t,ndim=1] _locationList,double stepsize,double disTh,int attemtps, int radius=220):
        self.c_rrt = RRT(startpointx, startpointy, goalpointx, goalpointy,<double*>_locationList.data,stepsize,disTh,attemtps,radius)

    def get_path(self,np.ndarray[np.double_t,ndim=1] path):
        self.c_rrt.FindPath(<double*> path.data)
        return path

    def get_rrt(self,np.ndarray[np.double_t,ndim=1] pone,np.ndarray[np.double_t,ndim=1] ptwo,np.ndarray[np.double_t,ndim=1] pthree,np.ndarray[np.double_t,ndim=1] pfour):
        mycounter = self.c_rrt.get_rrt(<double*> pone.data, <double*> ptwo.data, <double*> pthree.data,<double*> pfour.data)
        return mycounter
    
    def get_rrt_time(self,np.ndarray[np.double_t,ndim=1] leng_time, np.ndarray[np.double_t,ndim=1] theta_time):
        self.c_rrt.get_rrt_time(<double*>leng_time.data,<double*>theta_time.data)

    def get_rrt_length(self,np.ndarray[np.double_t,ndim=1] leng):
        self.c_rrt.get_rrt_length(<double*>leng.data)
    
    def get_rrt_theta(self,np.ndarray[np.double_t,ndim=1] theta):
        self.c_rrt.get_rrt_theta(<double*>theta.data)

