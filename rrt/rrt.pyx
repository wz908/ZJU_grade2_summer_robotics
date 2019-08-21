# distutils: language = c++
from rrtTest cimport RRT
import numpy as np
cimport numpy as np

cdef class PyRRT:
    cdef RRT c_rrt

    def __cinit__(self,double startpointx, double startpointy, double goalpointx,double goalpointy, np.ndarray[np.double_t,ndim=1] _locationList,double stepsize,double disTh,int attemtps ):
        self.c_rrt = RRT(startpointx, startpointy, goalpointx, goalpointy,<double*>_locationList.data,stepsize,disTh,attemtps)

    def get_path(self,np.ndarray[np.double_t,ndim=1] path):
        self.c_rrt.FindPath(<double*> path.data)
        return path