cdef extern from "rrtTest.cpp":
    pass

cdef extern from "rrtTest.h" namespace "myrrt":
    ctypedef struct point:
        double x
        double y
    
    ctypedef struct node:
        point p
        int prev
        double dist
    
    cdef cppclass RRT:
        RRT() except +
        RRT(double startpointx, double startpointy, double goalpointx,double goalpointy, double* _locationList,double stepsize,double disTh,int attemtps ) except +
        double* FindPath(double the_path[])
