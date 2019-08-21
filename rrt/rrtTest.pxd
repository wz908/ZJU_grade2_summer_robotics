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
        RRT(double startpointx, double startpointy, double goalpointx,double goalpointy, double* _locationList,double stepsize,double disTh,int attemtps, int radius ) except +
        double* FindPath(double the_path[])
        int get_rrt(double pone[], double ptwo[], double pthree[], double pfour[])
        void get_rrt_time(double leng_time[],double theta_time[])
        void get_rrt_length(double leng[])
        void get_rrt_theta(double theta[])


