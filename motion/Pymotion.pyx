# distutils: language = c++
from motion cimport MOTION
import numpy as np 
cimport numpy as np 

cdef class PyMOTION:
    cdef MOTION c_motion

    def __cinit__(self,double reallocation_x, double reallocation_y, double orientation, np.ndarray[np.double_t,ndim=1] the_path, double sampling_time, double set_v_x,double set_v_y, double begin_x, double begin_y):
        self.c_motion= MOTION(reallocation_x,reallocation_y,orientation,<double*> the_path.data,sampling_time,set_v_x,set_v_y,begin_x,begin_y)

    def get_motion(self):
        self.c_motion.get_motion()

    def updated_vx(self):
        vx = self.c_motion.updated_vx()
        return vx 

    def updated_vy(self):
        vy = self.c_motion.updated_vy()
        return vy 

    def updated_omega(self):
        omega = self.c_motion.updated_omega()
        return omega
