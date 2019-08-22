cdef extern from "motion.cpp":
    pass

cdef extern from "motion.h" namespace "mymotion":
    cdef cppclass MOTION:
        MOTION() except +
        MOTION(double reallocation_x, double reallocation_y, double orientation, double the_path[], double sampling_time, double set_v_x,double set_v_y, double begin_x, double begin_y)except +
        void get_motion()
        double updated_vx()
        double updated_vy()
        double updated_omega()

        