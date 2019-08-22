#include <iostream>
#include "motion.h"

using namespace mymotion;
int main()
{
    double reallocation_x = 5;
    double reallocation_y = 5.5;
    double begin_x = 0;
    double begin_y = 0;
    double sampling_time = 0.1;
    double the_path[] = {0,0,100,100} ;
    double orientation = 3.1415926/4;
    double set_v_x = 50;
	double set_v_y = 50;
    MOTION motion(reallocation_x,reallocation_y,orientation,the_path,sampling_time,set_v_x,set_v_y,begin_x,begin_y);
    motion.get_motion();
    std::cout<<motion.updated_vx()<<std::endl;
    std::cout<<motion.updated_vy()<<std::endl;
	getchar();
}