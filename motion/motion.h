#ifndef motion_H
#define motion_H

namespace mymotion {
	class MOTION {
		public:
			MOTION();
			~MOTION();
			MOTION(double reallocation_x, double reallocation_y, double orientation, double the_path[], double sampling_time, double set_v_x,double set_v_y, double begin_x, double begin_y);
//reallocation_x,y: image get blue 0, x and  y 
//orientation: tangent of path
//the_path[]: planned path
//sampling_time: the duration between sampling
//set_v: expected velocity, arbitrary 
//begin_x,y: the start point 

			double get_v_x(double pre_x, double reallocation_x, double sampling_time);
//pre_x,y: two images got by sampling
			double get_v_y(double pre_y, double reallocation_y, double sampling_time);
			double get_a_x(double pre_v_x, double real_v_x, double sampling_time);
			double get_a_y(double pre_v_y, double real_v_y, double sampling_time);
			double get_omega(double pre_orientation, double real_orientation, double sampling_time);
			
			bool is_in_point(double realloction_x, double reallocation_y, double expectedx, double expectedy);

//The core method, get updated vx, vy and omega
			void get_motion();

			double updated_vx();
			double updated_vy();
			double updated_omega();

		private:
			double pre_x;
			double pre_y;
			double pre_v_x;
			double pre_v_y;
			double pre_ori;
			double a_x;
			double a_y;
			double real_x;
			double real_y;
			double real_v_x;
			double real_v_y;
			double real_ori;
			double omega;
			double sampletime;
			double set_v_x;
			double set_v_y;
			double theta;
			double begin_x;
			double begin_y;
		//the result
			double my_v_x;
			double my_v_y;
			double my_omega;
	};
}

#endif // !motion_h

