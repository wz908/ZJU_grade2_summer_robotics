#include "motion.h"
#include <iostream>
#include <math.h>

namespace mymotion {
	MOTION::MOTION(){};
	MOTION::~MOTION() {};

	
	
	MOTION::MOTION(double reallocation_x, double reallocation_y, double orientation, double the_path[], double sampling_time, double set_v_x, double set_v_y, double begin_x, double begin_y) {
		this->pre_x = this->real_x;
		this->pre_v_y = this->real_v_y;
		this->pre_ori = this->real_ori;
		this->pre_v_x = this->real_v_x;
		this->pre_y = this->real_y;
		this->sampletime = sampling_time;
		this->real_x = reallocation_x;
		this->real_y = reallocation_y;
		this->real_v_x = get_v_x(this->pre_x, this->real_x, sampling_time);
		this->real_v_y = get_v_y(this->pre_y, this->real_y, sampling_time);
		this->theta = orientation;
		this->a_x = get_a_x(this->pre_v_x, this->real_v_x, sampling_time);
		this->a_y = get_a_y(this->pre_v_y, this->real_v_y, sampling_time);

		this->omega = get_omega(this->pre_ori, this->real_ori, sampling_time);
		this->set_v_x = set_v_x;
		this->set_v_y = set_v_y;
		this->begin_x = begin_x;
		this->begin_y = begin_y;
		this->my_omega = 0;
		

	}
	double distance(double x1, double y1, double x2, double y2) {
		return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
	}
	double MOTION::get_v_x(double pre_x,double reallocation_x, double sampling_time) {
		return((reallocation_x - pre_x) / sampling_time);
	}
	double MOTION::get_v_y(double pre_y, double reallocation_y, double sampling_time) {
		return((reallocation_y - pre_y) / sampling_time);
	}
	double MOTION::get_a_x(double pre_v_x, double real_v_x, double sampling_time) {
		return((real_v_x - pre_v_x) / sampling_time);
	}
	double MOTION::get_a_y(double pre_v_y, double real_v_y, double sampling_time) {
		return((real_v_y - pre_v_y) / sampling_time);
	}
	double MOTION::get_omega(double pre_ori, double real_ori, double sampling_time) {
		return((real_ori - pre_ori) / sampling_time);
	}

	bool MOTION::is_in_point(double realloction_x, double reallocation_y, double expectedx, double expectedy) {
		return ((realloction_x - expectedx)*(realloction_x - expectedx) + (reallocation_y - expectedy)*(reallocation_y - expectedy)) < 225;
	}

	void MOTION::get_motion() {
		this->my_v_x = this->set_v_x + (set_v_x*sampletime - (this->real_x - begin_x));
		this->my_v_y = this->set_v_y + (set_v_y*sampletime - (this->real_y - begin_y));
	}

	double MOTION::updated_vx(){return this->my_v_x;}
	double MOTION::updated_vy(){return this->my_v_y;}
	double MOTION::updated_omega(){return this->my_omega;}
}