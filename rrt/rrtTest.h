#include <iostream>
#include <time.h>
//#include "vision_detection.pb.h"

//Vision_DetectionFrame Frame;


namespace myrrt{
    struct point {
	    double x;
	    double y;
    };

    struct node {
	    point p;
	    int prev;
	    double dist;
    };

    class RRT{
        public:
			RRT();
			RRT(double startpointx, double startpointy, double goalpointx,double goalpointy, double* _locationList,double stepsize,double disTh,int attemtps, int radius = 220);
			~RRT();

            double distance(point p1, point p2);
            bool is_feasible(point p, double x[], double y[]);
			bool checkPath(point n, point newPos, double x[], double y[]);
			double myrand();
			int random(int x);
			double* FindPath();
			void get_rrt(double pone[], double ptwo[], double pthree[], double pfour[], int mycounter);
			void get_rrt_time(double leng_time[],double theta_time[]);
			void get_rrt_length(double leng[]);
			void get_rrt_theta(double theta[]);
/*
			void set_startpoint(double px, double py);
			void set_goalpoint(double px, double py);
			void set_locationList(double* _locationList);
			void set_stepsize(double stepsize);
			void set_disTh(double disTh);
			void set_maxFailedAttempts(int attempt);
*/
//		private:
			point startpoint;
			point goalpoint;
			double* locationList;
			double stepsize;
			double disTh;
			int maxFailedAttempts;
			double* path;
			double* prev_path;
			int pathlength;
			int length;
			int rrt_count;
			int radius;
			double*p1, *p2, *p3, *p4;
    };
}
