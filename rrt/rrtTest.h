#ifndef MYRRT_H
#define MYRRT_H
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
			RRT(double startpointx, double startpointy, double goalpointx,double goalpointy, double _locationList[],double stepsize,double disTh,int attemtps );
			~RRT();

            double distance(point p1, point p2);
            bool is_feasible(point p, double x[], double y[]);
			bool checkPath(point n, point newPos, double x[], double y[]);
			double myrand();
			int random(int x);
			double* FindPath(double the_path[]);

/*
			void set_startpoint(double px, double py);
			void set_goalpoint(double px, double py);
			void set_locationList(double* _locationList);
			void set_stepsize(double stepsize);
			void set_disTh(double disTh);
			void set_maxFailedAttempts(int attempt);
*/
		private:
			point startpoint;
			point goalpoint;
			double* locationList;
			double stepsize;
			double disTh;
			int maxFailedAttempts;
			double* path;

    };
}
#endif
