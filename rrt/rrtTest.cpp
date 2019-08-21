﻿#include "rrtTest.h"
#include <iostream>
#include <time.h>
#include <vector>
#include <math.h>
using namespace std;
using std::cin;
using std::cout;
using std::cerr;
using std::endl;
namespace myrrt{
RRT::RRT(){}
RRT::~RRT(){}
RRT::RRT(double startpointx, double startpointy, double goalpointx,double goalpointy, double _locationList[],double stepsize,double disTh,int attempts )
{
	this->startpoint.x = startpointx;
	this->startpoint.y = startpointy;
	this->goalpoint.x = goalpointx;
	this->goalpoint.y = goalpointy;
	this->locationList = _locationList;
	this->stepsize = stepsize;
	this->disTh = disTh;
	this->maxFailedAttempts = attempts;
	cout<<"the class is initialized"<<endl;
}

double RRT::distance(point p1, point p2) {return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));}
double RRT::myrand() {return (double)random(100) / 100;}
int RRT::random(int x){return rand()%x;}

bool RRT::is_feasible(point p, double x[], double y[]) {
	bool feasible = true;
	for (int i = 0; i < 15; i++) {
		if ((p.x - x[i])*(p.x - x[i]) + (p.y - y[i]) *(p.y - y[i]) <= 400) {
			feasible = false;
		}
	}
	return feasible;
}

bool RRT::checkPath(point n, point newPos, double x[], double y[]) {
	bool feasible = true;
	double theta = atan2(newPos.y - n.y, newPos.x - n.x);
	for (double r = 0; r <= distance(n, newPos); r += 0.5) {
		point posCheck;
		posCheck.x = n.x + r * cos(theta);
		posCheck.y = n.y + r * sin(theta);
		if (!is_feasible(posCheck, x, y)) {
			feasible = 0;
			break;
		}
	}
	return feasible;
}

double*  RRT::FindPath(double the_path[]) {
	srand((int)time(NULL));
	point closestNode;
	vector<node> rrt;

	//define the robots
	double x[15];
	double y[15];
/*
	for (int i = 0; i <= 7; i++) {
		x[i] = Frame.robots_blue(i).x() / 10;
		y[i] = Frame.robots_blue(i).y() / 10;
	}
	for (int i = 1; i <= 7; i++) {
		x[7 + i] = Frame.robots_yellow(i).x() / 10;
		y[7 + i] = Frame.robots_yellow(i).y() / 10;
	}
*/
	for (int i=0; i<15;i++)
	{
		x[i]=this->locationList[2*i];
		y[i]=this->locationList[2*i+1];
	}
	bool display = true;
	if (!is_feasible(this->startpoint, x, y)) {
		cout << ("this->startpoint lies on an obstacle or outside map");
		system("pause");
		return 0;
	}
	if (!is_feasible(this->goalpoint, x, y)) {
		cout << ("goalpoint lies on an obstacle or outside map");
		system("pause");
		return 0;
	}


	node sourcenode;
	point sample;
	sourcenode.p = this->startpoint;
	sourcenode.prev = -1;

	rrt.push_back(sourcenode);
	int failedAttempts = 0;
	int counter = 0;
	bool pathFound = false;
	int min_index;
	int min_index1;
	double min;
	double theta;
	point newPoint;
	node newNode;

	while (failedAttempts <= this->maxFailedAttempts) {
		if (myrand() < 0.4) {
			sample.x = (2 * myrand() - 1) * 300;
			sample.y = (2 * myrand() - 1) * 225;
		}
		else {
			sample = this->goalpoint;
		}
		vector<node>::iterator pt = rrt.begin();
		while (pt != rrt.end()) {
			pt->dist = distance(pt->p, sample);
			pt++;
		}
		min = rrt.begin()->dist;

		min_index = 0;
		min_index1 = 0;
		pt = rrt.begin();
		while (pt != rrt.end()) {
			if (pt->dist < min) {
				min = pt->dist;
			}
			pt++;
		}
		pt = rrt.begin();
		while (pt != rrt.end()) {
			if (pt->dist != min) {
				min_index++;
			}
			if (pt->dist == min) {
				break;
			}
			pt++;
		}

		closestNode = rrt[min_index].p;
		theta = atan2(sample.y - closestNode.y, sample.x - closestNode.x);

		newPoint.x = double((closestNode.x + this->stepsize * cos(theta)));
		newPoint.y = double((closestNode.y + this->stepsize * sin(theta)));
		if (!checkPath(closestNode, newPoint, x, y)) {
			failedAttempts = failedAttempts + 1;
			continue;
		}
		if (distance(newPoint, this->goalpoint) < this->disTh) {
			pathFound = true;
			newNode.p = this->goalpoint;
			newNode.prev = min_index;
			rrt.push_back(newNode);
			break;
		}

		pt = rrt.begin();
		while (pt != rrt.end()) {
			pt->dist = distance(pt->p, newPoint);
			pt++;
		}
		min = rrt.begin()->dist;
		pt = rrt.begin();
		while (pt != rrt.end()) {
			if (pt->dist < min) {
				min = pt->dist;
			}
			pt++;
		}
		pt = rrt.begin();
		while (pt != rrt.end()) {
			if (pt->dist != min) {
				min_index1++;
			}
			if (pt->dist == min) {
				break;
			}
			pt++;
		}
		if (distance(newPoint, rrt[min_index1].p) < this->disTh) {
			failedAttempts = failedAttempts + 1;
			continue;
		}

		newNode.p = newPoint;
		newNode.prev = min_index;
		rrt.push_back(newNode);
		failedAttempts = 0;
		if (display) {
			//cout << "(" << closestNode.x << "," << closestNode.y << ")" << "(" << newPoint.x << "," << newPoint.y << ")" << endl;
			counter++;
		}
	}
	if (display&&pathFound) {
			//cout << "(" << closestNode.x << "," << closestNode.y << ")" << "(" << goal.x << "," << goal.y << ")" << endl;
		counter++;
	}
	if (!pathFound) {
		cout << "no path found. maximum attempts reached" << endl;
	}

	vector<node>::iterator pt1 = rrt.begin();
	int length = 0;
	for (; pt1 != rrt.end(); pt1++) {
		length++;
	}

	vector<node>rrt1, rrt2;
	rrt1.push_back(rrt[length - 1]);
	rrt2.push_back(rrt[length - 1]);
	int cnt = 0;
	while (rrt1[cnt].prev > 0) {
		rrt1.push_back(rrt[rrt1[cnt].prev]);
		cnt++;
	}
	rrt1.push_back(rrt[0]);
	//vector<node>::iterator pt2 = updaterrt.begin();
	int cnt1 = 2, cnt2 = 0;
	while (rrt1[cnt1].p.x != this->startpoint.x) {
		if (checkPath(rrt1[cnt1].p, rrt1[cnt2].p, x, y)) {
			cnt1++;
		}
		if (checkPath(rrt1[cnt1].p, rrt1[cnt2].p, x, y) == 0) {
			rrt2.push_back(rrt1[cnt1 - 1]);
			cnt2 = cnt1;
		}
	}
	rrt2.push_back(rrt[0]);
	vector<node>::iterator pt2 = rrt2.begin();

	int num = 0;
	for (; pt2 != rrt2.end(); pt2++) {
		cout << pt2->p.x << " " << pt2->p.y << endl;
		num++;
	}

	double* points = (double*)malloc(2*num * sizeof(double));
	pt2 = rrt2.begin();
	
	for (int i = 0; i < 2 * num; i++) {
		if (i % 2 == 0) {
			points[i] = pt2->p.x;
			the_path[i] = points[i];
		}
		if (i % 2 == 1) {
			points[i] = pt2->p.y;
			the_path[i] = points[i];
			pt2++;
		}
	}
	return points;
}

}