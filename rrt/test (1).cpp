#include <iostream>
#include "rrtTest.h"
using namespace std;
using namespace myrrt;
int main()
{
    double locationList[] = {10,10,20,20,30,30,170,200,50,50,60,60,70,70,80,80,90,90,100,100,110,110,120,120,130,130,140,140,100,200}; 
    RRT myrrt(-200,200,200,200,locationList,40,20,10000);
	
    std::cout<<myrrt.FindPath()[0]<<std::endl;
	
	std::cout << myrrt.pathlength << std::endl;
	double* length = (double*)malloc((myrrt.pathlength-1) * sizeof(double));
	double* theta= (double*)malloc((myrrt.pathlength-1) * sizeof(double));
	double* length1 = (double*)malloc((myrrt.pathlength - 1) * sizeof(double));
	double* theta1 = (double*)malloc((myrrt.pathlength - 1) * sizeof(double));
	for (int i = 0; i < myrrt.pathlength - 1; i++) {
		length[i] = sqrt((myrrt.path[2 * i] - myrrt.path[2 * i + 2])*(myrrt.path[2 * i] - myrrt.path[2 * i + 2]) + (myrrt.path[2 * i+1] - myrrt.path[2 * i + 3])*(myrrt.path[2 * i+1] - myrrt.path[2 * i + 3]));
		theta[i] = atan2((myrrt.path[2 * i + 1] - myrrt.path[2 * i + 3]), (myrrt.path[2 * i] - myrrt.path[2 * i + 2]));
	}
	for (int i = 0; i < myrrt.pathlength - 1; i++) {
		length1[i] = length[myrrt.pathlength - 2 - i];
		theta1[i] = theta[myrrt.pathlength - 2 - i];
	}
/*	for (int i = 0; i < myrrt.pathlength - 1; i++) {
		cout << length1[i] << endl;
		cout << theta1[i] << endl;
	}*/
	double* t1= (double*)malloc((myrrt.pathlength-1) * sizeof(double));
	double* t2= (double*)malloc((myrrt.pathlength-2) * sizeof(double));
	for(int i = 0; i < myrrt.pathlength-1; i++) {
		t1[i] = length1[i] / 50;
	}

	for (int i = 0; i < myrrt.pathlength - 2; i++) {
		t2[i] = (theta1[i + 1] - theta1[i]) / 0.1;
	}
	for (int i = 0; i < myrrt.pathlength-1; i++) {
		cout << t1[i]<<endl;
	}
	for (int i = 0; i < myrrt.pathlength-2; i++) {
		cout << t2[i]<<endl;
	}
	cout << "-----------------------"<<endl;
	for (int i = 0; myrrt.p1[i] != 1000; i++) {
		cout << myrrt.p1[i] << "," << myrrt.p2[i] << "   " << myrrt.p3[i] << "," << myrrt.p4[i] << endl;
	}
	getchar();
}


