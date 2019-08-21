#include <iostream>
#include "rrtTest.h"

using namespace myrrt;
int main()
{
    double locationList[] = {10,10,20,20,30,30,170,200,50,50,60,60,70,70,80,80,90,90,100,100,110,110,120,120,130,130,140,140,150,150}; 
    RRT myrrt(-200,200,200,200,locationList,40,20,10000);
    double *path;
	myrrt.FindPath(path);
    std::cout<<path[0]<<std::endl;
	getchar();
}


