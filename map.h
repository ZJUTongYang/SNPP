#pragma once

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include "Definitions.h"

using namespace std;

struct ObstacleVertex{
	std::pair<double, double> loc_;
	// double inside_ori_;
	// std::pair<double, double> tangent_ori_;
	ObstacleVertex(double x, double y)
	{
		loc_.first = x;
		loc_.second = y;
	}
};

class Obs
{
public:
	std::pair<double, double> rep_point_;
	std::vector<std::pair<int, int> > grids_;
	std::vector<ObstacleVertex> ordered_vertices_;

	// YT 2023.8.28: When the obstacle is not convex and does not have small ``convex" vertex in the concave area, there might be problem
	// We store this for potential usage
	std::vector<std::pair<int, int> > detailed_ordered_vertices_;

};

class Costmap {
public:
	// replace the costmap_2d in ROS
	int xsize_;
	int ysize_;
    int* data_;
	Costmap(Param theParam, cv::Mat img);
    ~Costmap()
    {
        delete[] data_;
    }

	std::vector<int> calSwing(const std::vector<int>& oldH, int oldx, int oldy, int newx, int newy) const;

	void showMap(const Costmap& p, std::string window_name);

	std::vector<Obs> obs_;
	
	void generateCostMap(cv::Mat img);

	void sedFill();
	//
	//bool isLineSegmentCollisionFree(const std::pair<double, double>& s, 
	//	const std::pair<double, double>& g) const;


// YT: We identify each internal obstacle within the sedFill function, so there is no need to run this function
//	void getCriticalObs();

	// void ArrangeCriticalToObs();

	// std::vector<std::pair<double, double> > critical_points;
	// std::vector<double> critical_points_orientation;

	
};

