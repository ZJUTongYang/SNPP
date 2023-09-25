#pragma once
//#ifndef _DEFINITIONS_
//#define _DEFINITIONS_
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

// Claim some pre-definitions
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// in some float-point calculation, we need this threshold
#ifndef RAYSTAR_EPS
#define RAYSTAR_EPS 0.00000000000001
#endif

#define RECORD_RAYSTAR_LOG true
#define RECORD_PATH_LOG true


#define USE_OURS_BUT_NOT_EXISTING true

class Param {
public:
	
	int xsize_;// = MAX_X_SIZE;
	int ysize_;// = MAX_Y_SIZE;
	
};

struct Cross
{
	int crossing_index_;
	std::pair<double, double> intersection_;
	int the_index_of_obs_that_the_crossing_on_;
	int the_vertex_index_of_s_of_the_edge_that_o_hits_;
	int the_vertex_index_of_g_of_the_edge_that_o_hits_;

	double distance_to_c_power2;

	Cross(int crossing_index_, std::pair<double, double> intersection_, 
		int the_index_of_obs_that_the_crossing_on_, 
		int the_vertex_index_of_s_of_the_edge_that_o_hits_, int the_vertex_index_of_g_of_the_edge_that_o_hits_)
	{
		crossing_index_ = crossing_index_;
		intersection_ = intersection_;
		the_index_of_obs_that_the_crossing_on_ = the_index_of_obs_that_the_crossing_on_;
		the_vertex_index_of_s_of_the_edge_that_o_hits_ = the_vertex_index_of_s_of_the_edge_that_o_hits_;
		the_vertex_index_of_g_of_the_edge_that_o_hits_ = the_vertex_index_of_g_of_the_edge_that_o_hits_;

	}
};

// YT: 2023.8.24
struct Candidate
{
	int Nindex_;
	int Cindex_;	
	double Fcost_;

	Candidate(const Candidate& a)
	{
		Nindex_ = a.Nindex_;
		Cindex_ = a.Cindex_;
		Fcost_ = a.Fcost_;
	}
	Candidate(int node_index, int child_index, double cost)
	{
		Nindex_ = node_index;
		Cindex_ = child_index;
		Fcost_ = cost;
	}

	// bool operator()(const Candidate& a, const Candidate& b) const
	// {
	// 	return a.Fcost_ > b.Fcost_;
	// }
};



//#endif

