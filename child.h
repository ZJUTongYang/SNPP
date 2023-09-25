#pragma once
#include <utility>

class Costmap;
struct Child
{
	int Nindex_;
	int Cindex_;
	double start_angle_;
	double end_angle_;

	double gap_orientation_; // to store the ray direction, for sorting the children

	std::pair<double, double> c_;// The obstacle vertex that form the gap
	std::pair<double, double> o_;

	int c_obs_index_;
	int c_ver_index_;
	int o_obs_index_;
	int o_ver_index_;

	bool is_a_left_gap_;

	double c_gcost_; // The accumulated distance from the starting location to c
	double c_hcost_; // The heuristic

	Child(int nindex, int cindex,
		std::pair<double, double> c_location, bool is_left_gap)
	{
		Nindex_ = nindex;
		Cindex_ = cindex;
		c_ = c_location;
		is_a_left_gap_ = is_left_gap;
	}

	Child(const Child& a)
	{
		Nindex_ = a.Nindex_;
		Cindex_ = a.Cindex_;
		start_angle_ = a.start_angle_;
		end_angle_ = a.end_angle_;
		gap_orientation_ = a.gap_orientation_;
		c_ = a.c_;
		o_ = a.o_;

		c_obs_index_ = a.c_obs_index_;
		c_ver_index_ = a.c_ver_index_;
		o_obs_index_ = a.o_obs_index_;
		o_ver_index_ = a.o_ver_index_;

		is_a_left_gap_ = a.is_a_left_gap_;
		c_gcost_ = a.c_gcost_;
		c_hcost_ = a.c_hcost_;

	}

};