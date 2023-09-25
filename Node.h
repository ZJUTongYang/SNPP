#ifndef _NODE_
#define _NODE_

#include <utility>
#include <vector>
#include <string>
#include <list>
#include <iostream>
#include "Definitions.h"
#include "map.h"
#include "child.h"

using namespace std;

class Node
{
public:

	int Nindex_;

	// The critical point
	std::pair<double, double> seed_;
	int seed_obs_index_;
	int seed_ver_index_;

	double start_angle_;
	double end_angle_;
	int fatherindex_;

	std::pair<double, double> start_o_;
	int start_o_obs_index_;
	int start_o_ver_index_;
	std::pair<double, double> end_o_;
	int end_o_obs_index_;
	int end_o_ver_index_;

	bool as_a_child_left_gap_;

	double Gcost_;
	double Hcost_;
	double Fcost_;

	std::vector<Child> C_;

	std::vector< std::pair<double, double> > V_;

	std::vector<std::pair<double, double> > local_shortest_path_;
	std::vector<int> path_node_index_;
	
	// By default, the node is the first node
	Node(const Costmap* pMap, std::pair<double, double> position);

	Node(const Child& as_a_child);

	void constructV(const Costmap* pMap);

	void Expand(const Costmap* pMap, std::pair<double, double> goal_location);

//	void findO(std::pair<double, double> c_, const std::vector<Obs>& obs_);
	void findOForLastChild(const Costmap* pMap);

	bool isValidBranch(const Costmap* pMap, std::pair<double, double> c, 
		int c_obs_index, int c_ver_index, bool is_a_left_gap);
	
	//Node(const Node& a)
	//{
	//	Nindex_ = a.Nindex_;
	//	seed_ = a.seed_;
	//	start_angle_ = a.start_angle_;
	//	end_angle_ = a.end_angle_;

	//	Gcost_ = a.Gcost_;
	//	fatherindex_ = a.fatherindex_;

	//}
	//Node(int Nindex_, std::pair<double, double> seedpoint);

	//inline double adjustAngle(double theta)
	//{
	//	if(theta > start_angle_ && theta < end_angle_)
	//		return theta;
	//	if(theta < start_angle_)
	//	{
	//		theta = start_angle_ + normalize_angle_positive(theta - start_angle_);
	//		return theta;
	//	}
	//	std::cout << "YT: Angle error, we shouldn't reach here" << std::endl;
	//}

	//void tracePath(Path2D& result, int untilfather = 0);
	// void calAngle(std::pair<double, double> critical_points, Node node_father);
	void showDetails();

};


#endif