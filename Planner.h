#pragma once
#include <iostream>
#include "Node.h"
#include "map.h"
#include "Definitions.h"
#include "child.h"

using namespace std;

struct PathSolution
{
	std::vector<std::pair<double, double> > path_;
	double path_cost_;
	std::vector<int> path_node_index_;
	PathSolution(const std::vector<std::pair<double, double> >& the_path,
		double the_path_cost, std::vector<int> the_path_node_index)
	{
		path_.assign(the_path.begin(), the_path.end());
		path_cost_ = the_path_cost;
		path_node_index_.assign(the_path_node_index.begin(), the_path_node_index.end());
	}
	std::vector<int> h_signature_;
};

class Planner
{
public:
	int xsize_;
	int ysize_;
	
	Planner(Costmap* pMap, std::string log_filename, bool non_selfcrossing, int K,
		std::pair<double, double> start_position, std::pair<double, double> goal_position);

	bool isInGovernedRegion(std::pair<double, double> goal_position);


	std::vector<Candidate> Open_list_;
	std::vector<Node> N_;
	
	std::vector<PathSolution> path_solutions_;

	int K_;

	const bool non_selfcrossing_;

};
	


