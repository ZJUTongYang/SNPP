#include <iostream>
#include <fstream>
#include <queue>
#include "map.h"
#include "Planner.h"
#include "Node.h"
#include "Definitions.h"
#include "atom_functions.h"

using namespace std;


bool Planner::isInGovernedRegion(std::pair<double, double> goal_position)
{
	// To Implement: check whether the visibility region of the latest node covers the goal position
	auto& governed_region = N_.back().V_;

	if (pnpoly(governed_region, goal_position.first, goal_position.second))
	{
		return true;
	}
	return false;
}

Planner::Planner(Costmap* pMap, std::string log_filename, bool non_selfcrossing, int K, 
	std::pair<double, double> start_position, std::pair<double, double> goal_location):
	non_selfcrossing_(non_selfcrossing), K_(K){
	xsize_ = pMap->xsize_;
	ysize_ = pMap->ysize_;

	// YT: We create the first node
	int Nindex = 0;
	Node start_node(pMap, start_position);
	start_node.start_angle_ = 0;
	start_node.end_angle_ = 2 * M_PI;

	start_node.Gcost_ = 0;
	start_node.Hcost_ = distance(start_node.seed_, goal_location);
	start_node.Fcost_ = start_node.Gcost_ + start_node.Hcost_;
	start_node.local_shortest_path_.emplace_back(start_position);
	start_node.path_node_index_.emplace_back(start_node.Nindex_);

	start_node.Expand(pMap, goal_location);

	// We should compare Children
	auto comp = [](const Candidate& a, const Candidate& b) { return a.Fcost_ > b.Fcost_; };// for determining whether exchanging or not

	// N_.at(Nindex) = start_node;
	N_.emplace_back(start_node);


	// We set up the priority queue
	Open_list_.clear();
	for (auto iter = N_.back().C_.begin(); iter != N_.back().C_.end(); ++iter)
	{
		Open_list_.emplace_back(
			Candidate(iter->Nindex_, iter->Cindex_, iter->c_gcost_ + iter->c_hcost_)
		);
	}
	std::make_heap(Open_list_.begin(), Open_list_.end(), comp);


	// We might be able to find a new solution
	if (isInGovernedRegion(goal_location))
	{
		std::vector<std::pair<double, double> > locally_shortest_path(N_.back().local_shortest_path_);
		locally_shortest_path.emplace_back(goal_location);
		double new_path_length = N_.back().Gcost_ + distance(N_.back().seed_, goal_location);

		// We save the Nindex of nodes on this local_shortest_path_, for visualizing V of each node
		std::vector<int> path_node_index(N_.back().path_node_index_);
		path_node_index.emplace_back(-2);// set goal index is -2

		path_solutions_.emplace_back(PathSolution(locally_shortest_path, new_path_length, path_node_index));

		if (path_solutions_.size() >= K_ &&
			path_solutions_[K_ - 1].path_cost_ < Open_list_[0].Fcost_)
		{
			return ;
		}
	}

#if RECORD_RAYSTAR_LOG
	{
		std::ofstream ofs;
		ofs.open(log_filename);
		ofs << N_.back().Nindex_ << " " << N_.back().fatherindex_ << " "
			<< N_.back().seed_.first << " " << N_.back().seed_.second << " "
			<< N_.back().Gcost_ << " "
			<< std::endl;

		// We store the governed region
		for (auto iter = N_.back().V_.begin(); iter != N_.back().V_.end(); ++iter)
		{
			ofs << iter->first << " " << iter->second << " ";
		}
		ofs << std::endl;

		ofs.close();
	}
#endif

	// YT: We iteratively select the next best child and expand it into a new node

	int count = 0;

	while (!Open_list_.empty())
	{
		//if (++count == 27)
		//{
		//	break;
		//}
		Candidate best_candidate = Open_list_[0];
		std::pop_heap(Open_list_.begin(), Open_list_.end(), comp);
		Open_list_.pop_back();

		int parent_index = best_candidate.Nindex_;
		int child_index = best_candidate.Cindex_;
		std::pair<double, double> new_source_point = N_[parent_index].C_[child_index].c_;
		int new_node_index = N_.size();

		Child& as_a_child = N_[parent_index].C_[child_index];

		Node new_node(pMap, new_source_point);

		N_.emplace_back(new_node);
		N_.back().Nindex_ = new_node_index;
		N_.back().seed_obs_index_ = as_a_child.c_obs_index_;
		N_.back().seed_ver_index_ = as_a_child.c_ver_index_;
		N_.back().start_angle_ = as_a_child.start_angle_;
		N_.back().end_angle_ = as_a_child.end_angle_;
		N_.back().fatherindex_ = parent_index;
		if (as_a_child.is_a_left_gap_)
		{
			N_.back().start_o_ = as_a_child.o_;
			N_.back().start_o_obs_index_ = as_a_child.o_obs_index_;
			N_.back().start_o_ver_index_ = as_a_child.o_ver_index_;

			int next_obs_index = as_a_child.c_obs_index_;
			int next_ver_index = (as_a_child.c_ver_index_ == pMap->obs_[as_a_child.c_obs_index_].ordered_vertices_.size() - 1) ? 
								0 : as_a_child.c_ver_index_ + 1;
			std::pair<double, double> next_obs_vertex = pMap->obs_[next_obs_index].ordered_vertices_[next_ver_index].loc_;
			N_.back().end_o_ = next_obs_vertex;
			N_.back().end_o_obs_index_ = as_a_child.c_obs_index_;
			N_.back().end_o_ver_index_ = next_ver_index;

		}
		else
		{
			int prev_obs_index = as_a_child.c_obs_index_;
			int prev_ver_index = (as_a_child.c_ver_index_ == 0) ?
				pMap->obs_[as_a_child.c_obs_index_].ordered_vertices_.size() - 1 : as_a_child.c_ver_index_ - 1;
			std::pair<double, double> prev_obs_vertex = pMap->obs_[prev_obs_index].ordered_vertices_[prev_ver_index].loc_;

			N_.back().start_o_ = prev_obs_vertex;
			N_.back().start_o_obs_index_ = as_a_child.c_obs_index_;
			N_.back().start_o_ver_index_ = prev_ver_index;

			N_.back().end_o_ = as_a_child.o_;
			N_.back().end_o_obs_index_ = as_a_child.o_obs_index_;
			N_.back().end_o_ver_index_ = as_a_child.o_ver_index_;
		}

		N_.back().as_a_child_left_gap_ = as_a_child.is_a_left_gap_;

		N_.back().Gcost_ = as_a_child.c_gcost_;
		N_.back().Hcost_ = as_a_child.c_hcost_;
		N_.back().Fcost_ = N_.back().Gcost_ + N_.back().Hcost_;
		N_.back().local_shortest_path_.assign(N_[parent_index].local_shortest_path_.begin(), 
			N_[parent_index].local_shortest_path_.end());
		N_.back().local_shortest_path_.emplace_back(N_.back().seed_);
		N_.back().path_node_index_.assign(N_[parent_index].path_node_index_.begin(),
			N_[parent_index].path_node_index_.end());
		N_.back().path_node_index_.emplace_back(N_.back().Nindex_);


		N_.back().Expand(pMap, goal_location);

		for (auto iter = N_.back().C_.begin(); iter != N_.back().C_.end(); ++iter)
		{
			if (!non_selfcrossing_ || !isSelfCrossing(iter->c_, N_.back().seed_, N_.back().local_shortest_path_))
			{
				Open_list_.emplace_back(
					Candidate(iter->Nindex_, iter->Cindex_, iter->c_gcost_ + iter->c_hcost_)
				);
				std::push_heap(Open_list_.begin(), Open_list_.end(), comp);
			}
		}


#if RECORD_RAYSTAR_LOG
		{
			std::ofstream ofs;
			ofs.open(log_filename, std::ios::app);
			ofs << N_.back().Nindex_ << " " << N_.back().fatherindex_ << " "
				<< N_.back().seed_.first << " " << N_.back().seed_.second << " "
				<< N_.back().Gcost_ << " "
				<< std::endl;

			// We store the governed region
			for (auto iter = N_.back().V_.begin(); iter != N_.back().V_.end(); ++iter)
			{
				ofs << iter->first << " " << iter->second << " ";
			}
			ofs << std::endl;

			ofs.close();
		}
#endif

		// We might be able to find a new solution
		if (isInGovernedRegion(goal_location))
		{
			std::vector<std::pair<double, double> > locally_shortest_path(N_.back().local_shortest_path_);
			locally_shortest_path.emplace_back(goal_location);
			double new_path_length = N_.back().Gcost_ + distance(N_.back().seed_, goal_location);

			// We save the Nindex of nodes on this local_shortest_path_, for visualizing V of each node
			std::vector<int> path_node_index(N_.back().path_node_index_);
			path_node_index.emplace_back(-2);// set goal index is -2

			//path_solutions_.emplace_back(locally_shortest_path);
			//path_cost_.emplace_back(new_path_length);
			path_solutions_.emplace_back(PathSolution(locally_shortest_path, new_path_length, path_node_index));

			//// We check whether we have got an enough number of non-homotopic paths
			//std::sort(path_solutions_.begin(), path_solutions_.end(),
			//	[](const auto& a, const auto& b) {return a.path_cost_ > b.path_cost_; });

			//if (path_solutions_.size() >= K && 
			//	path_solutions_[K_ - 1].path_cost_ < Open_list_[0].Fcost_)
			if(path_solutions_.size() >= K)
			{
				break;
			}
		}


	}
}

