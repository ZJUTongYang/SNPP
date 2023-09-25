#pragma once

#include <utility>
#include <vector>
#include "map.h"

//void kim_main(std::pair<double, double> current_position);


namespace Kim2013 {

	typedef std::vector<int> Indices;

	class KimNode
	{
	public:
		int x_;
		int y_;
		double g_;
		double h_;
		std::vector<int> hindex_;

		int index_;
		int fatherindex_;

		bool open_;

//		std::vector<int> adj_kimnode_;

		//	std::vector<int> predecessors_; // This is for non-self-crossing checking. 

		KimNode()
		{
			g_ = 0;
			hindex_.clear();
			fatherindex_ = -1;
			open_ = true;
		}
		KimNode(int x, int y, double g, double h, std::vector<int> hindex, int index, int fatherindex)
		{
			x_ = x;
			y_ = y;
			g_ = g;
			h_ = h;
			hindex_ = hindex;

			index_ = index;
			fatherindex_ = fatherindex;

//			adj_kimnode_.clear();
			open_ = true;
		}
		KimNode(const KimNode& a)
		{
			x_ = a.x_;
			y_ = a.y_;
			g_ = a.g_;
			h_ = a.h_;

			hindex_ = a.hindex_;

			index_ = a.index_;
			fatherindex_ = a.fatherindex_;

//			adj_kimnode_ = a.adj_kimnode_;

			open_ = a.open_;
		}
		void operator=(const KimNode& a)
		{
			x_ = a.x_;
			y_ = a.y_;
			g_ = a.g_;
			h_ = a.h_;

			hindex_ = a.hindex_;

			index_ = a.index_;
			fatherindex_ = a.fatherindex_;

//			adj_kimnode_ = a.adj_kimnode_;

			open_ = a.open_;
		}

		void sons(std::vector<std::pair<int, int> >& A)
		{
			A.clear();
			A.emplace_back(std::pair<int, int>(x_ - 1, y_ - 1));
			A.emplace_back(std::pair<int, int>(x_ - 1, y_));
			A.emplace_back(std::pair<int, int>(x_ - 1, y_ + 1));
			A.emplace_back(std::pair<int, int>(x_, y_ - 1));
			A.emplace_back(std::pair<int, int>(x_, y_ + 1));
			A.emplace_back(std::pair<int, int>(x_ + 1, y_ - 1));
			A.emplace_back(std::pair<int, int>(x_ + 1, y_));
			A.emplace_back(std::pair<int, int>(x_ + 1, y_ + 1));
		}
	};

	struct PathSolution
	{
		std::vector<std::pair<double, double> > path_;
		double path_cost_;
		int path_node_index_;
		PathSolution(const std::vector<std::pair<double, double> >& the_path,
			double the_path_cost, int the_path_node_index)
		{
			path_.assign(the_path.begin(), the_path.end());
			path_cost_ = the_path_cost;
			path_node_index_ = the_path_node_index;
		}
		std::vector<int> h_signature_;
	};

	class Kim2013 {
	public:
		Kim2013(const Costmap* pMap, bool non_selfcrossing, int K, int option, 
			std::pair<double, double> robot_position, std::pair<double, double> goal_position);

//		bool C2LPlanning(const KimNode& startnode, const std::pair<int, int>& goalLocation, std::vector<std::pair<int, int> >& result_path, KimNode& goalNode);

		int findEqualHCNode(const std::vector<int>& newHindex, const std::vector<KimNode>& theStorage, const std::vector<std::vector<Indices> >& theI, int son_x, int son_y);

		bool isANonhomotopicPath(std::vector<int> new_hindex);

		bool isEqualHC(const std::vector<int>& newH, const std::vector<KimNode>& theStorage, const std::vector<std::vector<Indices> >& theI, int son_x, int son_y);

		bool isSelfCrossing(const std::vector<std::pair<int, int> >& P);

		void tracePath(const KimNode& goalnode, std::vector<std::pair<int, int> >& result_path, const std::vector<KimNode>& theStorage,
			const KimNode& startnode);

		// YT: this is for pre-constructed configuration space. For ICRA24, we only use local storage (i.e., every time the planning is finished, the storage is cleared)
//		std::vector<KimNode> Storage_;

	private:
		std::vector<KimNode> Queue_;
		std::vector<std::vector<Indices> > I_;
		std::pair<int, int>  start_point_;
		std::vector<PathSolution> path_solutions_;
	};

} // end namespace Kim2013