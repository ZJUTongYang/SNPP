#include "homotopy_aware_astar.h"
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include "atom_functions.h"

namespace Kim2013 {

	template <typename T> int sgn(T val)
	{
		return (T(0) < val) - (val < T(0));
	}

	Kim2013::Kim2013(const Costmap* pMap, bool non_selfcrossing, int K, int option, 
		std::pair<double, double> robot_position, std::pair<double, double> goal_position)
	{
		auto comp = [](const KimNode& a, const KimNode& b) {return a.g_ + a.h_ > b.g_ + b.h_; };

		int sizex = pMap->xsize_;
		int sizey = pMap->ysize_;

		KimNode startnode;
		startnode.index_ = 0;
		startnode.x_ = robot_position.first;
		startnode.y_ = robot_position.second;
		startnode.g_ = 0;
		startnode.h_ = distance(robot_position, goal_position);
		//	    startnode.h_ = sqrt( (snode.x_ - goalLocation.first)*(snode.x_ - goalLocation.first) + (snode.y_ - goalLocation.second)*(snode.y_ - goalLocation.second) ); // This is meaningless when precalculating the whole space
		startnode.hindex_.clear();
		startnode.fatherindex_ = -1;

		std::vector<KimNode> private_Storage; // The buffer for only this C2LPlanning
		private_Storage.clear();
		private_Storage.reserve(sizex*sizey);

		private_Storage.push_back(startnode);

		std::vector<std::vector<Indices> > private_I;
		private_I.resize(sizex);
		for (auto iter = private_I.begin(); iter != private_I.end(); ++iter)
		{
			iter->resize(sizey);
		}

		private_I[startnode.x_][startnode.y_].push_back(startnode.index_);

		std::vector<KimNode> private_Queue;
		private_Queue.reserve(sizex*sizey);
		private_Queue.clear();
		private_Queue.push_back(startnode);

		while (1)
		{
			if (private_Queue.empty())
			{
				std::cout << "The private_queue is empty" << std::endl;
				break;
			}

			// we choose the best element
			auto newnode = private_Queue[0];

			std::pop_heap(private_Queue.begin(), private_Queue.end(), comp);
			private_Queue.pop_back();
			private_Storage[newnode.index_].open_ = false;

			std::vector<std::pair<int, int> > result_temp;
			tracePath(newnode, result_temp, private_Storage, startnode);
			if (option == 2 && isSelfCrossing(result_temp))
			{
				continue;
			}

			if (newnode.x_ == goal_position.first && newnode.y_ == goal_position.second)
			{
				if (isANonhomotopicPath(newnode.hindex_))
				{
					// We need to make sure that we indeed find a non-homotopic path

					//std::cout << "number of expanded nodes: " << private_Storage.size() << std::endl;
					//std::cout << "YT: we have found one path.\n" << std::endl;
					std::vector<std::pair<int, int> > result_temp;
					tracePath(newnode, result_temp, private_Storage, startnode);

					// check self-crossing
					 if(option == 1 && isSelfCrossing(result_temp))
					 {
					     continue;
					 }
					std::vector<std::pair<double, double> > locally_shortest_path;
					locally_shortest_path.resize(result_temp.size());
					int n_temp = locally_shortest_path.size() - 1;
					for (unsigned int i = 0; i < locally_shortest_path.size(); ++i)
					{
						locally_shortest_path[i].first = result_temp[n_temp - i].first;
						locally_shortest_path[i].second = result_temp[n_temp - i].second;
					}

					double new_path_length = pathLength(locally_shortest_path);
					path_solutions_.emplace_back(PathSolution(locally_shortest_path, new_path_length, -1));
					path_solutions_.back().h_signature_.assign(newnode.hindex_.begin(), newnode.hindex_.end());

					if (path_solutions_.size() >= K)
					{
						break;
					}
				}


			}

			std::vector<std::pair<int, int> > son;
			newnode.sons(son);

			for (unsigned int j = 0; j < son.size(); ++j)
			{
				auto son_x = son[j].first;
				auto son_y = son[j].second;

				if (son_x < 0 || son_x >= sizex || son_y < 0 || son_y >= sizey)
					continue;

				if (pMap->data_[son_x + son_y * sizex] > 0)
					continue;

				std::vector<int> newHindex = pMap->calSwing(newnode.hindex_, newnode.x_, newnode.y_, son_x, son_y);

				double newg = newnode.g_ + sqrt((newnode.x_ - son_x)*(newnode.x_ - son_x) + (newnode.y_ - son_y)*(newnode.y_ - son_y));

				double newh = sqrt((goal_position.first - son_x)*(goal_position.first - son_x) + (goal_position.second - son_y)*(goal_position.second - son_y));

				int newsonindex = private_Storage.size();

				if (!isEqualHC(newHindex, private_Storage, private_I, son_x, son_y))
				{
					// std::cout << "no before: check son and son_g: (" << son_x << ", " << son_y << ", " << newg << ")" << std::endl;


					// If the node has not been visited
					// Or, we find non-homotopic paths to the son location, we preserve them both. I.e., we create a new node
					KimNode newson(son_x, son_y, newg, newh, newHindex, newsonindex, newnode.index_);
					private_Queue.emplace_back(newson);
					std::push_heap(private_Queue.begin(), private_Queue.end(), comp);

					private_Storage.emplace_back(newson);
					private_I[son_x][son_y].emplace_back(newsonindex);
				}
				else
				{
					//	                bool B = isEqualHC(newHindex, private_Storage, private_I, son_x, son_y);
										// We can find the equivalent Hindex in the I matrix, but we may carry out a shortcut
					int oldindex = findEqualHCNode(newHindex, private_Storage, private_I, son_x, son_y);
					// std::cout << "check oldindex: " << oldindex << std::endl;

					if (private_Storage[oldindex].g_ <= newg)
					{
						continue;
					}

					private_Storage[oldindex].g_ = newg;
					private_Storage[oldindex].h_ = newh;
					private_Storage[oldindex].hindex_ = newHindex; // In fact, it doesn't change
					private_Storage[oldindex].fatherindex_ = newnode.index_;

					// If the old node is open, then it is in the queue. We only change its cost and parent
					if (private_Storage[oldindex].open_)
					{
						// Find the position of the son in the queue, and update it
						auto iter = std::find_if(private_Queue.begin(), private_Queue.end(),
							[&](const KimNode& a) {return a.index_ == oldindex; });
						*iter = private_Storage[oldindex];
						std::make_heap(private_Queue.begin(), private_Queue.end(), comp);
					}
					else
					{
						// If the old node is closed, we may re-open it and push it into the queue
						private_Storage[oldindex].open_ = true;
						private_Queue.push_back(private_Storage[oldindex]);
						std::push_heap(private_Queue.begin(), private_Queue.end(), comp);
					}
				}
			}

			// std::cout << std::endl;
		}

	}

	bool Kim2013::isANonhomotopicPath(std::vector<int> new_hindex)
	{
		for (auto iter = path_solutions_.begin(); iter != path_solutions_.end(); ++iter)
		{
			auto& old_h = iter->h_signature_;
			auto& new_h = new_hindex;
			if (old_h.empty() && new_h.empty())
			{
				return false;
			}
			if (old_h.size() != new_h.size())
			{
				continue;
			}
			bool thesame = true;
			for (unsigned int i = 0; i < old_h.size(); ++i)
			{
				if (old_h[i] != new_h[i])
				{
					thesame = false;
					break;
				}

			}
			if (thesame)
			{
				return false;
			}
		}
		return true;
	}

	int Kim2013::findEqualHCNode(const std::vector<int>& newH, const std::vector<KimNode>& theStorage, const std::vector<std::vector<Indices> >& theI, int son_x, int son_y)
	{
		if (theI[son_x][son_y].empty())
		{
			return -1;
		}



		for (auto iter = theI[son_x][son_y].begin(); iter != theI[son_x][son_y].end(); ++iter)
		{
			//        std::vector<int> oldH = theStorage[*iter].hindex_;

			if (theStorage[*iter].hindex_.empty() && newH.empty())
			{
				return *iter;
			}


			if (theStorage[*iter].hindex_.size() == newH.size())
			{
				bool thesame = true;
				for (unsigned int i = 0; i < theStorage[*iter].hindex_.size(); ++i)
				{
					if (theStorage[*iter].hindex_[i] != newH[i])
					{
						thesame = false;
						break;
					}
				}
				if (thesame)
				{
					return *iter;
				}
			}
		}

		std::cout << "We check the info of newH: " << std::endl;
		for (auto iter = newH.begin(); iter != newH.end(); ++iter)
		{
			std::cout << *iter << ", ";
		}
		std::cout << std::endl;

		std::cout << "We check all existing indices: " << std::endl;
		for (auto iter = theI[son_x][son_y].begin(); iter != theI[son_x][son_y].end(); ++iter)
		{
			std::vector<int> oldH = theStorage[*iter].hindex_;
			std::cout << "[";
			for (unsigned int i = 0; i < oldH.size(); ++i)
			{
				std::cout << oldH[i] << ", ";
			}
			std::cout << "]" << std::endl;

		}

		std::cout << "findEqualHCNode: error, we should not reach here." << std::endl;
		return -1;
	}

	bool Kim2013::isEqualHC(const std::vector<int>& newH, const std::vector<KimNode>& theStorage, const std::vector<std::vector<Indices> >& theI, int son_x, int son_y)
	{
		if (theI[son_x][son_y].empty())
		{
			return false;
		}

		for (auto iter = theI[son_x][son_y].begin(); iter != theI[son_x][son_y].end(); ++iter)
		{
			//        std::vector<int> oldH = theStorage[*iter].hindex_;
			if (theStorage[*iter].hindex_.empty() && newH.empty())
				return true;

			if (theStorage[*iter].hindex_.size() == newH.size())
			{
				bool thesame = true;
				for (unsigned int i = 0; i < theStorage[*iter].hindex_.size(); ++i)
				{
					if (theStorage[*iter].hindex_[i] != newH[i])
					{
						thesame = false;
						break;
					}
				}
				if (thesame)
				{
					return true;
				}
			}
		}
		return false;
	}

	bool Kim2013::isSelfCrossing(const std::vector<std::pair<int, int> >& P)
	{
		const int tmpx = 10000;
		std::unordered_set<int> S;
		for (auto iter = P.begin(); iter != P.end(); ++iter)
		{
			S.insert(iter->second*tmpx + iter->first);
		}
		if (S.size() < P.size())
			return true;

		return false;
	}

	void Kim2013::tracePath(const KimNode& goalnode, std::vector<std::pair<int, int> >& result_path, const std::vector<KimNode>& theStorage,
		const KimNode& startnode)
	{
		result_path.clear();
		auto cur = goalnode;
		int COUNT = theStorage.size();
		int count = 0;
		while (count < COUNT)
		{

			result_path.push_back(std::pair<int, int>(cur.x_, cur.y_));
			// int father = cur.fatherindex_;
			// if(father == -1)
			if (cur.x_ == startnode.x_ && cur.y_ == startnode.y_ && cur.hindex_ == startnode.hindex_)
			{
				break;
			}

			cur = theStorage[cur.fatherindex_];
			count++;
		}
	}


} // namespace