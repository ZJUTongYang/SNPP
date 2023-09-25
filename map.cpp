#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "map.h"
#include "atom_functions.h"
using namespace std;

// void Costmap::getCriticalObs()
// {
// 	// We calculate the critical obstacles (grids)
// 	// For each obstacle, we check whether its 8-dir neighbour is a 4-neighbor obstacle-free grid
// 	// g_obs <-> costmap_.obs_.back().ObsPosition

// 	// We collect all obstacle grids
// 	std::vector<std::pair<int, int> > all_temp;
// 	all_temp.reserve(xsize_ * ysize_);
// 	for (unsigned int i = 0; i < xsize_; ++i)
// 	{
// 		for (unsigned int j = 0; j < ysize_; ++j)
// 		{
// 			if (data_[j * xsize_ + i] == 254)
// 				all_temp.emplace_back(std::pair<int, int>(i, j));
// 		}
// 	}

// 	// We get the 4-neighbours of obstacle grids
// 	// 4-neighbours = Ð±4ÁÚ£¿
// 	int n_temp = all_temp.size();
// 	all_temp.insert(all_temp.end(), all_temp.begin(), all_temp.end());//repeat 4 times all_temp, as their 4-neighbours' coordinates
// 	all_temp.insert(all_temp.end(), all_temp.begin(), all_temp.end());
// 	for (unsigned int i = 0; i < n_temp; ++i)
// 	{
// 		all_temp[i].first += 1;
// 		all_temp[i].second += 1;
// 		all_temp[i + n_temp].first -= 1;
// 		all_temp[i + n_temp].second -= 1;
// 		all_temp[i + 2 * n_temp].first += 1;
// 		all_temp[i + 2 * n_temp].second -= 1;
// 		all_temp[i + 3 * n_temp].first -= 1;
// 		all_temp[i + 3 * n_temp].second += 1;
// 	}

// 	//// Here we must remove the boundary obstacles, i.e., <= 0 but not < 0
// 	//// This is because we will find their adjacent obstacles in the next step
// 	auto all_temp_temp = all_temp;
// 	all_temp.clear();
// 	all_temp.reserve(all_temp_temp.size());
// 	for (auto iter = all_temp_temp.begin(); iter != all_temp_temp.end(); ++iter)
// 	{
// 		if (iter->first <= 0 || iter->first >= xsize_ - 1 || iter->second <= 0 || iter->second >= ysize_ - 1)
// 			continue;
// 		all_temp.emplace_back(*iter);
// 	}

// 	std::vector<std::pair<int, int> > updated_all_temp;
// 	updated_all_temp.reserve(all_temp.size());
// 	for (auto iter = all_temp.begin(); iter != all_temp.end(); ++iter)
// 	{
// 		if (data_[iter->second * xsize_ + (iter->first - 1)] == 254 ||
// 			data_[iter->second * xsize_ + (iter->first + 1)] == 254 ||
// 			data_[(iter->second - 1) * xsize_ + iter->first] == 254 ||
// 			data_[(iter->second + 1) * xsize_ + iter->first] == 254)
// 		{
// 			continue;
// 		}
// 		updated_all_temp.emplace_back(*iter);
// 	}

// 	// We use precise location of obstacle corners
// 	int x, y;
// 	for (auto iter = updated_all_temp.begin(); iter != updated_all_temp.end(); ++iter)
// 	{
// 		x = iter->first;
// 		y = iter->second;

// 		int count = 0;
// 		if (data_[(y - 1) * xsize_ + (x - 1)] == 254)
// 			count++;
// 		if (data_[(y - 1) * xsize_ + (x + 1)] == 254)
// 			count++;
// 		if (data_[(y + 1) * xsize_ + (x - 1)] == 254)
// 			count++;
// 		if (data_[(y + 1) * xsize_ + (x + 1)] == 254)
// 			count++;

// 		if (count >= 2)
// 			continue;

// 		// Here we don't add 0.5 because in C++ the obstacle occupies integer grids
// 		// point toward the inside of obstacles
// 		if (data_[(y - 1) * xsize_ + (x - 1)] == 254)
// 		{
// 			critical_points.emplace_back(std::pair<double, double>(x, y));
// 			critical_points_orientation.emplace_back(-3.0 / 4 * M_PI);
// 		}
// 		else if (data_[(y + 1) * xsize_ + (x - 1)] == 254)
// 		{
// 			critical_points.emplace_back(std::pair<double, double>(x, y + 1));
// 			critical_points_orientation.emplace_back(3.0 / 4 * M_PI);

// 		}
// 		else if (data_[(y - 1) * xsize_ + (x + 1)] == 254)
// 		{
// 			critical_points.emplace_back(std::pair<double, double>(x + 1, y));
// 			critical_points_orientation.emplace_back(-1.0 / 4 * M_PI);

// 		}
// 		else
// 		{
// 			critical_points.emplace_back(std::pair<double, double>(x + 1, y + 1));
// 			critical_points_orientation.emplace_back(1.0 / 4 * M_PI);

// 		}
// 	}
// }
//
//bool Costmap::isLineSegmentCollisionFree(const std::pair<double, double>& s, 
//	const std::pair<double, double>& g) const 
//{
//	// s: the starting location 
//	// g: the ending location
//
//	// We check the intersection between any two line segments
//	for(auto iter = obs_.begin(); iter != obs_.end(); ++iter)
//	{
//		for(auto iter2 = iter->ordered_vertices_.begin(); iter2 != iter->ordered_vertices_.end()-1; ++iter2)
//		{
//			// The collision with the endpoint itself does not count
//			std::pair<double, double> edge_s_ = iter->ordered_vertices_[iter2- iter->ordered_vertices_.begin()].loc_;
//			std::pair<double, double> edge_g_ = iter->ordered_vertices_[std::next(iter2) - iter->ordered_vertices_.begin()].loc_;
//			if(isCrossing(s, g, edge_s_, edge_g_) &&
//				!((s.first == edge_s_.first && s.second == edge_s_.second) || (s.first == edge_g_.first && s.second == edge_g_.second)
//				|| (g.first == edge_s_.first && g.second == edge_s_.second) || (g.first == edge_g_.first && g.second == edge_g_.second) )
//				)
//			{
//				return false;
//			}
//		}
//		// The last vertex and the first vertex also form an obstacle edge
//		// The collision with the endpoint itself does not count
//		if (isCrossing(s, g, iter->ordered_vertices_.back().loc_, iter->ordered_vertices_[0].loc_) &&
//			!((s.first == iter->ordered_vertices_.back().loc_.first && s.second == iter->ordered_vertices_.back().loc_.second)
//			|| (s.first == iter->ordered_vertices_[0].loc_.first && s.second == iter->ordered_vertices_[0].loc_.second)
//			|| (g.first == iter->ordered_vertices_.back().loc_.first && g.second == iter->ordered_vertices_.back().loc_.second)
//			|| (g.first == iter->ordered_vertices_[0].loc_.first && g.second == iter->ordered_vertices_[0].loc_.second) )
//		)
//		{
//			return false;
//		}
//	}
//	return true;
//}


std::vector<int> Costmap::calSwing(const std::vector<int>& oldH, int oldx, int oldy, int newx, int newy) const
{
	std::vector<int> newH;
	newH.assign(oldH.begin(), oldH.end());
	for (int i = 0; i < obs_.size(); ++i)
	{
		if (oldy > obs_[i].rep_point_.second && newy > obs_[i].rep_point_.second)
		{

			if (oldx > obs_[i].rep_point_.first && newx < obs_[i].rep_point_.first)
			{
				if (!oldH.empty() && oldH.back() == i)
					newH.pop_back();
				else
					newH.push_back(-i);

				return newH;
			}
			else if (oldx < obs_[i].rep_point_.first && newx > obs_[i].rep_point_.first)
			{
				if (!oldH.empty() && oldH.back() == -i)
					newH.pop_back();
				else
					newH.push_back(i);

				return newH;
			}
		}
	}
	return newH;
}


void Costmap::sedFill()
{
	obs_.clear();
	unsigned int nx = xsize_;
	unsigned int ny = ysize_;

	// We first copy the map cost value to another 2D array
	int** mask;
	mask = new int* [nx];
	for (unsigned int i = 0; i < nx; ++i)
	{
		mask[i] = new int[ny];
	}
	// Copy value
	for (unsigned int i = 0; i < nx; ++i)
	{
		for (unsigned int j = 0; j < ny; ++j)
		{
 			mask[i][j] = data_[i + j * nx] == 254 ? 254 : 0;// In matlab, we use the original map, so here we do the same as in matlab
		}
	}

	// We first collect the boundary of the outer obstacle
	// It is stored in the clockwise order, because it is inside the obstacle
	std::list<std::pair<int, int> > outer_obstacle_ordered_boundary_points;
	std::vector<ObstacleVertex> outer_obstacle_ordered_vertices;
	outer_obstacle_ordered_boundary_points.clear();
	outer_obstacle_ordered_vertices.clear();
	{
//		std::pair<int, int> seed(0, ny/2); // it must be an obstacle
		int x = 0;
		int y = ny/2;
		while(1)
		{
			if(mask[x][y] == 254 && mask[x+1][y] == 0)
				break;
			else
			{
				x++;
			}
		}
		
		// now we want clockwise order
		outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x + 1, y));
		outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x + 1, y + 1));

		while(1)
		{
			auto& cur_vertex = outer_obstacle_ordered_boundary_points.back();
			int x = cur_vertex.first;
			int y = cur_vertex.second;
			int lb_free = mask[x - 1][y - 1] == 0 ? 1 : 0; // left bottom
			int lt_free = mask[x - 1][y] == 0 ? 1 : 0; // left top
			int rb_free = mask[x][y - 1] == 0 ? 1 : 0; // right bottom
			int rt_free = mask[x][y] == 0 ? 1 : 0; // right top

			// a binary number representing [lb, lt, rb, rt] 
			int num = lb_free * 8 + lt_free * 4 + rb_free * 2 + rt_free;

			// position map
			//  2 | 4
			// -------
			//  1 | 3

			auto iter = outer_obstacle_ordered_boundary_points.end();

			switch (num)
			{
			case 1:
				// upward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y + 1));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			case 2:
				// rightward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x + 1, y));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			case 3:
				// upward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y + 1));
				break;
			case 4:
				// leftward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x - 1, y));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			case 5:
				// leftward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x - 1, y));
				break;
			case 6:
				//|   |    | b  | 
				// ----=========----
				//|   || 2 | 4 || a
				// ---  -------  ---
				//|   || 1 | 3 || 
				// ----=========----
				//|   |    |    | 
				// cur_vertex = (x, y) = (4)
				iter = std::prev(outer_obstacle_ordered_boundary_points.end(), 2);
				//std::cout << "(*iter).first = " << (*iter).first  << ", x = " << x << std::endl;
				if ((*iter).first == x || (*iter).second == y - 1) { // 3 -> 4 -> a, bottom -> current -> right, the obstacle is at the left top
					outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x - 1, y));
				}
				else if ((*iter).first == x || (*iter).second == y + 1) { // b -> 4 -> 2, top -> current -> left, the obstacle is at the right bottom
					outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x + 1, y));
				}
				break;
			case 7:
				// leftward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x - 1, y));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			case 8:
				// downward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y - 1));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			case 9:
				//|   |    | b  | 
				// ----=========----
				//|   || 2 | 4 || a
				// ---  -------  ---
				//|   || 1 | 3 || 
				// ----=========----
				//|   |    |    | 
				iter = std::prev(outer_obstacle_ordered_boundary_points.end(), 2);
				//std::cout << "(*iter).first = " << (*iter).first << ", x = " << x << std::endl;
				if ((*iter).first == x + 1 || (*iter).second == y) { // a -> 4 -> b, the obstacle is at the left bottom
					outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y - 1));
				}
				else if ((*iter).first == x - 1 || (*iter).second == y) { // 2 -> 4 -> 3, the obstacle is at the right top
					outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y + 1));
				}
				break;
			case 10:
				// rightward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x + 1, y));
				break;
			case 11:
				// upward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y + 1));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			case 12:
				// downward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y - 1));
				break;
			case 13:
				// downward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x, y - 1));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			case 14:
				// rightward
				outer_obstacle_ordered_boundary_points.emplace_back(std::pair<int, int>(x + 1, y));
				outer_obstacle_ordered_vertices.emplace_back(ObstacleVertex(x, y));
				break;
			}

			if (outer_obstacle_ordered_boundary_points.back().first == outer_obstacle_ordered_boundary_points.front().first && 
				outer_obstacle_ordered_boundary_points.back().second == outer_obstacle_ordered_boundary_points.front().second)
			{
				outer_obstacle_ordered_boundary_points.pop_back();
				break;
			}
		}
		// We store the outer obstacle as a normal obstacle
		obs_.emplace_back(Obs());
		obs_.back().rep_point_.first = x + 0.5;
		obs_.back().rep_point_.second = y + 0.5;

		obs_.back().ordered_vertices_ = outer_obstacle_ordered_vertices;

		obs_.back().detailed_ordered_vertices_.assign(outer_obstacle_ordered_boundary_points.begin(), outer_obstacle_ordered_boundary_points.end());

	}


	{
		// We clear the outer obstacle (if it exists)
		std::pair<int, int> seed(0, 0);
		std::vector<std::pair<int, int> > queue;
		queue.reserve(nx * ny);
		queue.emplace_back(seed);
		while (!queue.empty())
		{
			std::pair<int, int> cur = queue.back();
			queue.pop_back();
			if (cur.first >= 0 && cur.second >= 0 && cur.first <= nx - 1 && cur.second <= ny - 1
				&& mask[cur.first][cur.second] >= 254)
			{
				mask[cur.first][cur.second] = 0;
				if (cur.first - 1 >= 0 && mask[cur.first - 1][cur.second] >= 254)
					queue.emplace_back(std::pair<int, int>(cur.first - 1, cur.second));
				if (cur.first + 1 <= nx - 1 && mask[cur.first + 1][cur.second] >= 254)
					queue.emplace_back(std::pair<int, int>(cur.first + 1, cur.second));
				if (cur.second - 1 >= 0 && mask[cur.first][cur.second - 1] >= 254)
					queue.emplace_back(std::pair<int, int>(cur.first, cur.second - 1));
				if (cur.second + 1 <= ny - 1 && mask[cur.first][cur.second + 1] >= 254)
					queue.emplace_back(std::pair<int, int>(cur.first, cur.second + 1));
			}
		}
	}


	// We identify internal obstacles
	for (unsigned int i = 0; i < nx; ++i)
	{
		for (unsigned int j = 0; j < ny; ++j)
		{
			if (mask[i][j] == 0)
			{
				// free space 
				continue;
			}

			std::pair<int, int> seed(i, j);
			// YT: seed must be the left-bottom corner of the obstacle, so we can safely set (i, j) as a vertex

			//////////////////////////////////////////////////////////////////////////
			// YT: We obtain the ordered polygonal shape of the obstacles. 
			// This is done by looping through boundary grids
			std::list<std::pair<int, int> > ordered_boundary_points;
			std::vector<ObstacleVertex> ordered_vertices;
			ordered_boundary_points.clear();
			ordered_vertices.clear();

			// The left-top corner of the seed obstacle (which must be a corner)
			ordered_boundary_points.emplace_back(std::pair<int, int>(floor(seed.first), floor(seed.second)));
			//int nn = 0;
			while(1)
			{
				//++nn;
				//std::cout << nn << std::endl;
				auto& cur_vertex = ordered_boundary_points.back();
				int x = cur_vertex.first;
				int y = cur_vertex.second;
				int lb_free = mask[x-1][y-1] == 0? 1:0; // left bottom
				int lt_free = mask[x-1][y] == 0? 1:0; // left top
				int rb_free = mask[x][y-1] == 0? 1:0; // right bottom
				int rt_free = mask[x][y] == 0? 1:0; // right top

				// a binary number representing [lb, lt, rb, rt] 
				int num = lb_free * 8 + lt_free*4+rb_free*2+rt_free;

				// position map
				//  2 | 4
				// -------
				//  1 | 3
				// the (x, y) represents the left bottom corner of the pixel
				// We want to find the precise contour of the discretised obstacle.

				auto iter = ordered_boundary_points.end();

				switch(num)
				{
				case 1:
					// upward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x, y+1));
					//ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				case 2:
					// rightward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x+1, y));
					//ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				case 3:
					// upward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x, y+1));
					break;
				case 4:
					// leftward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x-1, y));
					//ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				case 5:
					// leftward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x-1, y));
					break;
				case 6:
					//|   |    | b  | 
					// ----=========----
					//|   || 2 | 4 || a
					// ---  -------  ---
					//|   || 1 | 3 || 
					// ----=========----
					//|   |    |    | 
					// cur_vertex = (x, y) = (4)
					iter = std::prev(ordered_boundary_points.end(), 2);
					//std::cout << "(*iter).first = " << (*iter).first  << ", x = " << x << std::endl;
					if ((*iter).first == x || (*iter).second == y-1){ // 3 -> 4 -> a, bottom -> current -> right, the obstacle is at the left top
				 		ordered_boundary_points.emplace_back(std::pair<int, int>(x - 1, y));
					 }
					 else if ((*iter).first == x || (*iter).second == y+1) { // b -> 4 -> 2, top -> current -> left, the obstacle is at the right bottom
				 		ordered_boundary_points.emplace_back(std::pair<int, int>(x+1, y));
					 }
					break;
				case 7:
					// leftward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x-1, y));
					ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				case 8:
					// downward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x, y-1));
					//ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				case 9:
					//|   |    | b  | 
					// ----=========----
					//|   || 2 | 4 || a
					// ---  -------  ---
					//|   || 1 | 3 || 
					// ----=========----
					//|   |    |    | 
					iter = std::prev(ordered_boundary_points.end(), 2);
					//std::cout << "(*iter).first = " << (*iter).first << ", x = " << x << std::endl;
					if ((*iter).first == x+1 || (*iter).second == y) { // a -> 4 -> b, the obstacle is at the left bottom
						ordered_boundary_points.emplace_back(std::pair<int, int>(x, y-1));
					}
					else if ((*iter).first == x-1 || (*iter).second == y) { // 2 -> 4 -> 3, the obstacle is at the right top
						ordered_boundary_points.emplace_back(std::pair<int, int>(x, y+1));
					}
					break;
				case 10:
					// rightward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x+1, y));
					break;
				case 11:
					// upward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x, y+1));
					ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				case 12:
					// downward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x, y-1));
					break;
				case 13:
					// downward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x, y-1));
					ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				case 14:
					// rightward
					ordered_boundary_points.emplace_back(std::pair<int, int>(x+1, y));
					ordered_vertices.emplace_back(ObstacleVertex(x, y));
					break;
				}

				if(ordered_boundary_points.back().first == ordered_boundary_points.front().first && 
					ordered_boundary_points.back().second == ordered_boundary_points.front().second)
				{
					ordered_boundary_points.pop_back();
					break;
				}
			}

			//////////////////////////////////////////////////////////////////////////
			// YT: we simplify co-linear obstacle vertices
			// Because the size of the vector is changing, so we use while-loop
			int i = 0; 
			while(1)
			{
				// The ordered vertices must have more than 4 elements. So no need to check the size

				int prev_index = (i == 0)? ordered_vertices.size()-1: i-1;
				int next_index = (i == ordered_vertices.size()-1)? 0: i+1;
				if(isColinear(ordered_vertices[prev_index].loc_, ordered_vertices[i].loc_, ordered_vertices[next_index].loc_))
				{
					ordered_vertices.erase(ordered_vertices.begin()+i);
					continue;
				}
				else
				{
					++i;
					if(i == ordered_vertices.size())
					{
						// it has reached the next element of the last element, i.e., a null position
						break;
					}
				}
			}

			//////////////////////////////////////////////////////////////////////////

			obs_.emplace_back(Obs());
			obs_.back().rep_point_.first = seed.first + 0.5;
			obs_.back().rep_point_.second = seed.second + 0.5;
			
			obs_.back().ordered_vertices_ = ordered_vertices;

			obs_.back().detailed_ordered_vertices_.assign(ordered_boundary_points.begin(), ordered_boundary_points.end());

			//std::cout << "obs , " << "ordered_vertices: " << ordered_vertices.size() << std::endl;

			// We start a sedFill to eliminate all connected obstacles;
			std::vector<std::pair<int, int> > queue;
			queue.reserve(nx * ny);
			queue.emplace_back(seed);
			while (!queue.empty())
			{
				auto cur = queue.back();
				if (cur.first >= 0 && cur.first <= nx - 1 &&
					cur.second >= 0 && cur.second <= ny - 1 &&
					mask[cur.first][cur.second] >= 254)
				{
					mask[cur.first][cur.second] = 0;

					// We store the grids of a connected obstacle (maybe we can develop visibility graph from it)
					obs_.back().grids_.emplace_back(std::pair<int, int>(cur.first, cur.second));

					queue.pop_back();
					if (cur.first - 1 >= 0 && mask[cur.first - 1][cur.second] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first - 1, cur.second));
					if (cur.first + 1 <= nx - 1 && mask[cur.first + 1][cur.second] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first + 1, cur.second));
					if (cur.second - 1 >= 0 && mask[cur.first][cur.second - 1] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first, cur.second - 1));
					if (cur.second + 1 <= ny - 1 && mask[cur.first][cur.second + 1] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first, cur.second + 1));
				}
				else
				{
					queue.pop_back();
				}
			}//while
		}
	}

	// delete mask
	for (unsigned int i = 0; i < nx; ++i)
		delete[] mask[i];

	delete[] mask;
	//std::cout << "obs_.size() = " << obs_.size() << std::endl;
	//std::cout << "obs_.back().grids_.size() = " << obs_[1].grids_.size() << std::endl;
}

void Costmap::showMap(const Costmap& p, std::string window_name)
{
	// Since this visualization is intrinsic to opencv, we write it here. 
	// Should the costmap be implemented by other interfaces, just discard this function

	cv::Mat img(p.ysize_, p.xsize_, CV_8UC1);

	for (unsigned int row = 0; row < p.ysize_; ++row)
	{
		uchar* current_row = img.ptr<uchar>(row);
		for (unsigned int col = 0; col < p.xsize_; ++col)
		{
			*(current_row + col) = p.data_[(p.ysize_ - 1 - row) * p.xsize_ + col];// / 255.0 * 100.0 + 155;
		}
	}
	imshow(window_name, img);
}


// void Costmap::ArrangeCriticalToObs()
// {

// 	for (int critical_i = 0; critical_i < this->critical_points.size(); ++critical_i)
// 	{
// 		std::pair<double, double> temp_points = this->critical_points.at(critical_i);
// 		auto it = find(this->obs_.back().grids_.begin(), this->obs_.back().grids_.end(), temp_points);
// 		//index = distance(this->obs_.back().grids_.begin(), it);// index in grids_
// 		this->obs_.back().vertices_.push_back(temp_points);
// 		this->obs_.back().critical_points_orientation_Obs_.push_back(this->critical_points_orientation.at(critical_i));

// 	}
// }


void Costmap::generateCostMap(cv::Mat img)
{
	unsigned int width = xsize_;
	unsigned int height = ysize_;
	// We need to store the costmap in Euclidean X-Y coordinate
	for (unsigned int row = 0; row < height; ++row)
	{
		uchar* current_row = img.ptr<uchar>(row);
		for (unsigned int col = 0; col < width; ++col)
		{
			data_[(height - 1 - row) * width + col] = *(current_row + col);
		}
	}

	//showMap(data_, "show the map as image map");

	// We transform the image map (free:255, obstacle:0) to the costmap (free: 0, obstacle: 254, inflation: 127)
	for (unsigned int row = 0; row < height; ++row)
	{
		for (unsigned int col = 0; col < width; ++col)
		{
			int temp = data_[row * width + col];
			if (temp == 0)
			{
				data_[row * width + col] = 254;
			}
			else if (temp >= 253) // a white grid, which corresponds to the obstacle-free point
			{
				data_[row * width + col] = 0;
			}
		}
	}

	//showMap(costmap_, "show after image map --> cost map");

	//  We add the boundary of the map
	for (unsigned int row = 0; row < height; ++row)
	{
		for (unsigned int col = 0; col < width; ++col)
		{
			if (row == 0 || row == height - 1)
			{
				data_[row * width + col] = 254;
			}
			else
			{
				if (col == 0 || col == width - 1)
				{
					data_[row * width + col] = 254;
				}
			}
		}
	}
	//showMap(costmap_, "show after boundary --> cost map");

	// We obtain distinct obstacles
	// obs_.ObsPosition: store the first point of each obstacle, 
	// obs_.grids: store the contour of each obstacle


}

Costmap::Costmap(Param theParam, cv::Mat img)
{
	data_ = new int[theParam.xsize_ * theParam.ysize_];
	this->xsize_ = theParam.xsize_;
	this->ysize_ = theParam.ysize_;


	// map.png -> costmap_(0, 253, 254, 255)
	generateCostMap(img);
	
	
	//std::cout << "theParam.xsize_ = " << theParam.xsize_ << std::endl;
	//double time_1 = clock();
	sedFill();
	//double time_2 = clock();
	//std::cout << "sedFill time = " << time_2 - time_1 << "ms" << std::endl;

	//std::cout << "There are " << obs_.size() << " internal obstacles. " << std::endl;
	//for(unsigned int i = 0; i < obs_.size(); ++i)
	//{
	//	std::cout << "We show the ordered vertices of the " << i << "-th internal obstacle" << std::endl;
	//	for(auto iter = obs_[i].ordered_vertices_.begin(); iter != obs_[i].ordered_vertices_.end(); ++iter)
	//	{
	//		std::cout << "[" << iter->loc_.first << ", " << iter->loc_.second << "], ";
	//	}
	//	std::cout << std::endl;
	//}
	//std::cout << std::endl;

	//critical_points.clear();
	//critical_points_orientation.clear();
	// obs_ -> critical_points, critical_points_orientation

//	getCriticalObs();


	// int critical_points_num = critical_points.size();
	// std::cout << "critical_points_num = " << critical_points_num << std::endl;

	// critical_points, critical_points_orientation -> obs_.back().vertices_, obs_.back().critical_points_orientation_Obs_
	// ArrangeCriticalToObs();
	

	//critical_points_x_ = new int[critical_points_num_];
	//critical_points_y_ = new int[critical_points_num_];

	//cv::waitKey(0);
	//for (unsigned int i = 0; i < critical_points.size(); ++i)
	//{
	//	// YT: here we can safely use round, because the critical points in C++ must be near an integer
	//	std::cout << "critical_points = " << critical_points[i].first << std::endl;
	//	critical_points_x_[i] = round(critical_points[i].first); 
	//	critical_points_y_[i] = round(critical_points[i].second);
	//}
}