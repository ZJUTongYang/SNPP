#include <iostream>
#include <cmath>
#include <algorithm>
#include "Node.h"
#include "Definitions.h"
#include "map.h"
#include "atom_functions.h"
#include "child.h"

Node::Node(const Costmap* pMap, std::pair<double, double> position): 
	seed_(position), Nindex_(0), seed_obs_index_(-1), seed_ver_index_(-1), fatherindex_(-1)
{
	V_.clear();
	
	start_angle_ = 0;
	end_angle_ = 2 * M_PI;
	std::pair<double, double> start_temp(seed_.first + 0.5*cos(start_angle_), seed_.second + 0.5*sin(start_angle_));
	std::pair<double, double> end_temp(seed_.first + 0.5*cos(end_angle_), seed_.second + 0.5*sin(end_angle_));

	double min_dis = 10000.0;
	std::pair<double, double> min_intersection;
	std::pair<double, double> the_intersection;
	for (auto iter = pMap->obs_.begin(); iter != pMap->obs_.end(); ++iter)
	{
		for (unsigned int j = 0; j < iter->ordered_vertices_.size(); ++j)
		{
			int next_index = (j == iter->ordered_vertices_.size() - 1) ? 0 : j + 1;
			if (findIntersectionBetweenFarRayAndSegment(seed_, start_temp,
				iter->ordered_vertices_[j].loc_, iter->ordered_vertices_[next_index].loc_, 
				the_intersection))
			{
				double the_dis = distance(seed_, the_intersection);
				if (the_dis < min_dis)
				{
					min_dis = the_dis;
					start_o_obs_index_ = iter - pMap->obs_.begin();
					start_o_ver_index_ = j;
					min_intersection = the_intersection;
				}
			}
		}
	}
	start_o_ = min_intersection;

	min_dis = 10000.0;
	for (auto iter = pMap->obs_.begin(); iter != pMap->obs_.end(); ++iter)
	{
		for (unsigned int j = 0; j < iter->ordered_vertices_.size(); ++j)
		{
			int next_index = (j == iter->ordered_vertices_.size() - 1) ? 0 : j + 1;
			if (findIntersectionBetweenFarRayAndSegment(seed_, end_temp,
				iter->ordered_vertices_[j].loc_, iter->ordered_vertices_[next_index].loc_,
				the_intersection))
			{
				double the_dis = distance(seed_, the_intersection);
				if (the_dis < min_dis)
				{
					min_dis = the_dis;
					end_o_obs_index_ = iter - pMap->obs_.begin();
					end_o_ver_index_ = j;
					the_intersection = min_intersection;
				}
			}
		}
	}
	end_o_ = min_intersection;
}

Node::Node(const Child& as_a_child)
{

}

void Node::constructV(const Costmap* pMap)
{

	// We begin with the start angle
	std::vector<std::pair<double, double> > V_boundary;
	V_boundary.emplace_back(start_o_);

	int cur_obs_index = start_o_obs_index_;
	int cur_ver_index = start_o_ver_index_;

	int next_obs_index, next_ver_index;
	for (unsigned int i = 0; i < C_.size(); ++i)
	{
		if (C_[i].is_a_left_gap_)
		{
			next_obs_index = C_[i].o_obs_index_;
			next_ver_index = C_[i].o_ver_index_;
			// Here cur_obs_index must be equal to next_obs_index because they must hit the same obstacle
			// And cur_ver_index must be larger than the next_ver_index, 
			// because we are to trace the obstacles' outer boundary in CCW order
			int temp;
			if (cur_ver_index != next_ver_index)
			{
				// Here cannot be "==", because cur might be 0 while next is not 0
				// We insert the obstacle vertex one by one
				temp = cur_ver_index;
				while (1)
				{
					V_boundary.emplace_back(pMap->obs_[cur_obs_index].ordered_vertices_[temp].loc_);
					temp = (temp == 0) ? pMap->obs_[cur_obs_index].ordered_vertices_.size() - 1 : temp - 1;
					if (temp == next_ver_index)
					{
						break;
					}
				}
			}
			V_boundary.emplace_back(C_[i].o_);
			V_boundary.emplace_back(C_[i].c_);
			cur_obs_index = C_[i].c_obs_index_;
			cur_ver_index = C_[i].c_ver_index_;
		}
		else
		{
			next_obs_index = C_[i].c_obs_index_;
			next_ver_index = C_[i].c_ver_index_;
			
			int temp;
			if (cur_ver_index != next_ver_index)
			{
				temp = cur_ver_index;
				while (1)
				{
					V_boundary.emplace_back(pMap->obs_[cur_obs_index].ordered_vertices_[temp].loc_);
					temp = (temp == 0) ? pMap->obs_[cur_obs_index].ordered_vertices_.size() - 1 : temp - 1;
					if (temp == next_ver_index)
					{
						// here "next_ver_index" is the vertex index of the next c_ point
						break;
					}
				}
			}
			V_boundary.emplace_back(C_[i].c_);
			V_boundary.emplace_back(C_[i].o_);
			cur_obs_index = C_[i].o_obs_index_;
			cur_ver_index = C_[i].o_ver_index_;
		}
	}

	// We are not sure whether the last child is at the same edge as end_o(but they must be at the same obstacle)
	// So we need to check them
	next_obs_index = end_o_obs_index_;
	next_ver_index = end_o_ver_index_;
	if (cur_ver_index != next_ver_index)
	{
		int temp = cur_ver_index;
		while (1)
		{
			V_boundary.emplace_back(pMap->obs_[cur_obs_index].ordered_vertices_[temp].loc_);
			temp = (temp == 0) ? pMap->obs_[cur_obs_index].ordered_vertices_.size() - 1 : temp - 1;
			if (temp == next_ver_index)
				break;
		}
	}
	V_boundary.emplace_back(end_o_);

	if (Nindex_ != 0)
	{
		V_boundary.emplace_back(seed_);
	}

	V_.assign(V_boundary.begin(), V_boundary.end());
}

// We find the ray (seed_ -> c_) hits which obstacle and which edge the crossing(intersection) position locates on.
// the index of obstacle and the start and end positions of the edge are recorded.
// The ray may crosses several edges, choose the nearest one to c_, and the distance is noted as distance_to_c_.
void Node::findOForLastChild(const Costmap* pMap)
{
	double min_dis = 10000.0;
	C_.back().o_obs_index_ = -1;
	C_.back().o_ver_index_ = -1;
	std::pair<double, double> the_intersection;
	std::pair<double, double> min_intersection;
	for (auto iter = pMap->obs_.begin(); iter != pMap->obs_.end(); ++iter)
	{
		for (unsigned int j = 0; j < iter->ordered_vertices_.size(); ++j)
		{
			int next_index = (j == iter->ordered_vertices_.size() - 1) ? 0 : j + 1;
			if (findIntersectionBetweenFarRayAndSegment(seed_, C_.back().c_,
				iter->ordered_vertices_[j].loc_, iter->ordered_vertices_[next_index].loc_,
				the_intersection))
			{
				double the_dis = distance(seed_, the_intersection);
				if (the_dis < min_dis)
				{
					min_dis = the_dis;
					C_.back().o_obs_index_ = iter - pMap->obs_.begin();
					C_.back().o_ver_index_ = j;
					min_intersection = the_intersection;
				}
			}
		}
	}
	C_.back().o_ = min_intersection;
}
//
//void Node::findO(std::pair<double, double> c_, const std::vector<Obs>& obs_)
//{
//	// We store all crossing information of each edge of all obstacles in this vector.
//	std::vector<Cross> Crossing_list;
//	std::pair<int, double> min_distance = std::make_pair(10000, 0);
//	for (auto iter = obs_.begin(); iter != obs_.end(); ++iter) {
//		for (unsigned int i = 0; i < iter->ordered_vertices_.size(); ++i) {
//			// Get an edge of a obstacle
//			int s_index_, g_index_;
//			std::pair<double, double> obs_edge_s_, obs_edge_e_;
//			
//			if (i == iter->ordered_vertices_.size()-1) {
//				s_index_ = i;
//				g_index_ = 0;
//			}
//			else {
//				s_index_ = i;
//				g_index_ = i+1;
//			}
//			obs_edge_s_ = iter->ordered_vertices_[s_index_].loc_;
//			obs_edge_e_ = iter->ordered_vertices_[g_index_].loc_;
//
//			// Checking whether the visibility edge(ray of seed_ -> c_) and the polygonal edge of obstacal is crossing
//			// First checking whether these two lines are crossing or not
//			// Then checking whether the intersection is on the RAY of visibility edge and the SEGMENT of obs_edge.
//			// Finally register the eligible intersecton into Crossing_list.
//			if (isCrossing(seed_, c_, obs_edge_s_, obs_edge_e_)) {
//				std::pair<double, double> intersection = calIntersection(seed_, c_, obs_edge_s_, obs_edge_e_);
//				bool flag_onRay_of_visibility_edge = onRay(seed_, c_, intersection);
//				bool flag_onSegment_of_obs_edge = onSegment(obs_edge_s_, obs_edge_e_, intersection);
//				if (flag_onRay_of_visibility_edge == true && flag_onSegment_of_obs_edge == true) {
//					int n_temp = Crossing_list.size();
//					// intersection_index, intersection position, obs_index, vertex_index
//					Crossing_list.push_back(Cross(n_temp, intersection, iter-obs_.begin(), s_index_, g_index_));
//					// only for comparing and choosing the minimum distance, no need to sqrt.
//					Crossing_list[n_temp].distance_to_c_power2 = (intersection.first - c_.first)* (intersection.first - c_.first) + (intersection.second - c_.second) * (intersection.second - c_.second);
//
//					// record the min_distance so far
//					if (n_temp == 0) {
//						min_distance = std::make_pair(n_temp, Crossing_list[n_temp].distance_to_c_power2);
//					}
//					else {
//						if (Crossing_list[n_temp].distance_to_c_power2 < min_distance.second) {
//							min_distance = std::make_pair(n_temp, Crossing_list[n_temp].distance_to_c_power2);
//						}
//					}
//				}
//			}
//			
//		}
//	}
//	C_.back().o_ = Crossing_list[min_distance.first].intersection_;
//	C_.back().the_index_of_obstacle_that_o_hits_ = Crossing_list[min_distance.first].the_index_of_obs_that_the_crossing_on_;
//	C_.back().the_index_of_s_of_the_edge_that_o_hits_ = Crossing_list[min_distance.first].the_vertex_index_of_s_of_the_edge_that_o_hits_;
//	C_.back().the_index_of_g_of_the_edge_that_o_hits_ = Crossing_list[min_distance.first].the_vertex_index_of_g_of_the_edge_that_o_hits_;
//
//}

void Node::Expand(const Costmap* pMap, std::pair<double, double> goal_location)
{
	C_.clear();

	// We deal with each internal obstacle separately
	for (auto iter = pMap->obs_.begin(); iter != pMap->obs_.end(); ++iter)
	{
		// Note that for each internal obstacle, the number of possible children is not at most 2. 
		// This is because the obstacle might be concave. 
		std::vector<double> theta_list;
		theta_list.resize(iter->ordered_vertices_.size());
		for (unsigned int i = 0; i < iter->ordered_vertices_.size(); ++i)
		{
			theta_list[i] = atan2(iter->ordered_vertices_[i].loc_.second - seed_.second, iter->ordered_vertices_[i].loc_.first - seed_.first);
			// We normalise the angle into [start_angle_, end_angle_]
			// theta_list[i]: [-PI, PI] -> [start_angle_ + 0, start_angle_ + 2*PI]
			theta_list[i] = start_angle_ + normalize_angle_positive(theta_list[i] - start_angle_);
		}

		std::vector<bool> is_a_gap;
		is_a_gap.resize(theta_list.size(), false);
		// At the beginning, all visibility edges may be valid. So we filter them next. 

		std::vector<bool> is_a_left_gap;
		is_a_left_gap.resize(theta_list.size(), false);
		// Left gap means that after the robot moves outside the visibility region, it turns leftward

		// We check each possible connection between the source point to the obstacle vertex
		for (unsigned int i = 0; i < theta_list.size(); ++i)
		{
			// Out of the angle range of this node, start_angle_ ~ end_angle_
			// If the vertex is the source point itself, we also skip
			if ((theta_list[i] > end_angle_) || 
				(seed_obs_index_ == iter - pMap->obs_.begin() && seed_ver_index_ == i))
			{
				// Here the =end_angle_ case must be preserved, because this is also a gap
				is_a_gap[i] = false;
			}
			else if (as_a_child_left_gap_ && normalize_angle_positive(theta_list[i]-start_angle_) <= RAYSTAR_EPS)
			{
				is_a_gap[i] = false;
			}
			else if (!as_a_child_left_gap_ && normalize_angle_positive(theta_list[i]-start_angle_) >= (end_angle_-start_angle_) - RAYSTAR_EPS)
			{
				is_a_gap[i] = false;
			}
			else
			{
				// We check whether there is a self-occlusion between consecutive obstacle vertices
				int prev_index = (i == 0)? theta_list.size()-1: i-1;
				int next_index = (i==theta_list.size()-1)? 0: i+1;
				double prev_relative_theta = normalize_angle(theta_list[prev_index] - theta_list[i]);
				double next_relative_theta = normalize_angle(theta_list[next_index] - theta_list[i]);
				
				//// If the vertices[i] is not the point of tangency, then the theta_list[prev_index] < theta_list[i] < theta_list[next_index] or otherwise
				//if((prev_relative_theta > 0 && next_relative_theta < 0) ||
				//	(prev_relative_theta < 0 && next_relative_theta > 0))
				//{
				//	is_a_gap[i] = false;
				//	continue;
				//}
				//
				//// The point of tangency is on the right-hand side among this obstacle, repect to the Node, so the following path is left-toward.
				//std::cout << "YT: check this point" << std::endl;
				//if(prev_relative_theta > 0 && next_relative_theta > 0)
				//{
				//	is_a_left_gap[i] = true;
				//}
				//if (prev_index == seed_ver_index_ && next_relative_theta > 0)
				//{
				//	is_a_left_gap[i] = true;
				//}

				// If c is the previous vertex of p, and prev_rela_theta < 0, then it is a right gap
				// If c is the next vertex of p, and next_rela_theta > 0, then it is a left gap
				// Otherwise, if prev_rela_theta >= 0, it can only be a left gap or not a gap
				//		if next_rela_theta > 0, it is a left gap
				// Otherwise, if prev_rela_theta < 0, it can only be a left gap or not a gap
				//		if next_rela_theta <= 0, it is a right gap
				if (seed_obs_index_ == iter - pMap->obs_.begin() && seed_ver_index_ == next_index)
				{
					if (prev_relative_theta < 0)
					{
						is_a_gap[i] = true;
						is_a_left_gap[i] = false;
					}
				}
				else if (seed_obs_index_ == iter - pMap->obs_.begin() && seed_ver_index_ == prev_index)
				{
					if (next_relative_theta > 0)
					{
						is_a_gap[i] = true;
						is_a_left_gap[i] = true;
					}
				}
				else
				{
					if (prev_relative_theta >= 0 && next_relative_theta > 0)
					{
						is_a_gap[i] = true;
						is_a_left_gap[i] = true;
					}
					else if (prev_relative_theta < 0 && next_relative_theta <= 0)
					{
						is_a_gap[i] = true;
						is_a_left_gap[i] = false;
					}
				}

				if (!is_a_gap[i])
				{
					continue;
				}
				// After it is angularly OK, we also need to check that the visibility edge is collision-free
//				if (!pMap->isLineSegmentCollisionFree(seed_, iter->ordered_vertices_[i].loc_))
				if(!isValidBranch(pMap, iter->ordered_vertices_[i].loc_, iter-pMap->obs_.begin(), i, is_a_left_gap[i]))
				{
					is_a_gap[i] = false;
				}

				if (!is_a_gap[i])
				{
					continue;
				}

				// We have found all gaps regarding this internal obstacle. 
				// Now we store them
				C_.emplace_back(Child(Nindex_, -1, iter->ordered_vertices_[i].loc_, is_a_left_gap[i]));
				if (is_a_left_gap[i] == true) {
					// In the left gap, start(theta_list[i]) -> end(contour tangency at vertices[i])
					// The theta_list[i] is relative to the start_angle of Node ([start_angle_ + 0, start_angle_ + 2*PI]), we normalize it to [-PI, PI]
					C_.back().start_angle_ = normalize_angle(theta_list[i]);
					double contour_angle_from_next = atan2(iter->ordered_vertices_[next_index].loc_.second - iter->ordered_vertices_[i].loc_.second, 
															iter->ordered_vertices_[next_index].loc_.first - iter->ordered_vertices_[i].loc_.first);
					C_.back().end_angle_ = C_.back().start_angle_ + normalize_angle_positive(contour_angle_from_next - C_.back().start_angle_);
				}
				else {
					// In the right gap, start(contour tangency at vertices[i]) -> end(theta_list[i])
					double contour_angle_from_prev = atan2(iter->ordered_vertices_[prev_index].loc_.second - iter->ordered_vertices_[i].loc_.second, 
															iter->ordered_vertices_[prev_index].loc_.first - iter->ordered_vertices_[i].loc_.first);
					C_.back().start_angle_ = contour_angle_from_prev;
					C_.back().end_angle_ = contour_angle_from_prev + normalize_angle_positive(theta_list[i] - contour_angle_from_prev);
				}

				C_.back().gap_orientation_ = start_angle_ + normalize_angle_positive(theta_list[i] - start_angle_);

				// We store the obstacle index of C_[n_temp] and the vertex index within the obstacle.
				C_.back().c_obs_index_ = iter - pMap->obs_.begin();
				C_.back().c_ver_index_ = i;

				// We should continue this visibility edge until hit an obstacle
				//findO(iter->ordered_vertices_[i].loc_, pMap->obs_);
				findOForLastChild(pMap);

				C_.back().c_gcost_ = Gcost_ + distance(seed_, C_.back().c_);
				C_.back().c_hcost_ = distance(C_.back().c_, goal_location);
			}
		}
	}

	// We first order all children in orientation order
	// The angle of the gap has been stored in the gap_orientation, and has been wrapped to [start_angle, start_angle+2pi]
	std::sort(C_.begin(), C_.end(),
		[](const Child& a, const Child& b) {return a.gap_orientation_ < b.gap_orientation_; });

	// After sorting the children based on gap orientation, 
	for (unsigned int i = 0; i < C_.size(); ++i)
	{
		C_[i].Cindex_ = i;
	}

	// construct the outer contour of the visibility region
	constructV(pMap);

}

bool Node::isValidBranch(const Costmap* pMap, std::pair<double, double> c, 
	int c_obs_index, int c_ver_index, bool is_a_left_gap)
{
	// Whenever it is time to check 

	int prev_c_ver_index = (c_ver_index == 0) ? pMap->obs_[c_obs_index].ordered_vertices_.size() - 1 : c_ver_index - 1;
	int next_c_ver_index = (c_ver_index == pMap->obs_[c_obs_index].ordered_vertices_.size() - 1) ? 0 : c_ver_index + 1;

	// s: the starting location 
	// g: the ending location
	const std::pair<double, double>& s = seed_;
	const std::pair<double, double>& g = c;

	// We check the intersection between any two line segments
	for (auto iter = pMap->obs_.begin(); iter != pMap->obs_.end(); ++iter)
	{
		for(unsigned int i = 0; i < iter->ordered_vertices_.size(); ++i)
		{
//			int prev_index = (i == 0) ? iter->ordered_vertices_.size() - 1 : i - 1;
			int prev_index = i;
			int next_index = (i == iter->ordered_vertices_.size() - 1) ? 0 : i + 1;

			// The collision with the endpoint itself does not count
			const std::pair<double, double>& edge_s = iter->ordered_vertices_[i].loc_;
			const std::pair<double, double>& edge_g = iter->ordered_vertices_[next_index].loc_;
			
			// If the path is exactly touching an edge (wall following), it is allowed. 
			if (isColinear(s, edge_s, g) && isColinear(s, edge_g, g))
				continue;

			// Here endpoint contacts are considered
			if (isCrossing(s, g, edge_s, edge_g))
			{
				if(iter - pMap->obs_.begin() == c_obs_index)
				{
					// We check whether there are some colinear cases
					// if this is a left-gap: 
					// (1) if next_index == prev_c_index && [p, prev_c_index, c] colinear, continue
					// (2) if prev_index == prev_c_index, the obstacle orientation must be OK, so continue;
					// (3) if prev_index == c_index, the obstacle orientation must be OK, so continue
					// (4) if prev_index == next_c_index, there will be no crossing, impossible
					// If this is not a left-gap: 
					// (1) If next_index == prev_c_index, there will be no crossing, impossible
					// (2) If prev_index == prev_c_index, the obstacle orientation must be OK, so continue
					// (3) If prev_index == c_index, the obstacle orientation must be OK, so continue;
					// (4) If prev_index == next_c_index && [p, next_c_index, c] colinear, continue;
					if (is_a_left_gap)
					{
						if (next_index == prev_c_ver_index && isColinear(s, iter->ordered_vertices_[prev_c_ver_index].loc_, g))
						{
							continue;
						}
						else if (prev_index == prev_c_ver_index)
						{
							continue;
						}
						else if (prev_index == c_ver_index)
						{
							continue;
						}
					}
					else
					{
						if (prev_index == prev_c_ver_index)
						{
							continue;
						}
						else if (prev_index == c_ver_index)
						{
							continue;
						}
						else if (prev_index == next_c_ver_index && isColinear(s, iter->ordered_vertices_[next_c_ver_index].loc_, g))
						{
							continue;
						}
					}
				}

				// Since the source point is also next to the obstacle, so we also need to check this
				if (iter - pMap->obs_.begin() == seed_obs_index_)
				{
					if (prev_index == seed_ver_index_)
					{
						continue;
					}
					else if (next_index == seed_ver_index_)
					{
						continue;
					}
				}
				return false;
			}
		}
	}

	//// If the branch is not crossed, we check whether it is identical to obstacle vertices
	//std::pair<double, double> ds1(g.first - s.first, g.second - s.second);
	//for (auto iter = pMap->obs_.begin(); iter != pMap->obs_.end(); ++iter)
	//{
	//	for (auto iter2 = iter->ordered_vertices_.begin(); iter2 != iter->ordered_vertices_.end() - 1; ++iter2)
	//	{
	//		const std::pair<double, double>& edge_s = iter->ordered_vertices_[iter2 - iter->ordered_vertices_.begin()].loc_;
	//		const std::pair<double, double>& edge_g = iter->ordered_vertices_[std::next(iter2) - iter->ordered_vertices_.begin()].loc_;

	//		std::pair<double, double> ds2(edge_g.first - edge_s.first, edge_g.second - edge_s.second);
	//		if (fabs(ds1.first*ds2.second - ds1.second*ds2.first) < RAYSTAR_EPS &&
	//			isColinear(edge_s, g, edge_g))
	//		{
	//			// The two edges are parallel
	//			return false;
	//		}
	//	}
	//}

	return true;
}


// void Node::MaxMinAngle(std::vector<std::pair<double, double> > critical_points, Node node_father)
// {
// 	//this->seed_;// node position
// 	double max_angle = -2 * M_PI;
// 	double min_angle = 2 * M_PI;
// 	int max_index, min_index;
// 	for (unsigned int i = 0; i < critical_points.size(); ++i)
// 	{ 
// 		double dx = round(critical_points[i].first) - seed_.first;
// 		double dy = round(critical_points[i].second) - seed_.second;
// 		double temp_angle;
// 		// Calculate the angle of seed->
// 		if (dy == 0 && dx > 0)
// 		{
// 			temp_angle = M_PI / 2;
// 		}
// 		else if (dy == 0 && dx < 0)
// 		{
// 			temp_angle = -M_PI / 2;
// 		}
// 		else
// 		{	
// 			temp_angle = atan2(dy, dx);
// 		}
// 		// -PI~0 -> PI~2PI 
// 		if (temp_angle < 0)
// 		{
// 			temp_angle += 2 * M_PI; 
// 		}


// 		if (temp_angle > max_angle)
// 		{	
// 			max_angle = temp_angle;
// 			max_index = i;
// 		}
// 		if (temp_angle < min_angle)
// 		{
// 			min_angle = temp_angle;
// 			min_index = i;
// 		}

// 	}
	
// 	//// HL: here the start and end angle may be set wrong, min_angle-> max_angle, or max_angle -> min_angle+2PI
// 	//this->start_angle_ = min_angle;
// 	//this->end_angle_ = max_angle;
// }


void Node::showDetails()
{
	std::cout << "Node " << Nindex_ 
		<< ": seed position [" << seed_.first << ", " << seed_.second << "]" 
		<< ", start_angle_: " << start_angle_ << ", end_angle_: " << end_angle_ 
		<< ", number of corridors: " << C_.size()
		<< ", fatherindex: " << fatherindex_ 
		//<< ", son_in_father_index: " << soninfatherindex_ 
		<< std::endl; 
}


