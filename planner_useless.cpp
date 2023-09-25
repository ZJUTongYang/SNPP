//
//void Node::MaxMinAngle(std::vector<std::pair<double, double> > critical_points, Node node_father)
//{
//	//this->seed_;// node position
//	double max_angle = -2 * M_PI;
//	double min_angle = 2 * M_PI;
//	int max_index, min_index;
//	for (unsigned int i = 0; i < critical_points.size(); ++i)
//	{
//		double dx = round(critical_points[i].first) - seed_.first;
//		double dy = round(critical_points[i].second) - seed_.second;
//		double temp_angle;
//		// Calculate the angle of seed->
//		if (dy == 0 && dx > 0)
//		{
//			temp_angle = M_PI / 2;
//		}
//		else if (dy == 0 && dx < 0)
//		{
//			temp_angle = -M_PI / 2;
//		}
//		else
//		{
//			temp_angle = atan2(dy, dx);
//		}
//		// -PI~0 -> PI~2PI 
//		if (temp_angle < 0)
//		{
//			temp_angle += 2 * M_PI;
//		}
//
//
//		if (temp_angle > max_angle)
//		{
//			max_angle = temp_angle;
//			max_index = i;
//		}
//		if (temp_angle < min_angle)
//		{
//			min_angle = temp_angle;
//			min_index = i;
//		}
//
//	}
//
//	//// HL: here the start and end angle may be set wrong, min_angle-> max_angle, or max_angle -> min_angle+2PI
//	//this->start_angle_ = min_angle;
//	//this->end_angle_ = max_angle;
//}
//


//
//double Planner::calAngle(std::pair<double, double> critical_points, Node node_father)
//{
//
//	double dx = round(critical_points.first) - node_father.seed_.first;
//	double dy = round(critical_points.second) - node_father.seed_.second;
//	double temp_angle;
//	// Calculate the angle of seed->
//	if (dy == 0 && dx > 0)
//	{
//		temp_angle = M_PI / 2;
//	}
//	else if (dy == 0 && dx < 0)
//	{
//		temp_angle = -M_PI / 2;
//	}Open_list_.push_back(make_pair(Nindex, node_son.Fcost_));
//	temp_angle = atan2(dy, dx);
//}
//// -PI~0 -> PI~2PI 
//if (temp_angle < 0)
//{
//	temp_angle += 2 * M_PI;
//}
//return temp_angle;
//
//}
//
//vector<int>& Planner::Find_side_points(std::vector<CriticalPoint>& vertices, Node node_father) {
//	std::vector<std::pair<int, double>> angle_list;
//	angle_list.reserve(vertices.size());
//	std::vector<int> expand_point_list;
//
//	for (int i = 0; i < vertices.size(); ++i) {
//		std::pair<double, double> point_i = vertices.at(i).position_;
//		double angle_i = CalAngle(point_i, node_father);
//		if (!NodeCollision(point_i, node_father.seed_)) {
//			angle_list.push_back(make_pair(i, angle_i));
//		}
//	}
//	if (angle_list.size() == 1) {
//		expand_point_list.push_back(angle_list.at(0).first);
//	}
//	else if (angle_list.size() > 1) {
//		auto maxPosition = max_element(angle_list.begin(), angle_list.end());
//		auto minPosition = min_element(angle_list.begin(), angle_list.end());
//
//		expand_point_list.push_back(angle_list.at(maxPosition - angle_list.begin()).first);
//		expand_point_list.push_back(angle_list.at(minPosition - angle_list.begin()).first);
//
//	}
//	return expand_point_list;
//}

// //int NodeIsNew(std::vector<std::map<int, std::pair<int, int>> > node_exist_map, Node node_i);
// int Planner::NodeIsNew(std::vector<std::map<int, std::pair<int, int>> > node_exist_map, Node node_i)
// {
// 	int flag;
// 	std::map<int, std::pair<int, int>> seed_node_list = node_exist_map[node_i.seed_.second * this->xsize_ + node_i.seed_.first];
// 	std::map<int, std::pair<int, int>>::iterator iter = seed_node_list.find(node_i.fatherindex_);
// 	if (iter != seed_node_list.end()) {
// 		flag = 1;
// 		std::cout << "This is a new node, at [" << node_i.seed_.first << ", " << node_i.seed_.second << "]. " << std::endl;
// 	}
// 	else {
// 		flag = 0;
// 		std::cout << "This node is expanded, to be checked in Open list or Close list." << std::endl;
// 	}
// 	return flag;

// }

// int Planner::NodeInOpen(std::vector<std::map<int, std::pair<int, int>> > node_exist_map, Node node_i)
// {
// 	int flag;
// 	std::map<int, std::pair<int, int>> seed_node_list = node_exist_map[node_i.seed_.second * this->xsize_ + node_i.seed_.first];
// 	std::map<int, std::pair<int, int>>::iterator iter = seed_node_list.find(node_i.fatherindex_);
// 	int flag = iter->second.second;// flag = 0: close_list, flag = 1: open_que_
// 	int exist_node_index = iter->second.first;
// 	return flag, exist_node_index;

// }

// void Planner::Planner_mergeNode(Costmap& pMap, std::pair<double, double> start_position, std::pair<double, double> goal_position) {
// 	this->xsize_ = pMap.xsize_;
// 	this->ysize_ = pMap.ysize_;
// 	int Nindex = 0;
// 	Node start_node_(start_position, Nindex, Nindex);
// 	start_node_.start_angle_ = 0;
// 	start_node_.end_angle_ = 2 * M_PI;
// 	start_node_.Gcost_ = 0;
// 	start_node_.Hcost_ = distance(start_node_, goal_position);
// 	start_node_.Fcost_ = start_node_.Gcost_ + start_node_.Hcost_;

// 	priority_queue<Node> Open_que_;
// 	std::vector<int> Close_list_;

// //	std::sort --> std::min_element()

// 	// <father, <Nindex, flag_in_open>>
// 	std::vector<std::map<int, std::pair<int, int>> >  node_exist_map;// vector<fatherindex_ of this node, <Nindex, Open_que(1) / Close_list(0)>> at each seed, the same seed with different fatherindex means different nodes
// 	node_exist_map.reserve(pMap.xsize_ * pMap.ysize_);

// 	Open_que_.push(start_node_);
// 	node_exist_map[start_node_.seed_.second * pMap.xsize_ + start_node_.seed_.first].insert(make_pair(Nindex, make_pair(Nindex, 1)));// the fathernode of start_node is itself, and it is in the Open_que.


// 	while (!Open_que_.empty())
// 	{
// 		Nindex++;
// 		// Select the node with the minimum Fcost in the Openlist queue, and add new nodes near it
// 		Node node_cur = Open_que_.top(); 
// 		Open_que_.pop();
// 		Close_list_.push_back(node_cur.Nindex_);

// 		// Ergodic all possible critical points£¨2 of N points£© of each obstacle
// 		for (int obs_index = 0; obs_index < pMap.obs_.size(); ++obs_index) {
// 			std::vector<std::pair<int, int>> point_to_check = find_side_points(pMap.obs_.back().vertices_.at(obs_index), node_cur);// find the critical points with the max and min angle respect to the seed point (cur_node_). select 2 critical points of a obstacle
// 			for (int point_index = 0; point_index < point_to_check.size(); ++point_index)
// 			{
// 				std::pair<int, int> point_i = point_to_check.at(point_index);
// 				Node node_i(point_i, Nindex, node_cur.Nindex_);
// 				node_i.Node_init();//calculate the start and end angle

// 				// if node doesn't exist
// 				if (NodeIsLegal(node_cur, point_i) && NodeIsNew(node_exist_map, node_i))
// 				{
// 					node_i.Gcost_ = node_cur.Gcost_ + distance(node_cur.seed_, node_i.seed_);
// 					node_i.Hcost_ = distance(node_i.seed_, goal_position);
// 					node_i.Fcost_ = node_i.Gcost_ + node_i.Hcost_;
// 					node_i.fatherindex_ = node_cur.Nindex_;
// 					Open_que_.push(node_i);
// 					node_exist_map[node_i.seed_.second * pMap.xsize_ + node_i.seed_.first].insert(make_pair(node_cur.Nindex_, make_pair(Nindex, 1)));
// 				}
// 				else {
// 					//node exist: node_in open_que -> check whether its fatherindex need to be changed, node in close_list -> continue
// 					int flag_in_open, int node_exist_index = NodeInOpen(node_exist_map, node_i);
// 					if (flag_in_open) {
// 						node_i.Gcost_ = node_cur.Gcost_ + distance(node_cur.seed_, node_i.seed_);
// 						node_i.Hcost_ = distance(node_i.seed_, goal_position);
// 						node_i.Fcost_ = node_i.Gcost_ + node_i.Hcost_;
// 						node_i.fatherindex_ = node_cur.Nindex_;
// 						Open_que_.push(node_i);
// 						//node_exist_map[node_i.seed_.second * pMap.xsize_ + node_i.seed_.first].insert(make_pair(node_cur.Nindex_, make_pair(Nindex, 1)));

// 						//Node node_exist = Open_que_.at(node_exist_index);//
// 						//if (node_i.Fcost_ < node_exist.Fcost_)
// 						//{
// 						//	node_exist.fatherindex_ = node_cur.Nindex_;
// 						//	node_exist.Gcost_ = node_i.Gcost_;
// 						//	node_exist.Hcost_ = node_i.Hcost_;
// 						//	node_exist.Fcost_ = node_i.Fcost_;
// 						//}
// 					}
// 				}

// 				//else if (NodeInOpen(node_exist_map, node_i))//node is in open_que_
// 				//{
// 				//	
// 				//}


// 			}


// 		}

// }
//
//// We find the ray (seed_ -> c_) hits which obstacle and which edge the crossing(intersection) position locates on.
//// the index of obstacle and the start and end positions of the edge are recorded.
//// The ray may crosses several edges, choose the nearest one to c_, and the distance is noted as distance_to_c_.
//void Node::findOForLastChild(const Costmap* pMap)
//{
//	double min_dis = 10000.0;
//	C_.back().the_index_of_obstacle_that_o_hits_ = -1;
//	C_.back().the_index_of_s_of_the_edge_that_o_hits_ = -1;
//	std::pair<double, double> the_intersection;
//	for (auto iter = pMap->obs_.begin(); iter != pMap->obs_.end(); ++iter)
//	{
//		for (unsigned int j = 0; j < iter->ordered_vertices_.size(); ++j)
//		{
//			int next_index = (j == iter->ordered_vertices_.size() - 1) ? 0 : j + 1;
//			if (findIntersectionBetweenFarRayAndSegment(seed_, C_.back().c_,
//				iter->ordered_vertices_[j].loc_, iter->ordered_vertices_[next_index].loc_,
//				the_intersection))
//			{
//				double the_dis = distance(seed_, the_intersection);
//				if (the_dis < min_dis)
//				{
//					min_dis = the_dis;
//					C_.back().the_index_of_obstacle_that_o_hits_ = iter - pMap->obs_.begin();
//					C_.back().the_index_of_s_of_the_edge_that_o_hits_ = j;
//				}
//			}
//		}
//	}
//}
////