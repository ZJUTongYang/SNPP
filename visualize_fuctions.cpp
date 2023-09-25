#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2\imgproc\types_c.h>
#include "Planner.h""

using namespace std;


std::vector<cv::Scalar> create_colormap_(int color_num) {
	std::vector<double> r;
	std::vector<cv::Scalar> c;

	for (int i = 0; i < color_num; ++i) {
		double temp1 = double(i) / ((color_num > 2) ? color_num - 1 : 1);
		r.emplace_back(temp1);

		//cv::Scalar temp2 = cv::Scalar( (0.8-r[i]) * 255.0, (1.0 - 0.5 * r[i]) * 255.0, r[i] * 255.0 ); // colormap SUMMER £¨in matlab rgb£© -> bgr
		cv::Scalar temp2 = cv::Scalar((0.4) * 255.0, (0.4 + 0.5 * r[i]) * 255.0, r[i] * 255.0);

		c.emplace_back(temp2);
	}
	return c;
}

void visualize_kPaths(cv::Mat& img, Planner& thePlanner,
	std::pair<double, double> robot_position, std::pair<double, double> goal_position, int height, std::string log_path_filename, std::string vis_map_filename, std::string log_V_filename) {
	// Convert grey img to rgb img_plot
	cv::Mat img_plot = cv::Mat(img.rows, img.cols, CV_8UC3);
	cvtColor(img, img_plot, CV_GRAY2BGR);

	// plot k shortest paths, each path stored in path list (position saved)
	// Nindex of nodes on each path are saved in path Nindex list, for plotting the V
	std::vector<std::vector<cv::Point> > path_list(thePlanner.path_solutions_.size());
	std::vector<std::vector<int> > path_Nindex_list(thePlanner.path_solutions_.size());

	// Plot k shortest paths, each path stored in path_list
//	std::vector<std::vector<cv::Point> > path_list;
	if (thePlanner.path_solutions_.size() > 0) {
		// pre-setting colors of k paths
		std::vector<cv::Scalar> path_colors = create_colormap_(thePlanner.path_solutions_.size());

		for (int path_i = 0; path_i < thePlanner.path_solutions_.size(); ++path_i) {// for all paths
			std::vector<cv::Point> each_path_points;
			cv::Point prev_point = cv::Point(thePlanner.path_solutions_[path_i].path_[0].first, height - thePlanner.path_solutions_[path_i].path_[0].second);
			each_path_points.push_back(prev_point);

			std::vector<int > each_path_points_Nindex;
			each_path_points_Nindex.push_back(thePlanner.path_solutions_[path_i].path_node_index_[0]);

			for (int path_node_i = 1; path_node_i < thePlanner.path_solutions_[path_i].path_.size(); ++path_node_i) { //for all points in a path
				cv::Point this_point = cv::Point(thePlanner.path_solutions_[path_i].path_[path_node_i].first, height - thePlanner.path_solutions_[path_i].path_[path_node_i].second);
				each_path_points.push_back(this_point);
				//local_shortest_path_nindex_ only have thePlanner.path_solutions_[path_i].path_.size()-1 elements, because the goal is not a node
				each_path_points_Nindex.push_back(thePlanner.path_solutions_[path_i].path_node_index_[path_node_i]);

				line(img_plot, prev_point, this_point, path_colors[path_i], 3);
				prev_point = this_point;
			}
			std::cout << "each_path_points = " << each_path_points.size() << std::endl;
			path_list.emplace_back(each_path_points);
			path_Nindex_list.emplace_back(each_path_points_Nindex);
		}
	}
	cv::circle(img_plot, cv::Point(robot_position.first, height - robot_position.second), 2, cv::Scalar(255, 0, 0), 2);//blue
	cv::circle(img_plot, cv::Point(goal_position.first, height - goal_position.second), 2, cv::Scalar(0, 0, 255), 2);//red
	cv::imshow("visualize_map", img_plot);
	cv::imwrite(vis_map_filename, img_plot);

#if RECORD_PATH_LOG
	{
		std::ofstream ofs1;
		ofs1.open(log_path_filename, std::ios::app);
		if (path_list.size() > 0) {
			for (int path_i = 0; path_i < path_list.size(); ++path_i) {
				for (int path_node_i = 1; path_node_i < path_list[path_i].size(); ++path_node_i) {
					ofs1 << path_list[path_i][path_node_i].x << " "
						<< height - path_list[path_i][path_node_i].y << " "
						<< path_Nindex_list[path_i][path_node_i] << " ";
				}
				ofs1 << std::endl;
			}
		}
		std::ofstream ofs2;// poly position of V
		ofs2.open(log_V_filename, std::ios::app);
		// We store all the governed regions of each node, according to the order of Nindex
		for (int node_index = 0; node_index < thePlanner.N_.size(); ++node_index) {
			for (auto iter = thePlanner.N_[node_index].V_.begin(); iter != thePlanner.N_[node_index].V_.end(); ++iter)
			{
				ofs2 << iter->first << " " << iter->second << " ";
			}
			ofs2 << std::endl;
		}
	}
#endif

}


void visualize_kPaths_in_paper(cv::Mat& img, Planner& thePlanner, 
	std::pair<double, double> robot_position, std::pair<double, double> goal_position, int height, std::string log_path_filename, std::string vis_map_filename) {
	// Convert grey img to rgb img_plot
	cv::Mat img_plot = cv::Mat(img.rows, img.cols, CV_8UC3);
	cvtColor(img, img_plot, CV_GRAY2BGR);

	// pre-setting colors of k paths
	std::vector<cv::Scalar> path_colors = { 
		cv::Scalar(50, 100, 50), 
		cv::Scalar(50, 180, 50), 
		cv::Scalar(50, 155, 50), 
		cv::Scalar(90, 90, 165), 
		cv::Scalar(90, 165, 90), 
		cv::Scalar(165, 90, 90),
		cv::Scalar(120, 199, 199), 
		cv::Scalar(199, 120, 199), 
		cv::Scalar(199, 199, 120)};

	// Plot k shortest paths, each path stored in path_list
	std::vector<std::vector<cv::Point> > path_list;
	if (thePlanner.path_solutions_.size() > 0) {
		for (int path_i = 0; path_i < thePlanner.path_solutions_.size(); ++path_i) {// for all paths
			std::vector<cv::Point> path_points;
			cv::Point prev_point = cv::Point(thePlanner.path_solutions_[path_i].path_[0].first, height - thePlanner.path_solutions_[path_i].path_[0].second);
			path_points.push_back(prev_point);
			for (int path_node_i = 1; path_node_i < thePlanner.path_solutions_[path_i].path_.size(); ++path_node_i) { //for all points in a path
				cv::Point this_point = cv::Point(thePlanner.path_solutions_[path_i].path_[path_node_i].first, height - thePlanner.path_solutions_[path_i].path_[path_node_i].second);
				//std:cout << "x = " << this_point.x << ", y = " << this_point.y << std::endl;
				path_points.push_back(this_point);
				line(img_plot, prev_point, this_point, path_colors[path_i], 3);
				prev_point = this_point;

			}
			path_list.push_back(path_points);
		}
	}
	cv::circle(img_plot, cv::Point(robot_position.first, height - robot_position.second), 2, cv::Scalar(255, 0, 0), 2);//blue
	cv::circle(img_plot, cv::Point(goal_position.first, height - goal_position.second), 2, cv::Scalar(0, 0, 255), 2);//red
	cv::imshow("visualize_map", img_plot);
	cv::imwrite(vis_map_filename, img_plot);

#if RECORD_PATH_LOG
	{
		std::ofstream ofs1;
		ofs1.open(log_path_filename, std::ios::app);
		if (path_list.size() > 0) {
			for (int path_i = 0; path_i < path_list.size(); ++path_i) {
				for (int path_node_i = 1; path_node_i < path_list[path_i].size(); ++path_node_i) {
					ofs1 << path_list[path_i][path_node_i].x << " "
						<< height - path_list[path_i][path_node_i].y << " " << std::endl;
				}
				ofs1 << std::endl;
			}
		}
	}
#endif

}


void visualize_kPaths_individual(cv::Mat& img, Planner& thePlanner,
	std::pair<double, double> robot_position, std::pair<double, double> goal_position, int height, std::string log_path_filename, std::string vis_map_filename) {

	if (thePlanner.path_solutions_.size() == 0)
		return;



	// pre-setting colors of k paths
	std::vector<cv::Scalar> path_colors = {
		cv::Scalar(50, 100, 50),
		cv::Scalar(50, 180, 50),
		cv::Scalar(50, 155, 50),
		cv::Scalar(90, 90, 165),
		cv::Scalar(90, 165, 90),
		cv::Scalar(165, 90, 90),
		cv::Scalar(120, 199, 199),
		cv::Scalar(199, 120, 199),
		cv::Scalar(199, 199, 120) };

	// Plot k shortest paths, each path stored in path_list
	//std::vector<std::vector<cv::Point> > path_list;
	for (int path_i = 0; path_i < thePlanner.path_solutions_.size(); ++path_i) {// for all paths
		// Convert grey img to rgb img_plot
		cv::Mat img_plot = cv::Mat(img.rows, img.cols, CV_8UC3);
		cvtColor(img, img_plot, CV_GRAY2BGR);

		std::vector<cv::Point> path_points;
		cv::Point prev_point = cv::Point(thePlanner.path_solutions_[path_i].path_[0].first, height - thePlanner.path_solutions_[path_i].path_[0].second);
		path_points.push_back(prev_point);
		for (int path_node_i = 1; path_node_i < thePlanner.path_solutions_[path_i].path_.size(); ++path_node_i) { //for all points in a path
			cv::Point this_point = cv::Point(thePlanner.path_solutions_[path_i].path_[path_node_i].first, height - thePlanner.path_solutions_[path_i].path_[path_node_i].second);
			//std:cout << "x = " << this_point.x << ", y = " << this_point.y << std::endl;
			path_points.push_back(this_point);
			line(img_plot, prev_point, this_point, path_colors[path_i], 3);
			prev_point = this_point;

		}
		//path_list.push_back(path_points);

		cv::circle(img_plot, cv::Point(robot_position.first, height - robot_position.second), 2, cv::Scalar(255, 0, 0), 2);//blue
		cv::circle(img_plot, cv::Point(goal_position.first, height - goal_position.second), 2, cv::Scalar(0, 0, 255), 2);//red
		cv::imshow("visualize_map"+std::to_string(path_i), img_plot);

	}
	

}
