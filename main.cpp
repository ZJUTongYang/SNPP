#include <iostream>
//#include "Atom.h"
#include "Definitions.h"
#include "map.h"
#include "Node.h"
#include "Planner.h"
#include <fstream>
#include <opencv2/opencv.hpp>
//#include <algorithm>
#include "visualize_functions.h"
#include <chrono>
#include "homotopy_aware_astar.h"

using namespace std;

int main()
{
	std::string data_folder = "";
	std::string log_filename = data_folder + "log_file.txt";

	std::pair<double, double> robot_position;
	std::pair<double, double> goal_position;

	bool non_selfcrossing = true;
	int K =6;
		
	cv::Mat img;
	cv::Mat img_grey;

	robot_position.first = 10;
	robot_position.second = 10;
	goal_position.first = 140;
	goal_position.second = 140;

	enum ALL_CASES { _3, _6, _7, _11, _22, _28 };

	ALL_CASES the_case = _28;
	switch (the_case)
	{
	case _3:
		img = cv::imread(data_folder + "maps\\black_3.png", 0);
		img_grey = cv::imread(data_folder + "maps\\3.png", 0);
		std::cout << "map index: 3" << std::endl;
		break;
	case _6:
		img = cv::imread(data_folder + "maps\\black_6.png", 0);
		img_grey = cv::imread(data_folder + "maps\\6.png", 0);
		std::cout << "map index: 6" << std::endl;
		break;
	case _7:
		img = cv::imread(data_folder + "maps\\black_7.png", 0);
		img_grey = cv::imread(data_folder + "maps\\7.png", 0);
		std::cout << "map index: 7" << std::endl;

		break;
	case _11:
		img = cv::imread(data_folder + "maps\\black_11.png", 0);
		img_grey = cv::imread(data_folder + "maps\\11.png", 0);
		std::cout << "map index: 11" << std::endl;

		break;
	case _22:
		img = cv::imread(data_folder + "maps\\black_22.png", 0);
		img_grey = cv::imread(data_folder + "maps\\22.png", 0);
		std::cout << "map index: 22" << std::endl;

		break;
	case _28:
		img = cv::imread(data_folder + "maps\\black_28.png", 0);
		img_grey = cv::imread(data_folder + "maps\\28.png", 0);
		std::cout << "map index: 28" << std::endl;
		break;
	}


	cv::imshow("test", img);

	if (img.empty())
	{
		return -1;
	}
	int height = img.rows;
	int width = img.cols;
	int ch = img.channels();

	Param param1;
	param1.xsize_ = width;
	param1.ysize_ = height;
//	std::cout << "check map size (should be the same as the #define-d dimension) height: " << height << ", width: " << width << ", ch: " << ch << std::endl;

	int repeat_test_count = 20;

	std::vector<double> all_planning_time;
	all_planning_time.resize(repeat_test_count);
	double sum_planning_time = 0;
	double sum_map_time = 0;

#if USE_OURS_BUT_NOT_EXISTING
	std::cout << "Using ours: " << std::endl;
#else
	std::cout << "using homotopy-aware A*: " << std::endl;
#endif


	for (unsigned int i = 0; i < repeat_test_count; ++i)
	{
		auto map_start_time = std::chrono::high_resolution_clock::now();

		//	clock_t map_start_time, map_end_time;
		//	map_start_time = clock();

		Costmap theMap(param1, img);

		//	map_end_time = clock();
	//	std::cout << "time for map prepartion: " << (double)(map_end_time - map_start_time) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

		auto map_end_time = std::chrono::high_resolution_clock::now();
		auto map_duration = std::chrono::duration_cast<std::chrono::microseconds>(map_end_time - map_start_time);
		std::cout << "time for map preparation: " << map_duration.count() / 1000.0 << "ms" << std::endl;
		sum_map_time += map_duration.count() / 1000.0;

		auto planner_start_time = std::chrono::high_resolution_clock::now();

#if USE_OURS_BUT_NOT_EXISTING

		//	clock_t planner_start_time, planner_end_time;
		//	planner_start_time = clock();
		Planner thePlanner(&theMap, log_filename, non_selfcrossing, K,
			robot_position, goal_position);
		//	planner_end_time = clock();
		//	std::cout << "time for planning: " << (double)(planner_end_time - planner_start_time) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
#else
		// We test homotopy-aware A*
		int option =2;// 0: no non-selfcrossing checking, 1: checking the resultant path, 2: checking all waypoints
		Kim2013::Kim2013 theHomotopyAwareAstar(&theMap, non_selfcrossing, K, option, 
			robot_position, goal_position);
#endif
		auto planner_end_time = std::chrono::high_resolution_clock::now();
		auto planner_duration = std::chrono::duration_cast<std::chrono::microseconds>(planner_end_time - planner_start_time);
		std::cout << "time for planning: " << planner_duration.count() / 1000.0 << "ms" << std::endl;

		all_planning_time[i] = planner_duration.count() / 1000.0;
		sum_planning_time += planner_duration.count() / 1000.0;

	}

	std::cout << "average map time: " << sum_map_time / repeat_test_count << std::endl;
	std::cout << "average testing time: " << sum_planning_time / repeat_test_count << std::endl;

	auto map_start_time = std::chrono::high_resolution_clock::now();

	//	clock_t map_start_time, map_end_time;
	//	map_start_time = clock();

	Costmap theMap(param1, img);

	//	map_end_time = clock();
//	std::cout << "time for map prepartion: " << (double)(map_end_time - map_start_time) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

	auto map_end_time = std::chrono::high_resolution_clock::now();
	auto map_duration = std::chrono::duration_cast<std::chrono::microseconds>(map_end_time - map_start_time);
	std::cout << "time for map preparation: " << map_duration.count() / 1000.0 << "ms" << std::endl;

	auto planner_start_time = std::chrono::high_resolution_clock::now();

	//	clock_t planner_start_time, planner_end_time;
	//	planner_start_time = clock();
	Planner thePlanner(&theMap, log_filename, non_selfcrossing, K,
		robot_position, goal_position);
	//	planner_end_time = clock();
	//	std::cout << "time for planning: " << (double)(planner_end_time - planner_start_time) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

	auto planner_end_time = std::chrono::high_resolution_clock::now();
	auto planner_duration = std::chrono::duration_cast<std::chrono::microseconds>(planner_end_time - planner_start_time);
	std::cout << "time for planning: " << planner_duration.count() / 1000.0 << "ms" << std::endl;


	// Visualize k shortest non-homotopy paths
	// If RECORD_PATH_LOG == true, store the paths position in the log_path_filename
	std::string log_path_filename = data_folder + "log_path_file.txt";
	std::string vis_map_filename = data_folder + "visualize_kPaths_map.png";
	visualize_kPaths_in_paper(img_grey, thePlanner, robot_position, goal_position, height, log_path_filename, vis_map_filename);

	//std::string log_V_filename = data_folder + "log_V_file.txt";
	//visualize_kPaths(img, thePlanner, robot_position, goal_position, height, log_path_filename, vis_map_filename, log_V_filename);

	//	visualize_kPaths_individual(img_grey, thePlanner, robot_position, goal_position, height, log_path_filename, vis_map_filename);


 	cv::waitKey(0);
	//return 0;
}