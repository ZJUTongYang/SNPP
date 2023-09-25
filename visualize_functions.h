#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2\imgproc\types_c.h>
#include "Planner.h""

std::vector<cv::Scalar> create_colormap_(int color_num);

void visualize_kPaths(cv::Mat& img, Planner& thePlanner,
	std::pair<double, double> robot_position, std::pair<double, double> goal_position, int height,
	std::string log_path_filename, std::string vis_map_filename, std::string log_V_filename);

void visualize_kPaths_in_paper(cv::Mat& img, Planner& thePlanner, 
	std::pair<double, double> robot_position, std::pair<double, double> goal_position, int height, std::string log_path_filename, std::string vis_map_filename);

void visualize_kPaths_individual(cv::Mat& img, Planner& thePlanner,
	std::pair<double, double> robot_position, std::pair<double, double> goal_position, int height, std::string log_path_filename, std::string vis_map_filename);
