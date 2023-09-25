#pragma once
#include <iostream>


double distance(const std::pair<double, double>& s, const std::pair<double, double>& g);



// HL 2023.8.28
double outerProduct(std::pair<double, double> vector1, std::pair<double, double> vector2);
// HL 2023.8.28
std::pair<double, double> pairMinus(std::pair<double, double> a, std::pair<double, double> b);

// HL 2023.8.28
std::pair<double, double> calIntersection(std::pair<double, double> s1, std::pair<double, double> g1, std::pair<double, double> s2, std::pair<double, double> g2);

// HL 2023.8.27
bool onRay(std::pair<double, double> seed_ray, std::pair<double, double> point_ray, std::pair<double, double> intersection);

// HL 2023.8.28
bool onSegment(std::pair<double, double> seed_ray, std::pair<double, double> point_ray, std::pair<double, double> intersection);

// YT: 2023.8.28
bool findIntersectionBetweenFarRayAndSegment(const std::pair<double, double>& p1,
	const std::pair<double, double>& p2, const std::pair<double, double>& p3,
	const std::pair<double, double>& p4, std::pair<double, double>& p);

// YT: 2023.8.28
bool isColinear(std::pair<double, double> p1, std::pair<double, double> p2, std::pair<double, double> p3);

// YT 2023.8.24
bool isCrossing(std::pair<double, double> s1, std::pair<double, double> g1,	std::pair<double, double> s2, std::pair<double, double> g2);

bool isSelfCrossing(std::pair<double, double> s1, std::pair<double, double> g1,
	const std::vector<std::pair<double, double> >& previous_tether);

// YT 2023.8.24
double normalize_angle(const double theta);

// YT 2023.8.24
double normalize_angle_positive(const double theta);

double pathLength(const std::vector<std::pair<double, double> >& the_path);

bool pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);

bool pnpoly(const std::vector<std::pair<double, double> >& ver, double testx, double testy);




