#include <iostream>
#include "Definitions.h"
#include "atom_functions.h"

using namespace std;

// Calculating the outer product of vector1 and vector2.
double outerProduct(std::pair<double, double> vector1, std::pair<double, double> vector2) {
	double result = vector1.first * vector2.second - vector1.second * vector2.first;
	return result;
}


double distance(const std::pair<double, double>& s, const std::pair<double, double>& g)
{
	double dx = s.first - g.first;
	double dy = s.second - g.second;
	return sqrt(dx*dx + dy * dy);
}

// The minus between pair<double>, to calculate vector from point a to point b.
std::pair<double, double> pairMinus(std::pair<double, double> a, std::pair<double, double> b) {
	std::pair<double, double> result = std::make_pair(b.first - a.first, b.second - a.second);
	return result;
}

// Calculate the intersection point of the line s1-g1 and line s2-g2
// whereas the intersection is on the line s1-g1, (intersection - s1) x (g1 - s1) == 0, noting here is the outer product
// and the intersection is on the line s2-g2, (intersection - s2) x (g2 - s2) == 0
// k = (g2-Inter) : (Inter-s2) = Area(s1-g2-g1) : Area(s1-s2-g1) = (s1g2 x s1g1) : (s1s2 x s1g1)
// Inter.x = (Area(s1-g2-g1) * s2.x + Area(s1-s2-g1) * g2.x) / (Area(s1-g2-g1) + Area(s1-s2-g1)) = (k * s2.x + g2.x) / (1 + k)
// Inter.y = (k * s2.y + g2.y) / (1 + k)
// Referring to url{https://blog.csdn.net/yzf279533105/article/details/130954313}.
std::pair<double, double> calIntersection(std::pair<double, double> s1, std::pair<double, double> g1, std::pair<double, double> s2, std::pair<double, double> g2) {
	double k = outerProduct(pairMinus(g2, s1), pairMinus(g1, s1)) / outerProduct(pairMinus(s2, s1), pairMinus(g1, s1));
//	std::pair<double, double> Intersection = std::make_pair((k * s2.first + g2.first) / (1 + k), (k * s2.second + g2.second) / (1 + k));
	return std::pair<double, double>((k * s2.first + g2.first) / (1 + k), (k * s2.second + g2.second) / (1 + k));
}


// Checking whether the point intersection is on the RAY of seed_ray->point_ray, 
// supposing the point is on the line of seed_ray-point_ray.
// (point_ray - seed_ray) * (intersection - seed_ray) > 0, noting here is inner product
// flag > 0 : on the RAY, flag < 0: on the opposite side
bool onRay(std::pair<double, double> seed_ray, std::pair<double, double> point_ray, std::pair<double, double> intersection) {
	//double flag_on_ray = (point_ray.first - seed_ray.first) * (intersection.second - seed_ray.second) - (point_ray.second - seed_ray.second) * (intersection.first - seed_ray.first);
	double flag_on_ray;
	if (intersection == seed_ray || intersection == point_ray) {
		flag_on_ray = true;
	}
	else {
		flag_on_ray = ((point_ray.first - seed_ray.first) * (intersection.first - seed_ray.first) > 0 || (point_ray.second - seed_ray.second) * (intersection.second - seed_ray.second) > 0);
	}
	return flag_on_ray;
}

// Checking whether the point intersection is on the segment of segment_s - segment_e.
// (intersection - segment_s) * (intersection - segment_e) < 0, noting here is inner product
bool onSegment(std::pair<double, double> segment_s, std::pair<double, double> segment_e, std::pair<double, double> intersection) {
	//double flag_on_ray = (intersection.first - segment_s.first) * (intersection.second - segment_e.second) - (intersection.second - segment_s.second) * (intersection.first - segment_e.first);
	double flag_on_segment;
	if (intersection == segment_s || intersection == segment_e) {
		flag_on_segment = true;
	}
	else {
		flag_on_segment = ((intersection.first - segment_s.first) * (intersection.first - segment_e.first) < 0 || (intersection.second - segment_s.second) * (intersection.second - segment_e.second) < 0);
	}
	return flag_on_segment;
}

bool isColinear(std::pair<double, double> p1, std::pair<double, double> p2, std::pair<double, double> p3)
{
	// judge whether the second point is in the middle of the first one and the third one
	// This function does not count endpoint contact
	double dx12 = p2.first - p1.first, dy12 = p2.second - p1.second, dx13 = p3.first - p1.first, dy13 = p3.second - p1.second;
	return fabs(dx12 * dy13 - dy12 * dx13) < RAYSTAR_EPS;
}



// Checking whether the lines s1-g1 and s2-g2 are crossing.
bool isCrossing(std::pair<double, double> s1, std::pair<double, double> g1,
	std::pair<double, double> s2, std::pair<double, double> g2)
{
	// In this function, we need to consider the endpoint contact. 
	// Otherwise, the path might go through a certain obstacle vertex

	if ((s1.first > g1.first ? s1.first : g1.first) < (s2.first < g2.first ? s2.first : g2.first) ||
		(s1.second > g1.second ? s1.second : g1.second) < (s2.second < g2.second ? s2.second : g2.second) ||
		(s2.first > g2.first ? s2.first : g2.first) < (s1.first < g1.first ? s1.first : g1.first) ||
		(s2.second > g2.second ? s2.second : g2.second) < (s1.second < g1.second ? s1.second : g1.second))
	{
		return false;
	}

	if ((((s1.first - s2.first)*(g2.second - s2.second) - (s1.second - s2.second)*(g2.first - s2.first))*
		((g1.first - s2.first)*(g2.second - s2.second) - (g1.second - s2.second)*(g2.first - s2.first))) > 0 ||
		(((s2.first - s1.first)*(g1.second - s1.second) - (s2.second - s1.second)*(g1.first - s1.first))*
		((g2.first - s1.first)*(g1.second - s1.second) - (g2.second - s1.second)*(g1.first - s1.first))) > 0)
	{
		return false;
	}
	return true;
}

bool isSelfCrossing(std::pair<double, double> s1, std::pair<double, double> g1,
	const std::vector<std::pair<double, double> >& previous_tether)
{
	if (previous_tether.size() <= 2)
	{
		// The tether state contains the source point of the node, so there are minimally two elements
		return false;
	}
	for (auto iter = previous_tether.begin(); iter != previous_tether.end()-2; ++iter)
	{
		// here endpoint contacts are considered
		if (isCrossing(s1, g1, *iter, *std::next(iter)))
		{
			return true;
		}
	}
	return false;
}

bool findIntersectionBetweenFarRayAndSegment(const std::pair<double, double>& p1, 
	const std::pair<double, double>& p2, const std::pair<double, double>& p3, 
	const std::pair<double, double>& p4, std::pair<double, double>& p)
{
	double dx_ray = p2.first - p1.first;
	double dy_ray = p2.second - p1.second;
	double dx_seg = p4.first - p3.first;
	double dy_seg = p4.second - p3.second;
	double denominator = dx_ray * dy_seg - dx_seg * dy_ray;
	if (fabs(denominator) < RAYSTAR_EPS)
	{
		return false;
	}

	double r = ((p1.second - p3.second)*dx_seg - (p1.first - p3.first)*dy_seg) / denominator;
	double s = ((p3.second - p1.second)*dx_ray - (p3.first - p1.first)*dy_ray) / (-denominator);
	if (s >= 0 && s <= 1 && r > 1)
	{
		p.first = p1.first + r * dx_ray;
		p.second = p1.second + r * dy_ray;
		return true;
	}
	
	// meaning that even if there will be an intersection, it is not within the given interval
	return false;
}

// -PI ~ PI
double normalize_angle(const double angle)
{
	const double result = fmod(angle + M_PI, 2.0*M_PI);
	if (result <= 0.0) return result + M_PI;
	return result - M_PI;
}

// 0 ~ 2*PI
double normalize_angle_positive(const double angle)
{
	const double result = fmod(angle, 2.0*M_PI);
	if (result < 0.0) return result + 2.0*M_PI;
	return result;
}

double pathLength(const std::vector<std::pair<double, double> >& the_path)
{
	double L = 0;
	for (unsigned int i = 0; i < the_path.size()-1; ++i)
	{
		L += distance(the_path[i], the_path[i + 1]);
	}
	return L;
}

bool pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
	int i, j;
	bool c = false;
	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
			c = !c;
	}
	return c;
}

bool pnpoly(const std::vector<std::pair<double, double> >& ver, double testx, double testy)
{
	int i, j;
	bool c = false;
	for (i = 0, j = ver.size() - 1; i < ver.size(); j = i++) {
		if (((ver[i].second > testy) != (ver[j].second > testy)) &&
			(testx < (ver[j].first - ver[i].first) * (testy - ver[i].second) / (ver[j].second - ver[i].second) + ver[i].first))
			c = !c;
	}
	return c;
}