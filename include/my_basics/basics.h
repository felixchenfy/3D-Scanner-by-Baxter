
/*This script is currently not used*/

#ifndef BASICS_H
#define BASICS_H

#include "stdio.h"
#include "iostream"
#include <string>
#include <vector>
#include <deque>
#include <algorithm>

#include <sstream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

using namespace std;

namespace my_basics
{

// Convert int to string, and fill it with zero before the number to make it specified width
string int2str(int num, int width, char char_to_fill='0');

// Please sort v1 and v2 first, then this functions returns the intersection of two vector.
vector<int> getIntersection(vector<int> v1, vector<int> v2);

void preTranslatePoint(const float T[4][4], float &x, float &y, float &z);
void preTranslatePoint(const vector<vector<float>> &T, float &x, float &y, float &z);

void inv(const float T_src[4][4], float T_dst[4][4]);

} // namespace my_basics

#endif