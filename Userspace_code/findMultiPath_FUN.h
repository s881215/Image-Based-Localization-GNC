#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include "DIP_FUN.h"

using namespace std;
using namespace cv;

struct Endpoint{
	Point p;
	int contourIdx;
	int localIdx;
};

struct CandidateEdge{
	int ei,ej;
	double dist2;
};

void drawThickPolyline(Mat& mask,const vector<Point> &pts,int thickness);
int myMinDist(const vector<Point>& a,const vector<Point>& b);
void findEndPoints(const vector<Point>& pts,Point& p1,Point& p2);
void sortByEndpointPolar(vector<vector<Point>> &endPoints,vector<vector<Point>> &centerPts,vector<vector<Point>> &contours0);
double pointSegmentDistance(const Point2f& p,const Point2f& a,const Point2f &b);
bool hitsAnyCenterline(const Point& p,int contourIdxP,const Point& q,int contourIdxQ,const vector<vector<Point>> &centerPts,double tol);
void mySort(vector<Point>& referencePath);
void generatePath(const Point& startPoint,const Point& endPoint,int direction,vector<Point>& referencePath);
void findReferencePath_Multi(int seg,const Mat& K0,const Mat& dist0,const Mat& K1,const Mat& dist1);
bool loadMultiPaths(vector<Point2f>& pts,int paths_idx);
