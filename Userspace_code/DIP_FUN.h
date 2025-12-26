#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
using namespace cv;
using namespace std;

extern vector<Point> midPoints;

Mat undistortImage(const Mat&,const Mat&,const Mat&);
bool computeBox(const Mat&,const Mat&,const Mat&,const Mat&,Rect&,Mat&);
bool extractReferencePoint(const Mat&,const Rect&,Mat&,Mat&,int&,int&);
bool computeNewHmatrix(double,double,const Mat&,Mat&,Mat&);
Point2d pix2world(double,double,const Mat&);
Mat draw_axes_canvas(double,double);
bool computeHmatrix();
bool computeHmatrix2(bool);
bool computeNewHmatrix_v2(double,double,const Mat&,Mat&,Mat&);
Point2d img2Ground(double,double,const Mat&);
void myThinning(const Mat& srcImg,const Mat& K,const Mat& dist ,const vector<vector<Point>>& contours/*contours*/,int targetIdx,vector<Point>* skelPts);
void findPath(const Mat& bk_img,const Mat& path_img,const Mat& K,const Mat& dist,const int segmentation);
bool loadRefPoints(vector<Point2f>& pts);
vector<Point2f> sliceMeans(const Mat& mask,const vector<Point>& contour,int n);
void findPath_version2(const Mat& bk_img,const Mat& path_img,const Mat& K,const Mat& dist,const int segmentation);
