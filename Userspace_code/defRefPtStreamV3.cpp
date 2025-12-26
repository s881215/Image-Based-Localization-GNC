#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "DIP_FUN.h"
#include "CameraApi.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <atomic>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <chrono>
#include "path_points.h"
#include "findMultiPath_FUN.h"

using namespace std;
using namespace cv;

#define background_path "/home/chen-ting/visual_localization/v1/background.jpg"
#define origin_path "/home/chen-ting/visual_localization/v1/origin_point.jpg"
#define origin_OnCar_path "/home/chen-ting/visual_localization/v1/origin_pointOnCar.jpg"
#define H_path "homography_from_chessboard.yml"

bool check;

Mat H_img0_to_img1;

static string groundMsg;
static string carMsg;
bool defRefPtCheck=false;

int pathLen;
int pathIdx=1;
bool endPath=false;
double pxPre=0.0;
double pyPre=0.0;

int extend_left;
int extend_right;
int extend_top;
int extend_bottom;

uint8_t rxBuffer[128];

using steadyClock = std::chrono::steady_clock;
using chrono::seconds;

// single camera
/*
unsigned char *g_pRgbBuffer;
int iCameraCounts=1;
int iStatus=-1;
tSdkCameraDevInfo tCameraEnumList;
int hCamera;
tSdkCameraCapbility tCapability;
tSdkFrameHead sFrameInfo;
BYTE* pbyBuffer;
*/

// two cameras
//
constexpr int MAX_CAMERA_COUNTS=2;
unsigned char *g_pRgbBuffer;
unsigned char *g_pRgbBuffer2;
int iCameraCounts=MAX_CAMERA_COUNTS;
int iStatus=-1;
int iStatus2=-1;
tSdkCameraDevInfo tCameraEnumList[MAX_CAMERA_COUNTS];
int hCamera[MAX_CAMERA_COUNTS]={-1,-1};
tSdkCameraCapbility tCapability[MAX_CAMERA_COUNTS];
tSdkFrameHead sFrameInfo[MAX_CAMERA_COUNTS];
BYTE* pbyBuffer;
BYTE* pbyBuffer2;
int display_max=0;


int iDisplayFrames=200;
int channel=3;

static string makeGroundMsg(double x,double y){
        ostringstream oss;
        oss.setf(ios::fixed);
        oss << setprecision(2)
            << "G"<<x<<","<<y;
        return oss.str();
}
static string makeCarMsg(double x,double y){
        ostringstream oss;
        oss.setf(ios::fixed);
        oss << setprecision(2)
            << "C"<<x<<","<<y<<"\n";
        return oss.str();
}


int open_serial(const char* dev,speed_t baud=B115200){
        int fd=::open(dev,O_RDWR | O_NOCTTY | O_SYNC);
        if(fd<0) return -1;
        termios tio{};
        tcgetattr(fd,&tio);
        cfmakeraw(&tio);
        cfsetispeed(&tio,baud);
        cfsetospeed(&tio,baud);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CRTSCTS;
        //
	tio.c_cc[VMIN]=0;
	tio.c_cc[VTIME]=0;
	//
	tcsetattr(fd,TCSANOW,&tio);
        return fd;
}

Point2d vehPixToGround(Point2d& c){
	double h=0.15;
	FileStorage fs("paramsFromMatlab1.yml",FileStorage::READ);
	if(!fs.isOpened()){
		cerr<<"defRefPtStreamV3.cpp: vehPixToGround(): cannot open paramsFromMatlab1.yml"<<endl;
		return Point2f(0,0);
	}
	Mat K,dist;
	fs["K"]>>K;
	fs["dist"]>>dist;
	fs.release();
	FileStorage fsd("homography_from_chessboard1.yml",FileStorage::READ);
	if(!fsd.isOpened()){
		cerr<<"defRefPtStreamV3.cpp: vehPixToGround(): cannot open homography_from_chessboard1.yml"<<endl;
		return Point2f(0,0);
	}
	Mat R,T,H_img2w;
	fsd["R"]>>R;
	fsd["T"]>>T;
	fsd["H_img2w"]>>H_img2w;
	fsd.release();
	// (1)
	vector<Point2f> src{c}, und;
	undistortPoints(src,und,K,dist,noArray(),K);
	Point2d u=und[0];
	// (2)
	Mat Kinv=K.inv();
	Mat x=(Mat_<double>(3,1) <<(double)u.x,(double)u.y,1.0);
	Mat Rt=R.t();
	Mat r_mat=Rt*(Kinv*x);
	Mat C_mat=-Rt*T;
	const double rx=r_mat.at<double>(0), ry=r_mat.at<double>(1), rz=r_mat.at<double>(2);
	const double Cx=C_mat.at<double>(0), Cy=C_mat.at<double>(1), Cz=C_mat.at<double>(2);
	// (3)
	if(abs(rz)<1e-10){
		cerr<<"paralle to ground"<<endl;
	}
	const double s=(h-Cz)/rz;
	if(s<=0){
		cerr<<"ERROR"<<endl;
		return Point2d(0,0);
	}
	const double X=Cx+s*rx;
	const double Y=Cy+s*ry;
	// (5)
	Mat q=H_img2w.inv()*(Mat_<double>(3,1)<<X,Y,1.0);
	const double qw=q.at<double>(2);
	return Point2d((double)(q.at<double>(0)/qw), (double)(q.at<double>(1)/qw));
}


int defineNewHmatrix2(){
        Mat img_bg=imread("background0.jpg");
        Mat img_ori;
	if(check){
		img_ori=imread("originOnCar0.jpg");
	}else{
		img_ori=imread("origin0.jpg");
	}
        if(img_bg.empty()||img_ori.empty()){
                cerr<<"Loading Image Failed."<<endl;
                return -1;
        }
        FileStorage fs("paramsFromMatlab.yml",FileStorage::READ);
        if(!fs.isOpened()){
                cerr<<"Cannot open paramsNew.yml haha"<<endl;
                return -1;
        }
        Mat K,dist;
        fs["K"]>>K;
        fs["dist"]>>dist;
        fs.release();
        FileStorage fs1("homography_from_chessboard.yml",FileStorage::READ);
        if(!fs1.isOpened()){
                cerr<<"Cannot open homography_from_cheddboard.yml"<<endl;
                return -1;
        }
        Mat H_img2w, H_img2w_onCar;
        if(check){
		fs1["H_img2w_onCar"]>>H_img2w_onCar;
	}else{
		fs1["H_img2w"]>>H_img2w;
	}
        fs1.release();

        Rect box1;
        Mat und1;
        if(!computeBox(img_bg,img_ori,K,dist,box1,und1)){
                cerr<<"computeBox() failed"<<endl;
                return -1;
        }
	if(check){
		imwrite("position_1_onCar_undistort.jpg",und1);
	}else{
    		imwrite("position_1_undistort.jpg",und1);
	}
	Mat out1,mask1;
        int cx1=0,cy1=0;
        if(!extractReferencePoint(und1,box1,out1,mask1,cx1,cy1)){
                cerr<<"extractReferencePoint() failed"<<endl;
                return -1;
        }
	cout<<"cx: "<<cx1<<endl;
        cout<<"cy: "<<cy1<<endl;
        //imwrite("check.jpg",out1);
        Mat H_star,H_star_inv;
	Mat H_star_onCar,H_star_onCar_inv;
	FileStorage fs2("H_matrix_new.yml",FileStorage::APPEND);
	if(!fs2.isOpened()){
		cerr<<"Cannot open H_matrix_new.yml"<<endl;
			return -1;
	}
	if(check){
		if(!computeNewHmatrix((double)cx1,(double)cy1,H_img2w_onCar,H_star_onCar,H_star_onCar_inv)){
                        cerr<<"computeNewHmatrix() failed"<<endl;
                        return -1;
                }
                fs2<<"H_onCar_new"<<H_star_onCar;
                fs2<<"H_onCar_inv_new"<<H_star_onCar_inv;
	}else{
        	if(!computeNewHmatrix((double)cx1,(double)cy1,H_img2w,H_star,H_star_inv)){
                	cerr<<"computeNewHmatrix() failed"<<endl;
                	return -1;
        	}
        	fs2<<"H_new"<<H_star;
        	fs2<<"H_inv_new"<<H_star_inv;
	}
        //if(!computeNewHmatrix_v2((double)cx1,(double)cy1,H_img2w,H_star,H_star_inv)){
        //      cerr<<"computeNewHmatrix_v2 failed"<<endl;
        //      return -1;
        //}
        //FileStorage fs2("H_matrix_new.yml",FileStorage::WRITE);
        //fs2<<"H_new"<<H_star;
        //fs2<<"H_inv_new"<<H_star_inv;
        //fs2.release();
	fs2.release();
        return 0;
}
static Point toImagePoint(const Point& winPt,const Size& imgSize,const string& win){
#if (CV_VERSION_MAJOR>=4) && (CV_VERSION_MINOR>=5)
	Rect r=getWindowImageRect(win);
	if(r.width>0 && r.height>0 && imgSize.width>0 && imgSize.height>0){
		double sx=static_cast<double>(imgSize.width)/r.width;
		double sy=static_cast<double>(imgSize.height)/r.height;
		int ix=lround((winPt.x-r.x)*sx);
		int iy=lround((winPt.y-r.y)*sy);
		ix=clamp(ix,0,imgSize.width-1);
		iy=clamp(iy,0,imgSize.height-1);
		return {ix,iy};
	}
#endif
	printf("return winPt\n");
	return winPt;
}
Point2d projectH(const Mat& H,const Point2d& p){
        Mat Hd;
        H.convertTo(Hd,CV_64F);
        double u=p.x;
        double v=p.y;
        const double* h=Hd.ptr<double>(0);
        double X=h[0]*u+h[1]*v+h[2];
        double Y=h[3]*u+h[4]*v+h[5];
        double W=h[6]*u+h[7]*v+h[8];
        if(abs(W)<1e-12){
                return Point2d(numeric_limits<double>::quiet_NaN(),numeric_limits<double>::quiet_NaN());
        }
        return Point2d(X/W,Y/W);
}
Point2d img1ToRef_exact(const Mat& H1_img0_to_ref,const Mat& H2_img1_to_img0,const Point2d& p1_img1){
        Mat Hr = H1_img0_to_ref * H2_img1_to_img0;
        return projectH(Hr,p1_img1);
}
Point2d img1ToRef(Point2d& p1){
        FileStorage fss("H_matrix_new.yml",FileStorage::READ);
        Mat H_to_ref;
        fss["H_onCar_new"]>>H_to_ref;
        fss.release();
	FileStorage ffff("H_img0_to_img1.yml",FileStorage::READ);
	Mat H_img0_to_img1;
	ffff["H_img0_to_img1"]>>H_img0_to_img1;
	ffff.release();
        Point2d ref_p1=img1ToRef_exact(H_to_ref,H_img0_to_img1.inv(),p1);
        cerr<<"p1 to ref: "<<ref_p1<<endl;
        return ref_p1;
}
int getCarPosition(Mat& frame, int num,const Mat& K,const Mat& dist){
    Mat img_bg;
	if(num==0){
		img_bg=imread("background0.jpg");
	}else if(num==1){
		img_bg=imread("background1.jpg");
	}
	Mat img_ori=frame.clone();
    if(img_bg.empty()){
        cerr<<"defRefPtStreamV3.cpp: getCarPosition(): Loading Background Image Failed."<<endl;
        return -1;
	}
	if(frame.empty()){
		cerr<<"defRefPtStreamV3.cpp: getCarPosition(): Loading Frame Failed."<<endl;
		return -1;
	}
    Rect box1;
    Mat und1;
    if(!computeBox(img_bg,img_ori,K,dist,box1,und1)){
        cerr<<"defRefPtStreamV3.cpp: getCarPosition(): computeBox() failed."<<endl;
        return -1;
    }

    Mat out1,mask1;
    int cx1=0,cy1=0;
    if(!extractReferencePoint(und1,box1,out1,mask1,cx1,cy1)){
        cerr<<"defRefPtStreamV3.cpp: getCarPosition(): extractReferencePoint() failed."<<endl;
        return -1;
    }
	if(num==0){
		Mat H0;
		FileStorage fsh0("H_matrix_new.yml",FileStorage::READ);
		if(!fsh0.isOpened()){
			cerr<<"defRefPtStreamV3.cpp: getCarPosition(): cannot open H_matrix_new.yml"<<endl;
			return -1;
		}
		fsh0["H_onCar_new"]>>H0;
		fsh0.release();
		Point2d w=pix2world((double)cx1,(double)cy1,H0);
		FileStorage fdCar("recordCar.yml",FileStorage::APPEND);
		if(!fdCar.isOpened()){
			cerr<<"defRefPtStreamV3.cpp: getCarPosition(num==0): cannot open recordCar.yml"<<endl;
			return -1;
		}
		fdCar<<"record"<<"{"
			 <<"x"<<w.x
			 <<"y"<<w.y
			 <<"}";
		fdCar.release();
		cerr<<"Position of Car:"<<endl;
		cerr<<fixed<<setprecision(3)
			<<"x(m):"<<w.x<<", "<<"y(m):"<<w.y<<endl;
		cerr<<"-----------------------------------"<<endl;
		carMsg=makeCarMsg(w.x,w.y);
	}else if(num==1){
		/* previous version
		Point2d c=Point2d((double)cx1,(double)cy1);
		// project on ground 
		Point2d cc=vehPixToGround(c);
		Point2d w=img1ToRef(cc);
		FileStorage fdCar("recordCar.yml",FileStorage::APPEND);
		if(!fdCar.isOpened()){
			cerr<<"defRefPtStream.cpp: getCarPosition(num==1): cannot open recordCar.yml"<<endl;
			return -1;
		}
		fdCar<<"record"<<"{"
			 <<"x"<<w.x
			 <<"y"<<w.y
			 <<"}";
		fdCar.release();
		cerr<<"Position of Car:"<<endl;
		cerr<<fixed<<setprecision(3)
			<<"x(m):"<<w.x<<", "<<"y(m):"<<w.y<<endl;
		cerr<<"-----------------------------------"<<endl;
		carMsg=makeCarMsg(w.x,w.y);
		*/
		
		// tow reference coordinate version
		Mat H1;
		FileStorage fsh1("H_matrix_new1.yml",FileStorage::READ);
		if(!fsh1.isOpened()){
			cerr<<"defRefPtStreamV3.cpp: getCarPosition(): cannot open H_matrix_new1.yml"<<endl;
			return -1;
		}
		fsh1["H_onCar_new"]>>H1;
		fsh1.release();
		Point2d w=pix2world((double)cx1,(double)cy1,H1);
		FileStorage fdCar("recordCar.yml",FileStorage::APPEND);
		if(!fdCar.isOpened()){
			cerr<<"defRefPtStreamV3.cpp: getCarPosition(): cannot open recordCar.yml"<<endl;
			return -1;
		}
		fdCar<<"record"<<"{"
		     <<"x"<<(w.x-1.2)
		     <<"y"<<(w.y)
		     <<"}";
		fdCar.release();
		cerr<<fixed<<setprecision(3)
                        <<"x(m):"<<(w.x-1.2)<<", "<<"y(m):"<<(w.y)<<endl;
                cerr<<"==================================="<<endl;
                carMsg=makeCarMsg((w.x-1.2),(w.y));

	}
	/*
	Mat H;
    FileStorage fsh("H_matrix_new.yml",FileStorage::READ);
    if(!fsh.isOpened()){
        cerr<<"getCarPos(): cannot open H_matrix_new.yml"<<endl;
    }
    fsh["H_onCar_new"]>>H;
    fsh.release();
    Point2d w=pix2world((double)cx1,(double)cy1,H);
	FileStorage fdCar("recordCar.yml",FileStorage::APPEND);
    if(!fdCar.isOpened()){
        cerr<<"getCarPosition(): cannot open recordCar.yml"<<endl;
    }
    fdCar<<"record"<<"{"
         <<"x"<<w.x
         <<"y"<<w.y
         <<"}";
    fdCar.release();
	cerr<<"Position of Car:"<<endl;
	cerr<<fixed<<setprecision(3)
	    <<"x(m):"<<w.x<<", "<<"y(m):"<<w.y<<endl;
	cerr<<"-----------------------------------"<<endl;
	carMsg=makeCarMsg(w.x,w.y);
	*/
	return 1;
}

void findReferencePath(int segmentation){
	// load background and path image
	Mat img_bg=imread("background.jpg");
	Mat img_path=imread("path.jpg");
	if(img_bg.empty() || img_path.empty()){
		cerr<<"Loading image failed."<<endl;
		return;
	}
	// load camera parameters
	FileStorage sf("paramsFromMatlab.yml",FileStorage::READ);
	if(!sf.isOpened()){
		cerr<<"defRefPtStreamV3.cpp : findReferencePath() : Cannot open paramsFromMatlab.yml"<<endl;
		return;
	}
	Mat KKK,DIST;
	sf["K"]>>KKK;
	sf["dist"]>>DIST;
	sf.release();
	//
	cerr<<"Finding path...."<<endl;
	findPath_version2(img_bg,img_path,KKK,DIST,segmentation);
	cerr<<"Done.."<<endl;
}
/*
int IndustryCameraInit(){
	CameraSdkInit(1);
	
	iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
	printf("state = %d\n",iStatus);
	printf("count = %d\n",iCameraCounts);
	if(iCameraCounts==0){
		return -1;
	}
	iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
	printf("state = %d\n",iStatus);
	if(iStatus!=CAMERA_STATUS_SUCCESS){
		return -1;
	}
	
	CameraGetCapability(hCamera,&tCapability);
	
	g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
	CameraPlay(hCamera);
	
	return 0;
}
*/
bool computeHmatrix_for_second_camera(){
	const Size kBoardSize=Size(8,5);
        const double kSquareSize=0.03;
	Mat img0,img1;
        if(check){ // check==true : on car
		img1=imread("chessboardOnCar1.jpg",IMREAD_COLOR);
	}else{	   // check==false : on ground
		img1=imread("chessboard1.jpg",IMREAD_COLOR);
	}
	if(img1.empty()){
                cerr<<"chessboard image does not exit."<<endl;
                return false;
    }
    int w1=img1.cols;
    int h1=img1.rows;
    Mat und1=img1.clone();
    FileStorage fs1("paramsFromMatlab1.yml",FileStorage::READ);
    if(!fs1.isOpened()){
	cerr<<"can not open paramsFromMatlab1.yml"<<endl;
	return false;
    }
    Mat K0,dist0,K1,dist1;
    fs1["K"]>>K1;
    fs1["dist"]>>dist1;
    fs1.release();
    if(K1.empty() || dist1.empty()){
        cerr<<"lack of parameters K1 or dist1."<<endl;
        return false;
    }
    Mat newK1=getOptimalNewCameraMatrix(K1,dist1,Size(w1,h1),0);
    undistort(img1,und1,K1,dist1,newK1);

    Mat gray0,gray1;
    cvtColor(und1,gray1,COLOR_BGR2GRAY);
    vector<Point2f> corners1;
    bool ok1=findChessboardCorners(gray1,kBoardSize,corners1,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
#if (CV_VERSION_MAJOR >=4)
    if(!ok1){
        ok1 = findChessboardCornersSB(gray1,kBoardSize,corners1);
    }
#endif
    if(!ok1){
       	cerr<<"1 : cannot find the chessboard corners"<<endl;
       	return false;
    }
    cornerSubPix(gray1,corners1,Size(11,11),Size(-1,-1),TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER,40,1e-3));
    const int cols = kBoardSize.width;
    const int rows = kBoardSize.height;
    vector<Point2f> world_pts1;
    world_pts1.reserve(cols*rows);
    for(int r=0;r<rows;++r){
        for(int c=0;c<cols;++c){
            world_pts1.emplace_back(static_cast<float>(c*kSquareSize),static_cast<float>(r*kSquareSize));
        }
    }

    Mat inlierMask0, inlierMask1;
    Mat H0_1=findHomography(corners1,world_pts1,RANSAC,2.0,inlierMask1);
    if(H0_1.empty()){
        cerr<<"1: findHomography() failed"<<endl;
        return false;
    }
    cerr<<"H_1 (img -> world) :\n"<<H0_1<<endl;
    FileStorage fsd1("homography_from_chessboard1.yml",FileStorage::APPEND);
    if(check){
	fsd1<<"H_img2w_onCar"<<H0_1;
	fsd1<<"H_img2w_onCar_original"<<H0_1;
    }else{
	fsd1<<"H_img2w"<<H0_1;
	fsd1<<"H_original"<<H0_1;
    }
    fsd1.release();
    imwrite("inlierMask111.jpg",inlierMask1);
    cerr<<"H matrix has saved to homography_from_chessboard1.yml"<<endl;
    
    /*
    // decompose H to get R, T
    
    CV_Assert(K1.type()==CV_64F && H0_1.inv().type()==CV_64F);
    Mat A=K1.inv()*H0_1.inv();
    Vec3d a1(A.at<double>(0,0), A.at<double>(1,0), A.at<double>(2,0));
    Vec3d a2(A.at<double>(0,1), A.at<double>(1,1), A.at<double>(2,1));
    Vec3d a3(A.at<double>(0,2), A.at<double>(1,2), A.at<double>(2,2));

    double s=sqrt(norm(a1)*norm(a2));

    Vec3d r1=a1/s;
    Vec3d r2=a2/s;
    Vec3d r3=r1.cross(r2);
    Vec3d tvec=a3/s;

    Mat R0=(Mat_<double>(3,3) << r1[0],r2[0],r3[0],
		    		 r1[1],r2[1],r3[1],
				 r1[2],r2[2],r3[2]);

    SVD svd(R0);
    Mat R=svd.u*svd.vt;
    if(determinant(R)<0){
	R=-R;
	tvec=-tvec;
    }
    Mat T=(Mat_<double>(3,1) <<tvec[0], tvec[1], tvec[2]);
    FileStorage fs("homography_from_chessboard1.yml",FileStorage::APPEND);
    fs<<"R"<<R;
    fs<<"T"<<T;
    fs.release();
    cerr<<"Computing and Saving R & T ... done!"<<endl;
    */
    return true;
}

bool defineNewHmatrix_for_second_camera(){
    Mat img_bg=imread("background1.jpg");
    Mat img_ori;
    if(check){
	img_ori=imread("originOnCar1.jpg");
    }else{
	img_ori=imread("origin1.jpg");
    }
    if(img_bg.empty()||img_ori.empty()){
        cerr<<"Loading Image Failed."<<endl;
        return false;
    }
    FileStorage fs("paramsFromMatlab1.yml",FileStorage::READ);
    if(!fs.isOpened()){
        cerr<<"Cannot open paramsFromMatlab1.yml"<<endl;
        return false;
    }
    Mat K,dist;
    fs["K"]>>K;
    fs["dist"]>>dist;
    fs.release();
    FileStorage fs1("homography_from_chessboard1.yml",FileStorage::READ);
    if(!fs1.isOpened()){
        cerr<<"Cannot open homography_from_cheddboard1.yml"<<endl;
        return false;
    }
    Mat H_img2w, H_img2w_onCar;
    if(check){
	fs1["H_img2w_onCar"]>>H_img2w_onCar;
    }else{
	fs1["H_img2w"]>>H_img2w;
    }
    fs1.release();

    Rect box1;
    Mat und1;
    if(!computeBox(img_bg,img_ori,K,dist,box1,und1)){
        cerr<<"computeBox() failed"<<endl;
        return false;
    }
    if(check){
	imwrite("img1_position_1_onCar_undistort.jpg",und1);
    }else{
	imwrite("img1_position_1_undistort.jpg",und1);
    }
    Mat out1,mask1;
    int cx1=0,cy1=0;
    if(!extractReferencePoint(und1,box1,out1,mask1,cx1,cy1)){
        cerr<<"extractReferencePoint() failed"<<endl;
        return false;
    }
    cout<<"cx: "<<cx1<<endl;
    cout<<"cy: "<<cy1<<endl;
        //imwrite("check.jpg",out1);
    Mat H_star,H_star_inv;
    Mat H_star_onCar,H_star_onCar_inv;
    FileStorage fs2("H_matrix_new1.yml",FileStorage::APPEND);
    if(!fs2.isOpened()){
	cerr<<"Cannot open H_matrix_new1.yml"<<endl;
	return false;
    }
    if(check){
	if(!computeNewHmatrix((double)cx1,(double)cy1,H_img2w_onCar,H_star_onCar,H_star_onCar_inv)){
            cerr<<"computeNewHmatrix() failed"<<endl;
            return false;
        }
        fs2<<"H_onCar_new"<<H_star_onCar;
        fs2<<"H_onCar_inv_new"<<H_star_onCar_inv;
    }else{
     	if(!computeNewHmatrix((double)cx1,(double)cy1,H_img2w,H_star,H_star_inv)){
        	cerr<<"computeNewHmatrix() failed"<<endl;
        	return false;
	}
       	fs2<<"H_new"<<H_star;
       	fs2<<"H_inv_new"<<H_star_inv;
    }
    fs2.release();
    return true;
}

int TwoIndustryCameraInit(){
	CameraSdkInit(1);
	
	iStatus=CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);
	printf("state = %d\n",iStatus);
	printf("count = %d\n",iCameraCounts);
	if(iCameraCounts==0){
		printf("No deveice found\n");
		return -1;
	}
	//tSdkCameraDevInfo info,info1;
	//CameraGetEnumInfo(hCamera[0],&info);
	//printf("cam 0: name=%s, sn=%s\n",info.acFriendlyName,info.acSn);
	//CameraGetEnumInfo(hCamera[1],&info1);
	//printf("cam 1: name=%s, sn=%s\n",info1.acFriendlyName,info1.acSn);
	display_max=iCameraCounts;
	for(int i=0;i<display_max;i++){
		iStatus=CameraInit(&tCameraEnumList[i],-1,-1,&hCamera[i]);
		printf("i - %d, status = %d\n",i,iStatus);
		printf("state = %d\n",iStatus);
		if(iStatus!=CAMERA_STATUS_SUCCESS){
			printf("One of Camera initialization failed.\n");
			return -1;
		}
		CameraGetCapability(hCamera[i],&tCapability[i]);
		if(i==0){
			g_pRgbBuffer=(unsigned char*)malloc(tCapability[i].sResolutionRange.iHeightMax*tCapability[i].sResolutionRange.iWidthMax*3);
		}else{
			g_pRgbBuffer2 = (unsigned char*)malloc(tCapability[i].sResolutionRange.iHeightMax*tCapability[i].sResolutionRange.iWidthMax*3);
		}
		CameraPlay(hCamera[i]);
		if(tCapability[i].sIspCapacity.bMonoSensor){
			channel=1;
       		CameraSetIspOutFormat(hCamera[i],CAMERA_MEDIA_TYPE_MONO8);
		}else{
			channel=3;
       		CameraSetIspOutFormat(hCamera[i],CAMERA_MEDIA_TYPE_BGR8);
		}
	}
	tSdkCameraDevInfo info00,info11;
        CameraGetEnumInfo(hCamera[0],&info00);
        printf("cam 0: name=%s, sn=%s\n",info00.acFriendlyName,info00.acSn);
        CameraGetEnumInfo(hCamera[1],&info11);
        printf("cam 1: name=%s, sn=%s\n",info11.acFriendlyName,info11.acSn);
	
	const char* SN_IMG0 = "029003510201";
	tSdkCameraDevInfo info0,info1;
	CameraGetEnumInfo(hCamera[0],&info0);
	CameraGetEnumInfo(hCamera[1],&info1);
	if(strcmp(info0.acSn,SN_IMG0) != 0 && strcmp(info1.acSn,SN_IMG0) ==0){
		swap(hCamera[0],hCamera[1]);
		swap(tCapability[0],tCapability[1]);
	}

	return 0;
}

void chessboard_mode(){
	printf("Saving Chessboard Image...\n");
	while(iDisplayFrames--){
		if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)){
				CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
				Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
				imshow("0 : chessboard",matImage);
				if(iDisplayFrames==25){
					imwrite("chessboard0.jpg",matImage);
					imwrite("chessboard.jpg",matImage);
					printf("Done!\n");
				}
				waitKey(1);
				CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
		}
	}
	printf("Computing Homography Matrix...\n");
	if(!computeHmatrix2(check)){			
		cerr<<"Failed..."<<endl;
		return;
	}
	cerr<<"Done."<<endl;
}

void second_camera_chessboard_mode(){
        printf("Saving Chessboard Image...\n");
        while(iDisplayFrames--){
                if((CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
                                CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
                                Mat matImage(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
                                imshow("0 : chessboard",matImage);
                                if(iDisplayFrames==25){
                                        imwrite("chessboard1.jpg",matImage);
                                        printf("Done!\n");
                                }
                                waitKey(1);
                                CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
                }
        }
        printf("Computing Homography Matrix...\n");
        if(!computeHmatrix_for_second_camera()){
                cerr<<"Failed..."<<endl;
                return;
        }
        cerr<<"Done."<<endl;
}
void second_camera_chessboardOnCar_mode(){
	printf("Saving Chessboard Image...\n");
        while(iDisplayFrames--){
                if((CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
                                CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
                                Mat matImage(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
                                imshow("0 : chessboard",matImage);
                                if(iDisplayFrames==25){
                                        imwrite("chessboardOnCar1.jpg",matImage);
                                        printf("Done!\n");
                                }
                                waitKey(1);
                                CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
                }
        }
        printf("Computing Homography Matrix...\n");
        if(!computeHmatrix_for_second_camera()){
                cerr<<"Failed..."<<endl;
                return;
        }
        cerr<<"Done."<<endl;
}
void chessboardOnCar_mode(){
	printf("Saving ChessboardOnCar Image...\n");
	while(iDisplayFrames--){
		if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)){
				CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
				Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
				imshow("0 : chessboardOnCar",matImage);
				if(iDisplayFrames==25){
					imwrite("chessboardOnCar0.jpg",matImage);
					printf("Done!\n");
				}
				waitKey(1);
				CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
		}
	}
	printf("Computing Homography Matrix (on car)...\n");
	if(!computeHmatrix2(check)){			
		cerr<<"Failed..."<<endl;
		return;
	}
	cerr<<"Done."<<endl;
}

void background_mode(){
	printf("Saving background Image...\n");
	while(iDisplayFrames--){
		if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS) && (CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
				CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
				Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
				imshow("0 : background",matImage);
				CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
				Mat matImage2(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
				imshow("1 : background",matImage2);
				if(iDisplayFrames==25){
					imwrite("background0.jpg",matImage);
					imwrite("background1.jpg",matImage2);
					printf("Done!\n");
				}
				waitKey(1);
				CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
				CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
		}
	}
}

void origin_mode(){
	printf("Saving origin Image...\n");
	while(iDisplayFrames--){
		if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)){
				CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
				Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
				imshow("0 : origin",matImage);
				if(iDisplayFrames==25){
					imwrite("origin0.jpg",matImage);
					printf("Done!\n");
				}
				waitKey(1);
				CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
		}
	}
	defineNewHmatrix2();
}
void second_camera_origin_mode(){
	printf("Saving origin Image...\n");
        while(iDisplayFrames--){
                if((CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
                                CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
                                Mat matImage(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
                                imshow("0 : origin",matImage);
                                if(iDisplayFrames==25){
                                        imwrite("origin1.jpg",matImage);
                                        printf("Done!\n");
                                }
                                waitKey(1);
                                CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
                }
        }
        if(!defineNewHmatrix_for_second_camera()){
		cerr<<"defRefPtStreamV3.cpp: second_camera_origin_mode(): defineNewHmatrix_for_second_camera(): Error"<<endl;
		return;
	}
}

void originOnCar_mode(){
	printf("Saving originOnCar Image...\n");
	while(iDisplayFrames--){
		if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)){
				CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
				Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
				imshow("0 : originOnCar",matImage);
				if(iDisplayFrames==25){
					imwrite("originOnCar0.jpg",matImage);
					printf("Done!\n");
				}
				waitKey(1);
				CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
		}
	}
	defineNewHmatrix2();
}

void second_camera_originOnCar_mode(){
        printf("Saving origin Image...\n");
        while(iDisplayFrames--){
                if((CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
                                CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
                                Mat matImage(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
                                imshow("0 : origin",matImage);
                                if(iDisplayFrames==25){
                                        imwrite("originOnCar1.jpg",matImage);
                                        printf("Done!\n");
                                }
                                waitKey(1);
                                CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
                }
        }
        if(!defineNewHmatrix_for_second_camera()){
                cerr<<"defRefPtStreamV3.cpp: second_camera_origin_mode(): defineNewHmatrix_for_second_camera(): Error"<<endl;
                return;
        }
}


void snapshotPath_mode(){
	printf("Saving Path Image...\n");
	while(iDisplayFrames--){
		if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS) && (CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
				CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
				Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
				imshow("0 : path",matImage);
				CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
				Mat matImage2(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
				imshow("1 : path",matImage2);
				if(iDisplayFrames==25){
					imwrite("path0.jpg",matImage);
					imwrite("path1.jpg",matImage2);
					printf("Done!\n");
				}
				waitKey(1);
				CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
				CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
		}
	}
	/*defineNewHmatrix2();*/ // need to modify the defineNewHmatrix()
}

void findReferencePath_(int seg,const Mat& K0,const Mat& dist0,const Mat& K1,const Mat& dist1){
		//snapshotPath_mode();
		Mat img_bg0=imread("background0.jpg");
		Mat img_bg1=imread("background1.jpg");
		Mat img_path0=imread("path0.jpg");
		Mat img_path1=imread("path1.jpg");
		Mat und_img_bg0,und_img_bg1,und_img_path0,und_img_path1;
		und_img_bg0=undistortImage(img_bg0,K0,dist0);
		und_img_path0=undistortImage(img_path0,K0,dist0);
		und_img_bg1=undistortImage(img_bg1,K1,dist1);
		und_img_path1=undistortImage(img_path1,K1,dist1);
		cerr<<"findReferencePath_(): undistorting image done."<<endl;
		Mat bg0_lab,bg1_lab,path0_lab,path1_lab,diff0,th0,mask0,diss1,th1,mask1;
		cvtColor(und_img_bg0,bg0_lab,COLOR_BGR2Lab);
		cvtColor(und_img_bg1,bg1_lab,COLOR_BGR2Lab);
		cvtColor(und_img_path0,path0_lab,COLOR_BGR2Lab);
		cvtColor(und_img_path1,path1_lab,COLOR_BGR2Lab);
		cerr<<"findReferencePath_(): converting color done."<<endl;
		vector<Mat> B_0(3),B_1(3),P_0(3),P_1(3);
		split(bg0_lab,B_0);
		split(path0_lab,P_0);
		split(bg1_lab,B_1);
		split(path1_lab,P_1);
		cerr<<"findReferencePath_(): spliting done."<<endl;
		Mat da0, da1, db0, db1, chromaDiff0, chromaDiff1;
		Mat a1f_0,a1f_1,b1f_0,b1f_1,a2f_0,a2f_1,b2f_0,b2f_1;
		B_0[1].convertTo(a1f_0,CV_32F);
		B_0[2].convertTo(b1f_0,CV_32F);
		P_0[1].convertTo(a2f_0,CV_32F);
		P_0[2].convertTo(b2f_0,CV_32F);
		B_1[1].convertTo(a1f_1,CV_32F);
		B_1[2].convertTo(b1f_1,CV_32F);
		P_1[1].convertTo(a2f_1,CV_32F);
		P_1[2].convertTo(b2f_1,CV_32F);
		cerr<<"findReferencePath_(): converting to CV_32F done."<<endl;
		absdiff(a2f_0,a1f_0,da0);
		absdiff(b2f_0,b1f_0,db0);
		absdiff(a2f_1,a1f_1,da1);
		absdiff(b2f_1,b1f_1,db1);
		cerr<<"findReferencePath_(): differential of image done."<<endl;
		magnitude(da0,db0,chromaDiff0);
		magnitude(da1,db1,chromaDiff1);
		cerr<<"findReferencePath_(): getting magnitude of differential image done."<<endl;
		chromaDiff0.convertTo(chromaDiff0,CV_8U);
		chromaDiff1.convertTo(chromaDiff1,CV_8U);
		cerr<<"findReferencePath_(): converting to CV_8U done."<<endl;
		GaussianBlur(chromaDiff0,chromaDiff0,{5,5},0);
		GaussianBlur(chromaDiff1,chromaDiff1,{5,5},0);
		cerr<<"findReferencePath-(): Gaussianblur done."<<endl;
		threshold(chromaDiff0,th0,0,255,THRESH_BINARY | THRESH_OTSU);
		threshold(chromaDiff1,th1,0,255,THRESH_BINARY | THRESH_OTSU);
		cerr<<"findReferencePath_(): thresholding done."<<endl;
		Mat k=getStructuringElement(MORPH_ELLIPSE,{9,9});
		morphologyEx(th0,th0,MORPH_CLOSE,k,{-1,-1},2);
		morphologyEx(th0,th0,MORPH_OPEN,k,{-1,-1},1);
		morphologyEx(th1,th1,MORPH_CLOSE,k,{-1,-1},2);
		morphologyEx(th1,th1,MORPH_OPEN,k,{-1,-1},1);
		cerr<<"findReferencePath_(): morphologuEx() done."<<endl;
		// imshow("th1",th1);
		// imshow("th0",th0);
		/* obtain the contour by area-filter */
		vector<vector<Point>> contours0,contours1;
		findContours(th0,contours0,RETR_EXTERNAL,CHAIN_APPROX_NONE);
		findContours(th1,contours1,RETR_EXTERNAL,CHAIN_APPROX_NONE);
		double minArea=1000.0;
		contours0.erase(remove_if(contours0.begin(),contours0.end(),[&](const auto& c){return fabs(contourArea(c))<minArea; }),contours0.end());
		contours1.erase(remove_if(contours1.begin(),contours1.end(),[&](const auto& c){return fabs(contourArea(c))<minArea; }),contours1.end());
		sort(contours0.begin(),contours0.end(),[](const auto& a,const auto& b){return contourArea(a)>contourArea(b);});
		sort(contours1.begin(),contours1.end(),[](const auto& a,const auto& b){return contourArea(a)>contourArea(b);});
		if(contours0.size()>2) contours0.resize(2);
		if(contours1.size()>2) contours1.resize(2);
		drawContours(und_img_path0,contours0,-1,Scalar(0,255,255),2,LINE_AA);
		drawContours(und_img_path1,contours1,-1,Scalar(0,255,255),2,LINE_AA);
		imwrite("contours0.jpg",und_img_path0);
		imwrite("contours1.jpg",und_img_path1);
		cerr<<"findReferencePath_(): saving contours.jpg done."<<endl;
		sort(contours0.begin(),contours0.end(),[](const auto& a,const auto& b){
                        return boundingRect(a).x<boundingRect(b).x;
                        });
		sort(contours1.begin(),contours1.end(),[](const auto& a,const auto& b){
                        return boundingRect(a).x<boundingRect(b).x;
                        });
		// obtain the mask from contours; put all the mask into masks
		vector<Mat> masks0,masks1;
		masks0.reserve(contours0.size());
		masks1.reserve(contours1.size());
		for(size_t i=0;i<contours0.size();i++){
			Mat mask=Mat::zeros(und_img_path0.size(),CV_8U);
			drawContours(mask,contours0,(int)i,Scalar(255),FILLED,LINE_8);
			masks0.push_back(mask);
		}
		for(size_t i=0;i<contours1.size();i++){
			Mat mask=Mat::zeros(und_img_path1.size(),CV_8U);
			drawContours(mask,contours1,(int)i,Scalar(255),FILLED,LINE_8);
			masks1.push_back(mask);
		}
		// devide img0 into "seg" section & get the center from each section
		vector<vector<Point2f>> centers0;
		centers0.reserve(contours0.size());
		for(size_t i=0;i<contours0.size();i++){
			auto c=sliceMeans(masks0[i],contours0[i],seg);
			for(size_t k=1;k<c.size();k++){
				line(und_img_path0,c[k-1],c[k],Scalar(0,255,0),3,LINE_AA);
			}
			centers0.emplace_back(move(c));
		}
		for(size_t i=0;i<centers0.size();i++){
			sort(centers0[i].begin(),centers0[i].end(),[](const Point2f& a,const Point2f& b){
				if(a.y!=b.y) return a.y>b.y;
				return a.x<b.x;
			});
		}
		vector<Point> midRefPts0;
		midRefPts0.reserve(centers0[0].size());
		for(size_t i=0;i<centers0[0].size();i++){
			midRefPts0.emplace_back((centers0[0][i].x+centers0[1][i].x)/2, (centers0[0][i].y+centers0[1][i].y)/2);
			circle(und_img_path0,midRefPts0.back(),5,{0,255,0},FILLED,LINE_AA);
		}
		// devide img1 into "seg+1" section & get the center from each section
		vector<vector<Point2f>> centers1;
		centers1.reserve(contours1.size());
		for(size_t i=0;i<contours1.size();i++){
			auto c=sliceMeans(masks1[i],contours1[i],seg+1);
			for(size_t k=1;k<c.size();k++){
				line(und_img_path1,c[k-1],c[k],Scalar(0,255,0),3,LINE_AA);
			}
			centers1.emplace_back(move(c));
		}
		for(size_t i=0;i<centers1.size();i++){
			sort(centers1[i].begin(),centers1[i].end(),[](const Point2f& a,const Point2f& b){
				if(a.y!=b.y) return a.y>b.y;
				return a.x<b.x;
			});
		}
		vector<Point> midRefPts1;
		midRefPts1.reserve(centers1[0].size());
		for(size_t i=0;i<centers1[0].size();i++){
			midRefPts1.emplace_back((centers1[0][i].x+centers1[1][i].x)/2, (centers1[0][i].y+centers1[1][i].y)/2);
			circle(und_img_path1,midRefPts1.back(),5,{0,255,0},FILLED,LINE_AA);
		}
		// combin all center points
		vector<Point> allRefPts;
		allRefPts.reserve(midRefPts1.size());
		for(size_t i=0;i<midRefPts1.size();i++){
			if(i==(midRefPts1.size()-1)){
				allRefPts.emplace_back(midRefPts1[i].x,midRefPts1[i].y);
			}else{
				allRefPts.emplace_back(midRefPts0[i].x,midRefPts0[i].y);
			}
		}
		imwrite("reference_path_0.jpg",und_img_path0);
		imwrite("reference_path_1.jpg",und_img_path1);
		
		// new way to combine two midRefPts
		FileStorage FSH("H_matrix_new.yml",FileStorage::READ);
		FileStorage FSH1("H_matrix_new1.yml",FileStorage::READ);
		Mat HHH,HHH1;
		FSH["H_new"]>>HHH;
		FSH1["H_new"]>>HHH1;
		FSH.release();
		FSH1.release();
		vector<Point2f> rp;
		rp.reserve(midRefPts0.size()+midRefPts1.size());
		float last_rx,last_ry;
		for(size_t i=0;i<midRefPts0.size();i++){
			Point2d w=pix2world( (double)midRefPts0[i].x,(double)midRefPts0[i].y, HHH);
			float rx=static_cast<float>(round(w.x*100.0)/100.0);
			float ry=static_cast<float>(round(w.y*100.0)/100.0);
			rp.emplace_back(rx,ry);
			cerr<<rx<<", "<<ry<<endl;
			last_rx=rx;
			last_ry=ry;
		}
		for(size_t i=0;i<midRefPts1.size();i++){
			Point2d w=pix2world( (double)midRefPts1[i].x,(double)midRefPts1[i].y,HHH1);
			float rx=static_cast<float>(round(w.x*100.0)/100.0);
                        float ry=static_cast<float>(round(w.y*100.0)/100.0);
			if( (fabs(rx+1.2)-fabs(last_rx))>=0.1 && (fabs(ry)-fabs(last_ry))>=0.01 ){
				rp.emplace_back(rx+1.2,ry);
				cerr<<rx+1.2<<",,, "<<ry<<endl;
			}
		}
		/*
		// pix -> reference coordinate
		FileStorage FSH("H_matrix_new.yml",FileStorage::READ);
		if(!FSH.isOpened()){
			cerr<<"defRefPtStreamV2.cpp: findReferencePath_(): cannot open H_matrix_new.yml"<<endl;
			return ;
		}
		FileStorage FH("H_img0_to_img1.yml",FileStorage::READ);
		if(!FH.isOpened()){
			cerr<<"defRefPtStreamV3.cpp: findReferencePath_(): cannot open H_img1_to_img0.yml"<<endl;
			return;
		}
		Mat HHH,H_img1Toimg0;
		FSH["H_new"]>>HHH;
		FSH.release();
		FH["H_img0_to_img1"]>>H_img1Toimg0;
		FH.release();
		vector<Point2f> rp;
		rp.reserve(allRefPts.size());
		for(size_t i=0;i<(allRefPts.size()-1);i++){
			Point2d w=pix2world( (double)allRefPts[i].x, (double)allRefPts[i].y, HHH);
			float rx=static_cast<float>(round(w.x*100.0)/100.0);
			float ry=static_cast<float>(round(w.y*100.0)/100.0);
			rp.emplace_back(rx,ry);
			cerr<<rx<<", "<<ry<<endl;
		}
		// my way to get position on frame1
		FileStorage fs1("H_matrix_new1.yml",FileStorage::READ);
		if(!fs1.isOpened()){
			cerr<<"defRefPtStreamV3.cpp: findReferencePaht_(): cannot open H_matrix_new1.yml"<<endl;
			return;
		}
		Mat hi;
		fs1["H_new"]>>hi;
		fs1.release();
		Point2d ww=Point2d((double)allRefPts[allRefPts.size()-1].x,(double)allRefPts[allRefPts.size()-1].y);
		Point2d www=pix2world(ww.x,ww.y,hi);
		float rx=static_cast<float>(round(www.x*100.0)/100.0);
                float ry=static_cast<float>(round(www.y*100.0)/100.0);
                rp.emplace_back(rx+1.2,ry);
		cerr<<rx+1.2<<", "<<ry<<endl;
		*/
		// ************************** //
		//Point2d ww=Point2d((double)allRefPts[allRefPts.size()-1].x,(double)allRefPts[allRefPts.size()-1].y);
		//Point2d www=img1ToRef_exact(HHH,H_img1Toimg0.inv(),ww);
		//float rx=static_cast<float>(round(www.x*100.0)/100.0);
		//float ry=static_cast<float>(round(www.y*100.0)/100.0);
		//rp.emplace_back(rx,ry);
		//cerr<<rx<<", "<<ry<<endl;
		Mat M((int)rp.size(),1,CV_32FC2);
		for(int i=0;i<M.rows;i++){
			M.at<Vec2f>(i,0)=Vec2f(rp[i].x,rp[i].y);
		}
		FileStorage ff("ref_points.yml",FileStorage::WRITE);
		ff<<"refPoints"<<M;
		ff.release();
		
}

void appendingImage(const Mat& und_img0,const Mat& und_img1,const Mat& H){
        Mat img0=und_img0.clone();
        Mat img1=und_img1.clone();
        int w0=img0.cols , h0=img0.rows;
        int w1=img1.cols, h1=img1.rows;

        vector<Point2f> c0={ {0,0}, {(float)w0,0}, {(float)w0,(float)h0}, {0,(float)h0} };
        vector<Point2f> c1={ {0,0}, {(float)w1,0}, {(float)w1,(float)h1}, {0,(float)h1} };

        Mat Hinv=H.inv();
        vector<Point2f> c1_in_0;
        perspectiveTransform(c1,c1_in_0,Hinv);

        double minx=0, miny=0, maxx=w0, maxy=h0;
        auto upd=[&](const Point2f& p){
                minx=min<double>(minx,p.x);
                miny=min<double>(miny,p.y);
                maxx=max<double>(maxx,p.x);
                maxy=max<double>(maxy,p.y);
        };
        for(auto& p : c1_in_0) upd(p);

        int minX=(int)floor(minx);
        int minY=(int)floor(miny);
        int maxX=(int)floor(maxx);
        int maxY=(int)floor(maxy);

        extend_left=max(0,-minX);
        extend_top=max(0,-minY);
        extend_right=max(0,maxX-w0);
        extend_bottom=max(0,maxY-h0);
	//
        int newW=extend_left+w0+extend_right;
        int newH=extend_top+h0+extend_bottom;
        Mat T=(Mat_<double>(3,3)<<1,0,extend_left, 0,1,extend_top, 0,0,1);

        Mat pano(newH,newW,img0.type(),Scalar::all(0));
        warpPerspective(img0,pano,T,Size(newW,newH));

        Mat M=T*Hinv;
        Mat img1_on_canvas;
        warpPerspective(img1,img1_on_canvas,M,Size(newW,newH));

        Mat gray,mask;
        cvtColor(img1_on_canvas,gray,COLOR_BGR2GRAY);
        threshold(gray,mask,0,255,THRESH_BINARY);
        img1_on_canvas.copyTo(pano,mask);
        imwrite("pano_debug.jpg",pano);
        cerr<<"Saving pano_debug.jpg done."<<endl;
	//cerr<<"new W: "<<newW<<endl;
        //cerr<<"new H: "<<newH<<endl;
}
void matching(const Mat& K0,const Mat& dist0, const Mat& K1,const Mat& dist1){
        // aim to get the H_img0_to_img1 matrix
        Mat img0=imread("path0.jpg");
        Mat img1=imread("path1.jpg");
        if(img0.empty() || img1.empty()){
                cerr<<"defRefPtStreamV3.cpp: matching(): Loading image failed."<<endl;
                return ;
        }

        Mat und_img0,und_img1;
        Mat newK=getOptimalNewCameraMatrix(K0,dist0,Size((int)img0.cols,(int)img0.rows),0);
        undistort(img0,und_img0,K0,dist0,newK);
        Mat newK1=getOptimalNewCameraMatrix(K1,dist1,Size((int)img1.cols,(int)img1.rows),0);
        undistort(img1,und_img1,K1,dist1,newK1);
	//und_img0=img0.clone();
	//und_img1=img1.clone();
	Mat g0,g1;
	cvtColor(und_img0,g0,COLOR_BGR2GRAY);
	cvtColor(und_img1,g1,COLOR_BGR2GRAY);
	Ptr<CLAHE> clahe=createCLAHE(3.0,Size(8,8));
	clahe->apply(g0,g0);
	clahe->apply(g1,g1);

        //Ptr<ORB> orb=ORB::create(3000,1.2f,8,15,0,2,ORB::HARRIS_SCORE,31,20);
        Ptr<ORB> orb=ORB::create(8000,1.1f,12,31,0,4,ORB::HARRIS_SCORE,31,8);
	vector<KeyPoint> k0,k1;
        Mat d0,d1;
        orb->detectAndCompute(und_img0,noArray(),k0,d0);
        orb->detectAndCompute(und_img1,noArray(),k1,d1);

        BFMatcher matcher(NORM_HAMMING,false);
        vector<vector<DMatch>> knn;
        matcher.knnMatch(d0,d1,knn,2);

        vector<DMatch> good;
        good.reserve(knn.size());
        for(const auto& m:knn){
                if(m.size()==2 && m[0].distance<0.75f*m[1].distance){
                        good.push_back(m[0]);
                }
        }

        Mat inlierMask;
        vector<Point2f> src,dst;
        src.reserve(good.size());
        dst.reserve(good.size());
	for(const auto& m:good){
                src.push_back(k0[m.queryIdx].pt);
                dst.push_back(k1[m.trainIdx].pt);
        }

        vector<DMatch> inliers;
        if(src.size() >=8){
                H_img0_to_img1=findHomography(src,dst,RANSAC,5.0,inlierMask);
                if(!H_img0_to_img1.empty()){
                        for(size_t i=0;i<good.size();i++){
                                if(inlierMask.at<uchar>(static_cast<int>(i))){
                                        inliers.push_back(good[i]);
                                }
                        }
                }
        }
        if(H_img0_to_img1.empty()) inliers=good;
	FileStorage fsf("H_img0_to_img1.yml",FileStorage::WRITE);
	fsf<<"H_img0_to_img1"<<H_img0_to_img1;
	fsf.release();
        Mat vis;
        drawMatches(und_img0,k0,und_img1,k1,inliers,vis,Scalar::all(-1),Scalar::all(-1),vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        cerr<<"saving inlier_matches.jpg......"<<endl;
	imwrite("inlier_matches.jpg",vis);
	cerr<<"saving inlier_matches.jpg...Done"<<endl;
        appendingImage(und_img0,und_img1,H_img0_to_img1);
}
void findPath_mode(int seg){
	cerr<<"findPath mode."<<endl;
	Mat K0,dist0,K1,dist1;
        FileStorage fs("paramsFromMatlab0.yml",FileStorage::READ);
        if(!fs.isOpened()){
                cerr<<"Cannot open paramsFromMatlab0.yml"<<endl;
                return ;
        }
        fs["K"]>>K0;
        fs["dist"]>>dist0;
        fs.release();
        FileStorage fss("paramsFromMatlab1.yml",FileStorage::READ);
        if(!fss.isOpened()){
                cerr<<"Cannot open paramsFromMatlab1.yml"<<endl;
                return ;
        }
        fss["K"]>>K1;
        fss["dist"]>>dist1;
        fss.release();

        // matching()
        //matching(K0,dist0,K1,dist1);
	//cerr<<"matching(): ok!"<<endl;
        // find path
        //findReferencePath_(seg,K0,dist0,K1,dist1);
	findReferencePath_Multi(seg,K0,dist0,K1,dist1);
	cerr<<"findReferencePath_(): ok!"<<endl;
	waitKey(1);
}
void stream_mode(/*int seg*/int path_idx){
	cerr<<"Streaming mode. Waiting the requirement from Car..."<<endl;
	cerr<<"Starting..."<<endl;
	
	Mat K0,dist0,K1,dist1;
	FileStorage fs("paramsFromMatlab0.yml",FileStorage::READ);
	if(!fs.isOpened()){
		cerr<<"Cannot open paramsFromMatlab0.yml"<<endl;
		return ;
	}
	fs["K"]>>K0;
	fs["dist"]>>dist0;
	fs.release();
	FileStorage fss("paramsFromMatlab1.yml",FileStorage::READ);
	if(!fss.isOpened()){
		cerr<<"Cannot open paramsFromMatlab1.yml"<<endl;
		return ;
	}
	fss["K"]>>K1;
	fss["dist"]>>dist1;
	fss.release();
	/*
	// matching()
	matching(K0,dist0,K1,dist1);
	
	// find path
	findReferencePath_(seg,K0,dist0,K1,dist1);
	*/
	// load referenct points
	vector<Point2f> refPoints2;
	//if(!loadRefPoints(refPoints2)){
	//	cerr<<"defRefPtStreamV3.cpp: stream_mode(): unable to get the reference points."<<endl;
	//	return ;
	//}
	if(!loadMultiPaths(refPoints2,path_idx)){
		cerr<<"defRefPtStreamV3.cpp: stream_mode(): unable to get the multi paths."<<endl;
		return ;
	}
	pathLen=static_cast<int>(refPoints2.size());
	
	// serial transimition (usage: send_msg("myString");)
	int serial_fd=open_serial("/dev/ttyUSB0");
	if(serial_fd<0){
		cerr<<"defRefPtStreamV3.cpp: stream_mode(): unable to open serial_fd."<<endl;
		return;
	}
	auto send_msg=[&](const string& s){
		string line=s+"\n";
		::write(serial_fd,line.data(),line.size());
		::tcdrain(serial_fd);
	};
	
	//
	auto& pFirst=refPoints2[0];
	double pFirstX=(double)pFirst.x;
	double pFirstY=(double)pFirst.y;
	groundMsg=makeGroundMsg(pFirstX,pFirstY);
	string buf;
	Mat undImage0,undImage1;
	bool inFrame1=false;
	while(1){
		if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS) && (CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
			// get frame
			CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
			Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
			undImage0=undistortImage(matImage,K0,dist0);
			CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
			Mat matImage2(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
			undImage1=undistortImage(matImage2,K1,dist1);
			//imshow("image 0",undImage0);
			//imshow("image 1",undImage1);
			
			//
			ssize_t nread=::read(serial_fd,rxBuffer,sizeof(rxBuffer));
			Mat frame0_copy,frame1_copy;
			//frame0_copy=undImage0.clone();
			//frame1_copy=undImage1.clone();
			
			if(nread>0){
				buf.append(rxBuffer,rxBuffer+nread);
				size_t pos;
				while( (pos=buf.find('\n')) != string::npos ){
					string line=buf.substr(0,pos);
					buf.erase(0,pos+1);
					if(!line.empty() && line.back()=='\r') line.pop_back();
					if(line=="REQ"){
						if(getCarPosition(undImage0,0,K0,dist0)>0 && inFrame1==false){
							// car in frame0_copy
							cerr<<"REQ: Getting Car Position in Frame 0."<<endl;
							send_msg(groundMsg+carMsg);
						}else if(getCarPosition(undImage1,1,K1,dist1)>0){
							// car in frame1_copy only
							inFrame1=true;
							cerr<<"REQ: Getting Car Position in Frmae 1."<<endl;
							send_msg(groundMsg+carMsg);
						}
					}else if(line=="UPDATE"){
						if(getCarPosition(undImage0,0,K0,dist0)>0 && inFrame1==false){
							// car in frame0_copy
							cerr<<"UPDATE: Getting Car Position in Frame 0."<<endl;
						}else if(getCarPosition(undImage1,1,K1,dist1)>0){
							// car in frame1_copy only
							cerr<<"UPDATE: Getting Car Position in Frmae 1."<<endl;
						}
						auto& p=refPoints2[pathIdx];
                        double px=(double)p.x;
                        double py=(double)p.y;
                        groundMsg=makeGroundMsg(px,py);
                        pathIdx=pathIdx+1;
                        cerr<<"**********************"<<endl;
                        cerr<<"px:"<<px<<"py:"<<py<<endl;
                        cerr<<"**********************"<<endl;
                        send_msg(groundMsg+carMsg);
                        if(pathIdx==pathLen){
                            pathIdx=pathLen-1;
                            cerr<<"End of the Path."<<endl;
                        }
					}else{}
				}
			}
			int key=waitKey(1);
			CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
			CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
		}
	}
	::close(serial_fd);
}

void unInitCamera(){
	for(int i=0;i<display_max;i++){
		CameraUnInit(hCamera[i]);
		if(i==0){
			free(g_pRgbBuffer);
		}else{
			free(g_pRgbBuffer2);
		}
	}
}

int main(int argc,char* argv[]){
	if(argc==1){
		printf("ERROR executing command!\n");
		return -1;
	}
	
	// two camera initialization
	if(TwoIndustryCameraInit()==0){
		printf("TwoIndustryCameraInit() : successed.\n");
	}else{
		printf("TwoIndustryCameraInit() : failed.\n");
		return -1;
	}
	
	if(strcmp(argv[1],"chessboard")==0){
		check=false;
		chessboard_mode();
		
	}else if(strcmp(argv[1],"stream")==0){
		//if(argc!=3){
		//	cerr<<"Please define the segmentation."<<endl;
		//	return -1;
		//}
		//int seg=stoi(argv[2]);
		int path_idx=stoi(argv[2]);
		stream_mode(path_idx);
		
	}else if(strcmp(argv[1],"background")==0){
        	background_mode();
		
	}else if(strcmp(argv[1],"second_chessboard")==0){
		check=false;
		second_camera_chessboard_mode();

        }else if(strcmp(argv[1],"origin")==0){
    		check=false;
		origin_mode();
		
    	}else if(strcmp(argv[1],"second_origin")==0){
		check=false;
		second_camera_origin_mode();

	}else if(strcmp(argv[1],"chessboardOnCar")==0){
        	check=true;
		chessboardOnCar_mode();
		
	}else if(strcmp(argv[1],"second_chessboardOnCar")==0){
		check=true;
		second_camera_chessboardOnCar_mode();

	}else if(strcmp(argv[1],"second_originOnCar")==0){
		check=true;
		second_camera_originOnCar_mode();

	}else if(strcmp(argv[1],"originOnCar")==0){
        	check=true;
		originOnCar_mode();
		
    	}else if(strcmp(argv[1],"findPath")==0){
		if(argc<3){
			cerr<<"Please define the segmentation."<<endl;
			return 0;
		}
		int s=stoi(argv[2]);
		snapshotPath_mode();
		findPath_mode(s);
	}else if(strcmp(argv[1],"findPathTest")==0){
		Mat K0,dist0,K1,dist1;
        	FileStorage fs("paramsFromMatlab0.yml",FileStorage::READ);
        	if(!fs.isOpened()){
                	cerr<<"Cannot open paramsFromMatlab0.yml"<<endl;
                	return 0;
        	}
        	fs["K"]>>K0;
        	fs["dist"]>>dist0;
        	fs.release();
        	FileStorage fss("paramsFromMatlab1.yml",FileStorage::READ);
        	if(!fss.isOpened()){
                	cerr<<"Cannot open paramsFromMatlab1.yml"<<endl;
                	return 0;
        	}
        	fss["K"]>>K1;
        	fss["dist"]>>dist1;
        	fss.release();

        	// matching()
        	//matching(K0,dist0,K1,dist1);
        	//cerr<<"matching(): ok!"<<endl;
        	// find path
        	findReferencePath_(10,K0,dist0,K1,dist1);
        	cerr<<"findReferencePath_(): ok!"<<endl;
	}else if(strcmp(argv[1],"errorTest")==0){
		Mat K0,K1,dist0,dist1;
		FileStorage fs0("paramsFromMatlab0.yml",FileStorage::READ);
		FileStorage fs1("paramsFromMatlab1.yml",FileStorage::READ);
		if(!fs0.isOpened()){
			cerr<<"cannot open paramsFromMatlab0.yml"<<endl;
		}
		if(!fs1.isOpened()){
			cerr<<"cannot open paramsFromMatlab1.yml"<<endl;
		}
		fs0["K"]>>K0;
		fs0["dist"]>>dist0;
		fs1["K"]>>K1;
		fs1["dist"]>>dist1;
		fs0.release();
		fs1.release();
		Mat undImage0,undImage1;
		while(iDisplayFrames--){
                if((CameraGetImageBuffer(hCamera[0],&sFrameInfo[0],&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS) && (CameraGetImageBuffer(hCamera[1],&sFrameInfo[1],&pbyBuffer2,1000) == CAMERA_STATUS_SUCCESS)){
                        // get frame
                        CameraImageProcess(hCamera[0],pbyBuffer,g_pRgbBuffer,&sFrameInfo[0]);
                        Mat matImage(Size(sFrameInfo[0].iWidth,sFrameInfo[0].iHeight), sFrameInfo[0].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer);
                        undImage0=undistortImage(matImage,K0,dist0);
                        CameraImageProcess(hCamera[1],pbyBuffer2,g_pRgbBuffer2,&sFrameInfo[1]);
                        Mat matImage2(Size(sFrameInfo[1].iWidth,sFrameInfo[1].iHeight), sFrameInfo[1].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,g_pRgbBuffer2);
                        undImage1=undistortImage(matImage2,K1,dist1);
                        //imshow("image 0",undImage0);
                        imshow("image 1",undImage1);
			//Mat frame0_copy=undImage0.clone();
			Mat frame1_copy=undImage1.clone();
			if(iDisplayFrames==25){
				if(getCarPosition(frame1_copy,1,K1,dist1)>0){
					cerr<<"Getting Car Position in Frame 1"<<endl;
				}
				
			}
			waitKey(1);
			CameraReleaseImageBuffer(hCamera[0],pbyBuffer);
			CameraReleaseImageBuffer(hCamera[1],pbyBuffer2);
		}
		}
	}
	unInitCamera();
	return 0;
}

