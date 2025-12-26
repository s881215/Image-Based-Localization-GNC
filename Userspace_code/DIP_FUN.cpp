#include "DIP_FUN.h"

vector<Point> midPoints;

Mat undistortImage(const Mat& img,const Mat& K,const Mat& dist){
	Mat newK=getOptimalNewCameraMatrix(K,dist,img.size(),0);
	Mat und;
	undistort(img,und,K,dist,newK);
	//imshow("und",und);
	//waitKey(0);
	return und;
}

bool computeBox(const Mat& img_bg,const Mat& img_pos,const Mat& K,const Mat& dist,Rect& box,Mat& pos1_undist){
	Mat background = undistortImage(img_bg,K,dist);
	pos1_undist = undistortImage(img_pos,K,dist);
	
	Mat bg_gray,pos_gray,diff,th,mask;
	cvtColor(background,bg_gray,COLOR_BGR2GRAY);
	cvtColor(pos1_undist,pos_gray,COLOR_BGR2GRAY);
	
	// normalization brightness
	bg_gray.convertTo(bg_gray,CV_32F);
	pos_gray.convertTo(pos_gray,CV_32F);

	const float eps=1.0f;
	Mat num, den, D;
	absdiff(bg_gray,pos_gray,num);
	add(bg_gray,pos_gray,den);
	den += eps;
	divide(num, den, D);

	double minv, maxv;
	minMaxLoc(D, &minv, &maxv);
	if (maxv<=0) maxv=1.0;
	D.convertTo(diff, CV_8U,255.0/maxv);


	//absdiff(bg_gray,pos_gray,diff);
	
	GaussianBlur(diff,diff,Size(5,5),0);
	threshold(diff,th,0,255,THRESH_BINARY | THRESH_OTSU);
	//imshow("th",th);
	//waitKey(0);
	Mat k = getStructuringElement(MORPH_RECT,Size(5,5));
	morphologyEx(th,mask,MORPH_OPEN,k,Point(-1,-1),1);
	dilate(mask,mask,k,Point(-1,-1),2);

	vector<vector<Point>> contours;
	findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

	const double MIN_AREA = 100.0;
	vector<int> x1s,y1s,x2s,y2s;
	for(auto& c:contours){
		if (contourArea(c) < MIN_AREA) continue;
		Rect r = boundingRect(c);
		x1s.push_back(r.x);
		y1s.push_back(r.y);
		x2s.push_back(r.x + r.width);
		y2s.push_back(r.y + r.height);
	}	
	if (x1s.empty()) return false;
	int x1 = *min_element(x1s.begin(),x1s.end());
	int y1 = *min_element(y1s.begin(),y1s.end());
	int x2 = *max_element(x2s.begin(),x2s.end());
	int y2 = *max_element(y2s.begin(),y2s.end());
	box = Rect(Point(x1,y1), Point(x2,y2));
	return true;
}

bool extractReferencePoint(const Mat& frame, const Rect& box,Mat& out, Mat& mask_full, int& cx_out, int& cy_out){
	if (box.area() <= 0) return false;
	out = frame.clone();

	Rect valid = box & Rect(0,0,frame.cols,frame.rows);
	if(valid.area() <= 0) return false;

	Mat roi = frame(valid).clone();
	Mat roi_hsv;
	cvtColor(roi,roi_hsv,COLOR_BGR2HSV);

	//Scalar lower_red1(0,120,120), upper_red1(10,255,255);
	//Scalar lower_red2(170,120,120), upper_red2(179,255,255);
	Scalar lower_red1(0,80,30), upper_red1(10,255,255);
	Scalar lower_red2(170,80,30), upper_red2(179,255,255);
	Mat m1,m2,mask;
	inRange(roi_hsv,lower_red1,upper_red1,m1);
	inRange(roi_hsv,lower_red2,upper_red2,m2);
	//imshow("m1",m1);
	//imshow("m2",m2);
	//waitKey(0);
	bitwise_or(m1,m2,mask);

	Mat k = getStructuringElement(MORPH_RECT,Size(5,5));
	morphologyEx(mask,mask,MORPH_OPEN,k,Point(-1,-1),1);
	morphologyEx(mask,mask,MORPH_CLOSE,k,Point(-1,-1),1);

	mask_full = Mat::zeros(frame.size(),CV_8UC1);
	mask.copyTo(mask_full(valid));

	vector<vector<Point>> contours;
	findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	if(contours.empty()) return false;

	size_t imax=0;
	double amax=0.0;
	for(size_t i=0;i<contours.size();++i){
		double a = contourArea(contours[i]);
		if(a>amax){
			amax=a;
			imax=i;
		}
	}

	Moments m = moments(contours[imax]);
	if(m.m00<=1e-9) return false;
	int cx = (int)(m.m10/m.m00);
	int cy = (int)(m.m01/m.m00);

	Rect r = boundingRect(contours[imax]);
	rectangle(out,Point(valid.x + r.x, valid.y+r.y),Point(valid.x + r.x + r.width,valid.y + r.y + r.height),Scalar(0,255,0),2);
	circle(out,Point(valid.x + cx, valid.y+cy),2,Scalar(0,255,0),-1);

	cx_out=valid.x+cx;
	cy_out=valid.y+cy;
	return true;
}

bool computeNewHmatrix(double u0,double v0,const Mat& H_img2w,Mat& H_star,Mat& H_star_inv){
	/*
	FileStorage fs(H_path,FileStorage:READ);
	if(!fs.isOpened()) {
		cerr<<"cannot open"<<H_path<<endl;
		return false;
	}
	Mat H0;
	fs["H_img2w"]>>H0;
	fs.release();
	*/
	Mat H0=H_img2w;
	cerr<<"H0:\n"<<H0<<endl;
	if(H0.empty() || H0.rows!=3 || H0.cols!=3){
		cerr<<"cannot read H_img2w (3x3)"<<endl;
		return false;
	}
	Mat F = (Mat_<double>(3,3) << 1,0,0, 0,-1,0, 0,0,1);
	Mat H1 = F * H0;
	cerr<<"H1:\n"<<H1<<endl;
	Mat p0 = (Mat_<double>(3,1) << u0, v0, 1.0);
	Mat q0 = H1 * p0;
	double w = q0.at<double>(2,0);
	if (fabs(w) < 1e-12) return false;
	q0 /= w;
	double x0 = q0.at<double>(0,0), y0 = q0.at<double>(1,0);
	cerr << fixed << setprecision(4) << "Car init in reference-frame (meters): x0="<<x0<<", y0="<<y0<<endl;

	Mat T = (Mat_<double>(3,3) << 1,0,-x0, 0,1,-y0, 0,0,1);
	Mat R = Mat::eye(3,3,CV_64F);
	H_star = R*T*F*H0;
	H_star_inv = H_star.inv();
	cerr<<"H* (img 2 world; origin at car init):\n"<<H_star<<endl;
	return true;
}

bool computeNewHmatrix_v2(double u0,double v0,const Mat& H_img2w,Mat& H_star,Mat& H_star_inv){
	FileStorage fsd("paramsFromMatlab.yml",FileStorage::READ);
	if(!fsd.isOpened()){
		cerr<<"cnnot open paramsFromMatlab.yml"<<endl;
		return false;
	}
	Mat K,dist;
	fsd["K"]>>K;
	fsd["dist"]>>dist;
	fsd.release();

	CV_Assert(K.rows==3 && K.cols==3 && H_img2w.rows==3 && H_img2w.cols==3);
	Mat H=H_img2w.inv();
	Mat K_inv=K.inv();
	Mat B=K_inv*H;

	Mat b1=B.col(0);
	Mat b2=B.col(1);
	Mat b3=B.col(2);

	double s1=1.0/norm(b1);
	double s2=1.0/norm(b2);
	double s=(s1+s2)*0.5;

	Mat r1=s*b1;
	Mat r2=s*b2;
	Mat r3=r1.cross(r2);

	Mat R_approx(3,3,CV_64F);
	r1.copyTo(R_approx.col(0));
	r2.copyTo(R_approx.col(1));
	r3.copyTo(R_approx.col(2));

	SVD svd(R_approx);
	Mat R=svd.u*svd.vt;
	if(determinant(R)<0){
		R.col(2) *= -1;
	}
	Mat t=s*b3;

	Mat uv1=(Mat_<double>(3,1) <<u0, v0, 1.0);

	Mat d_cam=K_inv*uv1;
	Mat d_w=R.t()*d_cam;
	Mat C_w=-R.t()*t;

	double lam=-C_w.at<double>(2,0) / d_w.at<double>(2,0);
	Mat Xg=C_w + lam*d_w;
	double x0=Xg.at<double>(0,0);
	double y0=Xg.at<double>(1,0);

	Mat T=(Mat_<double>(3,3)<< 1,0,-x0, 0,-1,-y0, 0,0,1);
	H_star=T * H_img2w;
	H_star_inv=H_star.inv();
	FileStorage fs2("H_matrix_new.yml",FileStorage::WRITE);
        fs2<<"H_new"<<H_star;
        fs2<<"H_inv_new"<<H_star_inv;
        fs2.release();
	cerr<<fixed<<setprecision(6)
	    <<"New H matrix version 2. :\n"<<H_star<<endl;
	return true;
}

Point2d img2Ground(double u0,double v0,const Mat& H_img2w){
	FileStorage fs("paramsFromMatlab.yml",FileStorage::READ);
	if(!fs.isOpened()){
		cerr<<"Cannot open paramsFromMatlab.yml"<<endl;
		return {0.0,0.0};
	}
	Mat K,dist;
	fs["K"]>>K;
	fs["dist"]>>dist;
	fs.release();
	
	CV_Assert(K.rows==3 && K.cols==3 && H_img2w.rows==3 && H_img2w.cols==3);
        Mat H=H_img2w.inv();
        Mat K_inv=K.inv();
        Mat B=K_inv*H;

        Mat b1=B.col(0);
        Mat b2=B.col(1);
        Mat b3=B.col(2);

        double s1=1.0/norm(b1);
        double s2=1.0/norm(b2);
        double s=(s1+s2)*0.5;

        Mat r1=s*b1;
        Mat r2=s*b2;
        Mat r3=r1.cross(r2);

        Mat R_approx(3,3,CV_64F);
        r1.copyTo(R_approx.col(0));
        r2.copyTo(R_approx.col(1));
        r3.copyTo(R_approx.col(2));

        SVD svd(R_approx);
        Mat R=svd.u*svd.vt;
        if(determinant(R)<0){
                R.col(2) *= -1;
        }
        Mat t=s*b3;

        Mat uv1=(Mat_<double>(3,1) <<u0, v0, 1.0);

        Mat d_cam=K_inv*uv1;
        Mat d_w=R.t()*d_cam;
        Mat C_w=-R.t()*t;

        double lam=-C_w.at<double>(2,0) / d_w.at<double>(2.0);
        Mat Xg=C_w + lam*d_w;
        double x0=Xg.at<double>(0,0);
        double y0=Xg.at<double>(1,0);
	//Mat T=(Mat_<double>(3,3)<< 1,0,-x0 ,0,-1,-y0, 0,0,1);
	//Mat H_=T*H_img2w;
	//Point2d w=pix2world(u0,v0,H_);
	FileStorage fsd("homography_from_chessboard.yml",FileStorage::READ);
	if(!fsd.isOpened()){
		cerr<<"img2graound():cannot open homography_from_chessboard.yml"<<endl;
		return {-1,-1};
	}
	Mat H_original;
	fsd["H_original"]>>H_original;
	fsd.release();
	Mat X=(Mat_<double>(3,1)<<x0,y0,1.0);
	Mat H_original_inv=H_original.inv();
	Mat q=H_original_inv * X;
	double w=q.at<double>(2);
	return {q.at<double>(0)/w, q.at<double>(1)/w};
}

bool computeHmatrix(){
	const Size kBoardSize=Size(8,5);
	const double kSquareSize=0.03;
	Mat img=imread("chessboard.jpg",IMREAD_COLOR);
	if(img.empty()){
		cerr<<"chessboard image does not exit."<<endl;
		return false;
	}
	int w=img.cols;
	int h=img.rows;
	Mat und=img.clone();
	FileStorage fs("paramsFromMatlab.yml",FileStorage::READ);
	if(!fs.isOpened()){
		cerr<<"can not open paramsNew.yml."<<endl;
		return false;
	}
	Mat K,dist;
	fs["K"]>>K;
	fs["dist"]>>dist;
	fs.release();
	if(K.empty() || dist.empty()){
		cerr<<"lack of parameters K or dist."<<endl;
		return false;
	}
	Mat newK=getOptimalNewCameraMatrix(K,dist,Size(w,h),0);
	undistort(img,und,K,dist,newK);
	
	Mat gray;
	cvtColor(und,gray,COLOR_BGR2GRAY);
	vector<Point2f> corners;
	bool ok=findChessboardCorners(gray,kBoardSize,corners,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
#if (CV_VERSION_MAJOR >=4)
	if(!ok){
		ok = findChessboardCornersSB(gray,kBoardSize,corners);
	}
#endif
	if(!ok){
		cerr<<"cannot find the chessboard corners"<<endl;
		return false;
	}
	cornerSubPix(gray,corners,Size(11,11),Size(-1,-1),TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER,40,1e-3));

	const int cols = kBoardSize.width;
	const int rows = kBoardSize.height;

	vector<Point2f> world_pts;
	world_pts.reserve(cols*rows);
	for(int r=0;r<rows;++r){
		for(int c=0;c<cols;++c){
			world_pts.emplace_back(static_cast<float>(c*kSquareSize),static_cast<float>(r*kSquareSize));
		}
	}

	Mat inlierMask;
	Mat H0=findHomography(corners,world_pts,RANSAC,2.0,inlierMask);
	if(H0.empty()){
		cerr<<"findHomography() failed"<<endl;
		return false;
	}
	cerr<<"H (img -> world) :\n"<<H0<<endl;
	{
		FileStorage fs("homography_from_chessboard.yml",FileStorage::WRITE);
		fs<<"H_img2w"<<H0;
		fs<<"H_original"<<H0;
		fs.release();
		cerr<<"H matrix has saved to homography_from_chessboard.yml"<<endl;
	}
	return true;
}

bool computeHmatrix2(bool check){
	const Size kBoardSize=Size(8,5);
        const double kSquareSize=0.03;
	Mat img;
        if(check){ // check==true : on car
		img=imread("chessboardOnCar.jpg",IMREAD_COLOR);
	}else{	   // check==false : on ground
		img=imread("chessboard.jpg",IMREAD_COLOR);
	}
	if(img.empty()){
                cerr<<"chessboard image does not exit."<<endl;
                return false;
        }
        int w=img.cols;
        int h=img.rows;
        Mat und=img.clone();
        FileStorage fs("paramsFromMatlab.yml",FileStorage::READ);
        if(!fs.isOpened()){
                cerr<<"can not open paramsNew.yml."<<endl;
                return false;
        }
        Mat K,dist;
        fs["K"]>>K;
        fs["dist"]>>dist;
        fs.release();
        if(K.empty() || dist.empty()){
                cerr<<"lack of parameters K or dist."<<endl;
                return false;
        }
        Mat newK=getOptimalNewCameraMatrix(K,dist,Size(w,h),0);
        undistort(img,und,K,dist,newK);

        Mat gray;
        cvtColor(und,gray,COLOR_BGR2GRAY);
        vector<Point2f> corners;
        bool ok=findChessboardCorners(gray,kBoardSize,corners,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
#if (CV_VERSION_MAJOR >=4)
        if(!ok){
                ok = findChessboardCornersSB(gray,kBoardSize,corners);
        }
#endif
	if(!ok){
                cerr<<"cannot find the chessboard corners"<<endl;
                return false;
        }
        cornerSubPix(gray,corners,Size(11,11),Size(-1,-1),TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER,40,1e-3));

        const int cols = kBoardSize.width;
        const int rows = kBoardSize.height;

        vector<Point2f> world_pts;
        world_pts.reserve(cols*rows);
        for(int r=0;r<rows;++r){
                for(int c=0;c<cols;++c){
                        world_pts.emplace_back(static_cast<float>(c*kSquareSize),static_cast<float>(r*kSquareSize));
                }
        }

        Mat inlierMask;
        Mat H0=findHomography(corners,world_pts,RANSAC,2.0,inlierMask);
        if(H0.empty()){
                cerr<<"findHomography() failed"<<endl;
                return false;
        }
        cerr<<"H (img -> world) :\n"<<H0<<endl;

                FileStorage fsd("homography_from_chessboard.yml",FileStorage::APPEND);
                if(check){
			fsd<<"H_img2w_onCar"<<H0;
			fsd<<"H_img2w_onCar_original"<<H0;
		}else{
			fsd<<"H_img2w"<<H0;
			fsd<<"H_original"<<H0;
		}
                fsd.release();
                cerr<<"H matrix has saved to homography_from_chessboard.yml"<<endl;
        
        return true;
}

Point2d pix2world(double u,double v,const Mat& H){
	Mat p = (Mat_<double>(3,1) << u,v,1.0);
	Mat q = H*p;
	double w = q.at<double>(2,0);
	if(fabs(w)<1e-12){
		return Point2d(numeric_limits<double>::quiet_NaN(),numeric_limits<double>::quiet_NaN());
	}
	q /= w;
	return Point2d(q.at<double>(0,0), q.at<double>(1,0));
}

Mat draw_axes_canvas(double x2, double y2){
	const int W=600, H=600;
	const double meters_range=2.0;
	const double scale = W/meters_range;
	auto toPix = [&](double x,double y){
		int u = (int)(W/2 + x * (scale/1.0));
		int v = (int)(H/2 - y * (scale/1.0));
		return Point(u,v);
	};

	Mat canvas(H,W,CV_8UC3,Scalar(255,255,255));

	for(int i=-1; i<=1; ++i){
		line(canvas, toPix(i,-1), toPix(i,1), Scalar(220,220,220),1,LINE_AA);
		line(canvas, toPix(-1,i), toPix(1,i), Scalar(220,220,220),1,LINE_AA);
	}

	arrowedLine(canvas,toPix(-1.0,0.0),toPix(1.1,0.0),Scalar(0,0,0),2,LINE_AA,0,0.03);
	arrowedLine(canvas,toPix(0.0,-1.0),toPix(0.0,1.1),Scalar(0,0,0),2,LINE_AA,0,0.03);
	putText(canvas,"x",toPix(1.08,0.02),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,0),1,LINE_AA);
	putText(canvas,"y",toPix(0.02,1.02),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,0),1,LINE_AA);

	circle(canvas,toPix(0,0),5,Scalar(0,0,0),FILLED,LINE_AA);
	circle(canvas,toPix(x2,y2),6,Scalar(0,0,255),FILLED,LINE_AA);
	return canvas;
}
void myThinning(const Mat& srcImg,const Mat& K,const Mat& dist ,const vector<vector<Point>>& contours/*contours*/,int targetIdx,vector<Point>* skelPts){
        Mat s=undistortImage(srcImg,K,dist);
        Mat mask=Mat::zeros(s.size(),CV_8UC1);
        drawContours(mask,contours,targetIdx,255,FILLED);
        Mat skel=Mat::zeros(mask.size(),CV_8UC1);
        Mat eroded, temp;
        Mat se=getStructuringElement(MORPH_CROSS,Size(3,3));
        Mat img=mask.clone();
        while(true){
                erode(img,eroded,se);
                dilate(eroded,temp,se);
                subtract(img,temp,temp);
                bitwise_or(skel,temp,skel);
                if(countNonZero(eroded)==0) break;
                eroded.copyTo(img);
        }
        findNonZero(skel,*skelPts);
}
void findPath(const Mat& bk_img,const Mat& path_img,const Mat& K,const Mat& dist,const int segmentation){
        Mat bk=undistortImage(bk_img,K,dist);
        Mat path=undistortImage(path_img,K,dist);
        Mat path_clone=path.clone();
        // BGR -> GRAY
        Mat bk_gray,path_gray,diff,th,mask;
        cvtColor(bk,bk_gray,COLOR_BGR2GRAY);
        cvtColor(path,path_gray,COLOR_BGR2GRAY);
        // normalization brightness
        const float eps=1.0f;
        double minv, maxv;
        bk_gray.convertTo(bk_gray,CV_32F);
        path_gray.convertTo(path_gray,CV_32F);
        Mat num, den, D;
        absdiff(bk_gray,path_gray,num);
        add(bk_gray,path_gray,den);
        den+=eps;
        divide(num,den,D);
        minMaxLoc(D,&minv,&maxv);
        if(maxv<=0) maxv=1.0;
        D.convertTo(diff,CV_8U,255.0/maxv);
        //gaussian blur and threshold
        GaussianBlur(diff,diff,Size(5,5),0);
        threshold(diff,th,0,255,THRESH_BINARY | THRESH_OTSU);
//        imshow("threshold figure",th);
        //morphology
        //Mat k=getStructuringElement(MORPH_RECT,Size(5,5));
        //morphologyEx(th,mask,MORPH_OPEN,k,Point(-1,-1),1);
        //dilate(mask,mask,k,Point(-1,-1),2);
        //imshow("dilate figure",mask);
        //contours
        vector<vector<Point>> contours;
        findContours(th,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);
        drawContours(path_clone,contours,-1,Scalar(0,255,255),2,LINE_AA);
//        imshow("contours figure",path_clone);
	// 1. find the max_x and min_x of each contour
        vector<pair<int,int>> x_range;
        x_range.reserve(contours.size());
        for(const auto& c : contours){
                if(c.empty()){
                        x_range.emplace_back(0,0);
                        continue;
                }
                Rect r=boundingRect(c);
                int xmin=r.x;
                int xmax=r.x+r.width-1;
                x_range.emplace_back(xmin,xmax);
        }
        // 2. devide the contour in n segmentations
        vector<Point> p1(segmentation+1);
        vector<Point> p2(segmentation+1);
        for(size_t i=0;i<contours.size();i++){
                int range=(x_range[(int)i].second - x_range[(int)i].first)/segmentation;
                for(int j=0;j<=6;j++){
                        for(const Point& p:contours[i]){
                                int x=p.x;
                                int y=p.y;
                                if( (x >= (x_range[(int)i].first+j*range-1)) && (x <= (x_range[(int)i].first+j*range+1)) ){
                                        if(i==0){
                                                p1[j]=Point(x,y);
                                        }else if(i==1){
                                                p2[j]=Point(x,y);
                                        }
                                        break;
                                }
                        }
                }
        }
	// 3. plot p1 & p2 on figure
        for(size_t i=0;i<p1.size();i++){
                Point p=p1[i];
                circle(path_clone,p,5,{0,0,255},FILLED,LINE_AA);
        }
        for(size_t i=0;i<p2.size();i++){
                Point p=p2[i];
                circle(path_clone,p,5,{0,0,255},FILLED,LINE_AA);
        }
//        imshow("points on path",path_clone);
        // 4. plot reference path
        vector<Point> refPts;
        refPts.reserve(p1.size());      // usage : refPts[i].x & refPts[i].y
        for(size_t i=0;i<p1.size();i++){
                int refX=(p1[i].x+p2[i].x)/2;
                int refY=(p1[i].y+p2[i].y)/2;
                refPts.emplace_back(refX,refY);
                circle(path_clone,refPts.back(),5,{0,255,0},FILLED,LINE_AA);
        }
        vector<vector<Point>> poly{refPts};
        polylines(path_clone,poly,false,Scalar(0,255,0),2,LINE_AA);
//        imshow("reference path",path_clone);
	// get thinning pts
        vector<Point> skelPts1;
        vector<Point> skelPts2;
        myThinning(path,K,dist,contours,0,&skelPts1);
        myThinning(path,K,dist,contours,1,&skelPts2);
        Mat path_skel=path.clone();
        Mat mm=Mat::zeros(path.size(),CV_8UC1);
        for(auto& pt:skelPts1) mm.at<uchar>(pt)=255;
        for(auto& pt:skelPts2) mm.at<uchar>(pt)=255;
        path_skel.setTo(Scalar(0,255,0),mm);
        imshow("skelton",path_skel);
        sort(skelPts1.begin(),skelPts1.end(),[](const Point& a,const Point& b){
                        if(a.x!=b.x) return a.x<b.x;
                        return a.y>b.y;
                        });
        sort(skelPts2.begin(),skelPts2.end(),[](const Point& a,const Point& b){
                        if(a.x!=b.x) return a.x<b.x;
                        return a.y>b.y;
                        });
        int r1=((int)skelPts1.size())/segmentation;
        int r2=((int)skelPts2.size())/segmentation;
        vector<Point> points1(segmentation+1);
        vector<Point> points2(segmentation+1);
        for(int i=0;i<=segmentation;i++){
                points1[i]=Point(skelPts1[0+r1*i].x,skelPts1[0+r1*i].y);
                points2[i]=Point(skelPts2[0+r2*i].x,skelPts2[0+r2*i].y);
        }
        for(size_t i=0;i<points1.size();i++){
                Point p=points1[i];
                circle(path_skel,p,5,{0,0,255},FILLED,LINE_AA);
        }
        for(size_t i=0;i<points2.size();i++){
                Point p=points2[i];
                circle(path_skel,p,5,{0,0,255},FILLED,LINE_AA);
        }
//	vector<Point> midPoints;
        for(size_t i=0;i<points1.size();i++){
                midPoints.emplace_back((points1[i].x+points2[i].x)/2,(points1[i].y+points2[i].y)/2);
                circle(path_skel,midPoints.back(),5,{0,255,0},FILLED,LINE_AA);
        }
        vector<vector<Point>> polyMidPoints{midPoints};
        polylines(path_skel,polyMidPoints,false,Scalar(0,255,0),2,LINE_AA);
        imshow("path with points",path_skel);
	sort(midPoints.begin(),midPoints.end(),[](const Point& a,const Point& b){
			if(a.y!=b.y) return a.y>b.y;
			return a.x<b.x;
			});
//        cerr<<midPoints.size()<<endl;
        waitKey(0);
	// pix -> reference coordinate
	FileStorage FSH("H_matrix_new.yml",FileStorage::READ);
	if(!FSH.isOpened()){
		cerr<<"findPath(): Cannot open H_matrix_new.yml."<<endl;
		return;
	}
	Mat H;
	FSH["H_new"]>>H;
	FSH.release();
	vector<Point2f> refPoints;
	for(size_t i=0;i<midPoints.size();i++){
		Point2d w=pix2world((double)midPoints[i].x,(double)midPoints[i].y,H);
		float rx=static_cast<float>(round(w.x*100.0)/100.0);
		float ry=static_cast<float>(round(w.y*100.0)/100.0);
		refPoints.emplace_back(rx,ry);
	}
	// save refPoints as .yml file
	Mat M((int)refPoints.size(),1,CV_32FC2);
	for(int i=0;i<M.rows;i++){
		M.at<Vec2f>(i,0) = Vec2f(refPoints[i].x,refPoints[i].y);
	}
	FileStorage fff("ref_points.yml",FileStorage::WRITE);
	fff<<"refPoints"<<M;
}
bool loadRefPoints(vector<Point2f>& pts){
	FileStorage refFS("ref_points.yml",FileStorage::READ);
	if(!refFS.isOpened()){
		cerr<<"loadRefPoints(): Cannot open ref_points.yml."<<endl;
		return false;
	}
	Mat M;
	refFS["refPoints"]>>M;
	if(M.empty()) return false;
	pts.resize(M.rows);
	if(M.type()==CV_32FC2){
		for(int i=0;i<M.rows;i++){
			Vec2f v=M.at<Vec2f>(i,0);
			pts[i]=Point2f(v[0],v[1]);
		}
	}else if(M.type()==CV_64FC2){
		for(int i=0;i<M.rows;i++){
			Vec2d v=M.at<Vec2d>(i,0);
			pts[i]=Point2f((float)v[0],(float)v[1]);
		}
	}else{
		return false;
	}
	return true;
}

vector<Point2f> sliceMeans(const Mat& mask,const vector<Point>& contour,int n){
        CV_Assert(mask.type()==CV_8UC1);
        n=max(1,n);

        Rect R=boundingRect(contour);
        int H=R.height;
        int step=max(1,(int)ceil(H/(double)n));

        vector<Point2f> centers;
        centers.reserve(n);
        for(int y0=0;y0<H;y0+=step){
                int hh=min(step,H-y0);
                Rect sr(R.x,R.y+y0,R.width,hh);
                Mat slice=mask(sr);

                Moments mu=moments(slice,true);
                if(mu.m00>0){
                        float cx=float(mu.m10/mu.m00) + sr.x;
                        float cy=float(mu.m01/mu.m00) + sr.y;
                        centers.emplace_back(cx,cy);
                }else{
                        centers.emplace_back(sr.x+sr.width*0.5, sr.y+hh*0.5);
                }
        }
        return centers;
}

void findPath_version2(const Mat& bk_img,const Mat& path_img,const Mat& K,const Mat& dist,const int segmentation){
        Mat bk=undistortImage(bk_img,K,dist);
        Mat path=undistortImage(path_img,K,dist);
        Mat path_clone=path.clone();
        // BGR -> LAB
        Mat bk_lab,path_lab,diff,th,mask;
        cvtColor(bk,bk_lab,COLOR_BGR2Lab);
        cvtColor(path,path_lab,COLOR_BGR2Lab);
        vector<Mat> B(3),P(3);
        split(bk_lab,B);
        split(path_lab,P);
        Mat da, db, chromaDiff;
        Mat a1f,b1f,a2f,b2f;
        B[1].convertTo(a1f,CV_32F);
        B[2].convertTo(b1f,CV_32F);
        P[1].convertTo(a2f,CV_32F);
        P[2].convertTo(b2f,CV_32F);
        absdiff(a2f,a1f,da);
        absdiff(b2f,b1f,db);
        magnitude(da,db,chromaDiff);
        chromaDiff.convertTo(chromaDiff,CV_8U);
        GaussianBlur(chromaDiff,chromaDiff,{5,5},0);
        threshold(chromaDiff,th,0,255,THRESH_BINARY | THRESH_OTSU);
        Mat k=getStructuringElement(MORPH_ELLIPSE,{9,9});
        morphologyEx(th,th,MORPH_CLOSE,k,{-1,-1},2);
        morphologyEx(th,th,MORPH_OPEN,k,{-1,-1},1);
        imshow("th",th);
        // obtain the contours & area filter
	vector<vector<Point>> contours;
        findContours(th,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);
        double minArea=1000.0;
        contours.erase(remove_if(contours.begin(),contours.end(),[&](const auto& c){return fabs(contourArea(c))<minArea; }),contours.end());
	sort(contours.begin(),contours.end(),[](const auto& a,const auto& b){return contourArea(a)>contourArea(b);});
        if(contours.size()>2) contours.resize(2);
        drawContours(path_clone,contours,-1,Scalar(0,255,255),2,LINE_AA);
        imshow("contours figure",path_clone);
        sort(contours.begin(),contours.end(),[](const auto& a,const auto& b){
                        return boundingRect(a).x<boundingRect(b).x;
                        });
        // obtain the mask from contours; put all the mask into masks
	vector<Mat> masks;
        masks.reserve(contours.size());
        for(size_t i=0;i<contours.size();i++){
                Mat mask=Mat::zeros(path_clone.size(),CV_8U);
                drawContours(mask,contours,(int)i,Scalar(255),FILLED,LINE_8);
                masks.push_back(mask);
        }
        imshow("first contour mask",masks[0]);
	imshow("second contour mask",masks[1]);
	// devide mask into "segmentation" section & get the center from each section
        vector<vector<Point2f>> centers;
        centers.reserve(contours.size());
        for(size_t i=0;i<contours.size();i++){
                auto c=sliceMeans(masks[i],contours[i],segmentation);
                for(size_t k=1;k<c.size();k++){
                        line(path_clone,c[k-1],c[k],Scalar(0,255,0),3,LINE_AA);
			circle(path_clone,c[k-1],5,{0,0,255},FILLED,LINE_AA);
                }
		circle(path_clone,c[c.size()-1],5,{0,0,255},FILLED,LINE_AA);
                centers.emplace_back(move(c));
        }
        for(size_t i=0;i<centers.size();++i){
                sort(centers[i].begin(),centers[i].end(),
                                [](const Point2f& a,const Point2f& b){
                                        if(a.y!=b.y) return a.y>b.y;
                                        return a.x<b.x;
                                });
        //        cerr<<centers[i]<<endl;
        }

	// get the middle point of each pair coordinate from "centers"
        vector<Point> midRefPts;
        midRefPts.reserve(centers[0].size());
        for(size_t i=0;i<centers[0].size();i++){
                midRefPts.emplace_back( (centers[0][i].x+centers[1][i].x)/2, (centers[0][i].y+centers[1][i].y)/2);
                circle(path_clone,midRefPts.back(),5,{0,255,0},FILLED,LINE_AA);
        }
        imshow("centers",path_clone);
        cerr<<"reference points: "<<endl<<midRefPts<<endl;
	waitKey(0);
	// pix -> reference coordinate
        FileStorage FSH("H_matrix_new.yml",FileStorage::READ);
        if(!FSH.isOpened()){
                cerr<<"findPath(): Cannot open H_matrix_new.yml."<<endl;
                return;
        }
        Mat H;
        FSH["H_new"]>>H;
        FSH.release();
        vector<Point2f> refPoints;
        for(size_t i=0;i<midRefPts.size();i++){
                Point2d w=pix2world((double)midRefPts[i].x,(double)midRefPts[i].y,H);
                float rx=static_cast<float>(round(w.x*100.0)/100.0);
                float ry=static_cast<float>(round(w.y*100.0)/100.0);
                refPoints.emplace_back(rx,ry);
        }
        // save refPoints as .yml file
        Mat M((int)refPoints.size(),1,CV_32FC2);
        for(int i=0;i<M.rows;i++){
                M.at<Vec2f>(i,0) = Vec2f(refPoints[i].x,refPoints[i].y);
        }
        FileStorage fff("ref_points.yml",FileStorage::WRITE);
        fff<<"refPoints"<<M;
}
