#include "findMultiPath_FUN.h"


void drawThickPolyline(Mat& mask,const vector<Point> &pts,int thickness){
	if(pts.size()<2) return;
	for(size_t i=1;i<pts.size();i++){
		line(mask,pts[i-1],pts[i],Scalar(255),thickness,LINE_AA);
	}
}


int myMinDist(const vector<Point>& a,const vector<Point>& b){
	double minDist=numeric_limits<double>::max();
	for(const auto& pa:a){
		for(const auto& pb:b){
			double dx=double(pa.x)-double(pb.x);
			double dy=double(pa.y)-double(pb.y);
			double d2=dx*dx+dy*dy;
			if(d2<minDist){
				minDist=d2;
			}
		}
	}
	return static_cast<int>(sqrt(minDist));
}
void findEndPoints(const vector<Point>& pts,Point& p1,Point& p2){
	if(pts.empty()){
		cerr<<"Empty points"<<endl;
		return;
	}
	if(pts.size()==1){
		p1=p2=pts[0];
		return;
	}
	double maxDist=-1.0;
	for(size_t i=0;i<pts.size();i++){
		for(size_t j=i+1;j<pts.size();j++){
			double dx=double(pts[i].x)-double(pts[j].x);
			double dy=double(pts[i].y)-double(pts[j].y);
			double d2=dx*dx+dy*dy;
			if(d2>maxDist){
				maxDist=d2;
				p1=pts[i];
				p2=pts[j];
			}
		}
	}
}
void sortByEndpointPolar(vector<vector<Point>> &endPoints,vector<vector<Point>> &centerPts,vector<vector<Point>> &contours0){
	size_t n=endPoints.size();
	if(n==0) return;
	if(centerPts.size() != n || contours0.size() != n) return;
	
	vector<Point2f> mids(n);
	for(size_t i=0;i<n;i++){
		CV_Assert(endPoints[i].size()==2);
		mids[i]=Point2f(0.5*(endPoints[i][0].x+endPoints[i][1].x),0.5*(endPoints[i][0].y+endPoints[i][1].y));
	}

	Point2f ctr(0.f,0.f);
	for(auto& m:mids){
		ctr.x+=m.x;
		ctr.y+=m.y;
	}
	ctr.x/=static_cast<float>(n);
	ctr.y/=static_cast<float>(n);

	vector<int> order(n);
	iota(order.begin(),order.end(),0);

	auto angleOf=[&](int idx){
		float dx=mids[idx].x-ctr.x;
		float dy=mids[idx].y-ctr.y;
		return atan2(dy,dx);
	};

	sort(order.begin(),order.end(),
			[&](int a,int b){
				return angleOf(b)>angleOf(a);
			});
	int startIdx=order[0];
	Point2f best=mids[startIdx];
	for(int idx:order){
		Point2f m=mids[idx];
		if(m.x<best.x || (m.x==best.x && m.y<best.y)){
			best=m;
			startIdx=idx;
		}
	}

	auto it=find(order.begin(),order.end(),startIdx);
	rotate(order.begin(),it,order.end());

	vector<vector<Point>> endSorted,centerSorted,contSorted;
	endSorted.reserve(n);
	centerSorted.reserve(n);
	contSorted.reserve(n);

	for(int idx:order){
		endSorted.push_back(endPoints[idx]);
		centerSorted.push_back(centerPts[idx]);
		contSorted.push_back(contours0[idx]);
	}
	endPoints.swap(endSorted);
	centerPts.swap(centerSorted);
	contours0.swap(contSorted);
}
double pointSegmentDistance(const Point2f& p,const Point2f& a,const Point2f &b){
	Point2f ab=b-a;
	float ab2=ab.x*ab.x+ab.y*ab.y;
	if(ab2<1e-6f){
		return norm(p-a);
	}
	float t=( (p.x-a.x)*ab.x+(p.y-a.y)*ab.y) / ab2;
	t=max(0.f,min(1.f,t));
	Point2f proj(a.x+t*ab.x, a.y+t*ab.y);
	return norm(p-proj);
}
bool hitsAnyCenterline(const Point& p,int contourIdxP,const Point& q,int contourIdxQ,const vector<vector<Point>> &centerPts,double tol=1.0){
	Point2f P(p.x,p.y);
	Point2f Q(q.x,q.y);
	for(size_t ci=0;ci<centerPts.size();ci++){
		if((int)ci==contourIdxP ||(int)ci==contourIdxQ) continue;
		const auto &cent=centerPts[ci];
		if(cent.size()<2) continue;
		for(size_t k=0;k<cent.size();k++){
			Point2f C(cent[k].x,cent[k].y);
			double d=pointSegmentDistance(C,P,Q);
			if(d<tol){
				return true;
			}
		}
	}
	return false;
}
void mySort(vector<Point>& referencePath){
	sort(referencePath.begin(),referencePath.end(),
			[](const Point &a,const Point &b){
				if(a.y!=b.y) return a.y>b.y;
				else return a.x>b.x;
			});
}
void generatePath(const Point& startPoint,const Point& endPoint,int direction,vector<Point>& referencePath){
	int sample=12;
	if(direction==0){
		for(int i=0;i<sample;i++){
			float t=static_cast<float>(i) / static_cast<float>(sample-1);
			Point p=(1-t)*endPoint+t*startPoint;
			referencePath.push_back(p);
		}
		mySort(referencePath);
	}else if((direction==1) || (direction==2)){
		Point midPt;
		if(direction==1){
			midPt.x=startPoint.x-30;
			midPt.y=endPoint.y+30;
		}else if(direction==2){
			midPt.x=startPoint.x+30;
			midPt.y=endPoint.y+30;
		}
		for(int i=1;i<=5;i++){
			float t=static_cast<float>(i)/static_cast<float>(5);
			Point p=(1-t)*midPt+t*startPoint;
			referencePath.push_back(p);
		}
		for(int i=1;i<=9;i++){
			float t=static_cast<float>(i)/static_cast<float>(9);
			Point p=(1-t)*midPt+t*endPoint;
			p.y=p.y-5*(i+1);
			referencePath.push_back(p);
		}
		mySort(referencePath);
		Point insertPt;
		if(direction==1){
			insertPt.x=(referencePath[4].x+referencePath[5].x)/2;
                	insertPt.y=(referencePath[4].y+referencePath[5].y)/2-20;
		}else if(direction==2){
			insertPt.x=(referencePath[4].x+referencePath[5].x)/2;
                        insertPt.y=(referencePath[4].y+referencePath[5].y)/2-20;
		}
		referencePath.insert(referencePath.begin()+5,insertPt);
		//referencePath[(sample/2)-1].x=(referencePath[(sample/2)-2].x+referencePath[(sample/2)].x)/2;
		//referencePath[(sample/2)-1].y=(referencePath[(sample/2)-2].y+referencePath[(sample/2)].y)/2;
	}
}
void findReferencePath_Multi(int seg,const Mat& K0,const Mat& dist0,const Mat& K1,const Mat& dist1){
                //snapshotPath_mode();
                Mat img_bg0=imread("background0.jpg");
                Mat img_path0=imread("path0.jpg");
                Mat und_img_bg0,und_img_path0;
                und_img_bg0=undistortImage(img_bg0,K0,dist0);
                und_img_path0=undistortImage(img_path0,K0,dist0);
                cerr<<"findReferencePath_Multi(): undistorting image done."<<endl;
                Mat bg0_lab,bg1_lab,path0_lab,path1_lab,diff0,th0,mask0,diss1,th1,mask1;
                cvtColor(und_img_bg0,bg0_lab,COLOR_BGR2Lab);
                cvtColor(und_img_path0,path0_lab,COLOR_BGR2Lab);
                cerr<<"findReferencePath_Multi(): converting color done."<<endl;
                vector<Mat> B_0(3),P_0(3);
                split(bg0_lab,B_0);
                split(path0_lab,P_0);
                cerr<<"findReferencePath_Multi(): spliting done."<<endl;
                Mat da0, da1, db0, db1, chromaDiff0, chromaDiff1;
                Mat a1f_0,a1f_1,b1f_0,b1f_1,a2f_0,a2f_1,b2f_0,b2f_1;
                B_0[1].convertTo(a1f_0,CV_32F);
                B_0[2].convertTo(b1f_0,CV_32F);
                P_0[1].convertTo(a2f_0,CV_32F);
                P_0[2].convertTo(b2f_0,CV_32F);
                cerr<<"findReferencePath_Multi(): converting to CV_32F done."<<endl;
                absdiff(a2f_0,a1f_0,da0);
                absdiff(b2f_0,b1f_0,db0);
                cerr<<"findReferencePath_Multi(): differential of image done."<<endl;
		magnitude(da0,db0,chromaDiff0);
                cerr<<"findReferencePath_Multi(): getting magnitude of differential image done."<<endl;
                chromaDiff0.convertTo(chromaDiff0,CV_8U);
                cerr<<"findReferencePath_Multi(): converting to CV_8U done."<<endl;
                GaussianBlur(chromaDiff0,chromaDiff0,{5,5},0);
                cerr<<"findReferencePath_Multi(): Gaussianblur done."<<endl;
                threshold(chromaDiff0,th0,0,255,THRESH_BINARY | THRESH_OTSU);
                cerr<<"findReferencePath_Multi(): thresholding done."<<endl;
                Mat k=getStructuringElement(MORPH_ELLIPSE,{9,9});
                morphologyEx(th0,th0,MORPH_CLOSE,k,{-1,-1},2);
                morphologyEx(th0,th0,MORPH_OPEN,k,{-1,-1},1);
                cerr<<"findReferencePath_Multi(): morphologuEx() done."<<endl;
                // imshow("th1",th1);
                // imshow("th0",th0);
                /* obtain the contour by area-filter */
		vector<vector<Point>> contours0,contours1;
                findContours(th0,contours0,RETR_EXTERNAL,CHAIN_APPROX_NONE);
                double minArea=1000.0;
                contours0.erase(remove_if(contours0.begin(),contours0.end(),[&](const auto& c){return fabs(contourArea(c))<minArea; }),contours0.end());
                sort(contours0.begin(),contours0.end(),[](const auto& a,const auto& b){return contourArea(a)>contourArea(b);});
               // if(contours0.size()>2) contours0.resize(2);
               
                drawContours(und_img_path0,contours0,-1,Scalar(0,255,255),2,LINE_AA);
                imshow("contours0",und_img_path0);
       
                cerr<<"findReferencePath_Multi(): saving contours.jpg done."<<endl;
                sort(contours0.begin(),contours0.end(),[](const auto& a,const auto& b){
                        return boundingRect(a).x<boundingRect(b).x;
                        });
        
		// obtain the mask from contours; put all the mask into masks
                vector<Mat> masks0,masks1;
                masks0.reserve(contours0.size());
       
                for(size_t i=0;i<contours0.size();i++){
                        Mat mask=Mat::zeros(und_img_path0.size(),CV_8U);
                        drawContours(mask,contours0,(int)i,Scalar(255),FILLED,LINE_8);
                        masks0.push_back(mask);
                }
		// obtain center-points in each contour
		vector<Point> allPtsForBounding;
		for(auto &c:contours0)
			allPtsForBounding.insert(allPtsForBounding.end(),c.begin(),c.end());
		Rect bigBox=boundingRect(allPtsForBounding);
		int midLine=bigBox.x+(bigBox.width/2);
		float minRadius=15.0f;
		vector<vector<Point>> centerPts;
		Mat D,T;
		for(int i=0;i<masks0.size();i++){
			distanceTransform(masks0[i],D,DIST_L2,3);
			T=Mat::zeros(masks0[i].size(),CV_8U);
			for(int y=1;y<D.rows-1;y++){
				for(int x=1;x<D.cols-1;x++){
					float v=D.at<float>(y,x);
					if(v<minRadius) continue;
					bool isMax=true;
					for(int j=-1;j<=1 && isMax;j++){
						for(int t=-1;t<=1;t++){
							if(D.at<float>(y+j,x+t) > v+1e-3f){
								isMax=false;
								break;
							}
						}
					}
					if(isMax) T.at<uchar>(y,x)=255;
				}
			}
			vector<Point> pts;
			findNonZero(T,pts);
			centerPts.push_back(pts);
		}
		for(size_t i=0;i<centerPts.size();i++){
			for(size_t c=0;c<centerPts[i].size();c++){
				circle(und_img_path0,centerPts[i][c],2,{0,255,0},FILLED,LINE_AA);
			}
		}
		imshow("b",und_img_path0);
		// 
		int minLen=9999;
		int all=0;
		int base=1;
		int combination;
		int idx[2];
		int idxIdx=0;
		for(int i=0;i<centerPts.size();i++){
			all=all+(base<<i);
		}
		for(int i=0;i<centerPts.size();i++){
			combination=all-(base<<i);
			for(int j=0;j<centerPts.size();j++){
				if( ((combination>>j)&1)==1 ){
					idx[idxIdx++]=j;
				}
			}
			idxIdx=0;
			int a=myMinDist(centerPts[idx[0]],centerPts[idx[1]]);
			if(a<minLen) minLen=a;
			cerr<<"min: "<<minLen<<endl;
		}
		cerr<<"Final min: "<<minLen<<endl;
		int expand=static_cast<int>(midLine/2);
		// obtain endPoints in each contour
		vector<vector<Point>> endPoints;
		for(size_t i=0;i<centerPts.size();i++){
			Point a,b;
			findEndPoints(centerPts[i],a,b);
			vector<Point> s;
			s.push_back(a);
			s.push_back(b);
			endPoints.push_back(s);
		}
		for(size_t i=0;i<endPoints.size();i++){
			for(size_t j=0;j<endPoints[i].size();j++){
				circle(und_img_path0,endPoints[i][j],5,{0,0,255},FILLED,LINE_AA);
			}
		}
		circle(und_img_path0,endPoints[0][1],5,{255,0,0},FILLED,LINE_AA);
		imshow("c",und_img_path0);
		
		// sort endPoints in clockwise 
		//vector<pair<Point,Point>> connectors;
		sortByEndpointPolar(endPoints,centerPts,contours0);
		for(size_t i=0;i<endPoints.size();i++){
			cerr<<"x0: "<<endPoints[i][0].x<<", y0: "<<endPoints[i][0].y<<endl;
			cerr<<"x1: "<<endPoints[i][1].x<<", y1: "<<endPoints[i][1].y<<endl;
			cerr<<"-------"<<endl;
		}

		vector<Endpoint> endpointsFlat;
		for(int ci=0;ci<(int)endPoints.size();ci++){
			for(int li=0;li<(int)endPoints[ci].size();li++){
				Endpoint ep;
				ep.p=endPoints[ci][li];
				ep.contourIdx=ci;
				ep.localIdx=li;
				endpointsFlat.push_back(ep);
			}
		}
		// obtain which endpoint should connect
		int M=(int)endpointsFlat.size();
		cerr<<"M: "<<M<<endl;
		vector<CandidateEdge> candidates;
		for(int i=0;i<M;i++){
			for(int j=i+1;j<M;j++){
				const Endpoint &E1=endpointsFlat[i];
				const Endpoint &E2=endpointsFlat[j];
				if(E1.contourIdx==E2.contourIdx) continue;
				if(hitsAnyCenterline(E1.p,E1.contourIdx,E2.p,E2.contourIdx,centerPts)){
					continue;
				}
				double dx=double(E1.p.x)-double(E2.p.x);
				double dy=double(E1.p.y)-double(E2.p.y);
				CandidateEdge ce;
				ce.ei=i;
				ce.ej=j;
				ce.dist2=dx*dx+dy*dy;
				candidates.push_back(ce);
			}
		}
		cerr<<"candidates size: "<<candidates.size()<<endl;
		sort(candidates.begin(),candidates.end(),
				[](const CandidateEdge &a,const CandidateEdge& b){
					return a.dist2<b.dist2;
				});
		vector<bool> used(M,false);
		vector<pair<Point,Point>> connectors;

		for(const auto& e:candidates){
			if(used[e.ei] || used[e.ej]) continue;
			used[e.ei]=used[e.ej]=true;
			const Endpoint &E1=endpointsFlat[e.ei];
			const Endpoint &E2=endpointsFlat[e.ej];
			connectors.emplace_back(E1.p,E2.p);
		}
		for(const auto& seg:connectors){
			line(und_img_path0,seg.first,seg.second,Scalar(0,255,0),2,LINE_AA);
			cerr<<"seg first: "<<seg.first<<", seg second: "<<seg.second<<endl;
		}
		cerr<<connectors[1].first.x<<endl;
		imshow("d",und_img_path0);

		Mat pathMask=Mat::zeros(und_img_path0.size(),CV_8U);
                for(const auto&c:centerPts){
                        drawThickPolyline(pathMask,c,4);
                }

		for(auto &cp:connectors){
			line(pathMask,cp.first,cp.second,Scalar(255),4,LINE_AA);
		}
		
//		imshow("e",pathMask);
//		Mat kkkk=getStructuringElement(MORPH_ELLIPSE,Size(5,5));
//                morphologyEx(pathMask,pathMask,MORPH_CLOSE,kkkk);
//		imshow("f",pathMask);
//		vector<vector<Point>> contourTemp;
//		findContours(pathMask,contourTemp,RETR_EXTERNAL,CHAIN_APPROX_NONE);
//		Mat pathMaskFilled=Mat::zeros(pathMask.size(),CV_8U);
//		drawContours(pathMaskFilled,contourTemp,-1,Scalar(255),FILLED);
//		imshow("g",pathMaskFilled);
		

		// obtain entry and exit
		vector<Point> entry;
		for(auto &cp:connectors){
                        int rx=(cp.first.x+cp.second.x)/2;
			int ry=(cp.first.y+cp.second.y)/2;
			Point w=Point(rx,ry);
			entry.push_back(w);
                }
		Point startPoint;
		int startPointIdx;
		int startY=-1;
		for(size_t i=0;i<entry.size();i++){
			circle(und_img_path0,entry[i],5,Scalar(255,255,0),FILLED,LINE_AA);
			if(entry[i].y > startY){
				startPoint.x=entry[i].x;
				startPoint.y=entry[i].y;
				startPointIdx=i;
				startY=entry[i].y;
			}
		}
		circle(und_img_path0,startPoint,5,Scalar(0,255,255),FILLED,LINE_AA);
		imshow("i",und_img_path0);
		

		// obtain paths with entry and exit
		vector<vector<Point>> referencePaths;
		int direction;
		for(size_t i=0;i<entry.size();i++){
			vector<Point> referencePath;
			if(i==startPointIdx) continue;
			double slope=double(entry[i].y-startPoint.y) / double(entry[i].x-startPoint.x);
			if( slope>5 || slope<(-5) ){
				direction=0;		// forward
			}else if( -slope<0 ){
				direction=1;		// left
			}else if( -slope>0){
				direction=2;		// right
			}
			generatePath(startPoint,entry[i],direction,referencePath);
			referencePaths.push_back(referencePath);
		}

		for(size_t i=0;i<referencePaths.size();i++){
			cerr<<i<<"th path: "<<endl;
			Mat und_img_path0_clone=und_img_path0.clone();
			for(size_t j=0;j<referencePaths[i].size();j++){
				circle(und_img_path0_clone,referencePaths[i][j],5,Scalar(0,i*255,255),FILLED,LINE_AA);
				//cerr<<"x: "<<referencePaths[i][j].x<<", y: "<<referencePaths[i][j].y<<endl;
			}
			for(size_t j=1;j<referencePaths[i].size();j++){
				line(und_img_path0_clone,referencePaths[i][j-1],referencePaths[i][j],Scalar(0,i*255,255),3,LINE_AA);
			}
			string filename="Multi path: "+to_string(i)+".jpg";
			imwrite(filename,und_img_path0_clone);	
		}
		
		// save as .yml file
		FileStorage fsh("H_matrix_new.yml",FileStorage::READ);
		Mat HHH;
		fsh["H_new"]>>HHH;
		fsh.release();
		vector<vector<Point2f>> rps;
		for(size_t i=0;i<referencePaths.size();i++){
			vector<Point2f> rp;
			cerr<<"path "<<i<<":"<<endl;
			for(size_t j=0;j<referencePaths[i].size();j++){
				Point2d w=pix2world( (double)referencePaths[i][j].x,(double)referencePaths[i][j].y,HHH);
				float rx=static_cast<float>(round(w.x*100.0)/100.0);
				float ry=static_cast<float>(round(w.y*100.0)/100.0);
				cerr<<"x: "<<rx<<", y: "<<ry<<endl;
				rp.emplace_back(rx,ry);
			}
			rps.push_back(rp);
		}
		FileStorage ff("multiPaths.yml",FileStorage::APPEND);
		for(size_t i=0;i<rps.size();i++){
			Mat M((int)rps[i].size(),1,CV_32FC2);
			for(int j=0;j<M.rows;j++){
				M.at<Vec2f>(j,0)=Vec2f(rps[i][j].x,rps[i][j].y);
			}
			ff<<"path_"+to_string(i)<<M;
		}
		ff.release();
		/*
		int paths_idx=1;
		vector<Point2f> pts;
		FileStorage ref("multiPaths.yml",FileStorage::READ);
		if(!ref.isOpened()){
			cerr<<"loading multiPaths.yml failed"<<endl;
			return ;
		}
		FileNode root=ref.root();
		int keyNum=(int)root.size();
		cerr<<"key nums: "<<keyNum<<endl;
		Mat MMM;
		ref["path_"+to_string(paths_idx)]>>MMM;
		if(MMM.empty()) return;
		pts.resize(MMM.rows);
		if(MMM.type()==CV_32FC2){
			for(int i=0;i<MMM.rows;i++){
				Vec2f v=MMM.at<Vec2f>(i,0);
				pts[i]=Point2f(v[0],v[1]);
			}
		}else if(MMM.type()==CV_64FC2){
			for(int i=0;i<MMM.rows;i++){
				Vec2d v=MMM.at<Vec2d>(i,0);
				pts[i]=Point2f((float)v[0],(float)v[1]);
			}
		}else{
			return;
		}
		for(size_t i=0;i<pts.size();i++){
			cerr<<"x: "<<pts[i].x<<", y: "<<pts[i].y<<endl;
		}
		*/
}

bool loadMultiPaths(vector<Point2f>& pts,int paths_idx){
        	FileStorage ref("multiPaths.yml",FileStorage::READ);
        	if(!ref.isOpened()){
                	cerr<<"loading multiPaths.yml failed"<<endl;
        	        return false;
	        }
	        FileNode root=ref.root();
                int keyNum=(int)root.size();
                cerr<<"key nums: "<<keyNum<<endl;
                Mat MMM;
                ref["path_"+to_string(paths_idx)]>>MMM;
                if(MMM.empty()) return false;
                pts.resize(MMM.rows);
                if(MMM.type()==CV_32FC2){
                        for(int i=0;i<MMM.rows;i++){
                                Vec2f v=MMM.at<Vec2f>(i,0);
                                pts[i]=Point2f(v[0],v[1]);
                        }
                }else if(MMM.type()==CV_64FC2){
                        for(int i=0;i<MMM.rows;i++){
                                Vec2d v=MMM.at<Vec2d>(i,0);
                                pts[i]=Point2f((float)v[0],(float)v[1]);
                        }
                }else{
                        return false;
                }
		return true;
}
