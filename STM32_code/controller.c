#include "controller.h"
float Pex,Pey;
float vc,wc;
float wmR_ref,wmL_ref,wml_ref,wmr_ref;
float eL,eR;
float ex,ey,etheta;
volatile int controller_check=0;
float theta_d;
float err;
const float TWO_PI=6.2831853071795864769f;
const float PI=3.14159265358979323846f;
typedef enum {TRACK=0,ALIGN=1,STOP=2} Mode;
static Mode mode=ALIGN;

float rho_in=0.03;

extern int headingOK;
extern float dir;
extern bool TRACK_OVER;
extern bool getNewCarPos;
extern bool headingFlag_specialCase;
extern float desiredTheta;

static inline float wrapToPi(float a){

	while(a>PI){
		a -= TWO_PI;
	}
	while(a<-PI){
		a += TWO_PI;
	}
	return a;
}
void input_output_controller(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB){
	controller_check=controller_check+1;
	float kv=5;
	float kw=6;
	float r=0.09;
	float R=0.2275;
	float n=51;
	float Ra=2.9;
	float kb=(0.000254*60*n)/(2*PI);
	float kT=0.02426*n;
	float J=0.1;
	float B=0.1;
	float dt=1;
	
	float d_min=1e-6f;
	float dx=car->x-car->x_pre;
	float dy=car->y-car->y_pre;
	float d=sqrtf(dx*dx+dy*dy);
	
	float thetac=car->theta_pre;
	if(d>d_min){
		float dir=atan2f(dy,dx);
		float delta=wrapToPi(dir-car->theta_pre);
		if(cosf(delta)>=0.0f){
			thetac=dir;
		}
	}
	car->theta_pre=thetac;
	car->x_pre=car->x;
	car->y_pre=car->y;
	
	ex=ground->x - car->x;
	ey=ground->y - car->y;
	//ex=ex*100;
	//ey=ey*100;
	//float Pex,Pey;
	Pex=cosf(thetac)*ex + sinf(thetac)*ey;
	Pey=-sinf(thetac)*ex + cosf(thetac)*ey;
	
	//float vc,wc;
	vc=kv*Pex;
	wc=kw*Pey;
	
	float vcmax=0.35;
	float wcmax=0.5;
	
	vc=(vc<vcmax)?vc:vcmax;
	vc=(vc>(-vcmax))?vc:(-vcmax);
	wc=(wc<wcmax)?wc:wcmax;
	wc=(wc>(-wcmax))?wc:(-wcmax);
	
	//float wmR_ref,wmL_ref;
	wmR_ref=(1/r)*(vc+R*wc);
	wmL_ref=(1/r)*(vc-R*wc);
	float wmMax=11.11;
	wmR_ref=(wmR_ref>(-wmMax))?wmR_ref:(-wmMax);
	wmR_ref=(wmR_ref<wmMax)?wmR_ref:wmMax;
	wmL_ref=(wmL_ref>(-wmMax))?wmL_ref:(-wmMax);
	wmL_ref=(wmL_ref<wmMax)?wmL_ref:wmMax;
	
	float a,b;
	a=(Ra*J)/kT;
	b=(Ra*B)/kT+kb;
	eL=a*(wmL_ref-state->wmL_ref_pre)/dt + b*wmL_ref;
	eR=a*(wmR_ref-state->wmR_ref_pre)/dt + b*wmR_ref;
	
	state->wmL_ref_pre=wmL_ref;
	state->wmR_ref_pre=wmR_ref;
	
	eL=(eL>(-11))?eL:(-11);
	eL=(eL<11)?eL:11;
	eR=(eR>(-11))?eR:(-11);
	eR=(eR<11)?eR:11;
	
	float DL,DR;
	DL=eL/20.1975;
	DR=eR/20.65425;
	
	if(DL<0){
		mA->dir=BACKWARD;
		eL=-eL;
		DL=(20.1975-eL)/20.1975;
	}else{
		mA->dir=FORWARD;
	}
	if(DR<0){
		mB->dir=BACKWARD;
		eR=-eR;
		DR=(20.65425-eR)/20.65425;
	}else{
		mB->dir=FORWARD;
	}
	DL=(DL>0.9)?0.9:DL;
	DR=(DR>0.9)?0.9:DR;
	
	if( ( ex>=(-0.04) && ex<=(0.04) ) && ( ey>=(-0.04) && ey<=(0.04) ) ){
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
	}else{
		mA->pwm=DL*(800);
		mB->pwm=DR*(800);
	}
}

void input_output_controller_v2_estimate(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB){
	float kv=6.4;		//6.4
	float kw=6.4;		//6.4
	float r=0.09;
	float R=0.2275;
	float n=51;
	float Ra=2.9;
	float kb=(0.000254*60*n)/(2*PI);
	float kT=0.02426*n;
	float J=0.1;
	float B=0.1;
	float dt=0.01;
	float x_cur,y_cur,theta_cur,w,v;
	
	float fb_dg_l=mA->EC_val/283.333;
	float fb_dg_r=mB->EC_val/283.333;
	float fb_av_l=fb_dg_l/mA->T;
	float fb_av_r=fb_dg_r/mB->T;
	fb_av_l=(fb_av_l*PI)/180;
	fb_av_r=(fb_av_r*PI)/180;
	w=(r/(2.0*R))*(fb_av_r-fb_av_l);
	v=(r/2.0)*(fb_av_r+fb_av_l);
	
	// before get the new position from host
	if(!getNewCarPos){
		//float theta_mid=wrapToPi(car->theta_pre+0.5*w*dt);
		theta_cur=wrapToPi(car->theta_pre+w*dt);
		x_cur=car->x_pre+v*cosf(theta_cur)*dt;
		y_cur=car->y_pre+v*sinf(theta_cur)*dt;
		car->x=x_cur;
		car->y=y_cur;
	}else{ // getNewCarPos==true
		////getNewCarPos=false;
		//theta_cur=wrapToPi(car->theta_pre+w*dt);
		//x_cur=car->x_pre+v*cosf(theta_cur)*dt;
		//y_cur=car->y_pre+v*sinf(theta_cur)*dt;
		//float zx=car->x;
		//float zy=car->y;
		//car->x=(1.0-alpha_pos)*x_cur+alpha_pos*zx;
		//car->y=(1.0-alpha_pos)*y_cur+alpha_pos*zy;
		//float zx=car->x;
		//float zy=car->y;
		//theta_cur=wrapToPi(car->theta_pre+w*dt);
		//x_cur=(zx+car->x_pre+v*cosf(theta_cur)*dt)/2.0;
		//y_cur=(zy+car->y_pre+v*sinf(theta_cur)*dt)/2.0;
		//car->x=x_cur;
		//car->y=y_cur;
	}
	
	float d_min=0.03;
	float dx=car->x-car->x_pre;
	float dy=car->y-car->y_pre;
	float d=dx*dx+dy*dy;
	
	float thetac=car->theta_pre;
	if(d>(d_min*d_min)){
		float dir=atan2f(dy,dx);
		float delta=wrapToPi(dir-car->theta_pre);
		if(cosf(delta)>=0.0f){
			thetac=dir;
		}
	}
	//// first-order low-pass filter to smooth
	//const float alpha=0.2f;
	//thetac=wrapToPi((1.0f-alpha)*car->theta_pre+alpha*thetac);
	
	if(!getNewCarPos){
		car->theta_pre=theta_cur;
		car->x_pre=x_cur;
		car->y_pre=y_cur;
		ex=ground->x-x_cur;
		ey=ground->y-y_cur;
		Pex=cosf(theta_cur)*ex + sinf(theta_cur)*ey;
		Pey=-sinf(theta_cur)*ex + cosf(theta_cur)*ey;
	}else{
		car->x_pre=car->x;
		car->y_pre=car->y;
		car->theta_pre=thetac;			//thetac
		ex=ground->x-car->x;
		ey=ground->y-car->y;
		Pex=cosf(thetac)*ex + sinf(thetac)*ey;		//thetac
		Pey=-sinf(thetac)*ex + cosf(thetac)*ey;		//thetac
		getNewCarPos=false;
	}
	//if( (Pex>=(-0.015)) && (Pex<=(0.015)) ){
	//	Pex=0.0;
	//}
	//if( (Pey>=(-0.015)) && (Pey<=(0.015)) ){
	//	Pey=0.0;
	//}
	
	//float vc,wc;
	vc=kv*Pex;
	wc=kw*Pey;
	//if( (ey==0) || (ex==0) ){
	//	wc=0;
	//}
	//float vcmax=0.3;
	//float wcmax=1;
	
	//vc=(vc<vcmax)?vc:vcmax;
	//vc=(vc>(-vcmax))?vc:(-vcmax);
	//wc=(wc<wcmax)?wc:wcmax;
	//wc=(wc>(-wcmax))?wc:(-wcmax);
	
	//float wmR_ref,wmL_ref;
	wmR_ref=(1/r)*(vc+R*wc);
	wmL_ref=(1/r)*(vc-R*wc);
	float wmMax=4.7;			// (1/r)*(vc_max+R*wc_max) (4.35)
	float s=fmaxf(fabsf(wmR_ref),fabsf(wmL_ref));
	if(s>wmMax){
		float ss=wmMax/s;
		wmR_ref*=ss;
		wmL_ref*=ss;
	}
	
	float a,b;
	a=(Ra*J)/kT;
	b=(Ra*B)/kT+kb;
	
	eL=a*(1)*(wmL_ref-state->wmL_ref_pre)/dt + b*wmL_ref;
	eR=a*(1)*(wmR_ref-state->wmR_ref_pre)/dt + b*wmR_ref;
	state->wmL_ref_pre=wmL_ref;
	state->wmR_ref_pre=wmR_ref;
	
	//eL=a*(wmL_ref-state->wmL_ref_pre)/dt + b*wmL_ref;
	//eR=a*(wmR_ref-state->wmR_ref_pre)/dt + b*wmR_ref;
	//state->wmL_ref_pre=fb_av_l;
	//state->wmR_ref_pre=fb_av_r;
	
	eL=(eL>(-11))?eL:(-11);
	eL=(eL<11)?eL:11;
	eR=(eR>(-11))?eR:(-11);
	eR=(eR<11)?eR:11;
	
	float DL,DR;
	DL=eL/20.1975;
	DR=eR/20.65425;
	
	if(DL<0){
		mA->dir=BACKWARD;
		eL=-eL;
		DL=(20.1975-eL)/20.1975;
	}else{
		mA->dir=FORWARD;
	}
	if(DR<0){
		mB->dir=BACKWARD;
		eR=-eR;
		DR=(20.65425-eR)/20.65425;
	}else{
		mB->dir=FORWARD;
	}
	
	if( ( Pex>=(-0.03) && Pex<=(0.03) ) && ( Pey>=(-0.03) && Pey<=(0.03) ) ){
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
		TRACK_OVER=true;
	}else{
		mA->pwm=DL*(800);
		mB->pwm=DR*(800);
	}
	
}

void input_output_controller_v2(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB){
	controller_check=controller_check+1;
	float kv=6.4;
	float kw=6.4;
	float r=0.09;
	float R=0.2275;
	float n=51;
	float Ra=2.9;
	float kb=(0.000254*60*n)/(2*PI);
	float kT=0.02426*n;
	float J=0.1;
	float B=0.1;
	float dt=0.01;
	
	float d_min=0.03;
	float dx=car->x-car->x_pre;
	float dy=car->y-car->y_pre;
	//float d=sqrtf(dx*dx+dy*dy);
	float d=dx*dx+dy*dy;
	
	float thetac=car->theta_pre;
	if(d>(d_min*d_min)){
		float dir=atan2f(dy,dx);
		float delta=wrapToPi(dir-car->theta_pre);
		if(cosf(delta)>=0.0f){
			thetac=dir;
		}
	}
	//// first-order low-pass filter to smooth
	//const float alpha=0.2f;
	//thetac=wrapToPi((1.0f-alpha)*car->theta_pre+alpha*thetac);
	
	car->theta_pre=thetac;
	car->x_pre=car->x;
	car->y_pre=car->y;
	
	ex=ground->x - car->x;
	ey=ground->y - car->y;
	
	//ex=ex*100;
	//ey=ey*100;
	//float Pex,Pey;
	Pex=cosf(thetac)*ex + sinf(thetac)*ey;
	Pey=-sinf(thetac)*ex + cosf(thetac)*ey;
	//if( (Pex>=(-0.015)) && (Pex<=(0.015)) ){
	//	Pex=0.0;
	//}
	//if( (Pey>=(-0.015)) && (Pey<=(0.015)) ){
	//	Pey=0.0;
	//}
	
	//float vc,wc;
	vc=kv*Pex;
	wc=kw*Pey;
	//if( (ey==0) || (ex==0) ){
	//	wc=0;
	//}
	//float vcmax=0.3;
	//float wcmax=1;
	
	//vc=(vc<vcmax)?vc:vcmax;
	//vc=(vc>(-vcmax))?vc:(-vcmax);
	//wc=(wc<wcmax)?wc:wcmax;
	//wc=(wc>(-wcmax))?wc:(-wcmax);
	
	//float wmR_ref,wmL_ref;
	wmR_ref=(1/r)*(vc+R*wc);
	wmL_ref=(1/r)*(vc-R*wc);
	float wmMax=4.35;			// (1/r)*(vc_max+R*wc_max)
	float s=fmaxf(fabsf(wmR_ref),fabsf(wmL_ref));
	if(s>wmMax){
		float ss=wmMax/s;
		wmR_ref*=ss;
		wmL_ref*=ss;
	}
	
	float fb_dg_l=mA->EC_val/283.333;
	float fb_dg_r=mB->EC_val/283.333;
	float fb_av_l=fb_dg_l/mA->T;
	float fb_av_r=fb_dg_r/mB->T;
	fb_av_l=(fb_av_l*PI)/180;
	fb_av_r=(fb_av_r*PI)/180;
	
	float a,b;
	a=(Ra*J)/kT;
	b=(Ra*B)/kT+kb;
	
	//eL=a*(wmL_ref-state->wmL_ref_pre)/dt + b*wmL_ref;
	//eR=a*(wmR_ref-state->wmR_ref_pre)/dt + b*wmR_ref;
	//state->wmL_ref_pre=wmL_ref;
	//state->wmR_ref_pre=wmR_ref;
	
	eL=a*(wmL_ref-state->wmL_ref_pre)/dt + b*wmL_ref;
	eR=a*(wmR_ref-state->wmR_ref_pre)/dt + b*wmR_ref;
	state->wmL_ref_pre=fb_av_l;
	state->wmR_ref_pre=fb_av_r;
	
	eL=(eL>(-11))?eL:(-11);
	eL=(eL<11)?eL:11;
	eR=(eR>(-11))?eR:(-11);
	eR=(eR<11)?eR:11;
	
	float DL,DR;
	DL=eL/20.1975;
	DR=eR/20.65425;
	
	if(DL<0){
		mA->dir=BACKWARD;
		eL=-eL;
		DL=(20.1975-eL)/20.1975;
	}else{
		mA->dir=FORWARD;
	}
	if(DR<0){
		mB->dir=BACKWARD;
		eR=-eR;
		DR=(20.65425-eR)/20.65425;
	}else{
		mB->dir=FORWARD;
	}
	//DL=(DL>0.9)?0.9:DL;
	//DR=(DR>0.9)?0.9:DR;
	
	if( ( Pex>=(-0.03) && Pex<=(0.03) ) && ( Pey>=(-0.03) && Pey<=(0.03) ) ){
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
		TRACK_OVER=true;
	}else{
		mA->pwm=DL*(800);
		mB->pwm=DR*(800);
	}
	//mA->pwm=DL*(800);
	//mB->pwm=DR*(800);
	/*
	float error_tol;
	error_tol=sqrtf(ex*ex+ey*ey);
	if( error_tol<0.04 ){
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
	}else{
		mA->pwm=DL*(800);
		mB->pwm=DR*(800);
	}
	*/
}

void heading_controller(positionReceiver* ground,positionReceiver* car,Motor* mA,Motor* mB,HeadingPI* head,stateKeeper* state){
	float r=0.09;
	float R=0.2275;
	float dt=0.01;
	float fb_dg_l=(mA->EC_val)/283.333;
	float fb_dg_r=(mB->EC_val)/283.333;
	float fb_av_l=(fb_dg_l)/dt;
	float fb_av_r=(fb_dg_r)/dt;
	
	fb_av_l=(fb_av_l*PI)/180;
	fb_av_r=(fb_av_r*PI)/180;
	mA->speed=fb_av_l;
	mB->speed=fb_av_r;
	//float wl=(mA->speed*3.1415926535)/180;
	//float wr=(mB->speed*3.1415926535)/180;
	float wl=mA->speed;
	float wr=mB->speed;
	float w=(r/(2.0f*R))*(wr-wl);
	float v=0.0;
	if(!headingFlag_specialCase){
		theta_d=atan2f( (ground->y-car->y),(ground->x-car->x) );
	}else{
		theta_d=desiredTheta;
	}
	float output,P,I;
	err=wrapToPi(theta_d-car->theta_pre);
	car->theta_pre=wrapToPi(car->theta_pre+w*dt);
	
	float ANG_ENTER=(6.0f*PI)/180;
	if(fabs(err)<=ANG_ENTER){
		head->i_term=0.0f;
		mA->pwm=0;
		mB->pwm=0;
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		state->wmlError_integral=0.0;
		state->wmrError_integral=0.0;
		state->wmlError_pre=0.0;
		state->wmrError_pre=0.0;
		state->theta_desired=theta_d;
		headingOK=1;
		return;
	}
	
	head->i_term+=(err*dt);
	if(head->i_term>head->i_max){
		head->i_term=head->i_max;
	}
	if(head->i_term< -head->i_max){
		head->i_term=-head->i_max;
	}
	P=head->kp*err;
	I=head->ki*head->i_term;
	output=P+I;
	head->err_pre=err;
	if(output>head->wc_max){
		output=head->wc_max;
	}
	if(output<head->wc_min){
		output=-head->wc_min;
	}
	
	wmr_ref=(v+dir*R*output)/r;
	wml_ref=(v-dir*R*output)/r;
	PID_controller(wml_ref,wmr_ref,mA,mB,state);
	
}

void myOdometry(positionReceiver* car,Motor* mA,Motor* mB){
	float r=0.09;
	float R=0.02275;
	float dt=0.1;
	
	float wl=(mA->speed*3.1415926535)/180;
	float wr=(mB->speed*3.1415926535)/180;
	
	float v=0.5f*r*(wl+wr);
	float w=(r/(2.0f*R))*(wr-wl);
	
	car->theta_pre=wrapToPi(car->theta_pre+w*dt);
	car->x+=v*cosf(car->theta_pre)*dt;
	car->y+=v*sinf(car->theta_pre)*dt;
}

void visualFusion(positionReceiver* car,float cx,float cy){
	float dx=cx-car->x;
	float dy=cy-car->y;
	if((dx*dx+dy*dy)<(0.5f*0.5f)){
		const float alpha=0.2;
		car->x=(1-alpha)*car->x+alpha*cx;
		car->y=(1-alpha)*car->y+alpha*cy;
	}
}

void PID_controller(float wmL_ref,float wmR_ref,Motor* mA,Motor* mB,stateKeeper* state){
	wmL_ref=(wmL_ref*180)/3.1415926535;
	wmR_ref=(wmR_ref*180)/3.1415926535;
	float fb_dg_l=(mA->EC_val)/283.333;
	float fb_dg_r=(mB->EC_val)/283.333;
	float fb_av_l=(fb_dg_l)/mA->T;
	float fb_av_r=(fb_dg_r)/mB->T;
	//fb_av_l=(fb_av_l*3.1415926535)/180;
	//fb_av_r=(fb_av_r*3.1415926535)/180;
	mA->speed=fb_av_l;
	mB->speed=fb_av_r;
	//mA->last_fb_dg=fb_av_l;
	//mB->last_fb_dg=fb_av_r;
	float err_l,err_r;
	float P_l,I_l,D_l;
	float P_r,I_r,D_r;
	err_l=wmL_ref-fb_av_l;
	err_r=wmR_ref-fb_av_r;
	P_l=mA->kp*err_l;
	P_r=mB->kp*err_r;
	I_l=mA->ki*(err_l+state->wmlError_integral);
	I_r=mB->ki*(err_r+state->wmrError_integral);
	D_l=mA->kd*(err_l-state->wmlError_pre)/mA->T;
	D_r=mB->kd*(err_r-state->wmrError_pre)/mB->T;
	int output_l,output_r;
	output_l=P_l+I_l+D_l;
	output_r=P_r+I_r+D_r;
	if(output_l>90){
		output_l=90;
	}else if(output_l<(-90)){
		output_l=-90;
	}
	if(output_r>90){
		output_r=90;
	}else if(output_r<(-90)){
		output_r=-90;
	}
	uint32_t pwm_val_l,pwm_val_r;
	if(output_l<0){
		mA->dir=BACKWARD;
		pwm_val_l=(-output_l) & 0x000000FF;
	}else{
		mA->dir=FORWARD;
		pwm_val_l=output_l & 0x000000FF;
	}
	if(output_r<0){
		mB->dir=BACKWARD;
		pwm_val_r=(-output_r) & 0x000000FF;
	}else{
		mB->dir=FORWARD;
		pwm_val_r=output_r & 0x000000FF;
	}
	if(mA->dir==FORWARD){
		mA->pwm=pwm_val_l*800/100;
	}else{
		mA->pwm=(100-pwm_val_l)*800/100;
	}
	if(mB->dir==FORWARD){
		mB->pwm=pwm_val_r*800/100;
	}else{
		mB->pwm=(100-pwm_val_r)*800/100;
	}
	state->wmlError_pre=err_l;
	state->wmrError_pre=err_r;
	state->wmlError_integral+=err_l;
	state->wmrError_integral+=err_r;
}

void input_output_PID_controller(positionReceiver* ground,positionReceiver* car,TrackingPI* track,stateKeeper* state,Motor* mA,Motor* mB){
	float r=0.09;
	float R=0.2275;
	float dt=0.01;
	float x_cur,y_cur,theta_cur,w,v;
	
	float fb_dg_l=mA->EC_val/283.333;
	float fb_dg_r=mB->EC_val/283.333;
	float fb_av_l=fb_dg_l/mA->T;
	float fb_av_r=fb_dg_r/mB->T;
	fb_av_l=(fb_av_l*PI)/180;
	fb_av_r=(fb_av_r*PI)/180;
	w=(r/(2.0*R))*(fb_av_r-fb_av_l);
	v=(r/2.0)*(fb_av_r+fb_av_l);
	
	// before get the new position from host
	if(!getNewCarPos){
		//float theta_mid=wrapToPi(car->theta_pre+0.5*w*dt);
		theta_cur=wrapToPi(car->theta_pre+w*dt);
		x_cur=car->x_pre+v*cosf(theta_cur)*dt;
		y_cur=car->y_pre+v*sinf(theta_cur)*dt;
		car->x=x_cur;
		car->y=y_cur;
	}else{ // getNewCarPos==true
	}
	
	float d_min=0.03;
	float dx=car->x-car->x_pre;
	float dy=car->y-car->y_pre;
	float d=dx*dx+dy*dy;
	
	float thetac=car->theta_pre;
	if(d>(d_min*d_min)){
		float dir=atan2f(dy,dx);
		float delta=wrapToPi(dir-car->theta_pre);
		if(cosf(delta)>=0.0f){
			thetac=dir;
		}
	}
	// first-order low-pass filter to smooth
	const float alpha=0.2f;
	thetac=wrapToPi((1.0f-alpha)*car->theta_pre+alpha*thetac);
	
	track->theta_desired=state->theta_desired;
	if(!getNewCarPos){
		car->x_pre=x_cur;
		car->y_pre=y_cur;
		ex=ground->x-x_cur;
		ey=ground->y-y_cur;
		etheta=wrapToPi(track->theta_desired-theta_cur);
		Pex=cosf(theta_cur)*ex + sinf(theta_cur)*ey;
		Pey=-sinf(theta_cur)*ex + cosf(theta_cur)*ey;
		car->theta_pre=theta_cur;
	}else{
		car->x_pre=car->x;
		car->y_pre=car->y;
		car->theta_pre=thetac;										//thetac
		ex=ground->x-car->x;
		ey=ground->y-car->y;
		etheta=wrapToPi(track->theta_desired-thetac);
		Pex=cosf(thetac)*ex + sinf(thetac)*ey;		//thetac
		Pey=-sinf(thetac)*ex + cosf(thetac)*ey;		//thetac
		getNewCarPos=false;
	}

	//float vc,wc;
	track->i_term += (Pex*dt);
	if(track->i_term>track->i_max){
		track->i_term=track->i_max;
	}
	if(track->i_term <-track->i_max){
		track->i_term=-track->i_max;
	}
	
	float vc_max=0.15;
	float wc_max=0.1;
	vc=track->k_pv*Pex + track->k_iv*track->i_term;
	wc=track->k_py*Pey + track->k_ptheta*etheta + track->k_dtheta*(etheta-track->err_pre)/dt;
	track->err_pre=etheta;
	if(vc>vc_max) vc=vc_max;
	if(vc<(-vc_max)) vc=-vc_max;
	if(wc>wc_max) wc=wc_max;
	if(wc<(-wc_max)) wc=-wc_max;
	
	//float wmR_ref,wmL_ref;
	wmR_ref=(1/r)*(vc+R*wc);
	wmL_ref=(1/r)*(vc-R*wc);
	float wmMax=4.7;			// (1/r)*(vc_max+R*wc_max) (4.35)
	float s=fmaxf(fabsf(wmR_ref),fabsf(wmL_ref));
	if(s>wmMax){
		float ss=wmMax/s;
		wmR_ref*=ss;
		wmL_ref*=ss;
	}
	
	if((ex*ex+ey*ey) <= rho_in*rho_in){
		state->wmlError_integral=0.0;
		state->wmrError_integral=0.0;
		state->wmlError_pre=0.0;
		state->wmrError_pre=0.0;
		track->i_term=0.0;
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
		TRACK_OVER=true;
	}else{
		PID_controller(wmL_ref,wmR_ref,mA,mB,state);
	}
	/*
	if(( Pex>=(-0.03) && Pex<=(0.03) ) && ( Pey>=(-0.03) && Pey<=(0.03) )){
		state->wmlError_integral=0.0;
		state->wmrError_integral=0.0;
		state->wmlError_pre=0.0;
		state->wmrError_pre=0.0;
		track->i_term=0.0;
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
		TRACK_OVER=true;
	}else{
		PID_controller(wmL_ref,wmR_ref,mA,mB,state);
	}
	*/
	//PID_controller(wmL_ref,wmR_ref,mA,mB,state);
}

void input_output_controller_v4(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB,HeadingPI* head){
	controller_check=controller_check+1;
	float kv=8;
	float kw=8;
	float r=0.09;
	float R=0.2275;
	float n=51;
	float Ra=2.9;
	float kb=(0.000254*60*n)/(2*3.14159265358979323846f);
	float kT=0.02426*n;
	float J=0.1;
	float B=0.1;
	float dt=1;
	
	float d_min=0.03;
	float dx=car->x-car->x_pre;
	float dy=car->y-car->y_pre;
	//float d=sqrtf(dx*dx+dy*dy);
	float d=dx*dx+dy*dy;
	float wc_cmd;
	//float wc_align=kw*wrapToPi(atan2f(ground->y-car->y,ground->x-car->x)-car->theta_pre);
	/*
	float thetac=car->theta_pre;
	if(mode==ALIGN){
		float thetad=atan2f(ground->y-car->y,ground->x-car->x);
		float etheta=wrapToPi(thetad-car->theta_pre);
		wc_cmd=HeadingPI_step(head,etheta,dt);
		thetac=wrapToPi(thetac+wc_cmd*dt);
	}else{
		if(d>(d_min*d_min)){
			float dir=atan2f(dy,dx);
			float delta=wrapToPi(dir-car->theta_pre);
			if(cosf(delta)>=0.0f){
				thetac=dir;
			}
		}
	}
	*/
	float thetac=car->theta_pre;
	if(d>(d_min*d_min)){
		float dir=atan2f(dy,dx);
		float delta=wrapToPi(dir-car->theta_pre);
		if(cosf(delta)>=0.0f){
			thetac=dir;
		}
	}
	//// first-order low-pass filter to smooth
	float alpha=((mode==ALIGN)?0.6f:0.2f);
	thetac=wrapToPi((1.0f-alpha)*car->theta_pre+alpha*thetac);
	
	car->theta_pre=thetac;
	car->x_pre=car->x;
	car->y_pre=car->y;
	
	ex=ground->x - car->x;
	ey=ground->y - car->y;

	Pex=cosf(thetac)*ex + sinf(thetac)*ey;
	Pey=-sinf(thetac)*ex + cosf(thetac)*ey;
	
	//float vc,wc;
	vc=kv*Pex;
	wc=kw*Pey;
	/*
// 1.0
	const float R_NO_BACK=0.25f;
	float d2=ex*ex+ey*ey;
	if( d2<(R_NO_BACK*R_NO_BACK) && vc<0.0f ){
		vc=0.0f;
	}
// 2.0
	float theta_d=atan2f(ground->y-car->y,ground->x-car->x);
	float e_theta=wrapToPi(theta_d-car->theta_pre);
	const float theta_cut=30.0f*(3.1415926f/180.0f);
	float scale=1.0f-fminf(fabsf(e_theta)/theta_cut,1.0f);
	vc*=scale;
// 3.0	
	const float POS_ENTER=0.05f, POS_EXIT=0.06f;
	const float ANG_ENTER=10.0f*(3.1415926f/180.f);
	const float ANG_EXIT=15.0f*(3.1415926f/180.f);
	float pos_err2=ex*ex+ey*ey;
	float pos_in=(pos_err2<=POS_ENTER*POS_ENTER);
	float pos_out=(pos_err2>=POS_EXIT*POS_EXIT);
	float ang_in=(fabs(e_theta)<=ANG_ENTER);
	float ang_out=(fabs(e_theta)>=ANG_EXIT);
	switch(mode){
		case TRACK:
			if(pos_in) mode=ALIGN;
		break;
		case ALIGN:
			if(!pos_out && ang_in) mode=STOP;
			if(pos_out) mode=TRACK;
		break;
		case STOP:
			if(pos_out || ang_out) mode=TRACK;
		break;
	}
	if(mode==STOP){
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
		return;
	}
	if(mode==ALIGN){
		vc=0.0f;
		float wc_align=wc_cmd;
		const float wc_max=0.35f;
		if(wc_align>wc_max) wc_align=wc_max;
		if(wc_align< (-wc_max)) wc_align=-wc_max;
		wc=wc_align;
	}
	*/
	float theta_d=atan2f(ground->y-car->y,ground->x-car->x);
	float e_theta=wrapToPi(theta_d-car->theta_pre);
	switch(mode){
		case TRACK:
			if((ground->x_pre!=ground->x) || (ground->y_pre!=ground->y)) mode=ALIGN;
		break;
		case ALIGN:
			if(fabsf(e_theta)<=0.1) mode=TRACK;
		break;
		case STOP:
			
		break;
	}
	if(mode==ALIGN){
		vc=0.0f;
		//float wc_align=HeadingPI_step(head,e_theta,dt);
		float wc_align=0;
		float wc_max=0.35f;
		if(wc_align>wc_max) wc_align=wc_max;
		if(wc_align< (-wc_max) ) wc_align=(-wc_max);
		wc=wc_align;
	}
	//float vcmax=0.3;
	//float wcmax=1;
	//float wmR_ref,wmL_ref;
	wmR_ref=(1/r)*(vc+R*wc);
	wmL_ref=(1/r)*(vc-R*wc);
	float wmMax=4.4;			// (1/r)*(vc_max+R*wc_max)
	float s=fmaxf(fabsf(wmR_ref),fabsf(wmL_ref));
	if(s>wmMax){
		float ss=wmMax/s;
		wmR_ref*=ss;
		wmL_ref*=ss;
	}
	
	
	float a,b;
	a=(Ra*J)/kT;
	b=(Ra*B)/kT+kb;
	eL=a*(wmL_ref-state->wmL_ref_pre)/dt + b*wmL_ref;
	eR=a*(wmR_ref-state->wmR_ref_pre)/dt + b*wmR_ref;
	
	state->wmL_ref_pre=wmL_ref;
	state->wmR_ref_pre=wmR_ref;
	
	eL=(eL>(-11))?eL:(-11);
	eL=(eL<11)?eL:11;
	eR=(eR>(-11))?eR:(-11);
	eR=(eR<11)?eR:11;
	
	float DL,DR;
	DL=eL/20.1975;
	DR=eR/20.65425;
	
	if(DL<0){
		mA->dir=BACKWARD;
		eL=-eL;
		DL=(20.1975-eL)/20.1975;
	}else{
		mA->dir=FORWARD;
	}
	if(DR<0){
		mB->dir=BACKWARD;
		eR=-eR;
		DR=(20.65425-eR)/20.65425;
	}else{
		mB->dir=FORWARD;
	}
	
	if( ( ex>=(-0.03) && ex<=(0.03) ) && ( ey>=(-0.03) && ey<=(0.03) ) ){
		mA->dir=FORWARD;
		mB->dir=FORWARD;
		mA->pwm=0;
		mB->pwm=0;
	}else{
		mA->pwm=DL*(800);
		mB->pwm=DR*(800);
	}
	

}

void controller_test(float voltage,Motor* mA,Motor* mB){
	float eL,eR;
	eL=voltage;
	eR=voltage;
	
	float DL,DR;
	DL=eL/20.1975;
	DR=eR/20.65425;
	
	if(DL<0){
		mA->dir=BACKWARD;
		eL=-eL;
		DL=(20.1975-eL)/20.1975;
	}else{
		mA->dir=FORWARD;
	}
	if(DR<0){
		mB->dir=BACKWARD;
		eR=-eR;
		DR=(20.65425-eR)/20.65425;
	}else{
		mB->dir=FORWARD;
	}
	DL=(DL>0.9)?0.9:DL;
	DR=(DR>0.9)?0.9:DR;
	
	float fb_dg_l=mA->EC_val/283.333;
	float fb_dg_r=mB->EC_val/283.333;
	float fb_av_l=(fb_dg_l-mA->last_fb_dg)/mA->T;
	float fb_av_r=(fb_dg_r-mB->last_fb_dg)/mB->T;
  mA->speed=fb_av_l;
	mB->speed=fb_av_r;
	mA->last_fb_dg=fb_dg_l;
	mB->last_fb_dg=fb_dg_r;
	
	mA->pwm=DL*(800);
	mB->pwm=DR*(800);
}

void receive_test(positionReceiver* ground,positionReceiver* car){
	ex=ground->x-car->x;
	ey=ground->y-car->y;
}

void MotorControl(Motor* m){
	driveMotor(m);
}