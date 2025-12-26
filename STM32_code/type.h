#include <stdlib.h>
#include <stm32f10x.h>
#include <stdbool.h>
#define FORWARD true
#define BACKWARD false

typedef struct trackingPI{
	float k_pv;
	float k_iv;
	float k_py;
	float k_ptheta;
	float k_dtheta;
	float i_term;
	float i_max;
	float wc_max;
	float wc_min;
	float err_pre;
	float theta_desired;
}TrackingPI;

typedef struct headingPI{
	float kp;
	float ki;
	float i_term;
	float i_max;
	float wc_max;
	float wc_min;
	float err_pre;
} HeadingPI;

typedef struct motor{
	uint8_t EC_num;
	int EC_val;
	bool dir;
	uint32_t pwm;
	float T;
	float kp;
	float ki;
	float kd;
	float speed;
	float last_fb_dg;
	int last_EC_val;
}Motor;

typedef struct position_receiver{
	volatile float x;
	volatile float y;
	float x_pre;
	float y_pre;
	float theta_pre;
}positionReceiver;

typedef struct state_keeper{
	float wmL_ref_pre;
	float wmR_ref_pre;
	float wmlError_pre;
	float wmrError_pre;
	float wmlError_integral;
	float wmrError_integral;
	float theta_desired; // for TrackingPI
}stateKeeper;

Motor* Motor_Init(uint8_t EC_num,bool direction);

positionReceiver* PR_Init(float x_init,float y_init,float theta_init);

stateKeeper* SK_Init(float,float);

HeadingPI* headingPI_init();

TrackingPI* trackingPI_init();

uint32_t getMotorPWM(Motor* m);

void driveMotor(Motor* m);

void RedefineMotor(Motor* mA,Motor* mB);

void getDesiredVelocity(Motor* m);
