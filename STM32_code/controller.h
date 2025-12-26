#include <stm32f10x.h>
#include <stdlib.h>
#include <math.h>
#include "type.h"
void MotorControl(Motor* m);
void input_output_controller(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB);
void controller_test(float voltage,Motor* mA,Motor* mB);
void receive_test(positionReceiver* ground,positionReceiver* car);
void input_output_controller_v2(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB);
void input_output_controller_v4(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB,HeadingPI* head);
float HeadingPI_step(HeadingPI* c,float e_theta,float dt);
void PID_controller(float wml_ref,float wmr_r,Motor* mA,Motor* mB,stateKeeper* state);
void input_output_PID_controller(positionReceiver* ground,positionReceiver* car,TrackingPI* track,stateKeeper* state,Motor* mA,Motor* mB);
void myOdometry(positionReceiver* car,Motor* mA,Motor* mB);
void visualFusion(positionReceiver* car,float cx,float cy);
void heading_controller(positionReceiver* ground,positionReceiver* car,Motor* mA,Motor* mB,HeadingPI* head,stateKeeper* state);
void input_output_controller_v2_estimate(positionReceiver* ground,positionReceiver* car,stateKeeper* state,Motor* mA,Motor* mB);