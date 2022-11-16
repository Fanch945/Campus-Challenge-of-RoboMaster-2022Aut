#include "streering.h"
#include "example_task.h"
#include "os.h"
#include "bsp_adc.h"
#include "components.h"
#include "power.h"

//extern int case_identifier;

void step_stop(){
	PWM_SetDutyRatio(&pwm2[1],0);
	PWM_SetDutyRatio(&pwm2[3],0);
	PWM_SetDutyRatio(&pwm2[2],0);
	PWM_SetDutyRatio(&pwm2[4],0);
}
void step_forward(int stepping_time_ms){
	PWM_SetDutyRatio(&pwm2[2],0);
	PWM_SetDutyRatio(&pwm2[4],0);
	PWM_SetDutyRatio(&pwm2[1],0.5);
	PWM_SetDutyRatio(&pwm2[3],0.5);
	delay(stepping_time_ms);
}
void step_backward(int stepping_time_ms){
	PWM_SetDutyRatio(&pwm2[1],0);
	PWM_SetDutyRatio(&pwm2[3],0);
	PWM_SetDutyRatio(&pwm2[2],0.5);
	PWM_SetDutyRatio(&pwm2[4],0.5);
	delay(stepping_time_ms);
}


void horizonally_catch(int waiting_time,int mv_for_time,int mv_ba_time){
	step_stop();
	delay(waiting_time);  //remotely adjust the holder  /////*****/////
	step_forward(mv_for_time);
	power_on();
	step_backward(mv_ba_time);
}


int case_identifier = 1;

void Streering_task (void *_){
	for (int i=1;i<=4;i++){
		PWM_SetFrequency(&pwm1[i], 50);
	}
	while(1){
		switch(case_identifier){
			case(0):
				break;
			case(7):
				break;
			case(1):  //No.1
				PWM_SetDutyRatio(&pwm1[1],0.125);  //top SG pi+B  /////*****/////
				PWM_SetDutyRatio(&pwm1[2],0.075);  //left MG ortha to ground
				PWM_SetDutyRatio(&pwm1[4],0.075);  //right MG up about 30drg  /////*****/////
				horizonally_catch(1500,1500,2000);  /////*****/////
				break;				
			case(2):
				break;
			case(3):
				break;
			case(4):
				break;
			case(5):
				break;
			case(6):
				break;
			default:
				break;
		}
		delay(5);
	}
}	
