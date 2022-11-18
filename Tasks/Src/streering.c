#include "streering.h"
#include "example_task.h"
#include "os.h"
#include "bsp_adc.h"
#include "components.h"
#include "power.h"

extern int case_identifier;

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
	step_stop();
}
void step_backward(int stepping_time_ms){
	PWM_SetDutyRatio(&pwm2[1],0);
	PWM_SetDutyRatio(&pwm2[3],0);
	PWM_SetDutyRatio(&pwm2[2],0.5);
	PWM_SetDutyRatio(&pwm2[4],0.5);
	delay(stepping_time_ms);
	step_stop();
}


void horizonally_catch(int waiting_time,int mv_for_time,int mv_ba_time){
	step_stop();
	delay(waiting_time);  //remotely adjust the holder  /////*****/////
	power_on();
	step_forward(mv_for_time);
	delay(500);
	step_backward(mv_ba_time);
	delay(500);
}
void horizonally_exchange(int waiting_time,int mv_for_time,int mv_ba_time){
	step_stop();
	delay(waiting_time);  //remotely adjust the holder  /////*****/////
	step_forward(mv_for_time);
	delay(500);
	power_off();
	step_forward(1000);
	step_backward(mv_ba_time);
	delay(500);
}



void Streering_task (void *_){
	for (int i=1;i<=4;i++){
		PWM_SetFrequency(&pwm1[i], 50);
	}
	int i = 0;
	float dx =0.005;
	float s1,s2,s4;
	while(1){
		switch(case_identifier){
			case(0):
				delay(2000);
				break;
			case(7):
				break;
			case(1):  //No.1
				PWM_SetDutyRatio(&pwm1[1],0.100);  //top SG pi+B  /////*****/////
				PWM_SetDutyRatio(&pwm1[2],0.048);  //left MG ortha to ground
				PWM_SetDutyRatio(&pwm1[4],0.025);  //right MG up about 30drg  /////*****/////
				horizonally_catch(1500,1500,2000);  /////*****/////
				break;
			case(2):  //No.2
				PWM_SetDutyRatio(&pwm1[1],0.125);  //top SG pi+B  /////*****/////
				PWM_SetDutyRatio(&pwm1[2],0.025);  //left MG horizon forward
				PWM_SetDutyRatio(&pwm1[4],0.060);  //right MG up about 30drg  /////*****/////
				horizonally_catch(1500,1500,2000);  /////*****/////
				break;
			case(3):  //No.3&4 center
				step_stop();
				delay(1500);  //remotely adjust the holder  
				PWM_SetDutyRatio(&pwm1[1],0.030);  //top SG pi/2+B  /////*****/////
				PWM_SetDutyRatio(&pwm1[2],0.048);  //left MG ortha to ground ++  /////*****/////
				PWM_SetDutyRatio(&pwm1[4],0.025);  //right MG down about 30drg  /////*****/////
				step_forward(1500);
				step_stop();
				delay(500); 
				power_on();
				i = 0;
				s1 = 0.048;
				s2 = 0.025;
				s4 = 0.075;
				while(i<=5){
					PWM_SetDutyRatio(&pwm1[1],s4);
					PWM_SetDutyRatio(&pwm1[2],s1);
					s1 += dx/5;
					s4 -= dx/5;
					delay(100);
				}
				i = 0;
				while(i<=5){
					PWM_SetDutyRatio(&pwm1[1],s4);
					PWM_SetDutyRatio(&pwm1[2],s1);
					s1 -= dx/5;
					s4 += dx/5;
					delay(100);
				}
				break;
			case(4):
				step_stop();
				delay(1500);  //remotely adjust the holder  
				PWM_SetDutyRatio(&pwm1[1],0.030);  //top SG pi/2+B  /////*****/////
				PWM_SetDutyRatio(&pwm1[2],0.048);  //left MG ortha to ground ++  /////*****/////
				PWM_SetDutyRatio(&pwm1[4],0.025);  //right MG down about 30drg  /////*****/////
				step_forward(1500);
				step_stop();
				delay(500); 
				power_on();
				i = 0;
				s1 = 0.048;
				s2 = 0.025;
				s4 = 0.075;
				while(i<=5){
					PWM_SetDutyRatio(&pwm1[1],s4);
					PWM_SetDutyRatio(&pwm1[2],s1);
					s1 += dx/5;
					s4 -= dx/5;
					delay(100);
				}
				i = 0;
				while(i<=5){
					PWM_SetDutyRatio(&pwm1[1],s4);
					PWM_SetDutyRatio(&pwm1[2],s1);
					s1 -= dx/5;
					s4 += dx/5;
					delay(100);
				}
				break;
			case(5):  //No.Ground
				PWM_SetDutyRatio(&pwm1[1],0.075);  //top SG pi/2+B  /////*****/////
				PWM_SetDutyRatio(&pwm1[2],0.048);  //left MG ortha to ground
				PWM_SetDutyRatio(&pwm1[4],0.025);  //right MG down about 30drg  /////*****/////
				step_forward(500);
				step_stop();
				delay(1500);
				power_on();
				i = 0;
				s2 = 0.025;
				s4 = 0.075;
				while(i<=7){
					PWM_SetDutyRatio(&pwm1[1],s4);  //top SG pi+B  /////*****/////
					PWM_SetDutyRatio(&pwm1[4],s2);  //right MG up about 30drg  /////*****/////
					s4 += dx;
					s2 += dx;
					delay(100);
				}
				delay(500);
				break;
			case(6):  //bank
				PWM_SetDutyRatio(&pwm1[1],0.100);  //top SG pi+B  /////*****/////
				PWM_SetDutyRatio(&pwm1[2],0.048);  //left MG ortha to ground
				PWM_SetDutyRatio(&pwm1[4],0.025);  //right MG down about 30drg  /////*****/////
				horizonally_exchange(1500,1500,2000);
				break;
			default:
				break;
		}
		delay(5);
	}
}	
