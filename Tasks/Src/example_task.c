#include "example_task.h"
#include "os.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_adc.h"
#include "components.h"
#include "stm32f1xx_hal_gpio.h"
#include "controller.h"
#include "pwm.h"
#include "user_main.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define left_triswitch_up 1
#define left_triswitch_middle 3
#define left_triswitch_down 2
#define right_triswitch_up 1
#define right_triswitch_middle 3
#define right_triswitch_down 2
#define y_axis_ratio 1.0f
#define x_axis_ratio 1.0f
#define servo_remote_control_speed_ratio 0.0002f
#define auto_servo_adjust_ratio 0.98f
#define automode_motor_const_dutyratio 0.5f
#define auto_servo_error_threshold 0.0025f
#define auto_servo_delaytime 20
#define adc_error_ratio 0.01f
#define forward 1
#define reverse -1
#define black 1
#define white 0
#define H 1  //servo 4 follow mode
#define S 0
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
float servo_dutyratio[5];
float servo_reset_dutyratio[5];
float servo_init_dutyratio[5];
bool  reseted=0;
bool  inited=0;
int   adc_colour[6];
bool  reach_stopline=0;
bool  leave_stopline=0;
bool  reach_identifier=0;
bool  auto_function_finished=0;
bool  init_finished=1;
int   colour_judge_threshold[6] = {2430,2300,2400,1500,0,1500};
float sum=0;
float last_dutyratio;
float target;
	
typedef struct{
	float left_dutyratio;
	float right_dutyratio;
}   Motor;

Motor motor;

typedef struct{
	float duty_ratio[5];
	float resetState_dutyratio[5];
	float initState_dutyratio[5];  //the array[0] won't be used
	float target_dutyratio[5];  //only used in autocatch #Fanch
} Servo;
Servo servo; // F

/* USER CODE END PV */

/* Private functions declaration ---------------------------------------------*/
/* USER CODE BEGIN PFD */
void PWM_setfrequency(int frequency){      
	for(int i=1;i<=4;i++){
		PWM_SetFrequency(&pwm1[i],frequency);
		PWM_SetFrequency(&pwm2[i],frequency);
	}
}

void assign_servo_reset_and_init_dutyratio(){
	servo_reset_dutyratio[1]=0.101f;
	servo_reset_dutyratio[2]=0.052f;
	servo_reset_dutyratio[3]=0.084f;
	servo_reset_dutyratio[4]=0.085f;
	
	servo_init_dutyratio[1]=0.067f;
	servo_init_dutyratio[2]=0.054f;
	servo_init_dutyratio[3]=0.082f;
	servo_init_dutyratio[4]=0.101f;
}

void check_servo_dutyratio(){       //make sure the servo_dutyratio won't exceed [0.025,0.125]
	for(int i=1;i<=3;i++){
		if(servo_dutyratio[i]>0.125f){
			servo_dutyratio[i]=0.125f;
		}
		if(servo_dutyratio[i]<0.025f){
			servo_dutyratio[i]=0.025f;
		}
	}
	if(servo_dutyratio[4]>0.123f){
			servo_dutyratio[4]=0.123f;
		}
	if(servo_dutyratio[4]<0.027f){
			servo_dutyratio[4]=0.027f;
		}
}



void set_servo_4_follow_mode(int mode){
	
	if(mode==H){
		servo_dutyratio[4]=0.052+(servo_dutyratio[2]-servo_reset_dutyratio[2]);
	}
	if(mode==S){
		servo_dutyratio[4]=servo_init_dutyratio[4]+(servo_dutyratio[2]-servo_init_dutyratio[2]);
	}
	sum=sum+control.channel[0]*servo_remote_control_speed_ratio;
	servo_dutyratio[4]=servo_dutyratio[4]+sum;
	check_servo_dutyratio();
	target=servo_dutyratio[4];
	while((last_dutyratio-target)>0.005 || (last_dutyratio-target)<-0.005){
		servo_dutyratio[4]=last_dutyratio*0.99+target*0.01;
	  delay(15);
		PWM_SetDutyRatio(&pwm1[4],servo_dutyratio[4]);
		last_dutyratio=servo_dutyratio[4];
	}
	servo_dutyratio[4]=target;
	PWM_SetDutyRatio(&pwm1[4],servo_dutyratio[4]);
	last_dutyratio=servo_dutyratio[4];
}




void servo_reset(){   //set all servo to the reset position
  while(1){
		bool reset_finished=1;

		for(int i=1;i<=4;i++){
			servo_dutyratio[i]=servo_dutyratio[i]*auto_servo_adjust_ratio+servo_reset_dutyratio[i]*(1-auto_servo_adjust_ratio);	
			PWM_SetDutyRatio(&pwm1[i],servo_dutyratio[i]);
		}
		delay(auto_servo_delaytime);
		for(int i=1;i<=4;i++){
			if(servo_dutyratio[i]-servo_reset_dutyratio[i]>auto_servo_error_threshold || servo_dutyratio[i]-servo_reset_dutyratio[i]<-auto_servo_error_threshold){
				reset_finished=0;
				break;
			}
		}
		if(reset_finished){
			break;
		}
	}
	for(int i=1;i<=4;i++){
		PWM_SetDutyRatio(&pwm1[i],servo_reset_dutyratio[i]);
	}
}


void servo_init(){   
  while(1){
		init_finished=1;
		
		for(int i=1;i<=4;i++){
			servo_dutyratio[i]=servo_dutyratio[i]*auto_servo_adjust_ratio+servo_init_dutyratio[i]*(1-auto_servo_adjust_ratio);
			PWM_SetDutyRatio(&pwm1[i],servo_dutyratio[i]);
		}
		delay(auto_servo_delaytime);
		for(int i=1;i<=4;i++){
			if(servo_dutyratio[i]-servo_init_dutyratio[i]>auto_servo_error_threshold || servo_dutyratio[i]-servo_init_dutyratio[i]<-auto_servo_error_threshold){
				init_finished=0;
				break;
			}
		}
		if(init_finished){
			break;
		}
	}
	for(int i=1;i<=4;i++){
		PWM_SetDutyRatio(&pwm1[i],servo_init_dutyratio[i]);
	}
}






void check_motor_dutyratio(){
	if(motor.left_dutyratio>1){
		motor.left_dutyratio=1;
	}
	if(motor.left_dutyratio<0){
		motor.left_dutyratio=0;
	}
	if(motor.right_dutyratio>1){
		motor.right_dutyratio=1;
	}
	if(motor.right_dutyratio<0){
		motor.right_dutyratio=0;
	}
}



void PWM_control_motor(int left_direction,float left_dutyratio,int right_direction,float right_dutyratio){
	if(left_direction==forward){
		check_motor_dutyratio();
		PWM_SetDutyRatio(&pwm2[1],left_dutyratio);
	  PWM_SetDutyRatio(&pwm2[2],0);
	}
	if(left_direction==reverse){
		check_motor_dutyratio();
		PWM_SetDutyRatio(&pwm2[1],0);
	  PWM_SetDutyRatio(&pwm2[2],left_dutyratio);
	}
	if(right_direction==forward){
		check_motor_dutyratio();
		PWM_SetDutyRatio(&pwm2[3],right_dutyratio);
	  PWM_SetDutyRatio(&pwm2[4],0);
	}
	if(right_direction==reverse){
		check_motor_dutyratio();
		PWM_SetDutyRatio(&pwm2[3],0);
	  PWM_SetDutyRatio(&pwm2[4],right_dutyratio);
	}
}


void motor_stop(){
	for(int i=1;i<=4;i++){
	  PWM_SetDutyRatio(&pwm2[i],0);
	}
}


void judge_adc_colour(){
	for(int i=0;i<=5;i++){
		if(adc_data[i].data<=colour_judge_threshold[i]){
			adc_colour[i]=black;
		}
		else{
			adc_colour[i]=white;
		}
	}
}



void follow_line(){
	float adc_error[6];
	adc_error[3]=adc_data[3].data-adc_data[3].last_data;//left adc
	adc_error[5]=adc_data[5].data-adc_data[5].last_data;//right adc
	
	motor.left_dutyratio=automode_motor_const_dutyratio+adc_error[3]*adc_error_ratio;
	motor.right_dutyratio=automode_motor_const_dutyratio+adc_error[5]*adc_error_ratio;
	
	check_motor_dutyratio();
	PWM_control_motor(forward,motor.left_dutyratio,forward,motor.right_dutyratio);
	
	
	
}

void reset_car_state(){
	reach_stopline=0;  //000
	leave_stopline=0;
	reach_identifier=0;
}




void servo_reach_target(float target_1,float target_2,float target_3,int servo_4_mode){
	
	float target[4];
	target[1]=target_1;
	target[2]=target_2;
	target[3]=target_3;
	
	while(1){
		bool finished=1;
		
		for(int i=1;i<=3;i++){
			servo_dutyratio[i]=servo_dutyratio[i]*auto_servo_adjust_ratio+target[i]*(1-auto_servo_adjust_ratio);
			PWM_SetDutyRatio(&pwm1[i],servo_dutyratio[i]);
		}
		set_servo_4_follow_mode(servo_4_mode);
		
		delay(auto_servo_delaytime);
		for(int i=1;i<=3;i++){
			if(servo_dutyratio[i]-target[i]>auto_servo_error_threshold || servo_dutyratio[i]-target[i]<-auto_servo_error_threshold){
				finished=0;
				break;
			}
		}
		if(finished){
			break;
		}
	}
	for(int i=1;i<=3;i++){
		PWM_SetDutyRatio(&pwm1[i],target[i]);
	}
}


//Fanch's auto function begin *****
void auto_step_forward(int stepping_time_ms){
	check_motor_dutyratio();
	PWM_SetDutyRatio(&pwm2[2],0);
	PWM_SetDutyRatio(&pwm2[4],0);
	PWM_SetDutyRatio(&pwm2[1],0.5);
	PWM_SetDutyRatio(&pwm2[3],0.5);
	delay(stepping_time_ms);
	motor_stop();
}
void auto_step_backward(int stepping_time_ms){
	check_motor_dutyratio();
	PWM_SetDutyRatio(&pwm2[1],0);
	PWM_SetDutyRatio(&pwm2[3],0);
	PWM_SetDutyRatio(&pwm2[2],0.5);
	PWM_SetDutyRatio(&pwm2[4],0.5);
	delay(stepping_time_ms);
	motor_stop();
}
void auto_horizonally_exchange(){
	power_off();
	auto_step_forward(1000);
	delay(100);
	auto_step_backward(1500);
	delay(500);
}
void auto_exchange_preparation(){
  int target_finished = 1;
	servo.target_dutyratio[1] = 0.066;
	servo.target_dutyratio[2] = 0.056;
	servo.target_dutyratio[4] = 0.115;
	while(target_finished == 1){
		servo.duty_ratio[1]=servo.duty_ratio[1]*auto_servo_adjust_ratio+servo.target_dutyratio[1]*(1-auto_servo_adjust_ratio);
		servo.duty_ratio[2]=servo.duty_ratio[2]*auto_servo_adjust_ratio+servo.target_dutyratio[2]*(1-auto_servo_adjust_ratio);
		servo.duty_ratio[4]=servo.duty_ratio[4]*auto_servo_adjust_ratio+servo.target_dutyratio[4]*(1-auto_servo_adjust_ratio);
		PWM_SetDutyRatio(&pwm1[1], servo.duty_ratio[1]);
		PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
		PWM_SetDutyRatio(&pwm1[4], servo.duty_ratio[4]);
		//through weighted average,let servo's duty cycle approach the target dutyratio
		delay(25);
		if(fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<auto_servo_error_threshold){
			target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
			break;
		}
	}
}
int identifier_cal(int colour[]){
	int i_c = 0;
	int c[3];
	for (int i=0;i<10;i++){
		c[0]=fmax(c[0],colour[0]);
		c[1]=fmax(c[1],colour[1]);
		c[2]=fmax(c[2],colour[2]);
		delay(10);
	}
	i_c = c[0]*4+c[1]*2+c[2];
	return i_c;
}
//Fanch's auto function finished *****


/* USER CODE END PFD */

/**
* @breif 		main task of example_task
* @param 		args: avoid some funtions put parameters to this function
* @return		none
* @note
*/

void Example_task(void * arg) {
	PWM_setfrequency(50);
	assign_servo_reset_and_init_dutyratio();
	motor_stop();
	
	for(int i=1;i<=4;i++){
		servo_dutyratio[i]=servo_reset_dutyratio[i];
	}
	servo_reset();
	reseted=1; 

	while(1) {
		/* You can write your own code here */
		/* USER CODE BEGIN */
		if(control.online && !inited){
			servo_init();
			inited=1;
			reseted=0;
		}
		if(!control.online && !reseted){
			motor_stop();
			servo_reset();
			reseted=1;
			inited=0;
			while(1){
				if(!control.online){
					delay(10);
				}
				if(control.online){
					break;
				}
			}
		}
		
		if(control.triSwitch[0]!=left_triswitch_up){
			auto_function_finished=0;
			reset_car_state();
		}
		
		switch(control.triSwitch[0]){
			case left_triswitch_up: 				//auto 
			  judge_adc_colour();
			  
			  if(!auto_function_finished){
					if(adc_colour[0]==black && adc_colour[1]==black && adc_colour[2]==black){
					  reach_stopline =1;
				  }		
				  if(!reach_stopline){
					  follow_line();
				  }
				  if(reach_stopline){
					  if(adc_colour[0]==white && adc_colour[1]==white && adc_colour[2]==white){
						  leave_stopline=1;
					  }
					  if(!leave_stopline){
						  follow_line();
					  }
					  if(leave_stopline){
					  	if(adc_colour[0]==black || adc_colour[1]==black || adc_colour[2]==black){
						  	reach_identifier=1;
					  	}
					  	if(!reach_identifier){
					  		follow_line();
					  	}
						  if(reach_identifier){
							  motor_stop();
								//******
								int case_identifier = identifier_cal(adc_colour);
								int target_finished;
								for (int i = 0;i<=4;i++){
									servo.duty_ratio[i] = servo_dutyratio[i];
								}
								switch(case_identifier){
									//??
									case(0):
										delay(2000);
										break;
									case(7):
										break;
									//??
									case(1):  //No.1 high horizon
										target_finished = 1;
										servo.target_dutyratio[1] = 0.066;
										servo.target_dutyratio[2] = 0.030;
										servo.target_dutyratio[4] = 0.080;  /////*****/////
										while(target_finished == 1){
											servo.duty_ratio[1]=servo.duty_ratio[1]*auto_servo_adjust_ratio+servo.target_dutyratio[1]*(1-auto_servo_adjust_ratio);
											servo.duty_ratio[2]=servo.duty_ratio[2]*auto_servo_adjust_ratio+servo.target_dutyratio[2]*(1-auto_servo_adjust_ratio);
											servo.duty_ratio[4]=servo.duty_ratio[4]*auto_servo_adjust_ratio+servo.target_dutyratio[4]*(1-auto_servo_adjust_ratio);
											PWM_SetDutyRatio(&pwm1[1], servo.duty_ratio[1]);
											PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
											PWM_SetDutyRatio(&pwm1[4], servo.duty_ratio[4]);
											//through weighted average,let servo's duty cycle approach the target dutyratio												
											delay(25);
											if(fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<0.002)
												target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
												break;
											}
										auto_step_forward(500);
										delay(1000);
										power_on();
										delay(1000);
										auto_step_backward(1000);
										delay(1000);
										auto_exchange_preparation();
										break;
									case(2):  //No.2 low horizon
										auto_step_backward(1500);  /////*****/////
										delay(1000);
										PWM_SetDutyRatio(&pwm1[1],0.066);  //left MG horizon forward
										target_finished = 1;
										servo.target_dutyratio[1] = 0.03;
										servo.target_dutyratio[2] = 0.08;
										servo.target_dutyratio[4] = 0.030;
										while(target_finished == 1){
											servo.duty_ratio[1]=servo.duty_ratio[1]*auto_servo_adjust_ratio+servo.target_dutyratio[1]*(1-auto_servo_adjust_ratio);
											servo.duty_ratio[2]=servo.duty_ratio[2]*auto_servo_adjust_ratio+servo.target_dutyratio[2]*(1-auto_servo_adjust_ratio);
											servo.duty_ratio[4]=servo.duty_ratio[4]*auto_servo_adjust_ratio+servo.target_dutyratio[4]*(1-auto_servo_adjust_ratio);
											PWM_SetDutyRatio(&pwm1[1], servo.duty_ratio[1]);
											PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
											PWM_SetDutyRatio(&pwm1[4], servo.duty_ratio[4]);
											//through weighted average,let servo's duty cycle approach the target dutyratio												
											delay(25);
											if(fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<auto_servo_error_threshold)
												target_finished=0;
												break;
											}
										delay(1000);
										auto_step_forward(1500);
										delay(1000);
										power_on();
										delay(1000);
										auto_step_backward(1000);
										delay(1000);
										auto_exchange_preparation();
										break;
									case(3):  //No.3&4 center
									case(4):
										delay(1500);  //remotely adjust the holder  
										PWM_SetDutyRatio(&pwm1[4],0.011);  //top SG pi/2+B 
										PWM_SetDutyRatio(&pwm1[1],0.055);  //left MG ortha to ground ++ 
										PWM_SetDutyRatio(&pwm1[2],0.035);  //right MG down about 30drg  
										auto_step_forward(1500);
										motor_stop();
										delay(500);
									///
										target_finished = 1;
										servo.target_dutyratio[2] = 0.048;
										while(target_finished == 1){
											servo.duty_ratio[2]=servo.duty_ratio[2]*auto_servo_adjust_ratio+servo.target_dutyratio[2]*(1-auto_servo_adjust_ratio);
											PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
												//through weighted average,let servo's duty cycle approach the target dutyratio
											delay(25);
											if(fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<auto_servo_error_threshold){
												target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
												break;
											}
										}
										delay(1000);
										power_on();
										delay(1000);
									///
										target_finished=1;
										servo.target_dutyratio[2] = 0.035;
										while(target_finished == 1){
											servo.duty_ratio[2]=servo.duty_ratio[2]*auto_servo_adjust_ratio+servo.target_dutyratio[2]*(1-auto_servo_adjust_ratio);
										  PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
												//through weighted average,let servo's duty cycle approach the target dutyratio
											delay(25);
											if(fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<auto_servo_error_threshold){
												target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
												break;
											}
										}
										delay(1000);
										auto_step_backward(1000);
										delay(1000);
										motor_stop();
										delay(1000);
										auto_exchange_preparation();
										break;
									case(5):  //No.Ground
										delay(1500);
										//
										target_finished=1;
										servo.target_dutyratio[1] = 0.045;
										servo.target_dutyratio[2] = 0.085;
										servo.target_dutyratio[4] = 0.075;
										while(target_finished == 1){
											servo.duty_ratio[1]=servo.duty_ratio[1]*auto_servo_adjust_ratio+servo.target_dutyratio[1]*(1-auto_servo_adjust_ratio);
											servo.duty_ratio[4]=servo.duty_ratio[4]*auto_servo_adjust_ratio+servo.target_dutyratio[4]*(1-auto_servo_adjust_ratio);
											servo.duty_ratio[2]=servo.duty_ratio[2]*auto_servo_adjust_ratio+servo.target_dutyratio[2]*(1-auto_servo_adjust_ratio);
											PWM_SetDutyRatio(&pwm1[1], servo.duty_ratio[1]);
											PWM_SetDutyRatio(&pwm1[4], servo.duty_ratio[4]);
											PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
												//t hrough weighted average,let servo's duty cycle approach the target dutyratio
											delay(25);
											if((fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<0.002) && (fabs(servo.duty_ratio[4]-servo.target_dutyratio[4])<0.002)){
												target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
												break;
											}
										}
										delay(1000);
										power_on();
										delay(1000);
										auto_exchange_preparation();
										break;
									case(6):  //bank
										auto_horizonally_exchange();
										break;
									default:
										break;
								}
								//******
								for (int i = 0;i<=4;i++){
									servo_dutyratio[i] = servo.duty_ratio[i];
								}
								
							  auto_function_finished=1;
						  }
					  }
				  }
				}
			  
				
				
				
			break;
			
			case left_triswitch_middle:				//remote control
				switch(control.triSwitch[1]){
					case right_triswitch_middle:    //move
						
						if(control.channel[3]>0){
							motor.left_dutyratio=control.channel[3]*y_axis_ratio+control.channel[0]*x_axis_ratio;
							motor.right_dutyratio=control.channel[3]*y_axis_ratio-control.channel[0]*x_axis_ratio;
							PWM_control_motor(forward,motor.left_dutyratio,forward,motor.right_dutyratio);
						}
						else if(control.channel[3]<0){
							motor.left_dutyratio=-control.channel[3]*y_axis_ratio-control.channel[0]*x_axis_ratio;
							motor.right_dutyratio=-control.channel[3]*y_axis_ratio+control.channel[0]*x_axis_ratio;
							PWM_control_motor(reverse,motor.left_dutyratio,reverse,motor.right_dutyratio);
						}
						else{
							if(control.channel[0]>0){
								motor.left_dutyratio=control.channel[0];
								motor.right_dutyratio=control.channel[0];
								PWM_control_motor(forward,motor.left_dutyratio,reverse,motor.right_dutyratio);
							}
							else if(control.channel[0]<0){
								motor.left_dutyratio=-control.channel[0];
								motor.right_dutyratio=-control.channel[0];
								PWM_control_motor(reverse,motor.left_dutyratio,forward,motor.right_dutyratio);
							}
							else{
								motor_stop();
							}
						}
						
						
					break;
					
					case right_triswitch_up://servo 4 horizontally
						servo_dutyratio[1]=servo_dutyratio[1]+control.channel[3]*servo_remote_control_speed_ratio;
					  servo_dutyratio[2]=servo_dutyratio[2]+control.channel[1]*servo_remote_control_speed_ratio;
					  servo_dutyratio[3]=servo_dutyratio[3]-control.channel[2]*servo_remote_control_speed_ratio;
						check_servo_dutyratio();
					  PWM_SetDutyRatio(&pwm1[1],servo_dutyratio[1]);
					  PWM_SetDutyRatio(&pwm1[2],servo_dutyratio[2]);
				  	PWM_SetDutyRatio(&pwm1[3],servo_dutyratio[3]);
					  set_servo_4_follow_mode(H);
					break;
					
					case right_triswitch_down:  //servo 4 vertically
						servo_dutyratio[1]=servo_dutyratio[1]+control.channel[3]*servo_remote_control_speed_ratio;
					  servo_dutyratio[2]=servo_dutyratio[2]+control.channel[1]*servo_remote_control_speed_ratio;
					  servo_dutyratio[3]=servo_dutyratio[3]-control.channel[2]*servo_remote_control_speed_ratio;
						check_servo_dutyratio();
					  PWM_SetDutyRatio(&pwm1[1],servo_dutyratio[1]);
					  PWM_SetDutyRatio(&pwm1[2],servo_dutyratio[2]);
				  	PWM_SetDutyRatio(&pwm1[3],servo_dutyratio[3]);
				  	set_servo_4_follow_mode(S);
					break;
				}
			break;
				
			case left_triswitch_down:            //bump
				switch(control.triSwitch[1]){
					case right_triswitch_up:
				    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
					  break;
					
					case right_triswitch_middle:
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
					  break;
					
					case right_triswitch_down:
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
					  break;
				}	
			break;
		}
		
		
		
		
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_8);
		/* USER CODE END */
		delay(10);

	}
}

/* Private functions ---------------------------------------------------------*/
/*	Remember to declare your functions at the beginning of this file and in the .h file */
/* USER CODE BEGIN PF */

/* USER CODE END PF */
