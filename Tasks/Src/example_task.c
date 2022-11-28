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
#define y_axis_ratio 1.0f   //getting forward and backward ratio
#define x_axis_ratio 1.0f   //turning left and right ratio
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
float servo_init_dutyratio[5];  //the array[0] won't be used
bool  reseted=0;
bool  inited=0;
int   adc_colour[6];
bool  reach_stopline=0;
bool  leave_stopline=0;
bool  reach_identifier=0;
bool  auto_function_finished=0;
bool  init_finished=1;
int   colour_judge_threshold[6]={2430,2300,2430,1500,0,1500};   //the dividing point between white and black
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

/*Some important Statements and Notes:
1.Below we call the arm's beginning state reset; the arm's game state init 
2.The corresponding relationship between the number of the servo and its position on the trolley: 
	servo1-left ;
	servo2-right ;
	servo3-bottom ;
	servo4-top;
3.different functions of the controller:
	(control.triSwitch[0] means the left triswitch
	 control.triSwitch[1] means the right triswitch)
case1:left_switch_up---automatic mode(be used for automatically inspecting line and identifier)
case2:left_switch_middle---remote mode(be used for completing all tasks.If possible,some functions can convert to auto)
			subcase1:right_switch_up--arm corresponding to servo 1,2 and 3 and servo 4 horizontally
			subcase2:right_switch_middle---car movement(left stick is responsible for getting forward and backward;right stick is responsible for turning left and right)
			subcase3:right_switch_down--arm corresponding to servo 1,2 and 3 and servo 4 vertically
case3:left_switch_dowm---air pump mode
			subcase1:right_switch_up---turn on the air pump
			subcase2:right_switch_middle---initialize the air pump	
			subcase3:right_switch_down---turn off the air pump
4.some corresponding relationship:
	pwm1[1]---servo 1
	pwm1[2]---servo 2
	pwm1[3]---servo 3
	pwm1[4]---servo 4
	pwm2[1]+[2]---left motor
	pwm2[3]+[4]---right motor
	adc[0]/[1]/[2]---at the front of the car(used for recognizing three identifiers)
	adc[3]/[5]---at the middle bottom of the car(used to adjust the deviation generated during the line tracking process, and automatically track the line forward)
*/


void PWM_setfrequency(int frequency){      
	for(int i=1;i<=4;i++){
		PWM_SetFrequency(&pwm1[i],frequency);
		PWM_SetFrequency(&pwm2[i],frequency);
	}
}
//this function is to set the frequency for 8 interfaces(4 for pwm1 + 4 for pwm2) 

void assign_servo_reset_and_init_dutyratio(){
	servo_reset_dutyratio[1]=0.101f;
	servo_reset_dutyratio[2]=0.052f;
	servo_reset_dutyratio[3]=0.084f;
	servo_reset_dutyratio[4]=0.065f;
	
	servo_init_dutyratio[1]=0.067f;
	servo_init_dutyratio[2]=0.054f;
	servo_init_dutyratio[3]=0.082f;
	servo_init_dutyratio[4]=0.101f;
}
//this function is to let the servo 1-4 to get to the reset position(folded to satisfy the rule) when the match begins and to get to the init position(easier to carry out the following tasks) during the match 

void check_servo_dutyratio(){       //make sure the servo_dutyratio won't exceed [0.025,0.125]
	for(int i=1;i<=4;i++){
		if(servo_dutyratio[i]>0.125f){
			servo_dutyratio[i]=0.125f;
		}
		if(servo_dutyratio[i]<0.025f){
			servo_dutyratio[i]=0.025f;
		}
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
/*this function is to make sure the servo.dutyratio 1-4 won't exceed [0.025,0.125]; */


void servo_reset(){   //set all servo to the reset position
  while(1){
		bool reset_finished=1;

		for(int i=1;i<=4;i++){
			servo_dutyratio[i]=servo_dutyratio[i]*auto_servo_adjust_ratio+servo_reset_dutyratio[i]*(1-auto_servo_adjust_ratio);	
			PWM_SetDutyRatio(&pwm1[i],servo_dutyratio[i]);
			//through weighted average,let servo's duty cycle approach the target reset dutyratio
		}
		delay(auto_servo_delaytime);
		for(int i=1;i<=4;i++){
			if(servo_dutyratio[i]-servo_reset_dutyratio[i]>auto_servo_error_threshold || servo_dutyratio[i]-servo_reset_dutyratio[i]<-auto_servo_error_threshold){
				reset_finished=0;//only every interface of the servo satisfy the condition can avoid obtaining the bool 0
				break;
			}
		}
		if(reset_finished){
			break;
		}
	}
	for(int i=1;i<=4;i++){
		PWM_SetDutyRatio(&pwm1[i],servo_reset_dutyratio[i]);
		//directly set the dutyratio of the servo to the reset dutyratio since the gap is so tiny and safe enough
	}
}
//this function is used to reset the servo in a mild way

void servo_init(){   
  while(1){
		init_finished=1;
		
		for(int i=1;i<=4;i++){
			servo_dutyratio[i]=servo_dutyratio[i]*auto_servo_adjust_ratio+servo_init_dutyratio[i]*(1-auto_servo_adjust_ratio);
			PWM_SetDutyRatio(&pwm1[i],servo_dutyratio[i]);
			//through weighted average,let servo's duty cycle approach the target init dutyratio
		}
		delay(auto_servo_delaytime);
		for(int i=1;i<=4;i++){
			if(servo_dutyratio[i]-servo_init_dutyratio[i]>auto_servo_error_threshold || servo_dutyratio[i]-servo_init_dutyratio[i]<-auto_servo_error_threshold){
				init_finished=0;//only every interface of the servo satisfy the condition can avoid obtaining the bool 0
				break;
			}
		}
		if(init_finished){
			break;
		}
	}
	for(int i=1;i<=4;i++){
		PWM_SetDutyRatio(&pwm1[i],servo_init_dutyratio[i]);
		//directly set the dutyratio of the servo to the initialize dutyratio since the gap is so tiny and safe enough
	}
}
//this function is used to initialize the servo in a mild way


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
//this function is to make sure the motor.dutyratio  won't exceed [0,1];


void PWM_control_motor(int left_direction,float left_dutyratio,int right_direction,float right_dutyratio){
	if(left_direction==forward){
		check_motor_dutyratio();  //for protection
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
//this function is to control the forward and reverse of the motor

void motor_stop(){
	for(int i=1;i<=4;i++){
	  PWM_SetDutyRatio(&pwm2[i],0);
	}
}
//this function is to stop the car

void judge_adc_colour(){
	int imdata[6] = {0,0,0,0,0,0};
	for(int j=0;j<=5;j++){
		for(int i=0;i<=5;i++){
			imdata[i] = fmax(adc_data[i].data,imdata[i]);
		}
		delay(5);
	}
	for(int i=0;i<=5;i++){
		if(imdata[i]<=colour_judge_threshold[i]){
			adc_colour[i]=black;
		}
		else{
			adc_colour[i]=white;
		}
	}
}
/*this function is to give the deinition value of white and black:
	white---above colour_judge_threshold
	black---0~colour_judge_threshold
*/


void follow_line(){
	float adc_error[6];
	adc_error[3]=adc_data[3].data-adc_data[3].last_data;//the error captured by left adc 
	adc_error[5]=adc_data[5].data-adc_data[5].last_data;//the error captured by right adc
	
	motor.left_dutyratio=automode_motor_const_dutyratio+adc_error[3]*adc_error_ratio;
	motor.right_dutyratio=automode_motor_const_dutyratio+adc_error[5]*adc_error_ratio;//revise the dutyratio
	
	check_motor_dutyratio();//make sure dutyratio not exceed the range
	PWM_control_motor(forward,motor.left_dutyratio,forward,motor.right_dutyratio);//reassign the motor's dutyratio	
}


void reset_car_state(){
	reach_stopline=0;
	leave_stopline=0;
	reach_identifier=0;
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
	i_c = colour[0]*4+colour[1]*2+colour[2];
	return i_c;
}
//Fanch's auto function finished *****


//this function is to reset the three state of the car


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
			//when the controll is online,we let the arm convert to init state 
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
		//when the controller is offline,we let the arm convert to reset state
		
		if(control.triSwitch[0]!=left_triswitch_up){
			auto_function_finished=0;
			reset_car_state();
		}
		//convert auto to other mode
		
		switch(control.triSwitch[0]){
			case left_triswitch_up: 				//auto 
			  judge_adc_colour();
			  
			  if(!auto_function_finished){
					if(adc_colour[0]==black && adc_colour[1]==black && adc_colour[2]==black){
					  reach_stopline =1;
					  //when three adc all return black,it means the car reaches the stopline 
				  }		
				  if(!reach_stopline){
					  follow_line();
				  }
				  if(reach_stopline){
					  if(adc_colour[0]==white && adc_colour[1]==white && adc_colour[2]==white){
						  leave_stopline=1;
						//when three adc all return white,it means the car leaves the stopline 
					  }
					  if(!leave_stopline){
						  follow_line();
					  }
					  if(leave_stopline){
					  	if(adc_colour[0]==black || adc_colour[1]==black || adc_colour[2]==black){
						  	reach_identifier=1;
						  	//when some of adc return white,it means the car reaches the stopline
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
										delay(1000);
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
							  // auto mode finished, convert to other mode 
						  }
					  }
				  }
				}
			break;
			
			case left_triswitch_middle:			//remote control mode
				switch(control.triSwitch[1]){
					case right_triswitch_middle:    //car movement
						
						if(control.channel[3]>0){//forward or turning while going forward
							motor.left_dutyratio=control.channel[3]*y_axis_ratio+control.channel[0]*x_axis_ratio;
							motor.right_dutyratio=control.channel[3]*y_axis_ratio-control.channel[0]*x_axis_ratio;
							PWM_control_motor(forward,motor.left_dutyratio,forward,motor.right_dutyratio);
						}
						else if(control.channel[3]<0){//backward or turning while going backward
							motor.left_dutyratio=-control.channel[3]*y_axis_ratio-control.channel[0]*x_axis_ratio;
							motor.right_dutyratio=-control.channel[3]*y_axis_ratio+control.channel[0]*x_axis_ratio;
							PWM_control_motor(reverse,motor.left_dutyratio,reverse,motor.right_dutyratio);
						}
						else{   //controller.channel[3]=0
							if(control.channel[0]>0){   //turning right at the same place
								motor.left_dutyratio=control.channel[0];
								motor.right_dutyratio=control.channel[0];
								PWM_control_motor(forward,motor.left_dutyratio,reverse,motor.right_dutyratio);
							}
							else if(control.channel[0]<0){   //turning left at the same place
								motor.left_dutyratio=-control.channel[0];
								motor.right_dutyratio=-control.channel[0];
								PWM_control_motor(reverse,motor.left_dutyratio,forward,motor.right_dutyratio);
							}
							else{   //controller.channel[3]=0+controller.channel[0]=0
								motor_stop();
							}
						}
						
						
					break;
					
					case right_triswitch_up://servo 4 horizontally
						servo_dutyratio[1]=servo_dutyratio[1]-control.channel[3]*servo_remote_control_speed_ratio;
					  servo_dutyratio[2]=servo_dutyratio[2]+control.channel[1]*servo_remote_control_speed_ratio;
					  servo_dutyratio[3]=servo_dutyratio[3]-control.channel[2]*servo_remote_control_speed_ratio;
					  //correspond the x/y axis value of controller with servo's dutyratio
						check_servo_dutyratio();
					  PWM_SetDutyRatio(&pwm1[1],servo_dutyratio[1]);
					  PWM_SetDutyRatio(&pwm1[2],servo_dutyratio[2]);
				  	PWM_SetDutyRatio(&pwm1[3],servo_dutyratio[3]);
					  set_servo_4_follow_mode(H);
					break;
					
					case right_triswitch_down:  //servo 4 vertically
						servo_dutyratio[1]=servo_dutyratio[1]-control.channel[3]*servo_remote_control_speed_ratio;
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
				
			case left_triswitch_down:            //air pump mode
				switch(control.triSwitch[1]){
					case right_triswitch_up:
						power_on();
						break;
					case right_triswitch_middle:
						Power_Init();
						break;
					case right_triswitch_down:
						power_off();
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
