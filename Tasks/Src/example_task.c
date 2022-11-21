#include "example_task.h"
#include "os.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "components.h"
#include "bsp_adc.h"
#include "controller.h"
#include "pwm.h"
#include "power.h"
#include "stm32f1xx_hal_gpio.h"
#include "user_main.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define proper_gap 0.0025f
#define servo_reset_ratio 0.99f
#define servo_init_ratio 0.99f
#define vertical 1
#define horizontal 0
#define vertical_sum 0.177
#define horizontal_sum 0.127
// above is PWM part

#define left_triswitch_up 1
#define left_triswitch_middle 3
#define left_triswitch_down 2
#define right_triswitch_up 1
#define right_triswitch_middle 3
#define right_triswitch_down 2
#define y_axis_ratio 1.0f  //getting forward and backward ratio
#define x_axis_ratio 1.0f  //turning left and right ratio
#define servo_speed_ratio 0.0002f  
//above is remote controller part

#define automode_motor_dutyratio_standard 0.5f 
#define error_ratio 0.002f
#define forward 1
#define reverse -1
#define colour_judge_threshold 1000  //the dividing point between white and black
#define black 1
#define white 0

// above is auto part 

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


bool  reseted=0;
bool  inited=0;
int   adc_colour[6];
bool  reach_stopline=0;
bool  leave_stopline=0;
bool  reach_identifier=0;
bool  auto_function_finished=0;

typedef struct{
	float duty_ratio[5];
	float resetState_dutyratio[5];
	float initState_dutyratio[5];  //the array[0] won't be used
	float target_dutyratio[5];  //only used in autocatch #Fanch
} Servo;
//the variables of servo

typedef struct{
	float left_dutyratio;
	float right_dutyratio;
} Motor;
//the variables of motor

Servo servo;
Motor motor;
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
			subcase1:right_switch_up---car movement(left stick is responsible for getting forward and backward;right stick is responsible for turning left and right)
			subcase2:right_switch_middle--arm corresponding to servo 1 and 2
			subcase3:right_switch_down--arm corresponding to servo 3 and 4
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

void reset_servo_positionValue(){   
	servo.resetState_dutyratio[1]=0.095f;/** *@note waiting to be revised  */
	servo.resetState_dutyratio[2]=0.058f;/** *@note waiting to be revised  */
	servo.resetState_dutyratio[3]=0.082f;
	servo.resetState_dutyratio[4]=0.075f;
}
//this function is to let the servo 1-4 to get to the reset position(folded to satisfy the rule) when the match begins
	

void initialize_servo_positionValue(){ 
	servo.initState_dutyratio[1]=0.048f;/** *@note waiting to be revised  */
	servo.initState_dutyratio[2]=0.054f;/** *@note waiting to be revised  */
	servo.initState_dutyratio[3]=0.082f;
	servo.initState_dutyratio[4]=0.123f;
}
// this function is to let the servo 1-4 to get to the init position(easier to carry out the following tasks) during the match 


void set_pwm_frequency(int frequency){
	for(int i=1;i<=4;i++){
		PWM_SetFrequency(&pwm1[i], frequency);
		PWM_SetFrequency(&pwm2[i], frequency);
	}
}

//this function is to set the frequency for 8 interfaces(4 for pwm1 + 4 for pwm2) 

void check_servo_dutyratio(){      
	for(int i=1;i<=3;i++){
		if(servo.duty_ratio[i]>0.125f){
			servo.duty_ratio[i]=0.125f;
		}
		if(servo.duty_ratio[i]<0.025f){
			servo.duty_ratio[i]=0.025f;
		}
	}
	if(servo.duty_ratio[4]>0.123f){
			servo.duty_ratio[4]=0.123f;
		}
	if(servo.duty_ratio[4]<0.027f){
			servo.duty_ratio[4]=0.027f;
		}
}
/*this function is to make sure the servo.dutyratio 1-3 won't exceed [0.025,0.125]; 
Particularly, the servo.dutyratio 4 won't exceed [0.027 , 0.123] since servo 4 we use SG90 */


void reset_servo(){
	while(1){
		bool reset_finished=1;
		for(int j=1;j<=4;j++){
			servo.duty_ratio[j]=servo.duty_ratio[j]*servo_reset_ratio+servo.resetState_dutyratio[j]*(1-servo_reset_ratio);
			PWM_SetDutyRatio(&pwm1[j], servo.duty_ratio[j]);
			//through weighted average,let servo's duty cycle approach the target reset dutyratio
		}
		delay(25);
		for(int k=1;k<=4;k++){
			if(fabs(servo.duty_ratio[k]-servo.resetState_dutyratio[k])>proper_gap){
				reset_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
				break;
			}
		}	
		if(reset_finished==1){
			break;
	}
	for(int t=1;t<=4;t++){
		PWM_SetDutyRatio(&pwm1[t], servo.resetState_dutyratio[t]);
	//directly set the dutyratio of the servo to the reset dutyratio since the gap is so tiny and safe enough
	}
	}
}
//this function is used to reset the servo in a mild way


void initialize_servo(){
	while(1){
		bool initialize_finished=1;
		for(int j=1;j<=4;j++){
			servo.duty_ratio[j]=servo.duty_ratio[j]*servo_init_ratio+servo.resetState_dutyratio[j]*(1-servo_init_ratio);
			PWM_SetDutyRatio(&pwm1[j], servo.duty_ratio[j]);
			//through weighted average,let servo's duty cycle approach the target init dutyratio
		}
		delay(25);
		for(int k=1;k<=4;k++){
			if(fabs(servo.duty_ratio[k]-servo.initState_dutyratio[k])>proper_gap){
				initialize_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
				break;
			}
		}	
		if(initialize_finished==1){
			break;
	}
	for(int t=1;t<=4;t++){
		PWM_SetDutyRatio(&pwm1[t], servo.initState_dutyratio[t]);
	//directly set the dutyratio of the servo to the initialize dutyratio since the gap is so tiny and safe enough
	}
	}
}
//this function is used to initialize the servo in a mild way

void servo4_fixed(int shift_mode){
		if(shift_mode==vertical){
			servo.duty_ratio[4]=vertical_sum-servo.duty_ratio[1];
		}	
		if(shift_mode==horizontal){
			servo.duty_ratio[4]=horizontal_sum-servo.duty_ratio[1];
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

//this function is to make sure the motor.dutyratio  won't exceed [0,1];

void PWM_control_motor(int left_direction,float left_dutyratio,int right_direction,float right_dutyratio){
	if(left_direction==forward){
		check_motor_dutyratio();//for protection
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
	for(int i=0;i<=5;i++){
		if(adc_data[i].data<=colour_judge_threshold){
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
	
	motor.left_dutyratio=automode_motor_dutyratio_standard+adc_error[3]*error_ratio; 
	motor.right_dutyratio=automode_motor_dutyratio_standard+adc_error[5]*error_ratio; //revise the dutyratio
	
	check_motor_dutyratio();//make sure dutyratio not exceed the range
	PWM_control_motor(forward,motor.left_dutyratio,forward,motor.right_dutyratio);//reassign the motor's dutyratio
		
}
//this function is to let the car follow the line by using the middle 2 of adc

void reset_car_state(){
	reach_stopline=0;
	leave_stopline=0;
	reach_identifier=0;
}
//this function is to reset the three state of the car

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
void auto_horizonally_catch(int waiting_time,int mv_for_time,int mv_ba_time){
	delay(waiting_time);  //remotely adjust the holder  /////*****/////
	power_on();
	auto_step_forward(mv_for_time);
	delay(500);
	auto_step_backward(mv_ba_time);
	delay(500);
}
void auto_horizonally_exchange(int waiting_time,int mv_for_time,int mv_ba_time){
	delay(waiting_time);  //remotely adjust the holder  /////*****/////
	auto_step_forward(mv_for_time);
	delay(500);
	power_off();
	auto_step_forward(1000);
	auto_step_backward(mv_ba_time);
	delay(500);
}
int identifier_cal(int colour[]){
	int i_c = 0;
	i_c = colour[0]*4+colour[1]*2+colour[2];
	return i_c;
}
//Fanch's auto function finished *****
/* USER CODE END PFD */

/**
* @brief 		main task of example_task
* @param 		args: avoid some funtions put parameters to this function
* @return		none
* @note
*/

void Example_task(void * arg) {
	/* You can write your own code here */
	/* USER CODE BEGIN */
	set_pwm_frequency(50);
	reset_servo_positionValue();
	initialize_servo_positionValue();
	motor_stop();
	
	/* USER CODE END */
	for(int i=1;i<=4;i++) {
		servo.duty_ratio[i]=servo.resetState_dutyratio[i];
	}
	reset_servo();
	reseted=1;
	
	while(1) {
		/* You can write your own code here */
		/* USER CODE BEGIN */
		if(control.online && !inited){
			initialize_servo();
			inited=1;
			reseted=0;
		}
		//when the controll is online,we let the arm convert to init state 
		
		if(!control.online && !reseted){
			motor_stop();
			reset_servo();
			reseted=1;
			inited=0;
		}
		//when the controller is offline,we let the arm convert to reset state
		
		if(control.triSwitch[0]!=left_triswitch_up){
			auto_function_finished=0;
			reset_car_state();
		}
		//convert auto to other mode
		
		switch(control.triSwitch[0]){
			case left_triswitch_up: 				//auto mode
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
								switch(case_identifier){
									//??
									case(0):
										delay(2000);
										break;
									case(7):
										break;
									//??
									case(1):  //No.1 high horizon
										PWM_SetDutyRatio(&pwm1[4],0.100);  //top SG pi+B  /////*****/////
										PWM_SetDutyRatio(&pwm1[1],0.048);  //left MG ortha to ground
										PWM_SetDutyRatio(&pwm1[2],0.025);  //right MG down about 30drg  /////*****/////
										auto_horizonally_catch(1500,1500,2000);  /////*****/////
										break;
									case(2):  //No.2 low horizon
										PWM_SetDutyRatio(&pwm1[4],0.125);  //top SG pi+B  /////*****/////
										PWM_SetDutyRatio(&pwm1[1],0.025);  //left MG horizon forward
										PWM_SetDutyRatio(&pwm1[2],0.075);  //right MG up about 30drg  /////*****/////
										auto_horizonally_catch(1500,1500,2000);  /////*****/////
										break;
									case(3):  //No.3&4 center
									case(4):
										motor_stop();
										delay(1500);  //remotely adjust the holder  
										PWM_SetDutyRatio(&pwm1[4],0.030);  //top SG pi/2+B  /////*****/////
										PWM_SetDutyRatio(&pwm1[1],0.048);  //left MG ortha to ground ++  /////*****/////
										PWM_SetDutyRatio(&pwm1[2],0.025);  //right MG down about 30drg  /////*****/////
										auto_step_forward(1500);
										motor_stop();
										delay(500); 
										power_on();
									///
										target_finished = 1;
										servo.target_dutyratio[2] = 0.075;
										servo.target_dutyratio[4] = 0.080;
										while(target_finished == 1){
											servo.duty_ratio[4]=servo.duty_ratio[4]*servo_reset_ratio+servo.target_dutyratio[4]*(1-servo_reset_ratio);
											servo.duty_ratio[2]=servo.duty_ratio[2]*servo_reset_ratio+servo.target_dutyratio[2]*(1-servo_reset_ratio);
											PWM_SetDutyRatio(&pwm1[4], servo.duty_ratio[4]);
											PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
												//through weighted average,let servo's duty cycle approach the target dutyratio
											delay(25);
											if((fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<proper_gap) && (fabs(servo.duty_ratio[4]-servo.target_dutyratio[4])<proper_gap)){
												target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
												break;
											}
										}
									///
										target_finished=1;
										servo.target_dutyratio[2] = 0.025;
										servo.target_dutyratio[4] = 0.030;
										while(target_finished == 1){
											servo.duty_ratio[4]=servo.duty_ratio[4]*servo_reset_ratio+servo.target_dutyratio[4]*(1-servo_reset_ratio);
											servo.duty_ratio[2]=servo.duty_ratio[2]*servo_reset_ratio+servo.target_dutyratio[2]*(1-servo_reset_ratio);
											PWM_SetDutyRatio(&pwm1[4], servo.duty_ratio[4]);
											PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
												//through weighted average,let servo's duty cycle approach the target dutyratio
											delay(25);
											if((fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<proper_gap) && (fabs(servo.duty_ratio[4]-servo.target_dutyratio[4])<proper_gap)){
												target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
												break;
											}
										}
										break;
									case(5):  //No.Ground
										PWM_SetDutyRatio(&pwm1[4],0.075);  //top SG pi/2+B  /////*****/////
										PWM_SetDutyRatio(&pwm1[1],0.048);  //left MG ortha to ground
										PWM_SetDutyRatio(&pwm1[2],0.025);  //right MG down about 30drg  /////*****/////
										auto_step_forward(500);
										delay(1500);
										power_on();
										//
										target_finished=1;
										servo.target_dutyratio[2] = 0.075;
										servo.target_dutyratio[4] = 0.075;
										while(target_finished == 1){
											servo.duty_ratio[4]=servo.duty_ratio[4]*servo_reset_ratio+servo.target_dutyratio[4]*(1-servo_reset_ratio);
											servo.duty_ratio[2]=servo.duty_ratio[2]*servo_reset_ratio+servo.target_dutyratio[2]*(1-servo_reset_ratio);
											PWM_SetDutyRatio(&pwm1[4], servo.duty_ratio[4]);
											PWM_SetDutyRatio(&pwm1[2], servo.duty_ratio[2]);
												//through weighted average,let servo's duty cycle approach the target dutyratio
											delay(25);
											if((fabs(servo.duty_ratio[2]-servo.target_dutyratio[2])<proper_gap) && (fabs(servo.duty_ratio[4]-servo.target_dutyratio[4])<proper_gap)){
												target_finished=0; //only every interface of the servo satisfy the condition can avoid obtaining the bool 0
												break;
											}
										}
										PWM_SetDutyRatio(&pwm1[1],0.025);
										delay(500);
										break;
									case(6):  //bank
										PWM_SetDutyRatio(&pwm1[4],0.100);  //top SG pi+B  /////*****/////
										PWM_SetDutyRatio(&pwm1[1],0.048);  //left MG ortha to ground
										PWM_SetDutyRatio(&pwm1[2],0.025);  //right MG down about 30drg  /////*****/////
										auto_horizonally_exchange(1500,1500,2000);
										break;
									default:
										break;
								}
								//******
								
							  auto_function_finished=1;
								// auto mode finished, convert to other mode 
						  }
					  }
				  }
				}
			  /*the auto mode is divided into four stages:
				1.follow the guiding line 
				2.reach the stopline 
				3.leave the stopline 
				4.reach the identifier
				Only completing the previous command can the car execute the next command
				*/
				
				
			break;
			
			case left_triswitch_middle:				//remote control mode
				switch(control.triSwitch[1]){
					case right_triswitch_up:    //car movement
						
						if(control.channel[3]>0){//forward or turning while going forward
							motor.left_dutyratio=control.channel[3]*y_axis_ratio+control.channel[0]*x_axis_ratio;
							motor.right_dutyratio=control.channel[3]*y_axis_ratio-control.channel[0]*x_axis_ratio;
							PWM_control_motor(forward,motor.left_dutyratio,forward,motor.right_dutyratio);
						}
						else if(control.channel[3]<0){//backward or turning while going backward
							motor.left_dutyratio=-control.channel[3]*y_axis_ratio+control.channel[0]*x_axis_ratio;
							motor.right_dutyratio=-control.channel[3]*y_axis_ratio-control.channel[0]*x_axis_ratio;/** *@note mayebe waiting to be revised  */
							PWM_control_motor(reverse,motor.left_dutyratio,reverse,motor.right_dutyratio);
						}
						else{//controller.channel[3]=0
							if(control.channel[0]>0){//turning right at the same place
								motor.left_dutyratio=control.channel[0];
								motor.right_dutyratio=control.channel[0];
								PWM_control_motor(forward,motor.left_dutyratio,reverse,motor.right_dutyratio);
							}
							else if(control.channel[0]<0){//turning left at the same place
								motor.left_dutyratio=-control.channel[0];
								motor.right_dutyratio=-control.channel[0];
								PWM_control_motor(reverse,motor.left_dutyratio,forward,motor.right_dutyratio);
							}
							else{//controller.channel[3]=0+controller.channel[0]=0
								motor_stop();
							}
						}
						
						
					break;
					
					case right_triswitch_middle: //arm controlled by servo 1/2
						servo.duty_ratio[1]=servo.duty_ratio[1]+control.channel[3]*servo_speed_ratio;
					  servo.duty_ratio[2]=servo.duty_ratio[2]+control.channel[1]*servo_speed_ratio;
						servo.duty_ratio[3]=servo.duty_ratio[3]-control.channel[2]*servo_speed_ratio;
						//correspond the x/y axis value of controller with servo's dutyratio
						servo4_fixed(horizontal);
						check_servo_dutyratio();
					  PWM_SetDutyRatio(&pwm1[1],servo.duty_ratio[1]);
					  PWM_SetDutyRatio(&pwm1[2],servo.duty_ratio[2]);
						PWM_SetDutyRatio(&pwm1[3],servo.duty_ratio[3]);
					  PWM_SetDutyRatio(&pwm1[4],servo.duty_ratio[4]);
					break;
					
					case right_triswitch_down:  //arm controlled by servo 3/4
						servo.duty_ratio[1]=servo.duty_ratio[1]+control.channel[3]*servo_speed_ratio;
					  servo.duty_ratio[2]=servo.duty_ratio[2]+control.channel[1]*servo_speed_ratio;
						servo.duty_ratio[3]=servo.duty_ratio[3]-control.channel[2]*servo_speed_ratio;
					  servo4_fixed(vertical);
						check_servo_dutyratio();
						PWM_SetDutyRatio(&pwm1[1],servo.duty_ratio[1]);
					  PWM_SetDutyRatio(&pwm1[2],servo.duty_ratio[2]);
					  PWM_SetDutyRatio(&pwm1[3],servo.duty_ratio[3]);
					  PWM_SetDutyRatio(&pwm1[4],servo.duty_ratio[4]);
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
		
		/* USER CODE END */
		delay(10);
	}
}

/* Private functions ---------------------------------------------------------*/
/*	Remember to declare your functions at the beginning of this file and in the .h file */
/* USER CODE BEGIN PF */

/* USER CODE END PF */
