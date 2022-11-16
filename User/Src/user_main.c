#include "user_main.h"
#include "os.h"
#include "example_task.h"


/*******************************************************************
 * >>>>>>>>>>>>>>>>>>>>>>>>>>> NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<<<< *
 *******************************************************************
 * You are allowed to do whatever you like to this file as long as *
 * the User_Init() function is kept here, and the header file      *
 * user_main.h are included.                                       *
 *                                                                 *
 * You are supposed to wrap your main codes in functions, not      *
 * placing them into User_Init(). This function is only meant to   *
 * be the place where you init your global variables, create and   *
 * start your own threads. Make sure you exit it as soon as possi- *
 * ble. NEVER should there be any infinite loops.                  *
 *                                                                 *
 * If you need to define any macros or global variables, please    *
 * create your own header file under the group Application/main.   *
 * So should your own source file (*.c) be.                        *
 *                                                                 *
 * If you have any problem, feel free to open an issue on GitLab.  *
 *                                                                 *
 * Happy coding, RoboMasters :)                                    *
 *                                                                 *
 *******************************************************************/
 
 
void User_Init(void){
	/* Copy this line and modify the first two parameters if you want to add new threads */
	/* The first parameter can be any string you want */
	/* The second parameter is the function name you want to run in the thread */
	osThreadCreate("example",Example_task,NULL,osPriorityNormal,128);
	/* USER CODE BEGIN */
	
	/* USER CODE END */
}
