#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "user_Serialdrv.h"
#include <stdio.h>
#include <string.h>

/*This is used to show our status in the auto mode.*/
typedef enum {START_UP,					//begins the autonoumous mode
			   PRE_INITIALIZE,				//this initializes the next step in the journey
				INITIALIZE,
			   CHECK_TO_SEE_IF_DONE,	//this is called to see if we are finished with the step we are on
			   COMPLETE}status_flags;	//states that we have completed the journey

status_flags status = START_UP;			//renames statuss_flags as status and starts it as start up.

/*this struct contains the information that we need to go on our journey*/
struct info{
	unsigned char left_speed;		//speed of left motor
	unsigned char right_speed;		//speed of right motor
	int left_journey[9];		//current count on left counter  (modified: J215, made an array different steps needed for different positions)
                                // the values, btw, are completely bogus
	int right_journey[9];		//current count on right counter (modified: J215, same as above only used for right)
	char turn_flag;				//used to add turn data into striaght path 
	char release_tetra;			//used to single time to release the tetra
	char look_at;				//0 is right 1 is left
}; 

/*This table contains the information from above. Each array goes in order of how info is set up/ For example, the first
line will set the left and right motors to 50 and will be that way until both counters are equal to 25.*/
const rom struct info journey_step[] =
{
// 1/3 power is 45, right needs to be about 36 so 80% power
//NEEDS INFO FOR TETRA FOUR ADDED	
// GL = nearest loading station	 
//l_drive, R_drive,  left tetra  1   2  3  4  5  6   7   8    GL  right tetra   1    2  3  4  5   6    7   8   GL  flags
  {150,    150,                {20, 10, 0, 0, 10, 5, 504, 0,  20},             {20,  10, 0, 0, 10, 12, 504, 0, 20}, 0, 0, 0},
  {160,    160,                {30, 10, 0, 0, 20, 5, 504, 0,  30},             {30,  10, 0, 0, 20, 12, 504, 0, 30}, 0, 0, 0},
  {170,    170,                {40, 10, 0, 0, 50, 5, 504, 0,  40},             {40,  10, 0, 0, 50, 12, 504, 0, 40}, 0, 0, 0},
  {160,    160,                {50, 10, 0, 0, 50, 5, 504, 0,  50},             {50,  10, 0, 0, 50, 12, 504, 0, 50}, 0, 0, 0},
  {150,    150,                {62, 10, 0, 0, 50, 5, 504, 0,  62},             {62,  10, 0, 0, 50, 12, 504, 0, 62}, 0, 0, 0}, 

  {127,    140,                {10, 10, 0, 0, 4, 5, 504, 0,  62},             {10,  10, 0, 0, 11, 12, 504, 0, 75}, 0, 0, 0}, 

  {140,    140,                {10, 10, 0, 0, 4, 5, 504, 0,  69},             {10,  10, 0, 0, 11, 12, 504, 0, 82}, 0, 0, 0},
  {145,    145,                {10, 10, 0, 0, 4, 5, 504, 0,  76},             {10,  10, 0, 0, 11, 12, 504, 0, 89}, 0, 0, 0},
  {155,    155,                {10, 10, 0, 0, 4, 5, 504, 0,  83},             {10,  10, 0, 0, 11, 12, 504, 0, 96}, 0, 0, 0},
  {127,    127,                {10, 10, 0, 0, 4, 5, 504, 0,  83},             {10,  10, 0, 0, 11, 12, 504, 0, 96}, 0, 0, 0},

};

/* This is where we keep the current step for memory purposes */
volatile struct info current_step;

/* Load step into work area */
void load_step(unsigned char step_number)
{
   memmovepgm2ram((void *)&current_step,
	(void *)&journey_step[step_number],
			sizeof(current_step));
}

static char step_now = 0;
static unsigned int L_counter = 0;
static unsigned int R_counter = 0;

char r_we_there_yet(int left, char left_speed, int right, char right_speed, char position, char turn_flag, char step, char look_at);
int camera_control(void);
char get_position(void);

void User_Autonomous_Code_1(void)
{
	static char position;
	static char tetra;

while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
        /* Add your own autonomous code here. */
		pressure_control();		//this controls the compressor on the robot.

		switch(status)			//determine the current status of the robot
		{
		case START_UP:			//if we are starting
		//current_step = &current_step;	//set us up on the first leg of the journey
		load_step(0);
        position = get_position();		
		tetra = camera_control();		//calls camera control, this function can be used to determine 
								//what tetra we are going after.
		status = PRE_INITIALIZE;  	//now that we know what tetra we are going after, we start the journey
		case PRE_INITIALIZE:		//if initialize
		//this can be used to do anything that needs to be done before the journey begins.
		status = INITIALIZE;
		break;
		case INITIALIZE:
		status = CHECK_TO_SEE_IF_DONE;	//change status
		break;
		case CHECK_TO_SEE_IF_DONE:	//if CHECK_TO_SEE_IF_DONE
				if(r_we_there_yet(current_step.left_journey[tetra],current_step.left_speed, current_step.right_journey[tetra],current_step.right_speed, position, current_step.turn_flag, step_now, current_step.look_at))
                   {
					if(current_step.left_speed == 127 && current_step.right_speed == 127)
						{
						status = COMPLETE;
						set_drive_L(127);
						set_drive_R(127);
						printf("We are done\r");
						}
					else {
						step_now++;
                        load_step(step_now);
						printf("\r\r\r\r\rCurrently on part %d\r", (int)step_now);                    
					    printf("left journey: %d left_speed: %d right journey: %d right speed: %d turn flag: %d\r",current_step.left_journey[tetra],
                		current_step.left_speed, current_step.right_journey[tetra], current_step.right_speed, current_step.turn_flag); 
						if(!current_step.look_at)
							L_counter = R_counter;
						else R_counter = L_counter; 
						status = INITIALIZE;
						 }	
					}
				
		
			break;
		case COMPLETE:
		break;
		}

        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}


char r_we_there_yet(int left, char left_speed, int right, char right_speed, char position, char turn_flag, char step, char look_at)
{
	char chk = 0;
	static char L_last_val;
	static char R_last_val;

	if (L_banner != L_last_val)
		{
		L_counter++;
		L_last_val = L_banner;
		}

	if (R_banner != R_last_val)
		{
		R_counter++;
		R_last_val = R_banner;
		}
				
//printf("the left counter is %d, right counter is %d\r", L_counter, R_counter);
	if(left_speed < 127)
		drive_L = ((left_speed - 127) * motor_bal + 127);	//ensure we are not over 254								//sets right drive to speed
	else drive_L = left_speed;					//set to highest speed


	if(right_speed > 127)
		drive_R = ((right_speed - 127) * motor_bal + 127);	//ensure we are not over 254								//sets right drive to speed
	else drive_R = right_speed;					//set to highest speed

//set_drive_L(left_speed);
//set_drive_R(right_speed);
printf("left and rights motor speeds %d %d\r", drive_L, drive_R);

	if(look_at)
		if (L_counter < left)
			chk++;

	if(!look_at)
		if (R_counter < right)
			chk++;

if(turn_flag)
	{
	drive_L = 127;
	drive_R = 127;
	Pwm2_green = 1;
	}


	if (chk != 0)
		return 0;
	else return 1;
}

/************************camera_control*/
int camera_control(){

return 8;
}

/*******************get_postion**********************
Blue-left = 0		same as red left
blue- middle = 1
blue - right = 2	same as red right
red-left = 3		same as blue left
red-middle = 4
red-right = 5		same as blue right

atm red = 0 blue = 1;
*/
char get_position(void)
{
	char red_blue_position;		//used to hold red/blue position
	char LCR;					//left right or center
	char position_code;

	red_blue_position = red_blue;	//decide whether we are red or blue side
	if(!left_side && right_side)
		LCR = 0;
		else if (!right_side && left_side)
			LCR = 1;
				else LCR = 2;

	switch(red_blue_position)
	{
	case 0:
		switch(LCR)
		{
		case 0:
		position_code = 3;
		break;
		case 1:
		position_code = 5;
		break;
		case 2:
		position_code = 4;
		break;
		}
	break;
	case 1:
		switch(LCR)
		{
		case 0:
		position_code = 0;
		break;
		case 1:
		position_code = 2;
		break;
		case 2:
		position_code = 1;
		break;
		}
	break;
	}
	//return position_code;
	return 3;
}


