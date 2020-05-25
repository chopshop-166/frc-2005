/*******************************************************************************
* FILE NAME: user_routines.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to user_routines.c and
*  user_routines_fast.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_

#define drive_L 	pwm01			//left drive motor
#define drive_R 	pwm03			//right drive motor
#define motor_bal	.80			//percent reduction for the reversed motor
#define lift    	pwm05			//vertical lift/arm, also known as otis
#define shoulder	pwm07			//elbow on th9 lift
#define lift_bottom rc_dig_in08		//bottom limit switch on lift
#define lift_high   rc_dig_in09	    //high limit switch on lift
#define goal_side   rc_dig_in11		//goal height, side, limit switch
#define goal_center rc_dig_in10		//goal height, center, limit switch
#define otis_counter rc_dig_in14	//counter for otis height
#define gripper_extended  relay1_fwd		//operates the solinoid for the gripper
#define gripper_in	 relay2_fwd		//operates a solinoid for the gripper
#define pressure_sensor rc_dig_in12	//sensor to watch pressure of system
#define compressor		relay3_fwd	//compresser to increase pressure in system	
#define lift_speed_down 60			//speed lift moves down
#define lift_speed_up	194			//speed lift moves up
#define shoulder_range		10		//tolerence on the shoulder
#define shoulder_pot		rc_ana_in05	//pot on the shoulder to tell us where we are at
#define gripper_relock_time	30		//amount of code loops before gripper resets
#define dead_zone			7		//amount for dead zone in one direction in the gain function
#define raw_data		p3_y		//input from pot switch on oi 
#define shoulder_dead_zone 20		//dead zone on shoulder
#define shoulder_speed_up	254		//speed up on shoulder
#define shoulder_speed_down 0		//speed down on shoulder
#define lift_max			108			//max number of clicks for the lift
#define acceleration_limit_max 38	//time of acceleration ramp in code loops; 19 code loops is about .5 seconds
#define L_banner			rc_dig_in15
#define R_banner			rc_dig_in16
/*autonomous slection stuff*/
#define red_blue			1	//rc_dig_in07
#define left_side			0	//rc_dig_in06
#define right_side			1	//rc_dig_in05
#define auto_mode_bit1		rc_dig_in01
#define auto_mode_bit2		rc_dig_in02
#define auto_mode_bit3		rc_dig_in03
#define auto_mode_bit4		rc_dig_in04

/* Used in limit switch routines in user_routines.c */
#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */

/*
 * This data structure defines the layout of our robot's
 * customizable configuration. 
 */
struct mhs166_profile
{
    unsigned char vision_tetra_rgb[3];       /* Color code for vision tetra */
	unsigned char joystick_used;		//used to save what joystic mode the bot is in.
};


/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in user_routines.c */
void User_Initialization(void);
void Process_Data_From_Master_uP(void);
void Default_Routine(void);
unsigned char gain(int input, float reduction);		//modifies input value
void set_drive_L(int speed);//set pwm value(speed) to left drive motor(s)
void set_drive_R(int speed);//set pwm value(speed) to right drive motor(s)
void lift_operate(unsigned char input);         //moves lift up or down
void otis_height_counter(void);		//used to rocord height of otis
int switch_joystick(void);          //changes 1 stick to 2 stick drive and vis versa
int gripper_control(void);		//controls the gripper.
void shoulder_control(void);		//this function is used to control the elbow
void pressure_control(void);	//used to control the compressor
int acceleration( int drive_value, int last_drive_value, unsigned char RorL,unsigned char override);
void activate_profile(void);	//used to activate on-board data saving
void store_profile(void);		//used to store our current profile

/* These routines reside in user_routines_fast.c */
void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
void Process_Data_From_Local_IO(void);
void User_Autonomous_Code_0(void);	//This is the do nothing autonomous mode
void User_Autonomous_Code_1(void);	//This is the vision tetra to center goal AM
void User_Autonomous_Code_2(void);	//This is vision tetra to side goal AM
void User_Autonomous_Code_3(void);	//This is cap alliance center, hit hanging AM
void User_Autonomous_Code_4(void);	//This is moving to the opponents side with tetra
void User_Autonomous_Code_5(void);	//This is going to auto loading station


#endif


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
