/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "user_Serialdrv.h"
#include "user_camera.h"


extern unsigned char aBreakerWasTripped;

/*
 * This is our persistent profile. It survives a power-off.
 */
#pragma romdata MHS166_PROFILE
const rom struct mhs166_profile saved_profile = {0, 1, 2, 3}; /* Factory settings */

#pragma romdata

/*
 * This is our volatile profile
 */
struct mhs166_profile volatile_profile;

/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  Putdata(&txdata);             /* DO NOT CHANGE! */
  Serial_Driver_Initialize();
	
  //printf("IFI 2005 User Processor Initialized ...\r");  /* Optional - Print initialization message. */
  /* Note:  use a '\r' rather than a '\n' with the new compiler (v2.4) */

#if _USE_CMU_CAMERA
//  cam_state_flag = 0;
#endif
  activate_profile();
  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
  static unsigned char i;
  static unsigned char delay;

  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

  Default_Routine();  /* Optional.  See below. */

  Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

  /* Eample code to check if a breaker was ever tripped. */

  if (aBreakerWasTripped)
  {
    for (i=1;i<29;i++)
    {
      if (Breaker_Tripped(i))
        User_Byte1 = i;  /* Update the last breaker tripped on User_Byte1 (to demonstrate the use of a user byte) 
                            Normally, you do something else if a breaker got tripped (ex: limit a PWM output)     */
    }
  }


  Putdata(&txdata);             /* DO NOT CHANGE! */
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
	float speed_reduction = .5;		//this is used to store the reduction value for the joysticks
	static int L_last_drive_value = 127;
	static int R_last_drive_value = 127;
	static unsigned char override_on_off = 0;
	static unsigned char prev_p1_sw_top = 0;
	
	/*This next section of code is used to change the type of joystick control.
	This can be changed by using p1_sw_aux1 to start this function. Other
	functions can be added here as controls and settings */
	if(p1_sw_aux1) 
	{
	volatile_profile.joystick_used = switch_joystick();		//changes gears on robot
	store_profile();
	}


	speed_reduction = (float)p4_aux / 254.0;					//this is used to determine the speed reduction on the bot
		/*******************************2 Joystick drive****************************
	This function allows the robot to be controled using 2 joysticks that are plugged into 
	port 1 and port 2 of the operator interface. It contains a "dead zone" that has a range
	of 7 each direction and a gain function to allow for finer control of the robot. */
	//this has been fixed to work on the test bot so that the back wheels drive


	if (!volatile_profile.joystick_used)
		{
#if 1
		if(!lift_bottom)
			{
			set_drive_L(gain(p2_y, speed_reduction));
			set_drive_R(gain(p1_y, speed_reduction));
			}
		else if(p2_sw_trig)
				{
			set_drive_L(gain(p2_y, speed_reduction));
			set_drive_R(gain(p1_y, speed_reduction));
				}
		else	
			{
			set_drive_L(gain(p2_y, .5));
			set_drive_R(gain(p1_y, .5));
//		/	printf("the left and right drives are %d %d\r", drive_L, drive_R);
			}	
		}
				


#else

			if(!lift_bottom)
				{
				override_on_off = 0;
				set_drive_L(acceleration(gain(p2_y, speed_reduction),L_last_drive_value,1,1));
				set_drive_R(acceleration(gain(p1_y, speed_reduction),R_last_drive_value,2,1));
				}
				else
					{
						if ((p1_sw_top)&&(!prev_p1_sw_top))  // switch toggled
							{
								if (override_on_off == 0)
									override_on_off = 1;
								else override_on_off = 0;
							}

									if (((override_on_off == 0)&&(p1_sw_trig == 1)) || override_on_off == 1)
									{
									set_drive_L(acceleration(gain(p2_y, speed_reduction),L_last_drive_value,1,1));
									set_drive_R(acceleration(gain(p1_y, speed_reduction),R_last_drive_value,2,1));
									}
									else
									{
									set_drive_L(acceleration(p2_y,L_last_drive_value,1,0));
									set_drive_R(acceleration(p1_y,R_last_drive_value,2,0));
									}
				}
		L_last_drive_value = p2_y;
		R_last_drive_value = p1_y;
		prev_p1_sw_top = p1_sw_top;
		}

#endif


	/********************************1 Joystick drive***********************
	This function allows the robot to be controled using 1 joystick that is plugged into 
	port 1 of the operator interface. It contains a "dead zone" that has a range
	of 7 each direction and a gain function to allow for finer control of the robot. */
	//this has been fixed to work on the test bot so that the back wheels drive

#if 0
		else if (volatile_profile.joystick_used)
					{
					set_drive_R(gain(p1_y - p1_x + 127, speed_reduction));
					set_drive_L(gain(p1_y + p1_x - 127, speed_reduction));
					}
#endif
	lift_operate(p3_x);			//operatates otis
	gripper_control();				//operates the gripper
	shoulder_control();				//operates the arm extension
	pressure_control();				//operators the compressor
	otis_height_counter();

	#if _USE_CMU_CAMERA
  	return;     //Don't execute the ROBOT FEEDBACK if the CMU camera is used
	#endif

 /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *------------------------------------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
  
  if (user_display_mode == 0) /* User Mode is Off */
    
  { /* Check position of Port 1 Joystick */

  } /* (user_display_mode = 0) (User Mode is Off) */
  
  else  /* User Mode is On - displays data in OI 4-digit display*/
  {
    User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
  }   
  
} /* END Default_Routine(); */

/***********************gain*****************************************
This function is used to reduce the value of the input so various robot functions
have their speed reduced. */
unsigned char gain(int input, float reduction)
{
//printf("in gain function input is %d\r", input);
	if (input >= 127 - dead_zone && input <= 127 + dead_zone)		//if in dead zone
    	return 127;							//return nuetral
	//else return the reduced value
	else return((floor)(input - 127) * reduction + 127); 	//reduction formula
}

/************SET_DRIVE FUNCTIONS***********/
/*These functions are used to set the drive on the robot. There is one for
the left drive wheel and the right drive wheel.*/
void set_drive_L(int speed)
{
//printf("in set left drive function input is %d\r", speed);

	if(speed < 127)
		drive_L = ((speed - 127) * motor_bal + 127);//gain(speed, motor_bal);			//ensures we are not over 254.											//sets left drive to speed
	else drive_L = speed;						//set to highest value
}

/*******************/
void set_drive_R(int speed)
{
//printf("in set right drive function input is %d\r", speed);

	if(speed > 127)
		drive_R = ((speed - 127) * motor_bal + 127);	//ensure we are not over 254								//sets right drive to speed
	else drive_R = speed;					//set to highest speed
}
/***************   lift_operate     *******************
Parameters: int input	input value for the function.
Input: p3_x, set number, p3_sw_aux2	Range: 0-254, 0-254, 0 or 1 
Output: 0-255	
This function is used to assign PWM values to the lift. The operator
 uses a three position switch to decide which direction to move the lift. 
It is given a range of values to account for any errors from the switch.
 If the value is greater than 200, the lift will raise. If the value is 
less than 30, the lift will go down. Else the lift will remain in position. 
The lift incorporates two limit switches that are used to ensure that the 
lift does not overextend or under extend itself. The lift has two limit switches 
that denounce the height of the center goal and the side goals that are used in
 autonomous mode. If the lift is at one of these limits, the lift will stop and 
not go any further in that direction. The speed of the lift is determined by 
constants. The operator also has the power to reduce the speed of the lift by 
50%. This is achieved by holding the p3_sw_aux2 switch.
  *
******************************************************/ 
void lift_operate(unsigned char input)
{

	if(input > 200)	//is input greater than 127?
		{
		if(lift_high)	//are we already at the max bottom?
			lift = lift_speed_up;	//send the lift up
		else lift = 127;
		}
	else if(input < 30)	//is input less than 127?
		{
		if(lift_bottom)		//are we are max bottom?
			lift = lift_speed_down;	//move lift down
		else lift = 127;
		}
	else lift = 127;
}

/********************  switch_joystick  ***********************/
/* This function is used to change the type of drive the robot uses. The
operator can change between 1 and 2 stick drive by using p1_sw_top*/
int switch_joystick(void)
{
	static char already_shifted = 0;  //keeps track of whether or not
                                   //the trigger has been pressed. 1 = changed value
	static char current_gear = 0;			   //This keeps track of what gear the 
							  	   //robot is currently in. Starts in high
								   //gear.

	if (p1_sw_aux2 && p1_sw_trig & !already_shifted)
	{
		if (current_gear == 0)				   //if currently in 2 stick
			current_gear = 1;				   //change to 1 stick
    	else current_gear = 0;				   //the robot must be in 1 stick
	  					        		   //so change to 2 stick
 		already_shifted = 1;           //tells program that gear has shifted
	}

		if (!p1_sw_trig) 			//is the switch not pressed?
	  		already_shifted = 0;             //if so, reset already_shifted
  	
	return current_gear;	//return current_gear
}

/********************  gripper_control  ***********************
Parameters: None
Input: p3_sw_trig				Range: 0 or 1
Output: 0 or 1	
This function is used to operate the gripper mechanism, which grabs and
 manipulates the tetras. It is operated by using two pneumatic solenoids.
 It has a mechanical release to grab the tetra.  Dual solenoids will always
 be attempting to lock the gripper. The release will allow the lock to 
complete. When the operator presses the switch, it will release for a
 set amount of time. This amount of time is determined using a constant.
 After the time has elapsed, the gripper will set itself into its locked
 position, waiting for the mechanical release to go off again. */

int gripper_control(void)
{

	static char prev_p3_sw_trig = 0;


	if (p3_sw_trig == 1 && p3_sw_trig != prev_p3_sw_trig )
			{
			gripper_in = 0;
			gripper_extended = 1;
			}
	else if (p3_sw_trig == 0 && p3_sw_trig != prev_p3_sw_trig)
			{
			gripper_extended = 0;
			gripper_in = 1; 
			}	 

	if (p3_sw_trig == prev_p3_sw_trig)
	{
	gripper_extended = 0;
	gripper_in = 0;
	}

	prev_p3_sw_trig = p3_sw_trig;

}
/*******************SHOULDER_control*******************************/
/*Parameters: none
Input: p3_y, p3_sw_aux1			Range: 0-254, 0 or 1
Output: 0-255
This function inputs the necessary information then passes this value, called raw_data,
through this equation:
 ((raw_data * 235) / 100) + 200;
 The computed value is compared to the input value from the potentiometer on the robot. 
If we are not in the dead zone, which ranges 40, then the robot will move the shoulder
 up or down in order to cause the values to fall within the dead zone. This function 
also incorporates an emergency manual switch. This will allow p3_sw_top to toggle between 
use of the pot and pure manual. This is used when the pot is not working correctly.
 In both modes, the operator has the choice of slowing the arm to 50% power by pressing 
and holding p3_sw_aux1. The power reduction is achieved by use of the gain function.
*/

void shoulder_control(void)
{
	int input;						//holds input from the pot on the arm
	unsigned int translated = 0;	//translated value from the oi
	static char mode = 0;			//current mode, auto or manual
	static char mode_db = 0;		//debounce

	if (p3_sw_top && !mode_db)		//if p3_sw_top and not debonce
	{
	/*switch mode to the opposent setting and make the debounce 1*/
		if (mode)					
			mode = 0;
		else mode = 1;

	mode_db = 1;	
	}

	if(!p3_sw_top)		//if the top switch is not pressed
		mode_db = 0;	//set debounce to 0

	input = Get_Analog_Value(shoulder_pot);		//input value from the sensor on robot
	Pwm1_green = 0;		//single for mode of arm
	if(mode)
	{
		translated = ((2.35 * raw_data) + 200);

			if(translated <= input + shoulder_dead_zone && translated >= input - shoulder_dead_zone)
				shoulder = 127;
			else if(translated < input - shoulder_dead_zone)
					shoulder = shoulder_speed_down;
				else if(translated > input + shoulder_dead_zone)
					shoulder = shoulder_speed_up;
	}
	else 
	{
		if(raw_data > 200)
			shoulder = shoulder_speed_up;
		else if (raw_data < 50)
			shoulder = shoulder_speed_down;
		else shoulder = 127;

		Pwm1_green  = 1;

	}
}

/*********************compressor control****************************
Parameters: None
Input: TBD					Range: 0 or 1
Output: 0 or 1
The pressure sensor prevents that air compressor from over pressurizing 
the pneumatics system. The pressure sensor will return 0 when it has 
reached 105 psi. It will remain at 0 until the pressure has returned
 to 95 psi, were it will again become 0.
*/
void pressure_control(void)
{
	/*Turns pump on and off. Turns off at 115 PSI turns on at ??  */
 		if(pressure_sensor == 1) compressor = 1;
    	else compressor = 0;
}

/***************************otis_height_counter
This function is used to dermine the height of otis. */
void otis_height_counter(void)
{
#if 0
	static char last_val = 0;		//last value from lift 
	static unsigned char mag_count = 1;
	
	if(lift > 127)					//if the lift is going down
	{
		if(otis_counter != last_val)
			{				
			mag_count++;
			last_val = otis_counter;
			} 
	}

	if(lift < 127)
	{
		if(otis_counter != last_val)
			{
			mag_count--;
			last_val = otis_counter;
			}
	}
#endif
}


int acceleration( int drive_value, int last_drive_value, unsigned char RorL,unsigned char override)
{

static int R_limit_tick = 0;
static int L_limit_tick = 0;
static int R_new_drive_value = 127;
static int L_new_drive_value = 127;
static int R_old_speed = 127;
static int L_old_speed = 127;


if (RorL == 1)   // Left side
{
	if (override==0) // override is not enabled
	{
		if (((drive_value-last_drive_value) >= 10) || ((drive_value-last_drive_value) <= -10))
		{	
			L_limit_tick = 1;
			L_old_speed = L_new_drive_value;
		}
		if (L_limit_tick <acceleration_limit_max)
		{
			L_new_drive_value = ((((100*(drive_value-L_old_speed))/acceleration_limit_max)*L_limit_tick)/100)+L_old_speed;
			L_limit_tick = L_limit_tick++;
		}
		else L_new_drive_value = drive_value;
	}
	else // override enabled
	{
		L_new_drive_value = drive_value;
		L_limit_tick = 1;
		L_old_speed = L_new_drive_value;
	}		
	return L_new_drive_value;
}
else  // know that RorL == 2  Right side
{
	if (override==0) // override is not enabled
	{
		if (((drive_value-last_drive_value) >= 10) || ((drive_value-last_drive_value) <= -10))
		{	
			R_limit_tick = 1;
			R_old_speed = R_new_drive_value;
		}
	
		if (R_limit_tick <acceleration_limit_max)
		{
			R_new_drive_value = ((((100*(drive_value-R_old_speed))/acceleration_limit_max)*R_limit_tick)/100)+R_old_speed;
			R_limit_tick = R_limit_tick++;
		}
		else R_new_drive_value = drive_value;
	}
	else // override enabled
	{
		R_new_drive_value = drive_value;
		R_limit_tick = 1;
		R_old_speed = R_new_drive_value;
	}
	return R_new_drive_value;
}

}



/*
 * This routine activates our profile. This routine should be called
 * at robot power-up
 */
void activate_profile(void) {

    /* Copy the persistent profile into our volatile */
    
    memcpy((void *)&volatile_profile, (void *)&saved_profile,
		sizeof(volatile_profile));
}

/*
 * This routine saves our active profile.
 */
void store_profile(void) {

    memcpy((void *)&saved_profile, (void *)&volatile_profile,
		sizeof(volatile_profile));
    
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
