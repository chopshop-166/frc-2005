#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "user_Serialdrv.h"

typedef enum {START,
              FORWARD,
			  RAISE_OTIS,
              COVER_GOAL,
			  COMPLETE}step_flags;

step_flags step = START;
void User_Autonomous_Code_4(void)
{
//pressure_control();
unsigned char desired_count_L = 162,
              desired_count_R = 162,
              shoulder_out = 500;  //random

while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */

        /* Add your own autonomous code here. */
        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
      switch (step)
      {
       case START:
       /*anything that needs to be done before we move can be here*/      
            step = FORWARD;
            break;
       case FORWARD:
	         if(L_banner < desired_count_L && R_banner < desired_count_R)
             {
              set_drive_L(254);
              set_drive_R(254);
             }
             else step = RAISE_OTIS;
             break;
      case RAISE_OTIS:
           if (otis_counter < 370)
              lift_operate(254);
           else step = COVER_GOAL;
           break;    
      case COVER_GOAL:
           if (shoulder_pot < shoulder_out)
               shoulder = shoulder_speed_up;
           else step = COMPLETE;
           break; 
      case COMPLETE:
		   break;
     }

//Add in the sperate autonomous files
        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}
