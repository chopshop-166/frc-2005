#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "user_Serialdrv.h"

void User_Autonomous_Code_2(void)
{
static unsigned char code_loops = 0;
while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */

        /* Add your own autonomous code here. */
        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
       
	  if (code_loops < 10)
          {
           set_drive_L(0);
           set_drive_R(0);
          }  
      
      else
         { 
          set_drive_L(127);
          set_drive_R(127);
         }
      code_loops++;
//Add in the sperate autonomous files
        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}
