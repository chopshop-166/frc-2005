/******************************************************************************************************
FILE NAME: user_camera.c

MODIFICATION HISTORY:
		11/28/04 First Version by Anthony Rowe

DESCRIPTION:
	This file contains a set of function that communicate with the CMUcam.  It also requires a 
	modification to the PicSerialdrv.c file that is standard with the FRC distribution. This 
	modification allows the uart interrupt routine to buffer the CMUcam packets.

LIMITATION:
  Sometimes, after pressing the reset button, the camera will not come up correctly.  If you 
  power cycle the camera when it is stuck - it will work correctly.  It seems to always work
  from an RC power up.

******************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include "user_Serialdrv.h"
#include "user_camera.h"
#include "user_routines.h"
#include "ifi_default.h"

//
// MHS_DEBUG: 0 = off, 1 = on
//
#define MHS_DEBUG 0

//
// Our working copy of our robot's profile
//
extern struct mhs166_profile volatile_profile;

#if _USE_CMU_CAMERA
extern unsigned int parse_mode;
extern volatile unsigned int cam_index_ptr;
extern volatile unsigned int data_rdy;
unsigned char cam_uart_buffer[64]; 

/**********************************************************************
camera_getACK

This function sends the final '\r' character and waits to see if the
camera returns an ACK or a NCK. This only works when parse_mode = 0 and
should be used for control commands to the camera, and not for tracking
commands.

    Return: 1 ACK
			0 no ACK, maybe a NCK or a timeout
**********************************************************************/
int camera_getACK(void)
{
  int cnt,i;
  Serial_Write(CAMERA_PORT,"\r",1);
  reset_rx_buffer();
  if( wait_for_data()==0) return 0;
  if(cam_uart_buffer[0]==65) 
  {
    reset_rx_buffer();
    return 1;
  }
  if(cam_uart_buffer[0]==84) 
  {
    return 2;
  }
  reset_rx_buffer();
  return 0;
  
}

/**********************************************************************
camera_const_cmd

This function is used to send constant string commands to the camera.
This means you have to send something in quotes.  If you are sending from
a buffer, you must use camera_buffer_cmd.  See camera_find_color for examples.

		rom const char *cmd_str - This is a constant string to be sent to the camera
								- It does not require the '\r' at the end

		Return: 0 - no ACK, or timed out
				1 - ACK, the command is good
**********************************************************************/
int camera_const_cmd(rom const char *cmd_str)
{
  int i;
  int len;
  for (i=0; i<MAX_BUF_SIZE; i++ )
  {
    if (cmd_str[i]=='\r' || cmd_str[i]==0 )
    {
      len = i;
      //	cmd_str[i]=0;
      break;
    }
  }
  Serial_Write(CAMERA_PORT,cmd_str,len);
  return camera_getACK();
}

/**********************************************************************
camera_buffer_cmd

This function is used to send a string from memory to the camera. It is
camera_const_cmd's twin, but is used for arrays that you make yourself.

		unsigned char *cmd_str - This is a string to be sent to the camera
								- It does not require the '\r' at the end

		Return: 0 - no ACK, or timed out
				1 - ACK, the command is good
**********************************************************************/
int camera_buffer_cmd(unsigned char *cmd_str)
{
  int i;
  int len;
  for(i=0; i<MAX_BUF_SIZE; i++ )
  {
    if(cmd_str[i]=='\r' || cmd_str[i]==0 )
    {
      len=i;
      break;
    }
  }
  Serial_Write_Bufr(CAMERA_PORT,cmd_str,len);
  return camera_getACK();
}

/**********************************************************************
reset_rx_buffer

This function will reset the pointer index and the data_rdy flag that
is used by the interrupt in PicSerialdrv.c
This is called right before the interrupt routine is supposed to look
for new data.
**********************************************************************/
void reset_rx_buffer(void)
{
	cam_index_ptr=0;
	data_rdy=0;
}

/**********************************************************************
wait_for_data

This function checks to see if the serial buffer for the camera has new
data ready.  Depending on what mode it is in, it may wait for a full T
packet, or just a '\r' terminated line. It only waits a short period of
time, and then it returns 0 if no data is ready.  This short period of
time is just long enough to catch ACKs from messages.

		Return: 0 - no new data
				1 - new packet is ready
**********************************************************************/
int wait_for_data(void)
{
  int i;

  // This loop below is a counter that gives just enough time to catch
  // an ACK from a normal command.  
  for(i=0; i<20000; i++ )     
	  if(data_rdy!=0 ) return 1;
	
  return 0;}

/**********************************************************************
camera_reset

This function resets the camera.  This will clear all register.  It then
checks to see if the camera is responding.

		Return: 0 - no ACK, and reset may not have happened
				1 - reset occured, and all is well
**********************************************************************/
int camera_reset(void)
{
  int i;
  parse_mode=0;
  Serial_Write(CAMERA_PORT,"rs\r",3);
  i=camera_getACK();
  return camera_getACK();
}


//
// This routine initializes the camera. It first resets the
// camera (so that it is in a known state). After that, we send
// a configuration commands to the camera so that it is setup
// for the specific use we intend to make of it.
//
void initialize_camera_window(void) {

#if MHS_DEBUG
   printf("initialize_camera_window()\r");
#endif
   camera_reset();
   if (!camera_const_cmd("BM 1")) { /* Keep frame, once captured */
#if MHS_DEBUG
	  printf("BM 1 command failed\r");
#endif
	  return;
   }
   if (!camera_const_cmd("PM 1")) {   /* Enable poll mode */
#if MHS_DEBUG
	  printf("PM 1 command failed\r");
#endif
	  return;
   }
   if (!camera_const_cmd("RM 1")) {   /* Enable raw mode */
#if MHS_DEBUG
	  printf("RM 1 command failed\r");
#endif
	  return;
   }
   if (!camera_const_cmd("NF 6")) {   /* Noise filter to level 6 */
#if MHS_DEBUG
	  printf("NF 6 command failed\r");
#endif
	  return;
   }
   if (!camera_const_cmd("CR 0 32")) {   /* Set AGC to MID level */
#if MHS_DEBUG
	  printf("CR 0 32 command failed\r");
#endif
	  return;
   }
   if (!camera_const_cmd("CR 18 0")) {   /* Set into yCrCb instead of RGB  */
#if MHS_DEBUG
	  printf("CR 18 0 command failed\r");
#endif
	  return;
   }
   if (!camera_const_cmd("SO 0 1")) {   /* Servo 0 setup  */
#if MHS_DEBUG
	  printf("SO 0 1 command failed\r");
#endif
	  return;
   }
#if MHS_DEBUG
   if (!camera_const_cmd("GW")) {   /* Get window size */
	  printf("GW command failed\r");
	  return;
   }
   if (!camera_const_cmd("VW 1 1 150 150")) {   /* Get window size */
	  printf("VW 1 1 150 150 command failed\r");
	  return;
   }
   dump_uart_buffer();
#endif
   wait_for_data();
   reset_rx_buffer();
   return;
}

/* Grab camera window for later processing */
void grab_camera_window(void) {

#if MHS_DEBUG
   printf("grab_camera_window()\r");
#endif
   if (!camera_const_cmd("RF")) { /* Read a new frame */
#if MHS_DEBUG
	  printf("RF command failed\r");
#endif
	  return;
   }
   wait_for_data();
   reset_rx_buffer();
   return;
}

/*
 * This routine can be used for debug. It dumps the contents 
 * of the UART buffer used in communication with the camera.
 * It displays each byte in the buffer, up to the count of
 * bytes in it as per the "cam_index_ptr".
 */
void dump_uart_buffer(void) {

    char i;

    /* Display contents of UART buffer */
    printf("UART buffer: ");
    for (i=0; i<cam_index_ptr; i++)
        printf("%d ", (int)cam_uart_buffer[i]);
    printf("\r");
}

#if 0 // psh: not used at this time
//
// This routine attempts to determine of the color just captured
// by the camera falls within range for Blue in RGB mode
//
int is_blue(void) {

    if ((cam_uart_buffer[2] <= 150) &&
	    (cam_uart_buffer[3] <= 150) &&
        (cam_uart_buffer[4] >= 135) &&
        (cam_uart_buffer[4] <= 255)) {
#if MHS_DEBUG
        printf(" BLUE ");
#endif
	    return (BLUE);
    } else {
		return (UNKNOWN_COLOR);
	}
}
#endif // psh: end of exclusion

#if 0 // psh: not used at this time
//
// This routine attempts to determine of the color just captured
// by the camera falls within range for Yellow in RGB mode
//
int is_yellow(void) {

    if ((cam_uart_buffer[2] <= 254) &&
        (cam_uart_buffer[2] >= 100) &&
	    (cam_uart_buffer[3] <= 150) &&
        (cam_uart_buffer[3] >= 75) &&
        (cam_uart_buffer[4] <= 20)) {
#if MHS_DEBUG
        printf(" YELLOW ");
#endif
		return (YELLOW);
	} else {
		return (UNKNOWN_COLOR);
	}	
}
#endif // psh: end of exclusion

//
// This routine attempts to determine of the color just captured
// by the camera falls within range for Green in yCrCb mode
//
int is_green(void) {

    if ((cam_uart_buffer[2] <= 120) &&
        (cam_uart_buffer[2] >= 85) &&
	    (cam_uart_buffer[4] >= 80) &&
        (cam_uart_buffer[4] <= 134)) {
        printf(" GREEN ");
        return (GREEN);
    } else {
 	   return (UNKNOWN_COLOR);
    }
}

#if 0 // psh: not used at this time
//
// This routine attempts to determine of the color just captured
// by the camera falls within range for Red in yCrCb mode
//
int is_red(void) {

    if ((cam_uart_buffer[2] >= 190) &&
        (cam_uart_buffer[4] <= 40)) {
#if MHS_DEBUG
        printf(" RED ");
#endif
        return (RED);
    } else {
        return (UNKNOWN_COLOR);
    }
}
#endif // psh: end of exclusion

/*
 * This routine makes up for the lack of a useful feature in the
 * sprintf() function. It takes an input integer (single byte),
 * unsigned, and converts it into ASCII and appends this to the output
 * buffer supplied. It also adds a blank space after the number.
 *
 * This function can be used to build up variable data command
 * strings for the camera, such as when we compute the viewport
 * we are going to use with the camera.
 */
void append_int(unsigned char *buf, unsigned char val) {

    int outpos = 0; /* Output position for next char */
    int outval; /* Next digit to output */
    int hs; /* Hundreds */

    /* Find place for next output byte */
    while (buf[outpos])
 	    outpos++;

    /* Put 100s into buffer */
    if (hs = val / 100)
	    buf[outpos++] = hs + '0';

    /* Put 10s into buffer */
    if ((outval = (val - (100 * hs)) / 10) || (hs))
	    buf[outpos++] = outval + '0';

    /* Put last digit into buffer */
    buf[outpos++] = (val % 10) + '0';

    /* Add a space and then NULL terminate */
    buf[outpos++] = ' ';
    buf[outpos] = 0;

}

/* These parameters describe the cells of our new camera grid */
unsigned char cam_x_step;
unsigned char cam_x_last;
unsigned char cam_x_count;
unsigned char cam_y_step;
unsigned char cam_y_last;
unsigned char cam_y_count;
unsigned int cam_total_slots;
unsigned int cam_next_slot;

//
// This routine allows you to configure the camera's view window into
// a matrix of "cells". When you call this routine, you specify how
// many cells you want in the X and Y dimension. The routine then
// converts the geometry of the camera into a set of cells. These
// cells can be subsequently examined.
//
void set_camera_grid(unsigned int x_count, unsigned int y_count) {

    /* Determine X step and X residual on last */
#if MHS_DEBUG
 	printf("set_camera_grid()\r");
#endif

    /* Determine X step and X residual on last cell */
    cam_x_step = 159 / x_count;
    cam_x_last = (159 - (cam_x_step * x_count));
    cam_x_count = x_count;

    /* Determine Y step and Y residual on last cell */
    cam_y_step = 150 / y_count;
    cam_y_last = (150 - (cam_y_step * y_count));
    cam_y_count = y_count;

    /* Initialize information about current grid */
    cam_total_slots = x_count * y_count;
#if MHS_DEBUG
    printf("x_count = %d, y_count = %d\r", x_count, y_count);
    printf("total slots %d\r", cam_total_slots);
#endif
    cam_next_slot = 0;
}

//
// This routine allows you to examine a specific cell in the grid. It
// returns a value as follows:
//
//    UNKNOWN_COLOR (0)
//    YELLOW				-- currently not enabled
//    GREEN
//    WHITE					-- currently not enabled
//    RED					-- currently not enabled
//    BLUE					-- currently not enabled
//
int examine_grid_position(unsigned char x_pos, unsigned char y_pos, unsigned char show) {

    unsigned char outbuf[(4*4)+3+1] = {'V', 'W', ' ', 0};
    unsigned char npos; /* Coordinate position */
    unsigned int looper; /* Wait loop for data */
	unsigned char color; /* Color seen */

#if MHS_DEBUG
	printf("Trying [%d,%d]\r", x_pos, y_pos);
#endif

    /* Insert upper coordinate (Cartesian) */
    append_int(outbuf, (x_pos * cam_x_step) + 1);
    append_int(outbuf, (y_pos * cam_y_step) + 1);

    /* Insert lower coordinate (Cartesian) */
    npos = (x_pos + 1) * cam_x_step;
    if (x_pos == (cam_x_count - 1)) npos += cam_x_last;
    append_int(outbuf, npos);
    npos = (y_pos + 1) * cam_y_step;
    if (y_pos == (cam_y_count - 1)) npos += cam_y_last;
    append_int(outbuf, npos);

#if MHS_DEBUG
 	printf("Trying %s .. ", &outbuf[0]);
#endif
    if (!camera_buffer_cmd(&outbuf[0])) {
#if MHS_DEBUG
	    printf("%s command failed\r", &outbuf[0]);
#endif
	    return (0);
   }

    /* Get mean colors for the window */
    if (!camera_const_cmd("GM")) {
#if MHS_DEBUG
 	    printf("GM command failed\r");
#endif
	    return (0);
   }

	/* Wait for a response from the camera. We would like to see */
	/* up to 5 characters, since this is all we need to check the color */
	color = UNKNOWN_COLOR;
    for (looper=0; looper<6000; looper++)
        if (cam_index_ptr >= 5) {

			/* Is this green? */
			color = is_green();

			/* Display information, if so requested */
			if (show) {
	       		printf(" [%d,%d] ", x_pos, y_pos);
			    dump_uart_buffer();
		 	    printf("\r");
			}
		    break;
		}

	/* Back to caller with whatever color we found */
	return (color);
}

//
// This routine allows you to sequentially examine all positions in the
// grid. The routine keeps a eye on if we have received something from
// the master processor. If not, it keeps going. This allows us to cover
// more than one cell per call.
//
extern packed_struct statusflag;
int examine_next_gp(void) {

    /* Enter loop to process as many cells as possible */
    while (!statusflag.NEW_SPI_DATA) {
    
        /* Advance to next slot, break out here if we are done */
        if (cam_next_slot >= cam_total_slots) {
#if MHS_DEBUG
            printf("We're done with our %d slots\r", cam_total_slots);
#endif
            return (1); /* We're done */
	    }

        /* Sweep X axis accross and then advance to next Y row */
        examine_grid_position(cam_next_slot / cam_y_count, cam_next_slot % cam_y_count, 1);
        cam_next_slot++;
	}
 	return (0);
}

//
// This routine sets the camera horizontal servo. The input is 
// in the format defined for the camera sevo, with 128 being
// dead center.
//
// NOTE! Changing the camera's position takes time. You must
// therefore wait after telling the servo to move before you
// ask the camera to grab a picture. If not, you will end up
// grabbing a frame while the camera is moving.
//
void set_camera_position(unsigned char camera_pos) {

    unsigned char outbuf[2+1+1+1+3+1+1] = {'S', 'V', ' ', '0', ' ', 0};

#if MHS_DEBUG
    printf("set_camera_position()\r");
#endif

    /* Insert upper coordinate (Cartesian) */
    append_int(outbuf, camera_pos);

	/* Tell servo. */
    if (!camera_buffer_cmd(&outbuf[0])) {
#if MHS_DEBUG
  	    printf("%s command failed\r", &outbuf[0]);
#endif
	    return;
    }
    wait_for_data();
}


// New camera code

//
// Convenience definitions for setting up tetra table
//
#define CAMERA_ANGLE (1)
#define INITIAL_POSITION (2)
#define END_OF_SCAN (3)
#define CSHIFT ((unsigned int)14)
#define CPOS(X,Y,TETRA) (((unsigned int)TETRA<<(unsigned int)10) | ((unsigned int)Y<<(unsigned int)5) | (unsigned int) X)
#define CANGLE(ANGLE) (((unsigned int)CAMERA_ANGLE<<CSHIFT) | (unsigned int)ANGLE)
#define IPOS(POS) ( ( (unsigned int)INITIAL_POSITION << CSHIFT ) | (unsigned int)POS )
#define CEND ((unsigned int)END_OF_SCAN<<CSHIFT)
#define CGRAB CANGLE(0)

//
// Virtual tetra positions. These are defined as tetra 0-4. We
// also have only two virtual start positions 0-1.
//
static rom const unsigned int vtetras[] =
{
	IPOS(0),
    	CANGLE(110),
		CGRAB,
			CPOS(18,20,1),
			CPOS(18,21,1),
			CPOS(18,22,1),
			CPOS(19,21,1),
			CPOS(19,22,1),
			CPOS(20,21,1),
			CPOS(20,22,1),
			CPOS(14,24,0),
			CPOS(15,24,0),
			CPOS(5,19,2),
			CPOS(5,20,2),
			CPOS(6,19,2),
			CPOS(6,20,2),
			CPOS(6,21,2),
			CPOS(7,20,2),
		CANGLE(128),
		CGRAB,
			CPOS(14,20,2),
			CPOS(14,21,2),
			CPOS(15,20,2),
			CPOS(15,21,2),
			CPOS(16,20,2),
		CANGLE(146),
		CGRAB,
			CPOS(15,21,3),
			CPOS(15,22,3),
			CPOS(16,21,3),
			CPOS(16,22,3),
			CPOS(10,16,4),
			CPOS(10,17,4),
			CPOS(10,18,4),
			CPOS(11,16,4),
			CPOS(11,17,4),
			CPOS(11,18,4),
			CPOS(11,16,4),
			CPOS(12,16,4),
			CPOS(12,17,4),
			CPOS(12,18,4),
	CEND
};

//
// This table converts the virtual tetra numbers to an actual tetra
// number. There are 6 translation vectors, each one representing one
// starting position in the field.
//
static unsigned char *atetra[6] =
{
	"\7\2\3\6\1",				// Blue Left
	"\3\6\1\4",					// Blue center
	"\6\1\4\5\0",				// Blue Right
	"\5\0\4\6\1",				// Red Left
	"\4\6\1\3",					// Red Center
	"\6\1\3\7\3"				// Red Right
};

//
// Persistent probe state accross calls
//
typedef enum {PRO_INIT, PRO_WAIT, PRO_POS, PRO_SCAN, PRO_SUM, PRO_DONE} PRO_STATE_t;
PRO_STATE_t pro_state = PRO_INIT;
PRO_STATE_t nxt_state = PRO_INIT;

//
// Actual results after probe
//
unsigned char avec[8];      // Actual tetra vector

//
// This routine tries to determine which tetras we can see based on where
// we are starting. This routine will have to be called repeatedly. It returns
// a nonzero value when the scanning is complete.
//
int probe_pos(unsigned char spos, unsigned int apos, unsigned char *resvec) {

	static int plooper;          // Keeps track of where we are in vtetra array
    unsigned int npos;           // Contents of current position in vtetra array
    int tetras_found;            // Tetras found in this scan
	static int sleep_count;      // Times we need to sleep
	
	// Dispatch based on our current probe state
	switch (pro_state) {


	// Initialize scan
	case PRO_INIT:

		// Initialize our loop variable
		plooper = 0;

		// Get camera initialized
#if MHS_DEBUG
		printf("INIT\r");
#endif
		initialize_camera_window();
		set_camera_grid(26, 26);

		// Clean up actual tetra vector; overload "npos" for loop variable
		for (npos=0; npos<7; npos++) {
			avec[npos] = 0;
			resvec[npos] = 0;
		}

	    // Find position in table representing our starting position
    	while (1) {

			// If we are at the end of the table, back to caller
			if ((npos = vtetras[plooper++]) == CEND) {
				pro_state = PRO_DONE;
#if MHS_DEBUG
				printf("CEND\r");
#endif
				return (1);
			}

			// Is this is the position we're looking for, get out
			if (npos = IPOS(spos))
				break;
		}
		sleep_count = 0;
		pro_state = PRO_WAIT;
		nxt_state = PRO_SCAN;

		// Fall thru to scan

	// Stall for a while to let camera adjust
	case PRO_WAIT:
		{
		int dummy1, dummy2;

		if (!sleep_count) {
			sleep_count++;
			examine_grid_position(25, 25, 0);
		}

		// Stall some
		for (dummy1=0; dummy1<9000; dummy1++)
			dummy2 = dummy1;

		// Change state if we've stalled enough
		if (++sleep_count == 3) {
			pro_state = nxt_state;
			sleep_count = 0;
		}

		// Back to caller
		return (0);
		}

	// Scan for matches
	case PRO_SCAN:

		// Process the table and examine the positions
#if MHS_DEBUG
		printf("SCAN\r");
#endif
	    while (!statusflag.NEW_SPI_DATA) {

			// If we're at the end of the table, get out now
			if ((npos = vtetras[plooper++]) == CEND) {
				pro_state = PRO_SUM;
				break;
			}
	
			// If we've reached a different "position" in the vtetra table, back out
			if (((npos & (IPOS(0))) == (IPOS(0))) &&
			 	(npos != IPOS(spos))) {
				pro_state = PRO_SUM;
				break;
			}

			// Change camera angle or grab if so requested
			if ((npos & CANGLE(0)) == CANGLE(0)) {
				if (npos == CGRAB) {
					grab_camera_window();
				} else {
					set_camera_position(npos & 255);
				}
				pro_state = PRO_WAIT;
				nxt_state = PRO_SCAN;
				return (0);
			}

			// Sniff this cell. Do we see anything here?
			if (examine_grid_position(npos & 31, ((npos >> 5) & 31), MHS_DEBUG)) {
				unsigned int tetra = (npos >> 10) & 15;

				// Increment count we've seen this tetra
				avec[tetra]++;
			}
		}

		// Back to caller if there is a packet from master processor
		if (pro_state == PRO_SCAN)
			return (0);

		// Fall thru to summing things up

	// Sum things up
	case PRO_SUM:

		// Convert each tetra found into an actual tetra number
#if MHS_DEBUG
		printf("SUM\r");
#endif
		for (plooper=0; plooper<7; plooper++)
#if MHS_DEBUG
			printf("[%d] = %d\r", plooper, avec[plooper]);
#endif
			if (avec[plooper]) {
				resvec[atetra[apos][plooper]] = avec[plooper];
#if MHS_DEBUG
				printf("Found tetra #%d %d times\r", atetra[apos][plooper], avec[plooper]);
#endif
			}
		
		// We're done
		pro_state = PRO_DONE;

		// Fall thru

	// We're done
	case PRO_DONE:
	default:
		return (1);
	}
}

#endif


