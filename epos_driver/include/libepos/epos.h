/*! \file epos.h
    
  header file for libEPOS functions

  mh, july 2006

*/

#ifndef _EPOS_H
#define _EPOS_H

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>  /* int types with given size */
#include <math.h>

/* added oct06 for openTCPEPOS() */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define E_NOERR         0x00000000   ///< Error code: no error
#define E_ONOTEX        0x06020000   ///< Error code: object does not exist
#define E_SUBINEX       0x06090011   ///< Error code: subindex does not exist
#define E_OUTMEM        0x05040005   ///< Error code: out of memory
#define E_NOACCES       0x06010000   ///< Error code: Unsupported access to an object
#define E_WRITEONLY     0x06010001   ///< Error code: Attempt to read a write-only object
#define E_READONLY      0x06010002   ///< Error code: Attempt to write a read-only object
#define E_PARAMINCOMP   0x06040043   ///< Error code: general parameter incompatibility 
#define E_INTINCOMP     0x06040047   ///< Error code: general internal incompatibility in the device 
#define E_HWERR         0x06060000   ///< Error code: access failed due to an hardware error
#define E_PRAGNEX       0x06090030   ///< Error code: value range of parameter exeeded
#define E_PARHIGH       0x06090031   ///< Error code: value of parameter written is too high
#define E_PARLOW        0x06090032   ///< Error code: value of parameter written is too low
#define E_PARREL        0x06090036   ///< Error code: maximum value is less than minimum value
#define E_NMTSTATE      0x0f00ffc0   ///< Error code: wrong NMT state
#define E_RS232         0x0f00ffbf   ///< Error code: rs232 command illegeal
#define E_PASSWD        0x0f00ffbe   ///< Error code: password incorrect
#define E_NSERV         0x0f00ffbc   ///< Error code: device not in service mode
#define E_NODEID        0x0f00fb9    ///< Error code: error in Node-ID

//#include "eposlib.h" -> moved to epos.c

/* all EPOS data exchange is based on 16bit words, but other types are
   also used...*/
typedef unsigned long DWORD ; ///< \brief 32bit type for EPOS data exchange
typedef unsigned short WORD ; ///< \brief 16bit type for EPOS data exchange
#ifndef CPP
typedef char BYTE ; ///< \brief 8bit type for EPOS data exchange
#endif

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      5
/*! \brief sleep TRYSLEEP usec between read() from EPOS, if no data available */
//#define TRYSLEEP  (unsigned int)1e5
#define TRYSLEEP  (unsigned int)4000





/* globals, defined in epos.c */

/*!serial port file descriptor*/
extern int sp; 
/*! EPOS global error status */
extern DWORD E_error; 

/* all high-level functions return <0 in case of error */

/*! open the connection to EPOS */
int openEPOS(char *device);
/*! open the connection to EPOS via RS232-over-TCP/IP (LSW special) */
int openTCPEPOS(char *ip, short unsigned port);
/*! close the connection to EPOS */
int closeEPOS();
/*! check if the connection to EPOS is alive */
int checkEPOS();


/*! \brief check global variable E_error for EPOS error code */
int checkEPOSerror();

/*! \brief check EPOS status, return state according to firmware spec 8.1.1 */
int checkEPOSstate();
/*! \brief pretty-print EPOS state */
int printEPOSstate();
/*! \brief change EPOS state   ==> firmware spec 8.1.3 */
int changeEPOSstate(int state);



/*! \brief example from EPOS com. guide: ask EPOS for software version 

firmware spec 14.1.33
** returns software version as HEX **

*/
int readSWversion();


/*! \brief ask for device name,  
   device name is placed in 'name' (string must be big enough, NO CHECKING!!)
*/
int readDeviceName(char *name);


/*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
int readRS232timeout();

/*! \brief read digital input polarity mask */
int readDInputPolarity(WORD* w);

/*! \brief set home switch polarity -- firmware spec 14.1.47 */
int setHomePolarity(int pol);



/*! \brief read Statusword; 14.1.58 */
int readStatusword(WORD *eposStatus);
/*! \brief pretty-print Statusword */
int printEPOSstatusword(WORD statusword);



/*! \brief read EPOS control word (firmware spec 14.1.57) */
int readControlword(WORD *w);
/*! \brief pretty-print Controlword */
int printEPOScontrolword(WORD controlword);


/*! \brief set EPOS mode of operation -- 14.1.59 */
int setOpMode(int OpMode);

/*! \brief read and returns  EPOS mode of operation -- 14.1.60 
here, RETURN(0) MEANS ERROR! 
'-1' is a valid OpMode, but 0 is not!
*/
int readOpMode();


/*! \brief read actual position; 14.1.61 */
int readDemandPosition(long *val);
/*! \brief read actual position; 14.1.62 */
int readActualPosition(long *val);

/*! \brief read position window; 14.1.64 */
int readPositionWindow(unsigned long int *value);
/*! \brief write position window; 14.1.64 */
int writePositionWindow(unsigned long int value);

/*! \brief read actual position; 14.1.67 */
int readDemandVelocity(long *val);
/*! \brief read actual position; 14.1.68 */
int readActualVelocity(long *val);

/*! \brief read actual current; 14.1.69 */
int readActualCurrent(short *val);

/*! \brief read target position; 14.1.70 */
int readTargetPosition(long *val);


int set_speed_profile( unsigned int max_velocity,        // rpm
		       unsigned int acceleration = 1000, // rpm/s
		       unsigned int deceleration = 1000, // rpm/s
		       bool trapezoidal = true );
  
/*! \brief does a homing move. Give homing mode (see firmware 9.3) and start
   position */
int doHoming(int method, long int start);


/*! \brief set OpMode to ProfilePosition and make relative movement */
int moveRelative(long int steps);
/*! \brief set OpMode to ProfilePosition and make absolute movement */
int moveAbsolute(long int steps);
/*! \brief set OpMode to ProfileVelocity and make movement */
int moveVelocity(int32_t steps);

/*! \brief reads position, velocity and current and displays them in an
   endless loop. Returns after target position has been reached */
int monitorStatus();
/*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
int monitorHomingStatus();

/*! \brief waits for positoning to finish, argument is timeout in
   seconds. give timeout==0 to disable timeout */
int waitForTarget(unsigned int t);


#endif
