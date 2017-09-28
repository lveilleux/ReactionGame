/*********************************************************************/
/* Reaction Timing Game                                              */
/*This exercise explores the use of the touch sensor (TSI) built     */
/* into the KL46 board. The game uses the UART0, PIT, and TSI        */
/* to run games of 10 rounds testing the users reaction time         */
/* from when the green LED lights, to when they touch the sensor     */
/* Name:  Luke Veilleux                                              */
/* Date:  November 14, 2016                                          */
/* Class:  CMPE 250                                                  */
/*********************************************************************/
typedef int Int32;
typedef short int Int16;
typedef char Int8;
typedef unsigned int UInt32;
typedef unsigned short int UInt16;
typedef unsigned char UInt8;

#define SLIDER_LENGTH 40
#define TOTAL_ELECTRODE 2

#define _TSI0 0
#define TSI1 1
#define TSI2 2
#define TSI3 3
#define TSI4 4
#define TSI5 5
#define TSI6 6
#define TSI7 7
#define TSI8 8
#define TSI9 9
#define TSI10 10
#define TSI11 11
#define TSI12 12
#define TSI13 13
#define TSI14 14
#define TSI15 15

/* Chose the correct TSI channel for the electrode number */
#ifdef FRDM_REVA
#define ELECTRODE0   TSI2
#define ELECTRODE1   TSI3
#else
#define ELECTRODE0   TSI9
#define ELECTRODE1   TSI10
#endif

#define ELECTRODE2   _TSI0
#define ELECTRODE3   TSI1
#define ELECTRODE4   TSI2
#define ELECTRODE5   TSI3
#define ELECTRODE6   TSI4
#define ELECTRODE7   TSI5
#define ELECTRODE8   TSI6
#define ELECTRODE9   TSI7
#define ELECTRODE10  TSI8
#define ELECTRODE11  TSI11
#define ELECTRODE12  TSI12
#define ELECTRODE13  TSI13
#define ELECTRODE14  TSI14
#define ELECTRODE15  TSI15

extern UInt8 total_electrode;
extern UInt8 elec_array[16];
extern UInt16 gu16TSICount[16];
extern UInt16 gu16Baseline[16];
extern UInt16 gu16Delta[16];
extern UInt8 ongoing_elec;
extern UInt8 AbsolutePercentegePosition;
extern UInt8 AbsoluteDistancePosition;
extern UInt8 end_flag;


extern char RunStopWatch;
extern UInt16 Count;
char earlyTouch;
char NotLookingForTS;

/* assembly language subroutines */
char GetChar (void);
void Init_UART0_IRQ (void);
void PutChar (char Character);
void PutNumHex (UInt32);
void PutNumU (UInt32);
void PutStringSB (char String[], int StringBufferCapacity);
void Init_GPIO_LED (void);
void Turn_On_LED (UInt8);
void Turn_Off_LED (UInt8);
void Init_PIT_IRQ (void);

/* actual C language functions */
void Init_TSI(void);
void TSI0_IRQHandler(void);
void self_calibration(void);
void TSI_slider_read(void);
void change_electrode(void);

UInt16 Play_Round(int, int);
