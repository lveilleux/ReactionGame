/*********************************************************************/
/*This is a reaction game that uses of the touch sensor (TSI) built  */
/* into the KL46 board. The game uses the UART0, PIT, and TSI        */
/* to run games of 10 rounds testing the users reaction time         */
/* from when the green LED lights to when they touch the sensor.     */
/* A score is given based on the time taken to touch the sensor. A   */
/* 'random' amount of time occurs before the green LED turns back on */
/* at the start of each round. If a touch is registered before the   */
/* LED turns on, max points are awarded for that round.              */
/*                   Lowest score wins.                              */
/* Name:  Luke Veilleux                                              */
/* Date:  11/29/2016                                                 */
/* Class:  CMPE 250                                                  */
/* Section:  Section 1, Tuesday 2:00 - 3:50                          */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            November 14, 2016                                      */
/*********************************************************************/
#include "MKL46Z4.h"
#include "reactionGame_C.h"

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79)

/*-------------------------------------------------------------*/
/* PORTx_PCRn                                                  */
/*     -->31-25:(reserved):read-only:0                         */
/*    1-->   24:ISF=interrupt status flag (write 1 to clear)   */
/*     -->23-20:(reserved):read-only:0                         */
/* 0000-->19-16:IRQC=interrupt configuration (IRQ/DMA diabled) */
/*     -->15-11:(reserved):read-only:0                         */
/*  ???-->10- 8:MUX=pin mux control                            */
/*     -->    7:(reserved):read-only:0                         */
/*    ?-->    6:DSE=drive strengh enable                       */
/*     -->    5:(reserved):read-only:0                         */
/*    0-->    4:PFE=passive filter enable                      */
/*     -->    3:(reserved):read-only:0                         */
/*    0-->    2:SRE=slew rate enable                           */
/*    0-->    1:PE=pull enable                                 */
/*    x-->    0:PS=pull select                                 */
/* Port E */
#define PTE30_MUX_DAC0_OUT (0u << PORT_PCR_MUX_SHIFT)
#define SET_PTE30_DAC0_OUT (PORT_PCR_ISF_MASK | PTE30_MUX_DAC0_OUT)
#define PTE31_MUX_TPM0_CH4_OUT (3u << PORT_PCR_MUX_SHIFT)
#define SET_PTE31_TPM0_CH4_OUT (PORT_PCR_ISF_MASK | \
                                PTE31_MUX_TPM0_CH4_OUT | \
                                PORT_PCR_DSE_MASK)
/*-------------------------------------------------------*/
/* SIM_SOPT2                                             */
/*   -->31-28:(reserved):read-only:0                     */
/*   -->27-26:UART0SRC=UART0 clock source select         */
/* 01-->25-24:TPMSRC=TPM clock source select (MCGFLLCLK) */
/*   -->23-19:(reserved):read-only:0                     */
/*   -->   18:USBSRC=USB clock source select             */
/*   -->   17:(reserved):read-only:0                     */
/*  1-->   16:PLLFLLSEL=PLL/FLL clock select (MCGFLLCLK) */
/*   -->15- 8:(reserved):read-only:0                     */
/*   --> 7- 5:CLKOUTSEL=CLKOUT select                    */
/*   -->    4:RTCCLKOUTSEL=RTC clock out select          */
/*   --> 3- 0:(reserved):read-only:0                     */
#define SIM_SOPT2_TPMSRC_MCGPLLCLK (1u << SIM_SOPT2_TPMSRC_SHIFT)
#define SIM_SOPT2_TPM_MCGPLLCLK_DIV2 (SIM_SOPT2_TPMSRC_MCGPLLCLK | \
                                            SIM_SOPT2_PLLFLLSEL_MASK)

/*-------------------------------------------------------*/
/*------------------Touch Sensor (TSI)-------------------*/
/*-------------------------------------------------------*/
UInt8 total_electrode = TOTAL_ELECTRODE;
UInt8 elec_array[16]= {
    ELECTRODE0, ELECTRODE1, ELECTRODE2, ELECTRODE3, ELECTRODE4, ELECTRODE5,
    ELECTRODE6, ELECTRODE7, ELECTRODE8, ELECTRODE9, ELECTRODE10, ELECTRODE11,
    ELECTRODE12, ELECTRODE13, ELECTRODE14, ELECTRODE15
};
UInt16 gu16TSICount[16];
UInt16 gu16Baseline[16];
UInt16 gu16Delta[16];
UInt8 ongoing_elec;
UInt8 end_flag = 1;
UInt8 SliderPercentegePosition[2] = {0, 0};
UInt8 SliderDistancePosition[2] = {0, 0};
UInt8 AbsolutePercentegePosition = 0;
UInt8 AbsoluteDistancePosition = 0;


void TSI0_IRQHandler() {
/*********************************************************************/
/* Handles Interrupts that occur on the TSI0 module.                 */
/* Interrupt flag is cleared and position is printed to the screen	 */
/* Calls:  change_electrode, TSI_slider_read                         */
/*********************************************************************/
	__asm("CPSID   I");
  end_flag = 1;
  /* Clear End of Scan Flag */
  TSI0->GENCS |= TSI_GENCS_EOSF_MASK;
  change_electrode();
	TSI_slider_read();
	if (AbsolutePercentegePosition > 0) {
		if (NotLookingForTS == 0) {
			/* Touch sensor registered input before system wanted it */
			earlyTouch = 1;
		} else {
			RunStopWatch = 0;
			Turn_Off_LED(1);
		}
	}
	__asm("CPSIE   I");
} /* TSI0_IRQHandler */

void change_electrode() {
/*********************************************************************/
/* Changes the electrode in use in the TSI to the one not currently  */
/* in use. Swaps the current electrode.                           	 */
/* Calls:  None											                          			 */
/*********************************************************************/	
  int16_t u16temp_delta;
  /* Save Counts for current electrode */
  gu16TSICount[ongoing_elec] = (TSI0->DATA & TSI_DATA_TSICNT_MASK);
  /* Obtains Counts Delta from callibration reference */
  u16temp_delta = gu16TSICount[ongoing_elec] - gu16Baseline[ongoing_elec];
  if (u16temp_delta < 0) {
    gu16Delta[ongoing_elec] = 0;
  } else {
    gu16Delta[ongoing_elec] = u16temp_delta;
  }
  /* Change Electrode to scan */
  if (total_electrode > 1) {
		if ((total_electrode-1) > ongoing_elec) {
			ongoing_elec++;
    } else {
      ongoing_elec = 0;
    }
    TSI0->DATA = ((elec_array[ongoing_elec]<<TSI_DATA_TSICH_SHIFT));
    TSI0->DATA |= TSI_DATA_SWTS_MASK;
  }
} /* change_electrode */

void TSI_Calibration() {
/*********************************************************************/
/* Provides the code to calibrate the Touch Sensor on the KL46Z      */
/* Sensor is calibrated for use with its current capacity set to 0	 */
/* Calls:  None											                          			 */
/*********************************************************************/	
  unsigned char cnt;
  unsigned char trigger_backup;
  /* Obtains Counts Delta from callibration reference */
  TSI0->GENCS |= TSI_GENCS_EOSF_MASK;
  /* Disable TSI module */
  TSI0->GENCS &= ~TSI_GENCS_TSIEN_MASK;
  /* Back-up TSI Trigger mode from application */
  if (TSI0->GENCS & TSI_GENCS_STM_MASK) {
		trigger_backup = 1;
  } else {
		trigger_backup = 0;
	}
  /* Use SW trigger */
  TSI0->GENCS &= ~TSI_GENCS_STM_MASK;
  /* Enable TSI interrupts */
  TSI0->GENCS &= ~TSI_GENCS_TSIIEN_MASK;
  /* Enable TSI module */
  TSI0->GENCS |= TSI_GENCS_TSIEN_MASK;
  /* Get Counts when Electrode not pressed */
  for(cnt=0; cnt < total_electrode; cnt++) {
		TSI0->DATA = ((elec_array[cnt] << TSI_DATA_TSICH_SHIFT) );
    TSI0->DATA |= TSI_DATA_SWTS_MASK;
    while(!(TSI0->GENCS & TSI_GENCS_EOSF_MASK));
		TSI0->GENCS |= TSI_GENCS_EOSF_MASK;
    gu16Baseline[cnt] = (TSI0->DATA & TSI_DATA_TSICNT_MASK);
  }
  /* Disable TSI module */
  TSI0->GENCS &= ~TSI_GENCS_TSIEN_MASK;
  /* Enale TSI interrupt */
  TSI0->GENCS |= TSI_GENCS_TSIIEN_MASK;
  /* Restore trigger mode */
  if (trigger_backup) {
    TSI0->GENCS |= TSI_GENCS_STM_MASK;
	} else {
    TSI0->GENCS &= ~TSI_GENCS_STM_MASK;
	}
  /* Enable TSI module */
  TSI0->GENCS |= TSI_GENCS_TSIEN_MASK;
  TSI0->DATA = ((elec_array[0]<<TSI_DATA_TSICH_SHIFT));
  TSI0->DATA |= TSI_DATA_SWTS_MASK;
} /* TSI_Calibration */

void Init_TSI(void) {
/*********************************************************************/
/* Initialize the Touch Sensor (TSI) on the KL46Z to check for touch */
/* only when the program calls for it. No interrupts, default init	 */
/* Calls:  TSI_Calibration					                          			 */
/*********************************************************************/	
	/* Enable clock gating for TSI */
  SIM->SCGC5 |= SIM_SCGC5_TSI_MASK;
  /* Enable proper GPIO as TSI channels */
  SIM->SCGC5 |=  SIM_SCGC5_PORTB_MASK;
  /* PTB16 as TSI channel 9 */
  PORTB->PCR[16] =  PORT_PCR_MUX(0);
  /* PTB17 as TSI channel 10 */
  PORTB->PCR[17] =  PORT_PCR_MUX(0);
	/* Configure NVIC for TSI Interrupts */
  NVIC_ClearPendingIRQ(TSI0_IRQn);
  NVIC_EnableIRQ(TSI0_IRQn);
  NVIC_SetPriority(TSI0_IRQn, 0);
  /* Enable End of Scan Interrupt */
  TSI0->GENCS |= TSI_GENCS_ESOR_MASK |
			/* enable capacitance sensing non noise mode */
      TSI_GENCS_MODE(0) |
      /* reference oscillator charge/discharge current value is 2uA */
      TSI_GENCS_REFCHRG(4) |
      /* setting appropriate oscillators voltage rails */
      TSI_GENCS_DVOLT(0) |
      /* electrode oscillator charge/discharge current value is 64uA */
      TSI_GENCS_EXTCHRG(7) |
      /* electrode oscillator frequency is divided by 16 */
      TSI_GENCS_PS(4) |
      /* 12 scans per each electrode */
      TSI_GENCS_NSCN(11) |
      /* enable TSI interrupt */
      TSI_GENCS_TSIIEN_MASK |
      TSI_GENCS_STPE_MASK;
	/* Clear Flags */
  TSI0->GENCS |= TSI_GENCS_OUTRGF_MASK | TSI_GENCS_EOSF_MASK;
  /* Select Desired Channel(9) to Scan */
  TSI0->DATA |= TSI_DATA_TSICH(9);
  /* enable TSI */
  TSI0->GENCS |= TSI_GENCS_TSIEN_MASK;
  TSI_Calibration();
	return;
} /* Init_TSI */

void TSI_slider_read(void) {
/*********************************************************************/
/* Function to read in the current position of toouch on the TSI     */
/* Due to program constrants, presence of a touch is all that matters*/
/* Calls:  None											                          			 */
/*********************************************************************/	
	if (end_flag) {
		end_flag = 0;
			if ((gu16Delta[0] > 100) || (gu16Delta[1] > 100)) {
				SliderPercentegePosition[0] = (gu16Delta[0]*100)/(gu16Delta[0]+gu16Delta[1]);
        SliderPercentegePosition[1] = (gu16Delta[1]*100)/(gu16Delta[0]+gu16Delta[1]);
        SliderDistancePosition[0] = (SliderPercentegePosition[0]* SLIDER_LENGTH)/100;
        SliderDistancePosition[1] = (SliderPercentegePosition[1]* SLIDER_LENGTH)/100;
        AbsolutePercentegePosition = ((100 - SliderPercentegePosition[0]) + SliderPercentegePosition[1])/2;
        AbsoluteDistancePosition = ((SLIDER_LENGTH - SliderDistancePosition[0]) + SliderDistancePosition[1])/2;
      } else {
        SliderPercentegePosition[0] = 0;
        SliderPercentegePosition[1] = 0;
        SliderDistancePosition[0] = 0;
        SliderDistancePosition[1] = 0;
        AbsolutePercentegePosition = 0;
        AbsoluteDistancePosition = 0;
     }
  }
}

UInt16 Play_Round(int currentRound, int waitTime) {
/*********************************************************************/
/* Runs a round of the game. Waits a random amount of time, before   */
/* turning on the green LED, reaction time is recorded and the round */
/* score is calculated and returned                                  */
/* Calls:  TSI_Calibration					                          			 */
/*********************************************************************/	
	char wrongTouch[] = "\r\n  Wait for the green light to touch the sensor. Forfeiting round.";
	char prompt2[] = "\r\nTouch the touch sensor when the green light turns on.";
	UInt16 currentScore;
	UInt16 *count = &Count;
	char *runStopWatch = &RunStopWatch;
	/* Random wait time before touch round starts */
			*runStopWatch = 1;
			while(*count < waitTime) {
				if (earlyTouch != 0) {
					break;
				}
			}
			*runStopWatch = 0;
			*count = 0;
			/* Check for early touch input */
			if( earlyTouch != 0) {
				PutStringSB( wrongTouch, sizeof( wrongTouch ) );
				Turn_On_LED(0); /* Turn on red LED */
				*count = 500;
				earlyTouch = 0;
			} else { 
				Turn_On_LED(1); /* Turn on green LED */
				*runStopWatch = 1;
				NotLookingForTS = 1; /* Tell TSI we want input */
				while( AbsolutePercentegePosition <= 0) {
					if( *count >= 500 ) {
						/* Timeout */
						Turn_On_LED(0); /* Turn on red LED */
						*count = 500;
						PutStringSB( prompt2, sizeof( prompt2 ) );
						break;
					}
				}
				*runStopWatch = 0;
				NotLookingForTS = 0; /* Tell TSI we don't want input */
				Turn_Off_LED(1); /* Turn off green LED */
			}
			/* Increment Score and reset for next round */
			currentScore = (*count * currentRound);
			return currentScore;
}

int main (void) {
	/* Prompt strings to print to terminal */
	char prompt[] = "\r\nWelcome to Touch Reaction Timer\r\n";
	char prompt2[] = "\r\nTouch the touch sensor when the green light turns on.";
	char round[] = "\r\nStarting round ";
	char final[] = "\r\n\nYour Final Score: ";
	char scoring[] = "\r\nThe lower the score the better!";
	char roundScore[] = "\r\nRound Score: ";
	char again[] = "\r\n Press 'y' to play again, or anything else to exit: ";
	/* Input char from user */
	char playAgain; /* Stores if user wants to play again or not */
	char roundCounter; /* Current round (1 - 10) */
	UInt16 currentScore; /* Score of the current round */
	UInt16 waitTime; /* Time to wait before turning on green LED */
	UInt16 *count = &Count; /* Count variable used in the PIT */
	char *runStopWatch = &RunStopWatch; /* Tells the PIT to increment Count or not */
	UInt32 Score; /* Total use score of the round */
	
	playAgain = 'y';
  __asm("CPSID   I");  /* mask interrupts */
  Init_UART0_IRQ();
	Init_GPIO_LED();
	Init_TSI();
	Init_PIT_IRQ();
	*runStopWatch = 0;
	*count = 0;
	NotLookingForTS = 0;
  __asm("CPSIE   I");  /* unmask interrupts */
		/* Print user prompt */
		PutStringSB( prompt, sizeof( prompt ) );
		PutStringSB( prompt2, sizeof( prompt2 ) );
		PutStringSB( scoring, sizeof( scoring ) );
		*runStopWatch = 1;
		/* Get user input to start game */
		PutStringSB( "\r\nPress a key to Start", 23 );
		GetChar();
		*runStopWatch = 0;
		waitTime = *count;
	while( playAgain == 'y' || playAgain == 'Y' ) {
		/* Reset variables for new game */
		roundCounter = 1;
		Score = 0;
		while( roundCounter <= 10 ) {
			/* Print new round start and reset round variables */
			PutStringSB( round, sizeof( round ) );
			PutNumU( roundCounter );
			PutStringSB( ": ", 3 );
			earlyTouch = 0;
			*count = 0;
			Turn_Off_LED(0); /* Turn off red LED */
			Turn_Off_LED(1); /* Turn off green LED */
			/* Keep waitTime between 1 and 3 seconds */
			if( waitTime < 100 ) {
				waitTime = (waitTime + 20) * roundCounter;
			} else if( waitTime > 300) {
				waitTime = waitTime % 300;
			}
			/* Play a round of the game */
			currentScore = Play_Round(roundCounter, waitTime);
			/* Print round score */
			PutStringSB( roundScore, sizeof( roundScore ) );
			PutNumU( currentScore );
			Score += currentScore;
			*count = 0;
			/* Wait period at the end of the round */
			*runStopWatch = 1;
			while( *count < 150 );
			*runStopWatch = 0;
			*count = 0;
			roundCounter++;
		}
		/* Print final score and play again prompt */
		PutStringSB( final, sizeof( final ) );
		PutNumU(Score);
		PutStringSB( again, sizeof( again ) );
		playAgain = GetChar();
		PutChar( playAgain );
	}
} /* main */
