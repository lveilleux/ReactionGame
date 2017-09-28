            TTL Reaction Timing Game
;****************************************************************
;This exercise explores the use of the touch sensor (TSI) built
; into the KL46 board. The game uses the UART0, PIT, and TSI
; to run games of 10 rounds testing the users reaction time
; from when the green LED lights, to when they touch the sensor
;Name:  Luke Veilleux
;Date:  11/29/2016
;Class:  CMPE-250
;Section:  Section 1, Tuesday 2:00 - 3:50
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;April 20, 2015
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  0x1F
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  0xC0
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;Most Significant Bit shift
MSBSHIFT	EQU		24
;Max string length with null terminator
MAX_STRING	EQU		79
;Management Record Structure fields
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17
;Queue sizes in memory
Q_BUF_SZ	EQU		80	;Size of Queue
Q_REC_SZ	EQU		18	;Size of Queue Record
Q_BUFFER_SZ	EQU		4	;Size of operation Queue
;Character values to move from hex to ASCII
ALPHA_CHAR	EQU		55  ;Value to go from 0xA to 'A'
NUM_CHAR	EQU		48	;Value to go from 0x0 to 0
	
;RED and GREEN LEDs
;Green LED Port D pin 5
PTD5_MUX_GPIO 	EQU		(1 << PORT_PCR_MUX_SHIFT)
SET_PTD5_GPIO 	EQU		(PORT_PCR_ISF_MASK :OR: PTD5_MUX_GPIO)
POS_GREEN	  	EQU		(5)
LED_GREEN_MASK 	EQU		(1 << POS_GREEN)
LED_PORTD_MASK 	EQU		(LED_GREEN_MASK)
;Red LED Port E pin 29
PTE29_MUX_GPIO 	EQU		(1 << PORT_PCR_MUX_SHIFT)
SET_PTE29_GPIO  EQU		(PORT_PCR_ISF_MASK :OR: PTE29_MUX_GPIO)
POS_RED			EQU		(29)
LED_RED_MASK 	EQU		(1 << POS_RED)
LED_PORTE_MASK 	EQU		(LED_RED_MASK)


;****************************************************************
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
			EXPORT  PutStringSB
			EXPORT  Init_UART0_IRQ
			EXPORT  Init_GPIO_LED
			EXPORT	UART0_IRQHandler
			EXPORT	GetChar
			EXPORT	PutChar
			EXPORT	PutNumHex
			EXPORT 	PutNumU
			EXPORT 	Turn_On_LED
            EXPORT 	Turn_Off_LED
			EXPORT	Init_PIT_IRQ
			EXPORT	PIT_IRQHandler
			ENTRY
;>>>>> begin subroutine code <<<<<
PutNumU
;Subroutine that prints out the text decimal representaion of the unsigned
; word value stored in R0.
;Input - R0 - Value to print out to the screen in decimal format.
;Output - None
; No registers change on return besides LR, PC, PSR.
			PUSH	{R0 - R2, LR}
			MOVS	R1,#0x04
			PUSH	{R1}
			MOVS	R2,#0x00	;Initialize boolean nothing printed flag
		;Continually divide by 10 and store resulting num in stack
PutNumLoop	MOVS	R1,#10
			BL		DIVU
			CMP		R1,#0x00
			BEQ		PutNumStopDiv
PutNumCont	ADDS	R1,R1,#NUM_CHAR
			PUSH	{R1}
			BAL		PutNumLoop
			
PutNumStopDiv
			CMP		R0,#0x00
			BEQ		PutNumPrint
			BAL		PutNumCont

		;Print out backwards from stack.
PutNumPrint 
			POP		{R0}
			CMP		R0,#0x04
			BEQ		PutNumEndTrans
			MOVS	R2,#1				;Change R2 to be non-zero
			BL		PutChar
			BAL		PutNumPrint
PutNumEndTrans
			CMP		R2,#0x00
			BNE		PutNumRtn
			MOVS	R0,#'0'
			BL		PutChar
		;Restore registers and return
PutNumRtn	
			POP		{R0 - R2, PC}
;****************************************************************
DIVU
; Divides the value in R1 by the value in R0.
; Parameters
;	Inputs:
;		R0:	Divisor [P]
;		R1: Dividend [Q]
;	Outputs:
;		R0: Quotient
;		R1: Remainder
			PUSH	{R2, R3}
			CMP		R1,#0			;Compare R1 and 0
			BEQ		RETURN_0		;If 0, branch to RETURN_0
			MOVS	R2,#0			;R2(Quotient) = 0
			
LOOP_DIV	CMP		R0,R1			;Compare R0 and R1
			BLO		RETURN_DIV		;If R0 < R1, goto RETURN_DIV
			ADDS	R2,R2,#1		;R2 += 1
			SUBS	R0,R0,R1		;R0 -= R1
			BEQ		RETURN_DIV		;Branch if SUBS had a 0 value
			BAL		LOOP_DIV		;Branch back to LOOP_DIV
			
RETURN_DIV	MOVS	R1,R0			;R1 = R0(Remainder)
			MOVS	R0,R2			;R0 = R2(Quotient)
			MRS		R2,APSR			;Store from flags register
			MOVS	R3,#0x20		;R3 = 32
			LSLS	R3,R3,#MSBSHIFT ;Shift to MSb position
			BICS	R2,R2,R3		;R2 && R3
			MSR		APSR,R2			;Restore flag register
			POP		{R2, R3}		;Restore register values
			BX		LR			;Return
			
RETURN_0	MRS		R2,APSR			
			MOVS	R3, #0x20		;R3 = 32
			LSLS	R3,R3,#MSBSHIFT	;Shift to MSb position
			ORRS	R2,R2,R3		;R2 || R3
			MSR		APSR,R2			;Restore flag register
			POP		{R2, R3}		;Restore register values
			BX		LR
;****************************************************************
PutNumHex
;Subroutine that takes in a hexadecimal number and prints it out to
; the terminal window, making use of the PutChar subroutine and the
; UART0 to do this.
;Inputs - R0 - Hexadecimal number to print out
;Outputs - None
;Modify - None besides PSR.
			PUSH	{R0 - R3, LR}
			
			MOVS	R3,#28			;Initialize Loop/shift counter
			MOVS	R2,R0
PutNumHexLoop
		;Get a single digit from hex number and print it.
		;Starting at MSB to LSB
			MOVS	R0,R2
			LDR		R1,=0x0000000F
			LSLS	R1,R1,R3
			ANDS	R0,R0,R1
			LSRS	R0,R0,R3
			CMP		R0,#9
			BLE		PutNumHexNumber		;Digit is a number
			ADDS	R0,R0,#ALPHA_CHAR
			BAL		PutNumHexPrint		;Digit is alphabetic
PutNumHexNumber
			ADDS	R0,R0,#NUM_CHAR
PutNumHexPrint
		;Print resulting character
			BL		PutChar
			SUBS	R3,R3,#4			;Decrement counter/shift
			BLO		PutNumHexDone
			BAL		PutNumHexLoop
PutNumHexDone
			POP		{R0 - R3, PC}
;**************************************************************
InitQueue
;Subroutine that takes in a queue and record structure as well as
; a max capacity and initializes the Queue and record structure.
;Inputs - R0 - Starting address for Queue buffer
;		  R1 - Starting address for Queue record structure
;		  R2 - Max capacity of the queue.
;Outputs - [R0] - Initialized Queue Buffer starting address
;		   [R1] - Initialized Queue Record starting address
;Only R0, R1 & PSR change value on return.
			PUSH	{R0 - R2, LR}
		;Store In/Out Pointers & Buffer start
			STR		R0,[R1,#IN_PTR]
			STR		R0,[R1,#OUT_PTR]
			STR		R0,[R1,#BUF_STRT]
		;Compute and store Buffer past and size
			ADDS	R0,R0,R2
			STR		R0,[R1,#BUF_PAST]
			STRB	R2,[R1,#BUF_SIZE]
		;Store 0 for number enqueued
			MOVS	R2,#0
			STRB	R2,[R1,#NUM_ENQD]
			
			POP		{R0 - R2, PC}
;******************************************************************
Enqueue
;If the queue(whose queue record structure's address is in R1) is
; not full, enqueue the character from R0 to the queue and report
;success by returning with the C flag cleared,
; otherwise set the C flag.
;Inputs - R0 - character to enqueue
;		  R1 - Address of the Queue Record
;Outputs - PSR C flag - Success (0) or failure (1)
;Modify: PSR. All other registers remain unchanged on return
		
			PUSH	{R0 - R4, LR}
		;Check if there is space in the queue
			LDRB	R2,[R1,#NUM_ENQD]
			LDRB	R3,[R1,#BUF_SIZE]
			CMP		R2,R3
			BGE		EnqueueFull			;Branch if queue is full
		;Queue has space, store character, update Record.
			LDR		R3,[R1,#IN_PTR]
			STRB	R0,[R3,#0]
			ADDS	R2,R2,#1			;Increment number enqueued
			ADDS	R3,R3,#1			;Increment InPointer
		;Check if InPointer is within the queue.
			LDR		R4,[R1,#BUF_PAST]
			CMP		R3,R4
			BLT		EnqueueStorePTR
			LDR		R3,[R1,#BUF_STRT]
EnqueueStorePTR
			STR		R3,[R1,#IN_PTR]
			STRB	R2,[R1,#NUM_ENQD]
			BAL		EnqueueSuccess

EnqueueFull
		;Set C flag to show failure
			MRS		R2,APSR		
			MOVS	R3, #0x20
			LSLS	R3,R3,#MSBSHIFT
			ORRS	R2,R2,R3
			MSR		APSR,R2
			BAL		EnqueueDone

EnqueueSuccess
		;Clear C flag to show Success
			MRS		R2,APSR
			MOVS	R3,#0x20
			LSLS	R3,R3,#MSBSHIFT
			BICS	R2,R2,R3
			MSR		APSR,R2
EnqueueDone
			POP		{R0 - R4, PC}
;********************************************************************
Dequeue
;If the queue(whose queue record structure's address is in R1) is not 
; empty, dequeue the next character from the queue and report success
; by returning with the C flag cleared, otherwise set the C flag.
;Inputs - R1 - Address of the Queue Record
;Output - R0 - Character dequeued
;	      PSR C flag - Success (0) or failure (1)
;Modify: PSR, R0. All other registers remain unchanged on return
			
			PUSH	{R1 - R4, LR}
		;Check if the Queue is empty
			LDRB	R2,[R1,#NUM_ENQD]
			CMP		R2,#0
			BEQ		DequeueEmpty
		;Dequeue the next character
			LDR		R3,[R1,#OUT_PTR]
			LDRB	R0,[R3,#0]
			SUBS	R2,R2,#1			;Increment Number Enqueued
			ADDS	R3,R3,#1			;Increment OutPointer
		;Check if OutPointer is within Queue
			LDR		R4,[R1,#BUF_PAST]
			CMP		R3,R4
			BLT		DequeueStorePTR
			LDR		R3,[R1,#BUF_STRT]
DequeueStorePTR
			STR		R3,[R1,#OUT_PTR]
			STRB	R2,[R1,#NUM_ENQD]
			BAL		DequeueSuccess

DequeueEmpty
		;Set C flag to show failure
			MRS		R2,APSR		
			MOVS	R3, #0x20
			LSLS	R3,R3,#MSBSHIFT
			ORRS	R2,R2,R3
			MSR		APSR,R2
			BAL		DequeueDone

DequeueSuccess
		;Clear C flag to show Success
			MRS		R2,APSR
			MOVS	R3,#0x20
			LSLS	R3,R3,#MSBSHIFT
			BICS	R2,R2,R3
			MSR		APSR,R2
DequeueDone
			POP		{R1 - R4, PC}
;***********************************************************************
PutStringSB
;Subroutine that prevents overrun of the buffer capacity specified in R1
; this subroutine displays a null-terminated string to the terminal screen
; from memory starting at the address in R0. It leaves the cursor positioned
; after the last character of the string.
;Input - R0 - Starting address of string to print.
;Output - None
; No registers change on return besides LR, PC, PSR.
			PUSH	{R0 - R2, LR}
			
			MOVS	R2,#0x00
			MOVS	R1,R0		;Move String location to R1
		;Loop loading chars and comparing to the null character.
PutStrLoop	LDRB	R0,[R1,#0]
			ADDS	R1,R1,#1
			BL		PutChar
			CMP		R0,R2		;Compare char with null char.
			BEQ		PutStrOut
			BAL		PutStrLoop
		;Restore registers and return
PutStrOut
			POP		{R0 - R2, PC}
;***********************************************************************
Turn_On_LED
;Subroutine that turns on either the red LED, or the green LED
;dependent on if the input is zero or not.
;Input - LEDSelect(R0) - 0 for red LED, otherwise green LED
;Output - None
;Modify - PSR, PC
			PUSH	{R0, R1}
;Check if R0 is 0 or not for red or green LED
			CMP		R0,#0
			BNE		Turn_On_GRN
;Turn on the red LED
			LDR		R1,=FGPIOE_BASE
			LDR		R0,=LED_RED_MASK
			STR		R0,[R1,#GPIO_PCOR_OFFSET]
			BAL		Turn_On_Done
Turn_On_GRN
;Turn on the green LED
			LDR		R1,=FGPIOD_BASE
			LDR		R0,=LED_GREEN_MASK
			STR		R0,[R1,#GPIO_PCOR_OFFSET]
Turn_On_Done
			POP		{R0, R1}
			BX		LR
;***********************************************************************
Turn_Off_LED
;Subroutine that turns off either the red LED, or the green LED
;dependent on if the input is zero or not.
;Input - LEDSelect(R0) - 0 for red LED, otherwise green LED
;Output - None
;Modify - PSR, PC
			PUSH	{R0, R1}
;Check if R0 is 0 or not for red or green LED
			CMP		R0,#0
			BNE		Turn_Off_GRN
;Turn on the red LED
			LDR		R1,=FGPIOE_BASE
			LDR		R0,=LED_RED_MASK
			STR		R0,[R1,#GPIO_PSOR_OFFSET]
			BAL		Turn_Off_Done
Turn_Off_GRN
;Turn on the green LED
			LDR		R1,=FGPIOD_BASE
			LDR		R0,=LED_GREEN_MASK
			STR		R0,[R1,#GPIO_PSOR_OFFSET]
Turn_Off_Done
			POP		{R0, R1}
			BX		LR
;***********************************************************************
Init_GPIO_LED
;Subroutine to initialize the two LEDs on the KL46Z board.
;Green LED is found at PORT D pin 5
;Red LED is found at PORT E pin 29
;Input - None
;Output - None
; No registers change on return besides LR, PC, PSR.
			PUSH	{R0 - R2}
;Enable clock on port D and E modules
			LDR		R2,=SIM_SCGC5
			LDR		R1,=(SIM_SCGC5_PORTD_MASK :OR: SIM_SCGC5_PORTE_MASK)
			LDR		R0,[R2,#0]
			ORRS	R0,R0,R1
			STR		R0,[R2,#0]
;Select PORT E PIN 29 for GPIO to red LED
			LDR		R2,=PORTE_BASE
			LDR		R0,=SET_PTE29_GPIO
			STR		R0,[R2,#PORTE_PCR29_OFFSET]
;Select PORT D PIN 5 for GPIO to green LED
			LDR		R2,=PORTD_BASE
			LDR		R0,=SET_PTD5_GPIO
			STR		R0,[R2,#PORTD_PCR5_OFFSET]
;Select data direction for LED pins
			LDR		R2,=FGPIOD_BASE
			LDR		R1,=LED_PORTD_MASK
			STR		R1,[R2,#GPIO_PDDR_OFFSET]
			LDR		R2,=FGPIOE_BASE
			LDR		R1,=LED_PORTE_MASK
			STR		R1,[R2,#GPIO_PDDR_OFFSET]
			POP		{R0 - R2}
			BX		LR

;***********************************************************************
Init_UART0_IRQ
;Subroutine to initialize UART0.
;UART0 is initialized for interrupt serial IO through Port A pins 1 & 2
;using format: 8 data bits, no parity, one stop bit at 9600 baud.
;Input - None
;Output - None
; No registers change on return besides LR, PC, PSR.
			PUSH	{R1 - R3, LR}
		;Select MCGPLLCLK / 2 as UART0 clock source
			LDR		R1,=SIM_SOPT2
			LDR		R2,=SIM_SOPT2_UART0SRC_MASK
			LDR		R3,[R1,#0]
			BICS	R3,R3,R2
			LDR		R2,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS	R3,R3,R2
			STR		R3,[R1,#0]
		;Enable external connection for UART0
			LDR		R1,=SIM_SOPT5
			LDR		R2,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR		R3,[R1,#0]
			BICS	R3,R3,R2
			STR		R3,[R1,#0]
		;Enable clock for UART0 module
			LDR		R1,=SIM_SCGC4
			LDR		R2,=SIM_SCGC4_UART0_MASK
			LDR		R3,[R1,#0]
			ORRS	R3,R3,R2
			STR		R3,[R1,#0]
		;Enable clock for Port A module
			LDR		R1,=SIM_SCGC5
			LDR		R2,=SIM_SCGC5_PORTA_MASK
			LDR		R3,[R1,#0]
			ORRS	R3,R3,R2
			STR		R3,[R1,#0]
		;Connect Port A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
			LDR		R1,=PORTA_PCR1
			LDR		R2,=PORT_PCR_SET_PTA1_UART0_RX
			STR		R2,[R1,#0]
		;Connect Port A Pin 2 (PTA2) to UART0 Rx (J1 Pin 04)
			LDR		R1,=PORTA_PCR2
			LDR		R2,=PORT_PCR_SET_PTA2_UART0_TX
			STR		R2,[R1,#0]
		;************************************************************
		;Disable UARTo receiver and transmitter
			LDR		R1,=UART0_BASE
			MOVS	R2,#UART0_C2_T_R
			LDRB	R3,[R1,#UART0_C2_OFFSET]
			BICS	R3,R3,R2
			STRB	R3,[R1,#UART0_C2_OFFSET]
		;Set UART0 for 9600 baud, 8n1 protocol
			MOVS	R2,#UART0_BDH_9600
			STRB	R2,[R1,#UART0_BDH_OFFSET]
			MOVS	R2,#UART0_BDL_9600
			STRB	R2,[R1,#UART0_BDL_OFFSET]
			MOVS	R2,#UART0_C1_8N1
			STRB	R2,[R1,#UART0_C1_OFFSET]
			MOVS	R2,#UART0_C3_NO_TXINV
			STRB	R2,[R1,#UART0_C3_OFFSET]
			MOVS	R2,#UART0_C4_NO_MATCH_OSR_16
			STRB	R2,[R1,#UART0_C4_OFFSET]
			
			MOVS	R2,#UART0_C5_NO_DMA_SSR_SYNC
			STRB	R2,[R1,#UART0_C5_OFFSET]
			MOVS	R2,#UART0_S1_CLEAR_FLAGS
			STRB	R2,[R1,#UART0_S1_OFFSET]
			MOVS	R2,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB	R2,[R1,#UART0_S2_OFFSET]
		;Enables UART0 receiver and transmitter
			MOVS	R2,#UART0_C2_T_RI
			STRB	R2,[R1,#UART0_C2_OFFSET]
		;Set UART0 IRQ Priority
			LDR		R1,=UART0_IPR
			LDR		R2,=NVIC_IPR_UART0_PRI_3
			LDR		R3,[R1,#0]
			ORRS	R3,R3,R2
			STR		R3,[R1,#0]
		;Clear any pending interrupts
			LDR		R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_UART0_MASK
			STR		R1,[R0,#0]
		;Unmask interrupts for UART0
			LDR		R0,=NVIC_ISER
			LDR		R1,=NVIC_ISER_UART0_MASK
			STR		R1,[R0,#0]
		
		;Initialize Input and Output Queues
			LDR		R0,=RxQBuffer
			LDR		R1,=RxQRecord
			MOVS	R2,#Q_BUF_SZ
			BL		InitQueue
			LDR		R0,=TxQBuffer
			LDR		R1,=TxQRecord
			BL		InitQueue
			POP		{R1 - R3, PC}
;****************************************************************
GetChar
;Get a character from the recieve queue and places it into R0.
;Input - None
;Output - R0
;Only Output register (R0) and LR, PC, & PSR are changed
			PUSH	{R1, LR}
			LDR		R1,=RxQRecord
GetCharLoop	CPSID	I
			BL		Dequeue
			CPSIE	I
			NOP
			BCS		GetCharLoop
			POP		{R1, PC}
;****************************************************************
PutChar
;Place the character in R0 into the transmit queue, and enable
; transmit interrupts.
;Input - R0
;Output - None
;Only LR, PC, & PSR are changed on return.
			PUSH	{R1, R2, LR}
			LDR		R1,=TxQRecord
PutCharLoop	CPSID	I
			BL		Enqueue
			CPSIE	I
			NOP
			BCS		PutCharLoop
		;Enable Transmit interrupts
			LDR		R1,=UART0_BASE
			MOVS	R2,#UART0_C2_TI_RI
			STRB	R2,[R1,#UART0_C2_OFFSET]
			POP		{R1, R2, PC}
;****************************************************************
UART0_IRQHandler
UART0_ISR
;Subroutine to handle transmit and recieve interrupts generated
;by the UART0. Recieve interrupts are always enabled, transmit
;interrupts are disabled if there is nothing to transmit.
;No Input or Output, Nothing modified on return.
			CPSID	I
			PUSH	{LR}
		;Determine source of interrupt
			LDR		R3,=UART0_BASE
			LDRB	R2,[R3,#UART0_C2_OFFSET]
			CMP		R2,#UART0_C2_TI_RI
			BNE		UART0_ISR_REC
		;Check if TDRE is set
			LDRB	R2,[R3,#UART0_S1_OFFSET]
			MOVS	R1,#0x7F			;Set mask for other bits in S1
			BICS	R2,R2,R1			;clear other bits in S1
			BEQ		UART0_ISR_REC
		;TDRE is set, dequeue a character and put it in UART0_D
			LDR		R1,=TxQRecord
			BL		Dequeue
			BCS		UART0_ISR_DQ_FAIL
		;Got a char from Dequeue, put it into data register
			STRB	R0,[R3,#UART0_D_OFFSET]
			BAL		UART0_ISR_DONE
UART0_ISR_DQ_FAIL
		;Nothing found to dequeue, disable transmit interrupts
			MOVS	R2,#UART0_C2_T_RI
			STRB	R2,[R3,#UART0_C2_OFFSET]
			BAL		UART0_ISR_DONE	
UART0_ISR_REC
			LDRB	R2,[R3,#UART_S1_OFFSET]
			MOVS	R1,#0xDF
			BICS	R2,R2,R1
			BEQ		UART0_ISR_DONE		;Not a Rec interrupt, exit
		;Assume if we got here, we are looking to take in a character
			LDRB	R0,[R3,#UART0_D_OFFSET]
			LDR		R1,=RxQRecord
			BL		Enqueue
UART0_ISR_DONE
			CPSIE	I
			POP		{PC}
;****************************************************************
PIT_ISR
PIT_IRQHandler
;Subroutine to handle interrupts generated from the PIT Interrupts
;are generated when the timer hits zero. Each time, the variable
;Count is incremented by one.

			CPSID	I
		;Check if clock in enabled RunStopWatch != 0
			LDR		R0,=RunStopWatch
			LDRB	R0,[R0,#0]
			CMP		R0,#0
			BEQ		PIT_ISR_CLEAR	;disabled; clear interrupt
			
		;Increment Count and store back in memory
			LDR		R0,=Count
			LDR		R1,[R0,#0]
			ADDS	R1,R1,#1
			STR		R1,[R0,#0]
PIT_ISR_CLEAR		
		;Clear Interrupt flag
			LDR		R1,=PIT_TFLG0
			LDR		R0,=PIT_TFLG_TIF_MASK
			STR		R0,[R1,#0]
			CPSIE	I
			BX		LR

;****************************************************************
Init_PIT_IRQ
;Subroutine to initialize the PIT module to send an interrupt
;every 0.01s. This is done in connection to the 24Mhz module clock
;and 48Mhz core clock of the KL46
			PUSH 	{R0 - R2}
		;Enable clock for PIT module
			LDR		R1,=SIM_SCGC6
			LDR		R2,=SIM_SCGC6_PIT_MASK
			LDR		R0,[R1,#0]
			ORRS	R0,R0,R2
			STR		R0,[R1,#0]
		;Disable PIT timer 0
			LDR		R1,=PIT_CH0_BASE
			LDR		R2,=PIT_TCTRL_TEN_MASK
			LDR		R0,[R1,#0]
			BICS	R0,R0,R2
			STR		R0,[R1,#0]
		;Set PIT Interrupt Priority
			LDR		R1,=PIT_IPR
			LDR		R2,=NVIC_IPR_PIT_MASK
			LDR		R0,[R1,#0]
			BICS	R0,R0,R2
			STR		R0,[R1,#0]
		;Clear any pending interrupts
			LDR		R1,=NVIC_ICPR
			LDR		R0,=NVIC_ICPR_PIT_MASK
			STR		R0,[R1,#0]
		;Unmask PIT interrupts
			LDR		R1,=NVIC_ISER
			LDR		R0,=NVIC_ISER_PIT_MASK
			STR		R0,[R1,#0]
		;Enable PIT module
			LDR		R1,=PIT_BASE
			LDR		R0,=PIT_MCR_EN_FRZ
			STR		R0,[R1,#PIT_MCR_OFFSET]
		;Set PIT timer 0 period for 0.001 s
			LDR		R1,=PIT_CH0_BASE
			LDR		R0,=PIT_LDVAL_10ms
			STR		R0,[R1,#PIT_LDVAL_OFFSET]
		;Enable PIT timer 0 interrupt
			LDR		R0,=PIT_TCTRL_CH_IE
			STR		R0,[R1,#PIT_TCTRL_OFFSET]
			
			POP		{R0 - R2}
			BX		LR
;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<

;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
			EXPORT	RunStopWatch
			EXPORT	Count
;>>>>> begin variables here <<<<<
			ALIGN
RxQRecord	SPACE	Q_REC_SZ
RxQBuffer	SPACE	Q_BUF_SZ
			ALIGN
TxQRecord	SPACE	Q_REC_SZ
TxQBuffer	SPACE	Q_BUF_SZ
			ALIGN
RunStopWatch SPACE	1
			ALIGN
Count		SPACE	4
;>>>>>   end variables here <<<<<
            END