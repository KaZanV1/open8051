;#include <SI_EFM8BB1_Defs.inc>
#include <SI_EFM8BB3_Defs.inc>


BTN0	BIT		P3.4 	;BUTONUL DIN PLACA =PULSADOR
CLK		BIT		P1.0
MISO	BIT		P1.2

RF_CE	BIT		P0.1
RF_SEL	BIT		P0.2


REL		BIT		P1.4
;REL		BIT		P3.0	;PT DEBUG LOW VOLTAGE

RED		BIT		P1.3
BLU		BIT		P1.2

TST1	BIT		P2.1
TST2	BIT		P2.0 	;led roshu

SWC		BIT		P2.6		;SWITCH FOR CRYSTAL DETECT

PWM0	BIT		P1.6
PWM1	BIT		P1.7

SW1		BIT		P3.4
SW2		BIT		P3.3


;POT		BIT		P1.5	;ANALOG INPUT FOR POTENTIOMETER
;CURENT		BIT		P3.0	;EROR MUST BE LOWER THAT 3.0 ANALOG INPUT FOR CURENT MEASURE
;TEMPERAT	BIT		P0.3	;ANALOG INPUT FOR TEMPERATURE MEASURMENT



;LED		BIT		P1.4




;RF_IRQ	BIT		P0.7
IRQ_R	BIT		P0.7

dbg		bit		p3.2


	




ACK		BIT		000H	;FLAG ACKNOLEDEGE
LINK	BIT		001H	;FLAG CA A PRINS LINKUL
OV_V	BIT		002H	;FLAG CARE ANUNTZA PESTE TENSIUNE
UND_V	BIT		003H	;FLAG CARE ANUNTZA SUB TENSIUNE CIND CONECTEZ CARGA		



T_ADR_4	DATA	024H	;ADRESA TX DEVICE
T_ADR_3	DATA	025H
T_ADR_2	DATA	026H
T_ADR_1	DATA	027H
T_ADR_0	DATA	028H

R_ADR_4	DATA	029H	;ADRESA RX DEVICE
R_ADR_3	DATA	02AH
R_ADR_2	DATA	02BH
R_ADR_1	DATA	02CH
R_ADR_0	DATA	02DH

D_CRC16_Hi	DATA	02EH
D_CRC16_Lo	DATA	02FH

CNTPCK		DATA	030H	;CONTOR DE PAKETE PRIMITE,ASHA DEAGUIESC UNU CITE UNU
T2_TOT		DATA	031H	;CONTOR D ETICK TIMER 2 APROX 30MS
VDET_H		DATA	032H	;NIVELUL D EDETE CTIE PT DESCONECTAT RELE/REZISTENTZA ETC
VDET_L		DATA	033H




CONFIG      EQU	0x00            //'Config' Reg
EN_AA  		EQU 0x01
EN_RXADDR	EQU 0x02
SETUP_PETR  EQU	0x04            //Setup of Automatic Retransmission Reg
RF_CH		EQU 0x05			//RF Channel Reg
RF_SETUP    EQU	0x06            //RF Setup Reg
DYNPD     	EQU	0x1C           //ENABLE DYNAMIC PAYLOAD LENGHT
FEATURE     EQU	0x1D           //FEATURE REGISTER
WR_STS		EQU	027H		;SCRIU STATUS


TIME5H		EQU	09BH		;VAL TIMER 5 PT 50MS
TIME5L		EQU	0A0H

T2_TICK		EQU	00AH		;CITE TICK DE TIMER 2 SA FACA




POT_PWR_SEL_H	EQU	008H	;LIMITA D EUND ECONTROLEAZA PUTEREA
POT_PWR_SEL_L	EQU 0D0H

POT_OFF_LVL_H	EQU	007H 	;POT PT SELECTAT FUNCTII BATIDORA IN OFF 
POT_OFF_LVL_L	EQU	040H


POT_A_LVL_H		EQU	006H 	;POT PT SELECTAT FUNCTII BATIDORA
POT_A_LVL_L		EQU	030H

POT_B_LVL_H		EQU	004H
POT_B_LVL_L		EQU	020H

POT_C_LVL_H		EQU	004H
POT_C_LVL_L		EQU	020H


;============================================

MAX_PWR_H		EQU	00CH	;PUTEREA MAXIMA DE DAT LA BATIDORA
MAX_PWR_L		EQU	0BAH

PA_PWR_H		EQU	003H	;PUTEREA A
PA_PWR_L		EQU	035H

PB_PWR_H		EQU	004H	;PUTEREA B DE DAT LA BATIDORA
PB_PWR_L		EQU	023H

PC_PWR_H		EQU	005H	;PUTEREA C DE DAT LA BATIDORA
PC_PWR_L		EQU	069H

POFF_PWR_H		EQU	001H	;PUTEREA DE LA STANDBY..MINIM PT DETECTAT 
POFF_PWR_L		EQU	023H


ORG	000H
	JMP INIT_DEVICE

ORG	03BH	;TIMER_2_ISR
	CALL TIMER_2_ISR
RETI

org 0073h	;TIMER 3 ISR
	CALL TIMER_3_ISR
RETI


org	0fdh	;FIRM VERSION
FRW:
DB	0C1H,004H,001H

ORG 0100H
; Peripheral specific initialization functions,
; Called from the Init_Device label
PCA_Init:
    mov  PCA0CN0,    #040h
    anl  PCA0MD,    #0BFh
    mov  PCA0MD,    #004h
    mov  PCA0CPM0,  #000h
    mov  PCA0CPH0,  #01Fh
    mov  PCA0CPL5,  #000h
    orl  PCA0MD,    #040h
    ret
Timer_Init:
mov  TCON,      #050h
    mov  TMOD,      #022h
    mov  CKCON0,     #008h
    mov  TH0,       #0F0h
    mov  TH1,       #096h
    mov  TMR3RLH,   #0AFh
    mov  TMR3H,     #0AFh
    ret

UART_Init:
    mov  SCON0,     #052h
    ret

SPI_Init:
    mov  SPI0CFG,   #040h
    mov  SPI0CN0,    #001h
    mov  SPI0CKR,   #003h
    ret

ADC_Init:
    mov  ADC0MX,    #00Eh
	mov  ADC0CF0,    #008h 	;SAR =CLK/(1+1)	=12Mhz
	mov  ADC0CF2,    #01Fh 	;ADC DAC REF = INTERNAL=2,4V
	mov  ADC0CN0,    #083h 	;TEMP SI GAIN DE 0,75
	mov  ADC0CN1,    #05Ah 	;repetat=8 just=3
;	mov  ADC0CN2,    #001h	;CONVERSION=TIMER 1 
;	mov  ADC0CN2,    #007h	;CONVERSION=TIMER 5
	mov  ADC0CN2,    #000h	;CONVERSION=ADBUSY

;	mov  ADC0GTH,   #009h
;    mov  ADC0GTL,   #030h
    ret

Voltage_Reference_Init:
    MOV A,REF0CN
	ORL A,#080H
	mov  REF0CN,A
    ret

Port_IO_Init:
 ; P0.0  -  Skipped,     Open-Drain, Analog
    ; P0.1  -  Skipped,     Push-Pull,  Digital
    ; P0.2  -  Skipped,     Push-Pull,  Digital
    ; P0.3  -  SCK  (SPI0), Push-Pull,  Digital
    ; P0.4  -  TX0 (UART0), Open-Drain, Digital
    ; P0.5  -  RX0 (UART0), Open-Drain, Digital
    ; P0.6  -  MISO (SPI0), Open-Drain, Digital
    ; P0.7  -  Skipped,     Open-Drain, Digital

    ; P1.0  -  MOSI (SPI0), Push-Pull,  Digital
    ; P1.1  -  CEX0 (PCA),  Push-Pull,  Digital
    ; P1.2  -  Unassigned,  Open-Drain, Digital
    ; P1.3  -  Unassigned,  Open-Drain, Digital
    ; P1.4  -  Unassigned,  Push-Pull,  Digital
    ; P1.5  -  Skipped,     Open-Drain, Analog
    ; P1.6  -  Unassigned,  Open-Drain, Digital
    ; P1.7  -  Unassigned,  Open-Drain, Digital

    ; P2.0  -  Unassigned,  Open-Drain, Digital
    ; P2.1  -  Unassigned,  Open-Drain, Digital
    ; P2.2  -  Unassigned,  Open-Drain, Digital
    ; P2.3  -  Skipped,     Open-Drain, Analog
    ; P2.4  -  Unassigned,  Open-Drain, Digital
    ; P2.5  -  Skipped,     Open-Drain, Analog
    ; P2.6  -  Unassigned,  Open-Drain, Digital
    ; P2.7  -  Skipped,     Open-Drain, Digital
MOV SFRPAGE,#020H
	mov  P0MDIN,    #0FEh
    mov  P1MDIN,    #0DFh
	mov  P2MDIN,    #0D7h
    mov  P0MDOUT,   #00Eh
    mov  P1MDOUT,   #013h
    mov  P0SKIP,    #087h
	mov  P1SKIP,    #020h
	mov  P2SKIP,    #0A8h
    mov  XBR0,      #003h
    mov  XBR1,      #001h
    mov  XBR2,      #040h

MOV SFRPAGE,#000H

Oscillator_Init:
;    mov  FLSCL,     #040h
    mov  CLKSEL,    #000h
    ret

Interrupts_Init:
    mov  EIE1,      #080h
    mov  IE,        #0A0h
    ret



INIT_DEVICE:
CLR REL
	;WATCHDOG DISABLE
	MOV WDTCN,#0DEh ; disable software watchdog timer
	MOV WDTCN,#0ADh
	MOV VDM0CN,#080H	;VDD ON ENABLED
	CLR PWM0
	CLR PWM1
	LCALL PCA_Init
	lcall Timer_Init
    lcall UART_Init
    lcall SPI_Init
    lcall ADC_Init
	lcall Voltage_Reference_Init
    lcall Port_IO_Init
    lcall Oscillator_Init
	lcall Interrupts_Init


MOV SFRPAGE,#010H
;	MOV EIE2,#008H	;TIMER5 ISR ENABLED
;	MOV EIE2,#00CH	;TIMER5 TIMER 4 ISR ENABLED
;	MOV EIP2,#008H	;TIMER 5 ISR HIGH PRIORITY
	MOV TMR5H ,#TIME5H
	MOV TMR5L ,#TIME5L
	MOV TMR5RLH,#TIME5H
	MOV TMR5RLL,#TIME5L
	CLR TMR5CN0_TF5H
	
MOV SFRPAGE,#000H








	 MOV SP,#050H
MOV CNTPCK,#000H	;PT DEBUG NUMAR PAKETELE	 
	 
	 
	 MOV DPL,#000H
	MOV DPH,#000H
	mov a,#00h
   CLEAN_XRAM:
   		mov a,#00h
		MOVX @DPTR,A
		INC DPTR
		MOV A,DPH
		CJNE A,#002H,CLEAN_XRAM
   NOP
;   CALL WAIT_SPI_BYTE
   NOP
;   	MOV DPTR,#SPACE
;	CALL SENDSTRING
;	MOV DPTR,#SPACE
;	CALL SENDSTRING

	CLR BLU
	CLR RED
;	CLR TST1
	CLR TST2

	SETB PWM0
	CLR PWM1
	ORL TMR3CN0,#00000100B	;TIMER 3 RUN
	MOV  T2_TOT,#000H


	mov  PCA0CPM0,  #000h
	CALL WAIT_20_MS
	;--------	INCARC NIVELUL DE DETECTIE LVL2 ESTE INNALT,SA SARA PESTE OPRESC
	MOV DPTR,#DET_LVL_2
		CLR A
		MOVC A,@A+DPTR
		MOV ADC0GTH,A
		INC DPTR
		CLR A
		MOVC A,@A+DPTR
		MOV ADC0GTL,A
		MOV ADC0LTL,#000H
		MOV ADC0LTH,#000H
		;===============================
	CLR OV_V	;STERG FLAG SUPRATENSIUNE
	
	;ACTIV BUZZER
	;mov  PCA0CPM0,  #046h
	;CALL WAIT_20_MS
	;mov  PCA0CPM0,  #000h



TIME_LOOP:
	MOV R0,#000H
	TIME_LOOP_1:
	CALL WAIT_4_5_MS
	DJNZ R0,TIME_LOOP_1


   	CALL CRC16_Init
	MOV A,#000H
	CALL CRC16_AddCalc
	MOV A,#000H
	CALL CRC16_AddCalc
	
	
	
	RF_RADIO_INIT:
		;____________________________	COPIEZ ADRESA DEFAULT DIN CODE IN IRAM
		MOV DPTR,#DEFAULT_ADRESS
		MOV R0,#024H		;POINTER UNDE INCEPE ADRESA
		RF_RADIO_INIT_1:
			MOV A,#000H
			MOVC A,@A+DPTR
			MOV @R0,A
			INC DPTR
			INC R0
			MOV A,R0
			CJNE A,#02EH,RF_RADIO_INIT_1

   TX_BUF_PREPARE:				;PREGATESC BUFERUL PT TX,CU VALORILE PREDETRMINATE DE START ADRESA DE EXEMPLU
   		MOV R0,#000H
		MOV DPTR,#TOOL_INIT_STRING
	   	TX_BUF_PREPARE_1:
			CLR A
			MOVC A,@A+DPTR
			MOVX @R0,A
			INC R0
			INC DPTR
			MOV A,R0
			CJNE A,#020H,TX_BUF_PREPARE_1
	;____________________________________________

;RF_RADIO_INIT
   	CLR RF_SEL
   		MOV A,#027H		;WRT STATUS IQ
		CALL WR_BYT
		MOV A,#0FFH
		CALL WR_BYT		;STERG TOATE FLAGURILE DE IRQ
	SETB RF_SEL
	;_____________________________
	CALL FLUSH_RX	
	CALL WR_TX_ADR
	CALL WR_RX0_ADR		;SCRIU ADRESA PIPE_0

;	CALL RD_RX_ADR_P0
;	CALL RD_TX_ADR
	   
	   ;ACTIVEZ AUTOAKNOLEDGE PIPE_0
		MOV A,#EN_AA
			MOV B,#001H		;ACK PIPE_0
;			MOV B,#000H		;NO ACK -PT SNIFF
		CALL WR_RF_REG
		;ACTIVEZ PIPE_0_ADR
		MOV A,#EN_RXADDR	
			MOV B,#001H
		CALL WR_RF_REG
		;2433 RETRANSMIT DELAY
		MOV A,#SETUP_PETR	
			MOV B,#033H		;TREI INCERCARI LA 1MS DISTANTZA
		CALL WR_RF_REG
		;260F RF_SETUP_SPEED,ETC
		MOV A,#RF_SETUP	
			MOV B,#00FH
		CALL WR_RF_REG
		;200E CONFIG
		MOV A,#CONFIG	
			MOV B,#00EH		;IRQ-RX,TX,RT CRC,CRC-2B STANBY_I PTX
;			MOV B,#00FH		;IRQ-RX,TX,RT CRC,CRC-2B STANBY_I PRX
		CALL WR_RF_REG

		CALL FLUSH_TX
		CALL FLUSH_RX

		;3C01 ENABLE DYNAMIC PIPE 1	 DYNPD
		MOV A,#DYNPD	
			MOV B,#001H		;PIPE_0_DINAMIC LENGHT
		CALL WR_RF_REG

		;3D06 DYNAMIC PAYLOAD,PAYLOAD WITH ACK
		MOV A,#FEATURE	
			MOV B,#006H		;DYNAMIC PAYLOAD WITH ACK
		CALL WR_RF_REG
	
		;2500--  CANAL 00
		MOV A,#RF_CH	
			MOV B,#000H		;DYNAMIC PAYLOAD WITH ACK
		CALL WR_RF_REG
	

		;-------------------
		MOV R0,#00BH
		MOV DPTR,#PLVL_0
		CLR A
		MOVC A,@A+DPTR
		MOVX @R0,A
			INC DPTR
			INC R0
			CLR A
			MOVC A,@A+DPTR
			MOVX @R0,A

LOOP:		
		CALL TX_XRAM_2_BUF	



;LUPU:
;CALL ADC_POT
;SJMP LUPU

;=====================================
;	PREPARE FIRST CONTACT
			MOV SFRPAGE,#010H
	SETB TMR5CN0_TR5	;START TIMER 4
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
		JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
		JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
		   MOV SFRPAGE,#000H




WAIT_FIRST_LINK:	;WAIT FOR ACK FROM FIRE CONTROLER
	  CALL WAIT_4_5_MS
	  CALL SEND_PCK
;SJMP WAIT_FIRST_LINK_SIM
	  JNB ACK,WAIT_FIRST_LINK

	WAIT_FIRST_LINK_SIM:	  
	  BUZZER_ON:		;TEST DACA A FOST DEJA PORNIT
	  SETB TMR2CN0_TR2
	  mov  PCA0CPM0,  #046h

		;	SELECT_PROGRAM_0:	
		;------------------------
			MOV DPTR,# PLVL_0
				MOV R0,#00BH
				;MOV A,#MAX_PWR_H
				CLR A
				MOVC A,@A+DPTR
				MOVX @R0,A
					MOV R0,#00CH
					;MOV A,#MAX_PWR_L
					INC DPTR
					CLR A
					MOVC A,@A+DPTR
					MOVX @R0,A
						MOVX @R0,A
		;------------------------	
		CALL SEND_PCK
		CALL WAIT_4_5_MS
		CALL SEND_PCK

		;SJMP $















SNDU:
;JB BTN0,$
	;WAIT TIMER 5
			MOV SFRPAGE,#010H
	SETB TMR5CN0_TR5	;START TIMER 4
			

	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
		JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
		JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
	JNB TMR5CN0_TF5H ,$
		CLR TMR5CN0_TF5H
		   MOV SFRPAGE,#000H




WAIT_LINK:	;WAIT FOR ACK FROM FIRE CONTROLER
	  CALL WAIT_4_5_MS
	  CALL SEND_PCK
;SJMP AVOID
	  JNB ACK,WAIT_LINK
	  ;	SETB BLU
	  SETB REL
;SJMP $
	  AVOID:
	  
	  
							;	ER



	BUTON:
;setb red
CLR TST2
		CALL WAIT_4_5_MS
		;CALL WAIT_20_MS
		;JNB BTN0,BUTON
;CALL VCC_DET
SJMP SELECT_PROGRAM_1		
			CALL ADC_POT
SJMP SELECT_PROGRAM
	SELECT_POWER_1:
		JB SW1,SELECT_POWER_2
			;------------------------
			MOV DPTR,# PLVL_1
				MOV R0,#00BH
				;MOV A,#MAX_PWR_H
				CLR A
				MOVC A,@A+DPTR
				MOVX @R0,A
					MOV R0,#00CH
					;MOV A,#MAX_PWR_L
					INC DPTR
					CLR A
					MOVC A,@A+DPTR
					MOVX @R0,A
						MOVX @R0,A
			;------------------------
			SJMP SELECT_PROGRAM_1
	SELECT_POWER_2:
		JB SW2,SELECT_PROGRAM
			;------------------------
			MOV DPTR,# PLVL_2
				MOV R0,#00BH
				;MOV A,#MAX_PWR_H
				CLR A
				MOVC A,@A+DPTR
				MOVX @R0,A
					MOV R0,#00CH
					;MOV A,#MAX_PWR_L
					INC DPTR
					CLR A
					MOVC A,@A+DPTR
					MOVX @R0,A
						MOVX @R0,A
			;------------------------			
			SJMP SELECT_PROGRAM_1
	
	
	SELECT_PROGRAM:	
		;JNB BTN0,SELECT_MAX_PWR
		;DACA ESTE PESTE 08FF,MERGEM PE PUTERE REGLABILA		
			;INCREMENT ONE VALUE OF BUFFER TO SEE RESULTS
	   	SJMP POR_REG_PWR
	
		MOV A,ADC0L
		MOV B,#POT_PWR_SEL_L
		CLR PSW_CY
		SUBB A,B
			MOV A,ADC0H
			MOV B,#POT_PWR_SEL_H
			SUBB A,B
			JNB PSW_CY,POR_REG_PWR
				CALL POT_MENU_SELECT
					SJMP SELECT_PROGRAM_1
		
		POR_REG_PWR:
;			SETB RED
;			CLR BLU
			SETB REL
		MOV DPTR,#POWER_LINEAL
			MOV A,ADC0H
			MOVC A,@A+DPTR
			;--------------
				MOV A,ADC0H
				CLR PSW_CY
				RRC A
				MOV R2,A
				MOV A,ADC0L
				RRC A
				MOV R3,A
					MOV A,R2
					ADD A,#002H
					MOV R2,A
	
	
	;SJMP SELECT_PROGRAM_1
			MOV R0,#00BH
			;MOV A,ADC0H
			MOV A,R2 
			MOVX @R0,A
				MOV R0,#00CH
				;MOV A,#000H
				;MOV A,ADC0L
				MOV A,R3
				
				
				ANL A,#11110000B	;STER GULTIMILE VALORI NESEMNIFICATIVE
				MOVX @R0,A
		 			SJMP SELECT_PROGRAM_1
		 SELECT_MAX_PWR:
		    SETB RED
			CPL BLU
			SETB REL
			
			MOV R0,#00BH
			MOV A,#MAX_PWR_H
			MOVX @R0,A
				MOV R0,#00CH
				MOV A,#MAX_PWR_L
				MOVX @R0,A
		
				SJMP SELECT_PROGRAM_1
		
		
		SELECT_PROGRAM_1:	
		CALL SEND_PCK
		CALL WAIT_4_5_MS
		CALL SEND_PCK


SETB TST2
;clr red
;		JNB ACK,SNDU
;		ORL RSTSRC,#010H
;		CALL READ_RF_STAT
JMP SNDU

;**************************************
;	DETECTEZ NIVELUL D ETENSIUNE IN COIL LA VDD
;**************************************
VCC_DET:
	NOP
	;10010
	mov  ADC0MX,    #10010B	;P2.5 INPUT ADC	
	CLR ADC0CN0_ADWINT
	SETB ADC0CN0_ADBUSY
	JNB ADC0CN0_ADINT,$
		CLR ADC0CN0_ADWINT
		CLR ADC0CN0_ADINT
   	SETB ADC0CN0_ADBUSY
	JNB ADC0CN0_ADINT,$
		CLR ADC0CN0_ADWINT
		CLR ADC0CN0_ADINT
	SETB ADC0CN0_ADBUSY
	JNB ADC0CN0_ADINT,$
		CLR ADC0CN0_ADINT
	JB ADC0CN0_ADWINT,VCC_DET_A
 RET	
	
	
	
	
		VCC_DET_A:
		JNB OV_V,VCC_DET_B
		;------------------------
		VCC_DET_A1:
			MOV DPTR,# PLVL_2
				MOV R0,#00BH
				;MOV A,#MAX_PWR_H
				CLR A
				MOVC A,@A+DPTR
				MOVX @R0,A
					MOV R0,#00CH
					;MOV A,#MAX_PWR_L
					INC DPTR
					CLR A
					MOVC A,@A+DPTR
					MOVX @R0,A
						MOVX @R0,A
		;---------	INCARC NOUA VALOARE DE ALARMA LVL1
		MOV DPTR,#DET_LVL_2
			CLR A
			MOVC A,@A+DPTR
			MOV ADC0GTH,A
			INC DPTR
			CLR A
			MOVC A,@A+DPTR
			MOV ADC0GTL,A
				MOV ADC0LTL,#000H
				MOV ADC0LTH,#000H
		CLR OV_V	;SEMNALEZ CA A SARIT PESTE VALOARE

RET
;-------------------------------------------------------	
		
		
		
		
		
		
		
		VCC_DET_B:	;A DEPSIT TENSIUNE DE PRAG LVL_2,DECI SO DECONECTAT REZISTENTZA,TRIMIT ORDIN SA COBOARE PUTEREA
		;------------------------
			MOV DPTR,# PLVL_1
				MOV R0,#00BH
				;MOV A,#MAX_PWR_H
				CLR A
				MOVC A,@A+DPTR
				MOVX @R0,A
					MOV R0,#00CH
					;MOV A,#MAX_PWR_L
					INC DPTR
					CLR A
					MOVC A,@A+DPTR
					MOVX @R0,A
						MOVX @R0,A
		;---------	INCARC NOUA VALOARE DE ALARMA LVL1
		MOV DPTR,#DET_LVL_1
			CLR A
			MOVC A,@A+DPTR
			MOV ADC0LTH,A
			INC DPTR
			CLR A
			MOVC A,@A+DPTR
			MOV ADC0LTL,A
				MOV ADC0GTL,#0FFH
				MOV ADC0GTH,#0FFH
		SETB OV_V	;SEMNALEZ CA A SARIT PESTE VALOARE
RET
;--------------------------------------





;******************************************************
;SELECTEZ TREI OPERATII FUNCTIE DACA ESTE POT IN POZ 1,2,SAU3 IN FCT DE VAL ADC
;*****************************************************
POT_MENU_SELECT:
		MOV A,ADC0L
		MOV B,#POT_OFF_LVL_L
		CLR PSW_CY
		SUBB A,B
			MOV A,ADC0H
			MOV B,#POT_OFF_LVL_H
			SUBB A,B
			JB PSW_CY,POT_A	
		;====================================
		MOV R0,#00BH
			MOV A,#POFF_PWR_H
			MOVX @R0,A		   
				MOV R0,#00CH
				MOV A,#POFF_PWR_L
				MOVX @R0,A
			;========================
			SETB BLU 
			CLR RED	
		CLR REL		  
RET
		POT_A:
		MOV A,ADC0L
		MOV B,#POT_A_LVL_L
		CLR PSW_CY
		SUBB A,B
			MOV A,ADC0H
			MOV B,#POT_A_LVL_H
			SUBB A,B
			JB PSW_CY,POT_B	
		;====================================
			MOV R0,#00BH
			MOV A,#PA_PWR_H
			MOVX @R0,A		   
				MOV R0,#00CH
				MOV A,#PA_PWR_L
				MOVX @R0,A
			;========================
			CPL BLU
			;SETB BLU
			SETB REL		  
RET
		POT_B:
		MOV A,ADC0L
		MOV B,#POT_B_LVL_L
		CLR PSW_CY
		SUBB A,B
			MOV A,ADC0H
			MOV B,#POT_B_LVL_H
			SUBB A,B
			JB PSW_CY,POT_C	
		;====================================
			MOV R0,#00BH
			MOV A,#PB_PWR_H
			MOVX @R0,A		   
				MOV R0,#00CH
				MOV A,#PB_PWR_L
				MOVX @R0,A
			;========================
			
			CPL RED
			CLR BLU
			SETB REL
RET
		POT_C:
;		MOV A,ADC0L
;		MOV B,#POT_C_LVL_L
;		CLR PSW_CY
;		SUBB A,B
;			MOV A,ADC0H
;			MOV B,#POT_C_LVL_H
;			SUBB A,B
;			JB PSW_CY,POT_X	
		;====================================
		MOV R0,#00BH
			MOV A,#PC_PWR_H
			MOVX @R0,A		   
				MOV R0,#00CH
				MOV A,#PC_PWR_L
				MOVX @R0,A
			;========================
		SETB RED
		SETB BLU
		SETB REL
		POT_X:
RET



;******************************************************
;READ POTENCIOMETER VALUES
;*****************************************************
;READ_POT:
	






;CALL BUFER_2_UART	
	
	
	





;**********************************************
;	TRIMIT DATELE DIN BUFER LA PORTUL SERIAL
;**********************************************
BUFER_2_UART:
	MOV DPTR,#SPACE
	CALL SENDSTRING
	MOV R0,#020H
	BUFER_2_UART_1:
		MOVX A,@R0
		CALL SENDVAL
		INC R0
		MOV A,R0
		CJNE R0,#040H,BUFER_2_UART_1
RET
;_______________________________________________




;*****************************************************************
;		TRIMIT PAKET SI ASTEPT ACK
;*****************************************************************8
SEND_PCK:		
;	SJMP SEND_FIFO
	CALL READ_RF_STAT
	TST_0E:
		CJNE A,#00EH,TST_1E
			JMP SEND_FIFO
	TST_1E:
	CJNE A,#01EH,TST_20
		MOV A,#WR_STS
		MOV B,#01EH			;STERG FLAG DE TOT
		CALL WR_RF_REG
			CALL FLUSH_RX
			CALL FLUSH_TX
			JMP SEND_FIFO
	TST_20:
	CJNE A,#02EH,TST_60
			MOV A,#WR_STS
			MOV B,#01EH			;STERG FLAG DE TXACK,NO PAYLOAD
			CALL WR_RF_REG
			JMP SEND_FIFO	
	TST_60:
	CJNE A,#060H,TST_XX
;			MOV A,#WR_STS
;			MOV B,#0FFH
;			CALL WR_RF_REG
;			CALL RD_FIFO_2_XRAM_32	
	 	SJMP SEND_FIFO_RECIVED_DATA
	TST_XX:
	;STERG ORICE POSIBIL FLAG
	MOV A,#WR_STS
	MOV B,#0FFH
	CALL WR_RF_REG
;
CALL FLUSH_TX
;CALL FLUSH_RX
;setb blu
CLR ACK
RET	
	SEND_FIFO:				;INCARC FIFO
;clr blu
		 CALL WAIT_75_US
			CLR RF_CE
		CALL TX_XRAM_2_BUF	
			SETB RF_CE
		CALL WAIT_220_US
		CALL READ_RF_STAT	
			

;		CALL WAIT_4_5_MS

		JB IRQ_R,$
		CALL READ_RF_STAT
		CJNE A,#02EH,SEND_FIFO_OUT
			MOV A,#WR_STS
			MOV B,#02EH			;STERG FLAG DE TXACK,NO PAYLOAD
			CALL WR_RF_REG

			CALL FLUSH_TX

			MOV A,#WR_STS
			MOV B,#0FEH			;STERG FLAG DE TXACK,NO PAYLOAD
			CALL WR_RF_REG
		SETB ACK
RET
			SJMP SEND_FIFO_RECIVED_DATA
	SEND_FIFO_OUT:
	CLR ACK
RET	

	SEND_FIFO_RECIVED_DATA:
		CALL RD_FIFO_2_XRAM_32
		MOV A,#WR_STS
		MOV B,#060H			;STERG FLAG DE TOT
		CALL WR_RF_REG
		
		CALL FLUSH_RX
			MOV R0,#010H
			DJNZ R0,$
					  
		MOV A,#WR_STS
		MOV B,#060H			;STERG FLAG DE TOT
		CALL WR_RF_REG
		SETB ACK
;		CALL RX_FIFIO_2_UART
RET
;___________________________________________________________________





;RUTINAS PLM
CALL WR_TX_ADR
CALL RD_TX_ADR
CALL RD_RX_ADR_P0

CLR RF_SEL
MOV A,#027H
	CALL WR_BYT
MOV A,#0FFH
	CALL WR_BYT
SETB RF_SEL

CALL READ_RF_STAT

CALL READ_RSS	
	
	
	SJMP $




		WAIT_SIGNAL:
;		JB CSN,$
		CALL RD_SOFT_SPI
;		JNB CSN,$
		MOVX @DPTR,A
		INC DPTR
;	cpl dbg
	SJMP WAIT_SIGNAL




;***********************************************************88
;		SOFT SPI BYTE
;**************************************************************
RD_SOFT_SPI:
	MOV R0,#008H
	MOV A,#000H
	RD_SOFT_SPI_1:
	JNB CLK,$
;SETB DBG
;CPL DBG
	;____________	;RISING EDGE
	MOV C, MISO
	RLC A
	DJNZ R0,RD_SOFT_SPI_3	;SA FIE OPT BITI
		JB CLK,$
;CLR DBG
RET
	
	
	
	RD_SOFT_SPI_3:
   		JB CLK,$	;ASTEPT SA TREACA IN ZERO CLOCKUL
		SJMP RD_SOFT_SPI_1	;ASTEPT URMATORUL EDGE
;______________________________________________________________

















SPI_2_XRAM:
	MOV DPL,#000H
	MOV DPH,#000H
	SPI_2_XRAM_1:
	CALL WAIT_SPI_BYTE
	MOVX @DPTR,A
	INC DPTR
		MOV A,DPH
		CJNE A,#002H,SPI_2_XRAM_1
	NOP
	SJMP $	





	NOP
	NOP
	NOP
WAIT_SPI_BYTE:
	JNB SPI0CN0_SPIF,$	;ASTEPT INTRERUPERE DE SPI PRIMIT
		CLR SPI0CN0_SPIF
		MOV A,SPI0DAT
RET
	SJMP WAIT_SPI_BYTE

;*****************************************************************************************************88
;		RUTINE 	SPI
;*****************************************************************************************************
;***********************************************************************************
;		SCRI0U UN BYT SPI
;***********************************************************************************
WR_BYT:
			MOV SPI0DAT,A
			JNB SPI0CN0_SPIF ,$			;CIND E PE 1 ATTERMINAT TRANSMISIA
			CLR SPI0CN0_SPIF			;IL STREG PT URMATORUL BYT
;			MOV R7,A
;			CALL _SPI_Transfer
		RET
;___________________________________________________________________________________

;***********************************************************************************
;		CITESC UN BYT SPI
;***********************************************************************************
RD_BYT:
		    MOV A,#0FFH
		    MOV SPI0DAT,A
		    JNB SPI0CN0_SPIF,$
		    MOV A,SPI0DAT
			CLR SPI0CN0_SPIF
			;MOV R7,#000H
			;CALL _SPI_Transfer
		RET    
;___________________________________________________________________________________

;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

;************************************************************************
READ_RF_STAT:
	CLR RF_SEL
		MOV A,#007H
			CALL WR_BYT
			CALL RD_BYT
	SETB RF_SEL
RET
;_______________________________

;_________________________________________________________________
FLUSH_TX:
	CLR RF_SEL
	MOV A,#0E1H
		CALL WR_BYT
	MOV A,#0FFH
		CALL RD_BYT
	SETB RF_SEL
RET
;__________________________________________________________________

;_________________________________________________________________
FLUSH_RX:
	CLR RF_SEL
	MOV A,#0E2H
		CALL WR_BYT
	MOV A,#0FFH		;NOP CITESC STATUS
		CALL WR_BYT	
	SETB RF_SEL
RET
;__________________________________________________________________



;******************************************************************
WR_TX_ADR:
	CLR RF_SEL
	MOV A,#030H
		CALL WR_BYT
	;_______________
	MOV A,T_ADR_4
		CALL WR_BYT
	MOV A,T_ADR_3
		CALL WR_BYT
	MOV A,T_ADR_2
		CALL WR_BYT
	MOV A,T_ADR_1
		CALL WR_BYT
	MOV A,T_ADR_0
		CALL WR_BYT
	
	
	SETB RF_SEL
RET
;___________________________________________

;*******************************************
WR_RX0_ADR:
	CLR RF_SEL
	MOV A,#02AH
		CALL WR_BYT
	;_______________
	MOV A,R_ADR_4
		CALL WR_BYT
	MOV A,R_ADR_3
		CALL WR_BYT
	MOV A,R_ADR_2
		CALL WR_BYT
	MOV A,R_ADR_1
		CALL WR_BYT
	MOV A,R_ADR_0
		CALL WR_BYT
	
	
	SETB RF_SEL
RET
;___________________________________________




;*******************************************
RD_TX_ADR:
	CLR RF_SEL
		MOV A,#10H
			CALL WR_BYT
		;_____________________
		CALL RD_BYT
			MOV T_ADR_4,A
		CALL RD_BYT
			MOV T_ADR_3,A
		CALL RD_BYT
			MOV T_ADR_2,A
		CALL RD_BYT
			MOV T_ADR_1,A
		CALL RD_BYT
			MOV T_ADR_0,A
		SETB RF_SEL
RET
;______________________________________

;*******************************************
RD_RX_ADR_P0:
	CLR RF_SEL
		MOV A,#00AH
			CALL WR_BYT
		;_____________________
		CALL RD_BYT
			MOV R_ADR_4,A
		CALL RD_BYT
			MOV R_ADR_3,A
		CALL RD_BYT
			MOV R_ADR_2,A
		CALL RD_BYT
			MOV R_ADR_1,A
		CALL RD_BYT
			MOV R_ADR_0,A
		SETB RF_SEL
RET
;______________________________________

;****************************************
READ_RSS:
	CLR RF_SEL
	MOV A,#009H		;RFD
	CALL WR_BYT
	CALL RD_BYT
	SETB RF_SEL
RET
;_____________________________

;*****************************************
;		SCRIU REGISTRU DE 1 BYTE  ADRESA E IN A,VALOAREA IN B
;*****************************************
WR_RF_REG:
	CLR RF_SEL
		ORL A,#00100000B	;CA SA IL TREC IN MODUL DE SCRIERE 001A AAAA
		CALL WR_BYT
		MOV A,B
		CALL WR_BYT
	SETB RF_SEL
RET
;________________________________

;*****************************************
;	CITESC DATELE DIN FIFO 32 BITI SI SALVEZ IN XRAM 0X20-0X3F
;*****************************************
RD_FIFO_2_XRAM_32:
	CLR RF_SEL
   		MOV R0,#020H	;POINTER PT RX BUFER XRAM
		MOV A,#061H
		CALL WR_BYT
		RD_FIFO_2_XRAM_32_1:
			CALL RD_BYT
			MOVX @R0,A
			INC R0
			MOV A,R0
			CJNE A,#040H,RD_FIFO_2_XRAM_32_1
	SETB RF_SEL
RET
;____________________________________

;*****************************************
;	SCRIU 32 BITI DIN XRAM 0X00-0X1F IN BUFER TX 
;*****************************************
TX_XRAM_2_BUF:
	CLR RF_SEL
		MOV R0,#000H
		MOV A,#0A0H
		CALL WR_BYT
		TX_XRAM_2_BUF_1:
			MOVX A,@R0
			CALL WR_BYT
			INC R0
			MOV A,R0
			CJNE A,#020H,TX_XRAM_2_BUF_1
		SETB RF_SEL
RET
;________________________________________


;************************************************************		
;	ASTEPT 5 MS		
;***********************************************************8
WAIT_75_US:
;CLR DBG
	MOV DPL,#000H
	MOV DPH,#000H
	WAIT_75_US_1:
		INC DPTR
		MOV A,DPH
		CJNE A,#001H,WAIT_75_US_1
;SETB DBG
RET
;______________________________________________________________

;************************************************************		
;	ASTEPT 5 MS		
;***********************************************************8
WAIT_220_US:
CLR DBG
	MOV DPL,#000H
	MOV DPH,#000H
	WAIT_220_US_1:
		INC DPTR
		MOV A,DPH
		CJNE A,#003H,WAIT_220_US_1
SETB DBG
RET
;______________________________________________________________			
		



;************************************************************		
;	ASTEPT 5 MS		
;***********************************************************8
WAIT_4_5_MS:
;CLR DBG
	MOV DPL,#000H
	MOV DPH,#000H
	WAIT_4_5_MS_1:
		INC DPTR
		MOV A,DPH
		CJNE A,#03EH,WAIT_4_5_MS_1
;SETB DBG
RET
;______________________________________________________________	


;************************************************************		
;	ASTEPT 5 MS		
;***********************************************************8
WAIT_20_MS:
;CLR DBG
	MOV DPL,#000H
	MOV DPH,#000H
	WAIT_20_MS_1:
		INC DPTR
		MOV A,DPH
		CJNE A,#0f8H,WAIT_20_MS_1
;SETB DBG
RET
;______________________________________________________________	







;*************************************************************
;	MEASURE POTENCIOMETER LEVEL
;*************************************************************
ADC_POT:
PUSH ADC0MX
	mov  ADC0MX,    #01011B	;P1.5 INPUT ADC	
	SETB ADC0CN0_ADBUSY
	JNB ADC0CN0_ADINT,$
		CLR ADC0CN0_ADINT
   	SETB ADC0CN0_ADBUSY
	JNB ADC0CN0_ADINT,$
		CLR ADC0CN0_ADINT
	SETB ADC0CN0_ADBUSY
	JNB ADC0CN0_ADINT,$
		CLR ADC0CN0_ADINT




;	JNB ADC0CN0_ADINT,$
;		CLR ADC0CN0_ADINT						  
;	MOV A,ADC0H
;		CALL SENDVAL
;	MOV A,ADC0L
;		CALL SENDVAL
;	JNB ADC0CN0_ADINT,$
;		CLR ADC0CN0_ADINT
;	JNB ADC0CN0_ADINT,$
;		CLR ADC0CN0_ADINT

POP ADC0MX
RET
;_______________________________________________________


;******************************************************
;	TIMER 2 ISR
;******************************************************
TIMER_2_ISR:
	CLR TMR2CN0_TF2H	;CLEAR FLAG
	INC T2_TOT
	MOV A,T2_TOT
	CJNE A,#T2_TICK,TIMER_2_ISR_1
	;--------------	STOP PWM BUZZER
	mov  PCA0CPM0,  #000h
	MOV  T2_TOT,#000H
RET
	TIMER_2_ISR_1:
RET
;_____________________________________________________

;******************************************************
;	TIMER 3 ISR
;******************************************************
TIMER_3_ISR:	;PENTRU GENERAT 50HZ PWM
	ANL TMR3CN0,#01111111B	;CLEAR FLAG
	PWM_UNU:
	JNB PWM0,PWM_ZERO
		CLR PWM0
			NOP
		SETB PWM1
RET	
	
	PWM_ZERO:
		CLR PWM1
			NOP
		SETB PWM0
RET	
;================================================







;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;
;			RUTI0NE UART
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

;____________________________________________________________________
                                                         ; SENDSTRI0NG

SENDSTRING:     ; sends ASCII stRI0ng to UART starTI0ng at locaTI0on
                ; DPTR and ending with a null (0) value

        PUSH    ACC
        PUSH    B
        CLR     A
        MOV     B,A
IO0010: CLR     A
        MOVC    A,@A+DPTR
        INC     DPTR
		JZ      IO0020
        CALL    SENDCHAR
        JMP     IO0010
IO0020: POP     B
        POP     ACC

        RET

;____________________________________________________________________
                                                           ; SENDCHAR

SENDCHAR:       ; sends ASCII value contained in A to UART
;CLR RED
        JNB     SCON0_TI,$            ; wait TI0l present char gone
        CLR     SCON0_TI             ; must clear TI0
        MOV     SBUF0,A
        RET
;SETB RED
;____________________________________________________________________
                                                            ; SENDVAL

SENDVAL:        ; converts the hex value of A into two ASCII chars,
		; and then spits these two characters up the UART.
                ; does not change the value of A.

        PUSH    ACC
        SWAP    A
        CALL    HEX2ASCII
        CALL    SENDCHAR        ; send high nibble
        POP     ACC
        PUSH    ACC
        CALL    HEX2ASCII
        CALL    SENDCHAR        ; send low nibble
        POP     ACC

        RET

;____________________________________________________________________
                                                          ; HEX2ASCII

HEX2ASCII:      ; converts A into the hex character represenTI0ng the
                ; value of A's least significant nibble

        ANL     A,#00Fh
        CJNE    A,#00Ah,$+3
        JC      IO0030
        ADD     A,#007h
IO0030: ADD     A,#'0'

        RET

;____________________________________________________________________
                                                          ; ASCII2HEX

ASCII2HEX:      ; converts A from an ASCII digit ('0'-'9' or 'A'-'F')
                ; into the corresponding number (0-15).  returns C=1
                ; when input is other than an ASCII digit,
                ; indicaTI0ng invalid output (returned as 255).

        CLR     C
        SUBB    A,#'0'
        CJNE    A,#10,$+3
        JC      IO0050          ; if '0'<=char<='9', return OK
        CJNE    A,#17,$+3
        JC      IO0040          ; if '9'<char<'A', return FAIL
        SUBB    A,#7
        CJNE    A,#10h,$+3
        JC      IO0050          ; if 'A'<=char<='F', return OK
        CJNE    A,#42,$+3
        JC      IO0040          ; if 'F'<char<'a', return FAIL
        SUBB    A,#20h
        CJNE    A,#10h,$+3
        JC      IO0050          ; if 'a'<=char<='f', return OK..

IO0040: CLR     C               ; ..else return FAIL
        MOV     A,#0FFh

IO0050: CPL     C
        RET

;____________________________________________________________________
                                                            ; GETCHAR

GETCHAR:        ; waits for a single ASCII character to be received
                ; by the UART.  places this character into A.
        JNB     SCON0_RI,$
        MOV     A,SBUF0
        CLR     SCON0_RI
        RET

;____________________________________________________________________
                                                             ; GETVAL

GETVAL:         ; waits for two ASCII hex digits to be received by
                ; the UART.  returns the hex value in A.

        PUSH    B
        PUSH    0
IO0060: CLR     SCON0_RI
        CALL    GETCHAR         ; first nibble
        MOV     0,A             ; store received char
        CALL    ASCII2HEX
        JC      IO0060          ; if not '0' thru 'F', don't accept
        SWAP    A               ; swap nibbles
        MOV     B,A             ; store nibble in B
        MOV     A,0             ; echo received char
        CALL    SENDCHAR
IO0070: CLR     SCON0_RI
        CALL    GETCHAR         ; second nibble
        MOV     0,A             ; store received char
        CALL    ASCII2HEX
        JC      IO0070          ; if not '0' thru 'F', don't accept
        ORL     A,B             ; combine nibbles
        MOV     B,A             ; store results in B
        MOV     A,0             ; echo received char
        CALL    SENDCHAR
        MOV     A,B             ; final result
        POP     0
        POP     B

        RET

SPACE:				;SEMNUL DE SPACE
	DB 10,13,0
;__________________________________________________________________________________________


TXT_TEST:
DB	10,13
DB	'TEST RESPUESTA',10,13
DB	'	VELOCIDAD UART',10,13,0


;*******************************************************************************
;	CRC_16_MODBUS LA INCEPT RESET D_CRC_X,VALOAREA BYTE IN A,,SE REPETA PT TOTI BITII
;**************************************************************************
CRC16_Init:                             ; Reset CRC Registar
        MOV D_CRC16_Hi, #0FFh
        MOV D_CRC16_Lo,#0FFh
        RET
;__________________________________________________________________________________________

CRC16_AddCalc:                     ; Calc CRC
        PUSH 000H                       ; Save Reg 0
        PUSH Acc                       ; Save Acc
        MOV     R0,#08h                 ; 8 Bits
        XRL     D_CRC16_Lo,A         ; Lo ^=Data
L_P1:
        CLR     C                         ; Clear Carry
        MOV     A,D_CRC16_Hi        ; D_CRC << 1
        RRC     A                         ; Shift Left
        MOV     D_CRC16_Hi,A        ; Store Back
        MOV     A,D_CRC16_Lo        ; Get Hi Byte
        RRC     A                         ; Shift Left
        MOV     D_CRC16_Lo,A        ; Store Back
        JNC     L_P2                      ; Skip if Bit 15 wasn't set
        XRL     D_CRC16_Hi,#0A0h   ; XOR in Polynomial High
        XRL     D_CRC16_Lo,#01h     ; XOR in Polynomial Lo
L_P2:
        DJNZ    R0,L_P1
        POP     Acc
        POP     000H	;RECOVER R0
        RET
;__________________________________________________________________________________________
;__________________________________________________________________________________________


POWER_LINEAL:	;INTRE 0200 SI 	OCB0  ADICA 0C00
;DB	

;00
DB	001H
;01
DB	001H
;02
DB	001H
;03
DB	002H
;04
DB	002H
;05
DB	003H
;06
DB	004H
;07
DB	005H
;08
DB	006H

;09
DB	007H
;0A
DB	008H
;0B
DB	009H
;0C
DB	00AH
;0D
DB	00BH
;0E
DB	00CH
;0F
DB	00DH

DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTODB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTODB 002H, 002H,002H,002H	;ASTA ASHA DE MISTODB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
DB 002H, 002H,002H,002H	;ASTA ASHA DE MISTO
								  






;************************************************************************************
;	CONFIGURATIA INITIALA DE LA TOOL CARE TRIMITE LA PRIMUL CONTACT
;************************************************************************************
TOOL_INIT_STRING:
DB	01FH,00AH,000H,010H,000H,000H,011H,036H,000H,002H,0A2H,000H,000H,000H,000H,000H
DB	000H,000H,000H,000H,000H,000H,000H,000H,000H,000H,000H,000H,000H,0B8H,05AH,0CDH



;---------------	buffer content
;a0
;B0	PVR_H
;C0	PVR_L



;*************************************************************88
;	CONFIGURATIE NRF
;***************************************************************
CNF_DATA:


DEFAULT_ADRESS:
;DB	011H,036H,000H,002H,0A2H
;DB	011H,036H,000H,002H,0A2H


DB	0F7H,0F7H,0F7H,0F7H,0F7H	;ADRESA CU CARE INCEPE APEL GENERAL	TX
DB	0F7H,0F7H,0F7H,0F7H,0F7H	;ADRESA CU CARE INCEPE APEL GENERAL	 PIPE 0





ORG 1800H

PLVL_0:	  		;nivel link activ
DB	007H,0d0H
PLVL_1:
DB	002H,013H
PLVL_2:
DB	004H,056H

DET_LVL_1:		;00B6 LA 15V
DB 	001H,000H
DET_LVL_2:		
DB 	001H,03BH 	;0139 LA 25V

DLY_RL1:		;INTIRZIERE DE LA RELEU	,nu e cazul aicia
DB	0C0H,090H


DB	0FFH,0FFH,0FFH,0FFH


