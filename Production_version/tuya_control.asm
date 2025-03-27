;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;========================================================================
;	TUYA REMOTE CONTROL
;========================================================================
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
TUYA_RMOTE_CONTROL:
	CLR TUYA_MSG
	CALL BUZZING
;	CALL DISPLAY_INIT	
	MOV SG3,#050H
	MOV SG4,#000H
	MOV SG5,#039H 	;r ct DISPLAY 4
	MOV SG6,#078H
;________________________
;	A=01 B=02 C=04 D=08
;	E=10 F=20 G=40 DP=80

;	MOV SG0,#006H  ;UNU
;	MOV SG1,#05BH 	;DOI
;	MOV SG2,#04FH	;TREI

   	MOV SG0,#000H  ;ZERO
	MOV SG1,#000H 	;ZERO
	MOV SG2,#000H	;ZERO
	MOV SG7,#000H	  ;01=PWRON,02?,04=WIFI,08=+,10=-,20=F3,40=F2,
		CALL WRT_LED_8_DATA
	;_________________	SETEZ LUMOZITATE MAXIMA
		MOV A,#08FH
	CALL WRT_BYTE_MCW_START
		CALL WRT_BYTE_MCW
	CALL WRT_BYTE_MCW_STOP


	CALL SPIRIT_SPACE_FORMAT
;___________________________________________
	MOV DPTR,#SPIRIT_TABLE
		MOV SPR_AD,#0C0H	;SPIRIT START ADDRES	

;	MOV FOC_A,#010H
;	MOV FOC_B,#000H
;		CALL SEND_CMD
	
	
	MOV INST,#060H
	MOV P1T	,#000H	  
	MOV P0T,#001H

		CALL SEND_COMMAND



	call wait_400ms
	
	
	
	
	
	
	CALL SYSTEM_CHECK
		mov r0,#000h
		djnz r0,$
	CALL SYSTEM_CHECK
		mov r0,#000h
		djnz r0,$






;	CALL SYSTEM_CHECK

	CLR TOT4	;DE MOMENT PT TIMOUT KEYBOARD
			MOV SFRPAGE,#010H
	SETB TMR4CN0_TR4	;START TIMER 4
			MOV SFRPAGE,#000H
	
	
	
	
	
	WAIT_CONFIRM_TY:
	JNB KEYT,WAIT_CONFIRM_TY
		CLR KEYT
	WAIT_CONFIRM_TY_1:
		MOV A,KEY_H
		CJNE A,#000H,WAIT_CONFIRM_TY_2
			MOV A,KEY_L
			CJNE A,#001H,WAIT_CONFIRM_TY_2
				SJMP WAIT_CONFIRM_TY_ON
	WAIT_CONFIRM_TY_2:
		MOV A,KEY_H
		CJNE A,#002H,WAIT_CONFIRM_TY
			MOV A,KEY_L
			CJNE A,#000H,WAIT_CONFIRM_TY
				MOV TY_OUTA,#0F0H
				MOV TY_OUTB,#000H
				CALL SEND_MSG_TO_COP_A
					MOV R0,#000H
					DJNZ R0,$
					ORL RSTSRC,#010H
	
	WAIT_CONFIRM_TY_ON:



CLR TOT4	;DE MOMENT PT TIMOUT KEYBOARD
			MOV SFRPAGE,#010H
	SETB TMR4CN0_TR4	;START TIMER 4
			MOV SFRPAGE,#000H

	;trimit mesajul de 6f01h
		MOV TY_OUTA,#0F0H
		MOV TY_OUTB,#001H
		CALL SEND_MSG_TO_COP_A
		CALL BUZZING
;MOV R0,#000H
; DJNZ R0,$
; SJMP $
;		JMP TEST_100

;--------------------------------------------------------------------------
TUYA_SELECT_OP:
;NU MAI ACCEPT ALTE ORDINE DEZACTIVEZ PMA
	JNB TUYA_MSG,TUYA_SELECT_OP_A
		CLR TUYA_MSG
		CALL BUZZING
			sjmp TEST_100
	TUYA_SELECT_OP_A:
	JNB ST_UPD,TUYA_SELECT_OP
		CLR ST_UPD
		;============	VERIFICA DACA AM ERORI
			MOV a,FOC1A
				ANL A,#11110000B
				CJNE A,#0E0H,T_E_F2
					CALL TY_SND_ERR_F1
					SJMP TUYA_SELECT_OP
		T_E_F2:

		TUYA_SELECT_OP_B:
			SJMP TUYA_SELECT_OP

CALL BUZZING
	TEST_100:
	TEST_101:	;PORNIT OPRIT FOC
		MOV A,TY_H
		CJNE A,#101D,TEST_102
			CALL TY_SEL_F1
		JMP TUYA_SELECT_OP

	TEST_102:	;PORNIT OPRIT FOC
		MOV A,TY_H
		CJNE A,#102D,TEST_103
			CALL TY_SEL_F2
		JMP TUYA_SELECT_OP
	TEST_103:
	
	




;======================		POWER CONTROL INDUCTION	
	TEST_104:
		MOV A,TY_H
		CJNE A,#104D,TEST_105
			CALL TY_SET_PWR_F1
		JMP TUYA_SELECT_OP
	
	TEST_105:
		MOV A,TY_H
		CJNE A,#105D,TEST_107
			CALL TY_SET_PWR_F2
		JMP TUYA_SELECT_OP	



;======================		


	TEST_107:
		MOV A,TY_H
		CJNE A,#107D,TEST_111
			CALL TY_SET_MOD_F1
		JMP TUYA_SELECT_OP






	TEST_111:
		MOV A,TY_H
		CJNE A,#111D,TEST_123
			ORL RSTSRC,#010H	;PLM RSET TOTAL



	TEST_123:
		MOV A,TY_H
		CJNE A,#123D,TEST_255
		CALL TY_CLEAN
		JMP TUYA_SELECT_OP
	
	
	TEST_255:
	;DUMB BYTE
	MOV A,#055H

;	CALL SENDCHAR1

	JMP TUYA_SELECT_OP
;SINGURA IESIRE ESTE CU RESET,ADICA POWER_OFF GENERAL
;II SPUN LA COPROCESOR SA II TRIMITA TOTUL OPRIT,DUPA CE VERIFICA PLM


TY_SELECT_FIREONOFF:
	TY_SEL_F1:
		MOV A,TY_L
		CJNE A,#001H,TY_SEL_F1_OFF
		;_____________________________________
			MOV A,#001H
			ANL A,#00001111B
			ORL A,#00010000B	;VITRO INSTRUCITON-1X
			MOV INST,A
			;MOV P1T,#001H		;COMMAND FOR START DETECTION
			MOV P1T,#080H		;COMMAND FOR START DETECTION
		CALL SEND_COMMAND	
			;ASTEPT DETECTIA...PLM
			;GET STATUS
			MOV TY_OUTA,#11H
			MOV TY_OUTB,#01H
			CALL SEND_MSG_TO_COP_A

		NOP
	RET
	TY_SEL_F1_OFF:
		;_____________________________________
			MOV A,#001H
			ANL A,#00001111B	;DELETE INSTRUCTION
			MOV FOC_A,A
			MOV FOC_B,#000H
			CALL SEND_CMD
			;PROBABIL AM S ACAUT SI STATUSUL
			MOV TY_OUTA,#01H
			MOV TY_OUTB,#02H
			CALL SEND_MSG_TO_COP_A
	RET
	;==================================================

	TY_SEL_F2:
		MOV A,TY_L
		CJNE A,#001H,TY_SEL_F2_OFF
		;_____________________________________
			MOV A,#002H
			ANL A,#00001111B
			ORL A,#00010000B	;VITRO INSTRUCITON-1X
			MOV INST,A
			;MOV P1T,#001H		;COMMAND FOR START DETECTION
			MOV P1T,#080H		;COMMAND FOR START DETECTION
		CALL SEND_COMMAND	
			;ASTEPT DETECTIA...PLM
			;GET STATUS
			MOV TY_OUTA,#12H
			MOV TY_OUTB,#01H
			CALL SEND_MSG_TO_COP_A

		NOP
	RET
	TY_SEL_F2_OFF:
		;_____________________________________
			MOV A,#002H
			ANL A,#00001111B	;DELETE INSTRUCTION
			MOV FOC_A,A
			MOV FOC_B,#000H
			CALL SEND_CMD
			;PROBABIL AM S ACAUT SI STATUSUL
			MOV TY_OUTA,#02H
			MOV TY_OUTB,#02H
			CALL SEND_MSG_TO_COP_A
	RET
	;==================================================
















	TY_SEL_F3:
TY_SELECT_FIREONOFF_OUT:
RET
;================================================

;-----------------------------------------------
;	SET POWER TO FIRE X
;-----------------------------------------------
	TY_SET_PWR_F1:
		MOV FOC_A,#011H
		MOV A,TY_L
		MOV FOC_B,A
		CALL SEND_CMD
RET
;_______________________________________________

;-----------------------------------------------
;	SET POWER TO FIRE X
;-----------------------------------------------
	TY_SET_PWR_F2:
		MOV FOC_A,#012H
		MOV A,TY_L
		MOV FOC_B,A
		CALL SEND_CMD
RET
;_______________________________________________

TY_SET_MOD_F1:
		;SEND STOP FIRE COMMAND		
		MOV FOC_A,#001H
		MOV A,#000H
		MOV FOC_B,A
		CALL SEND_CMD
			MOV R0,#000H
			DJNZ R0,$
TY_SET_MOD_F1_WIFI:
		MOV A,TY_L
		CJNE A,#001H,TY_SET_MOD_F1_IND
			MOV FOC_A,#021H
			MOV A,#000H
			MOV FOC_B,A
			CALL SEND_CMD
				MOV TY_OUTA,#21H
				MOV TY_OUTB,#00H
				CALL SEND_MSG_TO_COP_A
					SJMP TY_SET_MOD_F1_OUT 
TY_SET_MOD_F1_IND:		
			MOV TY_OUTA,#01H
			MOV TY_OUTB,#00H
			CALL SEND_MSG_TO_COP_A



TY_SET_MOD_F1_OUT:
RET
;_______________________________________________







;-----------------------------------------------
;	SET POWER TO FIRE X
;-----------------------------------------------
	TY_CLEAN:
		CALL CLEANER
			;--------------------	TRIMIT MESAJ CA AM TERMINAT CU CLEANUL
			MOV TY_OUTA,#0F5H
			MOV TY_OUTB,#123D	;CLEAN HEADER DP
			MOV TY_OUTC,#000H	;ESTE STOP .CLEANER OFF
			CALL SEND_MSG_TO_COP_A
RET
;_______________________________________________






;-----------------------------------------------
;	SET POWER TO FIRE X
;-----------------------------------------------
	TY_SND_ERR_F1:
		MOV TY_OUTA,FOC1A
		MOV TY_OUTB,FOC1B
		CALL SEND_MSG_TO_COP_A
RET
;_______________________________________________

;-----------------------------------------------
;	send firmware version TO COPROCESOR
;-----------------------------------------------
	TY_SND_FRW_KYB:
		MOV TY_OUTA,#0F1H
		MOV DPTR,#FRW
		CLR A
		MOVC A,@A+DPTR
		MOV TY_OUTB,A	;MODEL D EFIMRWARE
			INC DPTR
			CLR A
			MOVC A,@A+DPTR
			MOV TY_OUTC,A	;MODEL D EFIMRWARE
		INC DPTR
		CLR A
		MOVC A,@A+DPTR
		MOV TY_OUTD,A	;MODEL D EFIMRWARE
				
		CALL SEND_MSG_TO_COP_A
RET
;_______________________________________________






;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
;SEND MESAGE TO TUYA COPROCESOR
;****************************************************************
SEND_MSG_TO_COP_A:
CLR IE_EA
ANL EIE1,#11111101B		;OPRESC PMA ISR
	
		CLR TY_IRQ 	;SEMNALEZ CA VINE
   ;TRE S AFAC CEVA PT TIMEOUT DIN CONFIGURARE


MOV SFRPAGE,#020H
;	CALL GETCHAR1
		MOV R0,#080H
		DJNZ R0,$
SETB TY_IRQ




	MOV A,TY_OUTA
		CALL SENDCHAR1
	MOV A,TY_OUTB
		CALL SENDCHAR1
	MOV A,TY_OUTC
		CALL SENDCHAR1
	MOV A,TY_OUTD
		CALL SENDCHAR1
MOV SFRPAGE,#000H
	
ORL EIE1,#00000010B		;PORNESC PMA ISR
SETB IE_EA
RET

SEND_MSG_TO_COP_B:
NOP
RET
;________________________________________________________________



