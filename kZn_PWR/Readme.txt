This folder contain the firmware of POWER_PCB,half bridge for induction coktop

STAND_BY_LOOP
--SNC=1
	--RUTINA DE FAN
	
		--JB ERR_V,-----------------------------------STBY_LOOP_OUT_ERROR 
	--PCB_TEST
		--JB ERR_V,-----------------------------------STBY_LOOP_OUT_ERROR 
	--COIL_TEST
		--JB ERR_V,-----------------------------------STBY_LOOP_OUT_ERROR
	--STBY_LOOP_PLATE_1	;TEST HIGH TEMP
		--JB ERR_V,-----------------------------------STBY_LOOP_OUT_ERROR
		--TEMP HIGH	-> STPW,#003H
			--JMP---------------------------------STBY_LOOP_PLATE_4
	--STBY_LOOP_PLATE_2	;TEST MID TEMP 
		--JB ERR_V,-----------------------------------STBY_LOOP_OUT_ERROR
		--TEMP MID	-> STPW,#002H
			--JMP---------------------------------STBY_LOOP_PLATE_4
	--STBY_LOOP_PLATE_3
		--JB ERR_V,-----------------------------------STBY_LOOP_OUT_ERROR
