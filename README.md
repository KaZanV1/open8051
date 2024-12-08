Content of folder
This is an project for induction cooktop,where the main part is an EFM8BB32 MCU,an enhanced version of the venerable 8051 MCU,developed by Intel around 1980
The MCU control the main parts of the POWER_PCB,Keyboard And remote controled devices,so that is not need for wires:
--PWM frecvency and dead_time that are sent to half bridge IGB,--external modificables 
--Temperatures of coils(max---stop the power to protect the PCB,mid/low-that can be showen on Keyboard display,to warnning about hot srfaces of top--external modificables
--Temperature of IGB--over temperature protection--external modificables
--Pot detection/removal of pot--external modificables
--Uart comunication with the Keyboard or another external control device (this way have a great flexibility of control by another methid.bluetootw,wifi0esp32,IoT,mobile devices,etc)
--Internal timer if is necesary to stop the stove after a determinated time-fuly controlable by UART
--Power levels (10 levels),--external modificables
--Radio conection for cantroling the POWER_PCB from another device,that can recibe energi by the coil,like the wirelle charger of an phone,but with more power (>1200W)
--Over current and overvoltage protection--external modificables
--Posible automatic stop in case of lusing the conection with the keyboard or another contro device,by using STATUS coomands
--Modifications of all variable parameters wit one single configuration file,sent over UART


kazan_protocl_1_00== comunication protocol betwen power board and keyboard/external controler
