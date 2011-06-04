# MPLAB IDE generated this makefile for use with GNU make.
# Project: phase_control.mcp
# Date: Fri May 20 18:46:22 2011

AS = MPASMWIN.exe
CC = mcc18.exe
LD = mplink.exe
AR = mplib.exe
RM = rm

phase_control.cof : main.o
	$(LD) "..\..\MCC18\lkr\18f4520i.lkr" "main.o" /u_CRUNTIME /u_DEBUG /z__MPLAB_BUILD=1 /z__MPLAB_DEBUG=1 /m"phase_control.map" /w /o"phase_control.cof"

main.o : main.c ../../MCC18/h/delays.h ../../MCC18/h/math.h ../../MCC18/h/timers.h main.c ../../MCC18/h/p18cxxx.h ../../MCC18/h/p18f4520.h ../../MCC18/h/pconfig.h
	$(CC) -p=18F4520 /i"C:\MCC18\h" "main.c" -fo="main.o" -D__DEBUG -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

clean : 
	$(RM) "main.o" "phase_control.cof" "phase_control.hex" "phase_control.map"

