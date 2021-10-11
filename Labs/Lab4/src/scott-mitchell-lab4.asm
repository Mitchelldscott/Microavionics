;;;;;;; Lab 3 Original Template for ASEN 4067/5067 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Author:	    YOUR NAME HERE
; Date:		    DATE HERE
; Target:	    PIC18F87K22
; 	    
; REFERENCES:	    Ruben Hinojosa Torres, Lara Lufkin
;		Using as reference also: main.s by Dan1138
;		lab3_orig.asm by Scott Palo, Doug Weibel, Gabe LoDolce and Trudy Schwartz
; Date (Original):  2021-06-5
; 	    
; Compiler: pic-as(v2.32)
; IDE:      MPLABX v5.50
			      
			      
; !!!!!!!!!!!!!!!IMPORTANT!!!!!!!! 
; Compiler Notes: 
; Add this line to the Compiler flags i.e
;   Right click on project name -> Properties -> pic-as Global Options -> 
;   Additional options: 
;    -Wl,-presetVec=0h,-pHiPriISR_Vec=0008h,-pLoPriISR_Vec=0018h
			      
; Description: 
; On power up execute the following sequence:
; 	RD5 ON for ~1 second then OFF
; 	RD6 ON for ~1 second then OFF
; 	RD7 ON for ~1 second then OFF
; LOOP on the following forever:
; 	Blink "Alive" LED (RD4) ON for ~1sec then OFF for ~1sec
; 	Read input from RPG (at least every 2ms) connected to pins 
;		RD0 and RD1 and mirror the output onto pins RJ2 and RJ3
; 	ASEN5067 ONLY: Read input from baseboard RD3 button and toggle the value 
;		of RD2 such that the switch being pressed and RELEASED causes 
;		RD2 to change state from ON to OFF or OFF to ON
;	NOTE: ~1 second means +/- 100msec
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
; Loop
; Initial 	- 	Initialize ports and perform LED sequence
; WaitXXXms	- 	Subroutine to wait XXXms
; Wait1sec 	- 	Subroutine to wait 1 sec 
; Check_SW 	- 	Subroutine to check the status of RD3 button and change RD2 (ASEN5067 ONLY)
; Check_RPG	- 	Read the values of the RPG from RD0 and RD1 and display on RJ2 and RJ3	
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Hardware notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	RPG-A port/pin is RJ2
;	RPG-B port/pin is RJ3
// <editor-fold defaultstate="collapsed" desc="Pin Mapping">
/*
    Pin | Pin Name/Register Name
     1  | RH2/AN21/A18
     2  | RH3/AN20/A19
     3  | RE1/P2C/WR/AD9
     4  | RE0/P2D/RD/AD8
     5  | RG0/ECCP3/P3A
     6  | RG1/TX2/CK2/AN19/C3OUT
     7  | RG2/RX2/DT2/AN18/C3INA
     8  | RG3/CCP4/AN17/P3D/C3INB
     9  | MCLR/RG5
     10 | RG4/RTCC/T7CKI(Note:2)/T5G/CCP5/AN16/P1D/C3INC
     11 | VSS
     12 | VDDCORE/VCAP
     13 | RF7/AN5/SS1
     14 | RF6/AN11/C1INA
     15 | RF5/AN10/C1INB
     16 | RF4/AN9/C2INA
     17 | RF3/AN8/C2INB/CTMUI
     18 | RF2/AN7/C1OUT
     19 | RH7/CCP6(Note:3)/P1B/AN15
     20 | RH6/CCP7(Note:3)/P1C/AN14/C1INC
     21 | RH5/CCP8(Note:3)/P3B/AN13/C2IND
     22 | RH4/CCP9(Note:2,3)/P3C/AN12/C2INC
     23 | RF1/AN6/C2OUT/CTDIN
     24 | ENVREG
     25 | AVDD
     26 | AVSS
     27 | RA3/AN3/VREF+
     28 | RA2/AN2/VREF-
     29 | RA1/AN1
     30 | RA0/AN0/ULPWU
     31 | VSS
     32 | VDD
     33 | RA5/AN4/T1CKI/T3G/HLVDIN
     34 | RA4/T0CKI
     35 | RC1/SOSC/ECCP2/P2A
     36 | RC0/SOSCO/SCKLI
     37 | RC6/TX1/CK1
     38 | RC7/RX1/DT1
     39 | RJ4/BA0
     40 | RJ5/CE
     41 | RJ6/LB
     42 | RJ7/UB
     43 | RC2/ECCP1/P1A
     44 | RC3/SCK1/SCL1
     45 | RC4/SDI1/SDA1
     46 | RC5/SDO1
     47 | RB7/KBI3/PGD
     48 | VDD
     49 | OSC1/CLKI/RA7
     50 | OSC2/CLKO/RA6
     51 | VSS
     52 | RB6/KBI2/PGC
     53 | RB5/KBI1/T3CKI/T1G
     54 | RB4/KBI0
     55 | RB3/INT3/CTED2/ECCP2(Note:1)/P2A
     56 | RB2/INT2/CTED1
     57 | RB1/INT1
     58 | RB0/INT0/FLT0
     59 | RJ3/WRH
     60 | RJ2/WRL
     61 | RJ1/OE
     62 | RJ0/ALE
     63 | RD7/SS2/PSP7/AD7
     64 | RD6/SCK2/SCL2/PSP6/AD6
     65 | RD5/SDI2/SDA2/PSP5/AD5
     66 | RD4/SDO2/PSP4/AD4
     67 | RD3/PSP3/AD3
     68 | RD2/PSP2/AD2
     69 | RD1/T5CKI/T7G/PSP1/AD1
     70 | VSS
     71 | VDD
     72 | RD0/PSP0/CTPLS/AD0
     73 | RE7/ECCP2/P2A/AD15
     74 | RE6/P1B/CCP6(Note:3)/AD14
     75 | RE5/P1C/CCP7(Note:3)/AD13
     76 | RE4/P3B/CCP8(Note:3)/AD12
     77 | RE3/P3C/CCP9(Note:2,3)/REF0/AD11
     78 | RE2/P2B/CCP10(Note:2)/CS/AD10
     79 | RH0/AN23/A16
     80 | RH1/AN22/A17

Note (1) The ECCP2 pin placement depends on the CCP2MX Configuration bit 
	setting and whether the device is in Microcontroller or Extended 
	Microcontroller mode.
     (2) Not available on the PIC18F65K22 and PIC18F85K22 devices.
     (3) The CC6, CCP7, CCP8 and CCP9 pin placement depends on the 
	setting of the ECCPMX Configuration bit (CONFIG3H<1>).
*/
// </editor-fold>

;;;;;;;;;;;;;;;;;;;;;;;;;; Assembler Directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Processor Definition
PROCESSOR   18F87K22
; Radix Definition 
RADIX	DEC
; List Definition
;   C: Set the page (i.e., Column) width
;   N: Set the page length
;   X: Turn MACRO expansion on or off
; LIST	C = 160, N = 0, X = OFF
; Include File:
#include <xc.inc>

; PIC18F87K22 Configuration Bit Settings
// <editor-fold defaultstate="collapsed" desc="CONFIG Definitions">
; CONFIG1L
CONFIG  RETEN = ON            ; VREG Sleep Enable bit (Enabled)
CONFIG  INTOSCSEL = HIGH      ; LF-INTOSC Low-power Enable bit (LF-INTOSC in 
                              ;	    High-power mode during Sleep)
CONFIG  SOSCSEL = HIGH        ; SOSC Power Selection and mode Configuration bits 
			      ;	    (High Power SOSC circuit selected)
CONFIG  XINST = OFF           ; Extended Instruction Set (Disabled)

; CONFIG1H
CONFIG  FOSC = HS1            ; Oscillator (HS oscillator 
			      ;	    (Medium power, 4 MHz - 16 MHz))
CONFIG  PLLCFG = OFF          ; PLL x4 Enable bit (Disabled)
CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor (Disabled)
CONFIG  IESO = OFF            ; Internal External Oscillator Switch Over Mode 
			      ;	    (Disabled)

; CONFIG2L
CONFIG  PWRTEN = ON           ; Power Up Timer (Enabled)
CONFIG  BOREN = ON            ; Brown Out Detect (Controlled with SBOREN bit)
CONFIG  BORV = 1              ; Brown-out Reset Voltage bits (2.7V)
CONFIG  BORPWR = ZPBORMV      ; BORMV Power level (ZPBORMV instead of BORMV 
			      ;	    is selected)

; CONFIG2H
CONFIG  WDTEN = OFF           ; Watchdog Timer (WDT disabled in hardware; 
			      ;	    SWDTEN bit disabled)
CONFIG  WDTPS = 1048576       ; Watchdog Postscaler (1:1048576)

; CONFIG3L
CONFIG  RTCOSC = SOSCREF      ; RTCC Clock Select (RTCC uses SOSC)
CONFIG  EASHFT = ON           ; External Address Shift bit (Address Shifting 
			      ;	    enabled)
CONFIG  ABW = MM              ; Address Bus Width Select bits (8-bit 
			      ;	    address bus)
CONFIG  BW = 16               ; Data Bus Width (16-bit external bus mode)
CONFIG  WAIT = OFF            ; External Bus Wait (Disabled)

; CONFIG3H
CONFIG  CCP2MX = PORTC        ; CCP2 Mux (RC1)
CONFIG  ECCPMX = PORTE        ; ECCP Mux (Enhanced CCP1/3 [P1B/P1C/P3B/P3C] 
			      ;	    muxed with RE6/RE5/RE4/RE3)
; CONFIG  MSSPMSK = MSK7        ; MSSP address masking (7 Bit address masking 
			      ;	    mode)
CONFIG  MCLRE = ON            ; Master Clear Enable (MCLR Enabled, RG5 Disabled)

; CONFIG4L
CONFIG  STVREN = ON           ; Stack Overflow Reset (Enabled)
CONFIG  BBSIZ = BB2K          ; Boot Block Size (2K word Boot Block size)

; CONFIG5L
CONFIG  CP0 = OFF             ; Code Protect 00800-03FFF (Disabled)
CONFIG  CP1 = OFF             ; Code Protect 04000-07FFF (Disabled)
CONFIG  CP2 = OFF             ; Code Protect 08000-0BFFF (Disabled)
CONFIG  CP3 = OFF             ; Code Protect 0C000-0FFFF (Disabled)
CONFIG  CP4 = OFF             ; Code Protect 10000-13FFF (Disabled)
CONFIG  CP5 = OFF             ; Code Protect 14000-17FFF (Disabled)
CONFIG  CP6 = OFF             ; Code Protect 18000-1BFFF (Disabled)
CONFIG  CP7 = OFF             ; Code Protect 1C000-1FFFF (Disabled)

; CONFIG5H
CONFIG  CPB = OFF             ; Code Protect Boot (Disabled)
CONFIG  CPD = OFF             ; Data EE Read Protect (Disabled)

; CONFIG6L
CONFIG  WRT0 = OFF            ; Table Write Protect 00800-03FFF (Disabled)
CONFIG  WRT1 = OFF            ; Table Write Protect 04000-07FFF (Disabled)
CONFIG  WRT2 = OFF            ; Table Write Protect 08000-0BFFF (Disabled)
CONFIG  WRT3 = OFF            ; Table Write Protect 0C000-0FFFF (Disabled)
CONFIG  WRT4 = OFF            ; Table Write Protect 10000-13FFF (Disabled)
CONFIG  WRT5 = OFF            ; Table Write Protect 14000-17FFF (Disabled)
CONFIG  WRT6 = OFF            ; Table Write Protect 18000-1BFFF (Disabled)
CONFIG  WRT7 = OFF            ; Table Write Protect 1C000-1FFFF (Disabled)

; CONFIG6H
CONFIG  WRTC = OFF            ; Config. Write Protect (Disabled)
CONFIG  WRTB = OFF            ; Table Write Protect Boot (Disabled)
CONFIG  WRTD = OFF            ; Data EE Write Protect (Disabled)

; CONFIG7L
CONFIG  EBRT0 = OFF           ; Table Read Protect 00800-03FFF (Disabled)
CONFIG  EBRT1 = OFF           ; Table Read Protect 04000-07FFF (Disabled)
CONFIG  EBRT2 = OFF           ; Table Read Protect 08000-0BFFF (Disabled)
CONFIG  EBRT3 = OFF           ; Table Read Protect 0C000-0FFFF (Disabled)
CONFIG  EBRT4 = OFF           ; Table Read Protect 10000-13FFF (Disabled)
CONFIG  EBRT5 = OFF           ; Table Read Protect 14000-17FFF (Disabled)
CONFIG  EBRT6 = OFF           ; Table Read Protect 18000-1BFFF (Disabled)
CONFIG  EBRT7 = OFF           ; Table Read Protect 1C000-1FFFF (Disabled)

; CONFIG7H
CONFIG  EBRTB = OFF           ; Table Read Protect Boot (Disabled)
// </editor-fold>

;;;;;;;;;;;;;;;;;;;;;;;;; MACRO Definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; MACRO Definitions:

; MOVLF
; Description:
;   Move literal value to given register. 
; Input: 
;   lit: literal value
;   dest: destination 
;   access: Access bank or not. Possible values are 'a' for access bank or
;	'b' for banked memory.
  MOVLF	    MACRO   lit, dest, access
    MOVLW   lit	    ; Move literal into WREG
    BANKSEL	(dest)	; Select Bank for next file instruction
    MOVWF   BANKMASK(dest), access  ; Move WREG into destination file
  ENDM
  
;; POINT adapted from Reference: Peatman CH 7 LCD
;POINT
; Taken from lab4_example.asm
; Description:
;   Loads strings into table pointer. 
; Input: 
;   stringname: name of the variable containg the desired string.
  POINT	    MACRO stringname
    MOVLF high stringname, TBLPTRH, A 
    MOVLF low stringname, TBLPTRL, A
  ENDM
  
;DISPLAY
; Taken from lab4_example.asm
; Description:
;   Displays a given register in binary on the LCD. 
; Input: 
;   register: The register that is to be displayed on the LCD. 
  DISPLAY   MACRO register
    MOVFF register, BYTE 
    CALL ByteDisplay
  ENDM

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Objects to be defined in Access Bank for Variables.
; Examples:
PSECT	udata_acs
DUTY:	    DS	1   ; Track the PWM Duty Cycle (Decimal)
DutyBig:    DS  1   ; Duty Big
CNT:	    DS  1   ; Reserve 1 byte for CNT in access bank at 0x000 (literal or file location)
RE3TRACKER: DS  1   ; tracks the RD3 button value
RE3TEMP:    DS  1   ;
BOUNCECNT:  DS  1   ; delay after button press
ALIVECNT:   DS  2   ;
INTVAL:     DS  3   ; Reserve 3 bytes for the integer part of the display
DECVAL:     DS  3   ; Reserve 3 bytes for the decimal part of the string
PERIODCNT:  DS  1   ; Track the PWM period

    
; Objects to be defined in Bank 1
PSECT	udata_bank1
; not used
    
;;;;;;; Constant Strings (Program Memory) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Taken from lab4_example.asm
    
PSECT romData, space = 0, class = CONST  
LCDstr:  
    DB  0x33,0x32,0x28,0x01,0x0C,0x06,0x00  ;Initialization string for LCD
    
LCDTitle:
    DB 0x80,'A','S','E','N','5', '0', '6', '7', 0x00	; Write "ASEN5067" to first line of LCD
    
LCDpwmHeader:
    DB 0xC0, 'P','W', '=', '1', '.', '0', '0', 'm', 's', 0x00

;;;;;;;;;;;;;;;;;;;;;; Power-On-Reset entry point ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Taken from lab4_example.asm
    
PSECT	resetVec, class = CODE, reloc = 2
resetVec:
    NOP	    ; No Operation, give time for reset to occur
    goto    main    ; Go to main after reset

;;;;;;;;;;;;;;;;;;; Interrupt Service Routine Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;
; High Priority ISR Vector Definition:
PSECT	HiPriISR_Vec, class = CODE, reloc = 2
HiPriISR_Vec:
    GOTO    $	; Return to current Program Counter (For Now - no code here yet)
    
; Low Priority ISR Vector Definition:
PSECT	LoPriISR_Vec, class = CODE, reloc = 2
LoPriISR_Vec:
    GOTO    $	; Return to current Program Counter (For Now - no code here yet)


; Program Section: All Code Starts here
PSECT	code
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Mainline Code ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
    RCALL	Initial			; Call to Initial Routine
loop:
    RCALL	BlinkAlive
    
    RCALL	CheckSW1
    
    RCALL	SetPWM

Delay:
    BTFSS 	INTCON, 2, A		; Read Timer0 TMR0IF rollover flag and ...
    BRA		Delay			; Loop if timer has not rolled over
    MOVLF  	high loopval, TMR0H, A	; Then write the timer values into
    MOVLF  	low loopval, TMR0L, A	; the timer high and low registers
    BCF  	INTCON, 2, A
    BRA		loop

;;;;;;;;;;;;;;;;;;;;;; Initialization Routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Initial:

    CLRF	PORTE, A
    MOVLF	0x08h, TRISE, A		; set input for RE3
    
    CLRF	PORTC, A		; set PortB for display
    MOVLF	0x02h, TRISC, A
    
    CLRF	PORTB, A		; set PortB for display
    MOVLF	0xC0, TRISB, A
    
    MOVLF	0x00h,INTCON,A
    MOVLF	0x08h,T0CON,A		; Set up Timer0
    MOVLF	high timerval,TMR0H,A	; Writing binary 25536 to TMR0H / TMR0L
    MOVLF	low timerval,TMR0L,A	; Write high byte first, then low!
    MOVLF	100, CNT, A		; use 8bit CNT as 1 second timer only needs to initialize once
    BSF		T0CON,7,A		; Turn on Timer0
    
    RCALL	InitLCD			; initialize the LCD
    RCALL	Wait10ms
    
    POINT	LCDTitle
    RCALL	DisplayC
    
    MOVLF	0x20h, PORTE, A		; Turn on RE5
    RCALL	Wait1sec		; Pause for 1 second
    
    MOVLF	0x40h, PORTE, A		; turn off RD5 and turn on RD6
    RCALL	Wait1sec		; Pause for 1 second
    
    MOVLF	0x80h, PORTE, A
    RCALL	Wait1sec		; Pause for 1 second
    
    CLRF	PORTE, A
    CLRF	WREG, A
    
    MOVLF	125, ALIVECNT, A
    MOVLF	10, ALIVECNT+1, A
    MOVLF	5, PERIODCNT, A
    MOVLF	1, BOUNCECNT, A
    MOVLF	0, DUTY, A
    MOVLF	1, DutyBig, A
    
    MOVLF	0xC3h, INTVAL, A
    MOVLF	0x31h, INTVAL+1, A
    CLRF	INTVAL+2, A
    
    MOVLF	0xC5h, DECVAL, A
    MOVLF	0x30h, DECVAL+1, A
    CLRF	DECVAL+2, A
    
    POINT	LCDpwmHeader
    RCALL	DisplayC
    
    MOVFF	PORTE, RE3TRACKER, A	; initialize RD3Tracker
    
    MOVLW	5
    MULWF	DutyBig, A
    
    BTG		LATC, 2, A

    MOVLF	high loopval,TMR0H,A	; Writing binary 25536 to TMR0H / TMR0L
    MOVLF	low loopval,TMR0L,A	; Write high byte first, then low!
        
	RETURN				; Return to Mainline code

    
;;;;;;; InitLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; InitLCD Taken from Reference: lab4_example.asm and slightly modified
; InitLCD - modified version of subroutine in Reference: Peatman CH7 LCD
; Initialize the LCD.
; First wait for 0.1 second, to get past display's power-on reset time.
; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
InitLCD:
    MOVLF	10, CNT, A		; Wait 0.1 second for LCD to power up
    RCALL	Wait1sec		; With 10 loaded in CNT this subroutine is actually only 0.1 sec
    POINT	LCDstr			; Set up table pointer to initialization string
    TBLRD*				; Get first byte from string into TABLAT
LCDstrSender:
    CLRF	LATB, A			; First set LATB to all zero	
    BSF		LATB, 5, A		; Drive E high - enable LCD
    MOVF	TABLAT, W, A		; Move byte from program memory into working register
    ANDLW	0xF0h			; Mask to get only upper nibble
    SWAPF	WREG, W, A		; Swap so that upper nibble is in right position to move to LATB (RB0:RB3)
    IORWF	PORTB, W, A		; Mask with the rest of PORTB to retain existing RB7:RB4 states
    MOVWF	LATB, A			; Update LATB to send upper nibble
    BCF		LATB, 5, A		; Drive E low so LCD will process input
    RCALL	Wait10ms		; Wait ten milliseconds
	
    CLRF	LATB, A			; Reset LATB to all zero	    
    BSF		LATB, 5, A		; Drive E high
    MOVF	TABLAT, W, A		; Move byte from program memory into working register
    ANDLW	0x0Fh			; Mask to get only lower nibble
    IORWF	PORTB, W, A		; Mask lower nibble with the rest of PORTB
    MOVWF	LATB, A			; Update LATB to send lower nibble
    BCF		LATB, 5, A		; Drive E low so LCD will process input
    RCALL	Wait10ms		; Wait ten milliseconds
    TBLRD+*				; Increment pointer and get next byte
    MOVF	TABLAT, F, A		; Check if we are done, is it zero?
    BNZ		LCDstrSender
    
	RETURN
    
	
;;;;;;;;DisplayC subroutine;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; DisplayC taken from Reference: Peatman CH7 LCD
; This subroutine is called with TBLPTR containing the address of a constant
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position hex to ASCII.
; This subroutine expects a normal one-byte cursor-positioning code, 0xhh, and
; a null byte at the end of the string 0x00
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

DisplayC:
    BCF		LATB, 4, A		; Drive RS pin low for cursor positioning code
    TBLRD*				; Get byte from string into TABLAT
    MOVF	TABLAT, F, A		; Check for leading zero byte
    BNZ		Loop5
    TBLRD+*				; If zero, get next byte
Loop5:
    MOVLW	0xF0h
    ANDWF	LATB, F, A		; Clear RB0:RB3, which are used to send LCD data
    BSF		LATB, 5, A		; Drive E pin high
    MOVF	TABLAT, W, A		; Move byte from table latch to working register
    ANDLW	0xF0h			; Mask to get only upper nibble
    SWAPF	WREG, W, A		; swap so that upper nibble is in right position to move to LATB (RB0:RB3)
    IORWF	PORTB, W, A		; Mask to include the rest of PORTB
    MOVWF	LATB, A			; Send upper nibble out to LATB
    BCF		LATB, 5, A		; Drive E pin low so LCD will accept nibble

    MOVLW	0xF0h
    ANDWF	LATB, F, A		; Clear RB0:RB3, which are used to send LCD data
    BSF		LATB, 5, A		; Drive E pin high again
    MOVF	TABLAT,W,A		; Move byte from table latch to working register
    ANDLW	0x0Fh			; Mask to get only lower nibble
    IORWF	PORTB,W,A		; Mask to include the rest of PORTB
    MOVWF	LATB,A			; Send lower nibble out to LATB
    BCF		LATB,5,A		; Drive E pin low so LCD will accept nibble
    RCALL	T50			; Wait 50 usec so LCD can process
	
    BSF		LATB,4,A		; Drive RS pin high for displayable characters
    TBLRD+*				; Increment pointer, then get next byte
    MOVF	TABLAT,F,A		; Is it zero?
    BNZ		Loop5
    
	RETURN
	

;;;;;;; DisplayV subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; DisplayV taken from Reference: Peatman CH7 LCD
; This subroutine is called with FSR0 containing the address of a variable
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	

DisplayV:
    BCF		LATB, 4, A		;Drive RS pin low for cursor positioning code
Loop6:
    MOVLW	0xF0h
    ANDWF	LATB, F, A		;Clear RB0:RB3, which are used to send LCD data
    BSF		LATB, 5, A		;Drive E pin high
    MOVF	INDF0, W, A		;Move byte from FSR to working register
    ANDLW	0xF0h			;Mask to get only upper nibble
    SWAPF	WREG, W, A		;swap so that upper nibble is in right position to move to LATB (RB0:RB3)
    IORWF	PORTB, W, A		;Mask to include the rest of PORTB
    MOVWF	LATB, A			;Send upper nibble out to LATB
    BCF		LATB, 5, A		;Drive E pin low so LCD will accept nibble
	
    MOVLW	0xF0h
    ANDWF	LATB, F, A		;Clear RB0:RB3, which are used to send LCD data
    BSF		LATB, 5, A		;Drive E pin high again
    MOVF	INDF0, W, A		;Move byte from table latch to working register
    ANDLW	0x0Fh			;Mask to get only lower nibble
    IORWF	PORTB, W, A		;Mask to include the rest of PORTB
    MOVWF	LATB, A			;Send lower nibble out to LATB
    BCF		LATB, 5, A		;Drive E pin low so LCD will accept nibble
    RCALL	T50			;Wait 50 usec so LCD can process
	  
    BSF		LATB, 4, A		;Drive RS pin high for displayable characters
    MOVF	PREINC0, W, A		;Increment pointer, then get next byte
    BNZ		Loop6
        RETURN

	
;;;;;; ByteDisplay subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Display whatever is in BYTE with the const wrappers.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
ByteDisplay:
    MOVF	DutyBig, W, A		; Add 0x30 to value to convert to ascii
    ADDLW	0x30h
    MOVWF	INTVAL+1, A		; Move Ascii value into storage
    LFSR	0, INTVAL		; Load file Select Register
    RCALL	DisplayV
  
    MOVF	DUTY, W, A		; Repeat above
    ADDLW	0x30h
    MOVWF	DECVAL+1, A
    LFSR	0, DECVAL
    RCALL	DisplayV
	
    BRA		loop	
	
	
;;;;;;; T50 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; T50 modified version of T40 taken from Reference: Peatman CH 7 LCD
; Pause for 50 microseconds or 50/0.25 = 200 instruction cycles.
; Assumes 16/4 = 4 MHz internal instruction rate (250 ns)
; rcall(2) + movlw(1) + movwf(1) + COUNT*3 - lastBNZ(1) + return(2) = 200 
; Then COUNT = 195/3
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
T50:
    MOVLW	195/3			; Each loop L4 takes 3 ins cycles
    MOVWF	CNT, A		    
L4:
    DECF	CNT, F, A
    BNZ		L4
	RETURN	
	
    
;;;;;;;; LoopTime subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; This subroutine waits for Timer0 to complete its ten millisecond count
;; sequence. It does so by waiting for sixteen-bit Timer0 to roll over. To obtain
;; a period of 10ms/250ns = 40000 clock periods, it needs to remove
;; 65536-40000 or 25536 counts from the sixteen-bit count sequence.  
;; Taken from lab4_examble.asm
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
timerval    equ	25536 
loopval	    equ 64730
    
Wait10ms:
    BTFSS 	INTCON, 2, A		; Read Timer0 TMR0IF rollover flag and ...
    BRA		Wait10ms		; Loop if timer has not rolled over
    MOVLF  	high timerval, TMR0H, A	; Then write the timer values into
    MOVLF  	low timerval, TMR0L, A	; the timer high and low registers
    BCF  	INTCON, 2, A		; Clear Timer0 TMR0IF rollover flag
    
	RETURN	

	
;;;;;;; Wait1sec subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to wait 1 sec based on calling WaitXXXms YYY times or up to 3 nested loops
; calls wait10ms CNT times, can vary the delay	
Wait1sec:
    RCALL	Wait10ms
    DECF	CNT, F, A		; decrement second counter
    BNZ		Wait1sec
    MOVLF	100, CNT, A		; reset CNT
    
	RETURN		
	
	
;;;;;;; Check_SW1 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to check the status of RD3 button and change RD2 (ASEN5067 ONLY)
				
CheckSW1:
    MOVFF	PORTE, WREG, A		; Copy the value of PORTD to WREG
    ANDLW	0x08h			; mask the value of PORTD to isolate RE3
    MOVWF	RE3TEMP, A		; save this value incase the button was not pressed
    XORWF	RE3TRACKER, W, A	; using xor and & we can tell if the old value was
    ANDWF	RE3TRACKER, W, A	; high and the new is low, meaning the button was pressed
    BNZ		INCRDuty		; toggle RD2
    RCALL	ResetButton		; Reset the trackers if no toggle
	RETURN
    
    
;;;;;;; Button_Handler subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to handle sw debouncing

ButtonHandler:  
    DECF	BOUNCECNT, A		; Decrement debounce counter
    BZ		CheckSW1		; If Debounce safe check button
    
	RETURN

	
;;;;;;; Toggle_RD2 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to toggle the value of RD2
    
INCRDuty:
    MOVF	DutyBig, W, A
    XORLW	2
    BZ		INCRDutyBig
    MOVLW	2			; Put 2 in working reg
    ADDWF	DUTY, W, A		; Add WREG to Duty (Decimal)
    MOVWF	DUTY, A			; Put WREG back in Duty
    XORLW	0x0Ah			; Check if Duty is 10
    BZ		INCRDutyBig		; Resets Duty and incrments or resets DutyBig
    RCALL	ResetButton		; Reset the button trackers	
	RETURN
    
    
INCRDutyBig:
    MOVLF	0, DUTY, A		; Reset Duty
    MOVF	DutyBig, W, A		; Put 2 in WREG
    SUBLW	1			; Subtract duty from WREG
    BNZ		ResetDutyBig		; If zero roll over
    MOVLF	2, DutyBig, A		; Otherwise increment DutyBig
	RETURN
	
ResetDutyBig:
    MOVLF	1, DutyBig, A
	RETURN
	
    
;;;;;;; Reset_Button subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to reset the values tracking RD3
    
ResetButton:
    MOVLF	1, BOUNCECNT, A	    ; Reset Debounce counter
    MOVFF	RE3TEMP, RE3TRACKER, A	; save RD3 value
    CLRF	RE3TEMP, A		; clear garbage
    CLRF	WREG, A
    
	RETURN	
	
	
SetPWM:
    DECF	PERIODCNT, A
    BZ		Reset_Cycle
	RETURN
	
Reset_Cycle:
    BTG		LATC, 2, A
    MOVLW	0x05h
    MULWF	DutyBig, A
    MOVF	DUTY, W, A
    ADDWF	PRODL, W, A
    CLRF	PRODL, A
    BTFSS	PORTC, 2, A
    BRA		DownCycle
    MOVWF	PERIODCNT, F, A
    BRA		ByteDisplay
    
  
DownCycle:
    SUBLW	100
    MOVWF	PERIODCNT, A
    BRA		ByteDisplay
	
;;;;;;; Toggle_Handler subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to handle RD4 toggle timing
    
BlinkAlive:
    DECFSZ	ALIVECNT, F, A		; Decrement toggle delay counter
	RETURN
    MOVLF	125, ALIVECNT, A
    DECFSZ	ALIVECNT+1, F, A
	RETURN
    MOVLF	10, ALIVECNT+1, A	; reset counter
    BTG		LATE, 4, A		; toggle LED

	RETURN

    
    END     resetVec		    ; End program, return to reset vector ;;;;;;; ASEN 4-5067 Lab3 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

