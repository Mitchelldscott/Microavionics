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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Objects to be defined in Access Bank for Variables.
; Examples:
PSECT	udata_acs
CNT:	    DS  2   ; Reserve 2 byte for CNT in access bank at 0x000 (literal or file location)
SECCNT:	    DS  2   ; Counter for Wait1sec
CNTVAL:	    DS  2   ; value to refill CNT with
SECCNTVAL:  DS	2   ; value to refill SECCNT
RD3TRACKER: DS  1   ; tracks the RD3 button value
RD3TEMP:    DS  1   ;
BOUNCECNT:  DS  1   ; delay after button press
TOGGLECNT:  DS  2   ;
    
; Objects to be defined in Bank 1
PSECT	udata_bank1
; not used

;;;;;;;;;;;;;;;;;;;;;; Power-On-Reset entry point ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
    RCALL    Initial	; Call to Initial Routine
loop:
    
    RCALL Toggle_Handler	    ; Check if RD4 should be toggled and then toggle    
    
    RCALL Button_Handler	    ; Handle timing and logic of using RD3 to toggle RD2
    
    RCALL Check_RPG		    ; copy output of RPG (RD0 & RD1) to RJ2 & RJ3

    MOVFF CNTVAL, CNT, A	    ; need to set CNTVAL to CNT to iterate properly
    MOVFF CNTVAL+1, CNT+1, A
    
    RCALL WaitXXXms		    ; Normalizer sets loop to 500us
    
    
    BRA	    loop

;;;;;;;;;;;;;;;;;;;;;; Initialization Routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Initial:

    CLRF PORTD, A
    MOVLF 0x0Bh, TRISD, A		    ; Set TRISD so RDO & RD1 are input 
					    ; and the rest are output
    CLRF PORTJ, A
    MOVLF 0x00h, TRISJ, A		    ; Same as port D except for there are no inputs

    MOVLF 0x28h, SECCNT, A		    ; use 16bit SECCNT as 1 second timer
    MOVLF 0x01h, SECCNT+1, A	    
    MOVFF SECCNT, SECCNTVAL, A		    ; store SECCNT vals to reuse
    MOVFF SECCNT+1, SECCNTVAL+1, A
    
    MOVLF 0x20h, PORTD, A		    ; Turn on RD5
    
    MOVFF SECCNTVAL, SECCNT, A
    MOVFF SECCNTVAL+1, SECCNT+1, A
    
    RCALL	Wait1sec		    ; Pause for 1 second
    
    MOVLF 0x40h, PORTD, A		    ; turn off RD5 and turn on RD6
    
    MOVFF SECCNTVAL, SECCNT, A
    MOVFF SECCNTVAL+1, SECCNT+1, A
    
    RCALL	Wait1sec		    ; Pause for 1 second
    
    MOVLF 0x80h, PORTD, A
    
    MOVFF SECCNTVAL, SECCNT, A
    MOVFF SECCNTVAL+1, SECCNT+1, A
    
    RCALL	Wait1sec		    ; Pause for 1 second
    
    CLRF PORTD, A
    CLRF WREG, A
    
    MOVFF PORTD, RD3TRACKER, A		    ; initialize RD3Tracker
    
    MOVLF 0x01, TOGGLECNT, A
    MOVLF 0x01, TOGGLECNT+1, A
    
    MOVLF 0x82h, CNTVAL, A		    ; A setup for WaitXXXms
    MOVLF 0x05h, CNTVAL+1, A		    ; These values normalize loop to 500us
					    ; Without normalizing the counters cant count long enough to delay for 1 second
    MOVLF 0x01h, BOUNCECNT, A		    ; initialize debounce counter to prevent overflow
        
    RETURN				    ; Return to Mainline code

;;;;;;; WaitXXXms subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to wait XXX ms
; NOTE - STUDENTS replace XXX with some value of your choosing
; Choose a suitable value to decrement a counter in a loop structure and 
; not using an excessive amount of program memory - i.e. don't use 100 nop's
		
WaitXXXms:			;   this function depends on the value of cnt, cnt+1 and cntval
				; note that calulating the delay is done like 
				; 2 + (((CNT_CYClES * CNT) + CNT+1_CYCLES) * CNT+1) = 2 + (((3 * 0x01) + 4) * 0x01) = 8
				; where CNT_CYClES & CNT+1_CYCLES are 3 and 4
				; or in time divide the equation by 4
				; when using this function the CNTVAL must also be set in order to reset CNT for each CNT+1 
				; passing CNT or CNT+1 as zero will result in an overflow to a 0xFF,0xFF count down

    DECF CNT, f, A		; Decrement counter
    
    BNZ WaitXXXms
    
    MOVFF CNTVAL, CNT, A	; if upper byte has value repeat loop
    DECF CNT+1, f, A		; Decr upper byte
    
    BNZ WaitXXXms
    
    CLRF CNT, A			; clear counter for good practice
    
    RETURN	

;;;;;;; Wait1sec subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to wait 1 sec based on calling WaitXXXms YYY times or up to 3 nested loops
				
Wait1sec:
				    ; Uses the equation	    0.5 + (((0.75 * CNT) + 1) * CNT+1) = microseconds
				    ; re-write in terms of y(CNT+1)   (25,000 - 0.5) / ((0.75 * CNT) + 1) = CNT+1 
				    ; pairs of (CNT, CNT+1) that are close
				    ; (F2, 89) 25.003ms fast
				    ; (EB, 8D) 24.99275ms slow
				    ; (E3, 92) 25.003ms fast
				    ; (D3, 9D) 25.00275 fast
				    ; (CF, A0) 25.0005ms fast
				    ; (84, FA) 25.0005ms fast
    
    MOVLF 0x84h, CNT, A		    ; A setup for WaitXXXms
    MOVLF 0xFAh, CNT+1, A	    ; These values will create a perfect 10,000 us timer
    MOVFF CNT, CNTVAL, A	    ; need to set CNTVAL to CNT to iterate properly
    
    RCALL WaitXXXms
    
    DECF SECCNT, f, A		    ; decrement second counter
    
    BNZ Wait1sec
    
    MOVFF SECCNTVAL, SECCNT, A	    ; if upper byte has value repeat loop
    DECF SECCNT+1, f, A		    ; Decr upper byte
    
    BNZ Wait1sec
     
    RETURN	

;;;;;;; Check_SW1 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to check the status of RD3 button and change RD2 (ASEN5067 ONLY)
				
Check_SW1:
    MOVFF PORTD, WREG, A		; Copy the value of PORTD to WREG
    ANDLW 0x08h				; mask the value of PORTD to isolate RD3
    MOVWF RD3TEMP, A			; save this value incase the button was not pressed
    XORWF RD3TRACKER, W, A		; using xor and & we can tell if the old value was
    ANDWF RD3TRACKER, W, A		; high and the new is low, meaning the button was pressed
    BNZ	  Toggle_RD2			; toggle RD2
    RCALL Reset_Button			; Reset the trackers if no toggle

;;;;;;; Check_RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to read the values of the RPG and display on RJ2 and RJ3
				
Check_RPG:
    MOVFF PORTD, WREG, A		; Copy PORTD into the WREG
    ANDLW 0x03				; mask WREG to isolate RD0 & RD1
    RLNCF WREG, W, A			; Rotate right twice to align bits
    RLNCF WREG, W, A			; ignore the carry bit to prevent garbage
    MOVWF LATJ, A			; write to PORTJ using LATJ
    CLRF WREG, A			; Clear WREG to prevent garbage
    
    RETURN	 
    
;;;;;;; Toggle_RD2 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to toggle the value of RD2
    
Toggle_RD2:
    BTG LATD, 2, A			; Toggle RD2
    RCALL Reset_Button			; Reset the button trackers
    
;;;;;;; Button_Handler subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to handle sw debouncing

Button_Handler:  
    DECF BOUNCECNT, A			; Decrement debounce counter
    BZ Check_SW1			; If Debounce safe check button
    
    RETURN
    
;;;;;;; Toggle_RD4 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to toggle the value of RD4
    
Toggle_RD4:
    BTG LATD, 4, A			; Toggle RD4
    MOVLF 0x64h, TOGGLECNT, A		; Reset Toggle delay counter
    MOVLF 0x14h, TOGGLECNT+1, A
    
    RETURN
    
;;;;;;; Toggle_Handler subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to handle RD4 toggle timing
    
Toggle_Handler:
    DECF TOGGLECNT, A			; Decrement toggle lower delay counter
    BZ Toggle_Handler2			; handle upper counter if zero
    
    RETURN
    
;;;;;;; Toggle_Handler subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to handle RD4 toggle timing
    
Toggle_Handler2:
    DECF TOGGLECNT+1, A			; Decrement toggle upper counter
    BZ Toggle_RD4			; If zero Toggle RD4
    MOVLF 0x64h, TOGGLECNT, A		; otherwise reset toggle lower delay counter
    
    RETURN

;;;;;;; Reset_Button subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to reset the values tracking RD3
    
Reset_Button:
    MOVLF 0x04h, BOUNCECNT, A		; Reset Debounce counter
    MOVFF RD3TEMP, RD3TRACKER, A	; save RD3 value
    CLRF RD3TEMP, A			; clear garbage
    CLRF WREG, A
    
    RETURN
    
    END     resetVec		    ; End program, return to reset vector ;;;;;;; ASEN 4-5067 Lab3 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

