;**************************************************************
;       FINAL PROJECT COE538 - SECTION 4                      *
;       MANAV PATEL - 500967756                               *
;       MARIO IANNIELLO - 500894623                           *  
;       KAPILAN BALAKRISHNAN - 500959576                      *
;**************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

        ROMStart    EQU  $4000  

; variable/data section

            ORG RAMStart
 ; Insert here your data definition.
            START EQU 0
            FORWARD EQU 1
            BUMP EQU 2
            BACKTRACK EQU 3
            IS_INTERSECT EQU 4


            PRIMARY_PATH EQU 1
            ALTERNATE_PATH EQU 0
            PATHINGMODE DS.B 1
            DIRECTION DC.B 1
; code section
            ORG   ROMStart

Entry:
_Startup:
            LDS   #RAMEnd+1       ; initialize the stack pointer

            CLI    ;ENABLE INTERUPTS
            
            JSR   initPORTS                 ;                                           
                                                
            JSR   initAD                    ;                  
                                                                                         
            JSR   initLCD                   ;                     
            JSR   clrLCD                    ;                   
                                                                                           
            JSR   initTCNT                  ;   
                                                                                          
            CLI                             ; 
                                                                                          
            LDX   #msg1                     ; 
            JSR   putsLCD                   ;                                         
                                                                                           
            LDAA  #$8A                      ;
            JSR   cmd2LCD                   ; 
            LDX   #msg2                     ;
            JSR   putsLCD                   ;
                                                                        
            LDAA  #$C0                      ; 
            JSR   cmd2LCD                   ;
            LDX   #msg3                     ; 
            JSR   putsLCD                   ; 
                                            
            LDAA  #$C7                      ;
            JSR   cmd2LCD                   ; 
            LDX   #msg4                     ; 
            JSR   putsLCD                   ;      
                                                

            MOVB #$01,PATHINGMODE ;BY DEFAULT IT IS ONE (WILL TAKE RIGHT MOST PATH ALWAYS)


STANDBY     BRCLR PORTAD0,#$04,INITIATE_START ;WHILE IN SBY LOOP MAIN UNLESS FWD BUMP HAS A VALUE OF 0, BRANCH             
            BRA STANDBY           ;KEEP CHECKING UNTIL ITS PUSHED

INITIATE_START LDAA #START 
            STAA STATE_CURRENT   ;SET STATE TO START 

MAIN        JSR READING                 ;read sensor and update reading
            LDAA STATE_CURRENT 
            JSR DISPATCHER
            BRA MAIN 

;**************************************************************
;*                 State Subroutine                 *
;**************************************************************

;STATE BREAKDOWN
; PASS CURRENT STATE TO REGISTER A 
;
; DISP ---> SEARCH FOR CURRENT STATE REPEATEDLY 
;
; 

DISPATCHER   CMPA #START    ; IF START STATE CALL START ROUTINE
             BNE NOT_START
             JSR STATE_START   
             JMP DISP_EXIT ; EXIT (RTS)

NOT_START   CMPA #FOLLOW    ;ELSE IF ITS FOLLOW
            BNE NOT_FOLLOW  
            JSR STATE_FOLLOW  ;JUMP TO FOLLOW STATE
            JMP DISP_EXIT     ;BACK TO MAIN 

NOT_FOLLOW  CMPA #IS_INTERSECT  ;ELSE IF AT INTERSECT
            BNE NO_INTERSECT             
            JSR STATE_INTERSECT  ;GO TO STATE INTERSECT WHERE INFORMATION WILL BE STORED 
            JMP DISP_EXIT       ;BACK TO MAIN 

NO_INTERSECT CMPA #BUMP         ;ELSE IF BUMP 
            BNE NONE_OF_ABOVE  ;IF NONE OF THE ABOVE SWI SOMETHING IS WRONG   
            JSR BACKTRACK   ;IF BUMP BACKTRACK 
            JMP DISP_EXIT           
            ;;;
            
NONE_OF_ABOVE SWI
DISP_EXIT  RTS

;************;
;STATE SUBROUTINES
;***********;
STATE_START     JSR del_50us   ;DELAY TO AVOID DOUBLE REGISTERING INPUT FROM BUMPER (SOFTWARE DELAY IS ENOUGH BC NOTHING ELSE IS HAPPENING)
                MOVB #FOLLOW,STATE_CURRENT
                JSR STARTON
                JSR PORTON
                RTS            ;RETURN TO DISPATCH (WHICH WILL RETURN TO MAIN)
;
;;CHECK FOR ADJUSTMENTS (IS IT ON COURSE?)
 ;;;CHECK FOR INTERSECTION(IF SO SET STATE TO IS INTERSECT)
STATE_FOLLOW   BRCLR PORTAD0,#$04,RECOGNIZE_BUMP 


               LDAA DETECTION_F   ;CHECK F AND E FOR ALIGNMENT
               BEQ SHIFT_R   ;IF F IS  ZERO THEN IT NEED A SHIFT AND SHOULD TURN/LEAVE RIGHT MOTOR OFF
               JSR STARON     ; OTHERWISE THEY CAN STAY ON 
               LDAA DETECTION_E   ;IF E IS ZERO THEN IT NEEDS TO SHIFT LEFT AND SHOULD
               BEQ SHIFT_L       ;TURN/LEAVE LEFT MOTOR OFF 
               JSR PORTON       ;OTHERWISE LEFT CAN STAY ON 
  ;;CHECK FOR INTX- BY PROCESS OF ELIM
                ;TYPES OF INTXN
                ;---->----\|/---->   THIS IS TYPE 0 (\|/ IS AN ARROW DOWNWARD)

                ;---->----/|\---->   THIS IS TYPE 1 (/|\ IS AN ARROW UPWARD)

                ;---->----|  THIS IS TYPE 2 (T INTERSECT)
             
               LDAA DETECTION_A
               BNEQ NOT_2  ;IF THERE IS A PATH FORWARD ITS NOT TYPE 2
               LDAA DETECTION_B ;NO PATH FORWARDCHECK FOR L & R 
               BEQ NOT_INTXN   ; IF NO PATH LEFT NOT AN INTERSECTION (MAYBE A RIGHT TURN?) SO BRANCH 
               LDAA DETECTION_D 
               BEQ NOT_INTXN ;NO PATH RIGHT OR LEFT OR FORWARD YOU PROBABLY FINISHED, SO GO DOUBLE CHECK 
               ;IS INTXN TYPE2 (PUSH VALUE $2X WHERE X IS PATHING MODE )
               LDAA PATHINGMODE     ;CHECK PATH MODE 
               BEQ ALT1
               LDAA #$21         ;STORE INTXN TYPE AND DIRECTION TAKEN
               PSHA   
               BRA HANDLE_INTXN  ;SET STATE AND RETURN
                             
            
ALT1           LDAA #$20         ;STORE INTXN TYPE AND DIRECTION TAKEN
               PSHA 


      ;          #$21
      ;          PULA    ;A -> $21
       ;         ANDA #%00001111
        ;        ;A -> $01
        ;        CMPA $01   ; A - (VAL) 
       ;         BEQ (DO SOMETHING)


NOT_2         LDAA DETECTION_B ;THERE IS A PATH FORWARD SO CHECK LEFT 
               BEQ NOT_1   ;IF NO PATH LEFT ITS NOT PATH TYPE 1 
               ;IF WE GET HERE THERE IS A PATH FORWARD AND TO THE LEFT SO IT IS TYPE 1 ($1X )
               LDAA PATHINGMODE
               BEQ ALT2 
               LDAA #$11
               PSHA
                HANDLE_INTXN

ALT2          LDAA #$10
               PSHA               
                HANDLE_INTXN


NOT_1          LDAA DETECTION_D ;THERE IS A PATH FORWARD IF WE GET HERE 
                BEQ NOT_INTXN ;IF NO PATH RIGHT THEN ITS NOT AN INTXN JUST A PATH FORWARD      


SHIFT_R         STAROFF
                JMP STATE_FOLLOW ; LOOP UNTIL CORRECTION COMPLETE

SHIFT_L         PORTOFF
                JMP STATE_FOLLOW 

NOT_INTXN       LDAA DETECTION_A   ;WE GET HERE IF ITS NOT AN INTXN SO JUST CHECK IF FINISHED BEFORE RETURNING TO LOOP  
                BNEQ SAFE_EXIT     ;EXITS IF THERE IS A PATH FORWARD (NOT DONE)
                LDAA DETECTION_B
                BNEQ SAFE_EXIT
                LDAA DETECTION_D        



RECOGNIZE_BUMP MOVB #BUMP,STATE_CURRENT
                RTS  ;RETURN TO DISP THEN MAIN

SAFE_EXIT       RTS  ;RETURNS WITHOUT CHANGING STATE                
HANDLE_INTXN   MOVB #IS_INTERSECT,STATE_CURRENT
               RTS ;WILL RETURN TO DISPATCH (WILL RETURN TO MAIN )
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
STATE_INTERSECT         PORTOFF
                        STAROFF ;STOP TAKE A MOMENT TO CALCULATE NEXT MOVE
                        PULA ;PULL INTERSECTION IN STACK FOR OBSERVATION 
                        PSHA ;REPLACE IT INTO STACK TO PRESERVE INFORMATION
                        ANDA #%11110000 
                        CMPA #$00
                        BNEQ NOT_INT_0   ;CHECK IF TYPE 0
                        LDAA PATHINGMODE
                        BNEQ PIVOT_C      ; IF REGULAR PATHING TURN RIGHT OTHERWISE CONTINUE STRAIGHT
                        BRA EXIT_STATE 
                        
NOT_INT_0               CMPA #$10          ;ELSE IF TYPE 1
                        BNEQ NOT_INT_1 
                        LDAA PATHINGMODE
                        BEQ PIVOT_CC     ;IF REGULAR PATHING GO STRAIGHT
                        BRA EXIT_STATE   ;ELSE GO LEFT
                                            
                                    
NOT_INT_1               CMPA #$20           ;ELSE ITS TYPE 2 (CMPA JUST FOR READABILITY)
                        LDAA PATHINGMODE
                        BEQ PIVOT_CC
                        BRA PIVOT_C
                        
                        
                 
PIVOT_C                 JSR PORTFWD             
                        JSR STARREV
                        JSR PORTON
                        JSR STARON
PIVOTLOOP               LDAA DETECTION_A
                        BNEQ PIVOTLOOP             
FINDLINE                LDAA DETECTION_A
                        BEQ FINDLINE
                        JSR PORTOFF
                        JSR STAROFF
                        RTS

PIVOT_CC                 JSR PORTREV
                        JSR STARFWD
                        JSR PORTON
                        JSR STARON
PIVOTLOOP1               LDAA DETECTION_A
                        BNEQ PIVOTLOOP1             
FINDLINE1               LDAA DETECTION_A
                        BEQ FINDLINE1
                        JSR PORTOFF
                        JSR STAROFF
                        RTS
                        
EXIT_STATE              STARON
                        PORTON
                        JSR del_50us    ; WAIT FOR BOT TO LEAVE INTERSECTION BEFORE CHECKING FOR ONE AGAIN 
                        MOVB #FOLLOW,STATE_CURRENT ; IT IS READY TO CONTINUE FOLLOWING THE LINE FORWARD
                        RTS

  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
STATE_BACKTRACK         PORTOFF
                        STAROFF
                        PORTREV
                        STARREV
INIT_FIRST_INTERVAL    JSR PORTON      ;BEGIN REVERSING 
                        JSR STARON
                        del_50us
checkloop0              LDAA DETECTION_B ;REVERSE UNTIL FIRST INTERSECTION IS DETECTED
                        BNEQ checkint
                        LDAA DETECTION_D
                        BNEQ checkint
                        BRA checkloop0

checkint                PULA; PULLS LST DIRECTION AND INTERSECTION
                        PSHA
                        CMPA #$01
                        BEQ INTERSECTION_01 ;BRANCH IF PREV INTERSECTION WAS A TYPE 01
                        CMPA #$00
                        BEQ INTERSECTION_00 ; BRANCH IF PREV INTERSECTION WAS A TYPE 10
                        CMPA #$11
                        BEQ INTERSECTION_11;BRANCH IF PREV INTERSECTION WAS A TYPE 11
                        CMPA #$10
                        BEQ INTERSECTION_10;BRANCH IF PREV INTERSECTION WAS A TYPE 10
                        CMPA #$21
                        BEQ INTERSECTION_21;BRANCH IF PREV INTERSECTION WAS A TYPE 21
                        CMPA #$20
                        BEQ INTERSECTION_20;BRANCH IF PREV INTERSECTION WAS A TYPE 20
                        
INTERSECTION_01         JSR PIVOT_CC ;PIVOT COUNTER CLOCKWISE 
                        JSR PORTREV
                        JSR STARREV
                        JSR PORTON
                        JSR STARON
                        BNEQ CHANGEPATH0    ; CHANGE PATHINGMODE TO 0
                                          

INTERSECTION_00         JSR PIVOT_CC        ;PIVOT CLOCKWISE
                        JSR PORTREV
                        JSR STARREV
                        JSR PORTON
                        JSR STARON
                        BNEQ CHANGEPATH1    ;CHANGE PATHINGMODE TO 1

INTERSECTION_11         JSR PIVOT_C             ;PIVOT CLOCKWISE
                        JSR PORTREV
                        JSR STARREV
                        JSR PORTON
                        JSR STARON
                        BNEQ CHANGEPATH0    ; CHANGE PATHINGMODE TO 0

INTERSECTION_10         JSR PIVOT_CC            ;PIVOT COUNTER CLOCKWISE
                        JSR PORTREV
                        JSR STARREV
                        JSR PORTON
                        JSR STARON
                        BNEQ CHANGEPATH1    ;CHANGE PATHINGMODE TO 1

INTERSECTION_21         JSR PIVOT_CC            ;PIVOT COUNTER CLOCKWISE
                        JSR PORTREV
                        JSR STARREV
                        JSR PORTON
                        JSR STARON
                        BNEQ CHANGEPATH0

INTERSECTION_20         JSR PIVOT_C             ;PIVOT CLOCKWISE
                        JSR PORTREV
                        JSR STARREV
                        JSR PORTON
                        JSR STARON
                        BNEQ CHANGEPATH1    ;CHANGE PATHINGMODE TO 1

CHANGEPATH0              MOVB #$01,PATHINGMODE  ;CHANGE PATHINGMODE TO 1
                         JMP INIT_SECOND_INTERVAL

CHANGEPATH1              MOVB #$00,PATHINGMODE  ;CHANGE PATHINGMODE TO 01
                         JMP INIT_SECOND_INTERVAL

INIT_SECOND_INTERVAL    JSR PORTON      ;BEGIN REVERSING 
                        JSR STARON
                        del_50us
checkloop               LDAA DETECTION_B ;REVERSE UNTIL SECOND INTERSECTION IS DETECTED
                        BNEQ EXIT_LOOP
                        LDAA DETECTION_D
                        BNEQ EXIT_LOOP
                        BRA checkloop

EXIT_LOOP               JSR PORTOFF ;TURN OFF ALL MOTORS 
                        JSR STAROFF
                        SECOND_INTXN_DETECTED

SECOND_INTXN_DETECTED   JSR PORTOFF ;TURN OFF ALL MOTORS AND RETURN TO DISPATH(MAIN PART OF CODE)
                        JSR STAROFF
                        JSR STARFWD
                        JSR PORTFWD
                        MOVB #FOLLOW,STATE_CURRENT
                        RTS ; RETURN TO DISPATCH (WCHICH WILL RETURN TO MAIN )
;**************************************************************
;*                 MOTOR CONTROL        PORT = LEFT ;;;; STAR = RIGHT                       *
;**************************************************************
        BSET DDRA, %00000011
        BSET DDRT, %00110000
        JSR STARFWD
        JSR PORTFWD
        JSR STARON
        JSR PORTON
        JSR STARREV
        JSR PORTREV
        JSR STAROFF
        JSR PORTOFF
        BRA *
        
STARON      LDAA PTT
            ORAA #%00100000
            STAA PTT
            RTS
            
STAROFF     LDAA PTT
            ANDA #%11011111
            STAA PTT
            RTS
            
PORTON      LDAA PTT
            ORAA #%00010000
            STAA PTT
            RTS
            
PORTOFF     LDAA PTTLDAA 
            ANDA #%11101111
            STAA PTT
            RTS
            
STARFWD     LDAA PORTA
            ORAA #%00000010
            STAA PORTA
            RTS
            
STARREV     LDAA PORTA
            ANDA #%11111101
            STAA PORTA
            RTS
            
            
PORTFWD     LDAA PORTA
            ANDA #%11111110
            STAA PORTA
            RTS
            
PORTREV     LDAA PORTA
            ORAA #%00000001
            STAA PORTA
            RTS


;**************************************************************
;*                 SUBROUTINE SECTION FOR SENSOR              *
;**************************************************************
SUBMAN                  JSR G_LEDS_ON ; Enable the guider LEDs
                        JSR READ_SENSORS ; Read the 5 guider sensors
                        JSR G_LEDS_OFF ; Disable the guider LEDs
                        JSR DISPLAY_SENSORS ; and write them to the LCD
                        LDY #6000 ; 300 ms delay to avoid
                        JSR del_50us ; display artifacts
                        BRA SUBMAN ; Loop forever

                        
                        STAA DETECTION_A        ;Setting Sensor A
                        STAA DETECTION_B        ;Setting Sensor B
                        STAA DETECTION_C        ;Setting Sensor C
                        STAA DETECTION_D        ;Setting Sensor D
                        STAA DETECTION_E        ;Setting Sensor E
                        STAA DETECTION_F        ;Setting Sensor F

DETECTION_A             LDAA SENSOR_BOW         ;      LOAD SENSOR A
                        JNB SENSOR_BOW, DETECTION_B ;  IF SENSOR A HAS A VALUE OF 0, BRANCH 
                        INC DETECTION_A         ;      INCREMENT DECTECTION_A BY 1 IF SENSOR IS DETECTED
                        
DETECTION_B             LDAA SENSOR_PORT         ;       LOAD SENSOR B
                        JNB SENSOR_PORT, DETECTION_C ;  IF SENSOR B HAS A VALUE OF 0, BRANCH 
                        INC DETECTION_B         ;   INCREMENT DETECTION_B BY 1 IF SENSOR IS DETECTED

DETECTION_C             LDAA SENSOR_MID         ;       LOAD SENSOR C
                        JNB SENSOR_MID, DETECTION_D ;   IF SENSOR C HAS A VALUE OF 0, BRANCH 
                        INC DETECTION_C         ;   INCREMENT DETECTION_C BY 1 IF SENSOR IS DETECTED

DETECTION_D             LDAA SENSOR_STBD         ;       LOAD SENSOR D
                        JNB SENSOR_STBD, DETECTION_E ;     IF SENSOR D HAS A VALUE OF 0, BRANCH 
                        INC DETECTION_D         ;   INCREMENT DETECTION_D BY 1 IF SENSOR IS DETECTED

DETECTION_E             LDAA SENSOR_LINE         ;       LOAD SENSOR E
                        JNB SENSOR_BOW, DETECTION_F ;    IF SENSOR E HAS A VALUE OF 0, BRANCH 
                        INC DETECTION_E         ;   INCREMENT DETECTION_E BY 1 IF SENSOR IS DETECTED

DETECTION_F             INC DETECTION_F         ;   INCREMENT DETECTION_F BY 1 IF SENSOR IS DETECTED

FINISH                  RTS
                    
;---------------------------------------------------------------------------
; Guider LEDs ON
; This routine enables the guider LEDs so that readings of the sensor
; correspond to the ’illuminated’ situation.
; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed
G_LEDS_ON               BSET PORTA,%00100000 ; Set bit 5
                        RTS
;
; Guider LEDs OFF
; This routine disables the guider LEDs. Readings of the sensor
; correspond to the ’ambient lighting’ situation.
; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed
G_LEDS_OFF              BCLR PORTA,%00100000 ; Clear bit 5
                        RTS

; READ SENSORS
READ_SENSORS            CLR SENSOR_NUM ; Select sensor number 0
                        LDX #SENSOR_LINE ; Point at the start of the sensor array

RS_MAIN_LOOP            LDAA SENSOR_NUM ; Select the correct sensor input
                        JSR SELECT_SENSOR ; on the hardware
                        LDY #400 ; 20 ms delay to allow the
                        JSR del_50us ; sensor to stabilize

                        LDAA #%10000001 ; Start A/D conversion on AN1
                        STAA ATDCTL5
                        BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done

                        LDAA ATDDR0L ; A/D conversion is complete in ATDDR0L
                        STAA 0,X ; so copy it to the sensor register
                        CPX #SENSOR_STBD ; If this is the last reading
                        BEQ RS_EXIT ; Then exit

                        INC SENSOR_NUM ; Else, increment the sensor number
                        INX ; and the pointer into the sensor array
                        BRA RS_MAIN_LOOP ; and do it again
RS_EXIT                 RTS

; SELECT SENSORS
SELECT_SENSOR           PSHA ; Save the sensor number for the moment
                        LDAA PORTA ; Clear the sensor selection bits to zeros
                        ANDA #%11100011 ;
                        STAA TEMP ; and save it into TEMP
                        PULA ; Get the sensor number
                        ASLA ; Shift the selection number left, twice
                        ASLA ;
                        ANDA #%00011100 ; Clear irrelevant bit positions
                        ORAA TEMP ; OR it into the sensor bit positions
                        STAA PORTA ; Update the hardware
                        RTS

DISPLAY_SENSORS         LDAA SENSOR_BOW ; Get the FRONT sensor value
                        JSR BIN2ASC ; Convert to ascii string in D
                        LDX #DP_FRONT_SENSOR ; Point to the LCD buffer position
                        STD 0,X ; and write the 2 ascii digits there

                        LDAA SENSOR_PORT ; Repeat for the PORT value
                        JSR BIN2ASC
                        LDX #DP_PORT_SENSOR
                        STD 0,X

                        LDAA SENSOR_MID ; Repeat for the MID value
                        JSR BIN2ASC
                        LDX #DP_MID_SENSOR
                        STD 0,X

                        LDAA SENSOR_STBD ; Repeat for the STARBOARD value
                        JSR BIN2ASC
                        LDX #DP_STBD_SENSOR
                        STD 0,X

                        LDAA SENSOR_LINE ; Repeat for the LINE value
                        JSR BIN2ASC
                        LDX #DP_LINE_SENSOR
                        STD 0,X

                        LDAA #CLEAR_HOME ; Clear the display and home the cursor
                        JSR cmd2LCD ; "

                        LDY #40 ; Wait 2 ms until "clear display" command is complete
                        JSR del_50us

                        LDX #TOP_LINE ; Now copy the buffer top line to the LCD
                        JSR putsLCD

                        LDAA #LCD_SEC_LINE ; Position the LCD cursor on the second line
                        JSR LCD_POS_CRSR
                        
                        LDX #BOT_LINE ; Copy the buffer bottom line to the LCD
                        JSR putsLCD
                        RTS
                        
;*******************************************************************
initLCD     BSET     DDRS,%11110000 ; configure pins PS7,PS6,PS5,PS4 for output
            BSET     DDRE,%10010000      ; configure pins PE7,PE4 for output
            LDY      #2000          ; wait for LCD to be ready
            JSR      del_50us       ; -"-
            LDAA     #$28           ; set 4-bit data, 2-line display
            JSR      cmd2LCD        ; -"-
            LDAA     #$0C           ; display on, cursor off, blinking off
            JSR      cmd2LCD        ; -"-
            LDAA     #$06           ; move cursor right after entering a character
            JSR      cmd2LCD        ; -"-
            RTS

;*******************************************************************
clrLCD      LDAA     #$01           ; clear cursor and return to home position
            JSR      cmd2LCD        ; -"-
            LDY      #40            ; wait until "clear cursor" command is complete
            JSR      del_50us       ; -"-
            RTS
;---------------------------------------------------------------------------
; 50 Microsecond Delay
del_50us                PSHX ; (2 E-clk) Protect the X register
eloop                   LDX #300 ; (2 E-clk) Initialize the inner loop counter
iloop                   NOP ; (1 E-clk) No operation
                        DBNE X,iloop ; (3 E-clk) If the inner cntr not 0, loop again
                        DBNE Y,eloop ; (3 E-clk) If the outer cntr not 0, loop again
                        PULX ; (3 E-clk) Restore the X register
                        RTS ; (5 E-clk) Else return

;---------------------------------------------------------------------------
; Binary to ASCII
; Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
; character string in accumulator D
; Uses a table-driven method rather than various tricks.
; Passed: Binary value in ACCA
; Returns: ASCII Character string in D
; Side Fx: ACCB is destroyed
HEX_TABLE               FCC ’0123456789ABCDEF’ ; Table for converting values

BIN2ASC                 PSHA ; Save a copy of the input number on the stack
                        TAB ; and copy it into ACCB
                        ANDB #%00001111 ; Strip off the upper nibble of ACCB
                        CLRA ; D now contains 000n where n is the LSnibble
                        ADDD #HEX_TABLE ; Set up for indexed load
                        XGDX
                        LDAA 0,X ; Get the LSnibble character
                        PULB ; Retrieve the input number into ACCB
                        PSHA ; and push the LSnibble character in its place
                        RORB ; Move the upper nibble of the input number
                        RORB ; into the lower nibble position.
                        RORB
                        RORB
                        ANDB #%00001111 ; Strip off the upper nibble
                        CLRA ; D now contains 000n where n is the MSnibble
                        ADDD #HEX_TABLE ; Set up for indexed load
                        XGDX
                        LDAA 0,X ; Get the MSnibble character into ACCA
                        PULB ; Retrieve the LSnibble character into ACCB
                        RTS
;---------------------------------------------------------------------------
; Routines to control the Liquid Crystal Display
;---------------------------------------------------------------------------
; Initialize the LCD
openLCD                 LDY #2000 ; Wait 100 ms for LCD to be ready
                        JSR del_50us ; "
                        LDAA #INTERFACE ; Set 8-bit data, 2-line display, 5x8 font
                        JSR cmd2LCD ; "
                        LDAA #CURSOR_OFF ; Display on, cursor off, blinking off
                        JSR cmd2LCD ; "
                        LDAA #SHIFT_OFF ; Move cursor right (address increments, no char. shift)
                        JSR cmd2LCD ; "
                        LDAA #CLEAR_HOME ; Clear the display and home the cursor
                        JSR cmd2LCD ; "
                        LDY #40 ; Wait 2 ms until "clear display" command is complete
                        JSR del_50us ; "
                        RTS
;---------------------------------------------------------------------------
; Send a command in accumulator A to the LCD
cmd2LCD                 BCLR LCD_CNTR,LCD_RS ; Select the LCD Instruction register
                        JSR dataMov ; Send data to IR or DR of the LCD
                        RTS
;---------------------------------------------------------------------------
; Send a character in accumulator in A to LCD
putcLCD                 BSET LCD_CNTR,LCD_RS ; select the LCD Data register
                        JSR dataMov ; send data to IR or DR of the LCD
                        RTS
;---------------------------------------------------------------------------
; Send a NULL-terminated string pointed to by X
putsLCD                 LDAA 1,X+ ; get one character from the string
                        BEQ donePS ; reach NULL character?
                        JSR putcLCD
                        BRA putsLCD
donePS                  RTS

;---------------------------------------------------------------------------
; Send data to the LCD IR or DR depending on the RS signal
dataMov                 BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
                        STAA LCD_DAT ; send the 8 bits of data to LCD
                        NOP
                        NOP
                        NOP
                        BCLR LCD_CNTR,LCD_E ; pull the E signal low to complete the write operation
                        LDY #1 ; adding this delay will complete the internal
                        JSR del_50us ; operation for most instructions
                        RTS
;*******************************************************************
initAD      MOVB #$C0,ATDCTL2 ;power up AD, select fast flag clear
            JSR  del_50us ;wait for 50 us
            MOVB #$00,ATDCTL3 ;8 conversions in a sequence
            MOVB #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
            BSET ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
            RTS
;*******************************************************************
int2BCD     XGDX ;Save the binary number into .X
            LDAA #0 ;Clear the BCD_BUFFER
            STAA TEN_THOUS
            STAA THOUSANDS
            STAA HUNDREDS
            STAA TENS
            STAA UNITS
            STAA BCD_SPARE
            STAA BCD_SPARE+1
;*
            CPX #0 ;Check for a zero input
            BEQ CON_EXIT ;and if so, exit
;*
            XGDX ;Not zero, get the binary number back to .D as dividend
            LDX #10 ;Setup 10 (Decimal!) as the divisor
            IDIV ;Divide: Quotient is now in .X, remainder in .D
            STAB UNITS ;Store remainder
            CPX #0 ;If quotient is zero,
            BEQ CON_EXIT ;then exit
;*
            XGDX ;else swap first quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB TENS
            CPX #0
            BEQ CON_EXIT
;*
            XGDX ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB HUNDREDS
            CPX #0
            BEQ CON_EXIT
;*
            XGDX ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB THOUSANDS
            CPX #0
            BEQ CON_EXIT
;*
            XGDX ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB TEN_THOUS
;*
CON_EXIT    RTS ;We’re done the conversion
;*******************************************************************
BCD2ASC     LDAA #0 ;Initialize the blanking flag
            STAA NO_BLANK

;*******************************************************************
ENABLE_TOF  LDAA  #%10000000
            STAA  TSCR1      ; Enable TCNT
            STAA  TFLG2      ; Clear TOF
            LDAA  #%10000100 ; Enable TOI and select prescale factor equal to 16
            STAA  TSCR2
            RTS

;*******************************************************************
TOF_ISR     INC   TOF_COUNTER
            LDAA  #%10000000       ; Clear
            STAA  TFLG2        ; TOF
            RTI

;---------------------------------------------------------------------------
; Position the Cursor
; This routine positions the display cursor in preparation for the writing
; of a character or string.
; For a 20x2 display:
; The first line of the display runs from 0 .. 19.
; The second line runs from 64 .. 83.
; The control instruction to position the cursor has the format
; 1aaaaaaa
; where aaaaaaa is a 7 bit address.
; Passed: 7 bit cursor Address in ACCA
; Returns: Nothing
; Side Effects: None

LCD_POS_CRSR            ORAA #%10000000 ; Set the high bit of the control word
                        JSR cmd2LCD ; and set the cursor address
                        RTS

;**************************************************************
;*                 Interrupt Vectors                       *
;**************************************************************

            ORG   $FFFE
            DC.W  Entry           ; Reset Vector
