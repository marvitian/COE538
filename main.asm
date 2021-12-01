;**************************************************************
;       FINAL PROJECT COE538: EEBOT - SECTION 4
;       MANAV PATEL - 500967756
;       MARIO IANNIELLO - 
;       KAPILAN BALAKRISHNAN - 
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
            REVERSE EQU 2
            ALLSTOP EQU 3
            TURN_LEFT EQU 4
            TURN_RIGHT EQU 5

            PRIMARY_PATH EQU 0
            ALTERNATE_PATH EQU 1
            
            DIRECTION DC.B 1
; code section
            ORG   ROMStart

Entry:
_Startup:
            LDS   #RAMEnd+1       ; initialize the stack pointer

            CLI    ;ENABLE INTERUPTS
            
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
;
;
;




DISPATCHER   CMPA #START    ; IF START STATE CALL START ROUTINE
             BNE NOT_START
             JSR STATE_START   
             BRA DISP_EXIT ; EXIT (RTS)
             
NOT_START    CMPA #FORWARD  ;ELSE IF FORWARD
             BNE NOT_FORWARD
             JSR STATE_FORWARD
             JMP DISP_EXIT
             
NOT_FORWARD  CMPA #REVERSE  ;ELSE IF REVERSE 
             BNE NOT_REVERSE
             JSR STATE_REVERSE
             JMP DISP_EXIT
             
NOT_REVERSE  CMPA #TURN_RIGHT ;ELSE IF TURN RIGHT
             BNE NOT_ALL_STOP
             JSR STATE_RIGHT_TURN
             JMP DISP_EXIT 
             
NOT_RIGHT   CMPA #TURN_LEFT ;ELSE IF TURN LEFT
            BNE NOT_LEFT
            JSR STATE_LEFT_TURN
            JMP DISP_EXIT

NOT_LEFT    CMPA #ALLSTOP    
            BNE NOT_ALLSTOP
            JSR STATE_ALLSTOP  
            JMP DISP_EXIT     

        
            
NOT_ALL_STOP SWI
DISP_EXIT  RTS

;************;
;STATE SUBROUTINES
;***********;
STATE_START     BRCLR PORTAD0,$08,NO_FWD ; If REV BUMP
                JSR INIT_FWD ; Initialize the FORWARD state
                MOVB #FWD,CRNT_STATE ; Go into the FORWARD state
                BRA START_EXIT
;ADD TIMER DELAY TO AVOID DOUBLE PUSHING STOP
NO_FWD      NOP ; Else
START_EXIT  RTS ; return to the MAIN routine

;STATES ASSOCIATED WITH MOVING FORWARD 

STATE_FORWARD   PULD     ;PULL PREVIOUS DIRECTION 
                BRSET PORTAD0,$04,NO_FORWARD_BUMP  ;IF FRONT BUMP IS PRESSED
                LDAA ALTERNATE_PATH
                STAA DIRECTION                ; SWITCH DIRECTION
                JSR INIT_REV                  ; GET READY TO REVERSE
                MOVB #REVERSE,STATE_CURRENT   ;SET STATE TO REVERSE
                JMP MAIN 

NO_FORWARD_BUMP LDAA DETECTION_D  ; IF D SENSOR IS 1 THEN TURN RIGHT 
                BEQ NO_RIGHT_TURN ;
                LDAA DIRECTION      ;STORE DIRECTION IN STACK FOR REFERENCE 
                PSHA 
                LDAA PRIMARY_PATH ; THEN STORE THE FOLLOWING DIRECTION
                STAA DIRECTION
                JSR INIT_RIGHT_TURN   ; INITIALIZE RIGHT TURN
                MOVB #TURN_RIGHT,STATE_CURRENT
                JMP MAIN

NO_RIGHT_TURN   LDAA DETECTION_B   ;ELSE IF THERE IS A LEFT
                BEQ NO_LEFT_TURN  
                LDAA DETECTION_A   ;CHECK IF YOU CAN GO STRAIGHT
                BEQ IS_LEFT_TURN   ;IF SO YOU SHOULD GO STRAIGHT
                LDAA DIRECTION     ;STORE THIS DIRECTION
                PSHA                
                LDAA PRIMARY_PATH 
                STAA DIRECTION     ; NEXT DIRECTION SHOULD BE PRIMARY ROUTE 



;**************************************************************
;*                 MOTOR CONTROL                              *
;**************************************************************



;**************************************************************
;*                 SUBROUTINE SECTION FOR SENSOR              *
;**************************************************************
MAIN                    JSR G_LEDS_ON ; Enable the guider LEDs
                        JSR READ_SENSORS ; Read the 5 guider sensors
                        JSR G_LEDS_OFF ; Disable the guider LEDs
                        JSR DISPLAY_SENSORS ; and write them to the LCD
                        LDY #6000 ; 300 ms delay to avoid
                        JSR del_50us ; display artifacts
                        BRA MAIN ; Loop forever

                        
                        STAA DETECTION_A        ;Setting Sensor A
                        STAA DETECTION_B        ;Setting Sensor B
                        STAA DETECTION_C        ;Setting Sensor C
                        STAA DETECTION_D        ;Setting Sensor D
                        STAA DETECTION_E        ;Setting Sensor E
                        STAA DETECTION_F        ;Setting Sensor F



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
