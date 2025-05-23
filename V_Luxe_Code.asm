PROCESSOR 18F4331

#include <xc.inc>
CONFIG  OSC = IRC             ; Internal oscillator block, CLKO function on RA6 and port function on RA7
CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor enabled
CONFIG  IESO = OFF            ; Internal External Switchover mode disabled
CONFIG  PWRTEN = ON           ; Power-up Timer enabled
CONFIG  BOREN = ON            ; Brown-out Reset enabled
CONFIG  BORV = 27             ; Brown-out Reset Voltage trip point low
CONFIG  WDTEN = OFF           ; Watchdog Timer disabled
CONFIG  WDPS = 1              ; Watchdog Timer Postscale Select bits
CONFIG  WINEN = OFF           ; Watchdog Timamakemer Window Enable bit
CONFIG  PWMPIN = OFF          ; PWM output pins Reset state control
CONFIG  LPOL = LOW            ; PWM0, 2, 4 and 6 are active-low
CONFIG  HPOL = LOW            ; PWM1, 3, 5 and 7 are active-low
CONFIG  T1OSCMX = ON          ; Low-power Timer1 operation when microcontroller is in Sleep mode
CONFIG  FLTAMX = "RC1"        ; FLTA input is multiplexed with RC1
CONFIG  SSPMX = "RC7"         ; SCK/SCL clocks and SDA/SDI data are multiplexed with RC5 and RC4, respectively. SDO output is multiplexed with RC7.
CONFIG  PWM4MX = "RB5"        ; PWM4 output is multiplexed with RB5
CONFIG  EXCLKMX = "RC3"       ; TMR0/T5CKI external clock input is multiplexed with RC3
CONFIG  MCLRE = ON            ; MCLR/VPP pin function is MCLR
CONFIG  STVREN = ON           ; Stack full/underflow will cause Reset
CONFIG  LVP = OFF             ; Low-voltage ICSP enabled
CONFIG  DEBUG = OFF           ; Background debugger disabled; RB6 and RB7 configured as general purpose I/O pins
CONFIG  CP0 = OFF             ; Block 0 (000200-000FFFh) not code-protected
CONFIG  CP1 = OFF             ; Block 1 (001000-001FFF) not code-protected
CONFIG  CPB = OFF             ; Flash Program Memory Code Protection off
CONFIG  CPD = OFF             ; Data Memory Code Protection off
CONFIG  WRT0 = OFF            ; Block 0 (000200-000FFFh) not write-protected
CONFIG  WRT1 = OFF            ; Block 1 (001000-001FFF) not write-protected
CONFIG  WRTC = OFF            ; Configuration registers (300000-3000FFh) not write-protected
CONFIG  WRTB = OFF            ; Boot Block (000000-0001FFh) not write-protected
CONFIG  WRTD = OFF            ; Data EEPROM not write-protected
CONFIG  EBTR0 = OFF           ; Block 0 (000200-000FFFh) not protected from table reads executed in other blocks
CONFIG  EBTR1 = OFF           ; Block 1 (001000-001FFF) not protected from table reads executed in other blocks
CONFIG  EBTRB = OFF           ; Block 1 (001000-001FFF) not protected from table reads executed in other blocks

#define kp 0x0019
PSECT reset_vec, class = CODE, delta = 1
reset_vec:
    goto INITIALIZATION

PSECT isrh_vec, class = CODE, delta = 1
    goto ISRH
    retfie

PSECT isrl_vec, class = CODE, delta = 1
    goto ISRL
    retfie
    
ISRH:
    BTFSC INTCON,2,0
    goto ISR_TIMER_0    ; Test if interrupt is due to timer 0

    BTFSC PIR3,3,0
    goto DIRECTION_QEI

    BTFSC PIR3,2,0
    goto OVERFLOW_QEI

    retfie

ISRL:
    retfie  ; we have no interrupts of low priority

ISR_TIMER_0:
    ; === FAST FLAG MANAGEMENT ===
        MOVLW 0b01000010            ; max = 66 as 3.34Khz/66 = 50Hz 
        CPFSLT Fast_Counter,0
        call RISE_FAST_COUNTER      ; if fast counter > max, restart & rise flag fast
        INCF Fast_Counter,1,0

    ; === TIMER 0 MANAGEMENT ===
        MOVLW 0b11111111
        MOVWF TMR0H,0
        MOVLW 0b11000000            ; loads counter with predefined value
        MOVWF TMR0L,0               ; to speed up interrupts up to 3.34KHz

    ; === THINGS TO BE DONE AT 3.34KHz ===
        ; WARNING : SHOULD BE VERY SHORT, WE ARE IN 3.34KHz INTERRUPT
        
        ; Test if we are in an I2C transaction
        BTFSC D0_I2C,0,0
        call I2C_ISR

        BTFSC FLAG_I2C_SCREEN,0,0 ;nothing to send to the screen 
        call TIMER0_ISR_LCD

    ; === INTERRUPT MANAGEMENT ===
        BCF INTCON,2,0  ; Clears the interrupt flag
        BSF INTCON,5,0  ; Re-enables the interrupt
    retfie

RISE_FAST_COUNTER:
    ; === RISE FAST FLAG ===
        BSF Flag_fast,0,0
        CLRF Fast_Counter,0

    ; === SLOW FLAG MANAGEMENT ===
        MOVLW 0b00110010            ; max = 50 as 50Hz/50 = 1Hz
        CPFSLT Slow_Counter,0
        call RISE_SLOW_COUNTER      ; if slow counter > max, restart & rise flag slow
        INCF Slow_Counter,1,0

    return

RISE_SLOW_COUNTER:
    ; === RISE SLOW FLAG ===
        BSF Flag_slow,0,0
        CLRF Slow_Counter,0

    return 
; === CODE FOR THE QEI ==== 
DIRECTION_QEI:
    BCF PIR3,3,0
    ; code to do when Direction changes :
    retfie
    
OVERFLOW_QEI:
    BCF PIR3,2,0
    ; === THINGS TO BE DONE AT QEI OVERFLOW ===
    ;BTG LATD,7,0    ; Toggle Blue LED
    BCF LATC,5,0    ; Shuts the DC motor down to avoid de-railing
    retfie   


; ==== CODE FOR THE I2C ====
I2C_ISR:
    BCF TRANS_TYPE,0,0
    BTFSC READ_DATA_FLAG,0,0    ;if READ_DATA_FLAG is clear than no reading requested and TRANS_TYPE is kept clear
    BSF TRANS_TYPE,0,0       

    BTFSS TIM_STATUS,0,0        ;If TIM_STATUS is set the SCL is toggled, otherwise the SDA is changed (when writing)
    goto change_data
    goto change_clock

TIMER0_ISR_END:             ; Function for what must always be done at the end of the timer interupt
    BTG TIM_STATUS,0,0      ;toggle the time status
    return
    
change_clock: 
    BTFSC LATD,0,0
    goto check_ifACK        ; If we are in a High to low transition we need to check if the following I2C mission is a ACK to prepare it
    goto LH_transition

check_ifACK:
    BTG LATD,2,0            ; New value of SCL
    BTG LATD,0,0            ; New value of SCL
    BTFSC LAST_CLOCK,0,0
    BCF D0_I2C,0,0
    MOVLW 0b00000011
    CPFSEQ STATUS_I2C,0     ; check the status to see if we are going to send or read an ACK 
    goto prepa_send_ack
    goto prepa_read_ack
    
prepa_read_ack:
    BCF LATD,1,0
    BCF LATD,3,0
    BSF TRISD,1,0          ; Before reading an ACK we release the SDA line
    BSF TRISD,3,0          ; Before reading an ACK we release the SDA line
    BSF FLAG_PREPA_ACK,0,0
    goto TIMER0_ISR_END

prepa_send_ack:
    BTFSC FLAG_ACK,0,0      ; If the flag is set we already sent the ACK so -> end_send_ack
    goto end_send_ack
    MOVLW 0b00000101
    CPFSEQ STATUS_I2C,0     ; If the status is send_ack (because not check before)
    goto TIMER0_ISR_END
    BCF TRISD,1,0           ; If we didn't already send the ACK we take back the SDA line as output and clear the line (ACK)
    BCF LATD,1,0
    BCF TRISD,3,0           
    BCF LATD,3,0
    goto TIMER0_ISR_END

end_send_ack:
    BSF TRISD,1,0           ; If the ACK has been sent we release the SDA line 
    BSF TRISD,3,0           ; If the ACK has been sent we release the SDA line
    BCF FLAG_ACK,0,0        ; We clear the flag
    goto TIMER0_ISR_END

LH_transition:
    MOVLW 0b00000010
    CPFSGT STATUS_I2C,0
    goto end_transition
    MOVLW 0b00000011        ; On a low to high transition there are 3 possibilities: read_data, read_ack, send_ack if requested
    CPFSGT STATUS_I2C,0
    goto read_ACK
    MOVLW 0b00000100
    CPFSGT STATUS_I2C,0
    goto read_data
    MOVLW 0b00000101
    CPFSGT STATUS_I2C,0
    goto send_ACK
    goto end_transition

end_transition: 
    BTG LATD,2,0            ; New value of SCL
    BTG LATD,0,0            ; New value of SCL
    BTFSC LAST_CLOCK,0,0
    BCF D0_I2C,0,0
    goto TIMER0_ISR_END
    
change_data: 
    CLRF WREG,0
    MOVFF ASK_DATA, WREG
    IORWF READ_DATA_FLAG,0,0
    BTFSS WREG,0,0
    goto TIMER0_ISR_END     ; If neither ASK_DATA or _FLAG is set then we do nothing

    BTFSS LATD,0,0          ; Otherwise we go to the fonction corresponding to the current SCL level
    goto CLK_low            
    goto CLK_high           

CLK_high:               ;3 possible cases: start, stop or end of an ack we receive
    MOVLW 0b00000111
    CPFSLT STATUS_I2C,0
    goto stop

    MOVLW 0b00000110
    CPFSLT STATUS_I2C,0
    goto end_ACK
    
    TSTFSZ STATUS_I2C,0
    goto TIMER0_ISR_END
    goto start
    
start:
    MOVFF ADD_TH,CUR_TH     ; We load the current byte with the address of the TH sensor  
    CLRF COUNT_TH,0         ; We clear counters
    CLRF COUNT_DATA,0
    BTFSC TRANS_TYPE,0,0    ; If we are in reading mode, clear the flag because we will change the register   
    BSF CUR_TH,7,0          ; Change the bit 7 of the first byte sent to say if we are in writing and reading mode thanks to the value of the flag TRANS_TYPE
    BCF TRISD,1,0           ; We put SDA as output (security)
    BCF LATD,1,0            ; START (SDA lowered on a high level of SCL)
    BCF TRISD,3,0           ; We put SDA as output (security)
    BCF LATD,3,0            ; START (SDA lowered on a high level of SCL)
    MOVLW 0b00000001
    MOVWF STATUS_I2C,0      ; Status = send address
    goto TIMER0_ISR_END

stop:
    BCF TRISD,1,0           ; we put SDA as output (security)
    BSF LATD,1,0            ; STOP (SDA to an high level when SCL is high)
    BCF TRISD,3,0           ; we put SDA as output (security)
    BSF LATD,3,0            ; STOP (SDA to an high level when SCL is high)
    CLRF STATUS_I2C,0       ; status = start

    ; === DEBUGG TO BE ABLE TO READ REGISTERS ON MPLab ===
        MOVFF S_DATA_EXT, WREG 
        MOVFF H_DATA1_EXT, WREG
        MOVFF H_DATA2_EXT, WREG
        MOVFF HT_DATA_EXT, WREG
        MOVFF T_DATA1_EXT, WREG
        MOVFF T_DATA2_EXT, WREG
        MOVFF CRC_DATA_EXT, WREG
        
        MOVFF S_DATA_INT, WREG 
        MOVFF H_DATA1_INT, WREG
        MOVFF H_DATA2_INT, WREG
        MOVFF HT_DATA_INT, WREG
        MOVFF T_DATA1_INT, WREG
        MOVFF T_DATA2_INT, WREG
        MOVFF CRC_DATA_INT, WREG

    ; === FNINISHED (ASK MODE) ===
        BTFSS TRANS_TYPE,0,0
        BSF LAST_CLOCK,0,0          ; we have done the I2C, wait to re-enable again
        BTFSS TRANS_TYPE,0,0
        goto TIMER0_ISR_END

    ; === FORMAT DATA FOR TEMP EXT===
        SWAPF HT_DATA_EXT,0,0
        MOVWF T_DATA_EXT,0
        BCF T_DATA_EXT,0,0
        BCF T_DATA_EXT,1,0
        BCF T_DATA_EXT,2,0
        BCF T_DATA_EXT,3,0

        SWAPF T_DATA1_EXT,0,0
        BCF WREG,7,0
        BCF WREG,6,0
        BCF WREG,5,0
        BCF WREG,4,0
        IORWF T_DATA_EXT,1,0
        MOVFF T_DATA_EXT,WREG  ;debug 

    ; === FORMAT DATA FOR TEMP INT===
        SWAPF HT_DATA_INT,0,0
        MOVWF T_DATA_INT,0
        BCF T_DATA_INT,0,0
        BCF T_DATA_INT,1,0
        BCF T_DATA_INT,2,0
        BCF T_DATA_INT,3,0

        SWAPF T_DATA1_INT,0,0
        BCF WREG,7,0
        BCF WREG,6,0
        BCF WREG,5,0
        BCF WREG,4,0
        IORWF T_DATA_INT,1,0
        MOVFF T_DATA_INT,WREG   ;debug

    ; === PROPCESS DATA FOR HUM EXT ===
        MOVLW 0b00000011
        MOVWF HUM_EXT,0
        MOVLW 0b10011001
        CPFSGT H_DATA1_EXT,0
        BCF HUM_EXT,1,0
        MOVLW 0b01100110
        CPFSGT H_DATA1_EXT,0
        BCF HUM_EXT,0,0
        MOVFF HUM_EXT, WREG	;debug
    
    ; === PROPCESS DATA FOR HUM INT ===
        MOVLW 0b00000011
        MOVWF HUM_INT,0
        MOVLW 0b10011001
        CPFSGT H_DATA1_INT,0
        BCF HUM_INT,1,0
        MOVLW 0b01100110
        CPFSGT H_DATA1_INT,0
        BCF HUM_INT,0,0
        MOVFF HUM_INT, WREG	;debug

    ; === USES THE LOOK UP TABLE FOR TEMPERATURE EXT ====
        MOVLW 0b00111001
        SUBWF T_DATA_EXT, 0, 0      ; WREG = T_DATA_INT - 0b00111001 (donc l'index)
        RLNCF WREG,1,0              ; Multiply index by two to match memory
        CALL TEMP_LUT               ; Jump into table
        MOVWF TEMP_EXT, 0           ; Store result

    ; === USES THE LOOK UP TABLE FOR TEMPERATURE INT ===
        MOVLW 0b00111001
        SUBWF T_DATA_INT, 0, 0      ; WREG = T_DATA_INT - 0b00111001 (donc l'index)
        RLNCF WREG,1,0              ; Multiply index by two to match memory
        CALL TEMP_LUT               ; Jump into table
        MOVWF TEMP_INT, 0           ; Store result

    ; === FNINISHED (READ MODE) ===
        BSF LAST_CLOCK,0,0              ; we have done the I2C, wait to re-enable again
    goto TIMER0_ISR_END


CLK_low:                ;4 possible cases: send_add, send_data, read_data, send_ACK
    MOVLW 0b00000000
    CPFSGT STATUS_I2C,0
    goto TIMER0_ISR_END
    MOVLW 0b00000001
    CPFSGT STATUS_I2C,0
    goto send_add
    MOVLW 0b00000010
    CPFSGT STATUS_I2C,0
    goto send_data
    MOVLW 0b00000111
    CPFSGT STATUS_I2C,0
    goto TIMER0_ISR_END
    
send_add:
    BCF TRISD,1,0           ; We put SDA as output (security)
    BCF TRISD,3,0           ; We put SDA as output (security)
    
    BTFSS CUR_TH,0,0        ; We copy the value of the bit to sent (bit 0 of CUR_TH) in SDA
    call send_zero

    BTFSC CUR_TH,0,0
    call send_one
    
    RRNCF CUR_TH,1,0        ; We rotate the current byte
    INCF COUNT_DATA,1,0     ; We increment the counter of the number of data sent

    MOVLW 0x07
    ANDWF COUNT_DATA,0,0 
    TSTFSZ WREG,0
    goto TIMER0_ISR_END      ; If the counter is a multiple of 8 we sent the whole address else we continue sending
                        
    MOVFF TH1, CUR_TH       ; If the address has been sent, we load the next byte to sent
    MOVLW 0b00000001
    MOVWF COUNT_TH,0        ; We increment the counter of byte sent
    MOVLW 0b00000011
    MOVWF STATUS_I2C,0      ; Status = read_ack
    goto TIMER0_ISR_END

send_data: 
    BCF TRISD,1,0           ; We put SDA as output (security)
    BCF TRISD,3,0           ; We put SDA as output (security)

    BTFSS CUR_TH,0,0        ; We copy the value of the bit to sent (bit 0 of CUR_TH) in SDA
    call send_zero

    BTFSC CUR_TH,0,0
    call send_one
    
    RRNCF CUR_TH,1,0        ; We rotate the current byte
    INCF COUNT_DATA,1,0     ; We increment the counter of the number of data sent

    MOVLW 0x07
    ANDWF COUNT_DATA,0,0 
    TSTFSZ WREG,0
    goto TIMER0_ISR_END      ; If the counter is a multiple of 8 we need to change the current byte to send and go to read_ack


    MOVLW 0b00000011        ; Status = read_ACK
    MOVWF STATUS_I2C,0
    
    INCF COUNT_TH,1,0       ; We increment  the counter of byte

    MOVLW 0b00000001        ; Remplace the current byte thanks to the counter of byte 
    CPFSGT COUNT_TH,0
    goto PUT_TH1
    MOVLW 0b00000010
    CPFSGT COUNT_TH,0
    goto PUT_TH2
    MOVLW 0b00000011
    CPFSGT COUNT_TH,0
    goto PUT_TH3
    
    goto TIMER0_ISR_END

send_zero:
    BCF LATD,1,0
    BCF LATD,3,0
    return

send_one:
    BSF LATD,1,0
    BSF LATD,3,0
    return

PUT_TH1:
    MOVFF TH1, CUR_TH
    goto TIMER0_ISR_END

PUT_TH2:
    MOVFF TH2, CUR_TH
    goto TIMER0_ISR_END

PUT_TH3:
    MOVFF TH3, CUR_TH
    goto TIMER0_ISR_END

read_ACK:
    ; What do we do if we get a NACK ?
    ; -> nothing, the next communication will certainly work 

    BTFSS FLAG_PREPA_ACK,0,0
    goto end_transition
    MOVLW 0b00000110        
    MOVWF STATUS_I2C,0      ; Status = end_ack
    BCF FLAG_PREPA_ACK,0,0
    MOVLW 0b00011111        ; If the counter of the date is a multpile of 32 all the data have been sent -> status = stop
    ANDWF COUNT_DATA,0,0 
    TSTFSZ WREG,0
    goto end_transition
    
    MOVLW 0b00000111
    MOVWF STATUS_I2C,0

    goto end_transition
    
end_ACK: 
    BTFSC TRANS_TYPE,0,0    ; If we are in reading mode, we don't need to take back the line SDA
    goto start_reading

    BCF TRISD,1,0           ; If we are in writing mode we take back the line and put it to a low level to avoid conflict (START or STOP non-desired)
    BCF LATD,1,0
    BCF TRISD,3,0           ; If we are in writing mode we take back the line and put it to a low level to avoid conflict (START or STOP non-desired)
    BCF LATD,3,0
    MOVLW 0b00000010        
    MOVWF STATUS_I2C,0      ; Status = send_data
    goto TIMER0_ISR_END

start_reading:
    CLRF CUR_TH,0
    CLRF CUR_TH2,0
    MOVLW 0b00000100   
    MOVWF STATUS_I2C,0      ; Status = read_data
    goto TIMER0_ISR_END
        
send_ACK: 
    MOVLW 0b00000101
    CPFSEQ STATUS_I2C,0
    goto end_transition
    
    BCF TRISD,1,0           ; Send an ACK, pin as output and cleared
    BCF LATD,1,0
    BCF TRISD,3,0           ; Send an ACK, pin as output and cleared
    BCF LATD,3,0
    BSF FLAG_ACK,0,0        ; We raise the flag to know the ACK has been sent
    MOVLW 0b00000100
    MOVWF STATUS_I2C,0      ; Status = read_data

    MOVLW 0b00111111        ; If the counter of the date is a multpile of 64 all the data have been received -> status = stop
    ANDWF COUNT_DATA,0,0 
    TSTFSZ WREG,0
    goto end_transition
    
    MOVLW 0b00000111
    MOVWF STATUS_I2C,0

    goto end_transition

read_data:
    MOVLW 0b00000100        ; Check if the status is read_data
    CPFSEQ STATUS_I2C,0
    goto end_transition
    
    RLNCF CUR_TH,1,0        ; We rotate the current byte
    RLNCF CUR_TH2,1,0       ; We rotate the current byte

    BSF TRISD,1,0           ; Put SDA1 as input (normally no need but to be sure)
    BSF TRISD,3,0           ; Put SDA2 as input (normally no need but to be sure)

    BTFSS PORTD,1,0         ; We copy the value of the bit to received in SDA to the current register to fill
    BCF CUR_TH,0,0

    BTFSC PORTD,1,0
    BSF CUR_TH,0,0

    BTFSS PORTD,3,0         ; We copy the value of the bit to received in SDA to the current register to fill
    BCF CUR_TH2,0,0

    BTFSC PORTD,3,0
    BSF CUR_TH2,0,0
    
    INCF COUNT_DATA,1,0     ; We increment the counter of the number of data received

    MOVLW 0x07
    ANDWF COUNT_DATA,0,0 
    TSTFSZ WREG,0           ; If the counter is a multiple of 8 we need to fill the register and go to send_ack
    goto end_transition
    
    MOVLW 0b00000101        ; Status = send_ACK
    MOVWF STATUS_I2C,0
    
    MOVLW 0b00000000
    CPFSGT COUNT_TH,0
    goto TIMER0_ISR_END
    MOVLW 0b00000001
    CPFSGT COUNT_TH,0
    goto fill_S_DATA
    MOVLW 0b00000010
    CPFSGT COUNT_TH,0
    goto fill_H_DATA1
    MOVLW 0b00000011
    CPFSGT COUNT_TH,0
    goto fill_H_DATA2
    MOVLW 0b00000100
    CPFSGT COUNT_TH,0
    goto fill_HT_DATA
    MOVLW 0b00000101
    CPFSGT COUNT_TH,0
    goto fill_T_DATA1
    MOVLW 0b00000110
    CPFSGT COUNT_TH,0
    goto fill_T_DATA2
    MOVLW 0b00000111
    CPFSGT COUNT_TH,0
    goto fill_CRC_DATA

    goto end_transition
   
fill_S_DATA:
    MOVFF CUR_TH, S_DATA_EXT
    MOVFF CUR_TH2, S_DATA_INT
    INCF COUNT_TH,1,0           ; We increment  the counter of byte
    goto end_transition

fill_H_DATA1: 
    MOVFF CUR_TH, H_DATA1_EXT
    MOVFF CUR_TH2, H_DATA1_INT
    INCF COUNT_TH,1,0           ; We increment  the counter of byte
    goto end_transition

fill_H_DATA2:
    MOVFF CUR_TH, H_DATA2_EXT
    MOVFF CUR_TH2, H_DATA2_INT
    INCF COUNT_TH,1,0           ; We increment  the counter of byte
    goto end_transition

fill_HT_DATA:
    MOVFF CUR_TH, HT_DATA_EXT
    MOVFF CUR_TH2, HT_DATA_INT
    INCF COUNT_TH,1,0           ; We increment  the counter of byte
    goto end_transition

fill_T_DATA1:
    MOVFF CUR_TH, T_DATA1_EXT
    MOVFF CUR_TH2, T_DATA1_INT
    INCF COUNT_TH,1,0           ; We increment  the counter of byte
    goto end_transition

fill_T_DATA2:
    MOVFF CUR_TH, T_DATA2_EXT
    MOVFF CUR_TH2, T_DATA2_INT
    INCF COUNT_TH,1,0           ; We increment  the counter of byte
    goto end_transition

fill_CRC_DATA:
    MOVFF CUR_TH, CRC_DATA_EXT
    MOVFF CUR_TH2, CRC_DATA_INT
    INCF COUNT_TH,1,0           ;We increment  the counter of byte
    goto end_transition 
  
; === code for screen i2C ===
TIMER0_ISR_LCD:  
        BTFSS INTCON,2,0         ;Check if the flag for the timer0 interrupt is set 
        retfie
        BCF INTCON,2,0           ;Clear interrupt flag

        BTFSS TIM_STATUS_LCD,0,0     ;If TIM_STATUS_LCD is set the SCL is toggled, otherwise the SDA is changed (when writing)
        goto change_data_LCD
        goto change_clock_LCD


TIMER0_ISR_END_LCD:                 ;function for what must always be done at the end of the timer interupt
	MOVLW 0b11111111
	MOVWF TMR0H,0
	MOVLW 0b11000000
	MOVWF TMR0L,0
    BTG TIM_STATUS_LCD,0,0      ;toggle the time status
    BSF INTCON,5,0          ;Enable TMR0IE
    return
        
change_clock_LCD:
        BTFSC LATD,4,0
        goto check_ifACK_LCD        ;if we are in a High to low transition we need to check if the following I2C mission is a ACK to prepare it
        goto LH_transition_LCD

check_ifACK_LCD:
    BTG LATD,4,0            ; New value of SCL
    MOVLW 0b00000011
    CPFSEQ STATUS_I2C_LCD,0     ; check the status to see if we are going to read an ACK 
    goto TIMER0_ISR_END_LCD
    goto prepa_read_ack_LCD
    
prepa_read_ack_LCD:
    BCF LATD,5,0
    BSF TRISD,5,0          ; Before reading an ACK we release the SDA line
    BSF FLAG_PREPA_ACK_LCD,0,0
    goto TIMER0_ISR_END_LCD


LH_transition_LCD:
    MOVLW 0b00000010
    CPFSGT STATUS_I2C_LCD,0
    goto end_transition_LCD
    MOVLW 0b00000011        ; On a low to high transition there are 3 possibilities: read_data, read_ACK_LCD, send_ack if requested
    CPFSGT STATUS_I2C_LCD,0
    goto read_ACK_LCD
    goto end_transition_LCD

end_transition_LCD: 
    BTG LATD,4,0            ; New value of SCL
    goto TIMER0_ISR_END_LCD
          
change_data_LCD: 
        BTFSS LATD,4,0          ;otherwise we go to the fonction corresponding to the current SCL level
        goto CLK_low_LCD            
        goto CLK_high_LCD  
         

CLK_high_LCD:  ;3 possible cases: start, stop or end of an ack we received
        
        MOVLW 0b00000111
        CPFSLT STATUS_I2C_LCD,0
        goto stop_LCD

        MOVLW 0b00000110
        CPFSLT STATUS_I2C_LCD,0
        goto end_ACK_LCD
        
        
        TSTFSZ STATUS_I2C_LCD,0
        goto TIMER0_ISR_END_LCD
        goto start_LCD
        
        
start_LCD:
        MOVFF ADD_SCREEN,CUR_TH_LCD     ; we load the current byte with the address of the TH sensor  
        ;CLRF COUNT_TH_LCD,0         ; we clear counters
	BSF FLAG_FIRST_BYTE,0,0
        CLRF COUNT_DATA_LCD,0
        BCF TRISD,5,0           ; we put SDA as output (security)
        BCF LATD,5,0            ; START (SDA lowered on a high level of SCL)
        MOVLW 0b00000010
        MOVWF STATUS_I2C_LCD,0      ; status = send data
        goto TIMER0_ISR_END_LCD
stop_LCD:
        BCF TRISD,5,0           ; we put SDA as output (security)
        BSF LATD,5,0            ; STOP (SDA to an high level when SCL is high)
        CLRF STATUS_I2C_LCD,0       ; status = start

        BTFSS FLAG_INIT_DONE,0,0
        goto init_not_done_LCD

        BTFSS SCREEN_MODE,0,0
        goto clear_not_done_LCD

        MOVLW 0b00001100
        CPFSEQ COUNT_TH_LCD,0
        goto TIMER0_ISR_END_LCD
        CLRF COUNT_TH_LCD,0
        BCF FLAG_I2C_SCREEN,0,0

        goto TIMER0_ISR_END_LCD

init_not_done_LCD:
	MOVFF COUNT_TH_LCD, WREG
        MOVLW 0x18
        CPFSEQ COUNT_TH_LCD,0
        goto TIMER0_ISR_END_LCD
        CLRF COUNT_TH_LCD,0
        BSF FLAG_INIT_DONE,0,0
        BSF FLAG_I2C_SCREEN,0,0
        BSF SCREEN_MODE,0,0
        goto TIMER0_ISR_END_LCD

clear_not_done_LCD:
        MOVLW 0b00000100
        CPFSEQ COUNT_TH_LCD,0
        goto TIMER0_ISR_END_LCD
        CLRF COUNT_TH_LCD,0
        BSF FLAG_I2C_SCREEN,0,0
        BSF SCREEN_MODE,0,0
        goto TIMER0_ISR_END_LCD


CLK_low_LCD:                   
        MOVLW 0b00000000
        CPFSGT STATUS_I2C_LCD,0
        goto TIMER0_ISR_END_LCD
        MOVLW 0b00000010
        CPFSGT STATUS_I2C_LCD,0
        goto send_data_LCD
        MOVLW 0b00000111
        CPFSGT STATUS_I2C_LCD,0
        goto TIMER0_ISR_END_LCD
          
send_data_LCD: 

        RLNCF CUR_TH_LCD,1,0        ; we rotate the current byte
        BCF TRISD,5,0           ; we put SDA as output (security)

        BTFSS CUR_TH_LCD,0,0        ; we copy the value of the bit to sent (bit 0 of CUR_TH_LCD) in SDA
        BCF LATD,5,0

        BTFSC CUR_TH_LCD,0,0
        BSF LATD,5,0
        
        INCF COUNT_DATA_LCD,1,0     ; we increment the counter of the number of data sent

        MOVLW 0x07
        ANDWF COUNT_DATA_LCD,0,0 
        TSTFSZ WREG,0
        goto TIMER0_ISR_END_LCD      ;if the counter is a multiple of 8 we need to change the current byte to send and go to read_ACK_LCD

        MOVLW 0b00000011        ;status = read_ACK_LCD
        MOVWF STATUS_I2C_LCD,0
	BCF FLAG_PREPA_ACK_LCD,0,0
	
	MOVLW 0b00001000
	CPFSEQ COUNT_DATA_LCD,0   ;if COUNT_DATA_LCD = 8 the address was sent so we don't inc COUNT_TH_LCD  
	INCF COUNT_TH_LCD,1,0       ;we increment  the counter of byte
        goto TIMER0_ISR_END_LCD

	
find_next_byte_LCD:
	BCF FLAG_FIRST_BYTE,0,0
	
        BTFSS FLAG_INIT_DONE,0,0
        goto next_init_byte_LCD   ;if we are in init mode -> take the next byte of initialization

        BTFSS SCREEN_MODE,0,0
        goto next_clear_byte_LCD ;if we arez in clear mode -> next clear byte

        ;else, we are in cursor + temp mode

        MOVLW 0b00000000
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT1_LCD
        MOVLW 0b00000001
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT2_LCD
        MOVLW 0b00000010
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT3_LCD
        MOVLW 0b00000011
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT4_LCD
        MOVLW 0b00000100
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT5_LCD
        MOVLW 0b00000101
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT6_LCD
        MOVLW 0b00000110
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT7_LCD
        MOVLW 0b00000111
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT8_LCD
        MOVLW 0b00001000
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT9_LCD
        MOVLW 0b00001001
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT10_LCD
        MOVLW 0b00001010
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT11_LCD
        MOVLW 0b00001011
        CPFSGT COUNT_TH_LCD,0
        goto PUT_DAT12_LCD
        goto end_transition_LCD

next_init_byte_LCD:
        MOVFF COUNT_TH_LCD, WREG
        RLNCF WREG,1,0          ; Multiply index by two to match memory
        CALL INIT_BYTE
        MOVWF CUR_TH_LCD,0
        goto end_transition_LCD

next_clear_byte_LCD:
        MOVLW 0b00000000        ; Remplace the current byte thanks to the counter of byte 
        CPFSGT COUNT_TH_LCD,0
        goto PUT_CDAT1_LCD
        MOVLW 0b00000001
        CPFSGT COUNT_TH_LCD,0
        goto PUT_CDAT2_LCD
        MOVLW 0b00000010
        CPFSGT COUNT_TH_LCD,0
        goto PUT_CDAT3_LCD
        MOVLW 0b00000011
        CPFSGT COUNT_TH_LCD,0
        goto PUT_CDAT4_LCD

PUT_CDAT1_LCD:
        MOVFF CLEAR_DAT1, CUR_TH_LCD
        goto end_transition_LCD
PUT_CDAT2_LCD:
        MOVFF CLEAR_DAT2, CUR_TH_LCD
        goto end_transition_LCD
PUT_CDAT3_LCD:
        MOVFF CLEAR_DAT3, CUR_TH_LCD
        goto end_transition_LCD
PUT_CDAT4_LCD:
        MOVFF CLEAR_DAT4, CUR_TH_LCD
        goto end_transition_LCD

PUT_DAT1_LCD:
        MOVFF DAT1, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT2_LCD:
        MOVFF DAT2, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT3_LCD:
        MOVFF DAT3, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT4_LCD:
        MOVFF DAT4, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT5_LCD:
        MOVFF DAT5, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT6_LCD:
        MOVFF DAT6, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT7_LCD:
        MOVFF DAT7, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT8_LCD:
        MOVFF DAT8, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT9_LCD:
        MOVFF DAT9, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT10_LCD:
        MOVFF DAT10, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT11_LCD:
        MOVFF DAT11, CUR_TH_LCD
        goto end_transition_LCD
PUT_DAT12_LCD:
        MOVFF DAT12, CUR_TH_LCD
        goto end_transition_LCD

    
read_ACK_LCD:
        ;lire l'ack mais on fait quoi si c'est NACK ??
        BTFSS FLAG_PREPA_ACK_LCD,0,0
        goto end_transition_LCD
	
        MOVLW 0b00000110        
        MOVWF STATUS_I2C_LCD,0      ;status = end_ack
        BCF FLAG_PREPA_ACK_LCD,0,0
	
	
	MOVFF COUNT_TH_LCD, WREG
	
	BTFSC FLAG_FIRST_BYTE,0,0
	goto find_next_byte_LCD

        MOVLW 0b00000011        ;if the counter of the date is a multpile of 4 all the data have been sent -> status = stop
        ANDWF COUNT_TH_LCD,0,0 
        TSTFSZ WREG,0
        goto find_next_byte_LCD

        MOVLW 0b00000111
        MOVWF STATUS_I2C_LCD,0
   
        goto end_transition_LCD
        
end_ACK_LCD: 
        BCF TRISD,5,0           ;if we are in writing mode we take back the line and put it to a low level to avoid conflict (START or STOP non-desired)
        BCF LATD,5,0
        MOVLW 0b00000010        
        MOVWF STATUS_I2C_LCD,0      ;status = send_data
        goto TIMER0_ISR_END_LCD




PSECT lut_temp, class = CODE, delta = 1
TEMP_LUT:
    MOVFF PCL, DEBUGG                   ; Look up table for temperature
    ADDWF PCL, 1, 0                     ; Stores the result of index+PCL in PCL which will decide what is the next line to perform
    RETLW 0b00000001        ;  -5.0°C
    RETLW 0b00000010        ;  -4.0°C
    NOP                                 ; NOP lines for intermediates values that coresponds to the same values
    RETLW 0b00000011        ;  -3.0°C
    RETLW 0b00000100        ;  -2.0°C
    RETLW 0b00000101        ;  -1.0°C
    NOP
    RETLW 0b00000110        ;   0.0°C
    RETLW 0b00000111        ;   1.0°C
    RETLW 0b00001000        ;   2.0°C
    RETLW 0b00001001        ;   3.0°C
    NOP
    RETLW 0b00001010        ;   4.0°C
    RETLW 0b00001011        ;   5.0°C
    RETLW 0b00001100        ;   6.0°C
    RETLW 0b00001101        ;   7.0°C
    NOP
    RETLW 0b00001110        ;   8.0°C
    RETLW 0b00001111        ;   9.0°C
    RETLW 0b00010000        ;  10.0°C
    NOP
    RETLW 0b00010001        ;  11.0°C
    RETLW 0b00010010        ;  12.0°C
    RETLW 0b00010011        ;  13.0°C
    RETLW 0b00010100        ;  14.0°C
    NOP
    RETLW 0b00010101        ;  15.0°C
    RETLW 0b00010110        ;  16.0°C
    RETLW 0b00010111        ;  17.0°C
    NOP
    RETLW 0b00011000        ;  18.0°C
    RETLW 0b00011001        ;  19.0°C
    RETLW 0b00011010        ;  20.0°C
    RETLW 0b00011011        ;  21.0°C
    NOP
    RETLW 0b00011100        ;  22.0°C
    RETLW 0b00011101        ;  23.0°C
    RETLW 0b00011110        ;  24.0°C
    NOP
    RETLW 0b00011111        ;  25.0°C
    RETLW 0b00100000        ;  26.0°C
    RETLW 0b00100001        ;  27.0°C
    RETLW 0b00100010        ;  28.0°C
    NOP
    RETLW 0b00100011        ;  29.0°C
    RETLW 0b00100100        ;  30.0°C
    RETLW 0b00100101        ;  31.0°C
    RETLW 0b00100110        ;  32.0°C
    NOP
    RETLW 0b00100111        ;  33.0°C
    RETLW 0b00101000        ;  34.0°C
    RETLW 0b00101001        ;  35.0°C
    NOP
    RETLW 0b00101010        ;  36.0°C
    RETLW 0b00101011        ;  37.0°C
    RETLW 0b00101100        ;  38.0°C
    RETLW 0b00101101        ;  39.0°C
    NOP
    RETLW 0b00101110        ;  40.0°C
    RETLW 0b00101111        ;  41.0°C
    RETLW 0b00110000        ;  42.0°C
    NOP
    RETLW 0b00110001        ;  43.0°C
    RETLW 0b00110010        ;  44.0°C
    RETLW 0b00110011        ;  45.0°C
    RETLW 0b00110100        ;  46.0°C
    NOP
    RETLW 0b00110101        ;  47.0°C
    RETLW 0b00110110        ;  48.0°C
    RETLW 0b00110111        ;  49.0°C
    NOP
    RETLW 0b00111000        ;  50.0°C

PSECT lut_LCD, class = CODE, delta = 1
INIT_BYTE:
    MOVFF PCL, DEBUGG_LCD         
    ADDWF PCL, 1, 0
    RETLW 0x3C
    RETLW 0x38
    RETLW 0x3C
    RETLW 0x38
    
    RETLW 0x3C
    RETLW 0x38
    RETLW 0x2C
    RETLW 0x28
    
    RETLW 0x2C
    RETLW 0x28
    RETLW 0x8C
    RETLW 0x88
    
    RETLW 0x0C
    RETLW 0x08
    RETLW 0xCC
    RETLW 0xC8
   
    RETLW 0x0C
    RETLW 0x08
    RETLW 0x6C
    RETLW 0x68
   
    RETLW 0x0C
    RETLW 0x08
    RETLW 0x1C
    RETLW 0x18

PSECT lut_temp_LCD, class = CODE, delta = 1
CONVERT_TEMP:
    MOVFF PCL, DEBUGG         
    ADDWF PCL, 1, 0
    goto load_18
    goto load_19 
    goto load_20
    goto load_21
    goto load_22
    goto load_23
    goto load_24

PSECT udata_acs
    ; ==== TIMING FLAGS ====
        Flag_fast:      DB 1    ; Flag for the fast clock 50Hz
        Fast_Counter:   DS 1    ; Counter for the Fast clock

        Flag_slow:      DB 1    ; Flag for the slow clock 1Hz
        Slow_Counter:   DS 1    ; Counter for the Slow clock

    ; ==== BUTTONS ====
        Btn_Mode:       DB 1    ; Mode button has been pressed  (RB5)
        Btn_Close:      DB 1    ; Close button has been pressed (RB4)
        Btn_Open:       DB 1    ; Open button has been pressed  (RB3)
        Btn_Down:       DB 1    ; Down button has been pressed  (RB2)
        Btn_Up:         DB 1    ; Up button has been pressed    (RB0)

    ; ==== ADC_RESULTS ===
        ADC_AN0_SHUNT : DS 1    ; Memory for the ADC_AN0 conversion result
        ADC_AN8_LUM :   DS 1    ; Memory for the ADC_AN8 conversion result
        SUNNY_FLAG :    DB 1    ; Flag: 0 = not sunny outside, 1 = sunny outside

    ; ==== I2C ====
        DEBUGG:         DS 1
        D0_I2C:         DB 1    ; Do we need to do I2C ?
        TH3:            DS 1    ; Data to send to TH sensor, octet 3        
        TH2:            DS 1    ; Data to send to TH sensor, octet 2        
        TH1:            DS 1    ; Data to send to TH sensor, octet 1
        ADD_TH:         DS 1    ; Data to send to TH sensor, octet 0 (LSB)       
        COUNT_DATA:     DB 9    ; Counter for the data to send (32 for TH sensor)
        STATUS_I2C:     DB 3    ; State of the I2C transmission      
        TIM_STATUS:     DB 1    ; 0 = change clock, 1 = change data 
        TRANS_TYPE:     DB 1    ; 0= writing, 1=reading    
        CUR_TH:         DS 1    ; Charge l'octet qu'on veut envoyer       
        CUR_TH2:        DS 1    ; Charge l'octet qu'on veut envoyer   
        COUNT_TH:       DB 4    ; Compteur pour savoir dans quel octet de TH on est        
        S_DATA_EXT:     DS 1    ; State send by the TH sensor           
        H_DATA1_EXT:    DS 1    ; Humidity data 1 send by the TH sensor         
        H_DATA2_EXT:    DS 1    ; Humidity data 2 send by the TH sensor       
        HT_DATA_EXT:    DS 1    ; Humidity temperature data send by the TH sensor          
        T_DATA1_EXT:    DS 1    ; Temperature data 1 send by the TH sensor         
        T_DATA2_EXT:    DS 1    ; Temperature data 2 send by the TH sensor          
        CRC_DATA_EXT:   DS 1    ; Crc data send by the TH sensor        
        S_DATA_INT:     DS 1    ; State send by the TH sensor           
        H_DATA1_INT:    DS 1    ; Humidity data 1 send by the TH sensor        
        H_DATA2_INT:    DS 1    ; Humidity data 2 send by the TH sensor      
        HT_DATA_INT:    DS 1    ; Humidity temperature data send by the TH sensor         
        T_DATA1_INT:    DS 1    ; Temperature data 1 send by the TH sensor         
        T_DATA2_INT:    DS 1    ; Temperature data 2 send by the TH sensor        
        CRC_DATA_INT:   DS 1    ; Crc data send by the TH sensor       
        T_DATA_INT:     DS 1       
        T_DATA_EXT:     DS 1       
        ASK_DATA:       DB 1    ; Flag to say if asking data           
        READ_DATA_FLAG: DB 1    ; Flag to say if reading data          
        FLAG_ACK:       DB 1    ; Flag to say if we have already send the ack         
        FLAG_PREPA_ACK: DB 1    ; Flag to say if we already did the prepa to read the ack       
        HUM_INT:        DB 2    ; 00 = low, 01 = normal, 11 = high     
        HUM_EXT:        DB 2         
        TEMP_INT:       DS 1               
        TEMP_EXT:       DS 1           
        LAST_CLOCK:     DB 1 
        TEMP_WANTED:    DS 1

    ; ==== INTERFACE VARIABLES ====
        CUR_MODE:       DB 1    ; 0 = automatic, 1 = manual

    ; ==== BLIND VARIABLES AND CONTROL ====
        BLIND_TARGET:   DS 1    ; Target position of the blind 
        BLIND_POSITION: DS 1    ; Current position of the blind given by the QEI
        ERROR_POSITION: DS 1
        PID_OUT1 : DS 1
        PID_OUT0 : DS 1
        PID_REG : DB 3
	    TMP : DS 1

    ; ==== SERVO VARIABLES ===
        SERVO_TARGET:   DS 1    ; target PWM for the servo
    ; === I2C SCREEN ===
        CLEAR_DAT1:        ;Data to send to screen, octet 8 
                    DS 1
        CLEAR_DAT2:        ;Data to send to screen, octet 8 
                    DS 1
        CLEAR_DAT3:       ;Data to send to screen, octet 8 
                    DS 1
        CLEAR_DAT4:       ;Data to send to screen, octet 8 
                    DS 1
        DAT12:            ;Data to send to screen, octet 8 
                DS 1
        DAT11:           ;Data to send to screen, octet 7
                DS 1
        DAT10:           ;Data to send to screen, octet 6 
                DS 1
        DAT9:            ;Data to send to screen, octet 5
                DS 1
        DAT8:            ;Data to send to screen, octet 8 
                DS 1
        DAT7:            ;Data to send to screen, octet 7
                DS 1
        DAT6:            ;Data to send to screen, octet 6 
                DS 1
        DAT5:            ;Data to send to screen, octet 5
                DS 1
        DAT4:            ;Data to send to screen, octet 4
                DS 1
        DAT3:            ;Data to send to screen, octet 3 
                DS 1
        DAT2:            ;Data to send to screen, octet 2
                DS 1
        DAT1:            ;Data to send to screen, octet 1
                DS 1
        ADD_SCREEN:         ;Data to send to TH sensor, octet 0 (LSB)
                DS 1
        COUNT_DATA_LCD:     ;counter for the data to send (32 for TH sensor)
                DB 9
        STATUS_I2C_LCD:     ;state of the I2C transmission
                DB 3
        TIM_STATUS_LCD:     ;0 = change clock, 1 = change data
                DB 1      
        CUR_TH_LCD:         ;charge l'octet qu'on veut envoyer
                DS 1
        COUNT_TH_LCD:       ;compteur pour savoir dans quel octet de TH on est
                DS 1  
        FLAG_ACK_LCD:       ;Flag to say if we have already send the ack 
                DB 1
        FLAG_PREPA_ACK_LCD: ;Flag to say if we already did the prepa to read the ack 
                DB 1
        DEBUGG_LCD:     DS 1 
        FLAG_INIT_DONE:  DB 1 ;flag for the init of the screen if 1=done 0=not done
        FLAG_I2C_SCREEN: DB 1 ;1= need to send something 0=sleep
        SCREEN_MODE:    DB 1 ;0= CLEAR 1= cursor + data
        FLAG_FIRST_BYTE: DB 1 
            

PSECT code
INITIALIZATION:
; ========= GENERAL SETTINGS ==========================
    ; ==== GLOBAL OSC FREQUENCY (p.36) ====
        BSF OSCCON,6,0
        BSF OSCCON,5,0      ; 4Mhz
        BCF OSCCON,4,0
    
    ; ====  TMR0 CONFIG (p.127) ====
        MOVLW 0b10000000
        MOVWF T0CON,0
        CLRF TMR0L,0        ; Sets timer period
        CLRF TMR0H,0

        BCF INTCON,2,0      ; Flag for interrupt on TIMER0
        BSF INTCON2,2,0     ; High priority IR for TMR0
        BSF INTCON,5,0      ; Enable TMR0IE

    ; ==== BUTTON CONFIG (p.117) ====
        CLRF LATB,0
        BSF TRISB,0,0       ; Button 0
        BSF TRISB,2,0       ; Button 1
        BSF TRISB,3,0       ; Button 2
        BSF TRISB,4,0       ; Button 3
        BSF TRISB,5,0       ; Button 3
        BCF INTCON2,7,0     ; Activate pull-up resistor on RB pins
        
        BCF Btn_Mode,0,0    ; Buttons are unpressed at boot time
        BCF Btn_Close,0,0
        BCF Btn_Open,0,0
        BCF Btn_Down,0,0
        BCF Btn_Up,0,0

        BSF CUR_MODE,0,0
    
    ; === OUTPUTS CONFIG ===
        BCF PORTC,4,0   ; DC direction selection
        BCF TRISC,4,0
        BCF LATC,4,0    ; starts closing direction
        
        BCF PORTC,5,0   ; DC ON/OFF selection
        BCF TRISC,5,0
        BCF LATC,5,0    ; Starts OFF
        
        ; temporary, degubb
        BCF PORTA,1,0   ; Broken debugg LED (useless but still...)
        BCF TRISA,1,0
        BCF LATA,1,0    ; starts off
        BCF ANSEL0,1,0
        
        BCF PORTD,7,0   ; Debugg on blue LED
        BCF TRISD,7,0
        BSF LATD,7,0    ; starts off

    ; ========= I2C: SDA AND SCL ==========================
        BCF PORTD,0,0       ; RD0 -> SCL1
        BCF PORTD,1,0       ; RD1 -> SDA1
        BCF PORTD,2,0       ; RD2 -> SCL2
        BCF PORTD,3,0       ; RD3 -> SDA2

        BCF TRISD,0,0
        BCF TRISD,1,0
        BCF TRISD,2,0
        BCF TRISD,3,0

        BCF LATD,0,0
        BSF LATD,1,0
        BCF LATD,2,0
        BSF LATD,3,0
    
    ; === I2C SCREEN SDA SCL ===
        BCF PORTD,4,0       ; RD2 -> SCL2
        BCF PORTD,5,0       ; RD3 -> SDA2

        BCF TRISD,4,0
        BCF TRISD,5,0

        BSF LATD,4,0
        BSF LATD,5,0

    ; ==== I2C: INIT VARIABLES ====
        MOVLW 0x00
        MOVWF TH3,0
        MOVLW 0xCC
        MOVWF TH2,0
        MOVLW 0x35
        MOVWF TH1,0
        MOVLW 0x0E
        MOVWF ADD_TH,0 
        CLRF STATUS_I2C,0
        BCF TIM_STATUS,0,0
        CLRF COUNT_DATA,0
        CLRF COUNT_TH,0
        CLRF S_DATA_INT,0
        CLRF H_DATA1_INT,0
        CLRF H_DATA2_INT,0
        CLRF HT_DATA_INT,0
        CLRF T_DATA1_INT,0
        CLRF T_DATA2_INT,0
        CLRF CRC_DATA_INT,0
        CLRF S_DATA_EXT,0
        CLRF H_DATA1_EXT,0
        CLRF H_DATA2_EXT,0
        CLRF HT_DATA_EXT,0
        CLRF T_DATA1_EXT,0
        CLRF T_DATA2_EXT,0
        CLRF CRC_DATA_EXT,0
        MOVFF ADD_TH, CUR_TH
        BCF FLAG_ACK,0,0
        BCF FLAG_PREPA_ACK,0,0

        BCF D0_I2C,0,0
        BCF LAST_CLOCK,0,0
        BSF TRANS_TYPE,0,0 
        BCF ASK_DATA,0,0        ; First one is "ask", but will be toggles so set to "read"
        BSF READ_DATA_FLAG,0,0

    ; === I2C SCREEN INIT VARIABLES ===
        MOVLW 0x40
        MOVWF ADD_SCREEN,0
        MOVLW 0xCC
        MOVWF DAT1,0
        MOVLW 0xC8
        MOVWF DAT2,0
        MOVLW 0x9C
        MOVWF DAT3,0
        MOVLW 0x98
        MOVWF DAT4,0
        MOVLW 0x3D
        MOVWF DAT5,0
        MOVLW 0x39
        MOVWF DAT6,0

        MOVLW 0x1D
        MOVWF DAT7,0
        MOVLW 0x19
        MOVWF DAT8,0

        MOVLW 0x3D
        MOVWF DAT9,0
        MOVLW 0x39
        MOVWF DAT10,0

        MOVLW 0x9D
        MOVWF DAT11,0
        MOVLW 0x99
        MOVWF DAT12,0
	
	    MOVLW 0x0C
        MOVWF CLEAR_DAT1,0
        MOVLW 0x08
        MOVWF CLEAR_DAT2,0
        MOVLW 0x1C
        MOVWF CLEAR_DAT3,0
        MOVLW 0x18
        MOVWF CLEAR_DAT4,0

        CLRF STATUS_I2C_LCD,0      ;initialize all variables

        BCF TIM_STATUS_LCD,0,0
        CLRF COUNT_DATA_LCD,0
        CLRF COUNT_TH_LCD,0

        MOVFF ADD_SCREEN, CUR_TH_LCD
        
        BCF FLAG_ACK_LCD,0,0
        BCF FLAG_PREPA_ACK_LCD,0,0
        BSF FLAG_I2C_SCREEN,0,0
        BCF FLAG_INIT_DONE,0,0
	BSF FLAG_FIRST_BYTE,0,0
        BCF SCREEN_MODE,0,0

        MOVLW 0b00011001
        MOVWF TEMP_WANTED,0  ;initialization to 19°C
    
; ==== ADC INIT (p.248) ====
    ; ADC documentation section 21.0 (p.239)
    ; A/D channels are grouped in four sets of 2 or 3 channels
    ; Selected channel in each group using ADCHS
    ; ADCON1 controls the voltage reference settings
    ; Conversion of pin to analog input using ANSEL0 and ANSEL1
    ; ADRESH/L contains the high/low bits of conversion

    ; Pins configuered as input of A/D are AN8 & RE2 (pin 2 & 10)
    BSF TRISA,0,0
    BSF TRISE,2,0

    BSF ANSEL0,0,0
    BSF ANSEL1,0,0 

    ; Voltage reference (ADCON1<7:6>) 
    ; So result 1111111111 = 5V
    BCF ADCON1,7,0
    BCF ADCON1,6,0

    ; Single channel mode on group A for AN8 (Register 21-1)
    ; ACSCH (ADCON0<4>) to 0
    ; ACMOD<1:0> to 11
    BCF ADCON0,4,0
    BCF ADCON0,3,0
    BCF ADCON0,2,0

    ; Select AN8 (GASEL<1:0> to 10) 
    BSF ADCHS,1,0
    BCF ADCHS,0,0

    ; Select AN0 (GASEL<1:0> to 00)
    ; BCF ADCHS,1,0 		; Toggle to get AN8
    ; BCF ADCHS,0,0			; Is done nautomatically on interrupt


    ; Select the A/D Auto-Conversion mode
    BCF ADCON0,5,0 ;single shot (when asked politely)

    ; A/D conversion clock (ADCON2<2:0>) (here FOSC/2)
    BCF ADCON2,2,0
    BCF ADCON2,1,0
    BCF ADCON2,0,0

    ; Select the A/D conversion trigger to No trigger
    CLRF ADCON3,0

    ;Turn on ADC
    BSF ADCON0,0,0

; ========= PARTICULAR TO DC PWM ==========================
    ; ==== Configure port as output p.117 ====
        BCF TRISB,1,0    ; PWM1, RB1, pin 34
    
    ; ==== PWM configuration p.203 ====
        MOVLW 0b00000000  
        MOVWF PTCON0,0      ; No postscaler, Prescaler 1:1, Free Running
        
        BSF PTCON1,7,0      ; Timer Base ON
        BCF PTCON1,6,0      ; Counting Up (useless)
        
        MOVLW 0b00011111
        MOVWF PWMCON0,0     ; PWM Pin 1 ON, Independants pins
        
        MOVLW 0b00000000
        MOVWF PWMCON1,0     ; No postscaler, ?, Updates of Duty Cycle ON, ?
    
        MOVLW 0b00000000    ; Sets the restart value to set frequency
        MOVWF PTPERH,0
        MOVLW 0b00100000
        MOVWF PTPERL,0

        MOVLW 0b00111111
        MOVWF PDC0H,0        
        MOVLW 0x11111111
        MOVWF PDC0L,0

; ========= PARTICULAR TO QEI ==========================
    ; ==== SET THE PINS CORRECTLY ====
        ; Disables ansel in order not to have analogs on pins QEA and QEB
        BCF ANSEL0,3,0
        BCF ANSEL0,4,0
        
        ; Set the RA registers to inputs on pins QEA and QEB
        BSF TRISA,3,0
        BSF TRISA,4,0
    
    ; ==== SETS TIMER 5 ====
        ; PR5H, PR5L sets the max values of Timer 5 at wich it restarts
        SETF PR5H,0
        SETF PR5L,0     ; Here set to max
        
        ; T5CON: TIMER5 CONTROL REGISTER p.139
        MOVLW 0b00010001
        MOVWF T5CON,0
        
        ; Initializes the timer at 0x00
        CLRF TMR5L,0
        CLRF TMR5H,0
    
    ; ==== SETS THE QEI IN THE RIGHT SETTINGS ====
        ; Disable the Input capture in order to use the QEI
        MOVLW 0b00000000 ; IC module off
        MOVWF CAP1CON,0
        MOVWF CAP2CON,0
        MOVWF CAP3CON,0
        
        ; QEICON: QUADRATURE ENCODER INTERFACE CONTROL REGISTER p.162
        ; Position, 2x Update Mode, No Prescaler
        MOVLW 0b10001000
        MOVWF QEICON,0
        
        ; CAP2BUFH and CAP2BUFL contain the position
        ; We initialize the blind at the end of the wood piece at the upper side of the blind
        CLRF CAP2BUFH,0
        MOVLW 0b01001000
        MOVWF CAP2BUFL,0
        MOVLW 0b00100100
        MOVWF BLIND_TARGET,0
        

        ; CAP3BUFH and CAP3BUFL contain the max position
        ; The max is the upper position that should not be access
        MOVLW 0x02 
        MOVWF CAP3BUFH,0
        MOVLW 0x00
        MOVWF CAP3BUFL,0
        
        ; DFLTCON: DIGITAL FILTER CONTROL REGISTER p.169
        ; Filter on QEA and QEB
        MOVLW 0b00110000
        MOVWF DFLTCON,0
    
    ; ==== SETS INTERRUPTS ====
        ; PIE3: PERIPHERAL INTERRUPT ENABLE REGISTER 3, p.107
        ;BSF PIE3,0,0        ; enables interrupt on timer 5, bot needed
        BSF PIE3,2,0        ; enables QEI inteerupts (overflow ?)
        BSF PIE3,3,0        ; enables change of direction interrupt
        CLRF PIR3,0         ; initializes interrupts

; ========= PARTICULAR TO SERVO PWM ==========================
    ; ==== Timer 2 settings p.136 ====
        MOVLW 0x07
        MOVWF T2CON,0   ; Set prescaler to 16 and enable timer 2
    ; ==== Configure port as output p.121 ====
        BCF TRISC,2,0   ; CCP1, RC2, pin 17
    
    ; ==== Configure PWM p.149 ====
        MOVLW 0xD0
        MOVWF PR2,0         ; Control the PWM frequency

        MOVLW 0b00111100    ; CCP1CON<5:4> set 2 LSBs of the duty cycle
        MOVWF CCP1CON,0     ; CCP1 in PWM mode

        MOVLW 0b00100010    ; Initially closed
        MOVWF CCPR1L,0      ; 8 MSBS of the duty cycle

        MOVLW 0b00100010
        MOVWF SERVO_TARGET,0

; ==== GLOBAL INTERRUPTS ====
    BSF INTCON,7,0
; === PERIPHERAL INTERRUPTS ===
    BSF INTCON,6,0
loop:
    BTFSC Flag_fast,0,0               ; if Flag_fast == True
    call FLAGFAST

    BTFSC Flag_slow,0,0               ; if Flag_fast == True
    call FLAGSLOW
    goto loop

FLAGFAST:
    BCF Flag_fast,0,0

    ; === THINGS TO BE DONE AT 50Hz ===
        ;BTG LATD,7,0                ; Debugg on test point

        call SECURITY
        call PID
        call BUTTONS
    return

FLAGSLOW:
    BCF Flag_slow,0,0

    ; === THINGS TO BE DONE AT 1Hz ===
        BTG LATA,1,0                ; debugg on LED

        call SENSORS
        BTFSS CUR_MODE,0,0
        call DECISION
        call SERVO
    return

SECURITY:
    call ADC_MANAGEMENT
    
    ; Perform logic to decide if we exceed threshold
    ;BCF LATD,7,0
    MOVFF ADC_AN0_SHUNT, WREG
    MOVLW 0b01111100
    CPFSLT ADC_AN0_SHUNT,0
    NOP
    ;BSF LATD,7,0
    return

PID:
    BCF INTCON,7,0
    ; === GET POSITION ON 8 BITS  ===
        CLRF BLIND_POSITION,0

        BTFSC CAP2BUFH,0,0
        BSF BLIND_POSITION,7,0      ; if CAP2BUFH LSB = 1

        MOVFF CAP2BUFL, WREG        ; WREG = CAP2BUFL
        RRNCF WREG,0,0              ; WREG = CAP2BUFL rotated right
        BCF WREG,7,0                ; WREG = CAP2BUFL rotated right and MSB = 0
        
        IORWF BLIND_POSITION,1,0    ; Add correct MSB to WREG and place it into Pos_Blind
	MOVFF BLIND_POSITION, WREG ; debug

    ; === PERFORMS PID ===
    
	MOVFF BLIND_TARGET, WREG ; debug
    
    BSF PID_REG,1,0
    BCF PID_REG,0,0
	
	MOVFF BLIND_POSITION,WREG
	CPFSGT BLIND_TARGET,0
	BCF PID_REG,1,0
	
    MOVFF BLIND_POSITION, WREG
    SUBWF BLIND_TARGET,0,0 

    BTFSC STATUS,2,0
    BSF PID_REG,0,0

    BTFSS PID_REG,1,0
    call complement

    BTFSS PID_REG,0,0
    goto update_blind
    BCF LATC,5,0
    BSF INTCON,7,0 
  
    return

complement:
    COMF WREG,0,0
    INCF WREG,0,0
    return
    
update_blind:
    MOVWF ERROR_POSITION,0
    MOVLW kp
    MULWF ERROR_POSITION,0
    MOVFF PRODL,PID_OUT0
    MOVFF PRODH,PID_OUT1
    

    BTFSC PID_REG,1,0
    BCF LATC,4,0
    BTFSS PID_REG,1,0
    BSF LATC,4,0
    BSF LATC,5,0
    
    CLRF PDC0H,0
 
    MOVLW   0xF0 ;4 MSB bits from PID_OUT0
    ANDWF   PID_OUT0,0,0
    SWAPF   WREG,0,0
    MOVWF   TMP,0  
    
    MOVLW   0x0F ;3 LSB bits from PID_OUT1
    ANDWF   PID_OUT1,0,0
    SWAPF   WREG,0,0
    IORWF   TMP,0,0
    COMF    WREG,0,0
    BCF	    WREG,7,0
    MOVFF   WREG, PDC0L
    
    MOVLW 0x08
    CPFSLT PID_OUT1,0
	CLRF PDC0L,0
    BSF INTCON,7,0    
    RETURN

BUTTONS:
    ; This is used to detect single clicks on buttons
    ; Not long button push

    BTFSS PORTB,0,0     ; if Btn_Up is pressed
    call BTN_UP_PUSHED
    
    BTFSS PORTB,2,0     ; if Btn_Down is pressed
    call BTN_DOWN_PUSHED

    BTFSS PORTB,3,0     ; if Btn_Open is pressed
    call BTN_OPEN_PUSHED

    BTFSS PORTB,4,0     ; if Btn_Close is pressed
    call BTN_CLOSE_PUSHED

    BTFSS PORTB,5,0     ; if Btn_Mode is pressed
    call BTN_MODE_PUSHED

    ; Check if the buttons are released
    BTFSC PORTB,0,0     ; if Btn_Up is not pressed, release the lock
    BCF Btn_Up,0,0
    
    BTFSC PORTB,2,0     ; if Btn_Down is not pressed, release the lock
    BCF Btn_Down,0,0

    BTFSC PORTB,3,0     ; if Btn_Open is not pressed, release the lock
    BCF Btn_Open,0,0

    BTFSC PORTB,4,0 ;   if Btn_Close is not pressed, release the lock
    BCF Btn_Close,0,0

    BTFSC PORTB,5,0 ;   if Btn_Mode is not pressed, release the lock
    BCF Btn_Mode,0,0
    return
    
BTN_UP_PUSHED:
    BTFSC Btn_Up,0,0
    return
    BSF Btn_Up,0,0
    BTFSS CUR_MODE,0,0
    goto inc_temp       ; Automatic mode
    goto up_blind      ; Manual mode

inc_temp:
    MOVLW 0b00011110
    CPFSEQ TEMP_WANTED,0
    INCF TEMP_WANTED,1,0
    call convert_temp_to_screen
    return

up_blind:
      MOVLW 0b00100100
      MOVWF BLIND_TARGET,0
      return

BTN_DOWN_PUSHED:
    BTFSC Btn_Down,0,0
    return
    BSF Btn_Down,0,0
    BTFSS CUR_MODE,0,0
    goto dec_temp
    goto down_blind

dec_temp:
    MOVLW 0b00011000
    CPFSEQ TEMP_WANTED,0
    DECF TEMP_WANTED,1,0
    call convert_temp_to_screen
    return

down_blind:
     MOVLW 0b11101001
     MOVWF BLIND_TARGET,0
     return

BTN_OPEN_PUSHED:
    BTFSC Btn_Open,0,0
    return
    BSF Btn_Open,0,0
    
    MOVFF SERVO_TARGET, WREG

    BTFSS CUR_MODE,0,0
    RETURN

    MOVLW 0b00100010
    CPFSGT SERVO_TARGET,0
    goto W1quarter

    MOVLW 0b00101011
    CPFSGT SERVO_TARGET,0
    goto W1half

    MOVLW 0b00110100
    CPFSGT SERVO_TARGET,0
    goto W3quarter

    MOVLW 0b00111110
    CPFSGT SERVO_TARGET,0
    goto Wopen
    return

BTN_CLOSE_PUSHED:
    BTFSC Btn_Close,0,0
    return
    BSF Btn_Close,0,0
    BTFSS CUR_MODE,0,0
    RETURN

    MOVLW 0b00101011
    CPFSGT SERVO_TARGET,0
    goto Wclosed

    MOVLW 0b00110100
    CPFSGT SERVO_TARGET,0
    goto W1quarter

    MOVLW 0b00111110
    CPFSGT SERVO_TARGET,0
    goto W1half

    MOVLW 0b01000011
    CPFSGT SERVO_TARGET,0
    goto W3quarter
    return

Wclosed:
    MOVLW 0b00100010
    MOVWF SERVO_TARGET,0 
    return

W1quarter:
    MOVLW 0b00101011
    MOVWF SERVO_TARGET,0 
    MOVFF SERVO_TARGET, WREG
    return

W1half:
    MOVLW 0b00110100
    MOVWF SERVO_TARGET,0 
    return

W3quarter:
    MOVLW 0b00111110
    MOVWF SERVO_TARGET,0 
    return

Wopen:
    MOVLW 0b01000001
    MOVWF SERVO_TARGET,0 
    return

BTN_MODE_PUSHED:
    BTFSC Btn_Mode,0,0
    return
    BSF Btn_Mode,0,0
    BTG CUR_MODE,0,0
    BTG LATD,7,0
    return

SENSORS:
    call TH_I2C
    call LUMINOSITY
    return



DECISION:
    call BLIND_DECISION
    call WINDOW_DECISION
    return

SERVO:
    MOVFF SERVO_TARGET, CCPR1L
    return

TH_I2C:
    BTFSC D0_I2C,0,0
    return                  ; The previous transaction is not finished
    
    BTG ASK_DATA,0,0        ; Switch the transaction time  
    BTG READ_DATA_FLAG,0,0
    BCF LAST_CLOCK,0,0
    
    
    BSF D0_I2C,0,0          ; We ask for the transaction
    return

LUMINOSITY:
    BCF SUNNY_FLAG,0,0
    MOVLW 0b00111111
    CPFSGT ADC_AN8_LUM,0
    BSF SUNNY_FLAG,0,0
    return

BLIND_DECISION:
    BTFSS SUNNY_FLAG,0,0
    goto up_blind
    MOVFF TEMP_INT, WREG
    CPFSGT TEMP_WANTED,0
    goto down_blind     
    goto up_blind         
    return
     
    

WINDOW_DECISION:
    MOVLW 0b00000011
    CPFSLT HUM_INT,0
    goto high_humidity
    MOVLW 0b00000000
    CPFSGT HUM_INT,0
    goto low_humidity
    goto temp_compare

high_humidity:
    MOVLW 0b00000001
    CPFSGT HUM_EXT,0
    goto W1half
    goto temp_compare

low_humidity:
    MOVLW 0b000000001
    CPFSLT HUM_EXT,0
    goto W1half
    goto temp_compare
    return

temp_compare:
    MOVFF TEMP_WANTED, WREG
    CPFSGT TEMP_INT,0
    goto too_cold
    goto too_hot

too_cold: 
    MOVFF TEMP_INT, WREG
    CPFSGT TEMP_EXT,0
    goto Wclosed
    goto W1half

too_hot:
    MOVFF TEMP_INT, WREG    ; trop chaud, on veut que ça s'ouvre si il fait plus froid dehors 
    CPFSLT TEMP_EXT,0
    goto Wclosed
    goto W1half

ADC_MANAGEMENT:
    BTFSC ADCON0,1,0	    ; If conversion finished
        return

    ; Get result :
    BTFSS ADCHS,1,0
    MOVFF ADRESH, ADC_AN0_SHUNT     ; ADCHS,1,0 = 0 -> ADC_AN0
    
    BTFSC ADCHS,1,0
    MOVFF ADRESH, ADC_AN8_LUM       ; ADCHS,1,0 = 1 -> ADC_AN8
    
    ; Debug feature, breakpoint here to see ADC_AN0 / ADC_AN8
    MOVFF ADC_AN0_SHUNT, WREG
    MOVFF ADC_AN8_LUM, WREG
    
    ; ==== Start AD conversion p.240 ====    
        BTG ADCHS,1,0	    ; Change A/D Pin 0 <-> 8
        BSF ADCON0,1,0	    ; Start conversion
        BCF PIR1,6,0	    ; Clear the finished interrupt flag
    return

convert_temp_to_screen:
	MOVFF TEMP_WANTED, WREG
        MOVLW 0b00011000
        SUBWF TEMP_WANTED,0,0
        RLNCF WREG,1,0
	RLNCF WREG,1,0
        call CONVERT_TEMP
        BSF FLAG_I2C_SCREEN,0,0 ;launch the screen I2C
        BCF SCREEN_MODE,0,0    ;in clear mode
        RETURN

load_18:
        MOVLW 0x1D
        MOVWF DAT7,0
        MOVLW 0x19
        MOVWF DAT8,0

        MOVLW 0x8D
        MOVWF DAT11,0
        MOVLW 0x89
        MOVWF DAT12,0
        RETURN

load_19:
        MOVLW 0x1D
        MOVWF DAT7,0
        MOVLW 0x19
        MOVWF DAT8,0

        MOVLW 0x9D
        MOVWF DAT11,0
        MOVLW 0x99
        MOVWF DAT12,0
        RETURN
load_20:
        MOVLW 0x2D
        MOVWF DAT7,0
        MOVLW 0x29
        MOVWF DAT8,0

        MOVLW 0x0D
        MOVWF DAT11,0
        MOVLW 0x09
        MOVWF DAT12,0
        RETURN

load_21:
        MOVLW 0x2D
        MOVWF DAT7,0
        MOVLW 0x29
        MOVWF DAT8,0

        MOVLW 0x1D
        MOVWF DAT11,0
        MOVLW 0x19
        MOVWF DAT12,0
        RETURN
load_22:
        MOVLW 0x2D
        MOVWF DAT7,0
        MOVLW 0x29
        MOVWF DAT8,0

        MOVLW 0x2D
        MOVWF DAT11,0
        MOVLW 0x29
        MOVWF DAT12,0
        RETURN
load_23:
        MOVLW 0x2D
        MOVWF DAT7,0
        MOVLW 0x29
        MOVWF DAT8,0

        MOVLW 0x3D
        MOVWF DAT11,0
        MOVLW 0x39
        MOVWF DAT12,0
        RETURN

load_24:
        MOVLW 0x2D
        MOVWF DAT7,0
        MOVLW 0x29
        MOVWF DAT8,0

        MOVLW 0x4D
        MOVWF DAT11,0
        MOVLW 0x49
        MOVWF DAT12,0
        RETURN
end reset_vec

