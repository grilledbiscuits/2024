/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns

main_loop:
	LDR R0, GPIOA_BASE		@ You gotta keep reloading that register bud
	LDR R4, [R0, #0x10]		@IDR OFFSET VALUE

	@DEFAULT STATE			(LED increments by 1 every 0.7sec)

	MOVS R6, #0b1111
	ANDS R4, R4, R6
	CMP R4, #0b1111
	BEQ incr_normal

	@BUTTON 0 CHECK			(LED increments by 2)
	MOVS R6, #0b1111
	ANDS R4, R4, R6
	CMP R4, #0b1110
	BEQ incr_twiceandslow

	@BUTTON 1 CHECK			(LED increments by 1 every 0.3sec)
	MOVS R6, #0b1111
	ANDS R4, R4, R6
	CMP R4, #0b1101
	BEQ incr_fast

	@BUTTON 2 CHECK			(LEDs display 0xAA until released)
	CMP R4, #0b1011
	BEQ AlcoholicsAnonymous

	@BUTTON 1 AND 2 CHECK	(Increment by 2 every 0.3sec)
	CMP R4, #0b1100
	BEQ incr_twiceandfast

	@BUTTON 3 CHECK	(Freeze)
	CMP R4, #0b0111
	BEQ freeze

incr_normal:
	CMP R2, #0b00000000		@(Check if there are any on LEDs to shift)
	BEQ reset				@(Send to restart (R2=1) block
	LSLS R2, R2, #1			@Shift left by 1
	BL longdelay
	B maskled

incr_fast:
	CMP R2, #0b00000000		@(Check if there are any on LEDs to shift)
	BEQ reset				@(Send to restart (R2=1) block
	LSLS R2, R2, #1			@Shift left by 1
	BL shortdelay
	B maskled

incr_twiceandslow:
	CMP R2, #0b00000000		@(Check if there are any on LEDs to shift)
	BEQ reset				@(Send to restart (R2=1) block
	LSLS R2, R2, #2			@Shift left by 1
	BL longdelay
	B maskled

incr_twiceandfast:
	CMP R2, #0b00000000		@(Check if there are any on LEDs to shift)
	BEQ reset				@(Send to restart (R2=1) block
	LSLS R2, R2, #2			@Shift left by 1
	BL shortdelay
	B maskled

AlcoholicsAnonymous:
    MOVS R3, #0xAA			@Write AA to a dummy register
    LDR R0, GPIOB_BASE		@Update that register baby
    STR R3, [R0, #0x14] 	@ Write 0xAA to the LED ODR
    B main_loop

freeze:@!
	B maskled				@Literally do nothing

reset:
	MOVS R2, #1
	B maskled

maskled:
	MOVS R3, #0b11111111
	ANDS R2, R2, R3
	B write_leds

longdelay:
	LDR R3, LONG_DELAY_CNT
	looplong:
		SUBS R3, R3, #1
		BNE looplong
		BX LR

shortdelay:
	LDR R3, SHORT_DELAY_CNT
	loopshort:
		SUBS R3, R3, #1
		BNE loopshort
		BX LR

write_leds:
 	LDR R0, GPIOB_BASE
	STR R2, [R1, #0x14]
	B main_loop


@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 1400000
SHORT_DELAY_CNT: 	.word 600000
