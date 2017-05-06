/*************************** numeric constants ***************************/
__SREG__ = 0x3f

#Registers
DDRA = 0x1A
DDRB = 0x17
DDRC = 0x14
DDRD = 0x11

PORTA = 0x1B
PORTB = 0x18
PORTC = 0x15
PORTD = 0x12

GICR = 0x3b


#Bits
PA0 = 0
PB0 = 0
PC0 = 0
PD0 = 0

PA1 = 1
PB1 = 1
PC1 = 1
PD1 = 1

PA2 = 2
PB2 = 2
PC2 = 2
PD2 = 2

PA3 = 3
PB3 = 3
PC3 = 3
PD3 = 3

PA4 = 4
PB4 = 4
PC4 = 4
PD4 = 4

PA5 = 5
PB5 = 5
PC5 = 5
PD5 = 5

PA6 = 6
PB6 = 6
PC6 = 6
PD6 = 6

PA7 = 7
PB7 = 7
PC7 = 7
PD7 = 7

BIT_PA0 = 0x01
BIT_PB0 = 0x01
BIT_PC0 = 0x01
BIT_PD0 = 0x01

BIT_PA1 = 0x02
BIT_PB1 = 0x02
BIT_PC1 = 0x02
BIT_PD1 = 0x02

BIT_PA2 = 0x04
BIT_PB2 = 0x04
BIT_PC2 = 0x04
BIT_PD2 = 0x04

BIT_PA3 = 0x08
BIT_PB3 = 0x08
BIT_PC3 = 0x08
BIT_PD3 = 0x08

BIT_PA4 = 0x10
BIT_PB4 = 0x10
BIT_PC4 = 0x10
BIT_PD4 = 0x10

BIT_PA5 = 0x20
BIT_PB5 = 0x20
BIT_PC5 = 0x20
BIT_PD5 = 0x20

BIT_PA6 = 0x40
BIT_PB6 = 0x40
BIT_PC6 = 0x40
BIT_PD6 = 0x40

BIT_PA7 = 0x80
BIT_PB7 = 0x80
BIT_PC7 = 0x80
BIT_PD7 = 0x80

BIT_INT0 = 0x40
BIT_INT1 = 0x80

#Stack pointer registers
SP_H = 0x3e
SP_L = 0x3d

#memory locations
RAMEND_H = 0x04
RAMEND_L = 0x5F

/**
 * Timings
 * @16MHz: 1 cycle = 6.25e-08 =: C
 * desired delay in s =: D
 * constant to load into reg =: T
 *
 * subi takes 1 cycle
 * brne takes 2 cycles if taken, 1 if not
 * for n subi instr: D = (n+2)*C*(T-1) + (n+1)*C
 *
 * Python function:
 * def f (n, D): return int(1 + (D - (n+1)*6.25e-08)/(6.25e-08*(n+2)))
 */

/************************* interrupt vector table *************************/
.org 0x00
	rjmp main 		/* reset */
	reti
	rjmp vector_0 		/* INT0 */
	reti
	rjmp vector_lcd		/* INT1 */
	reti


/********************************** setup **********************************/
.org 0x30
main:
	/* setup stack pointer */
	ldi r24,RAMEND_L
	out SP_L,r24
	ldi r24,RAMEND_H
	out SP_H,r24

	/* LCD: set data pins as outputn and low (def value) */
	ldi r24,0xFF 		/* PA0 - PA7 */
	out DDRA,r24

	/* LEDs: set PB0 - PB2 as output and set them high */
	ldi r24,0x07 		/* PB0 - PB2 */
	out DDRB,r24
	out PORTB,r24

	/* setup interrupts */
	in r24,GICR
	ori r24,BIT_INT0 	/* enable INT0 = BIT(6) */
	ori r24,BIT_INT1 	/* enable INT1 = BIT(8) */
	out GICR,r24

	/*
	 * The CBI and SBI instructions work with registers $00 to $1F only
	 * they clear/set the specified bit
	 */
	sbi PORTD,PD2 		/* enable PD2 in PORTD */
	sbi PORTD,PD3 		/* enable PD3 in PORTD */

	sei 			/*enable interrupts */


/******************************** main loop  ********************************/
main_loop:
	/* load counter */
	ldi r18,lo8(2000000)
	ldi r19,hi8(2000000)
	ldi r20,hlo8(2000000)

main_wait:
	subi r18,1
	sbci r19,0		/* Subtract with Carry */
	sbci r20,0		/* Subtract with Carry */
	brne main_wait

	/* toggle led */
	in r24,PORTB
	ldi r23,BIT_PB0
	eor r24,r23 	/* XOR */
	out PORTB,r24
	rjmp main_loop




/******************************** ISR for INT0 ********************************/
vector_0:
	in r0,__SREG__ 		/* save status reg to r0 */
	push r0
	push r18
	push r19
	push r20
	push r24

	/* toggle led */
	ldi r24,BIT_PB1
	push r24
	rcall toggler

	/* take variable from stack after function call */
	pop r24

	/* debounce delay */
	ldi r18,lo8(500000)
	ldi r19,hi8(500000)
	ldi r20,hlo8(500000)
v0_deb_del:
	subi r18,1
	sbci r19,0
	sbci r20,0
	brne v0_deb_del

	/* pop all the registers in reverse order */
	pop r24
	pop r20
	pop r19
	pop r18
	pop r0

	/* restore sreg */
	out __SREG__,r0
	reti




/******************************** ISR for INT1 ********************************/
vector_lcd:
	in r0,__SREG__ 		/* save status reg to r0 */
	push r0
	push r18
	push r19
	push r20
	push r23
	push r24

	/*
	 * data to be displayed 'X' == 0101 1000 == 0x05 0x08
	 * LOWER byte has to be pushed FIRST
	 */
	ldi r25,0x08
	push r25
	ldi r25,0x05
	push r25
	rcall print_char

	/* take variables from stack after function call */
	pop r25
	pop r25

	/*##################### toggle led for confirmation #####################*/
	ldi r25,BIT_PB2
	push r25
	rcall toggler

	/* take variable from stack after function call */
	pop r25

	/* debounce delay */
	ldi r18,lo8(500000)
	ldi r19,hi8(500000)
	ldi r20,hlo8(500000)
v1_deb_del:
	subi r18,1
	sbci r19,0
	sbci r20,0
	brne v1_deb_del

	/* pop all the registers in reverse order */
	pop r24
	pop r23
	pop r20
	pop r19
	pop r18
	pop r0

	out __SREG__,r0 	/* restore sreg */
	reti




/******************************** FUNCTIONS ********************************/
toggler:
	/* Calculate parameter position in STACK in SRAM
	 *
	 * SP points to first unused STACK addr,
	 * the last two stack elements are the return addr,
	 * so the parameter lies at MEM[SP+3]
	 *
	 * stack grows towards lower addr
	 * avr has no addi instr so use subi
	 *
	 * Addr Register: Z = [R31, R30]
	 */
	in r30,SP_L
	in r31,SP_H
	subi r30,lo8(-3)

	/* get parameter from stack */
	ld r27, Z

	/* toggle led */
	in r24,PORTB
	eor r24,r27 		/* XOR */
	out PORTB,r24

	ret

print_char:
	/* get function parameters from stack */
	in r30,SP_L
	in r31,SP_H

	/* load first parameter */
	subi r30,lo8(-3)
	ld r25, Z

	/* load second parameter */
	subi r30,lo8(-1)
	ld r26, Z

	/* 	PORTA |= (1<<PA4);    // RS auf 1 setzen (send data) */
	in r24,PORTA
	ori r24,BIT_PA4
	out PORTA,r24

	/*######################### upper 4bit #########################*/

	/*		PORTA &= ~(0xF0>>(4-PA0));     #Maske löschen (shift out the lower 4 bits, then shift left (neg right shift) to p0) */
	/*		PORTA |= (data>>(4-PA0));      #Bits setzen */
	in r24,PORTA
	andi r24,0xF0 		/* 1111 0000 -> clear lower 4 bits*/
	or r24,r25 		/* write payload from r25 */

	/*	PORTA |= (1<<PA5);     #Enable auf 1 setzen */
	ori r24, BIT_PA5 	/* EN pin high */
	out PORTA,r24

	/*		_delay_us(  [20 us == 32 cycles]);  #kurze Pause */
	ldi r18,lo8(32)
wait2:
	subi r18,1
	brne wait2

	/*		PORTA &= ~(1<<PA5);    #Enable auf 0 setzen */
	in r24,PORTA
	andi r24,0xDF 		/*1101 1111 -> EN pin low */
	out PORTA,r24

	/*		_delay_us(  [46 us == 74cycles]);  # kurze Pause */
	ldi r18,lo8(74)
wait3:
	subi r18,1
	brne wait3

	/*######################### lower 4bit #########################*/

	/*		PORTA &= ~(0xF0>>(4-PA0));     #Maske löschen (shift out the lower 4 bits, then shift left (neg right shift) to p0) */
	/*		PORTA |= (data>>(4-PA0));      #Bits setzen */
	in r24,PORTA
	andi r24,0xF0 		/* 1111 0000 -> clear lower 4 bits*/
	or r24,r26 		/* write payload from r26 */

	/*	PORTA |= (1<<PA5);     #Enable auf 1 setzen */
	ori r24, BIT_PA5 	/* EN pin high */
	out PORTA,r24

	/*		_delay_us(  [20 us == 32 cycles]);  #kurze Pause */
	ldi r18,lo8(32)
wait4:
	subi r18,1
	brne wait4

	/*		PORTA &= ~(1<<PA5);    #Enable auf 0 setzen */
	in r24,PORTA
	andi r24,0xDF 		/*1101 1111 -> EN pin low */
	out PORTA,r24

	/*		_delay_us(  [46 us == 74cycles]);  # kurze Pause */
	ldi r18,lo8(74)
wait5:
	subi r18,1
	brne wait5

	ret
