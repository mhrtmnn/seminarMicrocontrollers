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

#Mem Adresses
__SP_H__ = 0x3e
__SP_L__ = 0x3d


/************************* interrupt vector table *************************/
.org 0x00
	rjmp main 			/* reset */
	reti
	rjmp vector_0 		/* INT0 */
	reti
	rjmp vector_1		/* INT1 */
	reti


/********************************** setup **********************************/
.org 0x30
main:
	/* setup stack pointer */
	in r28,__SP_L__
	in r29,__SP_H__
	out __SP_H__,r29
	out __SP_L__,r28


	/* LEDs: set PB0 - PB2 as output and set them high */
	ldi r24,0x07 	/* PB0 - PB2 */
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

	sei 				/*enable interrupts */


/******************************** main loop  ********************************/
main_loop:
	/* load counter */
	ldi r18,lo8(1999999)
	ldi r19,hi8(1999999)
	ldi r20,hlo8(1999999)

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
	push r23
	push r24
	/* stack size = 6 */

	/* toggle led */
	ldi r24,BIT_PB1
	push r24
	rcall toggler

	/* pop all the registers in reverse order */
	pop r24
	pop r23
	pop r20
	pop r19
	pop r18
	pop r0
	/* stack size = 0 */

	out __SREG__,r0 /* restore sreg */
	reti


/******************************** ISR for INT1 ********************************/
vector_1:
	in r0,__SREG__ 		/* save status reg to r0 */
	push r0
	push r18
	push r19
	push r20
	push r23
	push r24
	/* stack size = 6 */

	ldi r24,BIT_PB2
	push r24
	rcall toggler

	/* pop all the registers in reverse order */
	pop r24
	pop r23
	pop r20
	pop r19
	pop r18
	pop r0
	/* stack size = 0 */

	out __SREG__,r0 /* restore sreg */
	reti

toggler:
	pop r18
	pop r19
	pop r27

		/* toggle led */
	in r24,PORTB
	eor r24,r27 		/* XOR */
	out PORTB,r24

	/* debounce delay */
	ldi r18,lo8(189999)
	ldi r24,hi8(189999)
	ldi r25,hlo8(189999)
wait1:
	subi r18,1
	sbci r24,0		/* Subtract with Carry */
	sbci r25,0		/* Subtract with Carry */
	brne wait1

	push r19
	push r18
	ret
