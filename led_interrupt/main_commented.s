/*************************** numeric constants ***************************/
__SREG__ = 0x3f


/************************* interrupt vector table *************************/
.org 0x00
	rjmp main 			/* reset */
	reti
	rjmp vector_0 		/* INT0 */
	reti
	rjmp vector_1 		/* INT1 */
	reti


/********************************** main **********************************/
.org 0x30
main:
/* stack size = 0 */

	/* set PB0 - PB2 as output and set them high */
	ldi r24,0x07 	/* r24 = 0x07 */
	out 0x17,r24 	/* DDRB  = r24 */
	out 0x18,r24 	/* PORTB = r24 */

	/* setup interrupts */
	in r24,0x3b 	/* r24 = GICR  */
	ori r24,0x40 	/* enable INT0 = BIT(6) = 0x40 */
	ori r24,0x80 	/* enable INT1 = BIT(8) = 0x80*/
	out 0x3b,r24 	/* GICR = r24 */

	/* The CBI and SBI instructions work with registers $00 to $1F only */
	sbi 0x12,2 		/* enable PD2 in PORTD */
	sbi 0x12,3 		/* enable PD3 in PORTD */

	sei 			/*enable interrupts */


/******************************** main loop  ********************************/
main_loop:
	/* load counter */
	ldi r18,lo8(1999999)
	ldi r19,hi8(1999999)
	ldi r20,hlo8(1999999)

main_wait:
	subi r18,1
	sbci r19,0
	sbci r20,0
	brne main_wait

	/* toggle led */
	in r24,0x18 	/* r24 = PORTB */
	ldi r23,0x01 	/* r23 = 0x01 = PB0 */
	eor r24,r23 	/* r24 = r24^r23 */
	out 0x18,r24 	/* PORTB = r24 */
	rjmp main_loop


/******************************** ISR for INT0 ********************************/
vector_0:
	in r0,__SREG__ /* save status reg to r0 */
	push r0
	push r18
	push r19
	push r20
	push r23
	push r24
/* stack size = 6 */

	in r24,0x18 	/* r24 = PORTB */
	ldi r25,0x02 	/* r25 = 0x02 = PB1 */
	eor r24,r25 	/* r24 = r24^r25 */
	out 0x18,r24 	/* PORTB = r24 */

	/* 200 ms delay */
	ldi r18,lo8(189999)
	ldi r24,hi8(189999)
	ldi r25,hlo8(189999)
wait0:
	subi r18,1
	sbci r24,0
	sbci r25,0
	brne wait0

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
	in r0,__SREG__ /* save status reg to r0 */
	push r0
	push r18
	push r19
	push r20
	push r23
	push r24
/* stack size = 6 [6 push commands] */

	in r24,0x18 	/* r24 = PORTB */
	ldi r25,0x04 	/* r25 = 0x04 = PB2 */
	eor r24,r25 	/* r24 = r24^r25 */
	out 0x18,r24 	/* PORTB = r24 */

	/* 200 ms delay */
	ldi r18,lo8(189999)
	ldi r24,hi8(189999)
	ldi r25,hlo8(189999)
wait1:
	subi r18,1
	sbci r24,0
	sbci r25,0
	brne wait1

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
