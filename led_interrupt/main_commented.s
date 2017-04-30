	.file	"main.c"
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__SREG__ = 0x3f
__tmp_reg__ = 0
__zero_reg__ = 1
	.section	.text.startup,"ax",@progbits
.global	main
	.type	main, @function
main:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r24,lo8(1)
	out 0x17,r24 	/* DDRB  = 0x01 */
	out 0x18,r24 	/* PORTB = 0x01 */

	/* The CBI and SBI instructions work with registers $00 to $1F only */
	in r24,0x3b 	/* r24 = GICR  */
	ori r24,lo8(64) /* enable INT0 = BIT(6) = 64 */
	out 0x3b,r24 	/* GICR = r24 */

	sbi 0x12,2 		/* enable PD2 in PORTD */

	sei 			/*enable interrupts */

.L2: 				/* endless loop */
	rjmp .L2

	.size	main, .-main
	.text
.global	__vector_1
	.type	__vector_1, @function




__vector_1:
	push r1
	push r0
	in r0,__SREG__ /* save status reg to r0 */
	push r0
	clr __zero_reg__ /* Clear Register 1 */
	push r18
	push r24
	push r25

/* prologue: Signal */
/* frame size = 0 */
/* stack size = 6 [6 push commands] */
.L__stack_usage = 6
	in r24,0x18 	/* r24 = PORTB */
	ldi r25,lo8(1) 	/* r25 = 0x01 = PB0 */
	eor r24,r25 	/* r24 = r24^r25 */
	out 0x18,r24 	/* PORTB = r24 */

	/* 200 ms delay */
	ldi r18,lo8(319999)
	ldi r24,hi8(319999)
	ldi r25,hlo8(319999)
1:	subi r18,1
	sbci r24,0
	sbci r25,0
	brne 1b

/* epilogue start */
/* pop all the registers in reverse order */
	pop r25
	pop r24
	pop r18
	pop r0
	out __SREG__,r0 /* restore sreg */
	pop r0
	pop r1
	reti

	.size	__vector_1, .-__vector_1
	.ident	"GCC: (Fedora 6.2.0-1.fc25) 6.2.0"
