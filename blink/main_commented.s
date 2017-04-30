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

init:
	ldi r24,lo8(-1)
	out 0x17,r24	/* 	DDRB = 0xFF; */
	ldi r24,lo8(1)
turn_on_led:
	out 0x18,r24
	ldi r18,lo8(799999)
	ldi r19,hi8(799999)
	ldi r25,hlo8(799999)
sub_off:
	subi r18,1
	sbci r19,0
	sbci r25,0
	brne sub_off
turn_off_led:
	out 0x18,__zero_reg__
	ldi r18,lo8(799999)
	ldi r19,hi8(799999)
	ldi r25,hlo8(799999)
sub_on:
	subi r18,1
	sbci r19,0
	sbci r25,0
	brne sub_on
	rjmp turn_on_led
	.size	main, .-main
	.ident	"GCC: (Fedora 6.2.0-1.fc25) 6.2.0"
