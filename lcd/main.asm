	.file	"main.c"
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__SREG__ = 0x3f
__tmp_reg__ = 0
__zero_reg__ = 1
	.text
	.type	lcd_enable, @function
lcd_enable:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	sbi 0x12,5
	ldi r24,lo8(53)
1:	dec r24
	brne 1b
	nop
	cbi 0x12,5
	ret
	.size	lcd_enable, .-lcd_enable
	.type	lcd_out, @function
lcd_out:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	in r25,0x12
	andi r25,lo8(-16)
	out 0x12,r25
	in r25,0x12
	swap r24
	andi r24,lo8(15)
	or r24,r25
	out 0x12,r24
	jmp lcd_enable
	.size	lcd_out, .-lcd_out
.global	lcd_data
	.type	lcd_data, @function
lcd_data:
	push r28
/* prologue: function */
/* frame size = 0 */
/* stack size = 1 */
.L__stack_usage = 1
	mov r28,r24
	sbi 0x12,4
	call lcd_out
	mov r24,r28
	swap r24
	andi r24,lo8(-16)
	call lcd_out
	ldi r24,lo8(122)
1:	dec r24
	brne 1b
	rjmp .
/* epilogue start */
	pop r28
	ret
	.size	lcd_data, .-lcd_data
.global	lcd_command
	.type	lcd_command, @function
lcd_command:
	push r28
/* prologue: function */
/* frame size = 0 */
/* stack size = 1 */
.L__stack_usage = 1
	mov r28,r24
	cbi 0x12,4
	call lcd_out
	mov r24,r28
	swap r24
	andi r24,lo8(-16)
	call lcd_out
	ldi r24,lo8(112)
1:	dec r24
	brne 1b
/* epilogue start */
	pop r28
	ret
	.size	lcd_command, .-lcd_command
.global	lcd_clear
	.type	lcd_clear, @function
lcd_clear:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r24,lo8(1)
	call lcd_command
	ldi r24,lo8(3999)
	ldi r25,hi8(3999)
1:	sbiw r24,1
	brne 1b
	rjmp .
	nop
	ret
	.size	lcd_clear, .-lcd_clear
.global	lcd_init
	.type	lcd_init, @function
lcd_init:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	in r24,0x11
	ori r24,lo8(63)
	out 0x11,r24
	in r24,0x12
	andi r24,lo8(-64)
	out 0x12,r24
	ldi r24,lo8(29999)
	ldi r25,hi8(29999)
1:	sbiw r24,1
	brne 1b
	rjmp .
	nop
	ldi r24,lo8(48)
	call lcd_out
	ldi r24,lo8(9999)
	ldi r25,hi8(9999)
1:	sbiw r24,1
	brne 1b
	rjmp .
	nop
	call lcd_enable
	ldi r24,lo8(1999)
	ldi r25,hi8(1999)
1:	sbiw r24,1
	brne 1b
	rjmp .
	nop
	call lcd_enable
	ldi r24,lo8(1999)
	ldi r25,hi8(1999)
1:	sbiw r24,1
	brne 1b
	rjmp .
	nop
	ldi r24,lo8(32)
	call lcd_out
	ldi r24,lo8(9999)
	ldi r25,hi8(9999)
1:	sbiw r24,1
	brne 1b
	rjmp .
	nop
	ldi r24,lo8(40)
	call lcd_command
	ldi r24,lo8(12)
	call lcd_command
	ldi r24,lo8(6)
	call lcd_command
	jmp lcd_clear
	.size	lcd_init, .-lcd_init
.global	lcd_home
	.type	lcd_home, @function
lcd_home:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	ldi r24,lo8(2)
	call lcd_command
	ldi r24,lo8(3999)
	ldi r25,hi8(3999)
1:	sbiw r24,1
	brne 1b
	rjmp .
	nop
	ret
	.size	lcd_home, .-lcd_home
.global	lcd_setcursor
	.type	lcd_setcursor, @function
lcd_setcursor:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	cpi r22,lo8(2)
	breq .L10
	brsh .L11
	cpi r22,lo8(1)
	breq .L12
	ret
.L11:
	cpi r22,lo8(3)
	breq .L13
	cpi r22,lo8(4)
	breq .L14
	ret
.L12:
	subi r24,lo8(-(-128))
.L15:
	jmp lcd_command
.L10:
	subi r24,lo8(-(-64))
	rjmp .L15
.L13:
	subi r24,lo8(-(-112))
	rjmp .L15
.L14:
	subi r24,lo8(-(-48))
	rjmp .L15
	.size	lcd_setcursor, .-lcd_setcursor
.global	lcd_generatechar
	.type	lcd_generatechar, @function
lcd_generatechar:
	push r16
	push r17
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
/* stack size = 4 */
.L__stack_usage = 4
	movw r16,r22
	ldi r18,lo8(8)
	mul r24,r18
	movw r24,r0
	clr __zero_reg__
	ori r24,lo8(64)
	call lcd_command
	movw r28,r16
	subi r16,-8
	sbci r17,-1
.L17:
	ld r24,Y+
	call lcd_data
	cp r28,r16
	cpc r29,r17
	brne .L17
	ldi r24,lo8(-128)
/* epilogue start */
	pop r29
	pop r28
	pop r17
	pop r16
	jmp lcd_command
	.size	lcd_generatechar, .-lcd_generatechar
.global	lcd_string
	.type	lcd_string, @function
lcd_string:
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
/* stack size = 2 */
.L__stack_usage = 2
	movw r28,r24
.L20:
	ld r24,Y+
	cpse r24,__zero_reg__
	rjmp .L21
/* epilogue start */
	pop r29
	pop r28
	ret
.L21:
	call lcd_data
	rjmp .L20
	.size	lcd_string, .-lcd_string
	.section	.rodata.str1.1,"aMS",@progbits,1
.LC4:
	.string	"weather"
	.section	.rodata
.LC0:
	.byte	21
	.byte	21
	.byte	31
	.byte	10
	.byte	10
	.byte	31
	.byte	21
	.byte	21
.LC1:
	.byte	0
	.byte	10
	.byte	0
	.byte	4
	.byte	0
	.byte	17
	.byte	14
	.byte	0
.LC2:
	.byte	32
	.byte	16
	.byte	8
	.byte	4
	.byte	2
	.byte	1
	.byte	0
	.byte	0
.LC3:
	.byte	45
	.byte	3
	.byte	124
	.byte	47
	.section	.text.startup,"ax",@progbits
.global	main
	.type	main, @function
main:
	push r28
	push r29
	in r28,__SP_L__
	in r29,__SP_H__
	sbiw r28,28
	in __tmp_reg__,__SREG__
	cli
	out __SP_H__,r29
	out __SREG__,__tmp_reg__
	out __SP_L__,r28
/* prologue: function */
/* frame size = 28 */
/* stack size = 30 */
.L__stack_usage = 30
	call lcd_init
	ldi r24,lo8(1)
	out 0x17,r24
	ldi r24,lo8(8)
	ldi r30,lo8(.LC0)
	ldi r31,hi8(.LC0)
	movw r26,r28
	adiw r26,17
	0:
	ld r0,Z+
	st X+,r0
	dec r24
	brne 0b
	ldi r24,lo8(8)
	ldi r30,lo8(.LC1)
	ldi r31,hi8(.LC1)
	movw r26,r28
	adiw r26,9
	0:
	ld r0,Z+
	st X+,r0
	dec r24
	brne 0b
	ldi r24,lo8(8)
	ldi r30,lo8(.LC2)
	ldi r31,hi8(.LC2)
	movw r26,r28
	adiw r26,1
	0:
	ld r0,Z+
	st X+,r0
	dec r24
	brne 0b
	movw r22,r28
	subi r22,-17
	sbci r23,-1
	ldi r24,lo8(1)
	call lcd_generatechar
	movw r22,r28
	subi r22,-9
	sbci r23,-1
	ldi r24,lo8(2)
	call lcd_generatechar
	movw r22,r28
	subi r22,-1
	sbci r23,-1
	ldi r24,lo8(3)
	call lcd_generatechar
	ldi r24,lo8(.LC4)
	ldi r25,hi8(.LC4)
	call lcd_string
	ldi r24,lo8(-120)
	call lcd_command
	ldi r24,lo8(126)
	call lcd_data
	ldi r24,lo8(-118)
	call lcd_command
	ldi r24,lo8(1)
	call lcd_data
	ldi r24,lo8(-64)
	call lcd_command
	ldi r24,lo8(84)
	call lcd_data
	ldi r18,lo8(799999)
	ldi r24,hi8(799999)
	ldi r25,hlo8(799999)
1:	subi r18,1
	sbci r24,0
	sbci r25,0
	brne 1b
	rjmp .
	nop
	ldi r24,lo8(101)
	call lcd_data
	ldi r18,lo8(799999)
	ldi r24,hi8(799999)
	ldi r25,hlo8(799999)
1:	subi r18,1
	sbci r24,0
	sbci r25,0
	brne 1b
	rjmp .
	nop
	ldi r24,lo8(115)
	call lcd_data
	ldi r18,lo8(799999)
	ldi r24,hi8(799999)
	ldi r25,hlo8(799999)
1:	subi r18,1
	sbci r24,0
	sbci r25,0
	brne 1b
	rjmp .
	nop
	ldi r24,lo8(116)
	call lcd_data
	ldi r24,lo8(32)
	call lcd_data
	ldi r18,lo8(799999)
	ldi r24,hi8(799999)
	ldi r25,hlo8(799999)
1:	subi r18,1
	sbci r24,0
	sbci r25,0
	brne 1b
	rjmp .
	nop
	ldi r24,lo8(2)
	call lcd_data
	lds r24,.LC3
	lds r25,.LC3+1
	lds r26,.LC3+2
	lds r27,.LC3+3
	std Y+25,r24
	std Y+26,r25
	std Y+27,r26
	std Y+28,r27
	mov r15,__zero_reg__
	ldi r16,0
	ldi r17,0
.L23:
	ldi r24,lo8(-57)
	call lcd_command
	movw r12,r16
	ldi r18,-1
	sub r12,r18
	sbc r13,r18
	andi r16,3
	clr r17
	ldi r30,lo8(25)
	ldi r31,0
	add r30,r28
	adc r31,r29
	add r30,r16
	adc r31,r17
	ld r24,Z
	call lcd_data
	mov r24,r15
	andi r24,lo8(1)
	out 0x18,r24
	ldi r24,lo8(1279999)
	ldi r25,hi8(1279999)
	ldi r18,hlo8(1279999)
1:	subi r24,1
	sbci r25,0
	sbci r18,0
	brne 1b
	rjmp .
	nop
	inc r15
	mov r24,r15
	andi r24,lo8(1)
	mov r15,r24
	movw r16,r12
	rjmp .L23
	.size	main, .-main
	.ident	"GCC: (Fedora 6.2.0-1.fc25) 6.2.0"
.global __do_copy_data
