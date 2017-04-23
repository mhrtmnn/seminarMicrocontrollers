
main.o:     file format elf32-avr


Disassembly of section .text.startup:

00000000 <main>:
main():
/home/marco/Documents/Projects/University/seminarMicrocontrollers/blink/main.c:6
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRB = 0xFF;
   0:	8f ef       	ldi	r24, 0xFF	; 255
   2:	87 bb       	out	0x17, r24	; 23
/home/marco/Documents/Projects/University/seminarMicrocontrollers/blink/main.c:11

	while(1)
	{

		PORTB = 0x01;
   4:	81 e0       	ldi	r24, 0x01	; 1
   6:	88 bb       	out	0x18, r24	; 24
/usr/avr/include/util/delay.h:187
	#else
		//round up by default
		__ticks_dc = (uint32_t)(ceil(fabs(__tmp)));
	#endif

	__builtin_avr_delay_cycles(__ticks_dc);
   8:	2f ef       	ldi	r18, 0xFF	; 255
   a:	34 e3       	ldi	r19, 0x34	; 52
   c:	9c e0       	ldi	r25, 0x0C	; 12
   e:	21 50       	subi	r18, 0x01	; 1
  10:	30 40       	sbci	r19, 0x00	; 0
  12:	90 40       	sbci	r25, 0x00	; 0
  14:	01 f4       	brne	.+0      	; 0x16 <main+0x16>
  16:	00 c0       	rjmp	.+0      	; 0x18 <main+0x18>
  18:	00 00       	nop
/home/marco/Documents/Projects/University/seminarMicrocontrollers/blink/main.c:13
		_delay_ms(500);
		PORTB = 0x00;
  1a:	18 ba       	out	0x18, r1	; 24
/usr/avr/include/util/delay.h:187
  1c:	2f ef       	ldi	r18, 0xFF	; 255
  1e:	34 e3       	ldi	r19, 0x34	; 52
  20:	9c e0       	ldi	r25, 0x0C	; 12
  22:	21 50       	subi	r18, 0x01	; 1
  24:	30 40       	sbci	r19, 0x00	; 0
  26:	90 40       	sbci	r25, 0x00	; 0
  28:	01 f4       	brne	.+0      	; 0x2a <main+0x2a>
  2a:	00 c0       	rjmp	.+0      	; 0x2c <main+0x2c>
  2c:	00 00       	nop
  2e:	00 c0       	rjmp	.+0      	; 0x30 <__zero_reg__+0x2f>
