#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int main(void)
{
    //use PB0 as status indicator
    DDRB = (1 << PB0);

    //inital led on
    PORTB = (1 << PB0);

    //The low level of INT0 generates an interrupt request (0 is initial value)
    //MCUCR |= (0 << ISC01 | 0 << ISC00);

    //General Interrupt Control Register: activate INT0
    GICR |= (1 << INT0);

    // Activate pullup resistor of PD2 (INT0 pin)
    PORTD |= (1 << PD2);

    //set global interrupt enable (set I(nterrupt) flag in SREG)
    sei();

    // main loop
    while(1)
    {}

    return 0;
}

//Interrupt service routine for INT0 (vector_1)
ISR(INT0_vect)
{
    PORTB ^= (1 << PB0);
    _delay_ms(200);
}
