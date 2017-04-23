#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

int main(void)
{
	// Initialisierung des LCD
	// Nach der Initialisierung müssen auf dem LCD vorhandene schwarze Balken
	// verschwunden sein
	lcd_init();

	//led output
	DDRB = 0x01;

	//create custom icon
	uint8_t sun[8] = {
	//       .....
		0b00010101,
		0b00010101,
		0b00011111,
		0b00001010,
		0b00001010,
		0b00011111,
		0b00010101,
		0b00010101,
	//       ^^^^^  
	};

	uint8_t smiley[8] = {
	//       .....
		0b00000000,
		0b00001010,
		0b00000000,
		0b00000100,
		0b00000000,
		0b00010001,
		0b00001110,
		0b00000000,
	//       ^^^^^  
	};

	uint8_t backslash[8] = {
	//       .....
		0b00100000,
		0b00010000,
		0b00001000,
		0b00000100,
		0b00000010,
		0b00000001,
		0b00000000,
		0b00000000,
	//       ^^^^^  
	};

	lcd_generatechar(1, sun);
	lcd_generatechar(2, smiley);
	lcd_generatechar(3, backslash);

	//put string
	lcd_string("weather");

	//put rightarrow
	lcd_setcursor(8, 1);
	lcd_data(0x7E);

	//put sun
	lcd_setcursor(10, 1);
	lcd_data(0x01);

	// Die Ausgabemarke in die 2te Zeile setzen
	lcd_setcursor(0, 2);

	// Text in einzelnen Zeichen ausgeben
	lcd_data( 'T' );
	_delay_ms(500);
	lcd_data( 'e' );
	_delay_ms(500);
	lcd_data( 's' );
	_delay_ms(500);
	lcd_data( 't' );
	lcd_data( ' ' );
	_delay_ms(500);

	//put smiley
	lcd_data( 0x02 );

	int i = 0;
	char spin[] = {'-', 0x03, '|', '/'}; //0x03 is backslash
	while(1)
	{
		lcd_setcursor( 7, 2 );
		lcd_data(spin[i++ % 4]);

		//toggle led
		PORTB = (i % 2) ? 0x00 : 0x01;

		_delay_ms(800);

	}

	return 0;
}




// Ansteuerung eines HD44780 kompatiblen LCD im 4-Bit-Interfacemodus
// http://www.mikrocontroller.net/articles/HD44780
// http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung
//
// Die Pinbelegung ist über defines in lcd-routines.h einstellbar
 

////////////////////////////////////////////////////////////////////////////////
// Erzeugt einen Enable-Puls
static void lcd_enable( void )
{
		LCD_PORT |= (1<<LCD_EN);     // Enable auf 1 setzen
		_delay_us( LCD_ENABLE_US );  // kurze Pause
		LCD_PORT &= ~(1<<LCD_EN);    // Enable auf 0 setzen
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet eine 4-bit Ausgabeoperation an das LCD (can be both data and command)
static void lcd_out( uint8_t data )
{
		data &= 0xF0;                       // obere 4 Bit maskieren
 
		LCD_PORT &= ~(0xF0>>(4-LCD_DB));    // Maske löschen (shift out the lower 4 bits, then shift left (neg right shift) to p0)
		LCD_PORT |= (data>>(4-LCD_DB));     // Bits setzen
		lcd_enable();						// set EN for short period
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet ein Datenbyte an das LCD
void lcd_data( uint8_t data )
{
		LCD_PORT |= (1<<LCD_RS);    // RS auf 1 setzen (send data)
 
		lcd_out( data );            // zuerst die oberen, 
		lcd_out( data<<4 );         // dann die unteren 4 Bit senden
 
		_delay_us( LCD_WRITEDATA_US );
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet ein Befehlbyte an das LCD
void lcd_command( uint8_t data )
{
		LCD_PORT &= ~(1<<LCD_RS);    // RS auf 0 setzen (send command)
 
		lcd_out( data );              // zuerst die oberen, 
		lcd_out( data<<4 );           // dann die unteren 4 Bit senden
 
		_delay_us( LCD_COMMAND_US );
}
 


////////////////////////////////////////////////////////////////////////////////
// Initialisierung: muss ganz am Anfang des Programms aufgerufen werden.
void lcd_init( void )
{
		// verwendete Pins auf Ausgang schalten
		uint8_t pins = (0x0F << LCD_DB) |           // 4 Datenleitungen
					   (0x01 << LCD_RS) |           // R/S Leitung
					   (0x01 << LCD_EN);            // Enable Leitung
		LCD_DDR |= pins;
 
		// initial alle Ausgänge auf Null
		LCD_PORT &= ~pins;
 
		// warten auf die Bereitschaft des LCD
		_delay_ms( LCD_BOOTUP_MS );
		
		// Soft-Reset muss 3mal hintereinander gesendet werden zur Initialisierung
		lcd_out( LCD_SOFT_RESET );
		_delay_ms( LCD_SOFT_RESET_MS1 );
 
		lcd_enable();
		_delay_ms( LCD_SOFT_RESET_MS2 );
 
		lcd_enable();
		_delay_ms( LCD_SOFT_RESET_MS3 );
 
		// 4-bit Modus aktivieren 
		lcd_out( LCD_SET_FUNCTION | LCD_FUNCTION_4BIT );
		_delay_ms( LCD_SET_4BITMODE_MS );
 
		// 4-bit Modus / 2 Zeilen / 5x7
		lcd_command( LCD_SET_FUNCTION |
					 LCD_FUNCTION_4BIT |
					 LCD_FUNCTION_2LINE |
					 LCD_FUNCTION_5X7 );
 
		// Display ein / Cursor aus / Blinken aus
		lcd_command( LCD_SET_DISPLAY |
					 LCD_DISPLAY_ON |
					 LCD_CURSOR_OFF |
					 LCD_BLINKING_OFF); 
 
		// Cursor inkrement / kein Scrollen
		lcd_command( LCD_SET_ENTRY |
					 LCD_ENTRY_INCREASE |
					 LCD_ENTRY_NOSHIFT );
 
		lcd_clear();
}
	
////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl zur Löschung des Displays
void lcd_clear( void )
{
		lcd_command( LCD_CLEAR_DISPLAY );
		_delay_ms( LCD_CLEAR_DISPLAY_MS );
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl: Cursor Home
void lcd_home( void )
{
		lcd_command( LCD_CURSOR_HOME );
		_delay_ms( LCD_CURSOR_HOME_MS );
}
 
////////////////////////////////////////////////////////////////////////////////
// Setzt den Cursor in Spalte x (0..15) Zeile y (1..4) 
 
void lcd_setcursor( uint8_t x, uint8_t y )
{
		uint8_t data;
 
		switch (y)
		{
				case 1:    // 1. Zeile
						data = LCD_SET_DDADR + LCD_DDADR_LINE1 + x;
						break;
 
				case 2:    // 2. Zeile
						data = LCD_SET_DDADR + LCD_DDADR_LINE2 + x;
						break;
 
				case 3:    // 3. Zeile
						data = LCD_SET_DDADR + LCD_DDADR_LINE3 + x;
						break;
 
				case 4:    // 4. Zeile
						data = LCD_SET_DDADR + LCD_DDADR_LINE4 + x;
						break;
 
				default:
						return;                                   // für den Fall einer falschen Zeile
		}
 
		lcd_command( data );
}
 
////////////////////////////////////////////////////////////////////////////////
// Schreibt ein Zeichen in den Character Generator RAM
 
void lcd_generatechar( uint8_t startadresse, const uint8_t *data )
{
		// Startposition des Zeichens einstellen
		lcd_command( LCD_SET_CGADR | (startadresse<<3) ); //Startadressen: 0;1;2;3;4;5;6;7
 
		// Bitmuster übertragen
		for ( uint8_t i=0; i<8; i++ )
		{
				lcd_data( data[i] );
		}
		lcd_command(LCD_SET_DDADR); //DRAM auf 0 setzen
}

////////////////////////////////////////////////////////////////////////////////
// Schreibt einen String auf das LCD (convenience function)
 
void lcd_string( const char *data )
{
		while( *data != '\0' )
				lcd_data( *data++ );
}
 
