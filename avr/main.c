/*
    AVR code for PT100 readout board.
    
    Command set similar to ADAM4000 series.
    Interface: RS485 isolated, 8N1, baud rate configurable (default 9600).
    Default bus address: 0x01

    Room for improvement:
        - Calibration resistor hardcoded (470R)
        - Linear approximation of T(R) only good for 0°C < T < 100°C
        - RTD fault detection rudimentary

    Copyright (C) Jan 2018, M. W. Bruker

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#define F_CPU 14745600UL

#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <util/delay.h>
#include "uart.h"

// deltaT = R/(alpha * R0) - 1/alpha
// unit for calculation = 0.01 K
#define TEMP_SLOPE 1.8625236        // 470 Ohm / (alpha * R0 * (2^16 - 1))
#define TEMP_OFFSET (25970 - 27315) // 1/alpha + conversion from C to K
#define BUFFER_SIZE 20
#define AVERAGE_SAMPLES 32

uint8_t saveBusAddress EEMEM = 0x01;
uint8_t saveBaudRate EEMEM = 0x06;    // 9.6 kbps by default


// baud rate codes start at 0x03 = 1.2 kbps (see ADAM 4000 manual p. 121 / PDF p. 135)
// baud rates > 38.4 kbps are not supported by ADAM but might as well be supported by us
#define NUMBER_BAUD_RATES 9
const uint16_t baudRateCodes[NUMBER_BAUD_RATES] PROGMEM = {
    767,    //  1.2 kbps
    383,    //  2.4 kbps
    191,    //  4.8 kbps
    95,     //  9.6 kbps
    47,     // 19.2 kbps
    23,     // 38.4 kbps
    15,     // 57.6 kbps
    7,      // 115.2 kbps
    3       // 230.4 kbps
};


const char moduleNameString[] PROGMEM = "MWB-PT100";
const char firmwareVersionString[] PROGMEM = "2018/01/02";

uint8_t busAddress;
uint8_t baudRate;
uint16_t currentTemp;
volatile uint8_t ledTimer;
volatile uint8_t max31865_dataReady;

// MAX31865 DRDY falling edge
ISR(INT2_vect)
{
	max31865_dataReady = 1;
}

ISR(TIMER0_OVF_vect)
{
    // turn off data LED after 5 overflows (89 ms)
    if (++ledTimer >= 5) {
        TIMSK &= ~(1 << TOIE0);
        PORTA &= ~(1 << PA3);
    }
}

void max31865_init()
{
	MCUCSR &= ~(1 << ISC2); // trigger INT2 on falling edge
	GICR |= (1 << INT2);
	
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (1 << SPR0); // initialize SPI, f=fclk/16
	
	PORTB &= ~(1 << PB4); // pull CS low
	_delay_loop_1(1);
	
	SPDR = 0x80; // write to configuration register
	while (!(SPSR & (1 << SPIF)));
	SPDR = 0b11000001; // Vbias on, Conversion mode auto, 4-wire RTD, 50Hz filter
	while (!(SPSR & (1 << SPIF)));
	
	PORTB |= (1 << PB4);
	max31865_dataReady = !(PINB & (1 << PB2));
}
	
void setBaudRate()
{
    if ((baudRate < 3) || (baudRate > NUMBER_BAUD_RATES + 2))
        baudRate = 6; // 9.6 kbps

    uart_init(pgm_read_word(&(baudRateCodes[baudRate - 3])));
}

// Not proud of this. Is there a better way?
uint8_t hex2uint8(char byte1, char byte2)
{
    uint8_t result;
    if ((byte1 >= '0') && (byte1 <= '9'))       // 0-9
        result = (byte1 - '0') << 4;
    else if ((byte1 >= 'A') && (byte1 <= 'F'))  // A-F
        result = (byte1 + 10 - 'A') << 4;
    else                                        // a-f
        result = (byte1 + 10 - 'a') << 4;

    if ((byte2 >= '0') && (byte2 <= '9'))
        result |= (byte2 - '0');
    else if ((byte2 >= 'A') && (byte2 <= 'F'))
        result |= (byte2 + 10 - 'A');
    else
        result |= (byte2 + 10 - 'a');

    return result;
}

/*
    prefix: byte 1, one of %, $, #
    adress{1,2}: byte 2/3, hexadecimal ASCII string containing the bus address
    string: remainder of command after bus address, line terminator stripped
*/
void processCommand(char prefix, char address1, char address2, char *string, uint8_t len)
{
    if (hex2uint8(address1, address2) != busAddress)
        return;
    
    PORTA |= (1 << PA3);        // enable data LED    
    TCNT0 = 0;                  // reset timer
    ledTimer = 0;               // reset overflow counter
    TIMSK = (1 << TOIE0);       // enable timer overflow interrupt


    if (prefix == '#') {        // #AA: read analog data
        char buffer[10];
        uart_putc('>');
        utoa(currentTemp, buffer, 10);
        // insert fixed decimal point
        uint8_t l = strlen(buffer);
        if (l >= 3) {
            buffer[l + 1] = 0;
            buffer[l] = buffer[l - 1];
            buffer[l - 1] = buffer[l - 2];
            buffer[l - 2] = '.';
        }
        uart_puts(buffer);
        uart_putc('\r');
        uart_transmit();
        return;
    }

    if (prefix == '%') {        // %AANNTTCCFF: set configuration
        if (len < 8) {
            uart_putc('?');
            uart_putc(address1);
            uart_putc(address2);
            uart_putc('\r');
            uart_transmit();
            return;
        }
        busAddress = hex2uint8(string[0], string[1]);   // NN
        // TT ignored for now
        baudRate = hex2uint8(string[4], string[5]);     // CC
        // FF ignored for now

        uart_putc('!');
        uart_putc(string[0]);
        uart_putc(string[1]);
        uart_putc('\r');
        uart_transmit();

        eeprom_update_byte(&saveBusAddress, busAddress);
        eeprom_update_byte(&saveBaudRate, baudRate);
        
        _delay_ms(1000);
        setBaudRate();
    }

    if (prefix != '$')
        return;

	switch (string[0]) {
        // $AA2: read configuration status
        // type code unsupported, data format etc. unsupported
        case '2': {
            char buffer[3];
            uart_putc('!');
			uart_putc(address1);
            uart_putc(address2);
            uart_putc('F');
            uart_putc('F');
            utoa(baudRate, buffer, 16);
            if (!buffer[1])
                uart_putc('0');
            uart_puts(buffer);
            uart_putc('0');
            uart_putc('0');
            uart_putc('\r');
            uart_transmit();
            break;
        }

		// $AAM: read module name
		case 'M': {
			uart_putc('!');
			uart_putc(address1);
            uart_putc(address2);
			uart_puts_p(moduleNameString);
			uart_putc('\r');
            uart_transmit();
			break;
		}
		
        // $AAF: read firmware version
		case 'F': {
			uart_putc('!');
			uart_putc(address1);
            uart_putc(address2);
			uart_puts_p(firmwareVersionString);
			uart_putc('\r');
            uart_transmit();
			break;
		}

	}
}

int main()
{
	char commandBuffer[BUFFER_SIZE];
	uint8_t bufferIndex = 0;
	
	// set up ports, enable pull-up for unused pins
	// PA3 = data LED
	DDRA = (1 << PA3);
	PORTA = ~(1 << PA3);
	// PB2 = DRDY, PB4 = CS, PB5 = MOSI, PB6 = MISO, PB7 = SCK
	DDRB = (1 << PB4) | (1 << PB5) | (1 << PB7);
	PORTB = 0xff;
	DDRC = 0;
	PORTC = 0xff;

    busAddress = eeprom_read_byte(&saveBusAddress);
    baudRate = eeprom_read_byte(&saveBaudRate);
    setBaudRate();

    // set up timer for data LED
    TCCR0 = (1 << CS02) | (1 << CS00);      // clock prescaler = 1024, ~70µs per tick

	// PD2 = EN, PD3 = DE
	DDRD |= (1 << PD2) | (1 << PD3);
	// enable ADuM
	PORTD |= (1 << PD2);
	
	uint32_t tempSum = 0;
	uint8_t sampleCount = 0;
	
	currentTemp = 0;
	
	sei();
	
	max31865_init();
	
	while (1) {
		if (max31865_dataReady) {
			uint16_t rtdValue;
			max31865_dataReady = 0;
			
			PORTB &= ~(1 << PB4);
			_delay_loop_1(5);
			
			SPDR = 0x01;
			while (!(SPSR & (1 << SPIF)));
			SPDR = 0x00;
			while (!(SPSR & (1 << SPIF)));
			rtdValue = ((uint16_t) SPDR) << 8;
			SPDR = 0x00;
			while (!(SPSR & (1 << SPIF)));
			rtdValue |= SPDR;
			
			PORTB |= (1 << PB4);
			
            if (rtdValue & 0x0001) { // check fault bit
                tempSum = 0;
                currentTemp = 0;
                sampleCount = 0;
            } else {
    			tempSum += rtdValue;
    			if (++sampleCount == AVERAGE_SAMPLES) {
    				sampleCount = 0;
    				currentTemp = (tempSum / AVERAGE_SAMPLES) * TEMP_SLOPE - TEMP_OFFSET;
    				tempSum = 0;
    			}
            }
		}
        
		uint16_t c;
		while (!((c = uart_getc()) & UART_NO_DATA)) {
			if (((uint8_t) c == '\r') || ((uint8_t) c == '\n')) {
				commandBuffer[bufferIndex] = 0;
				if (bufferIndex > 2) {
                    processCommand(commandBuffer[0], commandBuffer[1], commandBuffer[2], &commandBuffer[3], bufferIndex - 3);
				}
				bufferIndex = 0;
			} else {
				commandBuffer[bufferIndex++] = c;
			}
		}
	}
}
