/*
 * main.c
 *
 *  Created on: 14.09.2016
 *      Author: marcinpaczkowski
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "Uart/Uart.h"
#include <stdio.h>
#include <stdlib.h>
#include "i2c/i2cmaster.h"

#define LED__SD_OK (1<<PB0);
#define LED__SD_OK_ON PORTB &= ~LED__SD_OK
#define LED__SD_OK_OFF PORTB |= LED__SD_OK
#define LED__SD_OK_TOG PORTB ^= LED__SD_OK

#define SWITCH_KEY (1<<PC7)

#define DAC_WRITE 0xC0
#define TIMER1_PRESCALER 256
#define MAX_SINE_SIGNAL_FREQUENCE 400

void InitTimer();
void InitUart();
void InitTestButton();
uint16_t* InitSinusSampleValues();

uint8_t CheckSwitchButton();
void TestRectangularSignal();
void GenerateSineSignal();
uint16_t RecalculateInterruptionInterval(uint32_t frequency);
void SetNewInterruptParameters(uint16_t ocrValue);

uint16_t* _sinusSamplesReference;

volatile char _levelPreamble;
volatile char _levelNum;
volatile uint8_t _frequencyValueBuffer[7];
volatile uint16_t _frequencyValueInt;
volatile uint8_t _generateSineSampleSignal;
volatile uint8_t _sinusSampleCounter;

volatile uint8_t _tempCounter;

int main(void) {
	InitTimer();
	InitUart();
	InitTestButton();
	i2c_init();

	DDRB |= LED__SD_OK;

	_sinusSamplesReference = InitSinusSampleValues();
	_levelPreamble = 0;
	_frequencyValueInt = 0;
	_sinusSampleCounter = 0;

	_tempCounter = 0;

	sei();

    while (1) {
    	if (CheckSwitchButton()) {
			char str[15];
    		sprintf(str, "%d", _frequencyValueInt);
    		uart0_puts(str);
    		uart0_puts("\r\n");
    	}

    	//GenerateSineSignal();

    	if (_generateSineSampleSignal) {
    		GenerateSineSignal();
    		_generateSineSampleSignal = 0;
    	}
    }
}

void InitTimer() {
	TCCR1B |= (1<<CS12); // Ustawiamy preskalera
	TCCR1B |= (1<<WGM12); // Tryb CTC;
	TIMSK |= (1<<OCIE1B); // Przerwanie od porownania
	OCR1A = RecalculateInterruptionInterval(1); // Wartosc top
}

void InitUart() {
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
}

void InitTestButton() {
	DDRC &= ~(SWITCH_KEY);
	PORTC |= SWITCH_KEY;
}

uint16_t* InitSinusSampleValues() {
	static uint16_t sinusSamples[] = {
			2046,
			2445,
			2829,
			3183,
			3494,
			3748,
			3937,
			4054,
			4093,
			4054,
			3937,
			3748,
			3494,
			3183,
			2829,
			2445,
			2046,
			1647,
			1263,
			909,
			599,
			344,
			155,
			39,
			0,
			39,
			155,
			344,
			599,
			909,
			1263,
			1647
	};

	return sinusSamples;
}

ISR(TIMER1_COMPB_vect) {
	_generateSineSampleSignal = 1;
	_tempCounter++;
	if(_tempCounter==15) {
		LED__SD_OK_TOG;
		_tempCounter = 0;
	}
}

ISR(USART_RXC_vect) {
	char receivedChar = UDR;
	if(_levelPreamble < 5 && _levelNum == 0) {
		switch(_levelPreamble) {
		case 0:
			if(receivedChar == 'f') { _levelPreamble = 1; } else { _levelPreamble=0; };
			break;
		case 1:
			if(receivedChar == 'r') {
				_levelPreamble = 2;
			} else {
				_levelPreamble=0;
			};
			break;
		case 2:
			if(receivedChar == 'e') {
				_levelPreamble = 3;
			} else {
				_levelPreamble=0;
			};
			break;
		case 3:
			if(receivedChar == 'q') {
				_levelPreamble = 4;
			} else {
				_levelPreamble = 0;
			}
			break;
		case 4:
			if(receivedChar == '_') {
				_levelPreamble = 5;

			} else {
				_levelPreamble = 0;
			}
			break;
		default:
			_levelPreamble = 0;
		}
	} else if (_levelPreamble == 5 && _levelNum < 6) {
		if(receivedChar > 47 && receivedChar < 57) {
			_frequencyValueBuffer[_levelNum] = receivedChar;

			_levelNum++;
		} else if(receivedChar == 59) {
			// Pojawil sie srednik
			_frequencyValueBuffer[_levelNum] = 10;
			uart0_puts("Wartosc: ");
			uart0_puts(&_frequencyValueBuffer[0]);
			uart0_puts("\r\n");
			_frequencyValueInt = atoi(_frequencyValueBuffer);
			uint16_t ocrValue = RecalculateInterruptionInterval(_frequencyValueInt);
			SetNewInterruptParameters(ocrValue);
			_levelNum = 0;
			_levelPreamble = 0;
		} else {
			_levelNum = 0;
			_levelPreamble = 0;
		}
	} else if(_levelPreamble > 5 || _levelNum > 5) {
		// Coś poszło nie tak, resetujemy
		_levelNum = 0;
		_levelPreamble = 0;
	}
}

uint8_t CheckSwitchButton() {
	if (!(PINC & SWITCH_KEY)) {
		_delay_ms(80);
		if (!(PINC & SWITCH_KEY)) {
			return 1;
		}
	}

	return 0;
}

void TestRectangularSignal() {
	static uint8_t isZero = 0;
	uint16_t Vout = 0;
	if (isZero == 0) {
		Vout = 3000;
		isZero = 1;
	}
	else {
		Vout = 1000;
		isZero = 0;
	}

	char lVout = Vout;
	char hVout = (Vout>>8) & 0x0F;
	i2c_start(DAC_WRITE);
	i2c_write(hVout);
	i2c_write(lVout);
	i2c_stop();
}

void GenerateSineSignal() {
	uint16_t sinusSample = *(_sinusSamplesReference + _sinusSampleCounter);

	char lVout = sinusSample;
	char hVout = (sinusSample>>8) & 0x0F;

	i2c_start(DAC_WRITE);
	i2c_write(hVout);
	i2c_write(lVout);
	i2c_stop();

	_sinusSampleCounter++;
	if (_sinusSampleCounter > 31)
		_sinusSampleCounter = 0;
}

uint16_t RecalculateInterruptionInterval(uint32_t frequency) {
	if (frequency > MAX_SINE_SIGNAL_FREQUENCE) {
		frequency = MAX_SINE_SIGNAL_FREQUENCE;
	}

	uint32_t tmpValue = ((F_CPU / (TIMER1_PRESCALER*frequency)) - 1) / 32;

	return tmpValue;
}

void SetNewInterruptParameters(uint16_t ocrValue) {
	cli();
	OCR1A = ocrValue;
	TCNT1=0;
	sei();
}
