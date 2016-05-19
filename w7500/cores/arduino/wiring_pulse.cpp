/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"
#include "wiring_private.h"
#include "wiring_pulse.h"
/* Measures the length (in microseconds) of a pulse on the pin; state is HIGH
 * or LOW, the type of pulse to measure.  Works on pulses from 2-3 microseconds
 * to 3 minutes in length, but must be called at least a few dozen microseconds
 * before the start of the pulse. */
extern uint32_t pulseIn( uint32_t pin, uint32_t state, uint32_t timeout )
{
    uint32_t startTick, endTick;

// timeout이 반영되지 않음
    startTick = *(volatile unsigned int *)0x4000530C;       // 현재의 System clock 값을 읽는다.
    while(digitalRead(pin) != state);                                    // state가 될때까지 기다린다.
    endTick = *(volatile unsigned int *)0x4000530C;         // 현재의 System clock 값을 읽는다.

    return (endTick - startTick) / 10;

#if 0
//    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
//    *(volatile unsigned long *) 0x4000D000 = 'E';

    uint32_t startTone, endTone;
    uint32_t startTick, endTick;
    gPulsePinNum = pin;

// tone으로 사용이 될 GPIO 설정
// pinMode(pin, INPUT);

    startTone = (*(volatile unsigned int *)0xE000E018) & 0xFFFFFF;       // 현재의 System clock 값을 읽는다.
    startTick = TimerCnt;
    while(digitalRead(pin) != state);                                    // state가 될때까지 기다린다.
    endTone = (*(volatile unsigned int *)0xE000E018) & 0xFFFFFF;         // 현재의 System clock 값을 읽는다.
    endTick = TimerCnt;

//    printf("a");

    if(endTick = startTick)
        return (startTone - endTone)/20;
    return ((startTone + (20000 - endTone)) / 20) + ((endTick - startTick - 1) * 1000);
#endif

#if 0
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// digitalRead() instead yields much coarser resolution.
	PinDescription p = g_APinDescription[pin];
	uint32_t width = 0; // keep initialization out of time critical area
	
	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 22 clock cycles per iteration.
	uint32_t numloops = 0;
	uint32_t maxloops = microsecondsToClockCycles(timeout) / 22;
	// wait for any previous pulse to end
	while (digitalRead(pin) == state)
		if (numloops++ == maxloops)
			return 0;
	
	
	// wait for the pulse to start
	while (digitalRead(pin) != state)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to stop
	while (digitalRead(pin) == state) {
		if (numloops++ == maxloops)
			return 0;
		width++;
	}

	// convert the reading to microseconds. The loop has been determined
	// to be 52 clock cycles long and have about 16 clocks between the edge
	// and the start of the loop. There will be some error introduced by
	// the interrupt handlers.
	return clockCyclesToMicroseconds(width * 52 + 16);
#endif
}
