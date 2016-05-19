/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 *  @brief Wire library, ported from Arduino. Provides a lean
 *  interface to I2C (two-wire) communication.
 */

//#include "wirish.h"
#include "Arduino.h"

#ifndef _WIRE_H_
#define _WIRE_H_


typedef struct {
  uint8_t scl;
  uint8_t sda;
} Port;

/* You must update the online docs if you change this value. */
#define WIRE_BUFSIZ 32

/* return codes from endTransmission() */
#define SUCCESS   0        /* transmission was successful */
#define EDATA     1        /* too much data */
#define ENACKADDR 2        /* readd nack on transmit of address */
#define ENACKTRNS 3        /* readd nack on transmit of data */
#define EOTHER    4        /* other error */

//#define SDA 20
//#define SCL 21
#define SDA 14
#define SCL 15

#define I2C_WRITE 0
#define I2C_READ  1

//#define I2C_DELAY do{for(int i=0;i<40;i++) {asm volatile("nop");}}while(0)
#define I2C_DELAY //

class TwoWire {
 private:
    uint8_t rx_buf[WIRE_BUFSIZ];      /* read buffer */
    uint8_t rx_buf_idx;               /* first unread idx in rx_buf */
    uint8_t rx_buf_len;               /* number of bytes read */

    uint8_t tx_addr;                  /* address transmitting to */
    uint8_t tx_buf[WIRE_BUFSIZ];      /* transmit buffer */
    uint8_t tx_buf_idx;  /* next idx available in tx_buf, -1 overflow */
    boolean tx_buf_overflow;
    Port port;
    uint8_t writeOneByte(uint8_t);
    uint8_t readOneByte(uint8_t, uint8_t*);
 public:
    TwoWire();
    void begin();
    void begin(uint8_t, uint8_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t requestFrom(uint8_t, int);
    uint8_t requestFrom(int, int);
    void write(uint8_t);
    void write(uint8_t*, int);
    void write(int);
    void write(int*, int);
    void write(char*);
    uint8_t available();
    uint8_t read();
};

void    i2c_start(Port port);
void    i2c_stop(Port port);
boolean i2c_get_ack(Port port);
void    i2c_write_ack(Port port);
void    i2c_write_nack(Port port);
uint8_t   i2c_shift_in(Port port);
void    i2c_shift_out(Port port, uint8_t val);

extern TwoWire Wire;

#endif // _WIRE_H_





















#if 0
/*
 * TwoWire.h - TWI/I2C library for Arduino Due
 * Copyright (c) 2011 Cristian Maglie <c.maglie@bug.st>.
 * All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef TwoWire_h
#define TwoWire_h

//#include <include/twi.h>

#include "Stream.h"
#include "variant.h"

#include "SERCOM.h"
#include "RingBuffer.h"

#define BUFFER_LENGTH 32


class TwoWire : public Stream
{
	public:
		TwoWire(SERCOM *s);
		void begin();
		void begin(uint8_t);

		void beginTransmission(uint8_t);
		uint8_t endTransmission(bool stopBit);
		uint8_t endTransmission(void);

		uint8_t requestFrom(uint8_t address, size_t quantity, bool stopBit);
		uint8_t requestFrom(uint8_t address, size_t quantity);

		size_t write(uint8_t data);
		size_t write(const uint8_t * data, size_t quantity);

		virtual int available(void);
		virtual int read(void);
		virtual int peek(void);
		virtual void flush(void);
		void onReceive(void(*)(int));
		void onRequest(void(*)(void));

		using Print::write;

		void onService(void);

	private:
		SERCOM * sercom;
		bool transmissionBegun;

		// RX Buffer
		RingBuffer rxBuffer;

		//TX buffer
		RingBuffer txBuffer;
		uint8_t txAddress;


		// Service buffer
		//uint8_t srvBuffer[BUFFER_LENGTH];
		//uint8_t srvBufferIndex;
		//uint8_t srvBufferLength;

		// Callback user functions
		void (*onRequestCallback)(void);
		void (*onReceiveCallback)(int);

		// TWI state
		//enum TwoWireStatus
    //{
		//	UNINITIALIZED,
		//	MASTER_IDLE,
		//	MASTER_SEND,
		//	MASTER_RECV,
		//	SLAVE_IDLE,
		//	SLAVE_RECV,
		//	SLAVE_SEND
		//};
		//TwoWireStatus status;

		// TWI clock frequency
		static const uint32_t TWI_CLOCK = 100000;

		// Timeouts
		//static const uint32_t RECV_TIMEOUT = 100000;
		//static const uint32_t XMIT_TIMEOUT = 100000;
};

extern TwoWire Wire;

#endif



#endif