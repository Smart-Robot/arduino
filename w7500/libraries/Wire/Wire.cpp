/******************************************************************************
 * The MIT License
 *
 *****************************************************************************/

/**
 *  @brief Wire library, ported from Arduino orginally by Leaf Labs. Provides a simplistic
 *  interface to i2c.
 *  Substantially rewritten after many hours with a logic analyzer by Martin Mason for the CM-900
 *  SDA and SCL must be tied high with ~2K resitors as SDA and SCL are tri-state.
 */

#include "Wire.h"
//#include "wirish.h"

/* low level conventions:
 * - SDA/SCL idle high (expected high)
 * - always start with i2c_delay rather than end
 */
uint32_t i2c_delay = 1;

void i2c_start(Port port) {
    I2C_DELAY;
    digitalWrite(port.sda,LOW);
    I2C_DELAY;
    digitalWrite(port.scl,LOW);
	I2C_DELAY;
}

void i2c_stop(Port port) {
    I2C_DELAY;
    digitalWrite(port.scl,HIGH);
	I2C_DELAY;
    digitalWrite(port.sda,HIGH);
}

boolean i2c_get_ack(Port port) {

    digitalWrite(port.scl,LOW);
    digitalWrite(port.sda,HIGH);
    I2C_DELAY;

    digitalWrite(port.scl,HIGH);

    if (!digitalRead(port.sda)) {
      I2C_DELAY;
      digitalWrite(port.scl,LOW);
	  I2C_DELAY;
	  digitalWrite(port.sda,HIGH);
      return true;
    } else {
      I2C_DELAY;
      digitalWrite(port.scl,LOW);
	  I2C_DELAY;
	  digitalWrite(port.sda,HIGH);
	  w_printf("i2c fail\n");
      return false;
    }
}


void i2c_write_ack(Port port) {
    I2C_DELAY;
    digitalWrite(port.scl,HIGH);
    I2C_DELAY;
	digitalWrite(port.scl,LOW);
    digitalWrite(port.sda,HIGH);
    I2C_DELAY;
    digitalWrite(port.sda,LOW);
}

void i2c_write_nack(Port port) {	

	digitalWrite(port.sda,LOW); //LOW
	I2C_DELAY;
	digitalWrite(port.scl,HIGH);
    I2C_DELAY;
	digitalWrite(port.scl,LOW);
	digitalWrite(port.sda,HIGH);
}

uint8_t i2c_shift_in(Port port) {
    uint8_t data = 0;
    int i;
    for (i=0;i<8;i++) {
        I2C_DELAY;
        digitalWrite(port.scl,HIGH);
        I2C_DELAY;
        data += digitalRead(port.sda) << (7-i);
        digitalWrite(port.scl,LOW);
    }

    return data;
}

void i2c_shift_out(Port port, uint8_t val) {
    int i;
    for (i=0;i<8;i++) {
        I2C_DELAY;
        digitalWrite(port.sda, !!(val & (1 << (7 - i))));
        I2C_DELAY;
        digitalWrite(port.scl, HIGH);
        I2C_DELAY;
        digitalWrite(port.scl, LOW);
////////// 디버깅을 위해서 추가
//        get_byte();
    }
}

// Constructors ////////////////////////////////////////////////////////////////

TwoWire::TwoWire()
{
	  i2c_delay = 0;
    rx_buf_idx = 0;
    rx_buf_len = 0;
    tx_addr = 0;
    tx_buf_idx = 0;
    tx_buf_overflow = false;
}

// Public Methods //////////////////////////////////////////////////////////////

void TwoWire::begin(void)
{
    begin(SDA, SCL);
}

/*
 * Joins I2C bus as master on given SDA and SCL pins.
 */
void TwoWire::begin(uint8_t sda, uint8_t scl) {
    port.sda = sda;
    port.scl = scl;
//    pinMode(scl, OUTPUT_OPEN_DRAIN);
//    pinMode(sda, OUTPUT_OPEN_DRAIN);
    pinMode(scl, OUTPUT);
    pinMode(sda, OUTPUT);
    digitalWrite(scl, HIGH);
    digitalWrite(sda, HIGH);
}

uint8_t TwoWire::requestFrom(uint8_t address, int num_bytes) {
    if (num_bytes > WIRE_BUFSIZ) num_bytes = WIRE_BUFSIZ;
    rx_buf_idx = 0;
    rx_buf_len = 0;
	i2c_start(port);
    //Send the address
    i2c_shift_out(port, (address << 1) | I2C_READ);
	//Check the ack on the address
    if (!i2c_get_ack(port)) 
	{
	i2c_stop(port);
	return ENACKADDR;
	}    //Start reading data one byte at a time.  
	while (rx_buf_len < (num_bytes-1)) {
	*(rx_buf + rx_buf_len) = i2c_shift_in(port);
	//Write_nac generates a propr ack response from the device (9th clock pulse)
	i2c_write_nack(port);
	rx_buf_len++;
	}
	
	*(rx_buf + rx_buf_len) = i2c_shift_in(port);
	//get_nac checks for a propr ack response from the device (9th clock pulse)
	i2c_get_ack(port);


	i2c_stop(port);
    return rx_buf_len;
}

uint8_t TwoWire::requestFrom(int address, int numBytes) {
    return TwoWire::requestFrom((uint8_t)address, (uint8_t) numBytes);
}

void TwoWire::beginTransmission(uint8_t slave_address) {
    tx_addr = slave_address;
    tx_buf_idx = 0;
    tx_buf_overflow = false;
    rx_buf_idx = 0;
    rx_buf_len = 0;
}

void TwoWire::beginTransmission(int slave_address) {
    beginTransmission((uint8_t)slave_address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to 
//	perform a repeated start. 
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(void) {
    if (tx_buf_overflow) return EDATA;

    i2c_start(port);

    i2c_shift_out(port, (tx_addr << 1) | I2C_WRITE);
    if (!i2c_get_ack(port)) 
  	{
        i2c_stop(port);
        return ENACKADDR;
    }
	
    // shift out the address we're transmitting to
    for (uint8_t i = 0; i < tx_buf_idx; i++) {
        uint8_t ret = writeOneByte(tx_buf[i]);
        if (ret) return ret;    // SUCCESS is 0
//w_printf("r[%x]", tx_buf[i]);
    }

    i2c_stop(port);

    tx_buf_idx = 0;
    tx_buf_overflow = false;
    return SUCCESS;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void TwoWire::write(uint8_t value) {
    if (tx_buf_idx == WIRE_BUFSIZ) {
        tx_buf_overflow = true;
        return;
    }

    tx_buf[tx_buf_idx++] = value;
}

void TwoWire::write(uint8_t* buf, int len) {
    for (uint8_t i = 0; i < len; i++) write(buf[i]);
}

void TwoWire::write(int value) {
    write((uint8_t)value);
}

void TwoWire::write(int* buf, int len) {
    write((uint8_t*)buf, (uint8_t)len);
}

void TwoWire::write(char* buf) {
    uint8_t *ptr = (uint8_t*)buf;
    while(*ptr) {
        write(*ptr);
        ptr++;
    }
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
uint8_t TwoWire::available() {
    return rx_buf_len - rx_buf_idx;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
uint8_t TwoWire::read() {
    if (rx_buf_idx == rx_buf_len) return 0;
    return rx_buf[rx_buf_idx++];
}

// private methods

uint8_t TwoWire::writeOneByte(uint8_t byte) {
    i2c_shift_out(port, byte);
    if (!i2c_get_ack(port)) 
    {
        i2c_stop(port);
        return ENACKTRNS;
    }
    //if (!i2c_get_ack(port)) return ENACKTRNS;

    return SUCCESS;
}

uint8_t TwoWire::readOneByte(uint8_t address, uint8_t *byte) {

    //Shift in toggles the clock line 8 times leaving it low checking the 
	//Date line on each clock cycle
    *byte = i2c_shift_in(port);
	//Write_nac generates a propr ack response from the device (9th clock pulse)
	i2c_write_nack(port);
	//i2c_stop(port);
    return SUCCESS;      // no real way of knowing, but be optimistic!
}




// Preinstantiate Objects //////////////////////////////////////////////////////

TwoWire Wire;






























#if 0
/*
 * TwoWire.h - TWI/I2C library for Arduino Zero
 * based on Copyright (c) 2011 Cristian Maglie <c.maglie@bug.st>.
 * Copyright (c) 2014 Arduino.
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

extern "C" {
#include <string.h>
}

#include "Wire.h"
#include "variant.h"
#include "wiring_digital.h"

TwoWire::TwoWire(SERCOM * s)
{
	this->sercom = s;
	transmissionBegun = false;
}

void TwoWire::begin(void) {
	//Master Mode
	sercom->initMasterWIRE(TWI_CLOCK);
	sercom->enableWIRE();

  pinPeripheral(PIN_WIRE_SDA, g_APinDescription[PIN_WIRE_SDA].ulPinType);
  pinPeripheral(PIN_WIRE_SCL, g_APinDescription[PIN_WIRE_SCL].ulPinType);
}

void TwoWire::begin(uint8_t address) {
	//Slave mode
	sercom->initSlaveWIRE(address);
	sercom->enableWIRE();
}

uint8_t TwoWire::requestFrom(uint8_t address, size_t quantity, bool stopBit)
{ 
  if(quantity == 0)
  {
    return 0;
  }


  size_t byteRead = 0;

  if(sercom->startTransmissionWIRE(address, WIRE_READ_FLAG))
  {
  
    // Read first data
    rxBuffer.store_char(sercom->readDataWIRE());

    // Connected to slave
    //while(toRead--)
    for(byteRead = 0; byteRead < quantity; ++byteRead)
    {
      if( byteRead == quantity - 1)  // Stop transmission
      {
        sercom->prepareNackBitWIRE(); // Prepare NACK to stop slave transmission
        //sercom->readDataWIRE(); // Clear data register to send NACK
        sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP); // Send Stop 
      }
      else // Continue transmission
      {
        sercom->prepareAckBitWIRE();  // Prepare Acknowledge
        sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_READ); // Prepare the ACK command for the slave
        rxBuffer.store_char( sercom->readDataWIRE() );  // Read data and send the ACK
      }
    }
  }

  return byteRead;
}

uint8_t TwoWire::requestFrom(uint8_t address, size_t quantity)
{
	return requestFrom(address, quantity, true);
}

void TwoWire::beginTransmission(uint8_t address) {
	// save address of target and clear buffer
	txAddress = address;
	txBuffer.clear();

	transmissionBegun = true;
}

// Errors:
//	0 : Success
//	1 : Data too long
//	2 : NACK on transmit of address
//	3 : NACK on transmit of data
//	4 : Other error
uint8_t TwoWire::endTransmission(bool stopBit)
{
	transmissionBegun = false ;

	// Check if there are data to send
	if ( txBuffer.available() == 0)
  {
		return 4 ;
  }

	// Start I2C transmission
	if ( !sercom->startTransmissionWIRE( txAddress, WIRE_WRITE_FLAG ) )
  {
    sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		return 2 ;	// Address error
  }

	// Send all buffer
	while( txBuffer.available() )
	{

		// Trying to send data
		if ( !sercom->sendDataMasterWIRE( txBuffer.read_char() ) )
    {
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
			return 3 ;	// Nack or error
    }

    
    if(txBuffer.available() == 0)
    {
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
    }
	}

	return 0;
}

uint8_t TwoWire::endTransmission()
{
	return endTransmission(true);
}

size_t TwoWire::write(uint8_t ucData)
{
	if(sercom->isMasterWIRE())
	{
		// No writing, without begun transmission or a full buffer
		if ( !transmissionBegun || txBuffer.isFull() )
    {
      return 0 ;
    }

		txBuffer.store_char( ucData ) ;

		return 1 ;
	}
	else
	{
		if(sercom->sendDataSlaveWIRE( ucData ))
    {
			return 1;
    }
	}

	return 0;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
	//Try to store all data
	for(size_t i = 0; i < quantity; ++i)
	{
		//Return the number of data stored, when the buffer is full (if write return 0)
		if(!write(data[i]))
			return i;
	}

	//All data stored
	return quantity;
}

int TwoWire::available(void)
{
	return rxBuffer.available();
}

int TwoWire::read(void)
{
	return rxBuffer.read_char();
}

int TwoWire::peek(void)
{
	return rxBuffer.peek();
}

void TwoWire::flush(void)
{
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}

void TwoWire::onReceive(void(*function)(int))
{
	onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void))
{
	onRequestCallback = function;
}


void TwoWire::onService(void)
{
	if ( sercom->isSlaveWIRE() )
	{
		//Received data
		if(sercom->isDataReadyWIRE())
		{
			//Store data
			rxBuffer.store_char(sercom->readDataWIRE());

			//Stop or Restart detected
			if(sercom->isStopDetectedWIRE() || sercom->isRestartDetectedWIRE())
			{
				//Calling onReceiveCallback, if exists
				if(onReceiveCallback)
				{
					onReceiveCallback(available());
				}
			}
		}

		//Address Match
		if(sercom->isAddressMatch())
		{
			//Is a request ?
			if(sercom->isMasterReadOperationWIRE())
			{
				//Calling onRequestCallback, if exists
				if(onRequestCallback)
				{
					onRequestCallback();
				}
			}
		}
	}
}

/*
void TwoWire::onService(void)
{
	// Retrieve interrupt status
	uint32_t sr = TWI_GetStatus(twi);

	if (status == SLAVE_IDLE && TWI_STATUS_SVACC(sr)) {
		TWI_DisableIt(twi, TWI_IDR_SVACC);
		TWI_EnableIt(twi, TWI_IER_RXRDY | TWI_IER_GACC | TWI_IER_NACK
				| TWI_IER_EOSACC | TWI_IER_SCL_WS | TWI_IER_TXCOMP);

		srvBufferLength = 0;
		srvBufferIndex = 0;

		// Detect if we should go into RECV or SEND status
		// SVREAD==1 means *master* reading -> SLAVE_SEND
		if (!TWI_STATUS_SVREAD(sr)) {
			status = SLAVE_RECV;
		} else {
			status = SLAVE_SEND;

			// Alert calling program to generate a response ASAP
			if (onRequestCallback)
				onRequestCallback();
			else
				// create a default 1-byte response
				write((uint8_t) 0);
		}
	}

	if (status != SLAVE_IDLE) {
		if (TWI_STATUS_TXCOMP(sr) && TWI_STATUS_EOSACC(sr)) {
			if (status == SLAVE_RECV && onReceiveCallback) {
				// Copy data into rxBuffer
				// (allows to receive another packet while the
				// user program reads actual data)
				for (uint8_t i = 0; i < srvBufferLength; ++i)
					rxBuffer[i] = srvBuffer[i];
				rxBufferIndex = 0;
				rxBufferLength = srvBufferLength;

				// Alert calling program
				onReceiveCallback( rxBufferLength);
			}

			// Transfer completed
			TWI_EnableIt(twi, TWI_SR_SVACC);
			TWI_DisableIt(twi, TWI_IDR_RXRDY | TWI_IDR_GACC | TWI_IDR_NACK
					| TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IER_TXCOMP);
			status = SLAVE_IDLE;
		}
	}

	if (status == SLAVE_RECV) {
		if (TWI_STATUS_RXRDY(sr)) {
			if (srvBufferLength < BUFFER_LENGTH)
				srvBuffer[srvBufferLength++] = TWI_ReadByte(twi);
		}
	}

	if (status == SLAVE_SEND) {
		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
			uint8_t c = 'x';
			if (srvBufferIndex < srvBufferLength)
				c = srvBuffer[srvBufferIndex++];
			TWI_WriteByte(twi, c);
		}
	}
}
*/

#if WIRE_INTERFACES_COUNT > 0
/*static void Wire_Init(void) {
	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SDA].pPort,
			g_APinDescription[PIN_WIRE_SDA].ulPinType,
			g_APinDescription[PIN_WIRE_SDA].ulPin,
			g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SCL].pPort,
			g_APinDescription[PIN_WIRE_SCL].ulPinType,
			g_APinDescription[PIN_WIRE_SCL].ulPin,
			g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);
	NVIC_SetPriority(WIRE_ISR_ID, 0);
	NVIC_EnableIRQ(WIRE_ISR_ID);
}*/


TwoWire Wire(&sercom3);

void SERCOM3_Handler(void) {
	Wire.onService();
}

#endif


#endif