#include "Uart.h"
#include "WVariant.h"
#include "wiring_digital.h"

Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX)
{
#if 0
	sercom = _s;
	uc_pinRX = _pinRX;
	uc_pinTX = _pinTX;
#endif
}

void Uart::begin(unsigned long baudrate)
{
#if 0
	begin(baudrate, (uint8_t)SERIAL_8N1);
#endif
}

void Uart::begin(unsigned long baudrate, uint8_t config)
{
#if 0
  pinPeripheral(uc_pinRX, g_APinDescription[uc_pinRX].ulPinType);
  pinPeripheral(uc_pinTX, g_APinDescription[uc_pinTX].ulPinType);

	sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
	sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
	sercom->initPads(UART_TX_PAD_2, SERCOM_RX_PAD_3);
	
	
	sercom->enableUART();
#endif
}

void Uart::end()
{
#if 0
	sercom->resetUART();
	rxBuffer.clear();
#endif
}

void Uart::flush()
{
#if 0
	sercom->flushUART();
#endif
}

void Uart::IrqHandler()
{
#if 0
	if(sercom->availableDataUART())
	{
		rxBuffer.store_char(sercom->readDataUART());
	}
	
	if(	sercom->isBufferOverflowErrorUART() ||
		sercom->isFrameErrorUART() ||
		sercom->isParityErrorUART())
	{
		sercom->clearStatusUART();
	}
#endif
}

int Uart::available()
{

  if( ((*(volatile unsigned long *) 0x4000D018) & (0x01 << 4)) ) 
  	return 0;
  else
  	return 1;

//	return rxBuffer.available();
}

int Uart::peek()
{
	return rxBuffer.peek();
}

int Uart::read()
{
	return (char) *(volatile unsigned long *) 0x4000D000;
//	return rxBuffer.read_char();
}

size_t Uart::write(const uint8_t data)
{
/////////////////////	sercom->writeDataUART(data);
  while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = data;

	return 1;
}

size_t Uart::write(const char * data)
{
	size_t writed = 0;
	
	while(*data != '\0')
	{
//    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
//    *(volatile unsigned long *) 0x4000D000 = *data;
		writed += write(*data);
		++data;
	}
	
	return writed;
}

SercomNumberStopBit Uart::extractNbStopBit(uint8_t config)
{
	switch(config & HARDSER_STOP_BIT_MASK)
	{
		case HARDSER_STOP_BIT_1:	
		default:
			return SERCOM_STOP_BIT_1;

		case HARDSER_STOP_BIT_2:	
			return SERCOM_STOP_BITS_2;
	}
}

SercomUartCharSize Uart::extractCharSize(uint8_t config)
{
	switch(config & HARDSER_DATA_MASK)
	{
		case HARDSER_DATA_5:
			return UART_CHAR_SIZE_5_BITS;

		case HARDSER_DATA_6:
			return UART_CHAR_SIZE_6_BITS;

		case HARDSER_DATA_7:
			return UART_CHAR_SIZE_7_BITS;

		case HARDSER_DATA_8:
		default:
			return UART_CHAR_SIZE_8_BITS;

	}
}

SercomParityMode Uart::extractParity(uint8_t config)
{
	switch(config & HARDSER_PARITY_MASK)
	{
		case HARDSER_PARITY_NONE:
		default:
			return SERCOM_NO_PARITY;

		case HARDSER_PARITY_EVEN:
			return SERCOM_EVEN_PARITY;

		case HARDSER_PARITY_ODD:
			return SERCOM_ODD_PARITY;
	}
}

void SERCOM0_Handler()
{
	Serial5.IrqHandler();
}

#if 0
void SERCOM5_Handler()
{
	Serial.IrqHandler();
}
#endif