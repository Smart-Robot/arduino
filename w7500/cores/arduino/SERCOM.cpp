#include "SERCOM.h"
#include "variant.h"

#if 0
SERCOM::SERCOM(Sercom* s)
{
	sercom = s;
}
#endif

/* 	=========================
 *	===== Sercom UART
 *	=========================
*/
void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{
#if 0
  resetUART();
  initClockNVIC();

  //Setting the CTRLA register
  sercom->USART.CTRLA.reg =	SERCOM_USART_CTRLA_MODE(mode) |
                SERCOM_USART_CTRLA_SAMPR(sampleRate);

  //Setting the Interrupt register
  sercom->USART.INTENSET.reg =	SERCOM_USART_INTENSET_RXC |  //Received complete
                                SERCOM_USART_INTENSET_ERROR; //All others errors

  if ( mode == UART_INT_CLOCK )
  {
    uint16_t sampleRateValue ;

    if ( sampleRate == SAMPLE_RATE_x16 )
    {
      sampleRateValue = 16 ;
    }
    else
    {
      if ( sampleRate == SAMPLE_RATE_x8 )
      {
        sampleRateValue = 8 ;
      }
      else
      {
        sampleRateValue = 3 ;
      }
    }

    // Asynchronous arithmetic mode
    // 65535 * ( 1 - sampleRateValue * baudrate / SystemCoreClock);
    // 65535 - 65535 * (sampleRateValue * baudrate / SystemCoreClock));
    sercom->USART.BAUD.reg = 65535.0f * ( 1.0f - (float)(sampleRateValue) * (float)(baudrate) / (float)(SystemCoreClock));
  }
#endif
}
void SERCOM::initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
#if 0
	//Setting the CTRLA register
	sercom->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_FORM( (parityMode == SERCOM_NO_PARITY ? 0 : 1) ) |
								dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

	//Setting the CTRLB register
	sercom->USART.CTRLB.reg |=	SERCOM_USART_CTRLB_CHSIZE(charSize) |
								nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
								(parityMode == SERCOM_NO_PARITY ? 0 : parityMode) << SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
#endif
}

void SERCOM::initPads(SercomUartTXPad txPad, SercomRXPad rxPad)
{
#if 0
	//Setting the CTRLA register
	sercom->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_TXPO(txPad) |
								SERCOM_USART_CTRLA_RXPO(rxPad);

  // Enable Transceiver and Receiver
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
#endif
}

void SERCOM::resetUART()
{
#if 0
  // Start the Software Reset
  sercom->USART.CTRLA.bit.SWRST = 1 ;

  while ( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
  {
    // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
  }
#endif
}

void SERCOM::enableUART()
{
#if 0
	//Setting  the enable bit to 1
	sercom->USART.CTRLA.bit.ENABLE = 0x1u;

	//Wait for then enable bit from SYNCBUSY is equal to 0;
	while(sercom->USART.SYNCBUSY.bit.ENABLE);
#endif
}

void SERCOM::flushUART()
{
#if 0
	// Wait for transmission to complete
	while(sercom->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE);
#endif
}

void SERCOM::clearStatusUART()
{
#if 0
	//Reset (with 0) the STATUS register
	sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
#endif
}

bool SERCOM::availableDataUART()
{
#if 0
	//RXC : Receive Complete
	return sercom->USART.INTFLAG.bit.RXC;
#endif
	return 1;
}

bool SERCOM::isBufferOverflowErrorUART()
{
#if 0
	//BUFOVF : Buffer Overflow
	return sercom->USART.STATUS.bit.BUFOVF;
#endif
	return 1;
}

bool SERCOM::isFrameErrorUART()
{
#if 0
	//FERR : Frame Error
	return sercom->USART.STATUS.bit.FERR;
#endif
	return 1;
}

bool SERCOM::isParityErrorUART()
{
#if 0
	//PERR : Parity Error
	return sercom->USART.STATUS.bit.PERR;
#endif
	return 1;
}

bool SERCOM::isDataRegisterEmptyUART()
{
#if 0
	//DRE : Data Register Empty
	return sercom->USART.INTFLAG.bit.DRE;
#endif
	return 1;
}

uint8_t SERCOM::readDataUART()
{
#if 0
	return sercom->USART.DATA.bit.DATA;
#endif
	return 1;
}

int SERCOM::writeDataUART(uint8_t data)
{
#if 0
	//Flush UART buffer
	flushUART();

	//Put data into DATA register
	sercom->USART.DATA.reg = (uint16_t)data;
	return 1;
#endif
	return 1;
}

/*	=========================
 *	===== Sercom SPI
 *	=========================
*/
void SERCOM::initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder)
{
#if 0
	resetSPI();
	initClockNVIC();

	//Setting the CTRLA register
	sercom->SPI.CTRLA.reg =	SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
							            SERCOM_SPI_CTRLA_DOPO(mosi) |
							            SERCOM_SPI_CTRLA_DIPO(miso) |
							            dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;

	//Setting the CTRLB register
	sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
							            SERCOM_SPI_CTRLB_RXEN;	//Active the SPI receiver.


#endif
}

void SERCOM::initSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate)
{
#if 0
	//Extract data from clockMode
	int cpha, cpol;

	if((clockMode & (0x1ul)) == 0 )
		cpha = 0;
	else
		cpha = 1;

	if((clockMode & (0x2ul)) == 0)
		cpol = 0;
	else
		cpol = 1;

	//Setting the CTRLA register
	sercom->SPI.CTRLA.reg |=	( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
								            ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

	//Synchronous arithmetic
	sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
#endif
}

void SERCOM::resetSPI()
{
#if 0
	//Setting the Software Reset bit to 1
	sercom->SPI.CTRLA.bit.SWRST = 1;

	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while(sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
#endif
}

void SERCOM::enableSPI()
{
#if 0
	//Setting the enable bit to 1
	sercom->SPI.CTRLA.bit.ENABLE = 1;

	while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
#endif
}

void SERCOM::disableSPI()
{
#if 0
	//Setting the enable bit to 0
	sercom->SPI.CTRLA.bit.ENABLE = 0;

	while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
#endif
}

void SERCOM::setDataOrderSPI(SercomDataOrder dataOrder)
{
#if 0
	//Register enable-protected
	disableSPI();

	sercom->SPI.CTRLA.bit.DORD = dataOrder;

	enableSPI();
#endif
}

void SERCOM::setBaudrateSPI(uint8_t divider)
{
#if 0
	//Can't divide by 0
	if(divider == 0)
		return;

	//Register enable-protected
	disableSPI();

	sercom->SPI.BAUD.reg = calculateBaudrateSynchronous( SERCOM_FREQ_REF / divider );

	enableSPI();
#endif
}

void SERCOM::setClockModeSPI(SercomSpiClockMode clockMode)
{
#if 0
	int cpha, cpol;
	if((clockMode & (0x1ul)) == 0)
		cpha = 0;
	else
		cpha = 1;

	if((clockMode & (0x2ul)) == 0)
		cpol = 0;
	else
		cpol = 1;

	//Register enable-protected
	disableSPI();

	sercom->SPI.CTRLA.bit.CPOL = cpol;
	sercom->SPI.CTRLA.bit.CPHA = cpha;

	enableSPI();
#endif
}
void SERCOM::writeDataSPI(uint8_t data)
{
#if 0
  while( sercom->SPI.INTFLAG.bit.DRE == 0 )
  {
    // Waiting Data Registry Empty
  }

	sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while( sercom->SPI.INTFLAG.bit.TXC == 0 || sercom->SPI.INTFLAG.bit.DRE == 0 )
  {
    // Waiting Complete Transmission
  }
#endif
}

uint16_t SERCOM::readDataSPI()
{
#if 0
  while( sercom->SPI.INTFLAG.bit.DRE == 0 || sercom->SPI.INTFLAG.bit.RXC == 0 )
  {
    // Waiting Complete Reception
  }

	return sercom->SPI.DATA.bit.DATA;  // Reading data
#endif
	return 1;
}

bool SERCOM::isBufferOverflowErrorSPI()
{
#if 0
	return sercom->SPI.STATUS.bit.BUFOVF;
#endif
	return 1;
}

bool SERCOM::isDataRegisterEmptySPI()
{
#if 0
	//DRE : Data Register Empty
	return sercom->SPI.INTFLAG.bit.DRE;
#endif
	return 1;
}

uint8_t SERCOM::calculateBaudrateSynchronous(uint32_t baudrate)
{
#if 0
	return SERCOM_FREQ_REF / (2 * baudrate) - 1;
#endif
	return 1;
}


/*	=========================
 *	===== Sercom WIRE
 *	=========================
*/

void SERCOM::resetWIRE()
{
#if 0
	//I2CM OR I2CS, no matter SWRST is the same bit.

	//Setting the Software bit to 1
	sercom->I2CM.CTRLA.bit.SWRST = 1;

	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while(sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);
#endif
}

void SERCOM::enableWIRE()
{
#if 0
  // I2C Master and Slave modes share the ENABLE bit function.

  // Enable the I�C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }

  // Setting bus idle mode
  sercom->I2CM.STATUS.bit.BUSSTATE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY coming back to 0
  }
#endif
}

void SERCOM::disableWIRE()
{
#if 0
  // I2C Master and Slave modes share the ENABLE bit function.

  // Enable the I�C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 0 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }
#endif
}

void SERCOM::initSlaveWIRE( uint8_t ucAddress )
{
#if 0
  // Initialize the peripheral clock and interruption
  initClockNVIC() ;
  resetWIRE() ;

  // Set slave mode
  sercom->I2CS.CTRLA.bit.MODE = I2C_SLAVE_OPERATION ;

  // Enable Quick Command
  sercom->I2CM.CTRLB.bit.QCEN = 1 ;

  sercom->I2CS.ADDR.reg = SERCOM_I2CS_ADDR_ADDR( ucAddress & 0x7Ful ) | // 0x7F, select only 7 bits
                          SERCOM_I2CS_ADDR_ADDRMASK( 0x3FFul ) ;        // 0x3FF all bits set

  // Set the interrupt register
  sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH |             // Address Match
                              SERCOM_I2CS_INTENSET_DRDY ;               // Data Ready

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY to come back to 0
  }
#endif
}

void SERCOM::initMasterWIRE( uint32_t baudrate )
{
#if 0
  // Initialize the peripheral clock and interruption
  initClockNVIC() ;

  resetWIRE() ;

  // Set master mode and enable SCL Clock Stretch mode (stretch after ACK bit)
  sercom->I2CM.CTRLA.reg =  SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION );

  // Enable Smart mode and Quick Command

  // Enable all interrupts

  // Synchronous arithmetic baudrate
  sercom->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * baudrate) - 1 ;
#endif
}

void SERCOM::prepareNackBitWIRE( void )
{
#if 0
  // Send a NACK
  sercom->I2CM.CTRLB.bit.ACKACT = 1;
#endif
}

void SERCOM::prepareAckBitWIRE( void )
{
#if 0
  // Send an ACK
  sercom->I2CM.CTRLB.bit.ACKACT = 0;
#endif
}

void SERCOM::prepareCommandBitsWire(SercomMasterCommandWire cmd)
{
#if 0
  sercom->I2CM.CTRLB.bit.CMD = cmd;

  while(sercom->I2CM.SYNCBUSY.bit.SYSOP)
  {
    // Waiting for synchronization
  }
#endif
}

bool SERCOM::startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag)
{
#if 0
  // 7-bits address + 1-bits R/W
  address = (address << 0x1ul) | flag;

  // Wait idle bus mode
  while ( !isBusIdleWIRE() );

  // Send start and address
  sercom->I2CM.ADDR.bit.ADDR = address;

  // Address Transmitted
  if ( flag == WIRE_WRITE_FLAG ) // Write mode
  {
    while( !sercom->I2CM.INTFLAG.bit.MB )
    {
      // Wait transmission complete
    }
  }
  else  // Read mode
  {
    while( !sercom->I2CM.INTFLAG.bit.SB )
    {
      // Wait transmission complete
    }

    // Clean the 'Slave on Bus' flag, for further usage.
    //sercom->I2CM.INTFLAG.bit.SB = 0x1ul;
  }


  //ACK received (0: ACK, 1: NACK)
  if(sercom->I2CM.STATUS.bit.RXNACK)
  {
    return false;
  }
  else
  {
    return true;
  }
#endif
}

bool SERCOM::sendDataMasterWIRE(uint8_t data)
{
#if 0
	//Send data
	sercom->I2CM.DATA.bit.DATA = data;

	//Wait transmission successful
	while(!sercom->I2CM.INTFLAG.bit.MB);

	//Problems on line? nack received?
	if(sercom->I2CM.STATUS.bit.RXNACK)
		return false;
	else
		return true;
#endif
	return 1;
}

bool SERCOM::sendDataSlaveWIRE(uint8_t data)
{
#if 0
	//Send data
	sercom->I2CS.DATA.bit.DATA = data;

	//Wait data transmission successful
	while(!sercom->I2CS.INTFLAG.bit.DRDY);

	//Problems on line? nack received?
	if(sercom->I2CS.STATUS.bit.RXNACK)
		return false;
	else
		return true;
#endif
	return 1;
}

bool SERCOM::isMasterWIRE( void )
{
#if 0
	return sercom->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION;
#endif
	return 1;
}

bool SERCOM::isSlaveWIRE( void )
{
#if 0
	return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
#endif
	return 1;
}

bool SERCOM::isBusIdleWIRE( void )
{
#if 0
	return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
#endif
	return 1;
}

bool SERCOM::isDataReadyWIRE( void )
{
#if 0
	return sercom->I2CS.INTFLAG.bit.DRDY;
#endif
	return 1;
}

bool SERCOM::isStopDetectedWIRE( void )
{
#if 0
	return sercom->I2CS.INTFLAG.bit.PREC;
#endif
	return 1;
}

bool SERCOM::isRestartDetectedWIRE( void )
{
#if 0
	return sercom->I2CS.STATUS.bit.SR;
#endif
	return 1;
}

bool SERCOM::isAddressMatch( void )
{
#if 0
	return sercom->I2CS.INTFLAG.bit.AMATCH;
#endif
	return 1;
}

bool SERCOM::isMasterReadOperationWIRE( void )
{
#if 0
  return sercom->I2CS.STATUS.bit.DIR;
#endif
	return 1;
}

bool SERCOM::isRXNackReceivedWIRE( void )
{
#if 0
  return sercom->I2CM.STATUS.bit.RXNACK;
#endif
	return 1;
}

int SERCOM::availableWIRE( void )
{
#if 0
	if(isMasterWIRE())
		return sercom->I2CM.INTFLAG.bit.SB;
	else
		return sercom->I2CS.INTFLAG.bit.DRDY;
#endif
	return 1;
}

uint8_t SERCOM::readDataWIRE( void )
{
#if 0
  if(isMasterWIRE())
  {
    while( sercom->I2CM.INTFLAG.bit.SB == 0 )
    {
      // Waiting complete receive
    }

    return sercom->I2CM.DATA.bit.DATA ;
  }
  else
  {
    return sercom->I2CS.DATA.reg ;
  }
#endif
	return 1;
}


void SERCOM::initClockNVIC( void )
{
#if 0
	uint8_t clockId = 0;
	IRQn_Type IdNvic;

	if(sercom == SERCOM0)
	{
		clockId = GCM_SERCOM0_CORE;
		IdNvic = SERCOM0_IRQn;
	}
	else if(sercom == SERCOM1)
	{
		clockId = GCM_SERCOM1_CORE;
		IdNvic = SERCOM1_IRQn;
	}
	else if(sercom == SERCOM2)
	{
		clockId = GCM_SERCOM2_CORE;
		IdNvic = SERCOM2_IRQn;
	}
	else if(sercom == SERCOM3)
	{
		clockId = GCM_SERCOM3_CORE;
		IdNvic = SERCOM3_IRQn;
	}
	else if(sercom == SERCOM4)
	{
		clockId = GCM_SERCOM4_CORE;
		IdNvic = SERCOM4_IRQn;
	}
	else if(sercom == SERCOM5)
	{
		clockId = GCM_SERCOM5_CORE;
		IdNvic = SERCOM5_IRQn;
	}

	// Setting NVIC
	NVIC_EnableIRQ(IdNvic);
	NVIC_SetPriority (IdNvic, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */

	//Setting clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) |       // Generic Clock 0 (SERCOMx)
						          GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
						          GCLK_CLKCTRL_CLKEN ;

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}
#endif
}
