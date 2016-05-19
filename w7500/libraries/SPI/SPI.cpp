/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"

void SSP_SendData_8bit_CH0(uint8_t Data)
{
    /* Write in the DR register the data to be sent */
//    *(volatile unsigned int *)0x4000A008 = Data;              // SSPx->DR = Data; 
    *(volatile unsigned int *)0x4000B008 = Data;              // SSPx->DR = Data; 
}

uint8_t SSP_ReceiveData_8bit_CH0(void)
{
    /* Return the data in the DR register */
//    return  *(volatile unsigned int *)0x4000A008;      //return SSPx->DR;
    return  *(volatile unsigned int *)0x4000B008;      //return SSPx->DR;
}

uint32_t SSP_GetTxFlagStatus_CH0(void)
{
    /* Check the status of the specified SSP/I2S flag */
//    if (*(volatile unsigned int *)(0x4000A00C) & (0x10)/*SSP_FLAG_BSY*/)     // if ((SSPx->SR & SSP_FLAG))
    if (*(volatile unsigned int *)(0x4000B00C) & (0x10)/*SSP_FLAG_BSY*/)     // if ((SSPx->SR & SSP_FLAG))
    {
        return  1;
    }
    else
    {
        return  0;
    }
}

uint32_t SSP_GetRxFlagStatus_CH0(void)
{
    /* Check the status of the specified SSP/I2S flag */
//    if (*(volatile unsigned int *)(0x4000A00C) & (0x04)/*SSP_FLAG_RNE*/)     // if ((SSPx->SR & SSP_FLAG))
    if (*(volatile unsigned int *)(0x4000B00C) & (0x04)/*SSP_FLAG_RNE*/)     // if ((SSPx->SR & SSP_FLAG))
    {
        return  1;
    }
    else
    {
        return  0;
    }
}


SPIClass SPI;

void SPIClass::begin()
{
    unsigned int tmpreg = 0;

    /*---------------------------- SSPx CR0 Configuration ------------------------*/
    /* Get the SSPx CR0 value */
//    tmpreg = *(volatile unsigned int *)(0x4000A000);     // tmpreg = SSPx->CR0;
    tmpreg = *(volatile unsigned int *)(0x4000B000);     // tmpreg = SSPx->CR0;
    /* Clear  bits */
    //tmpreg &= CR1_CLEAR_Mask;
    tmpreg |= ( (0x00 << 8) | (0x00000000 << 4)/*SSP_FrameFormat_MO*/ | (0x00 << 7)/*SSP_CPHA_1Edge*/ | (0x00 << 6)/*SSP_CPOL_Low*/ | (0x7 << 0)/*SSP_DataSize_8b*/ );
    
    //printf("CR0: %.8X \r\n",tmpreg);
//    *(volatile unsigned int *)(0x4000A000) = tmpreg;      // SSPx->CR0 = tmpreg; 
    *(volatile unsigned int *)(0x4000B000) = tmpreg;      // SSPx->CR0 = tmpreg; 

    /*---------------------------- SSPx CR1 Configuration ------------------------*/
    /* Write to SSPx CR1 */
//    tmpreg = *(volatile unsigned int *)(0x4000A004);      // tmpreg = SSPx->CR1;
    tmpreg = *(volatile unsigned int *)(0x4000B004);      // tmpreg = SSPx->CR1;
    /* Clear  bits */
    //tmpreg &= CR1_CLEAR_Mask;
    tmpreg |= ( (0x00 << 3)/*SSP_SOD_RESET*/ | (0x00 << 2)/*SSP_Mode_Master*/ /*| (0x02)SSP_NSS_Hard*/ | (0x01 << 1)/*SSP_SSE_SET*/ | (0x00 << 0)/*SSP_LBM_RESET*/ );
//    *(volatile unsigned int *)(0x4000A004) = tmpreg;      //  SSPx->CR1 = tmpreg;
    *(volatile unsigned int *)(0x4000B004) = tmpreg;      //  SSPx->CR1 = tmpreg;
    //printf("CR1: %.8X \r\n",tmpreg);
    /*---------------------------- SSPx Clock prescal register ------------------------*/
//    *(volatile unsigned int *)(0x4000A010) = (0x0008)/*SSP_BaudRatePrescaler_4*/;   // SSPx->CPSR = SSP_InitStruct->SSP_BaudRatePrescaler; 
    *(volatile unsigned int *)(0x4000B010) = (0x0008)/*SSP_BaudRatePrescaler_4*/;   // SSPx->CPSR = SSP_InitStruct->SSP_BaudRatePrescaler; 
}

void SPIClass::end() {

}























void SSP_SendData_8bit_CH1(unsigned char Data)
{

    /* Write in the DR register the data to be sent */
    *(volatile unsigned int *)0x4000B008 = Data;              // SSPx->DR = Data; 
}

unsigned short SSP_ReceiveData_8bit_CH1(void)
{
    /* Check the parameters */
//    assert_param(IS_SSP_ALL_PERIPH(SSPx));

    /* Return the data in the DR register */
    return  *(volatile unsigned int *)0x4000B008;      //return SSPx->DR;
}

unsigned int SSP_GetTxFlagStatus_CH1(void)
{
//    FlagStatus bitstatus = RESET;
    /* Check the parameters */
//    assert_param(IS_SSP_ALL_PERIPH(SSPx));
//    assert_param(IS_SSP_GET_FLAG(SSP_FLAG));
    /* Check the status of the specified SSP/I2S flag */
    if (*(volatile unsigned int *)(0x4000B00C) & (0x10)/*SSP_FLAG_BSY*/)     // if ((SSPx->SR & SSP_FLAG))
    {
        /* SSP_FLAG is set */
//        bitstatus = SET;
        return  1;
    }
    else
    {
        /* SSP__FLAG is reset */
//        bitstatus = RESET;
        return  0;
    }
    /* Return the SSP_I2S_FLAG status */
//    return  bitstatus;
}

unsigned int SSP_GetRxFlagStatus_CH1(void)
{
//    FlagStatus bitstatus = RESET;
    /* Check the parameters */
//    assert_param(IS_SSP_ALL_PERIPH(SSPx));
//    assert_param(IS_SSP_GET_FLAG(SSP_FLAG));
    /* Check the status of the specified SSP/I2S flag */
    if (*(volatile unsigned int *)(0x4000B00C) & (0x04)/*SSP_FLAG_RNE*/)     // if ((SSPx->SR & SSP_FLAG))
    {
        /* SSP_FLAG is set */
//        bitstatus = SET;
        return  1;
    }
    else
    {
        /* SSP__FLAG is reset */
//        bitstatus = RESET;
        return  0;
    }
    /* Return the SSP_I2S_FLAG status */
//    return  bitstatus;
}









SPIClass1 SPI1;

void SPIClass1::begin()
{
    unsigned int tmpreg = 0;

    /*---------------------------- SSPx CR0 Configuration ------------------------*/
    /* Get the SSPx CR0 value */
    tmpreg = *(volatile unsigned int *)(0x4000B000);     //tmpreg = SSPx->CR0; 
    /* Clear  bits */
    //tmpreg &= CR1_CLEAR_Mask;
    tmpreg |= ( 0x00 | (0x00000000)/*SSP_FrameFormat_MO*/ | (0x00)/*SSP_CPHA_1Edge*/ | (0x00)/*SSP_CPOL_Low*/ | (0x7)/*SSP_DataSize_8b*/ );
    
    //printf("CR0: %.8X \r\n",tmpreg);
    /* Write to SSPx CR0 */
    *(volatile unsigned int *)(0x4000B000) = tmpreg;      //SSPx->CR0 = tmpreg;

    /*---------------------------- SSPx CR1 Configuration ------------------------*/
    /* Write to SSPx CR1 */
    tmpreg = *(volatile unsigned int *)(0x4000B004);       //tmpreg = SSPx->CR1;
    /* Clear  bits */
    //tmpreg &= CR1_CLEAR_Mask;
    //tmpreg |= (uint32_t)(SSP_InitStruct->SSP_SOD | SSP_InitStruct->SSP_Mode |
    //        SSP_InitStruct->SSP_NSS | SSP_InitStruct->SSP_SSE | SSP_InitStruct->SSP_LBM );
    tmpreg |= ( (0x00)/*SSP_SOD_RESET*/ | (0x04)/*SSP_Mode_Slave*/ | (0x02)/*SSP_NSS_Hard*/ | (0x02)/*SSP_SSE_SET*/ | (0x00)/*SSP_LBM_RESET*/ );
    *(volatile unsigned int *)(0x4000B004) = tmpreg;      //SSPx->CR1 = tmpreg;  
//    printf("CR1: %.8X \r\n", tmpreg);
////    printf("CR1: %x \n", tmpreg);
    /*---------------------------- SSPx Clock prescal register ------------------------*/
    *(volatile unsigned int *)(0x4000B010) = (0x0008)/*SSP_BaudRatePrescaler_4*/;   // SSPx->CPSR = SSP_InitStruct->SSP_BaudRatePrescaler;
}

void SPIClass1::end() {

}






































#if 0
/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"
#include "wiring_digital.h"
#include "assert.h"
#include "variant.h"

SPIClass::SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI)
{
	assert(p_sercom != NULL );
	_p_sercom = p_sercom;

  _uc_pinMiso = uc_pinMISO;
  _uc_pinSCK = uc_pinSCK;
  _uc_pinMosi = uc_pinMOSI;
}

void SPIClass::begin()
{
  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

	// Default speed set to 4Mhz, SPI mode set to MODE 0 and Bit order set to MSB first.
	_p_sercom->initSPI(SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
	_p_sercom->initSPIClock(SERCOM_SPI_MODE_0, 4000000);
	
	_p_sercom->enableSPI();
}

void SPIClass::end()
{
	_p_sercom->resetSPI();
}

void SPIClass::setBitOrder(BitOrder order)
{
	if(order == LSBFIRST)
		_p_sercom->setDataOrderSPI(LSB_FIRST);
	else
		_p_sercom->setDataOrderSPI(MSB_FIRST);
}

void SPIClass::setDataMode(uint8_t mode)
{
	switch(mode)
	{
		case SPI_MODE0:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_0);
			break;
			
		case SPI_MODE1:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_1);
			break;
			
		case SPI_MODE2:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_2);
			break;
			
		case SPI_MODE3:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_3);
			break;
		
		default:
			break;
	}
}

void SPIClass::setClockDivider(uint8_t div)
{
	_p_sercom->setBaudrateSPI(div);
}

byte SPIClass::transfer(uint8_t data)
{
	//Writing the data
	_p_sercom->writeDataSPI(data);
	
	//Read data
	return _p_sercom->readDataSPI();
}

void SPIClass::attachInterrupt() {
	// Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
	// Should be disableInterrupt()
}

SPIClass SPI(&sercom4, 18, 20, 21);
#endif