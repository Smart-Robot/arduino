/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#include "WInterrupts.h"
#include "variant.h"
#include "wiring_digital.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

static struct
{
  uint32_t _ulPin ;
  voidFuncPtr _callback ;
} callbacksInt[EXTERNAL_NUM_INTERRUPTS] ;

//////// Ãß°¡ µÊ
static int_pin_tbl[4][16]={0, };
//////// Ãß°¡ µÊ

/* Configure I/O interrupt sources */
static void __initialize()
{
#if 0  // Ãß°¡

#if 0
  int i ;

  for ( i = 0 ; i < EXTERNAL_NUM_INTERRUPTS ; i++ )
  {
    callbacksInt[i]._callback = NULL ;
  }
#else
  memset( callbacksInt, 0, sizeof( callbacksInt ) ) ;
#endif

  NVIC_DisableIRQ( EIC_IRQn ) ;
  NVIC_ClearPendingIRQ( EIC_IRQn ) ;
  NVIC_SetPriority( EIC_IRQn, 0 ) ;
  NVIC_EnableIRQ( EIC_IRQn ) ;



  // Enable GCLK for IEC (External Interrupt Controller)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_EIC )) ;

  // Enable EIC
  EIC->CTRL.bit.ENABLE = 1 ;

  while ( EIC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for synchronisation
  }

#endif  // Ãß°¡ ³¡
}


/*
 * \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 *        Replaces any previous function that was attached to the interrupt.
 */

void attachInterrupt( uint32_t ulPin, voidFuncPtr callback, uint32_t ulMode )
{
#if 1
    // input mode·Î ¼³Á¤
    *(volatile unsigned int *)(0x42000014 + digital_pin_tbl[ulPin].port) |= 1 << digital_pin_tbl[ulPin].pin_num;   //GPIOx->OUTENSET

    // port ; Base 0x42000000  a 0x00000000  b 0x01000000  c 0x02000000  d 0x03000000
    // af_port : Base 0x41002000  a 0x00  b 0x40  c 0x80  d 0xC0
    *(volatile unsigned int *)(0x42000020 + digital_pin_tbl[ulPin].port) |= 1 << digital_pin_tbl[ulPin].pin_num;   //GPIOx->INTENSET
    *(volatile unsigned int *)(0x42000028 + digital_pin_tbl[ulPin].port) |= 1 << digital_pin_tbl[ulPin].pin_num;   //GPIOx->INTYPESET
    
    // INT Polarity setting 
    switch ( ulMode )
    {
      case LOW:
        ;//EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_LOW_Val << ulPos ;
      break ;

      case HIGH:
        ;//EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_HIGH_Val << ulPos ;
      break ;

      case CHANGE:
        ;//EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_BOTH_Val << ulPos ;
      break ;

      case FALLING:
        *(volatile unsigned int *)(0x42000030 + digital_pin_tbl[ulPin].port) |= 1 << digital_pin_tbl[ulPin].pin_num;   //GPIOx->INTPOLSET
      break ;

      case RISING:
        *(volatile unsigned int *)(0x42000034 + digital_pin_tbl[ulPin].port) |= 1 << digital_pin_tbl[ulPin].pin_num;   //GPIOx->INTPOLCLR
      break ;
    }

    // PAD setting
    *(volatile unsigned int *)(0x41002000 + digital_pin_tbl[ulPin].af_port + digital_pin_tbl[ulPin].pin_num *4) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002000 + digital_pin_tbl[ulPin].af_port + digital_pin_tbl[ulPin].pin_num *4) |= 0x01/*PAD_AF1*/;	// (0x01 <<  8)

    // Interrupt Enable
    *(volatile unsigned int *)0xE000E100 = (1 << ((uint32_t)(7 + (digital_pin_tbl[ulPin].af_port >> 6)) & 0x1F));        // 7 -> a port, 8 -> b port, 9 -> c port, 10 -> d port


  // Assign callback to interrupt
  int_pin_tbl[digital_pin_tbl[ulPin].af_port >> 6][digital_pin_tbl[ulPin].pin_num] = ulPin;
  callbacksInt[ulPin]._ulPin = ulPin ;
  callbacksInt[ulPin]._callback = callback ;

//  callbacksInt[digitalPinToInterrupt( ulPin )]._ulPin = ulPin ;
//  callbacksInt[digitalPinToInterrupt( ulPin )]._callback = callback ;
#endif

#if 0
  static int enabled = 0 ;
  uint32_t ulConfig ;
  uint32_t ulPos ;

  if ( digitalPinToInterrupt( ulPin ) == NOT_AN_INTERRUPT )
  {
    return ;
  }

  if ( !enabled )
  {
    __initialize() ;
    enabled = 1 ;
  }

  // Assign pin to EIC
  pinPeripheral( ulPin, PIO_EXTINT ) ;

  // Assign callback to interrupt
  callbacksInt[digitalPinToInterrupt( ulPin )]._ulPin = ulPin ;
  callbacksInt[digitalPinToInterrupt( ulPin )]._callback = callback ;

  // Check if normal interrupt or NMI
  if ( ulPin != 2 )
  {
    // Look for right CONFIG register to be addressed
    if ( digitalPinToInterrupt( ulPin ) > EXTERNAL_INT_7 )
    {
      ulConfig = 1 ;
    }
    else
    {
      ulConfig = 0 ;
    }

    // Configure the interrupt mode
    ulPos = ((digitalPinToInterrupt( ulPin ) - (8*ulConfig) ) << 2) ;
    switch ( ulMode )
    {
      case LOW:
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_LOW_Val << ulPos ;
      break ;

      case HIGH:
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_HIGH_Val << ulPos ;
      break ;

      case CHANGE:
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_BOTH_Val << ulPos ;
      break ;

      case FALLING:
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_FALL_Val << ulPos ;
      break ;

      case RISING:
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_RISE_Val << ulPos ;
      break ;
    }

    // Enable the interrupt
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT( 1 << digitalPinToInterrupt( ulPin ) ) ;
  }
  else // Handles NMI on pin 2
  {
    // Configure the interrupt mode
    switch ( ulMode )
    {
      case LOW:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_LOW ;
      break ;

      case HIGH:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_HIGH ;
      break ;

      case CHANGE:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_BOTH ;
      break ;

      case FALLING:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_FALL ;
      break ;

      case RISING:
        EIC->NMICTRL.reg= EIC_NMICTRL_NMISENSE_RISE ;
      break ;
    }

    // Enable the interrupt
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT( 1 << digitalPinToInterrupt( ulPin ) ) ;
  }
#endif
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt( uint32_t ulPin )
{

#if 0
/*
  // Retrieve pin information
  Pio *pio = g_APinDescription[pin].pPort;
  uint32_t mask = g_APinDescription[pin].ulPin;

  // Disable interrupt
  pio->PIO_IDR = mask;
*/
  if ( digitalPinToInterrupt( ulPin ) == NOT_AN_INTERRUPT )
  {
    return ;
  }

  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT( 1 << digitalPinToInterrupt( ulPin ) ) ;
#endif 
}

/*
 * External Interrupt Controller NVIC Interrupt Handler
 */
void EIC_Handler( void )
{
#if 0
  uint32_t ul ;

  // Test NMI first
  if ( (EIC->NMIFLAG.reg & EIC_NMIFLAG_NMI) == EIC_NMIFLAG_NMI )
  {
    // Call the callback function if assigned
    if ( callbacksInt[EXTERNAL_INT_NMI]._callback != NULL )
    {
      callbacksInt[EXTERNAL_INT_NMI]._callback() ;
    }

    // Clear the interrupt
    EIC->NMIFLAG.reg = EIC_NMIFLAG_NMI ;
  }

  // Test the 16 normal interrupts
  for ( ul = EXTERNAL_INT_0 ; ul < EXTERNAL_INT_NMI ; ul++ )
  {
    if ( (EIC->INTFLAG.reg & ( 1 << ul ) ) != 0 )
    {
      // Call the callback function if assigned
      if ( callbacksInt[ul]._callback != NULL )
      {
        callbacksInt[ul]._callback() ;
      }

      // Clear the interrupt
      EIC->INTFLAG.reg = 1 << ul ;
    }
  }
#endif
}

////////////// Ãß°¡µÊ
/**
  * @brief  This function handles PORT0 Handler.
  * @param  None
  * @retval None
  */
void PORT0_Handler(void)
{
    unsigned int port0_int_status = 0;
    int i ;
    port0_int_status = (*((volatile unsigned int *)(0x42000000 + 0x38)));                  // D port UARTx->INTSTATUS/INTCLEAR  

  for ( i = 0 ; i < EXTERNAL_NUM_INTERRUPTS ; i++ )
  {    
    if ( port0_int_status & (1 << i))
    {
        if ( callbacksInt[int_pin_tbl[0][i]]._callback != NULL )
        {
            callbacksInt[int_pin_tbl[0][i]]._callback() ;
        }
        (*((volatile unsigned int *)(0x42000000 + 0x38)))  |= 1 << i;                  // C port UARTx->INTSTATUS/INTCLEAR  
//        (*((volatile unsigned int *)(0x44000000 + 0x24)))  |= 0xffff;                  // C port UARTx->INTENCLR  
//        (*((volatile unsigned int *)(0x44000000 + 0x2C)))  |= 0xffff;                  // C port UARTx->INTTYPECLR  
    }
  }
#if 1
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'A';                           // UARTx->DR
#endif
}


/**
  * @brief  This function handles PORT1 Handler.
  * @param  None
  * @retval None
  */
void PORT1_Handler(void)
{
    unsigned int port1_int_status = 0;
    int i ;
    port1_int_status = (*((volatile unsigned int *)(0x43000000 + 0x38)));                  // D port UARTx->INTSTATUS/INTCLEAR  

  for ( i = 0 ; i < EXTERNAL_NUM_INTERRUPTS ; i++ )
  {    
    if ( port1_int_status & (1 << i))
    {
        if ( callbacksInt[int_pin_tbl[1][i]]._callback != NULL )
        {
            callbacksInt[int_pin_tbl[1][i]]._callback() ;
        }
        (*((volatile unsigned int *)(0x43000000 + 0x38)))  |= 1 << i;                  // C port UARTx->INTSTATUS/INTCLEAR  
//        (*((volatile unsigned int *)(0x44000000 + 0x24)))  |= 0xffff;                  // C port UARTx->INTENCLR  
//        (*((volatile unsigned int *)(0x44000000 + 0x2C)))  |= 0xffff;                  // C port UARTx->INTTYPECLR  
    }
  }
#if 1
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'D';                           // UARTx->DR
#endif
}


/**
  * @brief  This function handles PORT2 Handler.
  * @param  None
  * @retval None
  */
void PORT2_Handler(void)
{
    unsigned int port2_int_status = 0;
    int i ;
    port2_int_status = (*((volatile unsigned int *)(0x44000000 + 0x38)));                  // D port UARTx->INTSTATUS/INTCLEAR  

  for ( i = 0 ; i < EXTERNAL_NUM_INTERRUPTS ; i++ )
  {    
    if ( port2_int_status & (1 << i))
    {
        if ( callbacksInt[int_pin_tbl[2][i]]._callback != NULL )
        {
            callbacksInt[int_pin_tbl[2][i]]._callback() ;
        }
        (*((volatile unsigned int *)(0x44000000 + 0x38)))  |= 1 << i;                  // C port UARTx->INTSTATUS/INTCLEAR  
//        (*((volatile unsigned int *)(0x44000000 + 0x24)))  |= 0xffff;                  // C port UARTx->INTENCLR  
//        (*((volatile unsigned int *)(0x44000000 + 0x2C)))  |= 0xffff;                  // C port UARTx->INTTYPECLR  
    }
  }
#if 1
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'C';                           // UARTx->DR
#endif
}


/**
  * @brief  This function handles PORT3 Handler.
  * @param  None
  * @retval None
  */
void PORT3_Handler(void)
{
    unsigned int port3_int_status = 0;
    int i ;
    port3_int_status = (*((volatile unsigned int *)(0x45000000 + 0x38)));                  // D port UARTx->INTSTATUS/INTCLEAR  

  for ( i = 0 ; i < EXTERNAL_NUM_INTERRUPTS ; i++ )
  {    
    if ( port3_int_status & (1 << i))
    {
        if ( callbacksInt[int_pin_tbl[3][i]]._callback != NULL )
        {
            callbacksInt[int_pin_tbl[3][i]]._callback() ;
        }
        (*((volatile unsigned int *)(0x45000000 + 0x38)))  |= 1 << i;                  // C port UARTx->INTSTATUS/INTCLEAR  
//        (*((volatile unsigned int *)(0x44000000 + 0x24)))  |= 0xffff;                  // C port UARTx->INTENCLR  
//        (*((volatile unsigned int *)(0x44000000 + 0x2C)))  |= 0xffff;                  // C port UARTx->INTTYPECLR  
    }
  }
#if 1
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'D';                           // UARTx->DR
#endif
}
////////////// Ãß°¡µÊ
#ifdef __cplusplus
}
#endif
