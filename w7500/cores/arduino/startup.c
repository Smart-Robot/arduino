/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2014 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

////////// 추가
#include "W7500x.h"
////////// 추가
//#include "sam.h"
#include "variant.h"

/* Initialize segments */
extern uint32_t __etext ;
extern uint32_t __data_start__ ;
extern uint32_t __data_end__ ;
extern uint32_t __bss_start__ ;
extern uint32_t __bss_end__ ;
extern uint32_t __StackTop;

extern int main( void );
extern void __libc_init_array(void);
extern void SystemInit( void );

/* Default empty handler */
void Dummy_Handler(void);

/* Cortex-M0+ core handlers */
#if defined DEBUG
void NMI_Handler( void )
{
  while ( 1 )
  {
  }
}

void HardFault_Handler( void )
{
  while ( 1 )
  {
  }
}

void SVC_Handler( void )
{
  while ( 1 )
  {
  }
}

void PendSV_Handler( void )
{
  while ( 1 )
  {
  }
}

void SysTick_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#else
//1
void NMI_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HardFault_Handler       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif //DEBUG

void SSP1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UART0_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UART1_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UART2_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PORT0_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PORT1_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PORT2_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PORT3_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMA_Handler 		        ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DUALTIMER0_Handler     ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DUALTIMER1_Handler     ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM0_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM1_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM2_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM3_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM4_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM5_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM6_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM7_Handler 		      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler 		        ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_Handler	  	      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WZTOE_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

#if 0
/* Peripherals handlers */
void PM_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SYSCTRL_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void NVMCTRL_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

//void USB_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PORT0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

void EVSYS_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM4_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

void SERCOM5_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
/////////////////void PWM0_Handler 		       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/////////////////void TCC0_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM1_Handler 		       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

void TCC1_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/////////////////void TCC2_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PWM3_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

void TC3_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC4_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC5_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC6_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC7_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void AC_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PTC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2S_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif

/* Exception Table */
__attribute__ ((section(".isr_vector")))
const DeviceVectors exception_table=
{
  /* Configure Initial Stack Pointer, using linker-generated symbols */
  (void*) (&__StackTop),

  (void*) Reset_Handler,
  (void*) NMI_Handler,
  (void*) HardFault_Handler,
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) SVC_Handler,
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) PendSV_Handler,
  (void*) SysTick_Handler,

    /* External Interrupts */
  (void*) SSP1_Handler,               /* 16+ 1: SSP 1 Handler                   */
  (void*) UART0_Handler,              /* 16+ 2: UART 0 Handler                  */
  (void*) UART1_Handler,              /* 16+ 3: UART 1 Handler                  */
  (void*) UART2_Handler,              /* 16+ 4: UART 2 Handler                  */
  (void*) I2C0_Handler,               /* 16+ 5: I2C 0 Handler                   */
  (void*) I2C1_Handler,               /* 16+ 6: I2C 1 Handler                   */
  (void*) PORT0_Handler,              /* 16+ 7: GPIO Port 0 Combined Handler    */
  (void*) PORT1_Handler,              /* 16+ 8: GPIO Port 1 Combined Handler    */
  (void*) PORT2_Handler,              /* 16+ 9: GPIO Port 2 Combined Handler    */
  (void*) PORT3_Handler,              /* 16+10: GPIO Port 3 Combined Handler    */
  (void*) DMA_Handler,		            /* 16+11: DMA Combined Handler            */
  (void*) DUALTIMER0_Handler,         /* 16+12: Dual timer 0 handler            */ 
  (void*) DUALTIMER1_Handler,         /* 16+ 13: Dual timer 1 Handler	*/
  (void*) PWM0_Handler,		            /* 16+ 14: PWM0 Handler		*/
  (void*) PWM1_Handler,		            /* 16+ 15: PWM1 Handler		*/
  (void*) PWM2_Handler,		            /* 16+ 16: PWM2 Handler		*/
  (void*) PWM3_Handler,		            /* 16+ 17: PWM3 Handler		*/
  (void*) PWM4_Handler,		            /* 16+ 18: PWM4 Handler		*/
  (void*) PWM5_Handler,		            /* 16+ 19: PWM5 Handler		*/
  (void*) PWM6_Handler,		            /* 16+ 20: PWM6 Handler		*/
  (void*) PWM7_Handler,		            /* 16+ 21: PWM7 Handler		*/
  (void*) RTC_Handler,		            /* 16+ 22: RTC Handler			*/
  (void*) ADC_Handler,  	            /* 16+ 23: ADC Handler			*/
  (void*) WZTOE_Handler,              /* 16+ 24: WZTOE Handler		*/
  (void*) EXTI_Handler,               /* 16+ 25: EXTI Handler       */
/******************  dumy code for sector copy  *****************/
  (void*) (0UL),                      /* 16+ 26: Reserved       */
  (void*) (0UL),                      /* 16+ 27: Reserved       */
  (void*) (0UL),                      /* 16+ 28: Reserved       */
  (void*) (0UL),                      /* 16+ 29: Reserved       */
  (void*) (0UL),                      /* 16+ 30: Reserved       */
  (void*) (0UL),                      /* 16+ 31: Reserved       */
  (void*) (0UL),                      /* 16+ 32: Reserved       */
  (void*) (0UL),                      /* 16+ 33: Reserved       */
  (void*) (0UL),                      /* 16+ 34: Reserved       */
  (void*) (0UL),                      /* 16+ 35: Reserved       */
  (void*) (0UL),                      /* 16+ 36: Reserved       */
  (void*) (0UL),                      /* 16+ 37: Reserved       */
  (void*) (0UL),                      /* 16+ 38: Reserved       */
  (void*) (0UL),                      /* 16+ 39: Reserved       */
  (void*) (0UL),                      /* 16+ 40: Reserved       */
  (void*) (0UL),                      /* 16+ 41: Reserved       */
  (void*) (0UL),                      /* 16+ 42: Reserved       */
  (void*) (0UL),                      /* 16+ 43: Reserved       */
  (void*) (0UL),                      /* 16+ 44: Reserved       */
  (void*) (0UL),                      /* 16+ 45: Reserved       */
  (void*) (0UL),                      /* 16+ 46: Reserved       */
  (void*) (0UL),                      /* 16+ 47: Reserved       */

#if 0
  /* Configurable interrupts */
  (void*) PM_Handler,             /*  0 Power Manager */
  (void*) SYSCTRL_Handler,        /*  1 System Control */
  (void*) WDT_Handler,            /*  2 Watchdog Timer */
  (void*) RTC_Handler,            /*  3 Real-Time Counter */
  (void*) EIC_Handler,            /*  --------4 External Interrupt Controller */
  (void*) NVMCTRL_Handler,        /*  5 Non-Volatile Memory Controller */
  (void*) DMAC_Handler,           /*  6 Direct Memory Access Controller */

//  (void*) USB_Handler,            /*  7 Universal Serial Bus */
  (void*) PORT0_Handler,              /* 16+ 7: GPIO Port 0 Combined Handler    */

  (void*) EVSYS_Handler,          /*  8 Event System Interface */
  (void*) SERCOM0_Handler,        /*  --------9 Serial Communication Interface 0 */
  (void*) SERCOM1_Handler,        /* 10 Serial Communication Interface 1 */
  (void*) SERCOM2_Handler,        /* 11 Serial Communication Interface 2 */
  (void*) SERCOM3_Handler,        /* 12 Serial Communication Interface 3 */
  (void*) SERCOM4_Handler,        /* 13 Serial Communication Interface 4 */

  (void*) SERCOM5_Handler,        /* ---------14 Serial Communication Interface 5 */
//////////////////////  (void*) PWM0_Handler,		            /* 16+ 14: PWM1 Handler		*/

//////////////////////  (void*) TCC0_Handler,           /* 15 Timer Counter Control 0 */
  (void*) PWM1_Handler,		            /* 16+ 15: PWM1 Handler		*/

  (void*) TCC1_Handler,           /* 16 Timer Counter Control 1 */

//////////////////////  (void*) TCC2_Handler,           /* 17 Timer Counter Control 2 */
  (void*) PWM3_Handler,		            /* 16+ 17: PWM3 Handler		*/

  (void*) TC3_Handler,            /* 18 Basic Timer Counter 0 */
  (void*) TC4_Handler,            /* 19 Basic Timer Counter 1 */
  (void*) TC5_Handler,            /* --------20 Basic Timer Counter 2 */
  (void*) TC6_Handler,            /* 21 Basic Timer Counter 3 */
  (void*) TC7_Handler,            /* 22 Basic Timer Counter 4 */
  (void*) ADC_Handler,            /* 23 Analog Digital Converter */
  (void*) AC_Handler,             /* 24 Analog Comparators */
  (void*) DAC_Handler,            /* 25 Digital Analog Converter */
  (void*) PTC_Handler,            /* 26 Peripheral Touch Controller */
  (void*) I2S_Handler             /* 27 Inter-IC Sound Interface */
#endif



} ;

/**
 * \brief This is the code that gets called on processor reset.
 * It configures the needed clocks and according Flash Read Wait States.
 * At reset:
 * - OSC8M clock source is enabled with a divider by 8 (1MHz).
 * - Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 * We need to:
 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 * 2) Put XOSC32K as source of Generic Clock Generator 1
 * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 * 4) Enable DFLL48M clock
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 * 6) Modify PRESCaler value of OSCM to have 8MHz
 * 7) Put OSC8M as source for Generic Clock Generator 3
 */
// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)
// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
void Reset_Handler( void )
{
  uint32_t *pSrc, *pDest;

 //---------- System Initialization
    (*((volatile unsigned int *)(0x41001210))) = (*((volatile unsigned int *)(0x0003FDB8)));   // W7500x_TRIM_BGT - W7500x_INFO_BGT
    (*((volatile unsigned int *)(0x41001004))) = (*((volatile unsigned int *)(0x0003FDBC)));   // W7500x_TRIM_OSC - W7500x_INFO_OSC : reserved 영역에 있음

    // Set PLL input frequency
    //(*((volatile unsigned int *)(0x41001020))) = 0x1ul;                   // CRG->PLL_IFSR - External oscillator clock (OCLK, 8MHz ~ 24MHz)
    (*((volatile unsigned int *)(0x41001020))) = 0x0ul;                     // CRG->PLL_IFSR - Internal 8MHz RC oscillator clock(RCLK)

  //---------- UART_Init Initialization
    (*((volatile unsigned int *)(0x4000D000 + 0x30))) &= ~(0x1 <<  0);      // UARTx->CR  UART Disable

    // UART Baud rate을 위해 내부 OSC사용 
    // 8MHz 클럭
    (*((volatile unsigned int *)(0x41001150))) = 0x02ul;                    // CRG->UARTCLK_SSR - Set UART Clock using internal Oscilator ( 8MHz )

    // Set baudrate
    // 8000000 / 16 * 115200 = 4.340277777777778
    // 0.340277777777778 * 64 + 0.5 = 22.27777777777778
    (*((volatile unsigned int *)(0x4000D000 + 0x24))) = 4;                  // UARTx->IBRD  
    (*((volatile unsigned int *)(0x4000D000 + 0x28))) = 22;                 // UARTx->FBRD

    (*((volatile unsigned int *)(0x4000D000 + 0x2C))) |= 0x3 << 5;          // UARTx->LCR_H   8N1
    (*((volatile unsigned int *)(0x4000D000 + 0x30))) |= 0x1 << 0;          // UARTx->CR      UART ENABLE


  /* Initialize the initialized data section */
  pSrc = &__etext;
  pDest = &__data_start__;


  if ( (&__data_start__ != &__data_end__) && (pSrc != pDest) )
  {
    for (; pDest < &__data_end__ ; pDest++, pSrc++ )
    {
        /* Loop until UART0 TX FIFO Register is empty */
//        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
//        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'D';                           // UARTx->DR
      *pDest = *pSrc ;
    }
  }

  /* Clear the zero section */
  if ( (&__data_start__ != &__data_end__) && (pSrc != pDest) )
  {
    for ( pDest = &__bss_start__ ; pDest < &__bss_end__ ; pDest++ )
    {
        /* Loop until UART0 TX FIFO Register is empty */
//        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
//        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'E';                           // UARTx->DR
//      printf_c("pDest   : %x\n", pDest);
      *pDest = 0 ;
//      printf_c(" %x[%x]:%x ", &pDest, pDest, *pDest);
    }
  }

  //---------- UART_Init Initialization
//    while(1)
    {
        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = '\n';                           // UARTx->DR

        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'A';                           // UARTx->DR

        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'B';                           // UARTx->DR

    }



        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'C';                           // UARTx->DR

        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'C';                           // UARTx->DR

        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = '\n';                           // UARTx->DR


        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = '\r';                           // UARTx->DR


  /* Initialize the C library */
  __libc_init_array();


        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'F';                           // UARTx->DR


  SystemInit() ;

        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'G';                           // UARTx->DR

        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'H';                           // UARTx->DR


  /* Branch to main function */
  main() ;

  /* Infinite loop */
  while ( 1 )
  {
  }
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler( void )
{
  while ( 1 )
  {
  }
}
