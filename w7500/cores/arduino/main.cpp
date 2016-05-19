/*
  main.cpp - Main loop for Arduino sketches
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

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

#define ARDUINO_MAIN
#include "Arduino.h"


/////////// 새코드
extern void WDT_Init(void);
extern void WDT_Start(void);
extern void WDT_IntClear(void);
extern void WDT_SetWDTLoad(unsigned int Load);
extern unsigned int WDT_GetWDTValue(void);

#define IAP_ENTRY 0x1FFF1001            // Because Thum code
#define IAP_ERAS			0x010
#define IAP_ERAS_DAT0		(IAP_ERAS + 0)      //
#define IAP_ERAS_DAT1		(IAP_ERAS + 1)
#define IAP_ERAS_SECT		(IAP_ERAS + 2)				
#define IAP_ERAS_BLCK		(IAP_ERAS + 3)	
#define IAP_ERAS_CHIP		(IAP_ERAS + 4)	
#define IAP_ERAS_MASS		(IAP_ERAS + 5)	

#define IAP_PROG			0x020
#define IAP_PROG_DAT0		(IAP_PROG + 0)	
#define IAP_PROG_DAT1		(IAP_PROG + 1)	
#define IAP_PROG_CODE		(IAP_PROG + 2)	






#if 1
#include "wiring_digital.h"
//void PWM3_Handler(void)
/**
  * @brief  DO IAP Function
  */
void DO_IAP( uint32_t id, uint32_t dst_addr, uint8_t* src_addr, uint32_t size)
{
    uint32_t temp_interrupt;
    // Backup Interrupt Set Pending Register
    temp_interrupt = (NVIC->ISPR[0]);
    (NVIC->ISPR[0]) = (uint32_t)0xFFFFFFFF;
    
    // Call IAP Function
    ((void(*)(uint32_t,uint32_t,uint8_t*,uint32_t))IAP_ENTRY)( id,dst_addr,src_addr,size);

    // Restore Interrupt Set Pending Register
    (NVIC->ISPR[0]) = temp_interrupt;
}

uint32_t vector_copy(void)
{
    uint32_t i;
    uint32_t func;
    uint8_t flash_vector_area[0x100];
    uint32_t * flash_vector_address = (uint32_t *)flash_vector_area;

//    uint32_t iap_reset_addr;
    
//    for (i = 0; i < 0x8; i++)         flash_vector_area[i] = *(volatile uint8_t *)(0x00000000+i);
//    for (i = 8; i < 0xA8; i++)        flash_vector_area[i] = *(volatile uint8_t *)(0x00001000+i);
    for (i = 0x00; i < 0x10; i++)       flash_vector_area[i] = *(volatile uint8_t *)(0x00000000+i);
    for (i = 0x10; i < 0xA8; i++)       flash_vector_area[i] = *(volatile uint8_t *)(0x00001000+i);
    for (i = 0xA8; i < 0x100; i++)      flash_vector_area[i] = *(volatile uint8_t *)(0x00000000+i);

    func = (uint32_t)SysTick_Handler;
    *(flash_vector_address + 15) = func;

// GPIO interrupt
    func = (uint32_t)PORT0_Handler;
    *(flash_vector_address + 16 + 7) = func;

    func = (uint32_t)PORT1_Handler;
    *(flash_vector_address + 16 + 8) = func;

    func = (uint32_t)PORT2_Handler;
    *(flash_vector_address + 16 + 9) = func;

    func = (uint32_t)PORT3_Handler;
    *(flash_vector_address + 16 + 10) = func;

// PWM interrupt
    func = (uint32_t)PWM0_Handler;
    *(flash_vector_address + 16 + 14) = func;

    func = (uint32_t)PWM1_Handler;
    *(flash_vector_address + 16 + 15) = func;

    func = (uint32_t)PWM3_Handler;
    *(flash_vector_address + 16 + 17) = func;

    /* Erase interrupt vector table area : Sector 0 */
    DO_IAP(IAP_ERAS_SECT, 0x00000000, 0, 0);
    /* Write appl vector table to 0x00000000 */
    DO_IAP(IAP_PROG_CODE, 0x00000000, flash_vector_area , 0x100);
}

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
void WDT_Init(void)
{
    /* Check the parameters */
   
//    WDT->WDTLock = 0x1ACCE551;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1ACCE551;

//    WDT->WDTControl &= ~(WDTControl_IntEnable << WDTControl_IntEn_Pos); 
    *(volatile unsigned int *)(0x40000000 + 0x08) &= ~(0x1ul << 0);
    
    //WDT->WDTLoad      = WDT_InitStruct->WDTLoad;
    *(volatile unsigned int *)(0x40000000 + 0x00) = 0xFF0000;
//    *(volatile unsigned int *)(0x40000000 + 0x00) = 0x1000;
    //WDT->WDTControl   = (WDT_InitStruct->WDTControl_RstEn << WDTControl_RstEn_Pos);
    *(volatile unsigned int *)(0x40000000 + 0x08) = (0x1ul << 1);

    //WDT->WDTLock = 0x1;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1;
}

void WDT_Start(void)
{
    //WDT->WDTLock = 0x1ACCE551;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1ACCE551;
    
    //WDT->WDTControl |= (WDTControl_IntEnable << WDTControl_IntEn_Pos); 
    *(volatile unsigned int *)(0x40000000 + 0x08) |= (0x1ul << 0);
    
    //WDT->WDTLock = 0x1;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1;
}

void WDT_IntClear(void)
{
    //WDT->WDTLock = 0x1ACCE551;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1ACCE551;
    //WDT->WDTIntClr = 0x1;
    *(volatile unsigned int *)(0x40000000 + 0x0c) = 0x1;
    
//    *(volatile unsigned int *)(0x40000000 + 0x08) &= ~(0x1ul << 0 | 0x1ul << 1);

    //WDT->WDTLock = 0x1;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1;
}

void WDT_SetWDTLoad(unsigned int Load)
{
    //WDT->WDTLock = 0x1ACCE551;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1ACCE551;
    //WDT->WDTLoad = Load;
    *(volatile unsigned int *)(0x40000000 + 0x00) = Load;
    //WDT->WDTLock = 0x1;
    *(volatile unsigned int *)(0x40000000 + 0xC00) = 0x1;
}

unsigned int WDT_GetWDTValue(void)
{
    //return WDT->WDTValue;
    return *(volatile unsigned int *)(0x40000000 + 0x04);
}
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------

char check_byte(void)
{
    if( ((*(volatile unsigned long *) 0x4000D018) & (0x01 << 4)) )
    {
        return 0;
    }
    else
    {
        return (char) (*(volatile unsigned long *) 0x4000D000 & 0xff);
    }
}
#endif
////////// 새코드

#include "wiring_analog.h"

extern struct _PWM_PIN_TBL pwm_pin_tbl[];

// PWM0을 사용한 경우
void pwm0_init(unsigned int frequency)
{
//PWM-0
    // Interrupt  설정
    *(volatile unsigned int *)0xE000E100 = (1 << ((uint32_t)(14) & 0x1F));

    // Timer mode로 설정함
    /* Select Timer/Counter mode as Timer mode */ 
    *(volatile unsigned int *)(0x40005024 + pwm_pin_tbl[14/*pin*/].pwm_num) = (0x0);
    /* Set Prescale register value */
    *(volatile unsigned int *)(0x40005014 + pwm_pin_tbl[14/*pin*/].pwm_num) = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    /* Set Match register value */
    *(volatile unsigned int *)(0x40005018 + pwm_pin_tbl[14/*pin*/].pwm_num) = frequency / 2 /*400 * val*/;     //MR
    /* Set Limit register value */
    *(volatile unsigned int *)(0x4000501C + pwm_pin_tbl[14/*pin*/].pwm_num) = frequency/*102400*/; // 80% duty cycle
    /* Select Up-down mode */
    *(volatile unsigned int *)(0x40005020 + pwm_pin_tbl[14/*pin*/].pwm_num) = (0x0);        //PWM_CHn_UDMR_UpCount
    /* Select Periodic mode */ 
    *(volatile unsigned int *)(0x40005034 + pwm_pin_tbl[14/*pin*/].pwm_num) = (0x1);        //PWM_CHn_PDMR_Periodic

//------------------------------------------------------------------------------------------
    //PWM->SSR &= PWM_SSR_SS3_Stop;
    *(volatile unsigned int *)0x40005804 &= ~(0x1 << pwm_pin_tbl[14/*pin*/].pwm_pin);

    //PWM->IER |= PWM_IER_IE3_Enable;
    *(volatile unsigned int *)0x40005800 |= (0x1 << pwm_pin_tbl[14/*pin*/].pwm_pin);

    //PWM_CH1->IER |= PWM_CHn_IER_MIE | PWM_CHn_IER_OIE/*PWM_CHn_IER*/; 
    *(volatile unsigned int *)(0x40005004 + pwm_pin_tbl[14/*pin*/].pwm_num) |= (0x1 << 0 | 0x1 << 1);

//------------------------------------------------------------------------------------------
    //PWM->SSR |= PWM_SSR_SS3_Start;
    *(volatile unsigned int *)0x40005804 |= (0x1ul << pwm_pin_tbl[14/*pin*/].pwm_pin);
}


// PWM1을 사용한 경우
void pwm1_init(unsigned int frequency)
{
//PWM-1
    // Interrupt  설정
    *(volatile unsigned int *)0xE000E100 = (1 << ((uint32_t)(15) & 0x1F));

    // Timer mode로 설정함
    /* Select Timer/Counter mode as Timer mode */ 
    *(volatile unsigned int *)(0x40005024 + pwm_pin_tbl[12/*pin*/].pwm_num) = (0x0);
    /* Set Prescale register value */
    *(volatile unsigned int *)(0x40005014 + pwm_pin_tbl[12/*pin*/].pwm_num) = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    /* Set Match register value */
    *(volatile unsigned int *)(0x40005018 + pwm_pin_tbl[12/*pin*/].pwm_num) = frequency / 2 /*400 * val*/;     //MR
    /* Set Limit register value */
    *(volatile unsigned int *)(0x4000501C + pwm_pin_tbl[12/*pin*/].pwm_num) = frequency/*102400*/; // 80% duty cycle
    /* Select Up-down mode */
    *(volatile unsigned int *)(0x40005020 + pwm_pin_tbl[12/*pin*/].pwm_num) = (0x0);        //PWM_CHn_UDMR_UpCount
    /* Select Periodic mode */ 
    *(volatile unsigned int *)(0x40005034 + pwm_pin_tbl[12/*pin*/].pwm_num) = (0x1);        //PWM_CHn_PDMR_Periodic

//------------------------------------------------------------------------------------------
    //PWM->SSR &= PWM_SSR_SS3_Stop;
    *(volatile unsigned int *)0x40005804 &= ~(0x1 << pwm_pin_tbl[12/*pin*/].pwm_pin);

    //PWM->IER |= PWM_IER_IE3_Enable;
    *(volatile unsigned int *)0x40005800 |= (0x1 << pwm_pin_tbl[12/*pin*/].pwm_pin);

    //PWM_CH1->IER |= PWM_CHn_IER_MIE | PWM_CHn_IER_OIE/*PWM_CHn_IER*/; 
    *(volatile unsigned int *)(0x40005004 + pwm_pin_tbl[12/*pin*/].pwm_num) |= (0x1 << 0 | 0x1 << 1);

//------------------------------------------------------------------------------------------
    //PWM->SSR |= PWM_SSR_SS3_Start;
    *(volatile unsigned int *)0x40005804 |= (0x1ul << pwm_pin_tbl[12/*pin*/].pwm_pin);
}


// PWM3을 사용한 경우
void pwm3_init(unsigned int frequency)
{
//PWM-3
    // Interrupt  설정
    *(volatile unsigned int *)0xE000E100 = (1 << ((uint32_t)(17) & 0x1F));

    // Timer mode로 설정함
    /* Select Timer/Counter mode as Timer mode */ 
    *(volatile unsigned int *)(0x40005024 + pwm_pin_tbl[13/*pin*/].pwm_num) = (0x0);
    /* Set Prescale register value */
    *(volatile unsigned int *)(0x40005014 + pwm_pin_tbl[13/*pin*/].pwm_num) = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    /* Set Match register value */
    *(volatile unsigned int *)(0x40005018 + pwm_pin_tbl[13/*pin*/].pwm_num) = frequency / 2 /*400 * val*/;     //MR
    /* Set Limit register value */
    *(volatile unsigned int *)(0x4000501C + pwm_pin_tbl[13/*pin*/].pwm_num) = frequency/*102400*/; // 80% duty cycle
    /* Select Up-down mode */
    *(volatile unsigned int *)(0x40005020 + pwm_pin_tbl[13/*pin*/].pwm_num) = (0x0);        //PWM_CHn_UDMR_UpCount
    /* Select Periodic mode */ 
    *(volatile unsigned int *)(0x40005034 + pwm_pin_tbl[13/*pin*/].pwm_num) = (0x1);        //PWM_CHn_PDMR_Periodic

//------------------------------------------------------------------------------------------
    //PWM->SSR &= PWM_SSR_SS3_Stop;
    *(volatile unsigned int *)0x40005804 &= ~(0x1 << pwm_pin_tbl[13/*pin*/].pwm_pin);

    //PWM->IER |= PWM_IER_IE3_Enable;
    *(volatile unsigned int *)0x40005800 |= (0x1 << pwm_pin_tbl[13/*pin*/].pwm_pin);

    //PWM_CH1->IER |= PWM_CHn_IER_MIE | PWM_CHn_IER_OIE/*PWM_CHn_IER*/; 
    *(volatile unsigned int *)(0x40005004 + pwm_pin_tbl[13/*pin*/].pwm_num) |= (0x1 << 0 | 0x1 << 1);

//------------------------------------------------------------------------------------------
    //PWM->SSR |= PWM_SSR_SS3_Start;
    *(volatile unsigned int *)0x40005804 |= (0x1ul << pwm_pin_tbl[13/*pin*/].pwm_pin);
}

void Uart2Init(uint32_t baud)
 {
#if 0
	UART2->BAUDDIV = baud;
	UART2->CTRL	=  UART2_CTRL_RX_EN | UART2_CTRL_TX_EN;

	 return;
#endif
 }

void SystemInit( void )
{
#if 1
#if 1
    WDT_IntClear();
    vector_copy();
    WDT_Init();
    WDT_Start();
#endif
    WDT_SetWDTLoad(0xFF0000);
    printf("1111\n");
	//---------- System Initialization
////////////    *(volatile unsigned int *)0x41001210 = *(volatile unsigned int *)0x0003FDB8;   // W7500x_TRIM_BGT - W7500x_INFO_BGT
////////////    *(volatile unsigned int *)0x41001004 = *(volatile unsigned int *)0x0003FDBC;   // W7500x_TRIM_OSC - W7500x_INFO_OSC : reserved 영역에 있음

    // Set PLL input frequency
    //*(volatile unsigned int *)0x41001020 = 0x1ul;                   // CRG->PLL_IFSR - External oscillator clock (OCLK, 8MHz ~ 24MHz)
////////////    *(volatile unsigned int *)0x41001020 = 0x0ul;                     // CRG->PLL_IFSR - Internal 8MHz RC oscillator clock(RCLK)

/////// timer test start
//    pwm0_init(0xffffff);
//    pwm1_init(0xffffff);
/////// timer test end
    pwm3_init(0xffffffff);
//    pwm3_init(0x1000);
//    pwm3_init(9, 0xffffffff, 0);
    SysTick_Config((20000000/1000));
//    SysTick_Config((0xFFFFFUL));

    printf("2222\n");

    Uart2Init(173);
    WDT_SetWDTLoad(0xFF0000);

    printf("3333\n");
  //---------- UART_Init Initialization
////////////    *(volatile unsigned int *)(0x4000D000 + 0x30) &= ~(0x1 <<  0);      // UARTx->CR  UART Disable

    // UART Baud rate을 위해 내부 OSC사용 
    // 8MHz 클럭
////////////    *(volatile unsigned int *)0x41001150 = 0x02ul;                    // CRG->UARTCLK_SSR - Set UART Clock using internal Oscilator ( 8MHz )

    // Set baudrate
    // 8000000 / 16 * 115200 = 4.340277777777778
    // 0.340277777777778 * 64 + 0.5 = 22.27777777777778
////////////    *(volatile unsigned int *)(0x4000D000 + 0x24) = 4;                  // UARTx->IBRD  
////////////    *(volatile unsigned int *)(0x4000D000 + 0x28) = 22;                 // UARTx->FBRD

////////////    *(volatile unsigned int *)(0x4000D000 + 0x2C) |= 0x3 << 5;          // UARTx->LCR_H   8N1
////////////    *(volatile unsigned int *)(0x4000D000 + 0x30) |= 0x1 << 0;          // UARTx->CR      UART ENABLE
#endif


#if 0   // 기존 코드
  //----- Tx & Rx led blinking during transmission (pin declaration) ----- begin ----
  PORT->Group[1].DIRSET.reg=0x00000008;  //PB03 as output (RX_LED)
  PORT->Group[1].OUTSET.reg=0x00000008;  //PB03 as output (RX_LED)
  
  PORT->Group[0].DIRSET.reg=0x08000000;  //PB03 as output (TX_LED)
  PORT->Group[0].OUTSET.reg=0x08000000;  //PB03 as output (TX_LED)
  //----- Tx & Rx led blinking during transmission (pin declaration) ----- end ----
  
  /* Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet */
  NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val ;

  /* Turn on the digital interface clock */
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK ;

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator)
   */
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | /* cf table 15.10 of product datasheet in chapter 15.8.6 */
                         SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K ;
  SYSCTRL->XOSC32K.bit.ENABLE = 1 ; /* separate call, as described in chapter 15.6.3 */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0 )
  {
    /* Wait for oscillator stabilization */
  }

  /* Software reset the module to ensure it is re-initialized correctly */
  /* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
   * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete, as described in chapter 13.8.1
   */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST ;

  while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) )
  {
    /* Wait for reset to complete */
  }

  /* ----------------------------------------------------------------------------------------------
   * 2) Put XOSC32K as source of Generic Clock Generator 1
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_XOSC32K ) ; // Generic Clock Generator 1

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 1 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_XOSC32K ) | // Generic Clock Generator 1
                      GCLK_GENCTRL_SRC_XOSC32K | // Selected source is External 32KHz Oscillator
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
   */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GENERIC_CLOCK_MULTIPLEXER_DFLL48M ) | // Generic Clock Multiplexer 0
                      GCLK_CLKCTRL_GEN_GCLK1 | // Generic Clock Generator 1 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 4) Enable DFLL48M clock
   */

  /* DFLL Configuration in Closed Loop mode, cf product datasheet chapter 15.6.7.1 - Closed-Loop Operation */

  /* Remove the OnDemand mode, Bug http://avr32.icgroup.norway.atmel.com/bugzilla/show_bug.cgi?id=9905 */
  SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0 ;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
                         SYSCTRL_DFLLMUL_FSTEP( 511 ) | // Fine step is 511, half of the max value
                         SYSCTRL_DFLLMUL_MUL( (VARIANT_MCK/VARIANT_MAINOSC) ) ; // External 32KHz is the reference

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* Write full configuration to DFLL control register */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | /* Enable the closed loop mode */
                           SYSCTRL_DFLLCTRL_WAITLOCK |
                           SYSCTRL_DFLLCTRL_QLDIS ; /* Disable Quick lock */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* Enable the DFLL */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) == 0 ||
          (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) == 0 )
  {
    /* Wait for locks flags */
  }

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) ; // Generic Clock Generator 0

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 0 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | // Generic Clock Generator 0
                      GCLK_GENCTRL_SRC_DFLL48M | // Selected source is DFLL 48MHz
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_IDC | // Set 50/50 duty cycle
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 6) Modify PRESCaler value of OSC8M to have 8MHz
   */
  SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val ;
  SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;

  /* ----------------------------------------------------------------------------------------------
   * 7) Put OSC8M as source for Generic Clock Generator 3
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_OSC8M ) ; // Generic Clock Generator 3

  /* Write Generic Clock Generator 3 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC8M ) | // Generic Clock Generator 3
                      GCLK_GENCTRL_SRC_OSC8M | // Selected source is RC OSC 8MHz (already enabled at reset)
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /*
   * Now that all system clocks are configured, we can set CPU and APBx BUS clocks.
   * There values are normally the one present after Reset.
   */
  PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
  PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
  PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
  PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;

  SystemCoreClock=VARIANT_MCK ;
#endif   // 기존 코드
}












void print_byte(unsigned int c)
{
    if (c == '\n') print_byte('\r');
    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = c;
}

char get_byte(void)
{
    while( ((*(volatile unsigned long *) 0x4000D018) & (0x01 << 4)) ) ;
    return (char) *(volatile unsigned long *) 0x4000D000;
}

void SerialOutputString(const char *s);

void        printf_c(char *fmt, ...);
static void PrintChar(char *fmt, char c);
static void PrintDec(char *fmt, int value);
static void PrintHex(char *fmt, int value);
static void PrintString(char *fmt, char *cptr);
static int  Power(int num, int cnt);
#define SWAP8(A)                (A)
#define SWAP16(A)                ((((A)&0x00ff)<<8) | ((A)>>8))
#define SWAP32(A)                ((((A)&0x000000ff)<<24) | (((A)&0x0000ff00)<<8) | (((A)&0x00ff0000)>>8) | (((A)&0xff000000)>>24))

//typedef int                        bool;
#define        true                        1
#define false                        0


// print in hex value.
// type= 8 : print in format "ff".
// type=16 : print in format "ffff".
// type=32 : print in format "ffffffff".
typedef enum {
        VAR_LONG=32,
        VAR_SHORT=16,
        VAR_CHAR=8
} VAR_TYPE;

//typedef (char *) va_list;
#define va_start(ap, p)                (ap = (char *) (&(p)+1))
#define va_arg(ap, type)        ((type *) (ap += sizeof(type)))[-1]
#define va_end(ap)

// Write a null terminated string to the serial port.
void SerialOutputString(const char *s)
{
        while (*s != 0) 
        {
                print_byte(*s);
                // If \n, also do \r.
                if (*s == '\n') print_byte('\r');
                s++;        
        }
} // SerialOutputString.

// 문자열 s1, s2을 길이 len의 범위 이내에서 비교.
// return : 0 : equil                ret : s1 > s2                -ret : s1 < s2
int StrNCmp(char *s1, char *s2, int len){
        int i;


        for(i = 0; i < len; i++){
                if(s1[i] != s2[i])        return ((int)s1[i]) - ((int)s2[i]);
                if(s1[i] == 0)                return 0;
        }
        return 0;
} // StrNCmp.

// 문자열 s1, s2를 비교.
// return : 0 : equil                ret : s1 > s2                -ret : s1 < s2
int StrCmp(char *s1, char *s2){
        for (; *s1 && *s2; s1++, s2++){
                if (*s1 != *s2) return ((int)(*s1) - (int)(*s2));
        }
        if (*s1 || *s2) return ((int)(*s1) - (int)(*s2));
        return 0;
}        // StrCmp.

// 역할 : 10진수 문자열 s에서 정수를 만들어 retval이 가리키는 위치에 기록.
// 매개 : s      : 변환할 문자열의 주소.
//        retval : 변환된 값이 기록될 주소.
// 반환 : return : 1 : success                0 : failure.
// 주의 :
int DecToLong(char *s, long *retval){
        long remainder;
        if (!s || !s[0]) return false;


        for (*retval=0; *s; s++){
                if (*s < '0' || *s > '9') return false;
                remainder = *s - '0';
                *retval = *retval * 10 + remainder;
        }


        return true;
}        // DecToLong.

// 역할 : printf_c() 중 일부를 간단하게 구현.
// 매개 : fmt : printf_c()와 동일하나 "%s", "%c", "%d", "%x" 사용 가능.
//              %d, %x의 경우에는 "%08x", "%8x"와 같이 나타낼 길이와 빈 공간을 0으로 채울지 선택 가능.
// 반환 : 없음.
// 주의 : 없음.
void printf_c(char *fmt, ...)
{
        int                i;
//        va_list args;
        char        *args;
        char        *s=fmt;
        char        format[10];        // fmt의 인자가 "%08lx"라면, "08l"를 임시로 기록.
        
        va_start(args, fmt);
        while (*s){
                if (*s=='%'){
                        s++;
                        // s에서 "%08lx"형식을 가져와 format에 기록. 나중에 출력함수에 넘겨줌.
                        format[0] = '%';
                        for (i=1; i<10;){
                                if (*s=='c' || *s=='d' || *s=='x' || *s=='s' || *s=='%'){
                                        format[i++] = *s;
                                        format[i] = '\0';
                                        break;
                                }
                                else {
                                        format[i++] = *s++;
                                }
                        }
                        // "%s", "%c", "%d", "%x"를 찾아 출력할 함수 호출.
                        switch (*s++){
                                case 'c' :
                                        PrintChar(format, va_arg(args, int));
                                        break;
                                case 'd' :
                                        PrintDec(format, va_arg(args, int));
                                        break;
                                case 'x' :
                                        PrintHex(format, va_arg(args, int));
                                        break;
                                case 's' :
                                        PrintString(format, va_arg(args, char *));
                                        break;
                                case '%' :
                                        PrintChar("%c", '%');
                                        break;
                        }
                }
                else {
                        PrintChar("%c", *s);
                        s++;
                }
        }
        va_end(args);
        return;
}

void PrintChar(char *fmt, char c)
{
        print_byte(c);
        return;
}

void PrintDec(char *fmt, int l)
{
        int        i, j;
        char        c, *s=fmt, tol[10];
        bool        flag0=false, flagl=false;        // "%08lx"에서 '0', 'l'의 존재 여부.
        long        flagcnt=0;                                        // "%08lx"에서 "8"을 찾아서 long형으로.
        bool        leading_zero=true;                        // long형의 data를 출력하기 위한 변수.
        long        divisor, result, remainder;


        // fmt의 "%08lx"에서 '0', '8', 'l'을 해석.
        for (i=0; (c=s[i]) != 0; i++){
                if (c=='d') break;
                else if (c>='1' && c<='9'){
                        for (j=0; s[i]>='0' && s[i]<='9'; j++){
                                tol[j] = s[i++];
                        }
                        tol[j] = '\0';
                        i--;
                        DecToLong(tol, &flagcnt);
                }
                else if (c=='0') flag0=true;
                else if (c=='l') flagl=true;
                else continue;
        }


        // 위의 flag에 따라 출력.
        if (flagcnt){
                if (flagcnt>9) flagcnt=9;
                remainder = l%(Power(10, flagcnt));        // flagcnt보다 윗자리의 수는 걸러냄. 199에 flagcnt==2이면, 99만.


                for (divisor=Power(10, flagcnt-1); divisor>0; divisor/=10){
                        result = remainder/divisor;
                        remainder %= divisor;


                        if (result!=0 || divisor==1) leading_zero = false;


                        if (leading_zero==true){
                                if (flag0)        print_byte('0');
                                else                print_byte(' ');
                        }
                        else print_byte((char)(result)+'0');
                }
        } else {
                remainder = l;


                for (divisor=1000000000; divisor>0; divisor/=10){
                        result = remainder/divisor;
                        remainder %= divisor;


                        if (result!=0 || divisor==1) leading_zero = false;
                        if (leading_zero==false) print_byte((char)(result)+'0');
                }
        }
        return;
}

void PrintHex(char *fmt, int l){
        int                i, j;
        char        c, *s=fmt, tol[10];
        bool        flag0=false, flagl=false;        // flags.
        long        flagcnt=0;
        bool        leading_zero=true;
        char        uHex, lHex;
        int                cnt;                                                // "%5x"의 경우 5개만 출력하도록 출력한 개수.


        // fmt의 "%08lx"에서 '0', '8', 'l'을 해석.
        for (i=0; (c=s[i]) != 0; i++){
                if (c=='x') break;
                else if (c>='1' && c<='9'){
                        for (j=0; s[i]>='0' && s[i]<='9'; j++){
                                tol[j] = s[i++];
                        }
                        tol[j] = '\0';
                        i--;
                        DecToLong(tol, &flagcnt);
                }
                else if (c=='0') flag0=true;
                else if (c=='l') flagl=true;
                else continue;
        }


        s = (char *)(&l);
        l = SWAP32(l);                // little, big endian에 따라서.(big이 출력하기 쉬워 순서를 바꿈)
        
        // 위의 flag에 따라 출력.
        if (flagcnt){
                if (flagcnt&0x01){        // flagcnt가 홀수 일때, upper를 무시, lower만 출력.
                        c = s[(8-(flagcnt+1))/2]; // 홀수 일때 그 위치를 포함하는 곳의 값을 가져 옵니다.
                        
                        // lower 4 bits를 가져와서 ascii code로.
                        lHex = ((c>>0)&0x0f);
                        if (lHex!=0) leading_zero=false;
                        if (lHex<10) lHex+='0';
                        else         lHex+='A'-10;


                        // lower 4 bits 출력.
                        if (leading_zero){
                                if (flag0) print_byte('0');
                                else       print_byte(' ');
                        }
                        else print_byte(lHex);
                        
                        flagcnt--;
                }


                // byte단위의 data를 Hex로 출력.
                for (cnt=0, i=(8-flagcnt)/2; i<4; i++){
                        c = s[i];
                                
                        // get upper 4 bits and lower 4 bits.
                        uHex = ((c>>4)&0x0f);
                        lHex = ((c>>0)&0x0f);


                        // upper 4 bits and lower 4 bits to '0'~'9', 'A'~'F'.
                        // upper 4 bits를 ascii code로.
                        if (uHex!=0) leading_zero = false;
                        if (uHex<10) uHex+='0';
                        else         uHex+='A'-10;


                        // upper 4 bits 출력.
                        if (leading_zero){
                                if (flag0) print_byte('0');
                                else       print_byte(' ');
                        }
                        else print_byte(uHex);
                        
                        // lower 4 bits를 ascii code로.
                        if (lHex!=0) leading_zero = false;
                        if (lHex<10) lHex+='0';
                        else         lHex+='A'-10;


                        // lower 4 bits 출력.
                        if (leading_zero){
                                if (flag0) print_byte('0');
                                else       print_byte(' ');
                        }
                        else print_byte(lHex);
                }
        }
        else {
                for (i=0; i<4; i++){
                        c = s[i];
        
                        // get upper 4 bits and lower 4 bits.
                        uHex = ((c>>4)&0x0f);
                        lHex = ((c>>0)&0x0f);


                        // upper 4 bits and lower 4 bits to '0'~'9', 'A'~'F'.
                        if (uHex!=0) leading_zero = false;
                        if (uHex<10) uHex+='0';
                        else         uHex+='A'-10;
                        if (!leading_zero) print_byte(uHex);
                        
                        if (lHex!=0 || i==3) leading_zero = false;
                        if (lHex<10) lHex+='0';
                        else         lHex+='A'-10;
                        if (!leading_zero) print_byte(lHex);
                }
        }
        return;
}

void PrintString(char *fmt, char *s){
        if (!fmt || !s) return;
        while (*s) print_byte(*s++);
        return;
}

int Power(int num, int cnt){
        long retval=num;
        cnt--;


        while (cnt--){
                retval *= num;
        }
        return retval;
} 
























































/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{

  init();

        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'I';                           // UARTx->DR

#if 0
/////////////while(1)
/////////////  printf("1");
  delay(1);
#if defined(USBCON)
	USBDevice.init();
	USBDevice.attach();
#endif
#endif

        /* Loop until UART0 TX FIFO Register is empty */
        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'J';                           // UARTx->DR


  setup();

         /* Loop until UART0 TX FIFO Register is empty */
//        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
//        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'K';                           // UARTx->DR

 for (;;)
  {
        /* Loop until UART0 TX FIFO Register is empty */
//        while(((*((volatile unsigned int *)(0x4000D000 + 0x18))) & (0x01 << 7)) == 0);     // UARTx->FR
//        (*((volatile unsigned int *)(0x4000D000 + 0x00))) = 'L';                           // UARTx->DR
    loop();
//////    if (serialEventRun) serialEventRun();
  }

  return 0;
}
