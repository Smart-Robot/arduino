/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

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

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    B Hagman    09/08/18 Multiple pins
0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
0004    B Hagman    09/09/26 Fixed problems with ATmega8
0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                    09/11/25 Changed pin toggle method to XOR
                    09/11/25 Fixed timer0 from being excluded
0006    D Mellis    09/12/29 Replaced objects with functions
0007    M Sproul    10/08/29 Changed #ifdefs from cpu to register
0008    S Kanemoto  12/06/22 Fixed for Leonardo by @maris_HY
0009    Arduino.org 15/06/30 Add M0/M0 Pro support
*************************************************/


#include "Tone.h"


unsigned long int count_duration=0;
uint8_t pin_sound=0;

///// 추가

#include "Arduino.h"
//#include "delay.h"
//#include "wiring_analog.h"
uint8_t gTonePinNum;

#if 0
struct _PWM_PIN_TBL tone_pin_tbl[] =
{
#if 0    // 0 : W7500, 1 : W7500P
    { 0, 0,              0,  0,    0,   0},  // D0               
    { 0, 0,              0,  0,    0,   0},  // D1               
    { 0, 0,              0,  0,    0,   0},  // D2               
    { 2, 0x200, 0x02000000,  2, 0x80, 0x2},  // D3   A02  GPIOC.02      A00  GPIOC.00
    { 0, 0,              0,  0,    0,   0},  // D4                
    { 7, 0x700, 0x00000000,  1, 0x00, 0x2},  // D5   A07  GPIOA.01
    { 6, 0x600, 0x00000000,  0, 0x00, 0x2},  // D6   A06  GPIOA.00
    { 0, 0,              0,  0,    0,   0},  // D7                
    { 0, 0,              0,  0,    0,   0},  // D8                
    { 4, 0x100, 0x02000000,  1, 0x80, 0x2},  // D9   A01  GPIOC.01      A04  GPIOC.04
    { 2, 0x200, 0x00000000,  5, 0x00, 0x3},  // D10  A02  GPIOA.05
    { 5, 0x500, 0x00000000,  8, 0x00, 0x3},  // D11  A05  GPIOA.08

    { 0, 0x000, 0x02000000,  0, 0x80, 0x2},  // -    A00  GPIOC.00      A01  GPIOC.01  // tone으로 사용됨
    { 3, 0x300, 0x02000000,  3, 0x80, 0x2}  // -    A03  GPIOC.03

#else  // WIZ-WIKI W7500
    { 0, 0,              0,  0,    0,   0},  // D0               
    { 0, 0,              0,  0,    0,   0},  // D1               
    { 0, 0,              0,  0,    0,   0},  // D2               
    { 0, 0x000, 0x02000000,  0, 0x80, 0x2},  // D3   A00  GPIOC.00
    { 0, 0,              0,  0,    0,   0},  // D4                
    { 7, 0x700, 0x00000000,  1, 0x00, 0x2},  // D5   A07  GPIOA.01
    { 6, 0x600, 0x00000000,  0, 0x00, 0x2},  // D6   A06  GPIOA.00
    { 0, 0,              0,  0,    0,   0},  // D7                
    { 0, 0,              0,  0,    0,   0},  // D8                
    { 4, 0x400, 0x02000000,  4, 0x80, 0x2},  // D9   A04  GPIOC.04
    { 2, 0x200, 0x00000000,  5, 0x00, 0x3},  // D10  A02  GPIOA.05
    { 5, 0x500, 0x00000000,  8, 0x00, 0x3},  // D11  A05  GPIOA.08

    { 1, 0x100, 0x02000000,  1, 0x80, 0x2},  // -    A01  GPIOC.01  // tone으로 사용됨
    { 3, 0x300, 0x02000000,  3, 0x80, 0x2}  // -    A03  GPIOC.03
#endif
};
#endif
///// 추가 끝

void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
#if 1
    uint32_t startTone, endTone;
    gTonePinNum = pin;

// tone으로 사용이 될 GPIO 설정
    pinMode(pin, OUTPUT);

////// W7500
//PWM-1
    // Interrupt  설정
    *(volatile unsigned int *)0xE000E100 = (1 << ((uint32_t)(15) & 0x1F));


////// W7500P
//PWM-0
    // Interrupt  설정
////////////    *(volatile unsigned int *)0xE000E100 = (1 << ((uint32_t)(14) & 0x1F));

    // Timer mode로 설정함
    /* Select Timer/Counter mode as Timer mode */ 
    *(volatile unsigned int *)(0x40005024 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = (0x0);
    /* Set Prescale register value */
    *(volatile unsigned int *)(0x40005014 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    /* Set Match register value */
    *(volatile unsigned int *)(0x40005018 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = 20000000 / frequency / 4 /*400 * val*/;     //MR
    /* Set Limit register value */
    *(volatile unsigned int *)(0x4000501C + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = 20000000 / frequency / 2; // 80% duty cycle
    /* Select Up-down mode */
    *(volatile unsigned int *)(0x40005020 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = (0x0);        //PWM_CHn_UDMR_UpCount
    /* Select Periodic mode */ 
    *(volatile unsigned int *)(0x40005034 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = (0x1);        //PWM_CHn_PDMR_Periodic

//------------------------------------------------------------------------------------------
        //PWM->SSR &= PWM_SSR_SS1_Stop;
        *(volatile unsigned int *)0x40005804 &= ~(0x1 << pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_pin);

        //PWM->IER |= PWM_IER_IE1_Enable;
        *(volatile unsigned int *)0x40005800 |= (0x1 << pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_pin);

        //PWM_CH1->IER |= PWM_CHn_IER_MIE | PWM_CHn_IER_OIE/*PWM_CHn_IER*/; 
        *(volatile unsigned int *)(0x40005004 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) |= (0x1 << 0 | 0x1 << 1);

//------------------------------------------------------------------------------------------
        //PWM->SSR |= PWM_SSR_SS1_Start;
        *(volatile unsigned int *)0x40005804 |= (0x1ul << pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_pin);

        
//        startTone = TimerCnt;
//        while( (TimerCnt - startTone) < duration);
        delay(duration);

    /* Set Match register value */
    *(volatile unsigned int *)(0x40005018 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = 0;     //MR
    /* Set Limit register value */
    *(volatile unsigned int *)(0x4000501C + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) = 0xFFFFFFFF; // 80% duty cycle
 
        //PWM->SSR &= PWM_SSR_SS1_Stop;
        *(volatile unsigned int *)0x40005804 &= ~(0x1 << pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_pin);

        //PWM->IER &= PWM_IER_IE1_Disable;
        *(volatile unsigned int *)0x40005800 &= ~(0x1 << pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_pin);

        //PWM_CH1->IER &= ~(PWM_CHn_IER_MIE | PWM_CHn_IER_OIE/*PWM_CHn_IER*/); 
        *(volatile unsigned int *)(0x40005004 + pwm_pin_tbl/*tone_pin_tbl*/[12/*pin*/].pwm_num) &= ~(0x1 << 0 | 0x1 << 1);
#endif

#if 0
  unsigned int tim_per=0, duty=0,presc_tc=0, presc_tcc=0;
  double freq=0.0,div=0.0;
  uint8_t isTC = 0 ;
  uint8_t Channelx ;
  Tc* TCx ;
  Tcc* TCCx ;
  
  
  if((frequency < 20) | (frequency > 25000)) return;
  freq=frequency * 1.0;
  
  if(duration !=0)
  {
	  count_duration= 188 * duration;
	  set_timer5();
	  pin_sound=pin;
  }
  
  switch ( g_APinDescription[pin].ulPWMChannel )  //TC or TCC selection
    {
      case PWM3_CH0 :
            TCx = TC3 ;     // not work
            Channelx = 0 ;
            isTC = 1 ;
      break;

      case PWM3_CH1:        // not work
            TCx = TC3 ;
            Channelx = 1;
            isTC = 1;
      break;

      case PWM0_CH0 :
            TCCx = TCC0;
            Channelx = 0;
      break;

      case PWM0_CH1 :
            TCCx = TCC0;
            Channelx = 1;
      break;

      case PWM0_CH4 :
            TCCx = TCC0;
            Channelx = 0;
      break;

      case PWM0_CH5 :
            TCCx = TCC0;
            Channelx = 1;
      break;

      case PWM0_CH6 :
            TCCx = TCC0;
            Channelx = 2;
      break;

      case PWM0_CH7 :
            TCCx = TCC0;
            Channelx = 3;
      break;

      case PWM1_CH0 :
            TCCx = TCC1;
            Channelx = 0;
      break;

      case PWM1_CH1 :
            TCCx = TCC1;
            Channelx = 1;
      break;

      case PWM2_CH0 :
            TCCx = TCC2;
            Channelx = 0;
      break;

      case PWM2_CH1 :
            TCCx = TCC2;
            Channelx = 1;
      break;
    }
  
  
  switch ( GetTCNumber( g_APinDescription[pin].ulPWMChannel ) )  //clock selection using GCLK3
    {
      case 0: // TCC0
        //Enable GCLK for TCC0 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
      break ;

      case 1: // TCC1
        //Enable GCLK for TCC1 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
      break ;

      case 2: // TCC2
        //Enable GCLK for TCC2 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 )) ;
      break ;

      case 3: // TC3
        //Enable GCLK for TC3 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 ));
      break ;

      case 4: // TC4
        //Enable GCLK for TC4 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 ));
      break ;

      case 5: // TC5
        //Enable GCLK for TC5 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
      break ;
    }
    
    
    
    
    if((frequency >=20) & (frequency <= 100))
  {
    div=1024.0;
    presc_tcc=TCC_CTRLA_PRESCALER_DIV1024;
    presc_tc=TC_CTRLA_PRESCALER_DIV1024;
  }  
  else if((frequency > 100) & (frequency <= 350))
  {
    div=256.0;
    presc_tcc=TCC_CTRLA_PRESCALER_DIV256;
    presc_tc=TC_CTRLA_PRESCALER_DIV1024;
  }
  else if((frequency > 350) & (frequency <= 1900))
  {
    div=64.0;
    presc_tcc=TCC_CTRLA_PRESCALER_DIV64;
    presc_tc=TC_CTRLA_PRESCALER_DIV1024;
  }
  else if((frequency > 1900) & (frequency <= 5500))
  {
    div=16.0;
    presc_tcc=TCC_CTRLA_PRESCALER_DIV16;
    presc_tc=TC_CTRLA_PRESCALER_DIV1024;
  }
  else if((frequency > 5500) & (frequency <= 30000))
  {
    div=8.0;
    presc_tcc=TCC_CTRLA_PRESCALER_DIV8;
    presc_tc=TC_CTRLA_PRESCALER_DIV1024;
  }
  else;
  
  tim_per=(uint8_t)(floor((8.0e6/(2.0 * div * freq)) - 1.0));
  duty= (uint8_t)(tim_per / 2); 
  pinPeripheral( pin, g_APinDescription[pin].ulPinType ) ;  
    
    if ( isTC )
    {
      // -- Configure TC
      //DISABLE TCx
      TCx->COUNT8.CTRLA.reg &=~(TC_CTRLA_ENABLE);
      //Set Timer counter Mode to 8 bits
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8;
      TCx->COUNT8.CTRLA.reg |= presc_tc;
      //Set TCx as normal FRQ
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ;
      //Set TCx in waveform mode Normal PWM
      TCx->COUNT8.CC[Channelx].reg = duty;
      //Set PER to maximum counter value (resolution : 0xFF)
      TCx->COUNT8.PER.reg = tim_per;
      // Enable TCx
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    }
    else
    {
      // -- Configure TCC

      //DISABLE TCCx
      TCCx->CTRLA.reg &=~(TCC_CTRLA_ENABLE);
      TCCx->CTRLA.reg |=presc_tcc;
      //Set TCx as normal FRQ
      TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;
      //Set TCx in waveform mode Normal PWM
      TCCx->CC[Channelx].reg = duty;  
      //Set PER to maximum counter value (resolution : 0xFF)
      TCCx->PER.reg = tim_per;   
      //ENABLE TCCx
      TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	
	}
	  
#endif
	  return;
}


void noTone(uint8_t pin)
{
#if 0
	if((pin >= 2) & (pin <= 7)) TCC0->CTRLA.reg &=~(TCC_CTRLA_ENABLE);              // disable TCC0
	else if((pin == 8) | (pin == 9)) TCC1->CTRLA.reg &=~(TCC_CTRLA_ENABLE);         // disable TCC1
	else if ((pin == 10) | (pin ==12)) TC3->COUNT8.CTRLA.reg &=~(TC_CTRLA_ENABLE);  // disable TC3
	else if ((pin == 11) | (pin == 13)) TCC2->CTRLA.reg &=~(TCC_CTRLA_ENABLE);      // disable TCC2
	else;

return;	
#endif
}

void set_timer5(void)
{
#if 0
    NVIC_DisableIRQ( TC5_IRQn ) ;
    NVIC_ClearPendingIRQ( TC5_IRQn ) ;
    NVIC_SetPriority( TC5_IRQn, 0 ) ;

	//GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 ));
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 ));
	
	TC5->COUNT8.CTRLA.reg &=~(TC_CTRLA_ENABLE);
    TC5->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV1;
    TC5->COUNT8.INTENSET.reg |= TC_INTENSET_OVF;
    TC5->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    NVIC_EnableIRQ( TC5_IRQn ) ;
#endif
}

void TC5_Handler( void )
{
#if 0
  if((TC5->COUNT16.INTFLAG.reg & TC_INTFLAG_OVF)) 
  {
    
    count_duration--;
    if(count_duration == 0)
    {
      noTone(pin_sound);
      TC5->COUNT8.CTRLA.reg &=~(TC_CTRLA_ENABLE);  // disable TC5
	  
    }
  }	
    TC5->COUNT8.INTFLAG.reg |= ~(TC_INTFLAG_OVF);   // reset flag
#endif   
}  




////////////// 추가됨
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
void PWM0_Handler(void)
{
    uint32_t IntFlag = 0;

//    IntFlag = PWM_CH1->IR/*PWM_CHn_GetIntFlagStatus(PWM_CH1)*/;
    IntFlag = *(volatile unsigned int *)(0x40005000)/*PWM_CH1->IR*//*PWM_CHn_GetIntFlagStatus(PWM_CH1)*/;

    /* If match interrupt is occurred */
    if( (IntFlag & (0x01 << 0)/*PWM_CHn_IER_MI_Msk*/) != 0 )
    {
        /* Clear match interrupt */
//        PWM_CH1_ClearMatchInt(); 
//        PWM_CH1->ICR = PWM_CHn_ICR_MatchInterruptClear/*PWM_CHn_ICR*/;
        *(volatile unsigned int *)(0x40005008) = 0x01 << 0/*PWM_CHn_ICR*/;

        /* LED red is toggled */
        //if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_8) != (uint32_t)Bit_RESET)
        //    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        //else
        //    GPIO_SetBits(GPIOC, GPIO_Pin_8);
        //*(volatile unsigned int *)0x44000000 ^= 1 << 8;   //GPIOx->DATA
//////////////////////////////////////////////        digitalWrite(gTonePinNum, LOW);
//// test
                while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
                *(volatile unsigned long *) 0x4000D000 = 'Z';
    }
    /* If overflow interrupt is occurred */
    if( (IntFlag & (0x01 << 1)/*PWM_CHn_IER_OI_Msk*/) != 0 )
    {
        /* Clear overflow interrupt */
//        PWM_CH1_ClearOverflowInt();
//        PWM_CH1->ICR = PWM_CHn_ICR_OverflowInterruptClear/*PWM_CHn_ICR*/;
        *(volatile unsigned int *)(0x40005008) = (0x01 << 1)/*PWM_CHn_ICR*/;

        /* LED green is toggled */
        //if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9) != (uint32_t)Bit_RESET)
        //    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        //else
        //    GPIO_SetBits(GPIOC, GPIO_Pin_9);
        //*(volatile unsigned int *)0x44000000 ^= 1 << 9;   //GPIOx->DATA
//////////////////////////////////////////////        digitalWrite(gTonePinNum, HIGH);
//// test
                while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
                *(volatile unsigned long *) 0x4000D000 = 'X';
    }
}

void PWM1_Handler(void)
{
    uint32_t IntFlag = 0;

//    IntFlag = PWM_CH1->IR/*PWM_CHn_GetIntFlagStatus(PWM_CH1)*/;
    IntFlag = *(volatile unsigned int *)(0x40005100)/*PWM_CH1->IR*//*PWM_CHn_GetIntFlagStatus(PWM_CH1)*/;

    /* If match interrupt is occurred */
    if( (IntFlag & (0x01 << 0)/*PWM_CHn_IER_MI_Msk*/) != 0 )
    {
        /* Clear match interrupt */
//        PWM_CH1_ClearMatchInt(); 
//        PWM_CH1->ICR = PWM_CHn_ICR_MatchInterruptClear/*PWM_CHn_ICR*/;
        *(volatile unsigned int *)(0x40005108) = 0x01 << 0/*PWM_CHn_ICR*/;

        /* LED red is toggled */
        //if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_8) != (uint32_t)Bit_RESET)
        //    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        //else
        //    GPIO_SetBits(GPIOC, GPIO_Pin_8);
        //*(volatile unsigned int *)0x44000000 ^= 1 << 8;   //GPIOx->DATA
        digitalWrite(gTonePinNum, LOW);
//// test
//                while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
//                *(volatile unsigned long *) 0x4000D000 = 'I';
    }
    /* If overflow interrupt is occurred */
    if( (IntFlag & (0x01 << 1)/*PWM_CHn_IER_OI_Msk*/) != 0 )
    {
        /* Clear overflow interrupt */
//        PWM_CH1_ClearOverflowInt();
//        PWM_CH1->ICR = PWM_CHn_ICR_OverflowInterruptClear/*PWM_CHn_ICR*/;
        *(volatile unsigned int *)(0x40005108) = (0x01 << 1)/*PWM_CHn_ICR*/;

        /* LED green is toggled */
        //if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9) != (uint32_t)Bit_RESET)
        //    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        //else
        //    GPIO_SetBits(GPIOC, GPIO_Pin_9);
        //*(volatile unsigned int *)0x44000000 ^= 1 << 9;   //GPIOx->DATA
        digitalWrite(gTonePinNum, HIGH);
//// test
//                while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
//                *(volatile unsigned long *) 0x4000D000 = 'J';
    }
}

void PWM3_Handler(void)
{
    uint32_t IntFlag = 0;

//    IntFlag = PWM_CH3->IR/*PWM_CHn_GetIntFlagStatus(PWM_CH3)*/;
    IntFlag = *(volatile unsigned int *)(0x40005300)/*PWM_CH3->IR*//*PWM_CHn_GetIntFlagStatus(PWM_CH3)*/;

    /* If match interrupt is occurred */
    if( (IntFlag & (0x01 << 0)/*PWM_CHn_IER_MI_Msk*/) != 0 )
    {
        /* Clear match interrupt */
//        PWM_CH3_ClearMatchInt(); 
//        PWM_CH3->ICR = PWM_CHn_ICR_MatchInterruptClear/*PWM_CHn_ICR*/;
        *(volatile unsigned int *)(0x40005308) = 0x01 << 0/*PWM_CHn_ICR*/;

        /* LED red is toggled */
        //if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_8) != (uint32_t)Bit_RESET)
        //    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        //else
        //    GPIO_SetBits(GPIOC, GPIO_Pin_8);
//        *(volatile unsigned int *)0x44000000 ^= 1 << 8;   //GPIOx->DATA
//// test
                while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
                *(volatile unsigned long *) 0x4000D000 = 'K';
    }
    /* If overflow interrupt is occurred */
    if( (IntFlag & (0x01 << 1)/*PWM_CHn_IER_OI_Msk*/) != 0 )
    {
        /* Clear overflow interrupt */
//        PWM_CH3_ClearOverflowInt();
//        PWM_CH3->ICR = PWM_CHn_ICR_OverflowInterruptClear/*PWM_CHn_ICR*/;
        *(volatile unsigned int *)(0x40005308) = (0x01 << 1)/*PWM_CHn_ICR*/;

        /* LED green is toggled */
        //if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9) != (uint32_t)Bit_RESET)
        //    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        //else
        //    GPIO_SetBits(GPIOC, GPIO_Pin_9);
//        *(volatile unsigned int *)0x44000000 ^= 1 << 9;   //GPIOx->DATA
//// test
                while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
                *(volatile unsigned long *) 0x4000D000 = 'L';
    }
}
////////////// 추가됨 끝
