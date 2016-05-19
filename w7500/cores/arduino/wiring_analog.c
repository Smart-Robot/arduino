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

 //

#include "wiring_analog.h"
#include "wiring_digital.h"

#ifdef __cplusplus
extern "C" {
#endif

static int _readResolution = 10;
static int _writeResolution = 10;
static int ADC_RESOLUTION = 12;
static int DAC_RESOLUTION = 10;

void analogReadResolution(int res) {
#if 0
	_readResolution = res;
	while( ADC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for synchroinization
  }
	if(res == 8) ADC->CTRLB.bit.RESSEL= ADC_CTRLB_RESSEL_8BIT_Val; 
	else if(res == 10) ADC->CTRLB.bit.RESSEL= ADC_CTRLB_RESSEL_10BIT_Val;   
	else ADC->CTRLB.bit.RESSEL= ADC_CTRLB_RESSEL_12BIT_Val; 
#endif
}

void analogWriteResolution(int res) {
	_writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

void analogReference( eAnalogReference ulMode )
{

    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = 'E';

#if 0
  // ATTENTION : On this board the default is not 5volts or 3.3volts BUT 1.65 volt

   
   
   ADC->CTRLA.bit.ENABLE = 0; // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for synchroinization
  }
  switch(ulMode)
  {
    case AR_DEFAULT:
      //default:
      
	  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
	  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
	  break;
    case AR_INTERNAL:
	  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val; 
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;
      break;

    case AR_EXTERNAL:
	  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val; 
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
      break;
  }ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for synchroinization
  }
#endif
}


///////////// 추가된 함수 시작
void ADC_CH_Init(uint8_t pin)
{
  switch(pin)
  {
   case '7':    //  ADC7
    *(volatile unsigned int *)0x44000014 |= 1 << 8;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 8)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 8)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;

   case '6':    //  ADC6
    *(volatile unsigned int *)0x44000014 |= 1 << 9;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 9)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 9)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;

   case '5':    //  ADC5
    *(volatile unsigned int *)0x44000014 |= 1 << 10;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 10)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 10)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;

   case '4':    //  ADC4
    *(volatile unsigned int *)0x44000014 |= 1 << 11;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 11)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 11)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;

   case '3':    //  ADC3
    *(volatile unsigned int *)0x44000014 |= 1 << 12;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 12)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 12)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;

   case '2':    //  ADC2
    *(volatile unsigned int *)0x44000014 |= 1 << 13;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 13)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 13)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;

   case '1':    //  ADC1
    *(volatile unsigned int *)0x44000014 |= 1 << 14;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 14)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 14)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;

   case '0':    //  ADC0
    *(volatile unsigned int *)0x44000014 |= 1 << 15;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 15)) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + (4 * 15)) |=  0x03/*PAD_AF1*/;	// (0x01 <<  8)
    break;
  }
	// ADC_CLK on
	//ADC_PowerDownEnable(DISABLE);

//	if (NewState != DISABLE)    *(volatile unsigned int *)0x41000000/*ADC->ADC_CTR*/ = 0x3ul/*ADC_CTR_PWD_PD*/;
//	else                        *(volatile unsigned int *)0x41000000/*ADC->ADC_CTR*/ = 0x1ul/*ADC_CTR_PWD_NRMOP*/;
  *(volatile unsigned int *)0x41000000/*ADC->ADC_CTR*/ = 0x1/*ADC_CTR_PWD_NRMOP*/;
}
///////////// 추가된 함수 끝

uint32_t analogRead( uint32_t ulPin )
{
    ADC_CH_Init(ulPin);

    //ADC_ChannelSelect (num); ///< Select ADC channel to CH0
    *(volatile unsigned int *)0x41000004/*ADC->ADC_CHSEL*/ = ulPin/*num*/;

    //ADC_Start(); ///< Start ADC
    *(volatile unsigned int *)0x41000008/*ADC->ADC_START*/ = 0x1ul/*ADC_START_START*/;

    //while(ADC_IsEOC()); ///< Wait until End of Conversion
    while(  (((unsigned char)*(volatile unsigned int *)0x41000010/*ADC->ADC_INT*/ && 0x01ul))  ); ///< Wait until End of Conversion
    //return ((uint16_t)ADC_ReadData()); ///< read ADC Data
    return (((unsigned short) *(volatile unsigned int *)0x4100000C/*ADC->ADC_DATA*/   ) >> 2); ///< read ADC Data


#if 0
  uint32_t valueRead = 0;
  pinPeripheral(ulPin, g_APinDescription[ulPin].ulPinType);
  if ( ulPin == 24 )  // Only 1 DAC on A0 (PA02)
  {
    DAC->CTRLA.bit.ENABLE = 0; //disable DAC on A0
  }
  
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber;

  // Start conversion
  ADC->SWTRIG.bit.START = 1;

  while( ADC->INTFLAG.bit.RESRDY == 0 || ADC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for a complete conversion and complete synchronization
  }

  // Store the value
  valueRead = ADC->RESULT.reg;

  // Clear the Data Ready flag
  ADC->INTFLAG.bit.RESRDY = 1;

  // Flush the ADC for further conversions
  // ADC->SWTRIG.bit.FLUSH = 1;

  while( ADC->STATUS.bit.SYNCBUSY == 1 || ADC->SWTRIG.bit.FLUSH == 1 )
  {
    // Waiting for synchronization
  }
  
  //valueRead = mapResolution(valueRead, ADC_RESOLUTION, _readResolution);
  return valueRead;
#endif  
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite( uint32_t ulPin, uint32_t ulValue )
{
#if 0 // pwm test
#if 1 // port configuration
    *(volatile unsigned int *)0x44000010 |= 1 << 8;   //GPIOx->OUTENSET

    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002080 + 0x20) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
//    *(volatile unsigned int *)(0x41002080 + 0x20) |= 0x01/*PAD_AF1*/;	// (0x01 <<  8)

    *(volatile unsigned int *)0x44000010 |= 1 << 9;   //GPIOx->OUTENSET

    *(volatile unsigned int *)(0x41002080 + 0x24) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
//    *(volatile unsigned int *)(0x41002080 + 0x24) |= 0x01/*PAD_AF1*/;	// (0x01 <<  8)
#endif // port configuration
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
    /* Select Timer/Counter mode as Timer mode */ 
    //PWM_CHn->TCMR = PWM_CHn_TCMR_TimerMode;                      
//    *(volatile unsigned int *)0x40005324 = (0x0ul);  // ch3
    *(volatile unsigned int *)0x40005024 = (0x0);
    /* Set Prescale register value */
    //PWM_CHn->PR = PWM_TimerModeInitStruct->PWM_CHn_PR;        
//    *(volatile unsigned int *)0x40005314 = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    *(volatile unsigned int *)0x40005014 = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    /* Set Match register value */
    //PWM_CHn->MR = PWM_TimerModeInitStruct->PWM_CHn_MR;        
//    *(volatile unsigned int *)0x40005318 = 80000;
    *(volatile unsigned int *)0x40005018 = 80000;
    /* Set Limit register value */
    //PWM_CHn->LR = PWM_TimerModeInitStruct->PWM_CHn_LR;         
//    *(volatile unsigned int *)0x4000531C = 100000; // 80% duty cycle
    *(volatile unsigned int *)0x4000501C = 100000; // 80% duty cycle
    /* Select Up-down mode */
    //PWM_CHn->UDMR = PWM_TimerModeInitStruct->PWM_CHn_UDMR;     
//    *(volatile unsigned int *)0x40005320 = (0x0ul);        //PWM_CHn_UDMR_UpCount
    *(volatile unsigned int *)0x40005020 = (0x0);        //PWM_CHn_UDMR_UpCount
    /* Select Periodic mode */ 
    //PWM_CHn->PDMR = PWM_TimerModeInitStruct->PWM_CHn_PDMR;     
//    *(volatile unsigned int *)0x40005334 = (0x1ul);        //PWM_CHn_PDMR_Periodic
    *(volatile unsigned int *)0x40005034 = (0x1);        //PWM_CHn_PDMR_Periodic

//------------------------------------------------------------------------------------------
    /* Select Timer/Counter mode as Timer mode */ 
    //PWM_CHn->TCMR = PWM_CHn_TCMR_TimerMode;                      
//    *(volatile unsigned int *)0x40005324 = (0x0ul);  // ch3
    *(volatile unsigned int *)0x40005124 = (0x0ul);
    /* Set Prescale register value */
    //PWM_CHn->PR = PWM_TimerModeInitStruct->PWM_CHn_PR;        
//    *(volatile unsigned int *)0x40005314 = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    *(volatile unsigned int *)0x40005114 = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    /* Set Match register value */
    //PWM_CHn->MR = PWM_TimerModeInitStruct->PWM_CHn_MR;        
//    *(volatile unsigned int *)0x40005318 = 80000;
    *(volatile unsigned int *)0x40005118 = 80000;
    /* Set Limit register value */
    //PWM_CHn->LR = PWM_TimerModeInitStruct->PWM_CHn_LR;         
//    *(volatile unsigned int *)0x4000531C = 100000; // 80% duty cycle
    *(volatile unsigned int *)0x4000511C = 100000; // 80% duty cycle
    /* Select Up-down mode */
    //PWM_CHn->UDMR = PWM_TimerModeInitStruct->PWM_CHn_UDMR;     
//    *(volatile unsigned int *)0x40005320 = (0x0ul);        //PWM_CHn_UDMR_UpCount
    *(volatile unsigned int *)0x40005120 = (0x0ul);        //PWM_CHn_UDMR_UpCount
    /* Select Periodic mode */ 
    //PWM_CHn->PDMR = PWM_TimerModeInitStruct->PWM_CHn_PDMR;     
//    *(volatile unsigned int *)0x40005334 = (0x1ul);        //PWM_CHn_PDMR_Periodic
    *(volatile unsigned int *)0x40005134 = (0x1ul);        //PWM_CHn_PDMR_Periodic

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
        //PWM->SSR &= PWM_SSR_SS3_Stop;
//        *(volatile unsigned int *)0x40005804 &= ~(0x1ul << 3);
        *(volatile unsigned int *)0x40005804 &= ~(0x1ul << 0);
        //PWM_CHn->PEEER = outputEnDisable; 
//        *(volatile unsigned int *)0x40005328 = (0x2ul);
        *(volatile unsigned int *)0x40005028 = (0x2ul);

//------------------------------------------------------------------------------------------
        //PWM->SSR &= PWM_SSR_SS3_Stop;
//        *(volatile unsigned int *)0x40005804 &= ~(0x1ul << 3);
        *(volatile unsigned int *)0x40005804 &= ~(0x1ul << 1);
        //PWM_CHn->PEEER = outputEnDisable; 
//        *(volatile unsigned int *)0x40005328 = (0x2ul);
        *(volatile unsigned int *)0x40005128 = (0x2ul);

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

        //PWM->SSR |= PWM_SSR_SS3_Start;
//        *(volatile unsigned int *)0x40005804 |= (0x1ul << 3);
        *(volatile unsigned int *)0x40005804 |= (0x1ul << 0);

//------------------------------------------------------------------------------------------
        //PWM->SSR |= PWM_SSR_SS3_Start;
//        *(volatile unsigned int *)0x40005804 |= (0x1ul << 3);
        *(volatile unsigned int *)0x40005804 |= (0x1ul << 1);




    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = 'P';
    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = 'W';
    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = 'M';
while(1);
#endif

#if 1
//------------------------------------------------------------------------------------------
    *(volatile unsigned int *)(0x42000010 + pwm_pin_tbl[ulPin].port_num) |= 1 << pwm_pin_tbl[ulPin].pin_num;   //GPIOx->OUTENSET
    //PAD_AFConfig(PAD_PC, GPIO_Pin_8, 0x01/*PAD_AF1*/); ///< PAD Config - LED used 2nd Function	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002000 + pwm_pin_tbl[ulPin].af_base + pwm_pin_tbl[ulPin].pin_num * 4) &= ~0x03/*PAD_AF1*/;	// (0x01 <<  8)
    *(volatile unsigned int *)(0x41002000 + pwm_pin_tbl[ulPin].af_base + pwm_pin_tbl[ulPin].pin_num * 4) |= pwm_pin_tbl[ulPin].af_num/*PAD_AF1*/;	// (0x01 <<  8)
//------------------------------------------------------------------------------------------
//PWM-n
    /* Select Timer/Counter mode as Timer mode */ 
    *(volatile unsigned int *)(0x40005024 + pwm_pin_tbl[ulPin].pwm_num) = (0x0);
    /* Set Prescale register value */
    *(volatile unsigned int *)(0x40005014 + pwm_pin_tbl[ulPin].pwm_num) = (20000000 / 1000000) / 10 - 1; //PrescalerValue - 1;    
    /* Set Match register value */
    *(volatile unsigned int *)(0x40005018 + pwm_pin_tbl[ulPin].pwm_num) = 400 * ulValue;     //MR
    /* Set Limit register value */
    *(volatile unsigned int *)(0x4000501C + pwm_pin_tbl[ulPin].pwm_num) = 102400; // 80% duty cycle
    /* Select Up-down mode */
    *(volatile unsigned int *)(0x40005020 + pwm_pin_tbl[ulPin].pwm_num) = (0x0);        //PWM_CHn_UDMR_UpCount
    /* Select Periodic mode */ 
    *(volatile unsigned int *)(0x40005034 + pwm_pin_tbl[ulPin].pwm_num) = (0x1);        //PWM_CHn_PDMR_Periodic

//------------------------------------------------------------------------------------------
        //PWM->SSR &= PWM_SSR_SS3_Stop;
        *(volatile unsigned int *)0x40005804 &= ~(0x1 << pwm_pin_tbl[ulPin].pwm_pin);
        //PWM_CHn->PEEER = outputEnDisable; 
        *(volatile unsigned int *)(0x40005028 + pwm_pin_tbl[ulPin].pwm_num) = 0x2;

//------------------------------------------------------------------------------------------
        //PWM->SSR |= PWM_SSR_SS3_Start;
        *(volatile unsigned int *)0x40005804 |= (0x1ul << pwm_pin_tbl[ulPin].pwm_pin);

#if 0  // pwm serial character out
    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = 'P';
    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = 'W';
    while( !((*(volatile unsigned long *) 0x4000D018) & (0x01 << 7)) ) ;
    *(volatile unsigned long *) 0x4000D000 = 'M';
#endif  // pwm serial character out

#endif













#if 0
  uint32_t attr = g_APinDescription[ulPin].ulPinAttribute ;
//   uint32_t pwm_name = g_APinDescription[ulPin].ulTCChannel ;
  uint8_t isTC = 0 ;
  uint8_t Channelx ;
  Tc* TCx ;
  Tcc* TCCx ;

  if ( (attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG )
  {
    if ( ulPin == 24 )  // Only 1 DAC on A0 (PA02)
    {
      	ulValue = mapResolution(ulValue, _writeResolution, DAC_RESOLUTION);
    	DAC->DATA.reg = ulValue & 0x3FF;  // Dac on 10 bits.
		DAC->CTRLA.bit.ENABLE = 1; // DAC Enabled
/////////////////////////////////////		syncDAC();
      	return;
    }
	

  }

  if ( (attr & PIN_ATTR_PWM) == PIN_ATTR_PWM )
  {
    if ( (g_APinDescription[ulPin].ulPinType == PIO_TIMER) || g_APinDescription[ulPin].ulPinType == PIO_TIMER_ALT )
    {
      pinPeripheral( ulPin, g_APinDescription[ulPin].ulPinType ) ;
    }

    switch ( g_APinDescription[ulPin].ulPWMChannel )
    {
      case PWM3_CH0 :
                TCx = TC3 ;
                Channelx = 0 ;
                isTC = 1 ;
            break;

      case  PWM3_CH1:
                TCx = TC3 ;
                Channelx = 1;
                isTC = 1;
            break;

      case  PWM0_CH0 :
                TCCx = TCC0;
                Channelx = 0;
            break;

      case  PWM0_CH1 :
                TCCx = TCC0;
                Channelx = 1;
            break;

      case  PWM0_CH4 :
                TCCx = TCC0;
                Channelx = 0;
            break;

      case  PWM0_CH5 :
                TCCx = TCC0;
                Channelx = 1;
            break;

      case  PWM0_CH6 :
                TCCx = TCC0;
                Channelx = 2;
            break;

      case  PWM0_CH7 :
                TCCx = TCC0;
                Channelx = 3;
            break;

      case  PWM1_CH0 :
                TCCx = TCC1;
                Channelx = 0;
            break;

      case  PWM1_CH1 :
                TCCx = TCC1;
                Channelx = 1;
            break;

      case  PWM2_CH0 :
                TCCx = TCC2;
                Channelx = 0;
            break;

      case  PWM2_CH1 :
                TCCx = TCC2;
                Channelx = 1;
            break;
    }


    // Enable clocks according to TCCx instance to use
    switch ( GetTCNumber( g_APinDescription[ulPin].ulPWMChannel ) )
    {
      case 0: // TCC0
                //Enable GCLK for TCC0 (timer counter input clock)
                GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
            break;

      case 1: // TCC1
                //Enable GCLK for TCC1 (timer counter input clock)
                GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
            break;

      case 2: // TCC2
                //Enable GCLK for TCC2 (timer counter input clock)
                GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 )) ;
            break;

      case 3: // TC3
                //Enable GCLK for TC3 (timer counter input clock)
                GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 ));
            break;

      case 4: // TC4
                //Enable GCLK for TC4 (timer counter input clock)
                GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 ));
            break;

      case 5: // TC5
                //Enable GCLK for TC5 (timer counter input clock)
                GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
            break;
    }

    // Set PORT
    if ( isTC )
    {
      // -- Configure TC
      //DISABLE TCx
      TCx->COUNT8.CTRLA.reg &=~(TC_CTRLA_ENABLE);
      //Set Timer counter Mode to 8 bits
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8;
      //Set TCx as normal PWM
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
      //Set TCx in waveform mode Normal PWM
      TCx->COUNT8.CC[Channelx].reg = (uint8_t) ulValue;
      //Set PER to maximum counter value (resolution : 0xFF)
      TCx->COUNT8.PER.reg = 0xFF;
      // Enable TCx
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    }
    else
    {
      // -- Configure TCC

      //DISABLE TCCx
      TCCx->CTRLA.reg &=~(TCC_CTRLA_ENABLE);
      //Set TCx as normal PWM
      TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
      //Set TCx in waveform mode Normal PWM
      TCCx->CC[Channelx].reg = (uint32_t)ulValue;
      //Set PER to maximum counter value (resolution : 0xFF)
      TCCx->PER.reg = 0xFF;
      //ENABLE TCCx
      TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
    }

    return ;
  }

  // -- Defaults to digital write
  pinMode( ulPin, OUTPUT ) ;

  if ( ulValue < 128 )
  {
    digitalWrite( ulPin, LOW ) ;
  }
  else
  {
    digitalWrite( ulPin, HIGH ) ;
  }
#endif

}

#ifdef __cplusplus
}
#endif
