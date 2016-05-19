/*
 *********************************************************************
 * @file    : wztoe.c
 * @version : 0.0.1
 * @author  : WIZnet
 * @data    
 * @brief   : wztoe dirver for W7500
 *********************************************************************
 * @attention
 */
//#include "lstring.h"
//#include "W7500x_wztoe.h"
#include "Arduino.h"

//-----------------------------------------
// wztoe utils 
//-----------------------------------------
//#define htole32(x) \
//	(((((uint32_t)x) & 0xff000000) >> 24) | \
//	 ((((uint32_t)x) & 0x00ff0000) >> 8) | \
//	 ((((uint32_t)x) & 0x0000ff00) << 8) | \
//	 ((((uint32_t)x) & 0x000000ff) << 24))
//
//#define htole16(x) \
//	(((((uint16_t)x) & 0xff00) >> 8) | \
//	 ((((uint16_t)x) & 0x00ff) << 8)) 
/////////////////////////////////
// Common Register I/O function //
/////////////////////////////////
/**
 * @ingroup Version_register_access_function
 * @brief Get Version Register
 * @return uint8_t. The value of Version register.
 * @sa setVERSIONR()
 */
uint8_t getVERSIONR(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_VERSIONR);
}
/**
 * @ingroup Common_register_access_function
 * @brief Get 100US Register
 * @return uint16_t. The value of 100US register.
 * @sa set100US()
 * @WZTOE_TIC100US - 0x46002000
 */
uint16_t getTIC100US(void)
{
	  //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_TIC100US)));
}
/**
 * @ingroup Common_register_access_function
 * @brief Set 100US Register
 * @param (uint8_t)tic The value to be set.
 * @sa get100US()
 * @WZTOE_TIC100US - 0x46002000
 */
void setTIC100US(uint16_t tic)
{
	  //w_printf("%s : WZTOE_TIC100US[0x%x] = tic[0x%x]\n", __func__, WZTOE_TIC100US , tic);
  (*(volatile uint32_t *)(WZTOE_TIC100US) = tic);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref IR register
 * @param (uint8_t)ir Value to set @ref IR register.
 * @sa getIR()
 */
/*
void setIR(uint8_t ir)
{
		WIZCHIP_WRITE(WZTOE_IR, (ir & 0xF0));
}
*/

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref IR register
 * @return uint8_t. Value of @ref IR register.
 * @sa setIR()
 */
uint8_t getIR(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_IR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref IMR register
 * @param (uint8_t)imr Value to set @ref IMR register.
 * @sa getIMR()
 */
void setIMR(uint8_t imr)
{
	  //w_printf("%s : WZTOE_IMR[0x%x] = imr[0x%x]\n", __func__, WZTOE_IMR , (imr&0xF0));
    WIZCHIP_WRITE(WZTOE_IMR, (imr&0xF0));
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref IMR register
 * @return uint8_t. Value of @ref IMR register.
 * @sa setIMR()
 */
uint8_t getIMR(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_IMR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref ICR register
 * @param (uint8_t)icr Value to set @ref ICR register.
 * @sa getICR()
 */
void setICR(uint8_t icr)
{
	  //w_printf("%s : WZTOE_ICR[0x%x] = icr[0x%x]\n", __func__, WZTOE_ICR , icr&0xF0);
    WIZCHIP_WRITE(WZTOE_ICR, icr&0xF0);
}

void setIR(uint8_t ir)
{
	  //w_printf("%s : WZTOE_ICR[0x%x] = ir[0x%x]\n", __func__, WZTOE_ICR , ir&0xF0);
    setICR(ir);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref ICR register
 * @return uint8_t. Value of @ref ICR register.
 * @sa setICR()
 */
uint8_t getICR(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_ICR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref SIR register
 * @param (uint8_t)sir Value to set @ref SIR register.
 * @sa getSIR()
 */
void setSIR(uint8_t sir)
{
	  //w_printf("%s : WZTOE_SIR[0x%x] = sir[0x%x]\n", __func__, WZTOE_SIR , sir);
    WIZCHIP_WRITE(WZTOE_SIR, sir);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref SIR register
 * @return uint8_t. Value of @ref SIR register.
 * @sa setSIR()
 */
uint8_t getSIR(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_SIR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref SIMR register
 * @param (uint8_t)simr Value to set @ref SIMR register.
 * @sa getSIMR()
 */
void setSIMR(uint8_t simr)
{
	  //w_printf("%s : WZTOE_SIMR[0x%x] = simr[0x%x]\n", __func__, WZTOE_SIMR , simr);
    WIZCHIP_WRITE(WZTOE_SIMR, simr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref SIMR register
 * @return uint8_t. Value of @ref SIMR register.
 * @sa setSIMR()
 */
uint8_t getSIMR(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_SIMR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set INTLEVEL register
 * @param (uint16_t)intlevel Value to set @ref INTLEVEL register.
 * @sa getINTLEVEL()
 */
void setINTLEVEL(uint16_t intlevel)
{
	  //w_printf("%s : WZTOE_INTLEVEL[0x%x] = intlevel[0x%x]\n", __func__, WZTOE_INTLEVEL , intlevel);
    (*(volatile uint32_t *)(WZTOE_INTLEVEL) = intlevel);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get INTLEVEL register
 * @return uint16_t. Value of @ref INTLEVEL register.
 * @sa setINTLEVEL()
 */
uint16_t getINTLEVEL(void)
{
	  //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_INTLEVEL)));
}

/**
 * @ingroup Common_register_access_function
 * @brief Get Mode Register
 * @return uint8_t. The value of Mode register.
 * @sa setMR()
 */
uint8_t getMR(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_MR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set Mode Register
 * @param (uint8_t)mr The value to be set.
 * @sa getMR()
 */
void setMR(uint8_t mr)
{
	  //w_printf("%s : WZTOE_MR[0x%x] = mr[0x%x]\n", __func__, WZTOE_MR , mr);
    WIZCHIP_WRITE(WZTOE_MR, mr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get Mode Register
 * @return uint8_t. The value of Mode register.
 * @sa setMR()
 */
/*
uint8_t getMR1(void)
{
    return WIZCHIP_READ(WZTOE_MR1);
}
*/

/**
 * @ingroup Common_register_access_function
 * @brief Set Mode Register
 * @param (uint8_t)mr The value to be set.
 * @sa getMR()
 */
void setMR1(uint8_t mr)
{
	  //w_printf("%s : WZTOE_MR[0x%x] = mr[0x%x]\n", __func__, WZTOE_MR , mr&0x2);
    WIZCHIP_WRITE(WZTOE_MR, mr&0x2);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get Mode Register
 * @return uint8_t. The value of Mode register.
 * @sa setMR()
 */
uint8_t getMR1(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_MR1);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PTIMER register
 * @param (uint8_t)ptimer Value to set @ref PTIMER register.
 * @sa getPTIMER()
 */
void setPTIMER(uint8_t ptimer)
{
	  //w_printf("%s : WZTOE_PTIMER[0x%x] = ptimer[0x%x]\n", __func__, WZTOE_PTIMER , ptimer);
    WIZCHIP_WRITE(WZTOE_PTIMER, ptimer);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PTIMER register
 * @return uint8_t. Value of @ref PTIMER register.
 * @sa setPTIMER()
 */
uint8_t getPTIMER(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_PTIMER);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PMAGIC register
 * @param (uint8_t)pmagic Value to set @ref PMAGIC register.
 * @sa getPMAGIC()
 */
void setPMAGIC(uint8_t pmagic)
{
	  //w_printf("%s : WZTOE_PMAGIC[0x%x] = pmagic[0x%x]\n", __func__, WZTOE_PMAGIC , pmagic);
    WIZCHIP_WRITE(WZTOE_PMAGIC, pmagic);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PMAGIC register
 * @return uint8_t. Value of @ref PMAGIC register.
 * @sa setPMAGIC()
 */
uint8_t getPMAGIC(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_PMAGIC);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set PHAR address
 * @param (uint8_t*)phar Pointer variable to set PPP destination MAC register address. It should be allocated 6 bytes.
 * @sa getPHAR()
 */
void setPHAR(uint8_t* phar)
{
	  //w_printf("%s\n", __func__);
    WIZCHIP_WRITE(WZTOE_PHAR+3, phar[0]);
    WIZCHIP_WRITE(WZTOE_PHAR+2, phar[1]);
    WIZCHIP_WRITE(WZTOE_PHAR+1, phar[2]);
    WIZCHIP_WRITE(WZTOE_PHAR+0, phar[3]);
    WIZCHIP_WRITE(WZTOE_PHAR+7, phar[4]);
    WIZCHIP_WRITE(WZTOE_PHAR+6, phar[5]); 
}

/**
 * @ingroup Common_register_access_function
 * @brief Get local IP address
 * @param (uint8_t*)phar Pointer variable to PPP destination MAC register address. It should be allocated 6 bytes.
 * @sa setPHAR()
 */
void getPHAR(uint8_t* phar)
{
	  //w_printf("%s\n", __func__);
    phar[0] = WIZCHIP_READ(WZTOE_PHAR+3);
    phar[1] = WIZCHIP_READ(WZTOE_PHAR+2);
    phar[2] = WIZCHIP_READ(WZTOE_PHAR+1);
    phar[3] = WIZCHIP_READ(WZTOE_PHAR+0);
    phar[4] = WIZCHIP_READ(WZTOE_PHAR+7);
    phar[5] = WIZCHIP_READ(WZTOE_PHAR+6); 
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PSID register
 * @param (uint16_t)psid Value to set @ref PSID register.
 * @sa getPSID()
 */
void setPSID(uint16_t psid)
{
	  //w_printf("%s : WZTOE_PSID[0x%x] = psid[0x%x]\n", __func__, WZTOE_PSID , psid);
    WIZCHIP_WRITE(WZTOE_PSID, psid);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PSID register
 * @return uint16_t. Value of @ref PSID register.
 * @sa setPSID()
 */
uint16_t getPSID(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_PSID);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PMRU register
 * @param (uint16_t)pmru Value to set @ref PMRU register.
 * @sa setPMRU()
 */
void setPMRU(uint16_t pmru)
{
	  //w_printf("%s : WZTOE_PMRU[0x%x] = pmru[0x%x]\n", __func__, WZTOE_PMRU , pmru);
    WIZCHIP_WRITE(WZTOE_PMRU, pmru);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PMRU register
 * @return uint16_t. Value of @ref PMRU register.
 * @sa setPMRU()
 */
uint16_t getPMRU(void)
{
	  //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_PMRU);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set local MAC address
 * @param (uint8_t*)shar Pointer variable to set local MAC address. It should be allocated 6 bytes.
 * @sa getSHAR()
 */
void setSHAR(uint8_t* shar)
{
	  //w_printf("%s\n", __func__);
    WIZCHIP_WRITE(WZTOE_SHAR+3, shar[0]);
    WIZCHIP_WRITE(WZTOE_SHAR+2, shar[1]);
    WIZCHIP_WRITE(WZTOE_SHAR+1, shar[2]);
    WIZCHIP_WRITE(WZTOE_SHAR+0, shar[3]);
    WIZCHIP_WRITE(WZTOE_SHAR+7, shar[4]);
    WIZCHIP_WRITE(WZTOE_SHAR+6, shar[5]); 
}

/**
 * @ingroup Common_register_access_function
 * @brief Get local MAC address
 * @param (uint8_t*)shar Pointer variable to get local MAC address. It should be allocated 6 bytes.
 * @sa setSHAR()
 */
void getSHAR(uint8_t* shar)
{
	  //w_printf("%s\n", __func__);
    shar[0] = WIZCHIP_READ(WZTOE_SHAR+3);
    shar[1] = WIZCHIP_READ(WZTOE_SHAR+2);
    shar[2] = WIZCHIP_READ(WZTOE_SHAR+1);
    shar[3] = WIZCHIP_READ(WZTOE_SHAR+0);
    shar[4] = WIZCHIP_READ(WZTOE_SHAR+7);
    shar[5] = WIZCHIP_READ(WZTOE_SHAR+6); 
}

/**
 * @ingroup Common_register_access_function
 * @brief Set gateway IP address
 * @param (uint8_t*)gar Pointer variable to set gateway IP address. It should be allocated 4 bytes.
 * @sa getGAR()
 */
void setGAR(uint8_t* gar)
{
	  //w_printf("%s\n", __func__);
    WIZCHIP_WRITE((WZTOE_GAR+3), gar[0]);
    WIZCHIP_WRITE((WZTOE_GAR+2), gar[1]);
    WIZCHIP_WRITE((WZTOE_GAR+1), gar[2]);
    WIZCHIP_WRITE((WZTOE_GAR  ), gar[3]); 
}

/**
 * @ingroup Common_register_access_function
 * @brief Get gateway IP address
 * @param (uint8_t*)gar Pointer variable to get gateway IP address. It should be allocated 4 bytes.
 * @sa setGAR()
 */
void getGAR(uint8_t* gar)
{
	  //w_printf("%s\n", __func__);
    gar[0] = WIZCHIP_READ((WZTOE_GAR+3));
    gar[1] = WIZCHIP_READ((WZTOE_GAR+2));
    gar[2] = WIZCHIP_READ((WZTOE_GAR+1));
    gar[3] = WIZCHIP_READ((WZTOE_GAR+0));
}

/**
 * @ingroup Common_register_access_function
 * @brief Set subnet mask address
 * @param (uint8_t*)subr Pointer variable to set subnet mask address. It should be allocated 4 bytes.
 * @sa getSUBR()
 */
void setSUBR(uint8_t* subr)
{
	  //w_printf("%s\n", __func__);
    WIZCHIP_WRITE((WZTOE_SUBR+3), subr[0]);
    WIZCHIP_WRITE((WZTOE_SUBR+2), subr[1]);
    WIZCHIP_WRITE((WZTOE_SUBR+1), subr[2]);
    WIZCHIP_WRITE((WZTOE_SUBR  ), subr[3]); 
}

/**
 * @ingroup Common_register_access_function
 * @brief Get subnet mask address
 * @param (uint8_t*)subr Pointer variable to get subnet mask address. It should be allocated 4 bytes.
 * @sa setSUBR()
 */
void getSUBR(uint8_t* subr)
{
	  //w_printf("%s\n", __func__);
    subr[0] = WIZCHIP_READ((WZTOE_SUBR+3));
    subr[1] = WIZCHIP_READ((WZTOE_SUBR+2));
    subr[2] = WIZCHIP_READ((WZTOE_SUBR+1));
    subr[3] = WIZCHIP_READ((WZTOE_SUBR+0));
}

/**
 * @ingroup Common_register_access_function
 * @brief Set local IP address
 * @param (uint8_t*)sipr Pointer variable to set local IP address. It should be allocated 4 bytes.
 * @sa setSIPR()
 */
void setSIPR(uint8_t* sipr)
{
	  //w_printf("%s\n", __func__);
    WIZCHIP_WRITE((WZTOE_SIPR+3), sipr[0]);
    WIZCHIP_WRITE((WZTOE_SIPR+2), sipr[1]);
    WIZCHIP_WRITE((WZTOE_SIPR+1), sipr[2]);
    WIZCHIP_WRITE((WZTOE_SIPR  ), sipr[3]); 
}

/**
 * @ingroup Common_register_access_function
 * @brief Get local IP address
 * @param (uint8_t*)sipr Pointer variable to get local IP address. It should be allocated 4 bytes.
 * @sa setSIPR()
 */
uint8_t getSIPR(uint8_t* sipr)
{
	  //w_printf("%s\n", __func__);
    sipr[0] = WIZCHIP_READ((WZTOE_SIPR+3));
    sipr[1] = WIZCHIP_READ((WZTOE_SIPR+2));
    sipr[2] = WIZCHIP_READ((WZTOE_SIPR+1));
    sipr[3] = WIZCHIP_READ((WZTOE_SIPR+0));
	  //w_printf("%s : %d.%d.%d.%d \n", __func__, sipr[0], sipr[1], sipr[2], sipr[3]);
}

/**
 * @ingroup Common_register_access_function
 * @brief lock NETCFGLOCK 
 * @sa lockNETCFGLOCK()
 */
void lockNETCFGLOCK(void)
{
	  //w_printf("%s : WZTOE_NETCFGLOCK[0x%x] = [0x%x]\n", __func__, WZTOE_NETCFGLOCK , 0x00);
    (*(volatile uint32_t *)(WZTOE_NETCFGLOCK) = 0x00);
}

/**
 * @ingroup Common_register_access_function
 * @brief unlock NETCFGLOCK
 * @sa unlockNETCFGLOCK()
 */
void unlockNETCFGLOCK32(void)
{
	  //w_printf("%s : WZTOE_NETCFGLOCK[0x%x] = [0x%x]\n", __func__, WZTOE_NETCFGLOCK , 0x1ACCE55);
    (*(volatile uint32_t *)(WZTOE_NETCFGLOCK) = 0x1ACCE55);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref RTR register
 * @param (uint16_t)rtr Value to set @ref RTR register.
 * @sa setRTR()
 */
void setRTR(uint16_t rtr)
{
	  //w_printf("%s : WZTOE_RTR[0x%x] = rtr[0x%x]\n", __func__, WZTOE_RTR , rtr);
    (*(volatile uint32_t *)(WZTOE_RTR) = rtr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref RTR register
 * @return uint16_t. Value of @ref RTR register.
 * @sa getRTR()
 */
uint16_t getRTR(void)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_RTR)));
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref RCR register
 * @param (uint8_t)rcr Value to set @ref RCR register.
 * @sa getRCR()
 */
void setRCR(uint16_t rcr)
{
	  //w_printf("%s : WZTOE_RCR[0x%x] = rcr[0x%x]\n", __func__, WZTOE_RCR , rcr);
    WIZCHIP_WRITE(WZTOE_RCR, rcr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref RCR register
 * @return uint8_t. Value of @ref RCR register.
 * @sa setRCR()
 */
uint16_t getRCR(void)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_RCR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get unreachable IP address
 * @param (uint8_t*)uipr Pointer variable to get unreachable IP address. It should be allocated 4 bytes.
 */
//uint8_t getUIPR(uipr) (*(volatile uint32_t *)(uipr) = htole32(*(volatile uint32_t *)(UIPR)))
void getUIPR(uint8_t* uipr)
{
    //w_printf("%s\n", __func__);
     uipr[0] = WIZCHIP_READ((WZTOE_UIPR+3));
     uipr[1] = WIZCHIP_READ((WZTOE_UIPR+2));
     uipr[2] = WIZCHIP_READ((WZTOE_UIPR+1));
     uipr[3] = WIZCHIP_READ((WZTOE_UIPR));
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref UPORTR register
 * @return uint16_t. Value of @ref UPORTR register.
 */
uint16_t getUPORTR(void)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_UPORTR)));
}
/////////////////////////////////////

///////////////////////////////////
// Socket N register I/O function //
///////////////////////////////////
/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)mr Value to set @ref Sn_MR
 * @sa setSn_MR()
 */
void setSn_MR(uint8_t sn, uint8_t mr)
{
	  //w_printf("%s : WZTOE_Sn_MR(sn)[0x%x] = mr[0x%x]\n", __func__, WZTOE_Sn_MR(sn) , mr);
    WIZCHIP_WRITE(WZTOE_Sn_MR(sn), mr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_MR.
 * @sa getSn_MR()
 */
uint8_t getSn_MR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_MR(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_CR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)cr Value to set @ref Sn_CR
 * @sa getSn_CR()
 */
void setSn_CR(uint8_t sn, uint8_t cr)
{
	  //w_printf("%s : WZTOE_Sn_CR(sn)[0x%x] = cr[0x%x]\n", __func__, WZTOE_Sn_CR(sn) , cr);
    WIZCHIP_WRITE(WZTOE_Sn_CR(sn), cr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_CR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_CR.
 * @sa setSn_CR()
 */
uint8_t getSn_CR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_CR(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_IR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)ir Value to set @ref Sn_IR
 * @sa getSn_IR()
 */
/*
void setSn_IR(uint8_t sn, uint8_t ir)
{
		WIZCHIP_WRITE(WZTOE_Sn_IR(sn), (ir & 0x1F));
}
*/

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_IR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_IR.
 * @sa setSn_IR()
 */
uint8_t getSn_IR(uint8_t sn)
{
//    //w_printf("%s\n", __func__);
    return (WIZCHIP_READ(WZTOE_Sn_ISR(sn)) & 0x1F);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_IMR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)imr Value to set @ref Sn_IMR
 * @sa getSn_IMR()
 */
void setSn_IMR(uint8_t sn, uint8_t imr)
{
	  //w_printf("%s : WZTOE_Sn_IMR(sn)[0x%x] = imr[0x%x]\n", __func__, WZTOE_Sn_IMR(sn), (imr & 0x1F));
    WIZCHIP_WRITE(WZTOE_Sn_IMR(sn), (imr & 0x1F));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_IMR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_IMR.
 * @sa setSn_IMR()
 */
uint8_t getSn_IMR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return (WIZCHIP_READ(WZTOE_Sn_IMR(sn)) & 0x1F);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_ICR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)icr Value to set @ref Sn_ICR
 * @sa getSn_ICR()
 */
void setSn_ICR(uint8_t sn, uint8_t icr)
{
	  //w_printf("%s : WZTOE_Sn_ICR(sn)[0x%x] = icr[0x%x]\n", __func__, WZTOE_Sn_ICR(sn), (icr & 0x1F));
    WIZCHIP_WRITE(WZTOE_Sn_ICR(sn), (icr & 0x1F));
}


void setSn_IR(uint8_t sn, uint8_t icr)
{
	  //w_printf("%s : WZTOE_Sn_ICR(sn)[0x%x] = icr[0x%x]\n", __func__, WZTOE_Sn_ICR(sn), (icr & 0x1F));
    setSn_ICR(sn, icr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_ICR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_ICR.
 * @sa setSn_ICR()
 */
uint8_t getSn_ICR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    (WIZCHIP_READ(WZTOE_Sn_ICR(sn)) & 0x1F);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_SR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_SR.
 */
uint8_t getSn_SR(uint8_t sn)
{
//    //w_printf("%s ", __func__);
    return WIZCHIP_READ(WZTOE_Sn_SR(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_PROTO register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)tos Value to set @ref Sn_PROTO
 * @sa getSn_PROTO()
 */
void setSn_PROTO(uint8_t sn, uint8_t proto)
{
	  //w_printf("%s : WZTOE_Sn_PROTO(sn)[0x%x] = proto[0x%x]\n", __func__, WZTOE_Sn_PROTO(sn), proto);
    WIZCHIP_WRITE(WZTOE_Sn_PROTO(sn), proto);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_PROTO register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of Sn_PROTO.
 * @sa setSn_PROTO()
 */
uint8_t getSn_PROTO(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_PROTO(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TOS register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)tos Value to set @ref Sn_TOS
 * @sa getSn_TOS()
 */
void setSn_TOS(uint8_t sn, uint8_t tos)
{
	  //w_printf("%s : WZTOE_Sn_TOS(sn)[0x%x] = tos[0x%x]\n", __func__, WZTOE_Sn_TOS(sn), tos);
    WIZCHIP_WRITE(WZTOE_Sn_TOS(sn), tos);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TOS register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of Sn_TOS.
 * @sa setSn_TOS()
 */
uint8_t getSn_TOS(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_TOS(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TTL register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)ttl Value to set @ref Sn_TTL
 * @sa getSn_TTL()
 */
void setSn_TTL(uint8_t sn, uint8_t ttl)
{
	  //w_printf("%s : WZTOE_Sn_TTL(sn)[0x%x] = ttl[0x%x]\n", __func__, WZTOE_Sn_TTL(sn), ttl);
    WIZCHIP_WRITE(WZTOE_Sn_TTL(sn), ttl);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TTL register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_TTL.
 * @sa setSn_TTL()
 */
uint8_t getSn_TTL(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_TTL(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_FRAG register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)frag Value to set @ref Sn_FRAG
 * @sa getSn_FRAD()
 */
void setSn_FRAG(uint8_t sn, uint16_t frag)
{
    //w_printf("%s\n", __func__);
    (*(volatile uint32_t *)(WZTOE_Sn_FRAG(sn)) = frag);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_FRAG register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_FRAG.
 * @sa setSn_FRAG()
 */
uint16_t getSn_FRAG(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_FRAG(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_MSSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)mss Value to set @ref Sn_MSSR
 * @sa setSn_MSSR()
 */
void setSn_MSSR(uint8_t sn, uint16_t mss)
{
	  //w_printf("%s : WZTOE_Sn_MSSR(sn)[0x%x] = mss[0x%x]\n", __func__, WZTOE_Sn_MSSR(sn), mss);
    (*(volatile uint32_t *)(WZTOE_Sn_MSSR(sn)) = mss);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_MSSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_MSSR.
 * @sa setSn_MSSR()
 */
uint16_t getSn_MSSR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_MSSR(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_PORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)port Value to set @ref Sn_PORT.
 * @sa getSn_PORT()
 */
void setSn_PORT(uint8_t sn, uint16_t port)
{
	  //w_printf("%s : WZTOE_Sn_PORT(sn)[0x%x] = port[0x%x]\n", __func__, WZTOE_Sn_PORT(sn), port);
    (*(volatile uint32_t *)(WZTOE_Sn_PORT(sn)) = port);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_PORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_PORT.
 * @sa setSn_PORT()
 */
uint16_t getSn_PORT(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_PORT(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_DHAR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dhar Pointer variable to set socket n destination hardware address. It should be allocated 6 bytes.
 * @sa getSn_DHAR()
 */
void setSn_DHAR(uint8_t sn, uint8_t* dhar)
{
    //w_printf("%s\n", __func__);
    WIZCHIP_WRITE((WZTOE_Sn_DHAR(sn)+3), dhar[0]);
    WIZCHIP_WRITE((WZTOE_Sn_DHAR(sn)+2), dhar[1]);
    WIZCHIP_WRITE((WZTOE_Sn_DHAR(sn)+1), dhar[2]);
    WIZCHIP_WRITE((WZTOE_Sn_DHAR(sn)+0), dhar[3]);
    WIZCHIP_WRITE((WZTOE_Sn_DHAR(sn)+7), dhar[4]);
    WIZCHIP_WRITE((WZTOE_Sn_DHAR(sn)+6), dhar[5]); 
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dhar Pointer variable to get socket n destination hardware address. It should be allocated 6 bytes.
 * @sa setSn_DHAR()
 */
 //15.05.19 by justinKim
void getSn_DHAR(uint8_t sn, uint8_t* dhar)
{
    //w_printf("%s\n", __func__);
    dhar[0] = WIZCHIP_READ((WZTOE_Sn_DHAR(sn)+3));
    dhar[1] = WIZCHIP_READ((WZTOE_Sn_DHAR(sn)+2));
    dhar[2] = WIZCHIP_READ((WZTOE_Sn_DHAR(sn)+1));
    dhar[3] = WIZCHIP_READ((WZTOE_Sn_DHAR(sn)+0));
    dhar[4] = WIZCHIP_READ((WZTOE_Sn_DHAR(sn)+7));
    dhar[5] = WIZCHIP_READ((WZTOE_Sn_DHAR(sn)+6)); 
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_DPORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)dport Value to set @ref Sn_DPORT
 * @sa getSn_DPORT()
 */
void setSn_DPORT(uint8_t sn, uint16_t dport)
{
	  //w_printf("%s : WZTOE_Sn_DPORT(sn)[0x%x] = dport[0x%x]\n", __func__, WZTOE_Sn_DPORT(sn), dport);
    (*(volatile uint32_t *)(WZTOE_Sn_DPORT(sn)) = dport);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_DPORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_DPORT.
 * @sa setSn_DPORT()
 */
uint16_t getSn_DPORT(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_DPORT(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_DIPR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dipr Pointer variable to set socket n destination IP address. It should be allocated 4 bytes.
 * @sa getSn_DIPR()
 */
//15.05.19 by justinKim
void setSn_DIPR(uint8_t sn, uint8_t* dipr)
{
    //w_printf("%s\n", __func__);
    WIZCHIP_WRITE((WZTOE_Sn_DIPR3(sn)), dipr[0]);
    WIZCHIP_WRITE((WZTOE_Sn_DIPR2(sn)), dipr[1]);
    WIZCHIP_WRITE((WZTOE_Sn_DIPR1(sn)), dipr[2]);
    WIZCHIP_WRITE((WZTOE_Sn_DIPR(sn)),  dipr[3]);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_DIPR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dipr Pointer variable to get socket n destination IP address. It should be allocated 4 bytes.
 * @sa SetSn_DIPR()
 */
//uint8_t getSn_DIPR(uint8_t sn, dipr) (*(volatile uint32_t *)(dipr) = htole32(*(volatile uint32_t *)(Sn_DIPR(sn))))
//15.05.19 by justinKim
void getSn_DIPR(uint8_t sn, uint8_t* dipr)
{
    //w_printf("%s\n", __func__);
    (dipr)[0] = WIZCHIP_READ(WZTOE_Sn_DIPR3(sn));
    (dipr)[1] = WIZCHIP_READ(WZTOE_Sn_DIPR2(sn));
    (dipr)[2] = WIZCHIP_READ(WZTOE_Sn_DIPR1(sn));
    (dipr)[3] = WIZCHIP_READ(WZTOE_Sn_DIPR(sn)); 
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_KPALVTR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)kpalvt Value to set @ref Sn_KPALVTR
 * @sa getSn_KPALVTR()
 */
void setSn_KPALVTR(uint8_t sn, uint8_t kpalvt)
{
	  //w_printf("%s : WZTOE_Sn_KPALVTR(sn)[0x%x] = kpalvt[0x%x]\n", __func__, WZTOE_Sn_KPALVTR(sn), kpalvt);
    WIZCHIP_WRITE(WZTOE_Sn_KPALVTR(sn), kpalvt);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_KPALVTR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_KPALVTR.
 * @sa setSn_KPALVTR()
 */
uint8_t getSn_KPALVTR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_KPALVTR(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)txbufsize Value to set @ref Sn_TXBUF_SIZE
 * @sa getSn_TXBUF_SIZE()
 */
void setSn_TXBUF_SIZE(uint8_t sn, uint8_t txbufsize)
{
	  //w_printf("%s : WZTOE_Sn_TXBUF_SIZE(sn)[0x%x] = txbufsize[0x%x]\n", __func__, WZTOE_Sn_TXBUF_SIZE(sn), txbufsize);
    WIZCHIP_WRITE(WZTOE_Sn_TXBUF_SIZE(sn), txbufsize);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_TXBUF_SIZE.
 * @sa setSn_TXBUF_SIZE()
 */
uint8_t getSn_TXBUF_SIZE(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_TXBUF_SIZE(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TX_FSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_FSR.
 */
uint16_t getSn_TX_FSR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_TX_FSR(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_RD.
 */
uint16_t getSn_TX_RD(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_TX_RD(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)txwr Value to set @ref Sn_TX_WR
 * @sa GetSn_TX_WR()
 */
void setSn_TX_WR(uint8_t sn, uint16_t txwr)
{
	  //w_printf("%s : WZTOE_Sn_TX_WR(sn)[0x%x] = txwr[0x%x]\n", __func__, WZTOE_Sn_TX_WR(sn), txwr);
    (*(volatile uint32_t *)(WZTOE_Sn_TX_WR(sn)) = txwr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_WR.
 * @sa setSn_TX_WR()
 */
uint16_t getSn_TX_WR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
     return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_TX_WR(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_RXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)rxbufsize Value to set @ref Sn_RXBUF_SIZE
 * @sa getSn_RXBUF_SIZE()
 */
void setSn_RXBUF_SIZE(uint8_t sn, uint8_t rxbufsize)
{
	  //w_printf("%s : WZTOE_Sn_RXBUF_SIZE(sn)[0x%x] = rxbufsize[0x%x]\n", __func__, WZTOE_Sn_RXBUF_SIZE(sn), rxbufsize);
    WIZCHIP_WRITE(WZTOE_Sn_RXBUF_SIZE(sn), rxbufsize);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_RXBUF_SIZE.
 * @sa setSn_RXBUF_SIZE()
 */
uint8_t getSn_RXBUF_SIZE(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return WIZCHIP_READ(WZTOE_Sn_RXBUF_SIZE(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RX_RSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_RX_RSR.
 */
uint16_t getSn_RX_RSR(uint8_t sn)
{
//    //w_printf("%s\n", __func__);
    return  ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_RX_RSR(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @regurn uint16_t. Value of @ref Sn_RX_RD.
 * @sa setSn_RX_RD()
 */
void setSn_RX_RD(uint8_t sn, uint16_t rxrd)
{
	  //w_printf("%s : WZTOE_Sn_RX_RD(sn)[0x%x] = rxrd[0x%x]\n", __func__, WZTOE_Sn_RX_RD(sn), rxrd);
    (*(volatile uint32_t *)(WZTOE_Sn_RX_RD(sn)) = rxrd);
}
/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_RX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)rxrd Value to set @ref Sn_RX_RD
 * @sa getSn_RX_RD()
 */
uint16_t getSn_RX_RD(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return  ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_RX_RD(sn))));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_RX_WR.
 */
uint16_t getSn_RX_WR(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return ((uint16_t)(*(volatile uint32_t *)(WZTOE_Sn_RX_WR(sn))));
}
//////////////////////////////////////

/////////////////////////////////////
// Sn_TXBUF & Sn_RXBUF IO function //
/////////////////////////////////////
/**  
 * @brief Gets the max buffer size of socket sn passed as parameter.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of Socket n RX max buffer size.
 */
uint16_t getSn_RxMAX(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return (getSn_RXBUF_SIZE(sn) << 10);
}

/**  
 * @brief Gets the max buffer size of socket sn passed as parameters.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of Socket n TX max buffer size.
 */
//uint16_t getSn_TxMAX(uint8_t sn);
uint16_t getSn_TxMAX(uint8_t sn)
{
    //w_printf("%s\n", __func__);
    return (getSn_TXBUF_SIZE(sn) << 10);
}

uint8_t WIZCHIP_READ(uint32_t Addr)
{
    uint8_t ret;
//    //w_printf("%s\n", __func__);
    WIZCHIP_CRITICAL_ENTER();
    ret = (uint8_t)(*(volatile uint8_t *)(Addr));
    WIZCHIP_CRITICAL_EXIT();
    return ret;
}

void WIZCHIP_WRITE(uint32_t Addr, uint8_t Data)
{
//    //w_printf("%s\n", __func__);
    WIZCHIP_CRITICAL_ENTER();
    *(volatile uint8_t *)(Addr) = Data;
    WIZCHIP_CRITICAL_EXIT();
}

void WIZCHIP_READ_BUF (uint32_t BaseAddr, uint32_t ptr, uint8_t* pBuf, uint16_t len)
{
    uint16_t i = 0;
	  //w_printf("%s : BaseAddr[0x%x], pBuf[0x%x], len[0x%x]\n", __func__, BaseAddr, pBuf, len);
    WIZCHIP_CRITICAL_ENTER();

    for(i = 0; i < len; i++)
        pBuf[i] = *(volatile uint8_t *)(BaseAddr +((ptr+i)&0xFFFF));

    WIZCHIP_CRITICAL_EXIT();
}

void WIZCHIP_WRITE_BUF(uint32_t BaseAddr, uint32_t ptr, const uint8_t* pBuf, uint16_t len)
{
    uint16_t i = 0;
	  //w_printf("%s : BaseAddr[0x%x], ptr[0x%x], pBuf[0x%x], len[0x%x]\n", __func__, BaseAddr, ptr, pBuf, len);
    WIZCHIP_CRITICAL_ENTER();

    for( i=0; i<len; i++)
        *(volatile uint8_t *)(BaseAddr + ((ptr+i)&0xFFFF)) = pBuf[i];

    WIZCHIP_CRITICAL_EXIT();
}

//void wiz_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len)
int wiz_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len)
{
    uint32_t ptr = 0;
    uint32_t sn_tx_base = 0;

	  //w_printf("%s : sn[0x%x], wizdata[0x%x], len[0x%x]\n", __func__, sn, wizdata, len);
//    if(len == 0)  return;
    if(len == 0)  return 0;
    ptr = getSn_TX_WR(sn);
    sn_tx_base = (TXMEM_BASE) | ((sn&0x7)<<18);
    WIZCHIP_WRITE_BUF(sn_tx_base, ptr, wizdata, len);
    ptr += len;
    setSn_TX_WR(sn,ptr);
}

//void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
int wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
    uint32_t ptr = 0;
    uint32_t sn_rx_base = 0; 

	  //w_printf("%s : sn[0x%x], wizdata[0x%x], len[0x%x]\n", __func__, sn, wizdata, len);
//    if(len == 0) return;
    if(len == 0) return 0;
    ptr = getSn_RX_RD(sn);
    sn_rx_base = (RXMEM_BASE) | ((sn&0x7)<<18);
    WIZCHIP_READ_BUF(sn_rx_base, ptr, wizdata, len);
    ptr += len;
    setSn_RX_RD(sn,ptr);
}

void wiz_recv_ignore(uint8_t sn, uint16_t len)
{
    uint32_t ptr = 0;
    uint32_t sn_rx_base = 0; 

    //w_printf("%s\n", __func__);
    if(len == 0) return;
    ptr = getSn_RX_RD(sn);
    sn_rx_base = (RXMEM_BASE) | ((sn&0x7)<<18);
    ptr = sn_rx_base + ((ptr+len)&0xFFFF);
    setSn_RX_RD(sn,ptr);
}