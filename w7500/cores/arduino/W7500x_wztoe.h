/*
 *********************************************************************
 * @file    : wztoe.h
 * @version : 1.0.2
 * @author  : WIZnet
 * @data    20-May-2015
 * @brief   : WZTOE dirver for W7500
 *********************************************************************
 * @attention
 * @par Revision history
 *    <2015/05/20> V1.0.2 by justinKim
 *      1. Register & Macro name change WZTOE_xxx
 *    <2015/05/19> V1.0.1 by justinKim
 *      1. getSn_DIPR - IP Address Bug fix
 *      2. getSn_DHAR - SHAR -> DHAR  Bug fix
 *    <2015/05/01> 1st Release
 */


#ifndef __WZTOE_H
#define __WZTOE_H
//#include "W7500x.h"
//#include "Arduino.h"
//#include <stdio.h>
#include <stdint.h>

#define _WIZCHIP_                      7500 // 5500, 5100, 5200, 5500

/* Peripheral base address */
#define W7500x_WZTOE_BASE   (0x46000000)//(W7500x_AHB_BASE + 0x06000000UL)

/* WZTOE register and MEMs memory map */
//-----------------------------------------
// TX and RX Mermory address  
//-----------------------------------------
#define TXMEM_BASE          (W7500x_WZTOE_BASE + 0x00020000)
#define RXMEM_BASE          (W7500x_WZTOE_BASE + 0x00030000)
//-----------------------------------------
// Common Register address
//-----------------------------------------
#define WZTOE_VENDOR_INFO         (W7500x_WZTOE_BASE + 0x00000000) 
#define WZTOE_SYS_BASE            (W7500x_WZTOE_BASE + 0x00002000) 
#define WZTOE_PHY_BASE            (W7500x_WZTOE_BASE + 0x00004000) 
#define WZTOE_NETIPV4_BASE        (W7500x_WZTOE_BASE + 0x00006000) 

#define WZTOE_VERSIONR            (WZTOE_VENDOR_INFO)
#define WZTOE_TIC100US            (WZTOE_SYS_BASE)

#define WZTOE_IR                  (W7500x_WZTOE_BASE + 0x00002100) //Interrupt
#define WZTOE_IMR                 (W7500x_WZTOE_BASE + 0x00002104)
#define WZTOE_ICR                 (W7500x_WZTOE_BASE + 0x00002108)
#define WZTOE_SIR                 (W7500x_WZTOE_BASE + 0x00002110)
#define WZTOE_SIMR                (W7500x_WZTOE_BASE + 0x00002114)
#define WZTOE_INTLEVEL            (W7500x_WZTOE_BASE + 0x00002200)

#define WZTOE_MR                  (W7500x_WZTOE_BASE + 0x00002300) //Mode Register
#define WZTOE_MR1                 (W7500x_WZTOE_BASE + 0x00002301) //Mode Register

#define WZTOE_PTIMER              (W7500x_WZTOE_BASE + 0x00002400) //PPPoE Timer Register
#define WZTOE_PMAGIC              (W7500x_WZTOE_BASE + 0x00002404) //PPPoE LCP Magic number in PPPoE
#define WZTOE_PHAR                (W7500x_WZTOE_BASE + 0x00002408)
#define WZTOE_PSID                (W7500x_WZTOE_BASE + 0x00002410)
#define WZTOE_PMRU                (W7500x_WZTOE_BASE + 0x00002414)

#define WZTOE_SHAR                (W7500x_WZTOE_BASE + 0x00006000) //Network IPv4
#define WZTOE_GAR                 (W7500x_WZTOE_BASE + 0x00006008)
#define WZTOE_SUBR                (W7500x_WZTOE_BASE + 0x0000600C)
#define WZTOE_SIPR                (W7500x_WZTOE_BASE + 0x00006010)
#define WZTOE_NETCFGLOCK          (W7500x_WZTOE_BASE + 0x00006020) 
#define WZTOE_RTR                 (W7500x_WZTOE_BASE + 0x00006040) //Conf IPvr
#define WZTOE_RCR                 (W7500x_WZTOE_BASE + 0x00006044)
#define WZTOE_UIPR                (W7500x_WZTOE_BASE + 0x00006050) //Port unreachable
#define WZTOE_UPORTR              (W7500x_WZTOE_BASE + 0x00006054) 

//-----------------------------------------
// Socket Register address
//-----------------------------------------
#define WZTOE_Sn_MR(ch)          (W7500x_WZTOE_BASE + (0x00010000 + ((ch)<<18))) 
#define WZTOE_Sn_CR(ch)           (W7500x_WZTOE_BASE + (0x00010010 + ((ch)<<18))) 
#define WZTOE_Sn_ISR(ch)          (W7500x_WZTOE_BASE + (0x00010020 + ((ch)<<18))) 
#define WZTOE_Sn_IMR(ch)          (W7500x_WZTOE_BASE + (0x00010024 + ((ch)<<18))) 
#define WZTOE_Sn_ICR(ch)          (W7500x_WZTOE_BASE + (0x00010028 + ((ch)<<18))) 
#define WZTOE_Sn_SR(ch)           (W7500x_WZTOE_BASE + (0x00010030 + ((ch)<<18))) 

#define WZTOE_Sn_PROTO(ch)        (W7500x_WZTOE_BASE + (0x00010100 + ((ch)<<18))) 
#define WZTOE_Sn_TOS(ch)          (W7500x_WZTOE_BASE + (0x00010104 + ((ch)<<18))) 
#define WZTOE_Sn_TTL(ch)          (W7500x_WZTOE_BASE + (0x00010108 + ((ch)<<18)))
#define WZTOE_Sn_FRAG(ch)          (W7500x_WZTOE_BASE + (0x0001010C + ((ch)<<18)))
#define WZTOE_Sn_MSSR(ch)         (W7500x_WZTOE_BASE + (0x00010110 + ((ch)<<18))) 
#define WZTOE_Sn_PORT(ch)         (W7500x_WZTOE_BASE + (0x00010114 + ((ch)<<18))) 
#define WZTOE_Sn_DHAR(ch)         (W7500x_WZTOE_BASE + (0x00010118 + ((ch)<<18))) 
#define WZTOE_Sn_DPORT(ch)        (W7500x_WZTOE_BASE + (0x00010120 + ((ch)<<18))) 
#define WZTOE_Sn_DIPR(ch)         (W7500x_WZTOE_BASE + (0x00010124 + ((ch)<<18)))
#define WZTOE_Sn_DIPR1(ch)         (W7500x_WZTOE_BASE + (0x00010125 + ((ch)<<18)))
#define WZTOE_Sn_DIPR2(ch)         (W7500x_WZTOE_BASE + (0x00010126 + ((ch)<<18)))
#define WZTOE_Sn_DIPR3(ch)         (W7500x_WZTOE_BASE + (0x00010127 + ((ch)<<18)))

#define WZTOE_Sn_KPALVTR(ch)      (W7500x_WZTOE_BASE + (0x00010180 + ((ch)<<18))) 
#define WZTOE_Sn_TXBUF_SIZE(ch)   (W7500x_WZTOE_BASE + (0x00010200 + ((ch)<<18))) 
#define WZTOE_Sn_TX_FSR(ch)       (W7500x_WZTOE_BASE + (0x00010204 + ((ch)<<18))) 
#define WZTOE_Sn_TX_RD(ch)        (W7500x_WZTOE_BASE + (0x00010208 + ((ch)<<18))) 
#define WZTOE_Sn_TX_WR(ch)        (W7500x_WZTOE_BASE + (0x0001020C + ((ch)<<18))) 
#define WZTOE_Sn_RXBUF_SIZE(ch)   (W7500x_WZTOE_BASE + (0x00010220 + ((ch)<<18))) 
#define WZTOE_Sn_RX_RSR(ch)       (W7500x_WZTOE_BASE + (0x00010224 + ((ch)<<18))) 
#define WZTOE_Sn_RX_RD(ch)        (W7500x_WZTOE_BASE + (0x00010228 + ((ch)<<18))) 
#define WZTOE_Sn_RX_WR(ch)        (W7500x_WZTOE_BASE + (0x0001022C + ((ch)<<18))) 
//#define WZTOE_Sn_TSR(ch)        (W7500x_WZTOE_BASE + (0x00010400 + ((ch)<<18))) 
//-----------------------------------------

/* Variable Values */
//-----------------------------------------
// Register Values 
//-----------------------------------------
// MODE register values
#define MR_NOTCPRSTPKT      (0x0200) /**< No Send TCP RESET PACKET */
#define MR_SKIPSRCMAC       (0x0100) /**< SKIP Source MAC Check for test */
#define MR_RST              (0x0080) /**< reset */
#define MR_WOL              (0x0020) /**< Wake on Lan */
#define MR_PB               (0x0010) /**< ping block */
#define MR_PPPOE            (0x0008) /**< enable pppoe */
#define MR_FARP             (0x0002) /**< enbale FORCE ARP */

// IR register values 
#define IR_CONFLICT         (0x80) /**< check ip confict */
#define IR_UNREACH          (0x40) /**< get the destination unreachable message in UDP sending */
#define IR_PPPoE            (0x20) /**< get the PPPoE close message */
#define IR_MP               (0x10) /**< get the magic packet interrupt */
#define IM_IR7              IR_CONFLICT //0x80
#define IM_IR6              IR_UNREACH
#define IM_IR5              IR_PPPoE
#define IM_IR4              IR_MP

// Sn_MR values 
#define Sn_MR_MULTI         (0x80)     /**< support UDP Multicating */
#define Sn_MR_BCASTB        (0x40)     /**< support UDP Broadcasting */
#define Sn_MR_FPSHCLR       (0x40)     /**< IPv4 clear PSH flag in TCP */
#define Sn_MR_ND            (0x20)     /**< No Delayed Ack(TCP) flag */
#define Sn_MR_MC            (0x20)     /**< Multicast IGMP (UDP) flag */
#define Sn_MR_UCASTB        (0x10)     /**< Unicast Block in UDP Multicating*/
#define Sn_MR_PPPOE         (0x05)     /**< PPPoE */
#define Sn_MR_MACRAW        (0x04)     /**< MAC LAYER RAW SOCK */
#define Sn_MR_IPRAW         (0x03)     /**< IP LAYER RAW SOCK */
#define Sn_MR_UDP           (0x02)     /**< UDP */
#define Sn_MR_TCP           (0x01)     /**< TCP */
#define Sn_MR_CLOSE         (0x00)     /**< unused socket */

// Sn_MR values in MACRAW MODE 
#define Sn_MR_MFEN          (Sn_MR_MULTI)     /**< support MAC filter enable */
#define Sn_MR_MMB           (Sn_MR_ND)     /**< IPv4 Multicasting Block */
#define Sn_MR_MIP6B         (Sn_MR_UCASTB)     /**< IPv6 packet Block */

/**
 * @brief For Berkeley Socket API
 */
#define SOCK_STREAM         Sn_MR_TCP
#define SOCK_DGRAM          Sn_MR_UDP


// Sn_CR values 
#define Sn_CR_OPEN          (0x01)     /**< initialize or open socket */
#define Sn_CR_LISTEN        (0x02)     /**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT       (0x04)     /**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON        (0x08)     /**< send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE         (0x10)     /**< close socket */
#define Sn_CR_SEND          (0x20)     /**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC      (0x21)     /**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP     (0x22)     /**<  send keep alive message */
#define Sn_CR_RECV          (0x40)     /**< update rxbuf pointer, recv data */
#ifdef __DEF_IINCHIP_PPP__
#define Sn_CR_PCON          (0x23)      
#define Sn_CR_PDISCON       (0x24)      
#define Sn_CR_PCR           (0x25)      
#define Sn_CR_PCN           (0x26)     
#define Sn_CR_PCJ           (0x27)     
#endif


// Sn_IR values 
#define Sn_IR_SENDOK        (0x10)     /**< complete sending */
#define Sn_IR_TIMEOUT       (0x08)     /**< assert timeout */
#define Sn_IR_RECV          (0x04)     /**< receiving data */
#define Sn_IR_DISCON        (0x02)     /**< closed socket */
#define Sn_IR_CON           (0x01)     /**< established connection */
#ifdef __DEF_IINCHIP_PPP__
#define Sn_IR_PNEXT         (0x20)     
#define Sn_IR_PFAIL         (0x40)     
#define Sn_IR_PRECV         (0x80)     
#endif

// Sn_SR values 
#define SOCK_CLOSED         (0x00)     /**< closed */
#define SOCK_INIT           (0x13)     /**< init state */
#define SOCK_LISTEN         (0x14)     /**< listen state */
#define SOCK_SYNSENT        (0x15)     /**< connection state */
#define SOCK_SYNRECV        (0x16)     /**< connection state */
#define SOCK_ESTABLISHED    (0x17)     /**< success to connect */
#define SOCK_FIN_WAIT       (0x18)     /**< closing state */
#define SOCK_CLOSING        (0x1A)     /**< closing state */
#define SOCK_TIME_WAIT      (0x1B)     /**< closing state */
#define SOCK_CLOSE_WAIT     (0x1C)     /**< closing state */
#define SOCK_LAST_ACK       (0x1D)     /**< closing state */
#define SOCK_UDP            (0x22)     /**< udp socket */
#define SOCK_IPRAW          (0x32)     /**< ip raw mode socket */
#define SOCK_MACRAW         (0x42)     /**< mac raw mode socket */
#define SOCK_PPPOE          (0x5F)     /**< pppoe socket */

// IP PROTOCOL 
#define IPPROTO_IP          (0  )      /**< Dummy for IP */
#define IPPROTO_ICMP        (1  )      /**< Control message protocol */
#define IPPROTO_IGMP        (2  )      /**< Internet group management protocol */
#define IPPROTO_GGP         (3  )      /**< Gateway^2 (deprecated) */
#define IPPROTO_TCP         (6  )      /**< TCP */
#define IPPROTO_PUP         (12 )      /**< PUP */
#define IPPROTO_UDP         (17 )      /**< UDP */
#define IPPROTO_IDP         (22 )      /**< XNS idp */
#define IPPROTO_ND          (77 )      /**< UNOFFICIAL net disk protocol */
#define IPPROTO_RAW         (255)      /**< Raw IP packet */
/**
 * @brief Enter a critical section
 * @brief Exit a critical section
 */
#define WIZCHIP_CRITICAL_ENTER()     //WIZCHIP.CRIS._enter()
#define WIZCHIP_CRITICAL_EXIT()     //WIZCHIP.CRIS._exit()
////////////////////////
// Basic I/O Function //
////////////////////////
uint8_t WIZCHIP_READ(uint32_t Addr);
void WIZCHIP_WRITE(uint32_t Addr, uint8_t Data);
void WIZCHIP_READ_BUF (uint32_t BaseAddr, uint32_t ptr, uint8_t* pBuf, uint16_t len);
void WIZCHIP_WRITE_BUF(uint32_t BaseAddr, uint32_t ptr, const uint8_t* pBuf, uint16_t len);



/**
 * @ingroup Basic_IO_function
 * @brief It copies data to internal TX memory
 *
 * @details This function reads the Tx write pointer register and after that,
 * it copies the <i>wizdata(pointer buffer)</i> of the length of <i>len(variable)</i> bytes to internal TX memory
 * and updates the Tx write pointer register.
 * This function is being called by send() and sendto() function also.
 *
 * @note User should read upper byte first and lower byte later to get proper value.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param wizdata Pointer buffer to write data
 * @param len Data length
 * @sa wiz_recv_data()
 */
//void wiz_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len);
int wiz_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len);

/**
 * @ingroup Basic_IO_function
 * @brief It copies data to your buffer from internal RX memory
 *
 * @details This function read the Rx read pointer register and after that,
 * it copies the received data from internal RX memory
 * to <i>wizdata(pointer variable)</i> of the length of <i>len(variable)</i> bytes.
 * This function is being called by recv() also.
 *
 * @note User should read upper byte first and lower byte later to get proper value.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param wizdata Pointer buffer to read data
 * @param len Data length
 * @sa wiz_send_data()
 */
//void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len);
int wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len);

/**
 * @ingroup Basic_IO_function
 * @brief It discard the received data in RX memory.
 * @details It discards the data of the length of <i>len(variable)</i> bytes in internal RX memory.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param len Data length
 */
void wiz_recv_ignore(uint8_t sn, uint16_t len);











uint8_t getVERSIONR(void);
uint16_t getTIC100US(void);
void setTIC100US(uint16_t tic);
void setIR(uint8_t ir);
uint8_t getIR(void);
void setIMR(uint8_t imr);
uint8_t getIMR(void);
void setICR(uint8_t icr);
void setIR(uint8_t ir);
uint8_t getICR(void);
void setSIR(uint8_t sir);
uint8_t getSIR(void);
void setSIMR(uint8_t simr);
uint8_t getSIMR(void);
void setINTLEVEL(uint16_t intlevel);
uint16_t getINTLEVEL(void);
uint8_t getMR(void);
void setMR(uint8_t mr);
uint8_t getMR1(void);
void setMR1(uint8_t mr);
uint8_t getMR1(void);
void setPTIMER(uint8_t ptimer);
uint8_t getPTIMER(void);
void setPMAGIC(uint8_t pmagic);
uint8_t getPMAGIC(void);
void setPHAR(uint8_t* phar);
void getPHAR(uint8_t* phar);
void setPSID(uint16_t psid);
uint16_t getPSID(void);
void setPMRU(uint16_t pmru);
uint16_t getPMRU(void);
void setSHAR(uint8_t* shar);
void getSHAR(uint8_t* shar);
void setGAR(uint8_t* gar);
void getGAR(uint8_t* gar);
void setSUBR(uint8_t* subr);
void getSUBR(uint8_t* subr);
void setSIPR(uint8_t* sipr);
uint8_t getSIPR(uint8_t* sipr);
void lockNETCFGLOCK(void);
void unlockNETCFGLOCK32(void);
void setRTR(uint16_t rtr);
uint16_t getRTR(void);
void setRCR(uint16_t rcr);
uint16_t getRCR(void);
void getUIPR(uint8_t* uipr);
uint16_t getUPORTR(void);
void setSn_MR(uint8_t sn, uint8_t mr);
uint8_t getSn_MR(uint8_t sn);
void setSn_CR(uint8_t sn, uint8_t cr);
uint8_t getSn_CR(uint8_t sn);
void setSn_IR(uint8_t sn, uint8_t ir);
uint8_t getSn_IR(uint8_t sn);
void setSn_IMR(uint8_t sn, uint8_t imr);
uint8_t getSn_IMR(uint8_t sn);
void setSn_ICR(uint8_t sn, uint8_t icr);
uint8_t getSn_ICR(uint8_t sn);
uint8_t getSn_SR(uint8_t sn);
void setSn_PROTO(uint8_t sn, uint8_t proto);
uint8_t getSn_PROTO(uint8_t sn);
void setSn_TOS(uint8_t sn, uint8_t tos);
uint8_t getSn_TOS(uint8_t sn);
void setSn_TTL(uint8_t sn, uint8_t ttl);
uint8_t getSn_TTL(uint8_t sn);
void setSn_FRAG(uint8_t sn, uint16_t frag);
uint16_t getSn_FRAG(uint8_t sn);
void setSn_MSSR(uint8_t sn, uint16_t mss);
uint16_t getSn_MSSR(uint8_t sn);
void setSn_PORT(uint8_t sn, uint16_t port);
uint16_t getSn_PORT(uint8_t sn);
void setSn_DHAR(uint8_t sn, uint8_t* dharv);
void getSn_DHAR(uint8_t sn, uint8_t* dhar);
void setSn_DPORT(uint8_t sn, uint16_t dport);
uint16_t getSn_DPORT(uint8_t sn);
void setSn_DIPR(uint8_t sn, uint8_t* dipr);
void getSn_DIPR(uint8_t sn, uint8_t* diprv);
void setSn_KPALVTR(uint8_t sn, uint8_t kpalvt);
uint8_t getSn_KPALVTR(uint8_t sn);
void setSn_TXBUF_SIZE(uint8_t sn, uint8_t txbufsize);
uint8_t getSn_TXBUF_SIZE(uint8_t sn);
uint16_t getSn_TX_FSR(uint8_t sn);

uint16_t getSn_TX_RD(uint8_t sn);
void setSn_TX_WR(uint8_t sn, uint16_t txwr);
uint16_t getSn_TX_WR(uint8_t sn);
void setSn_RXBUF_SIZE(uint8_t sn, uint8_t rxbufsize);
uint8_t getSn_RXBUF_SIZE(uint8_t sn);
uint16_t getSn_RX_RSR(uint8_t sn);
void setSn_RX_RD(uint8_t sn, uint16_t rxrd);
uint16_t getSn_RX_RD(uint8_t sn);
uint16_t getSn_RX_WR(uint8_t sn);
uint16_t getSn_RxMAX(uint8_t sn);
uint16_t getSn_TxMAX(uint8_t sn);



#endif

