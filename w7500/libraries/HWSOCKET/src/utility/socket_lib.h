#ifndef	_SOCKET_LIB_H_
#define	_SOCKET_LIB_H_

#include "utility/w7500.h"

extern uint8_t socket_lib(SOCKET s, uint8_t protocol, uint16_t port, uint8_t flag); // Opens a socket(TCP or UDP or IP_RAW mode)
extern uint8_t socketStatus_lib(SOCKET s);
extern void close_lib(SOCKET s); // Close socket
extern uint8_t connect_lib(SOCKET s, uint8_t * addr, uint16_t port); // Establish TCP connection (Active connection)
extern void disconnect_lib(SOCKET s); // disconnect the connection
extern uint8_t listen_lib(SOCKET s);	// Establish TCP connection (Passive connection)
extern uint32_t send_lib(SOCKET s, const uint8_t * buf, uint16_t len); // Send data (TCP)
extern int32_t recv_lib(SOCKET s, uint8_t * buf, int16_t len);	// Receive data (TCP)
extern int16_t recvAvailable_lib(SOCKET s);
extern uint16_t peek_lib(SOCKET s, uint8_t *buf);
extern int32_t sendto_lib(SOCKET s, const uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port); // Send data (UDP/IP RAW)
//extern uint16_t recvfrom_lib(SOCKET s, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port); // Receive data (UDP/IP RAW)
extern int32_t recvfrom_lib(SOCKET s, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port); // Receive data (UDP/IP RAW)
extern void flush_lib(SOCKET s); // Wait for transmission to complete

extern uint32_t igmpsend_lib(SOCKET s, const uint8_t * buf, uint16_t len);

// Functions to allow buffered UDP send (i.e. where the UDP datagram is built up over a
// number of calls before being sent
/*
  @brief This function sets up a UDP datagram, the data for which will be provided by one
  or more calls to bufferData and then finally sent with sendUDP.
  @return 1 if the datagram was successfully set up, or 0 if there was an error
*/
extern int startUDP(SOCKET s, uint8_t* addr, uint16_t port);
/*
  @brief This function copies up to len bytes of data from buf into a UDP datagram to be
  sent later by sendUDP.  Allows datagrams to be built up from a series of bufferData calls.
  @return Number of bytes successfully buffered
*/
uint16_t bufferData_lib(SOCKET s, uint16_t offset, const uint8_t* buf, uint16_t len);
/*
  @brief Send a UDP datagram built up from a sequence of startUDP followed by one or more
  calls to bufferData.
  @return 1 if the datagram was successfully sent, or 0 if there was an error
*/
int sendUDP(SOCKET s);

#endif
/* _SOCKET_H_ */
