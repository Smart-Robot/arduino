//#include "w7500.h"
#include <W7500x_wztoe.h>
//#include <Arduino.h>
#include "socket_lib.h"

static uint16_t local_port;

/**
 * @brief	This Socket function initialize the channel in perticular mode, and set the port and wait for W7500 done it.
 * @return 	1 for success else 0.
 */
uint8_t socket_lib(SOCKET s, uint8_t protocol, uint16_t port, uint8_t flag)
{
    return socket(s, protocol, port, flag);
#if 0
    if ((protocol == SnMR::TCP) || (protocol == SnMR::UDP) || (protocol == SnMR::IPRAW) || (protocol == SnMR::MACRAW) || (protocol == SnMR::PPPOE))
    {
        close_lib(s);
        //W7500.writeSnMR(s, protocol | flag);
        setSn_MR(s, protocol | flag);
        if (port != 0) {
            //W7500.writeSnPORT(s, port);
            setSn_PORT(s, port);
        } 
        else {
            local_port++; // if don't set the source port, set local_port number.
            //W7500.writeSnPORT(s, local_port);
            setSn_PORT(s, local_port);
        }
        W7500.execCmdSn(s, Sock_OPEN);
        return 1;
    }
    return 0;
#endif
}


uint8_t socketStatus_lib(SOCKET s)
{
    //  uint8_t status = W7500.readSnSR(s);
    uint8_t status = getSn_SR(s);
    return status;
}


/**
 * @brief	This function close the socket and parameter is "s" which represent the socket number
 */
void close_lib(SOCKET s)
{
    close(s);
#if 0
    W7500.execCmdSn(s, Sock_CLOSE);
    //W7500.writeSnIR(s, 0xFF);
    setSn_IR(s, 0xFF);
#endif
}


/**
 * @brief	This function established  the connection for the channel in passive (server) mode. This function waits for the request from the peer.
 * @return	1 for success else 0.
 */
uint8_t listen_lib(SOCKET s)
{
    return listen(s);
#if 0
    //if (W7500.readSnSR(s) != SnSR::INIT) {
    if (getSn_SR(s) != SnSR::INIT) {
        return 0;
    }
    W7500.execCmdSn(s, Sock_LISTEN);
    return 1;
#endif
}


/**
 * @brief	This function established  the connection for the channel in Active (client) mode. 
 * 		This function waits for the untill the connection is established.
 * 		
 * @return	1 for success else 0.
 */
uint8_t connect_lib(SOCKET s, uint8_t * addr, uint16_t port)
{
    return connect(s, addr, port);
#if 0
    if(((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
       ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
       (port == 0x00) ) 
        return 0;

    //set destination IP
    //W7500.writeSnDIPR(s, addr);
    //W7500.writeSnDPORT(s, port);
    setSn_DIPR(s, addr);
    setSn_DPORT(s, port);
    W7500.execCmdSn(s, Sock_CONNECT);

    return 1;
#endif
}



/**
 * @brief	This function used for disconnect the socket and parameter is "s" which represent the socket number
 * @return	1 for success else 0.
 */
void disconnect_lib(SOCKET s)
{
    disconnect(s);
#if 0
    W7500.execCmdSn(s, Sock_DISCON);
#endif
}


/**
 * @brief	This function used to send the data in TCP mode
 * @return	1 for success else 0.
 */
uint32_t send_lib(SOCKET s, const uint8_t * buf, uint16_t len)
{
    return send(s, buf, len);
#if 0
    uint8_t status=0;
    uint16_t ret=0;
    uint16_t freesize=0;

    if (len > W7500.SSIZE) 
        ret = W7500.SSIZE; // check size not to exceed MAX size.
    else 
        ret = len;

    //if freebuf is available, start.
    do 
    {
        freesize = W7500.getTXFreeSize(s);
        //status = W7500.readSnSR(s);
        status = getSn_SR(s);
        if ((status != SnSR::ESTABLISHED) && (status != SnSR::CLOSE_WAIT))
        {
            ret = 0; 
            break;
        }
        yield();
    } 
    while (freesize < ret);

    // copy data
    W7500.send_data_processing(s, (uint8_t *)buf, ret);
    W7500.execCmdSn(s, Sock_SEND);

    /* +2008.01 bj */
    //while ( (W7500.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
    while ( (getSn_IR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
    {
        /* m2008.01 [bj] : reduce code */
        //if ( W7500.readSnSR(s) == SnSR::CLOSED )
        if ( getSn_SR(s) == SnSR::CLOSED )
        {
            close_lib(s);
            return 0;
        }
        yield();
    }
    /* +2008.01 bj */
    //W7500.writeSnIR(s, SnIR::SEND_OK);
    setSn_IR(s, SnIR::SEND_OK);
    return ret;
#endif
}


/**
 * @brief	This function is an application I/F function which is used to receive the data in TCP mode.
 * 		It continues to wait for data as much as the application wants to receive.
 * 		
 * @return	received data size for success else -1.
 */
int32_t recv_lib(SOCKET s, uint8_t *buf, int16_t len)
{
    return (recv(s, buf, len));
#if 0
    // Check how much data is available
    int16_t ret = W7500.getRXReceivedSize(s);
    if ( ret == 0 )
    {
        // No data available.
        //uint8_t status = W7500.readSnSR(s);
        uint8_t status = getSn_SR(s);
        if ( status == SnSR::LISTEN || status == SnSR::CLOSED || status == SnSR::CLOSE_WAIT )
        {
            // The remote end has closed its side of the connection, so this is the eof state
            ret = 0;
        }
        else
        {
            // The connection is still up, but there's no data waiting to be read
            ret = -1;
        }
    }
    else if (ret > len)
    {
        ret = len;
    }

    if ( ret > 0 )
    {
        W7500.recv_data_processing(s, buf, ret);
        W7500.execCmdSn(s, Sock_RECV);
    }
    return ret;
#endif
}


int16_t recvAvailable_lib(SOCKET s)
{
    int16_t ret = W7500.getRXReceivedSize(s);
    return ret;
}


/**
 * @brief	Returns the first byte in the receive queue (no checking)
 * 		
 * @return
 */
uint16_t peek_lib(SOCKET s, uint8_t *buf)
{
  W7500.recv_data_processing(s, buf, 1, 1);
  return 1;
}


/**
 * @brief	This function is an application I/F function which is used to send the data for other then TCP mode. 
 * 		Unlike TCP transmission, The peer's destination address and the port is needed.
 * 		
 * @return	This function return send data size for success else -1.
 */
//uint16_t sendto_lib(SOCKET s, const uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t port)
int32_t sendto_lib(SOCKET s, const uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t port)
{
    return sendto(s, buf, len, addr, port);
#if 0
    uint16_t ret=0;

    if (len > W7500.SSIZE) ret = W7500.SSIZE; // check size not to exceed MAX size.
    else ret = len;

    if
    (
        ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
        ((port == 0x00)) ||(ret == 0)
    ) 
    {
        /* +2008.01 [bj] : added return value */
        ret = 0;
    }
    else
    {
        //W7500.writeSnDIPR(s, addr);
        //W7500.writeSnDPORT(s, port);
        setSn_DIPR(s, addr);
        setSn_DPORT(s, port);
        // copy data
        W7500.send_data_processing(s, (uint8_t *)buf, ret);
        W7500.execCmdSn(s, Sock_SEND);
        /* +2008.01 bj */
        //while ( (W7500.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
        while ( (getSn_IR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
        {
            //if (W7500.readSnIR(s) & SnIR::TIMEOUT)
            if (getSn_IR(s) & SnIR::TIMEOUT)
            {
                /* +2008.01 [bj]: clear interrupt */
                //W7500.writeSnIR(s, (SnIR::SEND_OK | SnIR::TIMEOUT)); /* clear SEND_OK & TIMEOUT */
                setSn_IR(s, (SnIR::SEND_OK | SnIR::TIMEOUT)); /* clear SEND_OK & TIMEOUT */
                return 0;
            }
            yield();
        }
        /* +2008.01 bj */
        //W7500.writeSnIR(s, SnIR::SEND_OK);
        setSn_IR(s, SnIR::SEND_OK);
    }
    return ret;
#endif
}


/**
 * @brief	This function is an application I/F function which is used to receive the data in other then
 * 	TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well. 
 * 	
 * @return	This function return received data size for success else -1.
 */
//uint16_t recvfrom_lib(SOCKET s, uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t *port)
int32_t recvfrom_lib(SOCKET s, uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t *port)
{
    return recvfrom(s, buf, len, addr, port);
#if 0
    uint8_t head[8];
    uint16_t data_len=0;
    uint16_t ptr=0;

    if ( len > 0 )
    {
        ptr = W7500.readSnRX_RD(s);
        switch (W7500.readSnMR(s) & 0x07)
        {
            case SnMR::UDP :
                W7500.read_data(s, ptr, head, 0x08);
                ptr += 8;
                // read peer's IP address, port number.
                addr[0] = head[0];
                addr[1] = head[1];
                addr[2] = head[2];
                addr[3] = head[3];
                *port = head[4];
                *port = (*port << 8) + head[5];
                data_len = head[6];
                data_len = (data_len << 8) + head[7];

                W7500.read_data(s, ptr, buf, data_len); // data copy.
                ptr += data_len;

                //W7500.writeSnRX_RD(s, ptr);
                setSn_RX_RD(s, ptr);
                break;

            case SnMR::IPRAW :
                W7500.read_data(s, ptr, head, 0x06);
                ptr += 6;

                addr[0] = head[0];
                addr[1] = head[1];
                addr[2] = head[2];
                addr[3] = head[3];
                data_len = head[4];
                data_len = (data_len << 8) + head[5];

                W7500.read_data(s, ptr, buf, data_len); // data copy.
                ptr += data_len;

                //W7500.writeSnRX_RD(s, ptr);
                setSn_RX_RD(s, ptr);
                break;

            case SnMR::MACRAW:
                W7500.read_data(s, ptr, head, 2);
                ptr+=2;
                data_len = head[0];
                data_len = (data_len<<8) + head[1] - 2;

                W7500.read_data(s, ptr, buf, data_len);
                ptr += data_len;
                //W7500.writeSnRX_RD(s, ptr);
                setSn_RX_RD(s, ptr);
            break;

            default :
                break;
        }
        W7500.execCmdSn(s, Sock_RECV);
    }
    return data_len;
#endif
}

/**
 * @brief	Wait for buffered transmission to complete.
 */
void flush_lib(SOCKET s) 
{
    // TODO
}

uint32_t igmpsend_lib(SOCKET s, const uint8_t * buf, uint16_t len)
{
    uint16_t ret=0;

    if (len > W7500.SSIZE) 
        ret = W7500.SSIZE; // check size not to exceed MAX size.
    else 
        ret = len;

    if (ret == 0)
        return 0;

    W7500.send_data_processing(s, (uint8_t *)buf, ret);
    W7500.execCmdSn(s, Sock_SEND);

    //while ( (W7500.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
    while ( (getSn_IR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
    {
        //if (W7500.readSnIR(s) & SnIR::TIMEOUT)
        if (getSn_IR(s) & SnIR::TIMEOUT)
        {
            /* in case of igmp, if send fails, then socket closed */
            /* if you want change, remove this code. */
            close_lib(s);
            return 0;
        }
        yield();
    }

    //W7500.writeSnIR(s, SnIR::SEND_OK);
    setSn_IR(s, SnIR::SEND_OK);
    return ret;
}

uint16_t bufferData_lib(SOCKET s, uint16_t offset, const uint8_t* buf, uint16_t len)
{
    uint16_t ret =0;
    if (len > W7500.getTXFreeSize(s))
    {
        ret = W7500.getTXFreeSize(s); // check size not to exceed MAX size.
    }
    else
    {
        ret = len;
    }
    W7500.send_data_processing_offset(s, offset, buf, ret);
    return ret;
}

int startUDP(SOCKET s, uint8_t* addr, uint16_t port)
{
    if
    (
        ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
        ((port == 0x00))
    ) 
    {
    return 0;
    }
    else
    {
        //W7500.writeSnDIPR(s, addr);
        //W7500.writeSnDPORT(s, port);
        setSn_DIPR(s, addr);
        setSn_DPORT(s, port);
        return 1;
    }
}

int sendUDP(SOCKET s)
{
    W7500.execCmdSn(s, Sock_SEND);
		
    /* +2008.01 bj */
    //  while ( (W7500.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
    while ( (getSn_IR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) 
    {
        //if (W7500.readSnIR(s) & SnIR::TIMEOUT)
        if (getSn_IR(s) & SnIR::TIMEOUT)
        {
            /* +2008.01 [bj]: clear interrupt */
            //W7500.writeSnIR(s, (SnIR::SEND_OK|SnIR::TIMEOUT));
            setSn_IR(s, (SnIR::SEND_OK|SnIR::TIMEOUT));
            return 0;
        }
        yield();
    }

    /* +2008.01 bj */	
    //W7500.writeSnIR(s, SnIR::SEND_OK);
    setSn_IR(s, SnIR::SEND_OK);

    /* Sent ok */
    return 1;
}

