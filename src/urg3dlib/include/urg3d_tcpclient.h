#ifndef URG3D_TCPCLIENT_H
#define URG3D_TCPCLIENT_H

/*!
  \file
  \brief TCP/IP read/write functions

  \author Katsumi Kimoto

  $Id: urg3d_tcpclient.h,v 1d233c7a2240 2011/02/19 03:08:45 Satofumi $
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg3d_ring_buffer.h"
#include "urg3d_detect_os.h"
#include <sys/types.h>
#if defined(URG3D_WINDOWS_OS)
//#ifndef WIN32_LEAN_AND_MEAN
//#define WIN32_LEAN_AND_MEAN 1
//#endif
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
//#include <WinSock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

//2014-10-23 Doog
#define URG3D_MAX_RX_BUFFER_BIT		(16) //This is recommend 65536 byte
//#define URG3D_MAX_RX_BUFFER_BIT		(12) //This is minimize 4096 byte


// -- NOT INTERFACE, for internal use only --
// For urg3d_ringbuffer.h
// The size of buffer must be specified by the power of 2
// i.e. ring buffer size = two to the URG3D_RB_BITSHIFT-th power.
enum {
//    URG3D_RB_BITSHIFT = 8,   //2014-10-23 Doog
    URG3D_RB_BITSHIFT = URG3D_MAX_RX_BUFFER_BIT,
    URG3D_RB_SIZE = 1 << URG3D_RB_BITSHIFT,

    // caution ! available buffer size is less than the
    // size given to the ring buffer(URG3D_RB_SIZE).
    URG3D_BUFSIZE = URG3D_RB_SIZE - 1,
};


//! TCP/IP connection
typedef struct {
    // socket
    struct sockaddr_in server_addr;
    int sock_desc;
    int sock_addr_size;

    // buffer
    urg3d_ring_buffer_t rb;
    char buf[URG3D_RB_SIZE];

    // line reading functions
    int pushed_back; // for pushded back char

} urg3d_tcpclient_t;
// -- end of NON INTERFACE definitions --


// -- belows are MODULE INTERFACES --
/*!
  \brief constructor of tcp client module

  \param[in,out] cli tcp client type variable which must be allocated by a caller.
  \param[in] server_ip_str IP address expressed in string, i.e. "192.168.0.1"
  \param[in] port_num port number expressed in integer, i.e. port_num = 10200

  \retval 0 succeeded.
  \retval -1 error
*/
extern int urg3d_tcpclient_open(urg3d_tcpclient_t* cli,
                          const char* server_ip_str, int port_num);


/*!
  \brief destructor of tcp client module

  \param[in,out] cli : tcp client type variable which must be deallocated by a caller after closing.
*/
extern void urg3d_tcpclient_close(urg3d_tcpclient_t* cli);


/*!
  \brief read from socket.

  \param[in,out] cli : tcp client type variable which must be deallocated by a caller after closing.
  \param[out] userbuf : buffer to store read data which must be allocated by a caller.
  \param[in] req_size: data size requested to read in byte.
  \param[in] timeout : time out specification which unit is microsecond.

  \return the number of data read, -1 when error.
*/

extern int urg3d_tcpclient_read(urg3d_tcpclient_t* cli,
                          char* userbuf, int req_size, int timeout);

extern int urg3d_tcpclient_read_and_set_ringbuffer(urg3d_tcpclient_t* cli);


/*!
  \brief write to socket.

  \param[in,out] cli : tcp client type variable which must be deallocated by a caller after closing.
  \param[in] userbuf : data to write.
  \param[in] req_size: data size requested to write in byte.

  \return returns the number of data wrote, -1 when error.
*/
extern int urg3d_tcpclient_write(urg3d_tcpclient_t* cli,
                           const char* userbuf, int req_size);


//! \attention not implemented yet.
extern int urg3d_tcpclient_error(urg3d_tcpclient_t* cli,
                           char* error_message, int max_size);


/*!
  \brief read one line from socket.

  \param[in,out] cli : tcp client type variable which must be deallocated by a caller after closing.
  \param[out] userbuf : buffer to store read data which must be allocated by a caller.
  \param[in] buf_size: data size requested to read in byte.
  \param[in] timeout : time out specification which unit is microsecond.

  \return the number of data read, -1 when error.
*/
extern int urg3d_tcpclient_readline(urg3d_tcpclient_t* cli,
                              char* userbuf, int buf_size, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* !URG3D_TCPCLIENT_H */
