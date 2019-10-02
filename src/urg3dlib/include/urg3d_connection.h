#ifndef URG3D_CONNECTION_H
#define URG3D_CONNECTION_H

/*!
  \file
  \brief functions to handle communication with urg sensor (ethernet)

  \author Satofumi KAMIMURA

  $Id: urg3d_connection.h,v 1d233c7a2240 2011/02/19 03:08:45 Satofumi $
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg3d_tcpclient.h"


/*!
  \brief constants
*/
enum {
    URG3D_CONNECTION_TIMEOUT = -1, //!< return value of connection timeout
};


/*!
  \brief communication type
*/
typedef enum {
    URG3D_ETHERNET,               //!< ethernet
} urg3d_connection_type_t;


/*!
  \brief communication resource
*/
typedef struct
{
    urg3d_connection_type_t type; //!< connection type
    urg3d_tcpclient_t tcpclient;  //!< ethernet
} urg3d_connection_t;


/*!
  \brief function to connect sensor

  \param[in,out] connection communication resource
  \param[in] urg3d_connection_type connection type
  \param[in] device device file name or ip address
  \param[in] baudrate_or_port baudrate or port number

  \retval 0 success
  \retval <0 error

  Example (ethernet communication)
  \code
  urg3d_connection_t connection;
  if (! urg3d_connection_open(&connection, URG3D_ETHERNET, "192.168.0.10", 10940)) {
      return 1;
  } \endcode

  \see urg3d_connection_close()
*/
extern int urg3d_connection_open(urg3d_connection_t *connection,
                           urg3d_connection_type_t urg3d_connection_type,
                           const char *device, long baudrate_or_port);


/*!
  \brief function to disconnect sensor

  \param[in,out] connection communication resource

  \code
  urg3d_connection_close(&connection); \endcode

  \see urg3d_connection_open()
*/
extern void urg3d_connection_close(urg3d_connection_t *connection);


/*!
  \brief function to set baudrate
*/
extern int urg3d_connection_set_baudrate(urg3d_connection_t *connection, long baudrate);


/*!
  \brief function to send data

  \param[in,out] connection communication resource
  \param[in] data sending data
  \param[in] size byte length of sending data

  \retval >=0 the number of characters sent (on success)
  \retval <0 error code

  Example
  \code
  n = urg3d_connection_write(&connection, "QT\n", 3); \endcode

  \see urg3d_connection_read(), urg3d_connection_readline()
*/
extern int urg3d_connection_write(urg3d_connection_t *connection,
                            const char *data, int size);


/*!
  \brief function to receive data

  \param[in,out] connection communication resource
  \param[in] data buffer for received data
  \param[in] max_size byte length of receive buffer
  \param[in] timeout timeout [msec]

  \retval >=0 the number of characters received (on success)
  \retval <0 error code (if no characters are received, this function returns URG3D_CONNECTION_TIMEOUT)

  If a negative value is set as "timeout", this function waits for receiving data.

  Example
  \code
enum {
    BUFFER_SIZE = 256,
    TIMEOUT_MSEC = 1000,
};
char buffer[BUFFER_SIZE];
n = urg3d_connection_read(&connection, buffer, BUFFER_SIZE, TIMEOUT_MSEC); \endcode

  \see urg3d_connection_write(), urg3d_connection_readline()
*/
extern int urg3d_connection_read(urg3d_connection_t *connection,
                           char *data, int max_size, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* !URG3D_CONNECTION_H */
