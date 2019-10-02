/*!
  \file
  \brief functions to handle communication with urg sensor (ethernet)

  \author Satofumi KAMIMURA

  $Id: urg3d_connection.c,v 67b996051e49 2014/07/17 01:07:06 jun $
*/

#include "urg3d_connection.h"


int urg3d_connection_open(urg3d_connection_t *connection
                    , urg3d_connection_type_t urg3d_connection_type
                    , const char *device, long baudrate_or_port)
{
    connection->type = urg3d_connection_type;

    switch (urg3d_connection_type) {
    /*
    case URG3D_SERIAL:
        return serial_open(&connection->serial, device, baudrate_or_port);
        break;
    */
    case URG3D_ETHERNET:
        return urg3d_tcpclient_open(&connection->tcpclient,
                              device, baudrate_or_port);
        break;
    }
    return -1;
}


void urg3d_connection_close(urg3d_connection_t *connection)
{
    switch (connection->type) {
    /*
    case URG3D_SERIAL:
        serial_close(&connection->serial);
        break;
    */
    case URG3D_ETHERNET:
        urg3d_tcpclient_close(&connection->tcpclient);
        break;
    }
}


int urg3d_connection_set_baudrate(urg3d_connection_t *connection
                            , long baudrate)
{
    int ret = -1;

    switch (connection->type) {
    /*
    case URG3D_SERIAL:
        ret = serial_set_baudrate(&connection->serial, baudrate);
        break;
    */
    case URG3D_ETHERNET:
        ret = 0;
        break;
    }

    return ret;
}


int urg3d_connection_write(urg3d_connection_t *connection
                     , const char *data
                     , int size)
{
    switch (connection->type) {
    /*
    case URG3D_SERIAL:
        return serial_write(&connection->serial, data, size);
        break;
    */
    case URG3D_ETHERNET:
        return urg3d_tcpclient_write(&connection->tcpclient, data, size);
        break;
    }
    return -1;
}


int urg3d_connection_read(urg3d_connection_t *connection
                    , char *data
                    , int max_size
                    , int timeout)
{
    switch (connection->type) {
    /*
    case URG3D_SERIAL:
        return serial_read(&connection->serial, data, max_size, timeout);
        break;
    */
    case URG3D_ETHERNET:
        return urg3d_tcpclient_read(&connection->tcpclient, data, max_size, timeout);
        break;
    }
    return -1;
}
