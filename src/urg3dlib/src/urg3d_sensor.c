#include "urg3d_sensor.h"
#include "urg3d_errno.h"
#include "urg3d_ticks.h"
#if defined(URG3D_WINDOWS_OS)
#else
#include <unistd.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

//#define DEBUG_COMMON
//#define DEBUG_LOW
//#define DEBUG_HIGH

enum {
    URG3D_FALSE = 0,
    URG3D_TRUE = 1,
};

int urg3d_open(urg3d_t* const urg
               , const char* const address
               , int port)
{
    int ret = 0;
    urg->is_active = URG3D_FALSE;
    urg->last_errno = URG3D_NOT_CONNECTED;
    urg->blocking_timeout_ms = 1000;
    urg->nextHeaderReady = 0;
    urg->nextDataReady = 0;

    //connect
    if ((ret = urg3d_connection_open(&urg->connection, URG3D_ETHERNET, address, port)) < 0) {
        urg->last_errno = URG3D_ETHERNET_OPEN_ERROR;
        return urg->last_errno;
    }

    urg->is_active = URG3D_TRUE;
    return 0;
}

int urg3d_close( urg3d_t* const urg)
{
    urg3d_connection_close(&urg->connection);
    urg->is_active = URG3D_FALSE;
    return 0;
}

//same urg3d_ring_buffer.c
static void byte_move(char *dest
                      , const char *src
                      , int n)
{
    const char *last_p = dest + n;
    while (dest < last_p) {
        *dest++ = *src++;
    }
}

//modified urg3d_ring_buffer.c
//no move ring->first
static int urg3d_ring_read_pre(urg3d_ring_buffer_t *ring
                         , char *buffer
                         , int size)
{
    int now_size = urg3d_ring_size(ring);
    int pop_size = (size > now_size) ? now_size : size;

    if (ring->first <= ring->last) {
        byte_move(buffer, &ring->buffer[ring->first], pop_size);
    } else {
        int left_size = 0;
        int to_end = ring->buffer_size - ring->first;
        int move_size = (to_end > pop_size) ? pop_size : to_end;
        byte_move(buffer, &ring->buffer[ring->first], move_size);

        left_size = pop_size - move_size;
        if (left_size > 0) {
            byte_move(&buffer[move_size], ring->buffer, left_size);
        }
    }
    return pop_size;
}

//modified urg3d_tcpclient.c
//non-blocking and write all data for ring-buffer
static void urg3d_read( urg3d_t* const urg)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    int num_in_buf = urg3d_ring_size(&cli->rb);
    int sock       = cli->sock_desc;
    int n;
    char tmpbuf[URG3D_BUFSIZE];

    // receive with non-blocking mode.
#if defined(URG3D_WINDOWS_OS)
    int no_timeout = 1;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&no_timeout, sizeof(struct timeval));
    n = recv(sock, tmpbuf, URG3D_BUFSIZE - num_in_buf, 0);
#else
    n = recv(sock, tmpbuf, URG3D_BUFSIZE - num_in_buf, MSG_DONTWAIT);
#endif
    if (n > 0) {
        urg3d_ring_write(&cli->rb, tmpbuf, n); // copy socket to my buffer
    }
#ifdef DEBUG_COMMON
    printf("receive_num = %d\n", n);
#endif
}

int urg3d_next_receive_ready( urg3d_t* const urg)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    char VSSP_trush[4] = {0};

    //need
    urg3d_read(urg);

    if(urg->nextHeaderReady == 0) {
        //search VSSP
        while(1) {
            if(urg3d_ring_size(&cli->rb) <= 4) {
                return 0;
            }
            urg3d_ring_read_pre(&cli->rb, VSSP_trush,  4);
            if(strncmp(VSSP_trush, "VSSP", 4) == 0) {
                break;
            } else {
                urg3d_ring_read(&cli->rb, VSSP_trush, 1);
            }
        }

        //check VSSP header length
        if(urg3d_ring_size(&cli->rb) < URG3D_VSSP_HEADER_LENGTH) {
            return 0;
        } else {
            //pre read VSSP header
            urg3d_ring_read_pre(&cli->rb, (char*)(&urg->nextHeader), URG3D_VSSP_HEADER_LENGTH);
            urg->nextHeaderReady = 1;
        }
    }

    if(urg->nextDataReady == 0) {
        //check data length
        if(urg3d_ring_size(&cli->rb) < ((*(unsigned short *)(urg->nextHeader + 14)))) {
            return 0;
        } else {
            urg->nextDataReady = 1;
        }
    }
    return urg->nextHeaderReady && urg->nextDataReady;
}

static void urg3d_next_receive_flag_clear( urg3d_t* const urg)
{
    urg->nextHeaderReady = 0;
    urg->nextDataReady = 0;
}

int urg3d_low_request_command( urg3d_t* const urg
                               , const char* const command)
{
    return urg3d_connection_write(&urg->connection, command, strlen(command));
}

int urg3d_low_get_binary( urg3d_t* const urg
                          , urg3d_vssp_header_t* const header
                          , char* const data
                          , int* const length_data)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    int ret = 0;

    if((ret = urg3d_next_receive_ready(urg)) <= 0) {
        return ret;
    }

    //copy
    urg3d_ring_read(&cli->rb, (char*)header, URG3D_VSSP_HEADER_LENGTH);
    *length_data = header->length - header->header_length;
    if(*length_data != 0) {
        urg3d_ring_read(&cli->rb, (char*)data, *length_data);
    }
    data[*length_data] = '\0';

    urg3d_next_receive_flag_clear(urg);

#ifdef DEBUG_LOW
    printf("sizeof(*header) = %d\n", sizeof(*header));
    printf("header->mark = %.4s\n", header->mark);
    printf("header->type = %.3s\n" ,header->type);
    printf("header->status  = %.3s\n", header->status);
    printf("header->header_length = %u\n", header->header_length);
    printf("header->length = %u\n", header->length);
    printf("header->received_time_ms = %lu\n", header->received_time_ms);
    printf("header->send_time_ms = %lu\n", header->send_time_ms);
    printf("*length_data = %d\n", *length_data);
    printf("strlen(data) = %d\n", strlen(data));
    printf("data = %s\n", data);

    printf("\n");
#endif

    return 1;
}
static int urg3d_low_get_ri_ro_common( urg3d_t* const urg
                                       , urg3d_vssp_header_t* const header
                                       , urg3d_range_header_t* const range_header
                                       , urg3d_range_index_t* const range_index)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    char buf[URG3D_MAX_RX_LENGTH] = {0};

    //copy
    urg3d_ring_read(&cli->rb, (char*)header, URG3D_VSSP_HEADER_LENGTH);

    //copy range_header
    urg3d_ring_read(&cli->rb, buf, 2);
    range_header->header_length = (*(unsigned short *)(buf + 0));
    //*** CAUTION *** sizeof(range_header) != 20
    urg3d_ring_read(&cli->rb, buf, range_header->header_length -2);
    range_header->line_head_timestamp_ms = (*(unsigned int *)(buf + 0));
    range_header->line_tail_timestamp_ms = (*(unsigned int *)(buf + 4));
    range_header->line_head_h_angle_ratio = (*(signed short *)(buf + 8));
    range_header->line_tail_h_angle_ratio = (*(signed short *)(buf + 10));
    range_header->frame = (*(unsigned char *)(buf + 12));
    range_header->h_field = (*(unsigned char *)(buf + 13));
    range_header->line = (*(unsigned short *)(buf + 14));
    range_header->spot = (*(unsigned short *)(buf + 16));
    if(range_header->header_length == 24){
        range_header->v_field = (*(unsigned char *)(buf + 18));
        range_header->v_interlace = (*(unsigned char *)(buf + 19));
    }
    else{
        range_header->v_field = 0;
        range_header->v_interlace = 1;
    }

    //copy range_index
    urg3d_ring_read(&cli->rb, buf, 4);
    range_index->index_length = (*(unsigned short *)(buf + 0));
    range_index->nspots = (*(unsigned short *)(buf + 2));
    urg3d_ring_read(&cli->rb, (char*)(&range_index->index), (range_index->nspots+1) * sizeof(unsigned short));

    //skip reserve
    if(range_index->nspots % 2 == 0) {
        urg3d_ring_read(&cli->rb, buf, 2);
    }

    return 1;
}

int urg3d_low_get_ri( urg3d_t* const urg, urg3d_vssp_header_t* const header
                      , urg3d_range_header_t* const range_header
                      , urg3d_range_index_t* const range_index
                      , urg3d_data_range_intensity_t* const data_range_intensity)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    int ret = 0;

    if((ret = urg3d_next_receive_ready(urg)) <= 0) {
        return ret;
    }

    //check
    if(strncmp(&urg->nextHeader[4], "_ri", 3) != 0) {
        return 0;
    }

    //common method
    urg3d_low_get_ri_ro_common(urg, header, range_header, range_index);

    //copy data_range_intensity
    urg3d_ring_read(&cli->rb, (char*)data_range_intensity, range_index->index[range_index->nspots] * sizeof(urg3d_raw_range_intensity_t));

    urg3d_next_receive_flag_clear(urg);

#ifdef DEBUG_LOW
    printf("sizeof(*header) = %d\n", sizeof(*header));
    printf("header->mark = %.4s\n", header->mark);
    printf("header->type = %.3s\n", header->type);
    printf("header->status = %.3s\n", header->status);
    printf("header->header_length = %u\n", header->header_length);
    printf("header->length = %u\n", header->length);
    printf("header->received_time_ms = %lu\n", header->received_time_ms);
    printf("header->send_time_ms = %lu\n", header->send_time_ms);
    printf("sizeof(*range_header) = %d\n", sizeof(*range_header));
    printf("range_header->header_length = %u\n", range_header->header_length);
    printf("range_header->line_head_timestamp_ms = %lu\n", range_header->line_head_timestamp_ms);
    printf("range_header->line_tail_timestamp_ms = %lu\n", range_header->line_tail_timestamp_ms);
    printf("range_header->line_head_h_angle_ratio = %d\n", range_header->line_head_h_angle_ratio);
    printf("range_header->line_tail_h_angle_ratio = %d\n",range_header->line_tail_h_angle_ratio);
    printf("range_header->frame = %u\n", range_header->frame);
    printf("range_header->h_field = %u\n", range_header->h_field);
    printf("range_header->line = %u\n", range_header->line);
    printf("range_header->spot = %u\n", range_header->spot);
    printf("sizeof(*range_index) = %d\n", sizeof(*range_index));
    printf("range_index->index_length = %u\n", range_index->index_length);
    printf("range_index->nspots = %u\n", range_index->nspots);
    for(auto i = 0; i < range_index->nspots + 1; i++) {
        printf("range_index->index[%d] = %u\n", i, range_index->index[i]);
    }
    printf("sizeof(*data_range_intensity) = %d\n", sizeof(*data_range_intensity));
    for(auto i = 0; i < range_index->index[range_index->nspots]; i++) {
        //printf("data_range_intensity->raw[%d].range_mm = %u,  data_range_intensity->raw[%d].intensity = %u\n",  i, data_range_intensity->raw[i].range_mm, i, data_range_intensity->raw[i].intensity);
    }
    printf("\n");
#endif

    return 1;
}

int urg3d_low_get_ro( urg3d_t* const urg
                      , urg3d_vssp_header_t* const header
                      , urg3d_range_header_t* const range_header
                      , urg3d_range_index_t* const range_index
                      , urg3d_data_range_t* const data_range)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    int ret = 0;

    if((ret = urg3d_next_receive_ready(urg)) <= 0) {
        return ret;
    }

    //check
    if(strncmp(&urg->nextHeader[4], "_ro", 3) != 0) {
        return 0;
    }

    //common method
    urg3d_low_get_ri_ro_common(urg, header, range_header, range_index);

    //copy data_range
    urg3d_ring_read(&cli->rb, (char*)data_range, range_index->index[range_index->nspots] * sizeof(urg3d_raw_range_t));

    urg3d_next_receive_flag_clear(urg);

#ifdef DEBUG_LOW
    printf("sizeof(*header) = %d\n", sizeof(*header));
    printf("header->mark = %.4s\n", header->mark);
    printf("header->type = %.3s\n", header->type);
    printf("header->status = %.3s\n", header->status);
    printf("header->header_length = %u\n", header->header_length);
    printf("header->length = %u\n", header->length);
    printf("header->received_time_ms = %lu\n", header->received_time_ms);
    printf("header->send_time_ms = %lu\n", header->send_time_ms);
    printf("sizeof(*range_header) = %d\n", sizeof(*range_header));
    printf("range_header->header_length = %u\n", range_header->header_length);
    printf("range_header->line_head_timestamp_ms = %lu\n", range_header->line_head_timestamp_ms);
    printf("range_header->line_tail_timestamp_ms = %lu\n", range_header->line_tail_timestamp_ms);
    printf("range_header->line_head_h_angle_ratio = %d\n", range_header->line_head_h_angle_ratio);
    printf("range_header->line_tail_h_angle_ratio = %d\n",range_header->line_tail_h_angle_ratio);
    printf("range_header->frame = %u\n", range_header->frame);
    printf("range_header->h_field = %u\n", range_header->h_field);
    printf("range_header->line = %u\n", range_header->line);
    printf("range_header->spot = %u\n", range_header->spot);
    printf("sizeof(*range_index) = %d\n", sizeof(*range_index));
    printf("range_index->index_length = %u\n", range_index->index_length);
    printf("range_index->nspots = %u\n", range_index->nspots);
    for(auto i = 0; i < range_index->nspots+1; i++) {
        printf("range_index->index[%d] = %u\n", i, range_index->index[i]);
    }
    printf("sizeof(*data_range) = %d\n", sizeof(*data_range));
    for(auto i = 0; i < range_index->index[range_index->nspots]; i++) {
        //printf("data_range->raw[%d].range_mm = %u\n",  i, data_range->raw[i].range_mm);
    }
    printf("\n");
#endif

    return 1;
}

int urg3d_low_get_ax( urg3d_t* const urg
                      , urg3d_vssp_header_t* const header
                      , urg3d_ax_header_t* const ax_header
                      , urg3d_data_ax_t* const ax_data)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    int ret = 0, count = 0;
    char buf[URG3D_MAX_RX_LENGTH] = {0};

    if((ret = urg3d_next_receive_ready(urg)) <= 0) {
        return ret;
    }

    //check
    if(strncmp(&urg->nextHeader[4], "_ax", 3) != 0) {
        return 0;
    }

    //copy vssp_header
    urg3d_ring_read(&cli->rb, (char*)header, URG3D_VSSP_HEADER_LENGTH);

    //copy ax_header
    //*** CAUTION *** sizeof(ax_header) != 12
    urg3d_ring_read(&cli->rb, buf, 12);
    ax_header->header_length = (*(unsigned short *)(buf + 0));
    ax_header->timestamp_ms = (*(unsigned int *)(buf + 2));
    ax_header->data_bitfield = (*(unsigned int *)(buf + 6));
    ax_header->data_count = (*(unsigned char *)(buf + 10));
    ax_header->data_ms = (*(unsigned char *)(buf + 11));

    //copy ax_data
    count = (header->length - header->header_length - ax_header->header_length) / sizeof(signed int);
    urg3d_ring_read(&cli->rb, (char*)(ax_data->value), count * sizeof(signed int));

    urg3d_next_receive_flag_clear(urg);

#ifdef DEBUG_LOW
    printf("sizeof(*header) = %d\n", sizeof(*header));
    printf("header->mark = %.4s\n", header->mark);
    printf("header->type = %.3s\n", header->type);
    printf("header->status = %.3s\n", header->status);
    printf("header->header_length = %u\n", header->header_length);
    printf("header->length = %u\n", header->length);
    printf("header->received_time_ms = %lu\n", header->received_time_ms);
    printf("header->send_time_ms = %lu\n", header->send_time_ms);
    printf("sizeof(*ax_header) = %d\n", sizeof(*ax_header));
    printf("ax_header->header_length = %u\n", ax_header->header_length);
    printf("ax_header->timestamp_ms = %lu\n", ax_header->timestamp_ms);
    printf("ax_header->data_bitfield = 0x%lx\n", ax_header->data_bitfield);
    printf("ax_header->data_count = %d\n",ax_header->data_count);
    printf("ax_header->data_ms = %d\n", ax_header->data_ms);
    printf("sizeof(*ax_data) = %d\n", sizeof(*ax_data));
    for(auto i = 0; i < count; i++) {
        printf("ax_data->value[%d] = %ld\n", i, ax_data->value[i]);
    }
    printf("\n");
#endif

    return 1;
}

void urg3d_low_purge(urg3d_t * const urg)
{
    urg3d_tcpclient_t *cli = &urg->connection.tcpclient;
    urg3d_ring_clear(&cli->rb);
}

static int urg3d_high_blocking_common(urg3d_t *urg, urg3d_vssp_header_t* const header
                                      , char* const data
                                      , const char* const command
                                      , const char* const type
                                      , const char* const data_head)
{
    int length_data = 0;
    long start_time_ms;

    if(urg3d_low_request_command(urg, command) < 0) {
        return -1;
    }

    start_time_ms = urg3d_ticks_ms();
    while(1) {
        while(urg3d_low_get_binary(urg, header, data, &length_data) > 0) {
            if(strncmp(header->type, "ERR", 3) == 0) {
                return -1;
            }
            if(strncmp(header->type, "_er", 3) == 0) {
                return -1;
            }
            if(strncmp(header->type, type, 3) == 0) {
                if(data_head == NULL || strncmp(data, data_head, sizeof(data_head)-1) == 0) {
                    return 1;
                }
            }
        }
        if(urg3d_ticks_ms() - start_time_ms >= urg->blocking_timeout_ms) {
            return -2;
        }
#ifdef URG3D_WINDOWS_OS
        Sleep(10);
#else
        usleep(10000);
#endif
    }
}

static unsigned int hexCharsToU32(char* hex
                                  , int begin
                                  , int length)
{
    int i;
    unsigned int ret = 0;
    for(i=0; i<length; ++i) {
        if(hex[begin+i]>='0' && hex[begin+i]<='9') {
            ret |= ((unsigned int)hex[begin+i]-'0') << ((length-1-i) * 4);
        }
        if(hex[begin+i]>='A' && hex[begin+i]<='F') {
            ret |= ((unsigned int)hex[begin+i]-'A'+10) << ((length-1-i) * 4);
        }
        if(hex[begin+i]>='a' && hex[begin+i]<='f') {
            ret |= ((unsigned int)hex[begin+i]-'a'+10) << ((length-1-i) * 4);
        }
    }
    return ret;
}

static int hexCharsToS32(char* hex, int begin, int length)
{
    return (int)hexCharsToU32(hex, begin, length);
}

void urg3d_high_set_blocking_timeout_ms(urg3d_t* const urg
                                        , int t_ms)
{
    urg->blocking_timeout_ms = t_ms;
}

int urg3d_high_blocking_init(urg3d_t * const urg)
{
    int ret = 0;
    int i = 0, s = 0;
    urg3d_vssp_header_t header;
    char data[URG3D_MAX_RX_LENGTH] = {0};
    double intl = 0;

    for(i = 0; i < URG3D_MAX_V_INTERLACE_COUNT; ++i) {
        urg->spot_v_angle_loaded[i] = 0;
    }

    ret = urg3d_high_blocking_common(urg, &header, data, "GET:tblv\n", "GET", "GET:tblv\n");
    if(ret < 0) {
        return ret;
    }
    for(i = 9, s = 0; i+4 <= header.length - header.header_length; i+=5, ++s) {
        intl = hexCharsToU32(data, i, 4)*(2.0*M_PI)/65535.0;
        urg->spot_v_angle_rad[0][0][s] = intl;
        urg->cos_v_angle_rad[0][0][s] = cos(intl);
        urg->sin_v_angle_rad[0][0][s] = sin(intl);
    }

    urg->spot_v_angle_loaded[0] = 1;

#ifdef DEBUG_HIGH
    printf("sizeof(urg->spot_v_angle_rad) = %d\n", sizeof(urg->spot_v_angle_rad));
    for(i = 9, s = 0; i+4 <= header.length - header.header_length; i+=5, ++s) {
        printf("urg->spot_v_angle_rad[%d] = %lf\n", s, urg->spot_v_angle_rad[s]);
    }
    printf("\n");
#endif

    ret = urg3d_high_blocking_common(urg, &header, data, "GET:tblh\n", "GET", "GET:tblh\n");
    if(ret < 0) {
        return ret;
    }
    for(i = 9, s = 0; i+4 <= header.length - header.header_length; i+=5, ++s) {
        urg->spot_h_angle_ratio[s] = hexCharsToU32(data, i, 4)/65535.0;
    }

#ifdef DEBUG_HIGH
    printf("sizeof(urg->spot_h_angle_ratio) = %d\n", sizeof(urg->spot_h_angle_ratio));
    for(i = 9, s = 0; i+4 <= header.length - header.header_length; i+=5, ++s) {
        printf("urg->spot_h_angle_ratio[%d] = %lf\n", s, urg->spot_h_angle_ratio[s]);
    }
    printf("\n");
#endif

    return 1;
}

int urg3d_high_blocking_wait_finished_initialize(urg3d_t * const urg)
{
    int ret = 0;
    urg3d_vssp_header_t header;
    char *data_p, data[URG3D_MAX_RX_LENGTH] = {0};

    while(1){
        int ro = 0, ri = 0, ax = 0;
        ret = urg3d_high_blocking_common(urg, &header, data, "GET:stat\n", "GET", "GET:stat\n");
        if(ret < 0) {
            return ret;
        }
        data_p = data;
        while(data_p[0] != '\0') {
            if(strncmp(data_p, "_ro=", 4) == 0) {
                if(strncmp(data_p+4, "000", 3) == 0) { ro = 1; }
                if(strncmp(data_p+4, "099", 3) == 0) { ro = 99; }
                if(ro == 0) { ro = -1; }
            }
            if(strncmp(data_p, "_ri=", 4) == 0) {
                if(strncmp(data_p+4, "000", 3) == 0) { ri = 1; }
                if(strncmp(data_p+4, "099", 3) == 0) { ri = 99; }
                if(ro == 0) { ro = -1; }
            }
            if(strncmp(data_p, "_ax=", 4) == 0) {
                if(strncmp(data_p+4, "000", 3) == 0) { ax = 1; }
                if(strncmp(data_p+4, "099", 3) == 0) { ax = 99; }
                if(ax == 0) { ax = -1; }
            }
            while(data_p[0] != '\0' && data_p[0] != '\n') {
                ++data_p;
            }
            if(data_p[0] == '\n') {
                ++data_p;
            }
        }
#ifdef DEBUG_HIGH

#endif
        if(ro == -1 || ri == -1 || ax == -1){ return -1; }
        if(ro == 1 && ri == 1 && ax == 1){ break; }
    }

    return 1;
}

int urg3d_high_blocking_get_sensor_version(urg3d_t* const urg
                                           , urg3d_sensor_version_t* const version)
{
    urg3d_vssp_header_t header;
    char *data_p, data[URG3D_MAX_RX_LENGTH] = {0};
    int ret = 0;

    ret = urg3d_high_blocking_common(urg, &header, data, "VER\n", "VER", NULL);
    if(ret < 0) {
        return ret;
    }

    version->vendor[0] = '\0';
    version->product[0] = '\0';
    version->serial[0] = '\0';
    version->firmware[0] = '\0';
    version->protocol[0] = '\0';
    data_p = data;
    while(data_p[0] != '\0') {
        if(strncmp(data_p, "vend:", 5) == 0) {
            sscanf(data_p, "vend:%[^\n]\n", version->vendor);
        }
        if(strncmp(data_p, "prod:", 5) == 0) {
            sscanf(data_p, "prod:%[^\n]\n", version->product);
        }
        if(strncmp(data_p, "seri:", 5) == 0) {
            sscanf(data_p, "seri:%[^\n]\n", version->serial);
        }
        if(strncmp(data_p, "firm:", 5) == 0) {
            sscanf(data_p, "firm:%[^\n]\n", version->firmware);
        }
        if(strncmp(data_p, "prot:", 5) == 0) {
            sscanf(data_p, "prot:%[^\n]\n", version->protocol);
        }
        while(data_p[0] != '\0' && data_p[0] != '\n') {
            ++data_p;
        }
        if(data_p[0] == '\n') {
            ++data_p;
        }
    }

#ifdef DEBUG_HIGH
    printf("version->vendor = %s\n", version->vendor);
    printf("version->product = %s\n", version->product);
    printf("version->serial = %s\n", version->serial);
    printf("version->firmware = %s\n", version->firmware);
    printf("version->protocol = %s\n", version->protocol);
    printf("\n");
#endif

    return 1;
}

int urg3d_high_blocking_get_horizontal_interlace_count(urg3d_t *const urg
                                                       , int *count)
{
    urg3d_vssp_header_t header;
    char data[URG3D_MAX_RX_LENGTH] = {0};
    int ret = 0;

    ret = urg3d_high_blocking_common(urg, &header, data, "GET:_itl\n", "GET", "GET:_itl\n");
    if(ret < 0) {
        return ret;
    }

    sscanf(data, "GET:_itl\n0,%d", count);

#ifdef DEBUG_HIGH
    printf("sizeof(*count) = %d\n", sizeof(*count));
    printf("*count = %d\n", *count);
    printf("\n");
#endif
    return 1;
}

int urg3d_high_blocking_get_vertical_interlace_count(urg3d_t *const urg
                                                     , int *count)
{
    urg3d_vssp_header_t header;
    char data[URG3D_MAX_RX_LENGTH] = {0};
    int ret = 0;

    ret = urg3d_high_blocking_common(urg, &header, data, "GET:_itv\n", "GET", "GET:_itv\n");
    if(ret < 0) {
        return ret;
    }

    sscanf(data, "GET:_itv\n0,%d", count);

#ifdef DEBUG_HIGH
    printf("sizeof(*count) = %d\n", sizeof(*count));
    printf("*count = %d\n", *count);
    printf("\n");
#endif
    return 1;
}

int urg3d_high_blocking_set_horizontal_interlace_count(urg3d_t *const urg
                                                       , int count)
{
    urg3d_vssp_header_t header;
    char data[URG3D_MAX_RX_LENGTH] = {0};
    int ret = 0;
    char command[URG3D_MAX_TX_LENGTH] = {0};

    //check
    if(count < 1 || count > URG3D_MAX_H_INTERLACE_COUNT) {
        return -1;
    }

    sprintf(command, "SET:_itl=0,%02d\n", count);
    ret = urg3d_high_blocking_common(urg, &header, data, command, "SET", command);
    if(ret < 0) {
        return ret;
    }

    return 1;
}

int urg3d_high_blocking_set_vertical_interlace_count(urg3d_t *const urg
                                                     , int count)
{
    urg3d_vssp_header_t header;
    char data[URG3D_MAX_RX_LENGTH] = {0};
    int ret = 0, i = 0, s = 0;
    int v_intl_index = 0;
    char command[URG3D_MAX_TX_LENGTH] = {0};
    double intl = 0;

    //check
    if(count < 1 || count > URG3D_MAX_V_INTERLACE_COUNT) {
        return -1;
    }

    sprintf(command, "SET:_itv=0,%02d\n", count);
    ret = urg3d_high_blocking_common(urg, &header, data, command, "SET", command);
    if(ret < 0) {
        return ret;
    }

    if(urg->spot_v_angle_loaded[count -1] == 0){
        for(v_intl_index = 0; v_intl_index < count; ++v_intl_index)
        {
            sprintf(command, "GET:tv%02d\n", v_intl_index);
            ret = urg3d_high_blocking_common(urg, &header, data, command, "GET", command);
            if(ret < 0) {
                return ret;
            }
            for(i = 9, s = 0; i+4 <= header.length - header.header_length; i+=5, ++s) {
                intl = hexCharsToU32(data, i, 4)*(2.0*M_PI)/65535.0;
                urg->spot_v_angle_rad[count-1][v_intl_index][s] = intl;
                urg->cos_v_angle_rad[count-1][v_intl_index][s] = cos(intl);
                urg->sin_v_angle_rad[count-1][v_intl_index][s] = sin(intl);
            }
        }
        urg->spot_v_angle_loaded[count -1] = 1;
    }

    return 1;
}

int urg3d_high_start_data(urg3d_t* const urg
                          , urg3d_measurement_type_t meas)
{
    const char command_list[4][10] = {"", "DAT:ro=1\n", "DAT:ri=1\n", "DAT:ax=1\n"};
    if(meas == URG3D_NO_REQUEST) {
        return 0;
    }
    return urg3d_low_request_command(urg, command_list[meas]);
}

int urg3d_high_stop_data(urg3d_t* const urg
                         , urg3d_measurement_type_t meas)
{
    const char command_list[4][10] = {"", "DAT:ro=0\n", "DAT:ri=0\n", "DAT:ax=0\n"};
    if(meas == URG3D_NO_REQUEST) {
        return 0;
    }
    return urg3d_low_request_command(urg, command_list[meas]);
}

int urg3d_high_get_measurement_data(urg3d_t * const urg
                                    , urg3d_measurement_data_t *data)
{
    int ret_ri = 0, ret_ro = 0;
    int i = 0;
    int spot, echo;
    urg3d_vssp_header_t lib_header;
    urg3d_range_header_t lib_range_header;
    urg3d_range_index_t lib_range_index;
    urg3d_data_range_intensity_t lib_data_range_intensity;
    urg3d_data_range_t lib_data_range;

    ret_ri = urg3d_low_get_ri(urg, &lib_header, &lib_range_header, &lib_range_index, &lib_data_range_intensity);
    if(ret_ri <= 0) {
        ret_ro = urg3d_low_get_ro(urg, &lib_header, &lib_range_header, &lib_range_index, &lib_data_range);
        if(ret_ro <= 0) {
            return 0;
        }
    }

    data->timestamp_ms = lib_range_header.line_head_timestamp_ms;
    data->frame_number = lib_range_header.frame;
    data->h_field_number = lib_range_header.h_field;
    data->v_field_number = lib_range_header.v_field;
    data->line_number = lib_range_header.line;
    data->spot_count = lib_range_index.nspots;
    strncpy(data->status, lib_header.status, sizeof(data->status));

//#ifdef DEBUG_HIGH
//	printf("data->timestamp_ms = %d\n", data->timestamp_ms);
//	printf("data->frame_number = %d\n", data->frame_number);
//	printf("data->h_field_number = %d\n", data->h_field_number);
//	printf("data->v_field_number = %d\n", data->v_field_number);
//	printf("data->line_number = %d\n", data->line_number);
//	printf("data->spot_count = %d\n", data->spot_count);
//#endif

    for(spot=0; spot < lib_range_index.nspots; ++spot) {
        const double horizontal_rad = ((double)lib_range_header.line_head_h_angle_ratio + (double)(lib_range_header.line_tail_h_angle_ratio - lib_range_header.line_head_h_angle_ratio)*urg->spot_h_angle_ratio[spot])/65535*2*M_PI;
        const double cos_horizontal_rad = cos(horizontal_rad);
        const double sin_horizontal_rad = sin(horizontal_rad);

        const double vertical_rad = urg->spot_v_angle_rad[lib_range_header.v_interlace -1][lib_range_header.v_field][spot];
        const double cos_vertical_rad = urg->cos_v_angle_rad[lib_range_header.v_interlace -1][lib_range_header.v_field][spot];
        const double sin_vertical_rad = urg->sin_v_angle_rad[lib_range_header.v_interlace -1][lib_range_header.v_field][spot];

        const double x_rad = cos_vertical_rad * cos_horizontal_rad;
        const double y_rad = cos_vertical_rad * sin_horizontal_rad;
        const double z_rad = sin_vertical_rad;

        data->spots[spot].echo_count = lib_range_index.index[spot+1] - lib_range_index.index[spot];
        for(echo=0; echo<data->spots[spot].echo_count; ++echo) {
            data->spots[spot].polar[echo].range_m = 0;
            data->spots[spot].polar[echo].vertical_rad = vertical_rad;
            data->spots[spot].polar[echo].horizontal_rad = horizontal_rad;
            data->spots[spot].polar[echo].intensity = 0;
        }
        for(i=lib_range_index.index[spot], echo=0; i < lib_range_index.index[spot+1]; ++i,++echo) {
            const double range_m = (ret_ri > 0 ? lib_data_range_intensity.raw[i].range_mm : lib_data_range.raw[i].range_mm)/1000.0;
            data->spots[spot].polar[echo].range_m = range_m;
            data->spots[spot].polar[echo].intensity = ret_ri ? lib_data_range_intensity.raw[i].intensity : 0;

            data->spots[spot].point[echo].x_m = range_m * x_rad;
            data->spots[spot].point[echo].y_m = range_m * y_rad;
            data->spots[spot].point[echo].z_m = range_m * z_rad;
            data->spots[spot].point[echo].intensity = data->spots[spot].polar[echo].intensity;
        }
    }

#ifdef DEBUG_HIGH
	printf("data->frame_number = %d\n", data->frame_number);
    for(spot=0; spot < data->spot_count; ++spot) {
        for(echo=0; echo < data->spots[spot].echo_count; ++echo) {
            printf("data->spots[%d].point[%d].x_m = %lf\n", spot, echo, data->spots[spot].point[echo].x_m);
            printf("data->spots[%d].point[%d].y_m = %lf\n", spot, echo, data->spots[spot].point[echo].y_m);
            printf("data->spots[%d].point[%d].z_m = %lf\n", spot, echo, data->spots[spot].point[echo].z_m);
            printf("data->spots[%d].polar[%d].range_m = %lf\n", spot, echo, data->spots[spot].polar[echo].range_m);
            printf("data->spots[%d].polar[%d].vertical_rad = %lf\n", spot, echo, data->spots[spot].polar[echo].vertical_rad);
            printf("data->spots[%d].polar[%d].horizontal_rad = %lf\n", spot, echo, data->spots[spot].polar[echo].horizontal_rad);
            printf("data->spots[%d].polar[%d].intensity = %u\n", spot, echo, data->spots[spot].polar[echo].intensity);
        }
    }
    printf("\n");
#endif

    return 1;
}

int urg3d_high_get_auxiliary_data(urg3d_t* const urg
                                  , urg3d_auxiliary_data_t* data)
{
    int ret = 0;
    int i = 0, point = 0;
    urg3d_vssp_header_t lib_header;
    urg3d_ax_header_t lib_ax_header;
    urg3d_data_ax_t lib_ax_data;

    ret = urg3d_low_get_ax(urg, &lib_header, &lib_ax_header, &lib_ax_data);
    if(ret <= 0) {
        return ret;
    }

    data->timestamp_ms = lib_ax_header.timestamp_ms;
    data->type = URG3D_NO_RECORD;
    data->type |= ((lib_ax_header.data_bitfield & 0xe0000000) == 0xe0000000)?URG3D_GYRO_DATA:URG3D_NO_RECORD;
    data->type |= ((lib_ax_header.data_bitfield & 0x1c000000) == 0x1c000000)?URG3D_ACCEL_DATA:URG3D_NO_RECORD;
    data->type |= ((lib_ax_header.data_bitfield & 0x03800000) == 0x03800000)?URG3D_COMPASS_DATA:URG3D_NO_RECORD;
    data->type |= ((lib_ax_header.data_bitfield & 0x00400000) == 0x00400000)?URG3D_TEMPERATURE_DATA:URG3D_NO_RECORD;
    data->record_count = lib_ax_header.data_count;
    strncpy(data->status, lib_header.status, sizeof(data->status));

    //clear
    memset(&data->records, 0, sizeof(urg3d_auxiliary_record_t)*URG3D_MAX_AUX_COUNT);

    //set
    for(i = 0; i < data->record_count; i++) {
        data->records[i].timestamp_ms = data->timestamp_ms + lib_ax_header.data_ms * i;

        if(lib_ax_header.data_bitfield & (1 << 31)) {
            data->records[i].gyro_x = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 30)) {
            data->records[i].gyro_y = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 29)) {
            data->records[i].gyro_z = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 28)) {
            data->records[i].accel_x = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 27)) {
            data->records[i].accel_y = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 26)) {
            data->records[i].accel_z = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 25)) {
            data->records[i].compass_x = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 24)) {
            data->records[i].compass_y = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 23)) {
            data->records[i].compass_z = lib_ax_data.value[point];
            ++point;
        }
        if(lib_ax_header.data_bitfield & (1 << 22)) {
            data->records[i].temperature = lib_ax_data.value[point];
            ++point;
        }
    }

#ifdef DEBUG_HIGH
    printf("data->timestamp_ms = %lu\n", data->timestamp_ms);
    printf("data->status = %.3s\n", data->status);
    printf("data->type = %d\n", data->type);
    printf("data->recode_count = %d\n", data->record_count);
    for(i = 0; i < data->record_count; i++) {
        printf("data->records[%d].timestamp_ms = %lu\n", i, data->records[i].timestamp_ms);
        printf("data->records[%d].gyro_x = %ld\n", i, data->records[i].gyro_x);
        printf("data->records[%d].gyro_y = %ld\n", i, data->records[i].gyro_y);
        printf("data->records[%d].gyro_z = %ld\n", i, data->records[i].gyro_z);
        printf("data->records[%d].accel_x = %ld\n", i, data->records[i].accel_x);
        printf("data->records[%d].accel_y = %ld\n", i, data->records[i].accel_y);
        printf("data->records[%d].accel_z = %ld\n", i, data->records[i].accel_z);
        printf("data->records[%d].compass_x = %ld\n", i, data->records[i].compass_x);
        printf("data->records[%d].compass_y = %ld\n", i, data->records[i].compass_y);
        printf("data->records[%d].compass_z = %ld\n", i, data->records[i].compass_z);
        printf("data->records[%d].temperature = %ld\n", i, data->records[i].temperature);
    }
    printf("\n");
#endif

    return 1;
}

int urg3d_high_blocking_restart(urg3d_t * const urg)
{
    urg3d_vssp_header_t header;
    char data[URG3D_MAX_RX_LENGTH] = {0};
    int ret = 0;
    char command[URG3D_MAX_TX_LENGTH] = {0};

    sprintf(command, "RST\n");
    ret = urg3d_high_blocking_common(urg, &header, data, command, "RST", command);
    if(ret < 0) {
        return ret;
    }

    return 1;
}
