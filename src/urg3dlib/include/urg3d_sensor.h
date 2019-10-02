#ifndef URG3D_SENSOR_H
#define URG3D_SENSOR_H

#define URG3D_STATUS_LENGTH              (3)
#define URG3D_MAX_SPOTS_COUNT           (74)
#define URG3D_MAX_V_INTERLACE_COUNT     (10)
#define URG3D_MAX_H_INTERLACE_COUNT     (20)
#define URG3D_MAX_ECHOS_COUNT           (4)
#define URG3D_MAX_TYPE_COUNT            (32)
#define URG3D_MAX_AUX_COUNT             (255)
#define URG3D_MAX_STRING_SIZE           (64)
#define URG3D_MAX_RX_LENGTH             (1518)
#define URG3D_MAX_TX_LENGTH             (64)
#define URG3D_VSSP_HEADER_LENGTH        (24)

//#define URG3D_MAX_RX_BUFFER_BIT -> "urg3d_tcpclient.h"

#ifdef __cplusplus
extern "C" {
#endif
    #include "urg3d_connection.h"
    #include "urg3d_ring_buffer.h"

    /*!
      ==================== 3D-URG Common ====================
    */

    typedef struct {
        //connection
        int is_active;
        int last_errno;
        int blocking_timeout_ms;
        urg3d_connection_t connection;

        //vssp buffering
        char nextHeaderReady;
        char nextDataReady;
        char nextHeader[URG3D_VSSP_HEADER_LENGTH];

        //transform tables
        double spot_h_angle_ratio[URG3D_MAX_SPOTS_COUNT];
        double spot_v_angle_rad[URG3D_MAX_V_INTERLACE_COUNT][URG3D_MAX_V_INTERLACE_COUNT][URG3D_MAX_SPOTS_COUNT];
        double cos_v_angle_rad[URG3D_MAX_V_INTERLACE_COUNT][URG3D_MAX_V_INTERLACE_COUNT][URG3D_MAX_SPOTS_COUNT];
        double sin_v_angle_rad[URG3D_MAX_V_INTERLACE_COUNT][URG3D_MAX_V_INTERLACE_COUNT][URG3D_MAX_SPOTS_COUNT];
        char spot_v_angle_loaded[URG3D_MAX_V_INTERLACE_COUNT];
    } urg3d_t;

    /*!
      \brief open the connection to the sensor

      \param[in,out] urg : urg3d session
      \param[in] address : IP address expressed in string, i.e. address[] = "192.168.0.10"
      \param[in] port :  port number expressed in integer, i.e. port = 10940

      \retval 0 succeeded
      \retval other error
    */
    extern int urg3d_open( urg3d_t* const urg, const char* const address, int port);

    /*!
      \brief close the connection to the sensor

      \param[in,out] urg : urg3d session

      \retval 0 succeeded
      \retval other error
    */
    extern int urg3d_close( urg3d_t* const urg);

    /*!
      \brief check the next data has received

      \param[in,out] urg : urg3d session

      \retval 0 unreceived
      \retval other received
    */
    extern int urg3d_next_receive_ready( urg3d_t* const urg);

    /*!
      ==================== 3D-URG Low Layer ====================
    */

    typedef struct {
        char           mark[4];
        char           type[4];
        char           status[4];
        unsigned short header_length;
        unsigned short length;
        unsigned int   received_time_ms;
        unsigned int   send_time_ms;
    } urg3d_vssp_header_t;

    typedef struct {
        unsigned short header_length;
        unsigned int   line_head_timestamp_ms;
        unsigned int   line_tail_timestamp_ms;
        signed short   line_head_h_angle_ratio;
        signed short   line_tail_h_angle_ratio;
        unsigned char  frame;
        unsigned char  h_field;
        unsigned char  v_field;
        unsigned char  v_interlace;
        unsigned short line;
        unsigned short spot;
    } urg3d_range_header_t;

    typedef struct {
        unsigned short index_length;
        unsigned short nspots;
        unsigned short index[URG3D_MAX_SPOTS_COUNT+1]; //+1 means "unsigned short necho;"
    } urg3d_range_index_t;

    typedef struct {
        unsigned short range_mm;
        unsigned short intensity;
    } urg3d_raw_range_intensity_t;

    typedef struct {
        urg3d_raw_range_intensity_t raw[URG3D_MAX_SPOTS_COUNT*URG3D_MAX_ECHOS_COUNT];
    } urg3d_data_range_intensity_t;

    typedef struct {
        unsigned short range_mm;
    } urg3d_raw_range_t;

    typedef struct {
        urg3d_raw_range_t raw[URG3D_MAX_SPOTS_COUNT*URG3D_MAX_ECHOS_COUNT];
    } urg3d_data_range_t;

    typedef struct {
        unsigned short header_length;
        unsigned int   timestamp_ms;
        unsigned int   data_bitfield;
        unsigned char  data_count;
        unsigned char  data_ms;
    } urg3d_ax_header_t;

    typedef struct {
        signed int      value[URG3D_MAX_TYPE_COUNT*URG3D_MAX_AUX_COUNT];
    } urg3d_data_ax_t;

    /*!
      \brief send request command to device

      \param[in,out] urg : urg3d session
      \param[in] command  : VSSP command, i.e. command[] = "VER"

      \retval >=0 the number of data wrote
      \retval <0 error
    */
    extern int urg3d_low_request_command( urg3d_t* const urg, const char* const command);

    /*!
      \brief get binary data from revceived data buffer

      \param[in,out] urg : urg3d session
      \param[out] header  : urg3d_vssp_header_t struct
      \param[out] data  : binary data
      \param[out] length_data : the number of byte for data

      \retval 1 succeeded
      \retval 0 nothing happens
    */
    extern int urg3d_low_get_binary( urg3d_t* const urg, urg3d_vssp_header_t* const header,
                                     char* const data, int* const length_data);

    /*!
      \brief get range and intensity raw format from revceived data buffer

      \param[in,out] urg : urg3d session
      \param[out] header  :  urg3d_vssp_header_t struct
      \param[out] range_header  : urg3d_range_header_t struct
      \param[out] range_index : urg3d_range_index_t struct
      \param[out] data_range_intensity : urg3d_data_range_intensity_t struct

      \retval 1 succeeded
      \retval 0 nothing happens
    */
    extern int urg3d_low_get_ri( urg3d_t* const urg, urg3d_vssp_header_t* const header,
                                 urg3d_range_header_t* const range_header, urg3d_range_index_t* const range_index,
                                 urg3d_data_range_intensity_t* const data_range_intensity);

    /*!
      \brief get range only raw format from revceived data buffer

      \param[in,out] urg : urg3d session
      \param[out] header  : urg3d_vssp_header_t struct
      \param[out] range_header  : urg3d_range_header_t struct
      \param[out] range_index : urg3d_range_index_t struct
      \param[out] data_range : urg3d_data_range_intensity_t struct

      \retval 1 succeeded
      \retval 0 nothing happens
    */
    extern int urg3d_low_get_ro( urg3d_t* const urg, urg3d_vssp_header_t* const header,
                                 urg3d_range_header_t* const range_header, urg3d_range_index_t* const range_index,
                                 urg3d_data_range_t* const data_range);

    /*!
      \brief get auxiliary raw format from revceived data buffer

      \param[in,out] urg : urg3d session
      \param[out] header  : urg3d_vssp_header_t struct
      \param[out] ax_header  : urg3d_ax_header_t struct
      \param[out] ax_data : urg3d_data_ax_t sruct

      \retval 1 succeeded
      \retval 0  nothing happens
    */
    extern int urg3d_low_get_ax( urg3d_t* const urg, urg3d_vssp_header_t* const header,
                                 urg3d_ax_header_t* const ax_header, urg3d_data_ax_t* const ax_data);

    /*!
      \brief purge the data buffer

      \param[in,out] urg : urg3d session
    */
    extern void urg3d_low_purge(urg3d_t * const urg);

    /*!
      ==================== 3D-URG High Layer ====================
    */
    typedef enum {
        URG3D_NO_REQUEST                = 0, //No data available
        URG3D_DISTANCE                  = 1,
        URG3D_DISTANCE_INTENSITY        = 2,
        URG3D_AUXILIARY                 = 3,
    } urg3d_measurement_type_t;

    typedef struct {
        double range_m, vertical_rad, horizontal_rad;
        unsigned int intensity;
    } urg3d_polar_t;

    typedef struct {
        double x_m, y_m, z_m;
        unsigned int intensity;
    } urg3d_point_t;

    typedef struct {
        urg3d_polar_t  polar[URG3D_MAX_ECHOS_COUNT];
        urg3d_point_t  point[URG3D_MAX_ECHOS_COUNT];
        int            echo_count;
    } urg3d_spot_t;

    typedef struct {
        urg3d_spot_t   spots[URG3D_MAX_SPOTS_COUNT];
        unsigned int   timestamp_ms;
        unsigned char  frame_number;
        unsigned char  h_field_number;
        unsigned char  v_field_number;
        unsigned short line_number;
        unsigned short spot_count;
        char           status[URG3D_STATUS_LENGTH]; //without LF
    } urg3d_measurement_data_t;

    typedef struct {
        unsigned int   timestamp_ms;
        signed int     gyro_x, gyro_y, gyro_z;
        signed int     accel_x, accel_y, accel_z;
        signed int     compass_x, compass_y, compass_z;
        signed int     temperature;
    } urg3d_auxiliary_record_t;

    typedef enum {
        URG3D_NO_RECORD                 = (0),  //No data available
        URG3D_GYRO_DATA                 = (1 << 0),
        URG3D_ACCEL_DATA                = (1 << 1),
        URG3D_COMPASS_DATA              = (1 << 2),
        URG3D_TEMPERATURE_DATA          = (1 << 3),
    } urg3d_auxiliary_type_t;
	inline urg3d_auxiliary_type_t operator |(urg3d_auxiliary_type_t a, urg3d_auxiliary_type_t b)
	{ return static_cast<urg3d_auxiliary_type_t>(static_cast<int>(a) | static_cast<int>(b)); }
	inline urg3d_auxiliary_type_t& operator |=(urg3d_auxiliary_type_t& a, urg3d_auxiliary_type_t b)
	{ return a = a | b; }

    typedef struct {
        unsigned int   timestamp_ms;
        urg3d_auxiliary_record_t  records[URG3D_MAX_AUX_COUNT];
		urg3d_auxiliary_type_t   type;
        int                      record_count;
        char                     status[3]; //without LF
    } urg3d_auxiliary_data_t;

    typedef struct {
        char vendor[URG3D_MAX_STRING_SIZE];
        char product[URG3D_MAX_STRING_SIZE];
        char serial[URG3D_MAX_STRING_SIZE];
        char firmware[URG3D_MAX_STRING_SIZE];
        char protocol[URG3D_MAX_STRING_SIZE];
    } urg3d_sensor_version_t;

    /*!
      \brief set blocking function timeout

      \param[in,out] urg : urg3d session
      \param[in] t_ms  : timeout ms, i.e. t_ms = 1000
    */
    extern void urg3d_high_set_blocking_timeout_ms(urg3d_t* const urg, int t_ms);

    /*!
      \brief initialize the urg3d session (get transform tables)

      \param[in,out] urg : urg3d session

      \retval 1 succeeded
      \retval -1 received error message
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_init(urg3d_t * const urg);

    /*!
      \brief wait to finish initialize of mesurement

      \param[in,out] urg : urg3d session

      \retval 1 succeeded
      \retval -1 received error message
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_wait_finished_initialize(urg3d_t * const urg);

    /*!
      \brief receive version information from device

      \param[in,out] urg : urg3d session

      \retval 1 succeeded
      \retval -1 received error message
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_get_sensor_version(urg3d_t* const urg, urg3d_sensor_version_t* const version);

    /*!
      \brief receive horizontal interlace count from device

      \param[in,out] urg : urg3d session
      \param[out] count : interlace count

      \retval 1 succeeded
      \retval -1 received error message
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_get_horizontal_interlace_count(urg3d_t * const urg, int *count);

    /*!
      \brief receive vertical interlace count from device

      \param[in,out] urg : urg3d session
      \param[out] count : interlace count

      \retval 1 succeeded
      \retval -1 received error message
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_get_vertical_interlace_count(urg3d_t * const urg, int *count);

    /*!
      \brief send horizontal interlace count for device

      \param[in,out] urg : urg3d session
      \param[in] count : interlace count, i.e. count = 4

      \retval 1 succeeded
      \retval -1 received error message or settting count is out of range
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_set_horizontal_interlace_count(urg3d_t * const urg, int count);

    /*!
      \brief send vertical interlace count for device

      \param[in,out] urg : urg3d session
      \param[in] count : interlace count, i.e. count = 4

      \retval 1 succeeded
      \retval -1 received error message or settting count is out of range
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_set_vertical_interlace_count(urg3d_t * const urg, int count);

    /*!
      \brief request to start data

      \param[in,out] urg : urg3d session
      \param[in] meas : urg3d_measurement_type_t sturct, i.e. meas = URG3D_DISTANCE_INTENSITY

      \retval 1 succeeded
      \retval 0 nothing happens
    */
    extern int urg3d_high_start_data(urg3d_t * const urg, urg3d_measurement_type_t meas);

    /*!
      \brief request to stop data

      \param[in,out] urg : urg3d session
      \param[in] meas : urg3d_measurement_type_t sturct, i.e. meas = URG3D_DISTANCE_INTENSITY

      \retval 1 succeeded
      \retval 0 nothing happens
    */
    extern int urg3d_high_stop_data(urg3d_t * const urg, urg3d_measurement_type_t meas);

    /*!
      \brief get "range and intensity" or "range only" user format from revceived data buffer

      \param[in,out] urg : urg3d session
      \param[out] data : urg3d_measurement_data_t struct

      \retval 1 succeeded
      \retval 0 nothing happens
    */
    extern int urg3d_high_get_measurement_data(urg3d_t * const urg, urg3d_measurement_data_t *data);

    /*!
      \brief get "auxiliary" user format from revceived data buffer

      \param[in,out] urg : urg3d session
      \param[out] data : urg3d_auxiliary_data_t struct

      \retval 1 succeeded
      \retval 0 nothing happens
    */
    extern int urg3d_high_get_auxiliary_data(urg3d_t * const urg, urg3d_auxiliary_data_t *data);

    /*!
      \brief restart the device

      \param[in,out] urg : urg3d session

      \retval 1 succeeded
      \retval -1 received error message
      \retval -2 connection timeout
    */
    extern int urg3d_high_blocking_restart(urg3d_t * const urg);

#ifdef __cplusplus
}
#endif

#endif /* !URG3D_SENSOR_H */
