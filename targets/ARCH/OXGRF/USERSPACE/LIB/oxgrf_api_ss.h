/*
 * oxgrf_api_ss.h
 *
 *  Created on: 2016/5/9
 *      Author: Eric
 */
#ifndef __OXGRF_SS_API_H__
#define __OXGRF_SS_API_H__
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#if defined(__WINDOWS_) || defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#define OXGRFCALL __cdecl
#else
#define DLLEXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct oxgrf_transport OXGRF_TRANSPORT;


typedef struct oxgrf_device_descriptor {
    int32_t id;
    int32_t status;
    uint8_t nchips;
    uint8_t nsubdev;
    OXGRF_TRANSPORT *trans;
}OXGRF_DESCRIPTOR;

typedef enum {
    TX1_CHANNEL = 1,
    TX2_CHANNEL = 2,
    TX3_CHANNEL = 3,
    TX4_CHANNEL = 4,
    TX5_CHANNEL = 5,
    TX6_CHANNEL = 6,
    TX7_CHANNEL = 7,
    TX8_CHANNEL = 8,
}RF_TX_CHANNEL;

typedef enum {
    RX1_CHANNEL = 1,
    RX2_CHANNEL = 2,
    RX3_CHANNEL = 3,
    RX4_CHANNEL = 4,
    RX5_CHANNEL = 5,
    RX6_CHANNEL = 6,
    RX7_CHANNEL = 7,
    RX8_CHANNEL = 8,
}RF_RX_CHANNEL;

typedef enum rf_gain_ctrl_mode {
    RF_GAIN_MGC,
    RF_GAIN_FASTATTACK_AGC,
    RF_GAIN_SLOWATTACK_AGC,
}RF_GAIN_CTRL_MODE;

typedef enum ref_select {
    INTERNAL_REFERENCE = 0,
    EXTERNAL_REFERENCE,
}REF_SELECT;

typedef enum vco_cal_select {
    ADF4001 = 0,
    AUXDAC1,
}VCO_CAL_SELECT;

typedef enum duplex_select {
    TDD = 0,
    FDD,
}DUPLEX_SELECT;

typedef enum trx_switch {
    RX = 0,
    TX,
}TRX_SWITCH;

typedef enum {
    TX_CHANNEL_TIMEOUT   = 29,
    RX_CHANNEL_TIMEOUT   = 30,
    TX_CHANNEL_UNDERFLOW = 31,
    RX_CHANNEL_OVERFLOW  = 32,
    TX_CHANNEL_COUNT     = 33,
    RX_CHANNEL_COUNT     = 34,
}CHANNEL_EVENT;

typedef enum pps_enable {
    PPS_INTERNAL_EN,
    PPS_GPS_EN,
    PPS_EXTERNAL_EN,
}PPSModeEnum;

typedef enum device_configuration {
    SingleSubDevSingleRF = 0x1010,
    SingleSubDevDualRF   = 0x1020,
    SingleSubDevQuadRF   = 0x1040,
    DualSubDevDualRF     = 0x2011,
    DualSubDevTriple1RF  = 0x2012,
    DualSubDevTriple2RF  = 0x2021,
    DualSubDevQuadRF     = 0x2022,
    DualSubDevOctoRF     = 0x2044,
    NULLSubDevNULLRF     = 0,
}DEV_CFG;

typedef struct {
    uint8_t   receiver_mode;
    uint8_t   disciplining_mode;
    uint16_t  minor_alarms;
    uint8_t   gnss_decoding_status;
    uint8_t   disciplining_activity;
    uint8_t   pps_indication;
    uint8_t   pps_reference;
} GPS_STATUS;

typedef struct {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint32_t year;
} TIME_24H;

/*********************************************************************************************************************************************************/

DLLEXPORT uint64_t oxgrf_ticksToTimeNs (const uint64_t ticks, const double rate);
DLLEXPORT uint64_t oxgrf_timeNsToTicks (const uint64_t timeNs, const double rate);

/*********************************************************************************************************************************************************/
DLLEXPORT OXGRF_DESCRIPTOR *oxgrf_open_device (const char *url);
DLLEXPORT int32_t oxgrf_close_device (OXGRF_DESCRIPTOR *oxgrf);
DLLEXPORT int32_t oxgrf_get_device_configuration (OXGRF_DESCRIPTOR* oxgrf, DEV_CFG* cfg);

DLLEXPORT int32_t oxgrf_get_firmware_version(OXGRF_DESCRIPTOR* oxgrf, uint32_t* version);
DLLEXPORT int32_t oxgrf_get_model_version(OXGRF_DESCRIPTOR* oxgrf, uint32_t* version);
/*********************************************************************************************************************************************************/

DLLEXPORT int32_t oxgrf_get_gps_status (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, GPS_STATUS *g_status);
DLLEXPORT int32_t oxgrf_get_utc (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, TIME_24H *time);
DLLEXPORT int32_t oxgrf_get_xyz (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, double *longtitude, double *latitude, double *altitude);
DLLEXPORT int32_t oxgrf_get_sampling_freq_range (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint32_t *sampling_freq_hz_max, uint32_t *sampling_freq_hz_min);
DLLEXPORT int32_t oxgrf_get_rx_gain_range (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint32_t *gain_db_max, uint32_t *gain_db_min);
DLLEXPORT int32_t oxgrf_get_tx_gain_range (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint32_t *gain_db_max, uint32_t *gain_db_min);
DLLEXPORT int32_t oxgrf_get_rx_freq_range (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint64_t *lo_freq_hz_max, uint64_t *lo_freq_hz_min);
DLLEXPORT int32_t oxgrf_get_tx_freq_range (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint64_t *lo_freq_hz_max, uint64_t *lo_freq_hz_min);

/*********************************************************************************************************************************************************/

/* Get current TX LO frequency. */
DLLEXPORT int32_t oxgrf_get_tx_lo_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint64_t *lo_freq_hz);
/* Get current TX sampling frequency. */
DLLEXPORT int32_t oxgrf_get_tx_sampling_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t *sampling_freq_hz);
/* Get the TX RF bandwidth. */
DLLEXPORT int32_t oxgrf_get_tx_rf_bandwidth (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t *bandwidth_hz);
/* Get current transmit attenuation for the selected channel. */
DLLEXPORT int32_t oxgrf_get_tx1_attenuation (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t *attenuation_mdb);
/* Get current transmit attenuation for the selected channel. */
DLLEXPORT int32_t oxgrf_get_tx2_attenuation (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t *attenuation_mdb);        

/* Get current RX LO frequency. */
DLLEXPORT int32_t oxgrf_get_rx_lo_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint64_t *lo_freq_hz);
/* Get the RX RF bandwidth. */
DLLEXPORT int32_t oxgrf_get_rx_rf_bandwidth (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t *bandwidth_hz);
/* Get the gain control mode for the selected channel. */
DLLEXPORT int32_t oxgrf_get_rx1_gain_control_mode (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, RF_GAIN_CTRL_MODE *gc_mode);
/* Get the gain control mode for the selected channel. */
DLLEXPORT int32_t oxgrf_get_rx2_gain_control_mode (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, RF_GAIN_CTRL_MODE *gc_mode);
/* Get current receive RF gain for the selected channel. */
DLLEXPORT int32_t oxgrf_get_rx1_rf_gain (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, int32_t *gain_db);
/* Get current receive RF gain for the selected channel. */
DLLEXPORT int32_t oxgrf_get_rx2_rf_gain (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, int32_t *gain_db);
DLLEXPORT int32_t oxgrf_get_rx1_rssi (OXGRF_DESCRIPTOR* oxgrf, uint8_t rf_id, int32_t* rssi);
DLLEXPORT int32_t oxgrf_get_rx2_rssi (OXGRF_DESCRIPTOR* oxgrf, uint8_t rf_id, int32_t* rssi);

/*********************************************************************************************************************************************************/
/* Set the RX LO frequency. */
DLLEXPORT int32_t oxgrf_set_rx_lo_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint64_t lo_freq_hz);
/* Set the RX RF bandwidth. */
DLLEXPORT int32_t oxgrf_set_rx_rf_bandwidth (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t  bandwidth_hz);
/* Set the RX sampling frequency. */
DLLEXPORT int32_t oxgrf_set_rx_sampling_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t sampling_freq_hz);
/* Set the gain control mode for the selected channel. */
DLLEXPORT int32_t oxgrf_set_rx1_gain_control_mode (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, RF_GAIN_CTRL_MODE gc_mode);
DLLEXPORT int32_t oxgrf_set_rx2_gain_control_mode (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, RF_GAIN_CTRL_MODE gc_mode);
/* Set the receive RF gain for the selected channel. */
DLLEXPORT int32_t oxgrf_set_rx1_rf_gain (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, int32_t gain_db);
/* Set the receive RF gain for the selected channel. */
DLLEXPORT int32_t oxgrf_set_rx2_rf_gain (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, int32_t gain_db);

DLLEXPORT int32_t oxgrf_set_rx_fir_en_dis (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint8_t enable);

/* Set the TX LO frequency. */
DLLEXPORT int32_t oxgrf_set_tx_lo_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint64_t lo_freq_hz); 
/* Set the TX RF bandwidth. */
DLLEXPORT int32_t oxgrf_set_tx_rf_bandwidth (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t  bandwidth_hz);
/* Set the TX sampling frequency. */
DLLEXPORT int32_t oxgrf_set_tx_sampling_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t sampling_freq_hz);
/* Set the transmit attenuation for the selected channel. */
DLLEXPORT int32_t oxgrf_set_tx1_attenuation (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t attenuation_mdb);
/* Set the transmit attenuation for the selected channel. */
DLLEXPORT int32_t oxgrf_set_tx2_attenuation (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t attenuation_mdb);

/* Set the transmit abs power for the selected channel. */
DLLEXPORT int32_t oxgrf_set_tx1_abs_power (OXGRF_DESCRIPTOR* oxgrf, uint8_t rf_id, int32_t abs_power);
/* Set the transmit abs power for the selected channel. */
DLLEXPORT int32_t oxgrf_set_tx2_abs_power (OXGRF_DESCRIPTOR* oxgrf, uint8_t rf_id, int32_t abs_power);

DLLEXPORT int32_t oxgrf_set_tx_fir_en_dis (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint8_t status);

/*********************************************************************************************************************************************************/

DLLEXPORT int32_t oxgrf_get_rfchip_reg (OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t reg, uint32_t *value);
DLLEXPORT int32_t oxgrf_set_rfchip_reg(OXGRF_DESCRIPTOR *oxgrf, uint8_t rf_id, uint32_t reg, uint32_t value);

/*********************************************************************************************************************************************************/

DLLEXPORT int32_t oxgrf_get_extend_cmd(OXGRF_DESCRIPTOR *oxgrf, uint8_t cmd_id, uint64_t *value, bool with_param);
DLLEXPORT int32_t oxgrf_set_extend_cmd(OXGRF_DESCRIPTOR *oxgrf, uint8_t cmd_id, uint64_t value);

/*********************************************************************************************************************************************************/

DLLEXPORT int32_t oxgrf_set_tx_lo_int_ext (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint8_t enable);
DLLEXPORT int32_t oxgrf_set_rx_lo_int_ext (OXGRF_DESCRIPTOR *oxgrf,uint8_t dev_id, uint8_t enable);
DLLEXPORT int32_t oxgrf_set_ext_lo_freq (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint64_t lo_freq_hz);
DLLEXPORT int32_t oxgrf_do_mcs (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint8_t enable);

DLLEXPORT int32_t oxgrf_set_ref_clock (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, REF_SELECT source);
DLLEXPORT int32_t oxgrf_set_vco_select (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, VCO_CAL_SELECT vco);
DLLEXPORT int32_t oxgrf_set_auxdac1 (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint32_t vol_mV);
DLLEXPORT int32_t oxgrf_set_duplex_select (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, DUPLEX_SELECT duplex);
DLLEXPORT int32_t oxgrf_set_rx_ant_enable (OXGRF_DESCRIPTOR* oxgrf, uint8_t dev_id, uint8_t enable);
DLLEXPORT int32_t oxgrf_tx_cyclic_enable (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint8_t enable);
DLLEXPORT int32_t oxgrf_set_trxsw_fpga_enable (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint8_t enable);
DLLEXPORT int32_t oxgrf_set_hwbuf_depth (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint32_t depth);
DLLEXPORT int32_t oxgrf_get_hwbuf_depth (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint32_t *depth);

DLLEXPORT int32_t oxgrf_set_pps_select (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, PPSModeEnum pps);
DLLEXPORT int32_t oxgrf_set_rxchannel_coef (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, RF_RX_CHANNEL channel, int16_t coef1, int16_t coef2);
DLLEXPORT int32_t oxgrf_enable_rxchannel_corr (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, RF_RX_CHANNEL channel, uint32_t enable);
DLLEXPORT int32_t oxgrf_set_txchannel_coef (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, RF_TX_CHANNEL channel, int16_t coef1, int16_t coef2);
DLLEXPORT int32_t oxgrf_enable_txchannel_corr (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, RF_TX_CHANNEL channel, uint32_t enable);
DLLEXPORT int32_t oxgrf_set_txchannel_ampcoef (OXGRF_DESCRIPTOR* oxgrf, uint8_t dev_id, RF_TX_CHANNEL channel, int16_t coef);
DLLEXPORT int32_t oxgrf_set_rxchannel_ampcoef (OXGRF_DESCRIPTOR* oxgrf, uint8_t dev_id, RF_RX_CHANNEL channel, int16_t coef);

/*********************************************************************************************************************************************************/

DLLEXPORT int32_t oxgrf_enable_timestamp (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint8_t enable);
DLLEXPORT int32_t oxgrf_read_timestamp (OXGRF_DESCRIPTOR *oxgrf, uint8_t dev_id, uint64_t *timestamp);
DLLEXPORT int32_t oxgrf_get_channel_event (OXGRF_DESCRIPTOR *oxgrf, CHANNEL_EVENT event, uint8_t channel, uint32_t *count);

/*********************************************************************************************************************************************************/

DLLEXPORT int32_t oxgrf_read_samples (OXGRF_DESCRIPTOR *oxgrf, void *buffer, uint32_t count, RF_RX_CHANNEL channel, uint64_t *timestamp);
DLLEXPORT int32_t oxgrf_read_samples_multiport (OXGRF_DESCRIPTOR *oxgrf, void **buffer, uint32_t count, uint8_t channel_mask, uint64_t *timestamp);
DLLEXPORT int32_t oxgrf_read_samples_multiport_Matlab (OXGRF_DESCRIPTOR *oxgrf, void *buffer, uint32_t count, uint8_t channel_mask, uint64_t *timestamp);
DLLEXPORT int32_t oxgrf_dump_samples_multiport (OXGRF_DESCRIPTOR *oxgrf, uint32_t count, uint8_t channel_mask, uint64_t* timestamp);
/* Since the data contains a 16-byte header, the size of the buffer should be greater than or equal to (count*4+16) Bytes*/
DLLEXPORT int32_t oxgrf_read_samples_zerocopy (OXGRF_DESCRIPTOR *oxgrf, void *buffer, uint32_t count, RF_RX_CHANNEL channel, uint64_t *timestamp);

DLLEXPORT int32_t oxgrf_write_samples (OXGRF_DESCRIPTOR *oxgrf, void *buffer, uint32_t count, RF_TX_CHANNEL channel, uint64_t timestamp);
DLLEXPORT int32_t oxgrf_write_samples2 (OXGRF_DESCRIPTOR *oxgrf, void *buffer, uint32_t count, RF_TX_CHANNEL channel, uint64_t timestamp, uint32_t flags);
DLLEXPORT int32_t oxgrf_write_samples_multiport (OXGRF_DESCRIPTOR *oxgrf, const void **buffer, uint32_t count, uint8_t channel_mask, uint64_t timestamp, uint32_t flags);
DLLEXPORT int32_t oxgrf_write_samples_multiport_Matlab (OXGRF_DESCRIPTOR *oxgrf, const void *buffer, uint32_t count, uint8_t channel_mask, uint64_t timestamp, uint32_t flags);
DLLEXPORT int32_t oxgrf_write_samples_zerocopy (OXGRF_DESCRIPTOR *oxgrf, void *buffer, uint32_t count, RF_TX_CHANNEL channel, uint64_t timestamp);

/*********************************************************************************************************************************************************/

DLLEXPORT void float_to_int16 (int16_t *dst, const float *src, int n, float mult);
DLLEXPORT void int16_to_float (float *dst, const int16_t *src, int len, float mult);
DLLEXPORT void int16_to_float_avx2(float *outputVector, const int16_t *inputVector, unsigned int num_points);
DLLEXPORT void float_to_int16_avx2(int16_t *outputVector, const float *inputVector, unsigned int num_points);

/*********************************************************************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /*  __OXGRF_SS_API_H__ */
