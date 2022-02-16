/** oxgrf_lib.h
 *
 * Author: openxg
 */

#include "oxgrf_api_ss.h"
#include "common_lib.h"

/** @addtogroup _OXGRF_PHY_RF_INTERFACE_
 * @{
 */

/*! \brief OXGRF specific data structure */ 
typedef struct {

  //! opaque OXGRF device struct. An empty ("") or NULL device identifier will result in the first encountered device being opened (using the first discovered backend)
  OXGRF_DESCRIPTOR *dev;
  int16_t *rx_buffer;
  int16_t *tx_buffer;
  //! Sample rate
  unsigned int sample_rate;

  int rx_num_channels;
  int tx_num_channels;

  // --------------------------------
  // Debug and output control
  // --------------------------------
  //! Number of underflows
  int num_underflows;
  //! Number of overflows
  int num_overflows;
  //! number of RX errors
  int num_rx_errors;
  //! Number of TX errors
  int num_tx_errors;

  //! timestamp of current TX
  uint64_t tx_current_ts;
  //! timestamp of current RX
  uint64_t rx_current_ts;
  //! number of TX samples
  uint64_t tx_nsamps;
  //! number of RX samples
  uint64_t rx_nsamps;
  //! number of TX count
  uint64_t tx_count;
  //! number of RX count
  uint64_t rx_count;
  //! timestamp of RX packet
  openair0_timestamp rx_timestamp;

} oxgrf_state_t;

/*! \brief get current timestamp
 *\param device the hardware to use 
 */
openair0_timestamp trx_get_timestamp(openair0_device *device);

#define REMOVE_SUBSTRING_WITHCOMAS(S, TOREMOVE)                                                                        \
  remove_substring(S, TOREMOVE ",");                                                                                   \
  remove_substring(S, TOREMOVE ", ");                                                                                  \
  remove_substring(S, "," TOREMOVE);                                                                                   \
  remove_substring(S, ", " TOREMOVE);                                                                                  \
  remove_substring(S, TOREMOVE)

static inline void remove_substring(char* s, const char* toremove)
{
  while ((s = strstr(s, toremove))) {
    memmove(s, s + strlen(toremove), 1 + strlen(s + strlen(toremove)));
  }
}

static inline void copy_subdev_string(char* dst, char* src)
{
  int n   = 0;
  int len = (int)strlen(src);
  /* Copy until end of string or comma */
  while (n < len && src[n] != '\0' && src[n] != ',') {
    dst[n] = src[n];
    n++;
  }
  dst[n] = '\0';
}
/*@}*/
