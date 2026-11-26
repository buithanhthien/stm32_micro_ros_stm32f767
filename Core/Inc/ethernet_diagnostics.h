#ifndef ETHERNET_DIAGNOSTICS_H
#define ETHERNET_DIAGNOSTICS_H

#include <stdint.h>

/* Read-only debugger telemetry. Counters may wrap; snapshots are not atomic. */
typedef struct
{
  uint32_t hal_init_status; /* HAL status; UINT32_MAX before first init */
  uint32_t hal_start_status; /* HAL status; UINT32_MAX before first start */
  uint32_t phy_init_status; /* signed LAN8742 status represented as uint32_t */
  int32_t phy_link_state;
  uint32_t rx_irq_count;
  uint32_t tx_irq_count;
  uint32_t dma_error_count;
  uint32_t last_dma_error;
  uint32_t last_hal_error;
  uint32_t rx_alloc_fail_count;
  uint32_t rx_frames;
  uint32_t rx_input_errors;
  uint32_t tx_requests;
  uint32_t tx_submit_errors;
  uint32_t tx_last_status;
  uint32_t netif_flags;
  uint32_t ipv4_address; /* network byte order */
  uint32_t dma_status;
  uint32_t rx_desc_ready;
  uint32_t tx_free_count;
  uint32_t tx_timeout_count;
} EthernetDiagnostics;
extern volatile EthernetDiagnostics g_eth_diag;

#endif
