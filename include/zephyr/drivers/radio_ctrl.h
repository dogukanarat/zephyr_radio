#ifndef ZEPHYR_INCLUDE_DRIVERS_RADIO_CTRL_H_
#define ZEPHYR_INCLUDE_DRIVERS_RADIO_CTRL_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include "zephyr/drivers/ralf/ralf.h"
#include "zephyr/drivers/ral/ral.h"

#ifdef __cplusplus
extern "C" {
#endif

enum radio_type {
    RADIO_TYPE_SX126X,
    RADIO_TYPE_SX127X,
    RADIO_TYPE_SX128X,
    RADIO_TYPE_LR1110,
    RADIO_TYPE_LAST,
};

struct radio_ctrl_irq_stats {
    volatile uint32_t rx_done;
    volatile uint32_t tx_done;
    volatile uint32_t timeout;
    volatile uint32_t hdr_error;
    volatile uint32_t crc_error;
    volatile uint32_t cad_done;
    volatile uint32_t cad_ok;
    volatile uint32_t none;
};

struct radio_ctrl_stats {
    uint32_t tx_attempts;
    uint32_t tx_success;
    uint32_t tx_timeout;
    uint32_t rx_attempts;
    uint32_t rx_success;
    uint32_t rx_timeout;
    struct radio_ctrl_irq_stats irq;
};

struct radio_ctrl_msg_stats {
    int16_t rssi;        // [dBm]
    int16_t snr;         // [dB]
    int16_t signal_rssi; // [dBm]
};

typedef int (*radio_ctrl_api_set_config_gfsk)(const struct device *dev,
                                              const ralf_params_gfsk_t *rx_params,
                                              const ralf_params_gfsk_t *tx_params);

typedef int (*radio_ctrl_api_set_config_lora)(const struct device *dev,
                                              const ralf_params_lora_t *rx_params,
                                              const ralf_params_lora_t *tx_params,
                                              const ralf_params_lora_cad_t *cad_params);

typedef int (*radio_ctrl_api_set_config_flrc)(const struct device *dev,
                                              const ralf_params_flrc_t *rx_params,
                                              const ralf_params_flrc_t *tx_params);

typedef int (*radio_ctrl_api_set_config_lr_fhss)(const struct device *dev,
                                                 const ralf_params_lr_fhss_t *rx_params,
                                                 const ralf_params_lr_fhss_t *tx_params);

typedef int (*radio_ctrl_api_set_antenna_switch)(const struct device *dev, bool tx_enable);

typedef int (*radio_ctrl_api_get_lora_rx_params)(const struct device *dev,
                                                 ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_set_lora_rx_params)(const struct device *dev,
                                                 const ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_get_lora_tx_params)(const struct device *dev,
                                                 ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_set_lora_tx_params)(const struct device *dev,
                                                 const ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_set_lora_cad_params)(const struct device *dev,
                                                  const ralf_params_lora_cad_t *params);
typedef int (*radio_ctrl_api_get_lora_cad_params)(const struct device *dev,
                                                  ralf_params_lora_cad_t *params);

typedef int (*radio_ctrl_api_listen)(const struct device *dev, uint32_t timeout);
typedef int (*radio_ctrl_api_cad)(const struct device *dev, uint32_t timeout);
typedef int (*radio_ctrl_api_transmit)(const struct device *dev, const uint8_t *data, size_t size);
typedef int (*radio_ctrl_api_receive)(const struct device *dev, uint8_t *data, size_t size,
                                      struct radio_ctrl_msg_stats *stats, uint32_t timeout);

typedef int (*radio_ctrl_api_flush_rx_queue)(const struct device *dev);

typedef int (*radio_ctrl_api_get_stats)(const struct device *dev,
                                      struct radio_ctrl_stats *stats);

__subsystem struct radio_ctrl_driver_api {
    radio_ctrl_api_set_config_gfsk set_config_gfsk;
    radio_ctrl_api_set_config_lora set_config_lora;
    radio_ctrl_api_set_config_flrc set_config_flrc;
    radio_ctrl_api_set_config_lr_fhss set_config_lr_fhss;
    radio_ctrl_api_set_antenna_switch set_antenna_switch;
    radio_ctrl_api_get_lora_rx_params get_lora_rx_params;
    radio_ctrl_api_set_lora_rx_params set_lora_rx_params;
    radio_ctrl_api_get_lora_tx_params get_lora_tx_params;
    radio_ctrl_api_set_lora_tx_params set_lora_tx_params;
    radio_ctrl_api_get_lora_cad_params get_lora_cad_params;
    radio_ctrl_api_set_lora_cad_params set_lora_cad_params;
    radio_ctrl_api_listen listen;
    radio_ctrl_api_cad cad;
    radio_ctrl_api_transmit transmit;
    radio_ctrl_api_receive receive;
    radio_ctrl_api_flush_rx_queue flush_rx_queue;
    radio_ctrl_api_get_stats get_stats;
};

static inline int radio_ctrl_set_config_gfsk(const struct device *dev,
                                             const ralf_params_gfsk_t *rx_params,
                                             const ralf_params_gfsk_t *tx_params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_config_gfsk(dev, rx_params, tx_params);
}

static inline int radio_ctrl_set_config_lora(const struct device *dev,
                                             const ralf_params_lora_t *rx_params,
                                             const ralf_params_lora_t *tx_params,
                                             const ralf_params_lora_cad_t *cad_params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_config_lora(dev, rx_params, tx_params, cad_params);
}

static inline int radio_ctrl_set_config_flrc(const struct device *dev,
                                             const ralf_params_flrc_t *rx_params,
                                             const ralf_params_flrc_t *tx_params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_config_flrc(dev, rx_params, tx_params);
}

static inline int radio_ctrl_set_config_lr_fhss(const struct device *dev,
                                                const ralf_params_lr_fhss_t *rx_params,
                                                const ralf_params_lr_fhss_t *tx_params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_config_lr_fhss(dev, rx_params, tx_params);
}

static inline int radio_ctrl_set_antenna_switch(const struct device *dev, bool tx_enable)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_antenna_switch(dev, tx_enable);
}

static inline int radio_ctrl_get_lora_rx_params(const struct device *dev,
                                                ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->get_lora_rx_params(dev, params);
}

static inline int radio_ctrl_set_lora_rx_params(const struct device *dev,
                                                const ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_lora_rx_params(dev, params);
}

static inline int radio_ctrl_get_lora_tx_params(const struct device *dev,
                                                ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->get_lora_tx_params(dev, params);
}

static inline int radio_ctrl_set_lora_tx_params(const struct device *dev,
                                                const ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_lora_tx_params(dev, params);
}

static inline int radio_ctrl_get_lora_cad_params(const struct device *dev,
                                                 ralf_params_lora_cad_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->get_lora_cad_params(dev, params);
}

static inline int radio_ctrl_set_lora_cad_params(const struct device *dev,
                                                 const ralf_params_lora_cad_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_lora_cad_params(dev, params);
}

static inline int radio_ctrl_listen(const struct device *dev, uint32_t timeout)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->listen(dev, timeout);
}

static inline int radio_ctrl_cad(const struct device *dev, uint32_t timeout)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->cad(dev, timeout);
}

static inline int radio_ctrl_transmit(const struct device *dev, const uint8_t *data, size_t size)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->transmit(dev, data, size);
}

static inline int radio_ctrl_receive(const struct device *dev, uint8_t *data, size_t size,
                                     struct radio_ctrl_msg_stats *stats, uint32_t timeout)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->receive(dev, data, size, stats, timeout);
}

static inline int radio_ctrl_flush_rx_queue(const struct device *dev)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->flush_rx_queue(dev);
}

static inline int radio_ctrl_get_stats(const struct device *dev,
                                      struct radio_ctrl_stats *stats)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->get_stats(dev, stats);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RADIO_CTRL_H_ */
