#define DT_DRV_COMPAT da_sx126x_radio

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/radio_ctrl.h>
#include "zephyr/drivers/sx126x/sx126x_hal.h"
#include "zephyr/drivers/ralf/ralf_sx126x.h"

LOG_MODULE_REGISTER(radio_ctrl, CONFIG_RADIO_CTRL_LOG_LEVEL);

#define RADIO_CTRL_EVENT_TX_DONE           BIT(0)
#define RADIO_CTRL_EVENT_RX_DONE           BIT(1)
#define RADIO_CTRL_EVENT_RX_TIMEOUT        BIT(2)
#define RADIO_CTRL_EVENT_RX_HDR_ERROR      BIT(3)
#define RADIO_CTRL_EVENT_RX_CRC_ERROR      BIT(4)
#define RADIO_CTRL_EVENT_CAD_DONE          BIT(5)
#define RADIO_CTRL_EVENT_CAD_OK            BIT(6)

/* Maximum RX timeout value supported by the SX126x driver (2^18 ms) */
#define RADIO_CTRL_RX_TIMEOUT_MAX_MS       262144

/* Maximum SPI transfer size for stack-allocated buffers */
#define RADIO_CTRL_HAL_MAX_TRANSFER_SIZE   256

struct radio_ctrl_msg {
    uint16_t size;
    uint8_t data[CONFIG_RADIO_CTRL_MAX_MSG_SIZE];
    struct radio_ctrl_msg_stats stats;
};

struct radio_ctrl_config {
    const struct spi_dt_spec spi;
    struct gpio_dt_spec nreset;
    struct gpio_dt_spec busy;
    struct gpio_dt_spec irq;
    struct gpio_dt_spec ant_sw;
};

struct radio_ctrl_data {
    struct k_work irq_work;
    struct k_mutex mutex;
    struct gpio_callback irq_cb_data;
    const struct device *dev;
    bool is_cad_enabled;
    ralf_t radio;
    ralf_params_lora_t rx_params;
    ralf_params_lora_t tx_params;
    ralf_params_lora_cad_t cad_params;
    struct radio_ctrl_stats stats;
    sx126x_hal_context_t sx126x_hal_ctx;
    struct k_event event;
    struct radio_ctrl_msg msg_buffer[CONFIG_RADIO_CTRL_RX_MSGQ_MAX_MSGS];
    struct k_msgq rx_msgq;
    bool is_configured;
    bool is_ralf_initialized;
    bool is_initialized;
};

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command,
                                     const uint16_t command_length, const uint8_t *data,
                                     const uint16_t data_length)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct spi_dt_spec *spi = (const struct spi_dt_spec *)ctx->spi;
    const struct gpio_dt_spec *busy = (const struct gpio_dt_spec *)ctx->busy_pin;
    struct spi_buf tx_bufs[2];
    struct spi_buf_set tx;
    int busy_state;
    int32_t busy_start = k_uptime_get_32();

    do {
        busy_state = gpio_pin_get_dt(busy);
        if (busy_state == 0) {
            break;
        }
        k_msleep(1);
    } while ((k_uptime_get_32() - busy_start) < ctx->timeout);
    if ((k_uptime_get_32() - busy_start) >= ctx->timeout) {
        return SX126X_HAL_STATUS_ERROR;
    }

    tx_bufs[0].buf = (void *)command;
    tx_bufs[0].len = command_length;
    tx_bufs[1].buf = (void *)data;
    tx_bufs[1].len = data_length;
    tx.buffers = tx_bufs;
    tx.count = (data_length > 0) ? 2 : 1;

    int ret = spi_write_dt(spi, &tx);
    return (ret == 0) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length, uint8_t *data,
                                    const uint16_t data_length)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct spi_dt_spec *spi = (const struct spi_dt_spec *)ctx->spi;
    const struct gpio_dt_spec *busy = (const struct gpio_dt_spec *)ctx->busy_pin;
    uint8_t dummy_tx[RADIO_CTRL_HAL_MAX_TRANSFER_SIZE];
    uint8_t dummy_rx[RADIO_CTRL_HAL_MAX_TRANSFER_SIZE];
    struct spi_buf tx_bufs[2];
    struct spi_buf rx_bufs[2];
    struct spi_buf_set tx, rx;
    int busy_state;
    int32_t busy_start = k_uptime_get_32();

    if (data_length > RADIO_CTRL_HAL_MAX_TRANSFER_SIZE ||
        command_length > RADIO_CTRL_HAL_MAX_TRANSFER_SIZE) {
        LOG_ERR("HAL transfer size exceeds maximum: cmd=%u, data=%u, max=%u",
                command_length, data_length, RADIO_CTRL_HAL_MAX_TRANSFER_SIZE);
        return SX126X_HAL_STATUS_ERROR;
    }

    memset(dummy_tx, 0x00, data_length);

    do {
        busy_state = gpio_pin_get_dt(busy);
        if (busy_state == 0) {
            break;
        }
        k_msleep(1);
    } while ((k_uptime_get_32() - busy_start) < ctx->timeout);
    if ((k_uptime_get_32() - busy_start) >= ctx->timeout) {
        return SX126X_HAL_STATUS_ERROR;
    }

    /* TX: send command, then dummy bytes */
    tx_bufs[0].buf = (void *)command;
    tx_bufs[0].len = command_length;
    tx_bufs[1].buf = dummy_tx;
    tx_bufs[1].len = data_length;
    tx.buffers = tx_bufs;
    tx.count = 2;

    /* RX: ignore command response, then read data */
    rx_bufs[0].buf = dummy_rx;
    rx_bufs[0].len = command_length;
    rx_bufs[1].buf = data;
    rx_bufs[1].len = data_length;
    rx.buffers = rx_bufs;
    rx.count = 2;

    int ret = spi_transceive_dt(spi, &tx, &rx);
    return (ret == 0) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct gpio_dt_spec *nreset = (const struct gpio_dt_spec *)ctx->nreset_pin;
    int ret;
    ret = gpio_pin_set_dt(nreset, 0);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    ret = gpio_pin_set_dt(nreset, 1);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct gpio_dt_spec *nreset = (const struct gpio_dt_spec *)ctx->nreset_pin;
    int ret;
    ret = gpio_pin_set_dt(nreset, 0);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    ret = gpio_pin_set_dt(nreset, 1);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    return SX126X_HAL_STATUS_OK;
}

static int radio_ctrl_impl_set_type(const struct device *dev, enum radio_type type)
{
    struct radio_ctrl_data *data = dev->data;
    const struct radio_ctrl_config *cfg = dev->config;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    switch (type) {
    case RADIO_TYPE_SX126X: {
        data->sx126x_hal_ctx.spi = (void *)&cfg->spi;
        data->sx126x_hal_ctx.busy_pin = &cfg->busy;
        data->sx126x_hal_ctx.nreset_pin = &cfg->nreset;
        data->sx126x_hal_ctx.timeout = CONFIG_RADIO_CTRL_TIMEOUT_MS;

        ralf_t radio = RALF_SX126X_INSTANTIATE(&data->sx126x_hal_ctx);
        data->radio = radio;
        data->is_ralf_initialized = true;
        break;
    }
    case RADIO_TYPE_SX127X: {
        ret = -ENOTSUP;
        break;
    }
    case RADIO_TYPE_SX128X: {
        ret = -ENOTSUP;
        break;
    }
    case RADIO_TYPE_LR1110: {
        ret = -ENOTSUP;
        break;
    }
    default:
        ret = -EINVAL;
        break;
    }

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_set_config_gfsk(const struct device *dev,
                                           const ralf_params_gfsk_t *rx_params,
                                           const ralf_params_gfsk_t *tx_params)
{
    struct radio_ctrl_data *data = dev->data;

    if ((rx_params == NULL) || (tx_params == NULL)) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    /* TODO: implement GFSK config storage in data structure */
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_set_config_lora(const struct device *dev,
                                           const ralf_params_lora_t *rx_params,
                                           const ralf_params_lora_t *tx_params,
                                           const ralf_params_lora_cad_t *cad_params)
{
    struct radio_ctrl_data *data = dev->data;

    if ((rx_params == NULL) || (tx_params == NULL) || (cad_params == NULL)) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->rx_params = *rx_params;
    data->tx_params = *tx_params;
    data->cad_params = *cad_params;
    data->is_configured = true;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_set_config_flrc(const struct device *dev,
                                           const ralf_params_flrc_t *rx_params,
                                           const ralf_params_flrc_t *tx_params)
{
    struct radio_ctrl_data *data = dev->data;

    if ((rx_params == NULL) || (tx_params == NULL)) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    /* TODO: implement FLRC config storage in data structure */
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_set_config_lr_fhss(const struct device *dev,
                                              const ralf_params_lr_fhss_t *rx_params,
                                              const ralf_params_lr_fhss_t *tx_params)
{
    struct radio_ctrl_data *data = dev->data;

    if ((rx_params == NULL) || (tx_params == NULL)) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    /* TODO: implement LR-FHSS config storage in data structure */
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_set_antenna_switch(const struct device *dev, bool tx_enable)
{
    const struct radio_ctrl_config *cfg = dev->config;

    if (!cfg->ant_sw.port) {
        return -ENOTSUP;
    }

    return gpio_pin_set_dt(&cfg->ant_sw, tx_enable ? 1 : 0);
}

static int radio_ctrl_impl_get_lora_rx_params(const struct device *dev, ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = (struct radio_ctrl_data *)dev->data;

    if (params == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    *params = data->rx_params;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_set_lora_rx_params(const struct device *dev,
                                              const ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = dev->data;

    if (params == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->rx_params = *params;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_get_lora_tx_params(const struct device *dev, ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = dev->data;

    if (params == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    *params = data->tx_params;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_set_lora_tx_params(const struct device *dev,
                                              const ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = dev->data;

    if (params == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->tx_params = *params;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_get_lora_cad_params(const struct device *dev,
                                               ralf_params_lora_cad_t *params)
{
    struct radio_ctrl_data *data = dev->data;

    if (params == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    *params = data->cad_params;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_set_lora_cad_params(const struct device *dev,
                                               const ralf_params_lora_cad_t *params)
{
    struct radio_ctrl_data *data = dev->data;

    if (params == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->cad_params = *params;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int radio_ctrl_impl_listen(const struct device *dev, uint32_t timeout)
{
    struct radio_ctrl_data *ctrl_data = dev->data;
    int ret = 0;
    ral_status_t ral_status;
    ralf_params_lora_t lora_params = {0};

    k_mutex_lock(&ctrl_data->mutex, K_FOREVER);

    do {
        if (!ctrl_data->is_ralf_initialized) {
            ret = -ENODEV;
            LOG_ERR("Radio is not initialized");
            break;
        }

        if (!ctrl_data->is_configured) {
            ret = -EACCES;
            LOG_ERR("Radio is not configured");
            break;
        }

        memcpy(&lora_params, &ctrl_data->rx_params, sizeof(ralf_params_lora_t));
        lora_params.pkt_params.pld_len_in_bytes = CONFIG_RADIO_CTRL_MAX_MSG_SIZE;

        /* Set the radio to standby */
        ral_status = ral_set_standby(ral_from_ralf(&ctrl_data->radio), RAL_STANDBY_CFG_RC);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the radio to standby! Status: %08X", ral_status);
            break;
        }

        /* Set the LoRa modem parameters */
        ral_status = ralf_setup_lora(&ctrl_data->radio, &lora_params);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to setup the LoRa modem! Status: %08X", ral_status);
            break;
        }

        /* Set the DIO IRQ parameters */
        ral_status = ral_set_dio_irq_params(ral_from_ralf(&ctrl_data->radio),
                                            RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_RX_HDR_ERROR);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the DIO IRQ parameters! Status: %08X", ral_status);
            break;
        }

        if (timeout >= RADIO_CTRL_RX_TIMEOUT_MAX_MS) {
            timeout = RAL_RX_TIMEOUT_CONTINUOUS_MODE;
        }

        /* Set the radio to receive */
        ral_status = ral_set_rx(ral_from_ralf(&ctrl_data->radio), timeout);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the radio to receive! Status: %08X", ral_status);
            break;
        }

        LOG_DBG("Listening...");

    } while (0);

    k_mutex_unlock(&ctrl_data->mutex);

    return ret;
}

static int radio_ctrl_impl_cad(const struct device *dev, uint32_t timeout)
{
    struct radio_ctrl_data *data = dev->data;

    ARG_UNUSED(timeout);

    k_mutex_lock(&data->mutex, K_FOREVER);
    /* TODO: implement CAD logic here using data->radio and data->cad_params */
    k_mutex_unlock(&data->mutex);

    return -ENOTSUP;
}

static int radio_ctrl_impl_transmit(const struct device *dev, const uint8_t *data, size_t size)
{
    struct radio_ctrl_data *ctrl_data = dev->data;
    int ret = 0;
    ral_status_t ral_status;
    uint32_t timeout;
    bool switch_to_standby = false;
    ralf_params_lora_t lora_params = {0};

    k_mutex_lock(&ctrl_data->mutex, K_FOREVER);

    do {
        if (size > CONFIG_RADIO_CTRL_MAX_MSG_SIZE) {
            ret = -EINVAL;
            LOG_ERR("Data size exceeds maximum limit: %d > %d", size, CONFIG_RADIO_CTRL_MAX_MSG_SIZE);
            break;
        }

        ctrl_data->stats.tx_attempts++;

        if (!ctrl_data->is_ralf_initialized) {
            ret = -ENODEV;
            LOG_ERR("Radio is not initialized");
            break;
        }

        if (!ctrl_data->is_configured) {
            ret = -EACCES;
            LOG_ERR("Radio is not configured");
            break;
        }

        timeout = ral_get_lora_time_on_air_in_ms(ral_from_ralf(&ctrl_data->radio),
                                                 &ctrl_data->tx_params.pkt_params,
                                                 &ctrl_data->tx_params.mod_params);
        timeout += 100; /* Add 100 ms for processing delay */

        memcpy(&lora_params, &ctrl_data->tx_params, sizeof(ralf_params_lora_t));
        lora_params.pkt_params.pld_len_in_bytes = size;

        /* Set the radio to standby */
        ral_status = ral_set_standby(ral_from_ralf(&ctrl_data->radio), RAL_STANDBY_CFG_RC);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the radio to standby! Status: %08X", ral_status);
            break;
        }

        /* Set the LoRa modem parameters */
        ral_status = ralf_setup_lora(&ctrl_data->radio, &lora_params);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to setup the LoRa modem! Status: %08X", ral_status);
            break;
        }

        /* Set the DIO IRQ parameters */
        ral_status = ral_set_dio_irq_params(ral_from_ralf(&ctrl_data->radio), RAL_IRQ_TX_DONE);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the DIO IRQ parameters! Status: %08X", ral_status);
            break;
        }

        /* Set the packet payload */
        ral_status = ral_set_pkt_payload(ral_from_ralf(&ctrl_data->radio), data, size);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the packet payload! Status: %08X", ral_status);
            break;
        }

        switch_to_standby = true;
        /* Set the radio to transmit */
        ral_status = ral_set_tx(ral_from_ralf(&ctrl_data->radio));
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the radio to transmit! Status: %08X", ral_status);
            break;
        }

        LOG_DBG("Transmitting... data size: %d, timeout: %d ms", size, timeout);

        uint32_t events = k_event_wait(&ctrl_data->event, RADIO_CTRL_EVENT_TX_DONE, true,
                                        K_MSEC(timeout));
        if ((events & RADIO_CTRL_EVENT_TX_DONE) == 0) {
            ret = -ETIMEDOUT;
            ctrl_data->stats.tx_timeout++;
            LOG_ERR("Transmit timeout");
            break;
        }

        ret = 0;
        ctrl_data->stats.tx_success++;
        LOG_DBG("Transmit done!");

    } while (0);

    if (switch_to_standby) {
        ral_status = ral_set_standby(ral_from_ralf(&ctrl_data->radio), RAL_STANDBY_CFG_RC);
        if (RAL_STATUS_OK != ral_status) {
            ret = -EIO;
            LOG_ERR("Failed to set the radio to standby! Status: %08X", ral_status);
        }
    }

    k_mutex_unlock(&ctrl_data->mutex);

    return ret;
}

static int radio_ctrl_impl_receive(const struct device *dev, uint8_t *data, size_t size,
                                   struct radio_ctrl_msg_stats *stats, uint32_t timeout)
{
    struct radio_ctrl_data *ctrl_data = dev->data;
    int ret = 0;
    struct radio_ctrl_msg msg = {0};

    do {
        if (!data) {
            ret = -EINVAL;
            break;
        }

        if (k_msgq_get(&ctrl_data->rx_msgq, &msg, K_MSEC(timeout)) == 0) {
            size_t copy_size = (size < msg.size) ? size : msg.size;
            memcpy(data, msg.data, copy_size);
            if (stats) {
                *stats = msg.stats;
            }
            ret = copy_size;
        } else {
            LOG_ERR("No RX message available");
            ret = -EIO;
        }

    } while (0);

    return ret;
}

static int radio_ctrl_impl_flush_rx_queue(const struct device *dev)
{
    struct radio_ctrl_data *ctrl_data = dev->data;
    struct radio_ctrl_msg msg;

    k_mutex_lock(&ctrl_data->mutex, K_FOREVER);

    while (k_msgq_get(&ctrl_data->rx_msgq, &msg, K_NO_WAIT) == 0) {
        // Discard messages
    }

    k_mutex_unlock(&ctrl_data->mutex);

    return 0;
}

static int radio_ctrl_impl_get_stats(const struct device *dev,
                                      struct radio_ctrl_stats *stats)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    if (!stats) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);

    *stats = data->stats;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static const struct radio_ctrl_driver_api radio_ctrl_api = {
    .set_config_gfsk = radio_ctrl_impl_set_config_gfsk,
    .set_config_lora = radio_ctrl_impl_set_config_lora,
    .set_config_flrc = radio_ctrl_impl_set_config_flrc,
    .set_config_lr_fhss = radio_ctrl_impl_set_config_lr_fhss,
    .set_antenna_switch = radio_ctrl_impl_set_antenna_switch,
    .get_lora_rx_params = radio_ctrl_impl_get_lora_rx_params,
    .set_lora_rx_params = radio_ctrl_impl_set_lora_rx_params,
    .get_lora_tx_params = radio_ctrl_impl_get_lora_tx_params,
    .set_lora_tx_params = radio_ctrl_impl_set_lora_tx_params,
    .get_lora_cad_params = radio_ctrl_impl_get_lora_cad_params,
    .set_lora_cad_params = radio_ctrl_impl_set_lora_cad_params,
    .listen = radio_ctrl_impl_listen,
    .cad = radio_ctrl_impl_cad,
    .transmit = radio_ctrl_impl_transmit,
    .receive = radio_ctrl_impl_receive,
    .flush_rx_queue = radio_ctrl_impl_flush_rx_queue,
    .get_stats = radio_ctrl_impl_get_stats,
};

static void radio_ctrl_rx_work(struct radio_ctrl_data *data)
{
    ral_status_t ral_status = RAL_STATUS_OK;
    ral_lora_rx_pkt_status_t lora_rx_status = {0};
    struct radio_ctrl_msg msg = {0};

    do {
        ral_status = ral_get_lora_rx_pkt_status(ral_from_ralf(&data->radio), &lora_rx_status);
        if (RAL_STATUS_OK != ral_status) {
            LOG_ERR("Failed to get the LoRa RX packet status! Status: %08X", ral_status);
            break;
        }

        msg.stats.rssi = lora_rx_status.rssi_pkt_in_dbm;
        msg.stats.snr = lora_rx_status.snr_pkt_in_db;
        msg.stats.signal_rssi = lora_rx_status.signal_rssi_pkt_in_dbm;

        ral_status =
            ral_get_pkt_payload(ral_from_ralf(&data->radio), sizeof(msg.data), msg.data, &msg.size);
        if (RAL_STATUS_OK != ral_status) {
            LOG_ERR("Failed to get the received payload! Status: %08X", ral_status);
            break;
        }

        if (k_msgq_put(&data->rx_msgq, &msg, K_NO_WAIT) != 0) {
            LOG_WRN("RX message queue full, dropping packet");
            break;
        }

        LOG_INF("Received packet (%d/%d): size=%d, rssi=%d dBm, snr=%d dB",
                k_msgq_num_used_get(&data->rx_msgq), CONFIG_RADIO_CTRL_RX_MSGQ_MAX_MSGS, msg.size,
                msg.stats.rssi, msg.stats.snr);
    } while (0);
}

static void radio_ctrl_irq_work(struct k_work *work)
{
    struct radio_ctrl_data *data = CONTAINER_OF(work, struct radio_ctrl_data, irq_work);
    ral_status_t ral_status = RAL_STATUS_OK;
    ral_irq_t irq_status = RAL_IRQ_NONE;

    do {
        ral_status = ral_get_and_clear_irq_status(ral_from_ralf(&data->radio), &irq_status);
        if (RAL_STATUS_OK != ral_status) {
            LOG_ERR("Failed to get and clear the IRQ status! Status: %08X", ral_status);
            break;
        }

        if (irq_status & RAL_IRQ_TX_DONE) {
            LOG_DBG("TX Done IRQ");
            data->stats.irq.tx_done++;
            k_event_post(&data->event, RADIO_CTRL_EVENT_TX_DONE);
        }
        if (irq_status & RAL_IRQ_RX_DONE) {
            LOG_DBG("RX Done IRQ");
            data->stats.irq.rx_done++;
            radio_ctrl_rx_work(data);
            k_event_post(&data->event, RADIO_CTRL_EVENT_RX_DONE);
        }
        if (irq_status & RAL_IRQ_RX_TIMEOUT) {
            LOG_DBG("RX Timeout IRQ");
            data->stats.irq.timeout++;
            k_event_post(&data->event, RADIO_CTRL_EVENT_RX_TIMEOUT);
        }
        if (irq_status & RAL_IRQ_RX_HDR_ERROR) {
            LOG_DBG("RX Header Error IRQ");
            data->stats.irq.hdr_error++;
            k_event_post(&data->event, RADIO_CTRL_EVENT_RX_HDR_ERROR);
        }
        if (irq_status & RAL_IRQ_RX_CRC_ERROR) {
            LOG_DBG("RX CRC Error IRQ");
            data->stats.irq.crc_error++;
            k_event_post(&data->event, RADIO_CTRL_EVENT_RX_CRC_ERROR);
        }
        if (irq_status & RAL_IRQ_CAD_DONE) {
            LOG_DBG("CAD Done IRQ");
            data->stats.irq.cad_done++;
            k_event_post(&data->event, RADIO_CTRL_EVENT_CAD_DONE);
        }
        if (irq_status & RAL_IRQ_CAD_OK) {
            LOG_DBG("CAD OK IRQ");
            data->stats.irq.cad_ok++;
            k_event_post(&data->event, RADIO_CTRL_EVENT_CAD_OK);
        }

        if (irq_status == RAL_IRQ_NONE) {
            LOG_DBG("Spurious IRQ");
            data->stats.irq.none++;
        }

    } while (0);
}

static void radio_ctrl_irq_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    struct radio_ctrl_data *data = CONTAINER_OF(cb, struct radio_ctrl_data, irq_cb_data);
    k_work_submit(&data->irq_work);
}

static int radio_ctrl_init(const struct device *dev)
{
    const struct radio_ctrl_config *cfg = dev->config;
    struct radio_ctrl_data *data = dev->data;
    ral_status_t ral_status = RAL_STATUS_OK;
    int ret = 0;

    data->dev = dev;

    k_work_init(&data->irq_work, radio_ctrl_irq_work);

    ret = k_mutex_init(&data->mutex);
    if (ret < 0) {
        LOG_ERR("Failed to initialize mutex");
        return ret;
    }

    k_event_init(&data->event);

    k_msgq_init(&data->rx_msgq, (char *)data->msg_buffer, sizeof(struct radio_ctrl_msg),
                CONFIG_RADIO_CTRL_RX_MSGQ_MAX_MSGS);

    if (!device_is_ready(cfg->spi.bus)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    if (!device_is_ready(cfg->nreset.port) || !device_is_ready(cfg->busy.port) ||
        !device_is_ready(cfg->irq.port)) {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }

    if (cfg->ant_sw.port && !device_is_ready(cfg->ant_sw.port)) {
        LOG_ERR("Antenna switch GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&cfg->nreset, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure NRESET pin");
        return ret;
    }
    ret = gpio_pin_configure_dt(&cfg->busy, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure BUSY pin");
        return ret;
    }
    ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT | GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ pin");
        return ret;
    }
    ret = gpio_pin_configure_dt(&cfg->ant_sw, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure ANT_SW pin");
        return ret;
    }

    gpio_init_callback(&data->irq_cb_data, radio_ctrl_irq_isr, BIT(cfg->irq.pin));
    ret = gpio_add_callback(cfg->irq.port, &data->irq_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add IRQ callback");
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ pin interrupt");
        return ret;
    }

    ret = gpio_pin_set_dt(&cfg->nreset, 1);
    if (ret < 0) {
        LOG_ERR("Failed to set NRESET pin high");
        return ret;
    }
    ret = gpio_pin_set_dt(&cfg->ant_sw, 1);
    if (ret < 0) {
        LOG_ERR("Failed to set ANT_SW pin high");
        return ret;
    }

    ret = radio_ctrl_impl_set_type(dev, RADIO_TYPE_SX126X);
    if (ret < 0) {
        LOG_ERR("Failed to set radio type");
        return ret;
    }

    ral_status = ral_init(ral_from_ralf(&data->radio));
    if (RAL_STATUS_OK != ral_status) {
        LOG_ERR("Failed to initialize RAL! Status: %08X", ral_status);
        return -EIO;
    }

    LOG_INF("SX1262 radio initialized");

    data->is_initialized = true;

    return 0;
}

#define RADIO_CTRL_INIT(inst)                                                                      \
    static struct radio_ctrl_data radio_ctrl_data_##inst;                                          \
                                                                                                   \
    static const struct radio_ctrl_config radio_ctrl_config_##inst = {                             \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),                  \
        .nreset = GPIO_DT_SPEC_INST_GET(inst, nreset_gpios),                                       \
        .busy = GPIO_DT_SPEC_INST_GET(inst, busy_gpios),                                           \
        .irq = GPIO_DT_SPEC_INST_GET(inst, dio1_gpios),                                            \
        .ant_sw = GPIO_DT_SPEC_INST_GET_OR(inst, antenna_switch_gpios, {0}),                       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, radio_ctrl_init, NULL, &radio_ctrl_data_##inst,                    \
                          &radio_ctrl_config_##inst, POST_KERNEL, CONFIG_RADIO_CTRL_INIT_PRIORITY, \
                          &radio_ctrl_api);

DT_INST_FOREACH_STATUS_OKAY(RADIO_CTRL_INIT)
