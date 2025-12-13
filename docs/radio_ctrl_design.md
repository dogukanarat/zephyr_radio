# Radio Control Module Design Document

## Overview

The `radio_ctrl` module is a Zephyr RTOS driver for Semtech wireless transceivers (SX126x, SX127x, SX128x, LR11xx). It provides a hardware abstraction layer for radio operations including transmission, reception, and channel activity detection (CAD).

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   radio_ctrl API (radio_ctrl.h)             │
│  - Configuration (LoRa, GFSK, FLRC, LR-FHSS)               │
│  - TX/RX Operations                                         │
│  - Statistics & Monitoring                                  │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   RALF (Radio Abstraction Layer Framework)   │
│  - ralf_sx126x, ralf_sx127x, ralf_sx128x, ralf_lr11xx       │
│  - High-level radio parameter setup                         │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   RAL (Radio Abstraction Layer)              │
│  - ral_sx126x, ral_sx127x, ral_sx128x, ral_lr11xx           │
│  - Low-level radio command abstraction                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   HAL (Hardware Abstraction Layer)           │
│  - SPI communication                                        │
│  - GPIO control (RESET, BUSY, IRQ)                         │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Hardware (SX126x Radio Chip)              │
└─────────────────────────────────────────────────────────────┘
```

## Module Components

### 1. Public API (`include/zephyr/drivers/radio_ctrl.h`)

The public API exposes the following capabilities:

| Category | Functions |
|----------|-----------|
| **Configuration** | `set_config_gfsk`, `set_config_lora`, `set_config_flrc`, `set_config_lr_fhss` |
| **LoRa Parameters** | `get/set_lora_rx_params`, `get/set_lora_tx_params`, `get/set_lora_cad_params` |
| **Operations** | `listen`, `cad`, `transmit`, `receive`, `flush_rx_queue` |
| **Hardware** | `set_antenna_switch` |
| **Monitoring** | `get_stats` |

### 2. Data Structures

#### Configuration Structures

```c
struct radio_ctrl_config {
    const struct spi_dt_spec spi;      // SPI bus specification
    struct gpio_dt_spec nreset;         // Hardware reset pin
    struct gpio_dt_spec busy;           // Busy state indicator
    struct gpio_dt_spec irq;            // Interrupt pin (DIO1)
    struct gpio_dt_spec ant_sw;         // Antenna switch (optional)
};
```

#### Runtime Data

```c
struct radio_ctrl_data {
    struct k_work irq_work;             // Deferred IRQ processing
    struct k_mutex mutex;               // Thread synchronization
    struct gpio_callback irq_cb_data;   // GPIO callback
    const struct device *dev;           // Device reference
    bool is_cad_enabled;                // CAD mode flag
    ralf_t radio;                       // RALF instance
    ralf_params_lora_t rx_params;       // RX configuration
    ralf_params_lora_t tx_params;       // TX configuration
    ralf_params_lora_cad_t cad_params;  // CAD configuration
    struct radio_ctrl_stats stats;      // Statistics
    sx126x_hal_context_t sx126x_hal_ctx;// HAL context
    struct k_event event;               // Event signaling
    struct radio_ctrl_msg msg_buffer[]; // RX message buffer
    struct k_msgq rx_msgq;              // RX message queue
    bool is_configured;                 // Configuration state
    bool is_ralf_initialized;           // RALF init state
    bool is_initialized;                // Full init state
};
```

### 3. Synchronization Primitives

| Primitive | Purpose |
|-----------|---------|
| `k_mutex` | Protects shared data structures during configuration and operations |
| `k_event` | Signals completion of TX/RX operations from ISR to thread context |
| `k_msgq` | Buffers received packets for application retrieval |
| `k_work` | Defers interrupt handling to thread context |

### 4. Interrupt Handling Flow

```
┌──────────┐    ┌──────────────┐    ┌─────────────────┐    ┌──────────────┐
│ DIO1 IRQ │───▶│ GPIO ISR     │───▶│ k_work_submit() │───▶│ Work Queue   │
│ (HW)     │    │ (ISR ctx)    │    │                 │    │ (Thread ctx) │
└──────────┘    └──────────────┘    └─────────────────┘    └──────────────┘
                                                                    │
                                                                    ▼
                                                           ┌──────────────────┐
                                                           │ radio_ctrl_irq_  │
                                                           │ work()           │
                                                           │ - Read IRQ status│
                                                           │ - Process RX     │
                                                           │ - Post events    │
                                                           └──────────────────┘
```

### 5. Event Flags

| Event | Value | Description |
|-------|-------|-------------|
| `RADIO_CTRL_EVENT_TX_DONE` | BIT(0) | Transmission complete |
| `RADIO_CTRL_EVENT_RX_DONE` | BIT(1) | Packet received |
| `RADIO_CTRL_EVENT_RX_TIMEOUT` | BIT(2) | RX timeout |
| `RADIO_CTRL_EVENT_RX_HDR_ERROR` | BIT(3) | Header error |
| `RADIO_CTRL_EVENT_RX_CRC_ERROR` | BIT(4) | CRC error |
| `RADIO_CTRL_EVENT_CAD_DONE` | BIT(5) | CAD complete |
| `RADIO_CTRL_EVENT_CAD_OK` | BIT(6) | Channel detected |

## Supported Radio Types

| Type | Status | Notes |
|------|--------|-------|
| SX126x | **Active** | Fully implemented |
| SX127x | Defined | Not implemented |
| SX128x | Defined | Not implemented |
| LR1110 | Defined | Not implemented |

## Modulation Modes

| Mode | Status | Use Case |
|------|--------|----------|
| LoRa | **Implemented** | Long-range, low-power |
| GFSK | Stub | Higher data rate, shorter range |
| FLRC | Stub | Fast Long Range Communication |
| LR-FHSS | Stub | Frequency hopping spread spectrum |

## Configuration (Kconfig)

| Option | Default | Description |
|--------|---------|-------------|
| `RADIO_CTRL` | n | Enable radio controller drivers |
| `RADIO_CTRL_SX1262` | y (if DT enabled) | Enable SX1262 driver |
| `RADIO_CTRL_LOG_LEVEL` | 3 | Log verbosity (0-4) |
| `RADIO_CTRL_INIT_PRIORITY` | 90 | Device init priority |
| `RADIO_CTRL_MAX_MSG_SIZE` | 256 | Maximum packet size (bytes) |
| `RADIO_CTRL_TIMEOUT_MS` | 1000 | Operation timeout (ms) |
| `RADIO_CTRL_RX_MSGQ_MAX_MSGS` | 10 | RX queue depth |

## Device Tree Binding

Compatible string: `da,sx126x-radio`

### Required Properties

| Property | Type | Description |
|----------|------|-------------|
| `nreset-gpios` | phandle-array | Hardware reset pin |
| `busy-gpios` | phandle-array | Busy state indicator |
| `dio1-gpios` | phandle-array | Interrupt pin |

### Optional Properties

| Property | Type | Description |
|----------|------|-------------|
| `dio2-gpios` | phandle-array | Secondary interrupt |
| `xtal-sel-gpios` | phandle-array | Crystal selection |
| `freq-sel-gpios` | phandle-array | Frequency band selection |
| `antenna-switch-gpios` | phandle-array | RF switch control |

### Example Device Tree Node

```dts
&spi1 {
    status = "okay";
    cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;

    lora: sx1262@0 {
        compatible = "da,sx126x-radio";
        reg = <0>;
        spi-max-frequency = <16000000>;
        nreset-gpios = <&gpio0 3 GPIO_ACTIVE_LOW>;
        busy-gpios = <&gpio0 4 GPIO_ACTIVE_HIGH>;
        dio1-gpios = <&gpio0 5 GPIO_ACTIVE_HIGH>;
        antenna-switch-gpios = <&gpio0 25 GPIO_ACTIVE_HIGH>;
        status = "okay";
    };
};
```

## Usage Example

```c
#include <zephyr/drivers/radio_ctrl.h>

const struct device *radio = DEVICE_DT_GET(DT_NODELABEL(lora));

/* Configure LoRa parameters */
ralf_params_lora_t rx_params = { /* ... */ };
ralf_params_lora_t tx_params = { /* ... */ };
ralf_params_lora_cad_t cad_params = { /* ... */ };

radio_ctrl_set_config_lora(radio, &rx_params, &tx_params, &cad_params);

/* Transmit a packet */
uint8_t data[] = "Hello, LoRa!";
radio_ctrl_transmit(radio, data, sizeof(data));

/* Start listening */
radio_ctrl_listen(radio, 5000);  /* 5 second timeout */

/* Receive a packet */
uint8_t rx_buffer[256];
struct radio_ctrl_msg_stats stats;
int len = radio_ctrl_receive(radio, rx_buffer, sizeof(rx_buffer), &stats, 10000);

if (len > 0) {
    printk("Received %d bytes, RSSI: %d dBm\n", len, stats.rssi);
}
```

## Statistics

The driver maintains comprehensive statistics:

```c
struct radio_ctrl_stats {
    uint32_t tx_attempts;    // Total TX attempts
    uint32_t tx_success;     // Successful transmissions
    uint32_t tx_timeout;     // TX timeouts
    uint32_t rx_attempts;    // Total RX attempts
    uint32_t rx_success;     // Successful receptions
    uint32_t rx_timeout;     // RX timeouts
    struct radio_ctrl_irq_stats irq;  // IRQ counters
};
```

## Thread Safety

The driver is designed to be thread-safe for concurrent access from multiple threads:

### Synchronization Mechanisms

| Resource | Protection | Notes |
|----------|------------|-------|
| SPI bus | Mutex | All SPI transactions protected by shared mutex |
| Configuration params | Mutex | Get/set operations hold mutex |
| TX/RX stats | Mutex | Protected for consistent reads |
| IRQ stats | Atomic ops | Lock-free using `atomic_t` counters |
| RX message queue | k_msgq | Inherently thread-safe |
| Event flags | k_event | Kernel-provided thread safety |

### Concurrent Operation Support

- **TX + RX**: Supported. `receive()` doesn't hold mutex while waiting.
- **Multiple TX**: Serialized via mutex - only one TX at a time.
- **TX during IRQ**: Safe. TX releases mutex before waiting, allowing IRQ work to proceed.

### Deadlock Prevention

The transmit function uses a careful lock ordering to avoid deadlock:
1. Acquire mutex for TX setup
2. Release mutex before `k_event_wait()`
3. IRQ work handler can now acquire mutex
4. Re-acquire mutex for cleanup after event

### IRQ Context Safety

- GPIO ISR immediately defers to work queue (no SPI in ISR)
- Work queue handler acquires mutex before SPI operations
- Atomic operations used for IRQ statistics (no lock needed)
- Events posted after mutex release to avoid priority inversion

## Limitations

1. Only SX126x radios are fully implemented
2. GFSK, FLRC, and LR-FHSS modes have stub implementations
3. CAD (Channel Activity Detection) is not implemented
4. Single radio instance per device (no multi-radio support in current design)

## File Structure

```
zephyr-radio/
├── include/zephyr/drivers/
│   ├── radio_ctrl.h          # Public API
│   ├── ral/                   # Radio Abstraction Layer headers
│   ├── ralf/                  # RALF headers
│   └── sx126x/                # SX126x-specific headers
├── lib/drivers/radio/
│   ├── radio_ctrl.c          # Main implementation
│   ├── ral/                   # RAL implementations
│   ├── ralf/                  # RALF implementations
│   └── sx126x/                # SX126x driver
├── dts/bindings/radio/
│   └── da,sx126x-radio.yaml  # Device tree binding
├── CMakeLists.txt            # Build configuration
├── Kconfig                   # Configuration options
└── zephyr/module.yml         # Zephyr module definition
```
