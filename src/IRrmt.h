// Copyright 2023 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IR_RMT_H_
#define IR_RMT_H_

#if defined(ESP_PLATFORM) && !defined(ARDUINO)

#include "soc/soc_caps.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "hal/rmt_ll.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "IRremoteESP8266.h"

#if SOC_RMT_SUPPORTED

typedef enum {
  RMT_MEM_NUM_BLOCKS_1 = 1,
  RMT_MEM_NUM_BLOCKS_2 = 2,
#if SOC_RMT_TX_CANDIDATES_PER_GROUP > 2
  RMT_MEM_NUM_BLOCKS_3 = 3,
  RMT_MEM_NUM_BLOCKS_4 = 4,
#if SOC_RMT_TX_CANDIDATES_PER_GROUP > 4
  RMT_MEM_NUM_BLOCKS_5 = 5,
  RMT_MEM_NUM_BLOCKS_6 = 6,
  RMT_MEM_NUM_BLOCKS_7 = 7,
  RMT_MEM_NUM_BLOCKS_8 = 8,
#endif
#endif
} rmt_reserve_memsize_t;

typedef enum {
  ESP32_BUS_TYPE_INIT,      // IO has not been attached to a bus yet
  ESP32_BUS_TYPE_GPIO,      // IO is used as GPIO
  ESP32_BUS_TYPE_UART_RX,   // IO is used as UART RX pin
  ESP32_BUS_TYPE_UART_TX,   // IO is used as UART TX pin
  ESP32_BUS_TYPE_UART_CTS,  // IO is used as UART CTS pin
  ESP32_BUS_TYPE_UART_RTS,  // IO is used as UART RTS pin
#if SOC_SDM_SUPPORTED
  ESP32_BUS_TYPE_SIGMADELTA,  // IO is used as SigmeDelta output
#endif
#if SOC_ADC_SUPPORTED
  ESP32_BUS_TYPE_ADC_ONESHOT,  // IO is used as ADC OneShot input
  ESP32_BUS_TYPE_ADC_CONT,     // IO is used as ADC continuous input
#endif
#if SOC_DAC_SUPPORTED
  ESP32_BUS_TYPE_DAC_ONESHOT,  // IO is used as DAC OneShot output
  ESP32_BUS_TYPE_DAC_CONT,     // IO is used as DAC continuous output
  ESP32_BUS_TYPE_DAC_COSINE,   // IO is used as DAC cosine output
#endif
#if SOC_LEDC_SUPPORTED
  ESP32_BUS_TYPE_LEDC,  // IO is used as LEDC output
#endif
#if SOC_RMT_SUPPORTED
  ESP32_BUS_TYPE_RMT_TX,  // IO is used as RMT output
  ESP32_BUS_TYPE_RMT_RX,  // IO is used as RMT input
#endif
#if SOC_I2S_SUPPORTED
  ESP32_BUS_TYPE_I2S_STD_MCLK,  // IO is used as I2S STD MCLK pin
  ESP32_BUS_TYPE_I2S_STD_BCLK,  // IO is used as I2S STD BCLK pin
  ESP32_BUS_TYPE_I2S_STD_WS,    // IO is used as I2S STD WS pin
  ESP32_BUS_TYPE_I2S_STD_DOUT,  // IO is used as I2S STD DOUT pin
  ESP32_BUS_TYPE_I2S_STD_DIN,   // IO is used as I2S STD DIN pin

  ESP32_BUS_TYPE_I2S_TDM_MCLK,  // IO is used as I2S TDM MCLK pin
  ESP32_BUS_TYPE_I2S_TDM_BCLK,  // IO is used as I2S TDM BCLK pin
  ESP32_BUS_TYPE_I2S_TDM_WS,    // IO is used as I2S TDM WS pin
  ESP32_BUS_TYPE_I2S_TDM_DOUT,  // IO is used as I2S TDM DOUT pin
  ESP32_BUS_TYPE_I2S_TDM_DIN,   // IO is used as I2S TDM DIN pin

  ESP32_BUS_TYPE_I2S_PDM_TX_CLK,    // IO is used as I2S PDM CLK pin
  ESP32_BUS_TYPE_I2S_PDM_TX_DOUT0,  // IO is used as I2S PDM DOUT0 pin
  ESP32_BUS_TYPE_I2S_PDM_TX_DOUT1,  // IO is used as I2S PDM DOUT1 pin

  ESP32_BUS_TYPE_I2S_PDM_RX_CLK,   // IO is used as I2S PDM CLK pin
  ESP32_BUS_TYPE_I2S_PDM_RX_DIN0,  // IO is used as I2S PDM DIN0 pin
  ESP32_BUS_TYPE_I2S_PDM_RX_DIN1,  // IO is used as I2S PDM DIN1 pin
  ESP32_BUS_TYPE_I2S_PDM_RX_DIN2,  // IO is used as I2S PDM DIN2 pin
  ESP32_BUS_TYPE_I2S_PDM_RX_DIN3,  // IO is used as I2S PDM DIN3 pin
#endif
#if SOC_I2C_SUPPORTED
  ESP32_BUS_TYPE_I2C_MASTER_SDA,  // IO is used as I2C master SDA pin
  ESP32_BUS_TYPE_I2C_MASTER_SCL,  // IO is used as I2C master SCL pin
  ESP32_BUS_TYPE_I2C_SLAVE_SDA,   // IO is used as I2C slave SDA pin
  ESP32_BUS_TYPE_I2C_SLAVE_SCL,   // IO is used as I2C slave SCL pin
#endif
#if SOC_GPSPI_SUPPORTED
  ESP32_BUS_TYPE_SPI_MASTER_SCK,   // IO is used as SPI master SCK pin
  ESP32_BUS_TYPE_SPI_MASTER_MISO,  // IO is used as SPI master MISO pin
  ESP32_BUS_TYPE_SPI_MASTER_MOSI,  // IO is used as SPI master MOSI pin
  ESP32_BUS_TYPE_SPI_MASTER_SS,    // IO is used as SPI master SS pin
#endif
#if SOC_SDMMC_HOST_SUPPORTED
  ESP32_BUS_TYPE_SDMMC_CLK,  // IO is used as SDMMC CLK pin
  ESP32_BUS_TYPE_SDMMC_CMD,  // IO is used as SDMMC CMD pin
  ESP32_BUS_TYPE_SDMMC_D0,   // IO is used as SDMMC D0 pin
  ESP32_BUS_TYPE_SDMMC_D1,   // IO is used as SDMMC D1 pin
  ESP32_BUS_TYPE_SDMMC_D2,   // IO is used as SDMMC D2 pin
  ESP32_BUS_TYPE_SDMMC_D3,   // IO is used as SDMMC D3 pin
#endif
#if SOC_TOUCH_SENSOR_SUPPORTED
  ESP32_BUS_TYPE_TOUCH,  // IO is used as TOUCH pin
#endif
#if SOC_USB_SERIAL_JTAG_SUPPORTED || SOC_USB_OTG_SUPPORTED
  ESP32_BUS_TYPE_USB_DM,  // IO is used as USB DM (+) pin
  ESP32_BUS_TYPE_USB_DP,  // IO is used as USB DP (-) pin
#endif
#if SOC_GPSPI_SUPPORTED
  ESP32_BUS_TYPE_ETHERNET_SPI,  // IO is used as ETHERNET SPI pin
#endif
#if CONFIG_ETH_USE_ESP32_EMAC
  ESP32_BUS_TYPE_ETHERNET_RMII,  // IO is used as ETHERNET RMII pin
  ESP32_BUS_TYPE_ETHERNET_CLK,   // IO is used as ETHERNET CLK pin
  ESP32_BUS_TYPE_ETHERNET_MCD,   // IO is used as ETHERNET MCD pin
  ESP32_BUS_TYPE_ETHERNET_MDIO,  // IO is used as ETHERNET MDIO pin
  ESP32_BUS_TYPE_ETHERNET_PWR,   // IO is used as ETHERNET PWR pin
#endif
#if CONFIG_LWIP_PPP_SUPPORT
  ESP32_BUS_TYPE_PPP_TX,   // IO is used as PPP Modem TX pin
  ESP32_BUS_TYPE_PPP_RX,   // IO is used as PPP Modem RX pin
  ESP32_BUS_TYPE_PPP_RTS,  // IO is used as PPP Modem RTS pin
  ESP32_BUS_TYPE_PPP_CTS,  // IO is used as PPP Modem CTS pin
#endif
  ESP32_BUS_TYPE_MAX
} peripheral_bus_type_t;

// Each RMT Symbols has 4 bytes
// Total number of bytes per RMT_MEM_BLOCK is RMT_SYMBOLS_PER_CHANNEL_BLOCK * 4 bytes
typedef union {
  struct {
    uint32_t duration0 : 15;
    uint32_t level0    : 1;
    uint32_t duration1 : 15;
    uint32_t level1    : 1;
  };
  uint32_t val;
} rmt_data_t;

// Reading and Writing shall use as rmt_symbols_size this unit
// ESP32 has 8 MEM BLOCKS in total shared with Reading and/or Writing
// ESP32-S2 has 4 MEM BLOCKS in total shared with Reading and/or Writing
// ESP32-S3 has 4 MEM BLOCKS for Reading and another 4 MEM BLOCKS for Writing
// ESP32-C3 has 2 MEM BLOCKS for Reading and another 2 MEM BLOCKS for Writing
#define RMT_SYMBOLS_PER_CHANNEL_BLOCK SOC_RMT_MEM_WORDS_PER_CHANNEL

// Used to tell rmtRead() to wait for ever until reading data from the RMT channel
#define RMT_WAIT_FOR_EVER ((uint32_t)portMAX_DELAY)

// Helper macro to calculate the number of RTM symbols in a array or type
#define RMT_SYMBOLS_OF(x) (sizeof(x) / sizeof(rmt_data_t))

struct rmt_obj_s {
  // general RMT information
  rmt_channel_handle_t rmt_channel_h;       // IDF RMT channel handler
  rmt_encoder_handle_t rmt_copy_encoder_h;  // RMT simple copy encoder handle

  uint32_t signal_range_min_ns;  // RX Filter data - Low Pass pulse width
  uint32_t signal_range_max_ns;  // RX idle time that defines end of reading

  EventGroupHandle_t rmt_events;   // read/write done event RMT callback handle
  bool rmt_ch_is_looping;          // Is this RMT TX Channel in LOOPING MODE?
  size_t *num_symbols_read;        // Pointer to the number of RMT symbol read by IDF RMT RX Done
  rmt_reserve_memsize_t mem_size;  // RMT Memory size
  uint32_t frequency_Hz;           // RMT Frequency
  uint8_t rmt_EOT_Level;           // RMT End of Transmission Level - default is LOW

#if !CONFIG_DISABLE_HAL_LOCKS
  SemaphoreHandle_t g_rmt_objlocks;  // Channel Semaphore Lock
#endif                               /* CONFIG_DISABLE_HAL_LOCKS */
};

typedef struct rmt_obj_s *rmt_bus_handle_t;

class IRrmt {
  public:
    explicit IRrmt(uint16_t pin, rmt_ch_dir_t direction, bool inverted = false);
    // explicit IRrmt();
    bool begin();
    bool enableOut(uint32_t freq, uint8_t duty = 50);
    bool write(rmt_data_t *data, size_t num_rmt_symbols, bool blocking, uint32_t timeout_ms);
    bool read(rmt_data_t *data, size_t *num_rmt_symbols, bool wait_for_data, uint32_t timeout_ms);
    bool readCompleted();
    bool end();

  protected:
    uint16_t _rmt_pin;
    rmt_ch_dir_t _rmt_direction;
    bool _rmt_inverted;
    rmt_bus_handle_t _rmt_bus = nullptr;
};

#endif /* SOC_RMT_SUPPORTED */
#endif /* ESP_PLATFORM */
#endif /* IR_RMT_H_ */