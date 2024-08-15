// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
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

#include "soc/soc_caps.h"

#if defined(ESP_PLATFORM) && !defined(ARDUINO)

#if SOC_RMT_SUPPORTED
// #include "esp32-hal.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "hal/rmt_ll.h"

#include "IRrmt.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #define delay(x) vTaskDelay(x / portTICK_PERIOD_MS)

// RMT Events
#define RMT_FLAG_RX_DONE (1)
#define RMT_FLAG_TX_DONE (2)

/// Constructor for an IRrmt object.
IRrmt::IRrmt(uint16_t pin, rmt_ch_dir_t direction, bool inverted) {
  this->_rmt_pin = pin;
  this->_rmt_direction = direction;
  this->_rmt_inverted = inverted;
}

// This is called from an IDF ISR code, therefore this code is part of an ISR
static bool _rmt_tx_done_callback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *data, void *args) {
  BaseType_t high_task_wakeup = pdFALSE;
  rmt_bus_handle_t bus = (rmt_bus_handle_t)args;
  // set RX event group and signal the received RMT symbols of that channel
  xEventGroupSetBitsFromISR(bus->rmt_events, RMT_FLAG_TX_DONE, &high_task_wakeup);
  // A "need to yield" is returned in order to execute portYIELD_FROM_ISR() in the main IDF RX ISR
  return high_task_wakeup == pdTRUE;
}

// This is called from an IDF ISR code, therefore this code is part of an ISR
static bool _rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *data, void *args) {
  BaseType_t high_task_wakeup = pdFALSE;
  rmt_bus_handle_t bus = (rmt_bus_handle_t)args;
  // sets the returning number of RMT symbols (32 bits) effectively read
  *bus->num_symbols_read = data->num_symbols;
  // set RX event group and signal the received RMT symbols of that channel
  xEventGroupSetBitsFromISR(bus->rmt_events, RMT_FLAG_RX_DONE, &high_task_wakeup);
  // A "need to yield" is returned in order to execute portYIELD_FROM_ISR() in the main IDF RX ISR
  return high_task_wakeup == pdTRUE;
}

bool IRrmt::begin() {
  // allocate the rmt bus object and sets all fields to NULL
  if (this->_rmt_bus == nullptr){
    this->_rmt_bus = (rmt_bus_handle_t)heap_caps_calloc(1, sizeof(struct rmt_obj_s), MALLOC_CAP_DEFAULT);
    if (this->_rmt_bus == NULL) {
      ESP_LOGE(RMT_TAG, "GPIO %d - Bus Memory allocation fault.", this->_rmt_pin);
      return false;
    }
  }
  else{
    ESP_LOGE(RMT_TAG, "RMT Bus already created");
    return false;
  }

  // check is pin is valid and in the right direction
  if ((this->_rmt_direction == RMT_TX_MODE && !GPIO_IS_VALID_OUTPUT_GPIO(this->_rmt_pin)) || (!GPIO_IS_VALID_GPIO(this->_rmt_pin))) {
    ESP_LOGE(RMT_TAG, "GPIO %d is not valid or can't be used for output in TX mode.", this->_rmt_pin);
    return false;
  }

  // store the RMT Freq and mem_size to check Initialization, Filter and Idle valid values in the RMT API
  this->_rmt_bus->frequency_Hz = 1000000;
  this->_rmt_bus->mem_size = RMT_MEM_NUM_BLOCKS_1;
  // pulses with width smaller than min_ns will be ignored (as a glitch)
  //bus->signal_range_min_ns = 0; // disabled  --> not necessary CALLOC set all to ZERO.
  // RMT stops reading if the input stays idle for longer than max_ns
  this->_rmt_bus->signal_range_max_ns = (1000000000 / this->_rmt_bus->frequency_Hz) * RMT_LL_MAX_IDLE_VALUE;  // maximum possible
  // creates the event group to control read_done and write_done
  this->_rmt_bus->rmt_events = xEventGroupCreate();
  if (this->_rmt_bus->rmt_events == NULL) {
    ESP_LOGE(RMT_TAG, "GPIO %d - RMT Group Event allocation fault.", this->_rmt_pin);
    return false;
  }

  // Starting with Receive|Transmit DONE bits set, for allowing a new request from user
  xEventGroupSetBits(this->_rmt_bus->rmt_events, RMT_FLAG_RX_DONE | RMT_FLAG_TX_DONE);

  bool ret = true;
  // channel particular configuration
  if (this->_rmt_direction == RMT_TX_MODE) {
    // TX Channel
    rmt_tx_channel_config_t tx_cfg;
    tx_cfg.gpio_num = (gpio_num_t)this->_rmt_pin;
    // CLK_APB for ESP32|S2|S3|C3 -- CLK_PLL_F80M for C6 -- CLK_XTAL for H2
    tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz = this->_rmt_bus->frequency_Hz;
    tx_cfg.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL * this->_rmt_bus->mem_size;
    tx_cfg.trans_queue_depth = 10;  // maximum allowed
    tx_cfg.flags.invert_out = this->_rmt_inverted;
    tx_cfg.flags.with_dma = 0;
    tx_cfg.flags.io_loop_back = 0;
    tx_cfg.flags.io_od_mode = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2)
    tx_cfg.intr_priority = 0;
#endif

    if (rmt_new_tx_channel(&tx_cfg, &this->_rmt_bus->rmt_channel_h) != ESP_OK) {
      ESP_LOGE(RMT_TAG, "GPIO %d - RMT TX Initialization error.", this->_rmt_pin);
      ret = false;
    }

    // set TX Callback
    rmt_tx_event_callbacks_t cbs = {.on_trans_done = _rmt_tx_done_callback};
    if (ESP_OK != rmt_tx_register_event_callbacks(this->_rmt_bus->rmt_channel_h, &cbs, this->_rmt_bus)) {
      ESP_LOGE(RMT_TAG, "GPIO %d RMT - Error registering TX Callback.", this->_rmt_pin);
      ret = false;
    }

  } else {
    // RX Channel
    rmt_rx_channel_config_t rx_cfg;
    rx_cfg.gpio_num = (gpio_num_t)this->_rmt_pin;
    // CLK_APB for ESP32|S2|S3|C3 -- CLK_PLL_F80M for C6 -- CLK_XTAL for H2
    rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_cfg.resolution_hz = this->_rmt_bus->frequency_Hz;
    rx_cfg.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL * this->_rmt_bus->mem_size;
    rx_cfg.flags.invert_in = this->_rmt_inverted;
    rx_cfg.flags.with_dma = 0;
    rx_cfg.flags.io_loop_back = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2)
    rx_cfg.intr_priority = 0;
#endif
    // try to allocate the RMT Channel
    if (ESP_OK != rmt_new_rx_channel(&rx_cfg, &this->_rmt_bus->rmt_channel_h)) {
      ESP_LOGE(RMT_TAG, "GPIO %d RMT - RX Initialization error.", this->_rmt_pin);
      ret = false;
    }

    // set RX Callback
    rmt_rx_event_callbacks_t cbs = {.on_recv_done = _rmt_rx_done_callback};
    if (ESP_OK != rmt_rx_register_event_callbacks(this->_rmt_bus->rmt_channel_h, &cbs, this->_rmt_bus)) {
      ESP_LOGE(RMT_TAG, "GPIO %d RMT - Error registering RX Callback.", this->_rmt_pin);
      ret = false;
    }
  }

  // allocate memory for the RMT Copy encoder
  rmt_copy_encoder_config_t copy_encoder_config = {};
  if (rmt_new_copy_encoder(&copy_encoder_config, &this->_rmt_bus->rmt_copy_encoder_h) != ESP_OK) {
    ESP_LOGE(RMT_TAG, "GPIO %d - RMT Encoder Memory Allocation error.", this->_rmt_pin);
    ret = false;
  }

  if (rmt_enable(this->_rmt_bus->rmt_channel_h) != ESP_OK) {  // starts/enables the channel
    ESP_LOGE(RMT_TAG, "GPIO %d - RMT enable error.", this->_rmt_pin);
    ret = false;
  }

  if (ret == false){
    this->end();
  }
  return ret;
}

bool IRrmt::enableOut(uint32_t freq, uint8_t duty){
  // int pin, bool carrier_en, bool carrier_level, uint32_t frequency_Hz, float duty_percent) {
  bool carrier_level = false;
  float duty_percent = (float)duty/100;

  if (this->_rmt_bus == NULL) {
    ESP_LOGE(RMT_TAG, "enableRmtOut: RMT Bus NULL");
    return false;
  }

  if (duty_percent > 1) {
    ESP_LOGW(RMT_TAG, "GPIO %d - RMT Carrier must be a float percentage from 0 to 1 -> Setting to 50%%.", this->_rmt_pin);
    duty_percent = 0.5;
  }
  rmt_carrier_config_t carrier_cfg = { freq, duty_percent, false, carrier_level };

  // modulate carrier to TX channel
  if (ESP_OK != rmt_apply_carrier(this->_rmt_bus->rmt_channel_h, &carrier_cfg)) {
    ESP_LOGE(RMT_TAG, "GPIO %d - Error applying RMT carrier.", this->_rmt_pin);
    return false;
  }

  return true;
}

bool IRrmt::write(rmt_data_t *data, size_t num_rmt_symbols, bool blocking, uint32_t timeout_ms){
  bool loop = false;
  if (this->_rmt_bus == NULL) {
    return false;
  }
  if (this->_rmt_direction != RMT_TX_MODE) {
    return false;
  }
  bool loopCancel = false;  // user wants to cancel the writing loop mode
  if (data == NULL || num_rmt_symbols == 0) {
    if (!loop) {
      ESP_LOGE(RMT_TAG, "GPIO %d - RMT Write Data NULL pointer or size is zero.", this->_rmt_pin);
      return false;
    } else {
      loopCancel = true;
    }
  }

  ESP_LOGV(RMT_TAG, "GPIO: %d - Request: %d RMT Symbols - %s - Timeout: %ld", this->_rmt_pin, num_rmt_symbols, blocking ? "Blocking" : "Non-Blocking", timeout_ms);
  ESP_LOGV(RMT_TAG,
    "GPIO: %d - Currently in Loop Mode: [%s] | Asked to Loop: %s, LoopCancel: %s", this->_rmt_pin, this->_rmt_bus->rmt_ch_is_looping ? "YES" : "NO", loop ? "YES" : "NO",
    loopCancel ? "YES" : "NO"
  );

  if ((xEventGroupGetBits(this->_rmt_bus->rmt_events) & RMT_FLAG_TX_DONE) == 0) {
    ESP_LOGE(RMT_TAG, "GPIO %d - RMT Write still pending to be completed.", this->_rmt_pin);
    return false;
  }

  rmt_transmit_config_t transmit_cfg = { 0, 0, 0 };  // loop mode disabled
  bool retCode = true;

  // wants to start in writing or looping over a previous looping --> resets the channel
  if (this->_rmt_bus->rmt_ch_is_looping == true) {
    // must force stopping a previous loop transmission first
    rmt_disable(this->_rmt_bus->rmt_channel_h);
    // enable it again for looping or writing
    rmt_enable(this->_rmt_bus->rmt_channel_h);
    this->_rmt_bus->rmt_ch_is_looping = false;  // not looping anymore
  }
  // sets the End of Transmission level to HIGH if the user has requested so
  if (this->_rmt_bus->rmt_EOT_Level) {
    transmit_cfg.flags.eot_level = 1;  // EOT is HIGH
  }
  if (loopCancel) {
    // just resets and releases the channel, maybe, already done above, then exits
    this->_rmt_bus->rmt_ch_is_looping = false;
  } else {  // new writing | looping request
    // looping | Writing over a previous looping state is valid
    if (loop) {
      transmit_cfg.loop_count = -1;  // enable infinite loop mode
      // keeps RMT_FLAG_TX_DONE set - it never changes
    } else {
      // looping mode never sets this flag (IDF 5.1) in the callback
      xEventGroupClearBits(this->_rmt_bus->rmt_events, RMT_FLAG_TX_DONE);
    }
    // transmits just once or looping data
    if (ESP_OK != rmt_transmit(this->_rmt_bus->rmt_channel_h, this->_rmt_bus->rmt_copy_encoder_h, (const void *)data, num_rmt_symbols * sizeof(rmt_data_t), &transmit_cfg)) {
      retCode = false;
      ESP_LOGW(RMT_TAG, "GPIO %d - RMT Transmission failed.", this->_rmt_pin);
    } else {  // transmit OK
      if (loop) {
        this->_rmt_bus->rmt_ch_is_looping = true;  // for ever... until a channel canceling or new writing
      } else {
        if (blocking) {
          // wait for transmission confirmation | timeout
          retCode = (xEventGroupWaitBits(this->_rmt_bus->rmt_events, RMT_FLAG_TX_DONE, pdFALSE /* do not clear on exit */, pdFALSE /* wait for all bits */, timeout_ms)
                     & RMT_FLAG_TX_DONE)
                    != 0;
        }
      }
    }
  }
  return retCode;
}

bool IRrmt::read(rmt_data_t *data, size_t *num_rmt_symbols, bool wait_for_data, uint32_t timeout_ms) {
  if (this->_rmt_bus == NULL) {
    return false;
  }
  if (this->_rmt_direction != RMT_RX_MODE) {
    return false;
  }
  if (data == NULL || num_rmt_symbols == NULL) {
    ESP_LOGE(RMT_TAG, "GPIO %d - RMT Read Data and/or Size NULL pointer.", this->_rmt_pin);
    return false;
  }
  ESP_LOGV(RMT_TAG, "GPIO: %d - Request: %d RMT Symbols - %s - Timeout: %ld", this->_rmt_pin, *num_rmt_symbols, wait_for_data ? "Blocking" : "Non-Blocking", timeout_ms);
  bool ret = true;

  // request reading RMT Channel Data
  rmt_receive_config_t receive_config;
  receive_config.signal_range_min_ns = this->_rmt_bus->signal_range_min_ns;
  receive_config.signal_range_max_ns = this->_rmt_bus->signal_range_max_ns;

  xEventGroupClearBits(this->_rmt_bus->rmt_events, RMT_FLAG_RX_DONE);
  this->_rmt_bus->num_symbols_read = num_rmt_symbols;

  rmt_receive(this->_rmt_bus->rmt_channel_h, data, *num_rmt_symbols * sizeof(rmt_data_t), &receive_config);
  // wait for data if requested
  if (wait_for_data) {
    ret = (xEventGroupWaitBits(this->_rmt_bus->rmt_events, RMT_FLAG_RX_DONE, pdFALSE /* do not clear on exit */, pdFALSE /* wait for all bits */, timeout_ms)
               & RMT_FLAG_RX_DONE)
              != 0;
  }

  return ret;
}

bool IRrmt::readCompleted(){
  if (this->_rmt_bus == NULL) {
    return false;
  }
  if (this->_rmt_direction != RMT_RX_MODE) {
    return false;
  }

  bool ret = (xEventGroupGetBits(this->_rmt_bus->rmt_events) & RMT_FLAG_RX_DONE) != 0;
  return ret;
}

bool IRrmt::end() {
  if (this->_rmt_bus == NULL) return false;

  bool ret = true;
  ESP_LOGV(RMT_TAG, "Detaching RMT GPIO Bus");

  // free Event Group
  if (this->_rmt_bus->rmt_events != NULL) {
    vEventGroupDelete(this->_rmt_bus->rmt_events);
    this->_rmt_bus->rmt_events = NULL;
  }
  // deallocate the channel encoder
  if (this->_rmt_bus->rmt_copy_encoder_h != NULL) {
    if (ESP_OK != rmt_del_encoder(this->_rmt_bus->rmt_copy_encoder_h)) {
      ESP_LOGW(RMT_TAG, "RMT Encoder Deletion has failed.");
      ret = false;
    }
  }
  // disable and deallocate RMT channel
  if (this->_rmt_bus->rmt_channel_h != NULL) {
    // force stopping rmt TX/RX processing and unlock Power Management (APB Freq)
    rmt_disable(this->_rmt_bus->rmt_channel_h);
    if (ESP_OK != rmt_del_channel(this->_rmt_bus->rmt_channel_h)) {
      ESP_LOGW(RMT_TAG, "RMT Channel Deletion has failed.");
      ret = false;
    }
  }
  free(this->_rmt_bus);
  return ret;
}

#endif /* SOC_RMT_SUPPORTED */
#endif /* ESP_PLATFORM */