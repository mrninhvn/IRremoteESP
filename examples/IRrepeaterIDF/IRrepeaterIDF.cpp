#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

#include "IRremoteESP8266.h"
#include "IRsend.h"
#include <IRrecv.h>
#include <IRutils.h>

const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).

IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.

const uint16_t kRecvPin = 3;
// As this program is a special purpose capture/resender, let's use a larger
// than expected buffer so we can handle very large IR messages.
const uint16_t kCaptureBufferSize = 1024;  // 1024 == ~511 bits

// kTimeout is the Nr. of milli-Seconds of no-more-data before we consider a
// message ended.
const uint8_t kTimeout = 50;  // Milli-Seconds

// kFrequency is the modulation frequency all UNKNOWN messages will be sent at.
const uint16_t kFrequency = 38000;  // in Hz. e.g. 38kHz.

// The IR receiver.
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false);
// Somewhere to store the captured message.
decode_results results;

static void decode_cb(void){
    decode_type_t protocol = results.decode_type;
    ESP_LOGI("DECODE Callback", "Decoded protocol: %s", typeToString(protocol).c_str());

    uint16_t size = results.bits;
    bool success = true;
    // Is it a protocol we don't understand?
    if (protocol == decode_type_t::UNKNOWN) {  // Yes.
    // Convert the results into an array suitable for sendRaw().
    // resultToRawArray() allocates the memory we need for the array.
    uint16_t *raw_array = resultToRawArray(&results);
    // Find out how many elements are in the array.
    size = getCorrectedRawLength(&results);
    #if SEND_RAW
    // Send it out via the IR LED circuit.
    irsend.sendRaw(raw_array, size, kFrequency);
    #endif  // SEND_RAW
    // Deallocate the memory allocated by resultToRawArray().
    delete [] raw_array;
    } else if (hasACState(protocol)) {  // Does the message require a state[]?
    // It does, so send with bytes instead.
    success = irsend.send(protocol, results.state, size / 8);
    } else {  // Anything else must be a simple message protocol. ie. <= 64 bits
    success = irsend.send(protocol, results.value, size);
    }
    // Resume capturing IR messages. It was not restarted until after we sent
    // the message so we didn't capture our own message.
    #ifndef ESP32_RMT
    irrecv.resume();
    #endif

    // Display a crude timestamp & notification.
    ESP_LOGI("IR", "A %d-bit %s message was %ssuccessfully retransmitted.\n",
                    size, typeToString(protocol).c_str(), success ? "" : "un");
}

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%luMB %s flash\n", flash_size / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %ld bytes\n", esp_get_minimum_free_heap_size());

    /* IR Demo */
    vTaskDelay(5000/portTICK_PERIOD_MS);
    irsend.begin();
    irrecv.enableIRIn();
    irrecv.enableDecodeLoop(&results, &decode_cb);
}