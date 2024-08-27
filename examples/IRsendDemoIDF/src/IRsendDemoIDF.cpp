#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "IRremoteESP8266.h"
#include "IRsend.h"
#include "IRac.h"

const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).

IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.
IRac ac(kIrLed);  // Create a A/C object using GPIO to sending messages with.

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

    /* IR Send Demo */
    irsend.begin();

    ac.next.protocol = decode_type_t::DAIKIN176;  // Set a protocol to use.
    ac.next.model = 1;  // Some A/Cs have different models. Try just the first.
    ac.next.mode = stdAc::opmode_t::kCool;  // Run in cool mode initially.
    ac.next.celsius = true;  // Use Celsius for temp units. False = Fahrenheit
    ac.next.degrees = 28;  // 25 degrees.
    ac.next.fanspeed = stdAc::fanspeed_t::kLow;  // Start the fan at medium.
    ac.next.swingv = stdAc::swingv_t::kOff;  // Don't swing the fan up or down.
    ac.next.swingh = stdAc::swingh_t::kOff;  // Don't swing the fan left or right.
    ac.next.light = false;  // Turn off any LED/Lights/Display that we can.
    ac.next.beep = false;  // Turn off any beep from the A/C if we can.
    ac.next.econo = false;  // Turn off any economy modes if we can.
    ac.next.filter = false;  // Turn off any Ion/Mold/Health filters if we can.
    ac.next.turbo = false;  // Don't use any turbo/powerful/etc modes.
    ac.next.quiet = false;  // Don't use any quiet/silent/etc modes.
    ac.next.sleep = -1;  // Don't set any sleep time or modes.
    ac.next.clean = false;  // Turn off any Cleaning options if we can.
    ac.next.clock = -1;  // Don't set any current time if we can avoid it.
    ac.next.power = false;  // Initially start with the unit off.

    vTaskDelay(5000/portTICK_PERIOD_MS);
    while (1){
        // printf("irsend.sendSymphony\n");
        // irsend.sendSymphony(0xDC3);
        // vTaskDelay(1000);
        // ac.next.power = !ac.next.power;  // We want to turn on the A/C unit.
        // printf("Sending a message to the A/C unit\n");
        // ac.sendAc();  // Have the IRac class create and send a message.
        // vTaskDelay(5000/portTICK_PERIOD_MS);
        // For every protocol the library has ...
        for (int i = 125; i < kLastDecodeType; i++) {
            decode_type_t protocol = (decode_type_t)i;
            // If the protocol is supported by the IRac class ...
            if (ac.isProtocolSupported(protocol)) {
                printf("\nProtocol ");
                printf(typeToString(protocol).c_str()); printf(" is supported.\n");
                // printf("%d", protocol); printf(" is supported.");
                ac.next.protocol = protocol;  // Change the protocol used.
                ac.next.power = true;  // We want to turn on the A/C unit.
                printf("Sending a message to turn ON the A/C unit.\n");
                ac.sendAc();  // Have the IRac class create and send a message.
                vTaskDelay(2000/portTICK_PERIOD_MS);  // Wait 5 seconds.
                // ac.next.power = false;  // Now we want to turn the A/C off.
                // printf("Send a message to turn OFF the A/C unit.\n");
                // ac.sendAc();  // Send the message.
                vTaskDelay(1000/portTICK_PERIOD_MS);  // Wait 1 second.
                printf("Free heap size: %ld bytes\n", esp_get_free_heap_size());

                printf(typeToString(protocol).c_str());
                printf(" end ----------------------------------\n\n");
            }
        }
        printf("Starting from the begining again ...\n");
    }
}