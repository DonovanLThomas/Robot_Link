#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    if (cyw43_arch_init()) {
        printf("Wi-Fi initialization failed\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    uint8_t mac[6];
    if (cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, mac)) {
        printf("Could not read MAC address\n");
        cyw43_arch_deinit();
        return 1;
    }

    if (!(mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5])) {
        printf("MAC unavailable: Wi-Fi hardware did not initialize successfully\n");
        cyw43_arch_deinit();
        return 1;
    }

    while (true) {
        printf("Wi-Fi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        sleep_ms(2000);
    }
}
