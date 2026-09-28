#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include "wifi_build_config.h"
#include "teleop_udp.h"

static struct udp_pcb *teleop_pcb;
static ip_addr_t jetson_addr;
static bool teleop_ready;

void teleop_udp_init(void) {
#if TELEOP_UDP_ENABLED
    if (!WIFI_SSID[0]) {
        printf("Teleop UDP disabled: Wi-Fi credentials are not configured.\n");
        return;
    }
    if (!ipaddr_aton(JETSON_IP, &jetson_addr)) {
        printf("Teleop UDP disabled: invalid JETSON_IP \"%s\".\n", JETSON_IP);
        return;
    }
    teleop_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (!teleop_pcb) {
        printf("Teleop UDP disabled: could not allocate UDP PCB.\n");
        return;
    }
    teleop_ready = true;
    printf("Teleop UDP target: %s:%d\n", JETSON_IP, JETSON_UDP_PORT);
#else
    printf("Teleop UDP disabled: set TELEOP_UDP_ENABLED to 1 in wifi_config.h.\n");
#endif
}

void teleop_udp_send(const char *data, size_t length) {
#if TELEOP_UDP_ENABLED
    if (!teleop_ready || !teleop_pcb || !data || length == 0) return;
    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) return;

    struct pbuf *packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)length, PBUF_RAM);
    if (!packet) return;
    memcpy(packet->payload, data, length);
    udp_sendto(teleop_pcb, packet, &jetson_addr, JETSON_UDP_PORT);
    pbuf_free(packet);
#else
    (void)data;
    (void)length;
#endif
}
