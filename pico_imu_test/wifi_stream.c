#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "wifi_build_config.h"
#include "wifi_stream.h"

static struct tcp_pcb *listener, *client;
static bool enabled, connected;
static absolute_time_t retry_at;

static void report_link_status(int status) {
    switch (status) {
        case CYW43_LINK_NONET:
            printf("Wi-Fi network not found. Check hotspot name, 2.4 GHz mode, and range.\n");
            break;
        case CYW43_LINK_BADAUTH:
            printf("Wi-Fi authentication failed. Check password and WPA2 hotspot compatibility.\n");
            break;
        case CYW43_LINK_FAIL:
            printf("Wi-Fi connection failed. Check hotspot settings and try restarting the hotspot.\n");
            break;
        case CYW43_LINK_JOIN:
        case CYW43_LINK_NOIP:
            printf("Wi-Fi joined; waiting for an IP address from the hotspot/router (DHCP).\n");
            break;
        case CYW43_LINK_UP:
            printf("Wi-Fi ready: %s TCP port %d\n",
                   ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])),
                   WIFI_TCP_PORT);
            if (!listener) printf("TCP server not ready yet; retrying server setup.\n");
            break;
        default:
            printf("Wi-Fi not connected (status %d); connection attempts repeat every 30 seconds.\n", status);
            break;
    }
}

static void drop_client(void) {
    if (client) {
        tcp_arg(client, NULL);
        tcp_err(client, NULL);
        tcp_recv(client, NULL);
        tcp_abort(client);
        client = NULL;
    }
}

static void client_error(void *arg, err_t err) {
    (void)arg;
    (void)err;
    client = NULL; // lwIP has already freed the PCB.
}

static err_t receive(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    (void)arg;
    (void)err;
    if (!p) {
        drop_client();
        return ERR_ABRT;
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t accept_client(void *arg, struct tcp_pcb *pcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || !pcb) return err;
    if (client) { // One viewer at a time; preserve the existing session.
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    client = pcb;
    tcp_recv(client, receive);
    tcp_err(client, client_error);
    tcp_nagle_disable(client);
    return ERR_OK;
}

static bool start_server(void) {
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) return false;
    if (tcp_bind(pcb, IP_ANY_TYPE, WIFI_TCP_PORT) != ERR_OK) {
        tcp_close(pcb);
        return false;
    }
    listener = tcp_listen_with_backlog(pcb, 1);
    if (!listener) {
        tcp_close(pcb);
        return false;
    }
    tcp_accept(listener, accept_client);
    return true;
}

void wifi_stream_init(void) {
    if (!WIFI_SSID[0]) {
        printf("Wi-Fi disabled: set wifi_config.h and rebuild.\n");
        return;
    }
    if (cyw43_arch_init()) {
        printf("Wi-Fi initialization failed; USB remains available.\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    enabled = true;
    retry_at = get_absolute_time();
}

void wifi_stream_status(void) {
    if (!WIFI_SSID[0]) {
        printf("Wi-Fi disabled: set wifi_config.h and rebuild.\n");
    } else if (!enabled) {
        printf("Wi-Fi initialization failed; restart the board to retry.\n");
    } else {
        report_link_status(cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA));
    }
}

void wifi_stream_poll(void) {
    static int last_status = 999;
    if (!enabled) return;
    cyw43_arch_poll();
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (status != last_status) {
        if (status != CYW43_LINK_UP) report_link_status(status);
        last_status = status;
    }
    if (status == CYW43_LINK_UP) {
        if (!listener && !start_server()) return;
        if (!connected) {
            printf("Wi-Fi ready: %s TCP port %d\n",
                   ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])),
                   WIFI_TCP_PORT);
            connected = true;
        }
    } else {
        if (connected) {
            printf("Wi-Fi disconnected; retrying.\n");
            drop_client();
            connected = false;
            retry_at = get_absolute_time();
        }
        if (time_reached(retry_at)) {
            printf("Connecting to Wi-Fi...\n");
            int result = cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                WIFI_PASSWORD[0] ? CYW43_AUTH_WPA2_AES_PSK : CYW43_AUTH_OPEN);
            if (result) printf("Wi-Fi connection request failed: %d\n", result);
            retry_at = make_timeout_time_ms(30000);
        }
    }
}

void wifi_stream_send(const char *data, size_t length) {
    if (!client || !connected) return;
    // Never stall sensor acquisition or deliver a partial record to a slow client.
    if (length > tcp_sndbuf(client) ||
        tcp_write(client, data, (u16_t)length, TCP_WRITE_FLAG_COPY) != ERR_OK) {
        drop_client();
        return;
    }
    err_t result = tcp_output(client);
    if (result != ERR_OK && result != ERR_MEM) drop_client();
}
