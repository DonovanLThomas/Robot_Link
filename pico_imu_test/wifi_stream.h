#pragma once
#include <stddef.h>
#ifdef IMU_WIFI
void wifi_stream_init(void);
void wifi_stream_poll(void);
void wifi_stream_status(void);
void wifi_stream_send(const char *data, size_t length);
#else
static inline void wifi_stream_init(void) {}
static inline void wifi_stream_poll(void) {}
static inline void wifi_stream_status(void) {}
static inline void wifi_stream_send(const char *data, size_t length) {
    (void)data;
    (void)length;
}
#endif
