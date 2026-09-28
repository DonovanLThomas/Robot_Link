#pragma once
#include <stddef.h>

#ifdef IMU_WIFI
void teleop_udp_init(void);
void teleop_udp_send(const char *data, size_t length);
#else
static inline void teleop_udp_init(void) {}
static inline void teleop_udp_send(const char *data, size_t length) {
    (void)data;
    (void)length;
}
#endif
