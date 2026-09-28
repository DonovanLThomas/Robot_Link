#pragma once
// Copy to wifi_config.h (ignored by Git), edit, then rebuild.
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define WIFI_TCP_PORT 4242

// Optional SO-101 teleoperation stream. Copy this file to wifi_config.h,
// set TELEOP_UDP_ENABLED to 1, and point JETSON_IP at the Jetson.
#define TELEOP_UDP_ENABLED 0
#define JETSON_IP "192.168.1.100"
#define JETSON_UDP_PORT 5005
#define TELEOP_SEND_PERIOD_MS 25
