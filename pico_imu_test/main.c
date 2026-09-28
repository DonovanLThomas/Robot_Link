#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "wifi_stream.h"
#include "teleop_udp.h"
#ifdef IMU_WIFI
#include "wifi_build_config.h"
#endif

#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

#define MUX_ADDRESS 0x70
#define IMU_ADDRESS 0x68
#define TIMEOUT_US 10000
#define RAW_TEXT_PERIOD_MS 500

#ifndef TELEOP_SEND_PERIOD_MS
#define TELEOP_SEND_PERIOD_MS 25
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SHOULDER_IMU_CHANNEL 1
#define UPPER_ARM_IMU_CHANNEL 0
#define FOREARM_IMU_CHANNEL 2
#define IMU_CHANNEL_COUNT 3

static bool usb_streaming = false;

static void service_connections(void) {
    static bool usb_was_connected = false;
    wifi_stream_poll();
    bool usb_connected = stdio_usb_connected();
    if (usb_connected && !usb_was_connected) {
        printf("\nUSB raw IMU output %s. Teleop JSON enabled. Commands: s = stream, p = pause, w = Wi-Fi status.\n",
               usb_streaming ? "running" : "paused");
        wifi_stream_status();
    }
    usb_was_connected = usb_connected;
    // Nonblocking: networking and sensor acquisition continue while paused.
    int command = getchar_timeout_us(0);
    if (command == 'p' || command == 'P') {
        usb_streaming = false;
        printf("USB IMU output paused. Press s to resume, w for Wi-Fi status.\n");
    } else if (command == 's' || command == 'S') {
        usb_streaming = true;
        printf("USB IMU output resumed. Press p to pause.\n");
    } else if (command == 'w' || command == 'W') {
        wifi_stream_status();
    }
}

// MPU6050 registers
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;
} IMUData;

typedef struct {
    float shoulder_pan;
    float shoulder_lift;
    float elbow_flex;
    float wrist_flex;
    float wrist_roll;
} HumanJointState;

static float radians_to_degrees(float radians) {
    return radians * 180.0f / (float)M_PI;
}

static float vector_norm3(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

static float angle_between3(float ax, float ay, float az, float bx, float by, float bz) {
    float a_norm = vector_norm3(ax, ay, az);
    float b_norm = vector_norm3(bx, by, bz);
    if (a_norm < 1e-6f || b_norm < 1e-6f) return 0.0f;

    float cosine = (ax * bx + ay * by + az * bz) / (a_norm * b_norm);
    if (cosine > 1.0f) cosine = 1.0f;
    if (cosine < -1.0f) cosine = -1.0f;
    return radians_to_degrees(acosf(cosine));
}

static float accel_pitch_degrees(const IMUData *imu) {
    return radians_to_degrees(atan2f(-imu->accel_x,
        sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z)));
}

static float accel_roll_degrees(const IMUData *imu) {
    return radians_to_degrees(atan2f(imu->accel_y, imu->accel_z));
}

static HumanJointState estimate_human_joints_from_current_samples(
    const IMUData *shoulder,
    const IMUData *upper_arm,
    const IMUData *forearm
) {
    float shoulder_pitch = accel_pitch_degrees(shoulder);
    float upper_pitch = accel_pitch_degrees(upper_arm);
    float forearm_pitch = accel_pitch_degrees(forearm);
    float upper_roll = accel_roll_degrees(upper_arm);
    float forearm_roll = accel_roll_degrees(forearm);

    // This is an initial teleop packet contract, not full orientation fusion.
    // MPU6050 accel tilt cannot observe yaw, so shoulder pan is held neutral.
    // The elbow field preserves the existing triangle-angle experiment by
    // measuring the angle at IMU 2 between the three acceleration endpoints.
    HumanJointState joints = {
        .shoulder_pan = 0.0f,
        .shoulder_lift = upper_pitch - shoulder_pitch,
        .elbow_flex = angle_between3(
            shoulder->accel_x - upper_arm->accel_x,
            shoulder->accel_y - upper_arm->accel_y,
            shoulder->accel_z - upper_arm->accel_z,
            forearm->accel_x - upper_arm->accel_x,
            forearm->accel_y - upper_arm->accel_y,
            forearm->accel_z - upper_arm->accel_z
        ),
        .wrist_flex = forearm_pitch - upper_pitch,
        .wrist_roll = forearm_roll - upper_roll,
    };
    return joints;
}

static void send_teleop_packet(const HumanJointState *joints) {
    static uint32_t sequence = 0;
    char packet[256];
    int length = snprintf(packet, sizeof(packet),
        "{\"seq\":%lu,\"timestamp_ms\":%llu,"
        "\"shoulder_pan\":%.3f,\"shoulder_lift\":%.3f,"
        "\"elbow_flex\":%.3f,\"wrist_flex\":%.3f,\"wrist_roll\":%.3f}\n",
        (unsigned long)sequence++,
        (unsigned long long)to_ms_since_boot(get_absolute_time()),
        joints->shoulder_pan,
        joints->shoulder_lift,
        joints->elbow_flex,
        joints->wrist_flex,
        joints->wrist_roll);

    if (length > 0 && (size_t)length < sizeof(packet)) {
        if (stdio_usb_connected()) printf("%s", packet);
        teleop_udp_send(packet, (size_t)length);
    }
}

static bool mux_select_channel(uint8_t channel) {
    if (channel > 7) {
        return false;
    }

    // Enable only one channel. The other IMUs are disconnected from I2C,
    // so all three can safely use the same address, 0x68.
    uint8_t mask = (uint8_t)(1u << channel);

    int result = i2c_write_timeout_us(
        I2C_PORT,
        MUX_ADDRESS,
        &mask,
        1,
        false,
        TIMEOUT_US
    );

    return result == 1;
}

static bool imu_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};

    int result = i2c_write_timeout_us(
        I2C_PORT,
        IMU_ADDRESS,
        buffer,
        2,
        false,
        TIMEOUT_US
    );

    return result == 2;
}

static bool imu_read_registers(uint8_t start_reg, uint8_t *buffer, size_t length) {

    int result = i2c_write_timeout_us(
        I2C_PORT,
        IMU_ADDRESS,
        &start_reg,
        1,
        true,   // repeated start
        TIMEOUT_US
    );

    if (result != 1) {
        return false;
    }

    result = i2c_read_timeout_us(
        I2C_PORT,
        IMU_ADDRESS,
        buffer,
        length,
        false,
        TIMEOUT_US
    );

    return result == (int)length;
}

static int16_t combine_bytes(uint8_t high, uint8_t low) {
    // Shift the high byte into bits 15..8 and put the low byte in bits 7..0.
    // int16_t interprets the combined bits as a signed sensor value on Pico.
    return (int16_t)(((uint16_t)high << 8) | low);
}

bool read_imu(uint8_t channel, IMUData *imu) {
    if (imu == NULL || channel >= IMU_CHANNEL_COUNT) {
        return false;
    }

    // Select before every read because the previous read may have used
    // a different IMU. Only the selected channel can respond at 0x68.
    if (!mux_select_channel(channel)) {
        return false;
    }

    uint8_t data[14];
    if (!imu_read_registers(ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    int16_t accel_x_raw = combine_bytes(data[0], data[1]);
    int16_t accel_y_raw = combine_bytes(data[2], data[3]);
    int16_t accel_z_raw = combine_bytes(data[4], data[5]);
    int16_t temp_raw = combine_bytes(data[6], data[7]);
    int16_t gyro_x_raw = combine_bytes(data[8], data[9]);
    int16_t gyro_y_raw = combine_bytes(data[10], data[11]);
    int16_t gyro_z_raw = combine_bytes(data[12], data[13]);

    // Keep the original default ranges: +/-2 g and +/-250 degrees/second.
    // The -> operator stores a value in the struct supplied by the caller.
    imu->accel_x = accel_x_raw / 16384.0f;
    imu->accel_y = accel_y_raw / 16384.0f;
    imu->accel_z = accel_z_raw / 16384.0f;
    imu->gyro_x = gyro_x_raw / 131.0f;
    imu->gyro_y = gyro_y_raw / 131.0f;
    imu->gyro_z = gyro_z_raw / 131.0f;
    imu->temperature = temp_raw / 340.0f + 36.53f;

    return true;
}

static void print_imu(uint8_t channel, const IMUData *imu, bool success) {
    char record[384];
    int length = snprintf(record, sizeof(record), "IMU %u - Channel %u\n",
           (unsigned int)channel + 1u, (unsigned int)channel);

    if (!success) {
        length += snprintf(record + length, sizeof(record) - length,
               "ERROR: Channel %u failed initialization or could not be read.\n\n",
               (unsigned int)channel);
    } else {
        length += snprintf(record + length, sizeof(record) - length,
            "Accel: X=%7.3f Y=%7.3f Z=%7.3f g\n"
            "Gyro : X=%7.2f Y=%7.2f Z=%7.2f deg/s\n"
            "Temp : %7.2f C\n\n",
            imu->accel_x, imu->accel_y, imu->accel_z,
            imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->temperature);
    }
    if (length > 0 && (size_t)length < sizeof(record)) {
        if (usb_streaming) printf("%s", record);
        wifi_stream_send(record, (size_t)length);
    }
}

int main(void) {
    stdio_init_all();

    // Start even when powered by a charger with no USB serial terminal.
    sleep_ms(500);
    wifi_stream_init();
    teleop_udp_init();

    i2c_init(I2C_PORT, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("\nTHREE MPU6050 IMU TEST\n");
    printf("I2C0 initialized: SDA = GP%d, SCL = GP%d\n\n", SDA_PIN, SCL_PIN);
    printf("Teleop roles: shoulder=channel %u, upper_arm=channel %u, forearm=channel %u.\n",
           (unsigned int)SHOULDER_IMU_CHANNEL,
           (unsigned int)UPPER_ARM_IMU_CHANNEL,
           (unsigned int)FOREARM_IMU_CHANNEL);

    bool initialized[IMU_CHANNEL_COUNT] = {false};

    // Each MPU6050 must be woken while its own mux channel is selected.
    for (uint8_t channel = 0; channel < IMU_CHANNEL_COUNT; channel++) {
        if (!mux_select_channel(channel)) {
            printf("ERROR: Could not select mux channel %u.\n",
                   (unsigned int)channel);
            continue;
        }

        if (!imu_write_register(PWR_MGMT_1, 0x00)) {
            printf("ERROR: Could not wake IMU on channel %u.\n",
                   (unsigned int)channel);
            continue;
        }

        initialized[channel] = true;
        printf("IMU %u initialized on channel %u.\n",
               (unsigned int)channel + 1u, (unsigned int)channel);
    }

    // Allow all successfully awakened sensors time to start.
    sleep_ms(100);

    IMUData imu_by_channel[IMU_CHANNEL_COUNT] = {0};

    absolute_time_t next_sample = get_absolute_time();
    absolute_time_t next_raw_text = get_absolute_time();

    while (true) {
        service_connections();
        if (!time_reached(next_sample)) {
            sleep_ms(1);
            continue;
        }

        // Each read has its own result; a failure does not skip other IMUs.
        // Do not report samples from an IMU that failed to wake at startup.
        bool ok_by_channel[IMU_CHANNEL_COUNT];
        for (uint8_t channel = 0; channel < IMU_CHANNEL_COUNT; channel++) {
            ok_by_channel[channel] = initialized[channel] &&
                read_imu(channel, &imu_by_channel[channel]);
        }

        if (ok_by_channel[SHOULDER_IMU_CHANNEL] &&
            ok_by_channel[UPPER_ARM_IMU_CHANNEL] &&
            ok_by_channel[FOREARM_IMU_CHANNEL]) {
            HumanJointState joints =
                estimate_human_joints_from_current_samples(
                    &imu_by_channel[SHOULDER_IMU_CHANNEL],
                    &imu_by_channel[UPPER_ARM_IMU_CHANNEL],
                    &imu_by_channel[FOREARM_IMU_CHANNEL]);
            send_teleop_packet(&joints);
        }

        if (time_reached(next_raw_text)) {
            if (usb_streaming) printf("========================\n");
            for (uint8_t channel = 0; channel < IMU_CHANNEL_COUNT; channel++) {
                print_imu(channel, &imu_by_channel[channel], ok_by_channel[channel]);
            }
            if (usb_streaming) printf("========================\n");
            next_raw_text = make_timeout_time_ms(RAW_TEXT_PERIOD_MS);
        }

        next_sample = make_timeout_time_ms(TELEOP_SEND_PERIOD_MS);
    }
}
