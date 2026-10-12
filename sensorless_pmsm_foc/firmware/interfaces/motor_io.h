#ifndef MOTOR_IO_H
#define MOTOR_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t phase_u;
    uint16_t phase_v;
    uint16_t phase_w;
    uint32_t timestamp_us;
    bool valid;
} motor_io_current_sample_t;

typedef struct {
    bool (*initialize)(void);
    void (*start_pwm)(void);
    void (*enable_bridge)(void);
    void (*disable_bridge)(void);
    void (*write_duties)(float u, float v, float w);
    bool (*driver_fault)(void);
    bool (*button_pressed)(void);
    void (*led_write)(bool on);
    bool (*read_current_sample)(motor_io_current_sample_t *sample);
    bool (*read_bus_voltage)(float *volts);
    bool (*read_potentiometer)(uint16_t *raw);
    bool (*read_temperature)(uint16_t *raw);
    bool (*start_uart_rx)(void);
    bool (*uart_rx_pop)(uint8_t *byte);
    bool (*uart_tx)(const uint8_t *data, size_t length);
    uint32_t (*cycles)(void);
    uint32_t (*cycle_budget)(uint32_t microseconds);
    uint32_t (*micros)(void);
} motor_io_t;

void motor_io_bind(const motor_io_t *io);
const motor_io_t *motor_io(void);
void motor_io_fast_sample_ready(void);
bool motor_io_get_current_sample(motor_io_current_sample_t *sample);
void motor_io_clear_current_sample(void);

typedef void (*motor_io_sample_callback_t)(const motor_io_current_sample_t *sample);
void motor_io_set_sample_callback(motor_io_sample_callback_t callback);

#endif
