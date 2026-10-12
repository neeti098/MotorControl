#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_io.h"

typedef enum {
    MOTOR_BOOT, MOTOR_SELF_TEST, MOTOR_CALIBRATION, MOTOR_IDLE, MOTOR_ALIGN,
    MOTOR_STARTUP, MOTOR_HANDOVER, MOTOR_RUN, MOTOR_STOPPING, MOTOR_FAULT
} motor_state_t;

typedef enum {
    MOTOR_FAULT_NONE, MOTOR_FAULT_CONFIG, MOTOR_FAULT_DRIVER, MOTOR_FAULT_ADC,
    MOTOR_FAULT_OVERCURRENT, MOTOR_FAULT_UNDERVOLTAGE, MOTOR_FAULT_OVERVOLTAGE,
    MOTOR_FAULT_STARTUP_TIMEOUT, MOTOR_FAULT_OBSERVER, MOTOR_FAULT_STALL,
    MOTOR_FAULT_OVERRUN, MOTOR_FAULT_RS, MOTOR_FAULT_LS, MOTOR_FAULT_FLUX
} motor_fault_t;

typedef enum { MOTOR_MODE_OFF, MOTOR_MODE_FORCED, MOTOR_MODE_CLOSED_LOOP } motor_mode_t;
typedef enum { MOTOR_SOURCE_CONSOLE, MOTOR_SOURCE_POT } motor_source_t;

typedef struct {
    uint32_t timestamp_ms;
    motor_state_t state;
    motor_mode_t mode;
    motor_mode_t control_mode;
    motor_fault_t fault;
    float requested_rpm, ramped_rpm, estimated_rpm;
    float electrical_angle, bus_v, iu, iv, iw, id, iq, id_ref, iq_ref;
    float vd, vq, duty_u, duty_v, duty_w, flux_wb, rs_ohm, ls_h;
    float observer_confidence;
    uint32_t control_cycles, overrun_count;
    bool current_valid, observer_locked, rs_measured, ls_measured, flux_measured;
    bool polarity_corrected;
} motor_snapshot_t;

void motor_control_init(void);
void motor_supervisor_tick_1ms(void);
void motor_control_fast_sample(uint16_t iu_raw, uint16_t iv_raw, uint16_t iw_raw, bool valid);
void motor_control_fast_sample_callback(const motor_io_current_sample_t *sample);
void motor_control_set_resistance(float ohms, bool measured);
void motor_control_set_inductance(float henry, bool measured);
void motor_control_set_flux(float wb, bool measured);
void motor_fast_adc_event(void);
void motor_control_set_platform_time(uint32_t now_ms);
void motor_control_set_cycle_counter(uint32_t cycles);
void motor_set_start(void);
void motor_set_stop(void);
void motor_emergency_disable(void);
void motor_set_speed(float rpm);
bool motor_set_direction(int direction);
bool motor_set_source(motor_source_t source);
bool motor_clear_fault(void);
void motor_get_snapshot(motor_snapshot_t *out);
const char *motor_state_name(motor_state_t state);
const char *motor_fault_name(motor_fault_t fault);
#endif
