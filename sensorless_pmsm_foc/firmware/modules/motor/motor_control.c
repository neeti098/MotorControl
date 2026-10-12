#include "motor_control.h"
#include "app_config.h"
#include "foc.h"
#include "motor_io.h"
#include <math.h>
#include <string.h>

#define MC_PI 3.14159265358979323846f
#define MC_TWO_PI (2.0f * MC_PI)
#define MC_FAST_DT (1.0f / (float)APP_PWM_FREQUENCY_HZ)
#define MC_ALIGN_SAMPLES (APP_ALIGNMENT_TIME_MS * APP_PWM_FREQUENCY_HZ / 1000U)
#define MC_RS_SAMPLES 256U
#define MC_LS_SETTLE_SAMPLES 32U
#define MC_LS_PULSE_SAMPLES 8U
#define MC_FLUX_SAMPLES 64U
#define MC_RS_FALLBACK 5.0f

typedef enum {
    COMM_IDLE,
    COMM_ALIGN_HOLD,
    COMM_RS_AVERAGE,
    COMM_LS_SETTLE,
    COMM_LS_PULSE,
    COMM_COMPLETE
} commission_stage_t;

typedef struct {
    volatile bool startup;
    volatile bool handover;
    volatile bool run;
    volatile bool idle;
} transition_requests_t;

static motor_snapshot_t s;
static foc_pi_t s_id_pi, s_iq_pi;
static foc_observer_t s_observer;
static motor_state_t s_state;
static motor_fault_t s_fault, s_pending_fault;
static motor_source_t s_speed_source;
static commission_stage_t s_commission;
static transition_requests_t s_transition;
static volatile bool s_start_request, s_stop_request;
static bool s_initialized, s_calibrated, s_reverse_pending;
static bool s_rs_measured, s_ls_measured, s_flux_measured, s_polarity_corrected;
static int s_direction;
static float s_reverse_target_rpm;
static float s_current_sign;
static float s_current_offset_u, s_current_offset_v;
static float s_id_reference, s_iq_reference;
static float s_forced_angle, s_forced_speed;
static float s_speed_integral;
static float s_rs_current_sum, s_rs_voltage_sum;
static float s_ls_start_current, s_ls_start_voltage, s_ls_current_sum;
static float s_flux_sum;
static uint32_t s_offset_sum_u, s_offset_sum_v, s_offset_count;
static uint32_t s_align_count, s_rs_count, s_ls_settle_count, s_ls_pulse_count;
static uint32_t s_flux_count, s_lock_count, s_unlock_count, s_invalid_count;
static uint32_t s_overrun_count;
static uint32_t s_state_started_ms, s_now_ms;
static uint32_t s_fallback_count;

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void latch_fault(motor_fault_t code)
{
    if (code != MOTOR_FAULT_NONE && s_pending_fault == MOTOR_FAULT_NONE) s_pending_fault = code;
}

static void enter_state(motor_state_t next)
{
    s_state = next;
    s_state_started_ms = s_now_ms;
    s_lock_count = 0U;
    s_unlock_count = 0U;
    if (next == MOTOR_ALIGN || next == MOTOR_STARTUP || next == MOTOR_STOPPING || next == MOTOR_IDLE) {
        s_id_pi.integral = 0.0f;
        s_iq_pi.integral = 0.0f;
        s_speed_integral = 0.0f;
    }
    if (next == MOTOR_ALIGN) {
        s_commission = COMM_ALIGN_HOLD;
        s_align_count = 0U;
        s_rs_count = 0U;
        s_ls_settle_count = 0U;
        s_ls_pulse_count = 0U;
        s_flux_count = 0U;
        s_rs_current_sum = 0.0f;
        s_rs_voltage_sum = 0.0f;
        s_ls_current_sum = 0.0f;
        s_flux_sum = 0.0f;
        s_rs_measured = false;
        s_ls_measured = false;
        s_flux_measured = false;
        s_polarity_corrected = false;
    } else if (next == MOTOR_STARTUP) {
        s_commission = COMM_COMPLETE;
        s_forced_angle = 0.0f;
        s_forced_speed = 0.0f;
        s_flux_sum = 0.0f;
        s_flux_count = 0U;
        foc_observer_init(&s_observer, s.flux_wb);
    } else if (next == MOTOR_IDLE || next == MOTOR_FAULT) {
        s_commission = COMM_IDLE;
    }
}

static void apply_pending_fault(void)
{
    if (s_pending_fault == MOTOR_FAULT_NONE) return;
    if (s_state != MOTOR_FAULT) {
        s_fault = s_pending_fault;
        enter_state(MOTOR_FAULT);
        motor_emergency_disable();
    }
    s_pending_fault = MOTOR_FAULT_NONE;
}

static float adc_current(uint16_t raw, float offset)
{
    float adc_v = ((float)raw - offset) * APP_ADC_REFERENCE_V / APP_ADC_MAX_COUNTS;
    return -s_current_sign * adc_v / APP_CURRENT_VOLTS_PER_AMP;
}

void motor_control_init(void)
{
    memset(&s, 0, sizeof(s));
    memset(&s_transition, 0, sizeof(s_transition));
    s_state = MOTOR_BOOT;
    s_fault = MOTOR_FAULT_NONE;
    s_pending_fault = MOTOR_FAULT_NONE;
    s_speed_source = MOTOR_SOURCE_CONSOLE;
    s_direction = 1;
    s_current_sign = APP_CURRENT_SIGN < 0.0f ? -1.0f : 1.0f;
    s.requested_rpm = APP_DEFAULT_SPEED_RPM;
    s.rs_ohm = MC_RS_FALLBACK;
    s.ls_h = APP_MOTOR_LS_HENRY;
    s.flux_wb = APP_MOTOR_FLUX_LINKAGE_WB_EST;
    s_id_pi = (foc_pi_t){APP_CURRENT_PI_KP, APP_CURRENT_PI_KI, 0.0f, APP_CURRENT_LIMIT_A};
    s_iq_pi = s_id_pi;
    s_initialized = false;
    s_calibrated = false;
    s_start_request = false;
    s_stop_request = false;
    s_reverse_pending = false;
    s_rs_measured = false;
    s_ls_measured = false;
    s_flux_measured = false;
    s_polarity_corrected = false;
    s_offset_sum_u = s_offset_sum_v = s_offset_count = 0U;
    s_now_ms = s_state_started_ms = 0U;
    s_align_count = s_rs_count = s_ls_settle_count = s_ls_pulse_count = 0U;
    s_flux_count = s_lock_count = s_unlock_count = s_invalid_count = s_overrun_count = 0U;
    s_fallback_count = 0U;
    foc_observer_init(&s_observer, s.flux_wb);
}

void motor_set_start(void) { s_start_request = true; }
void motor_set_stop(void) { s_stop_request = true; }

void motor_emergency_disable(void)
{
    const motor_io_t *io = motor_io();
    if (io != NULL && io->disable_bridge != NULL) io->disable_bridge();
}

void motor_set_speed(float rpm)
{
    if (!isfinite(rpm)) return;
    if (fabsf(rpm) < APP_SENSORLESS_MIN_RPM) {
        s.requested_rpm = 0.0f;
        return;
    }
    s.requested_rpm = clampf(rpm, -APP_SENSORLESS_MAX_RPM, APP_SENSORLESS_MAX_RPM);
}

bool motor_set_direction(int direction)
{
    if (direction != 1 && direction != -1) return false;
    if (s_state == MOTOR_ALIGN || s_state == MOTOR_STARTUP || s_state == MOTOR_HANDOVER || s_state == MOTOR_RUN) {
        s_reverse_pending = true;
        s_reverse_target_rpm = fabsf(s.requested_rpm) * (float)direction;
        s.requested_rpm = 0.0f;
        s_stop_request = true;
        return true;
    }
    s_direction = direction;
    return true;
}

bool motor_set_source(motor_source_t source)
{
    if (s_state != MOTOR_IDLE) return false;
    s_speed_source = source;
    return true;
}

bool motor_clear_fault(void)
{
    const motor_io_t *io = motor_io();
    if (s_state != MOTOR_FAULT || (io != NULL && io->driver_fault != NULL && io->driver_fault())) return false;
    if (s.bus_v < APP_UNDERVOLTAGE_TRIP_V || s.bus_v > APP_OVERVOLTAGE_TRIP_V) return false;
    s_fault = MOTOR_FAULT_NONE;
    enter_state(MOTOR_IDLE);
    return true;
}

void motor_control_set_resistance(float ohms, bool measured)
{
    if (isfinite(ohms) && ohms > 0.0f) { s.rs_ohm = ohms; s_rs_measured = measured; }
}
void motor_control_set_inductance(float henry, bool measured)
{
    if (isfinite(henry) && henry > 0.0f) { s.ls_h = henry; s_ls_measured = measured; }
}
void motor_control_set_flux(float wb, bool measured)
{
    if (isfinite(wb) && wb > 0.0f) { s.flux_wb = wb; s_flux_measured = measured; }
}

/* One ADC-pair callback invokes exactly one FOC loop and one coherent PWM update. */
void motor_control_fast_sample(uint16_t raw_u, uint16_t raw_v, uint16_t raw_w, bool valid)
{
    (void)raw_w;
    const motor_io_t *io = motor_io();
    uint32_t cycle_start = io != NULL && io->cycles != NULL ? io->cycles() : 0U;
    if (io != NULL && io->driver_fault != NULL && io->driver_fault()) latch_fault(MOTOR_FAULT_DRIVER);
    if (s_pending_fault != MOTOR_FAULT_NONE || s_state == MOTOR_FAULT) {
        motor_emergency_disable();
        return;
    }
    if (!valid) {
        s.current_valid = false;
        if (++s_invalid_count >= APP_INVALID_SAMPLE_LIMIT) latch_fault(MOTOR_FAULT_ADC);
        return;
    }
    s_invalid_count = 0U;
    if (s_state == MOTOR_CALIBRATION) {
        s_offset_sum_u += raw_u;
        s_offset_sum_v += raw_v;
        if (++s_offset_count >= APP_ADC_OFFSET_SAMPLES) {
            s_current_offset_u = (float)s_offset_sum_u / (float)s_offset_count;
            s_current_offset_v = (float)s_offset_sum_v / (float)s_offset_count;
            s_calibrated = s_current_offset_u >= APP_ADC_OFFSET_MIN_COUNTS && s_current_offset_u <= APP_ADC_OFFSET_MAX_COUNTS &&
                           s_current_offset_v >= APP_ADC_OFFSET_MIN_COUNTS && s_current_offset_v <= APP_ADC_OFFSET_MAX_COUNTS;
            if (s_calibrated) s_transition.idle = true;
            else latch_fault(MOTOR_FAULT_ADC);
        }
        return;
    }

    s.iu = adc_current(raw_u, s_current_offset_u);
    s.iv = adc_current(raw_v, s_current_offset_v);
    s.iw = -(s.iu + s.iv);
    if (!isfinite(s.iu) || !isfinite(s.iv) || !isfinite(s.iw)) { latch_fault(MOTOR_FAULT_ADC); return; }
    s.current_valid = true;
    if (fabsf(s.iu) > APP_CURRENT_LIMIT_A || fabsf(s.iv) > APP_CURRENT_LIMIT_A || fabsf(s.iw) > APP_CURRENT_LIMIT_A) {
        latch_fault(MOTOR_FAULT_OVERCURRENT);
        return;
    }
    if (s_state != MOTOR_ALIGN && s_state != MOTOR_STARTUP && s_state != MOTOR_HANDOVER &&
        s_state != MOTOR_RUN && s_state != MOTOR_STOPPING) return;

    if (io != NULL && io->read_bus_voltage != NULL) {
        float bus;
        if (io->read_bus_voltage(&bus) && isfinite(bus)) s.bus_v = bus;
    }
    if (s.bus_v < APP_UNDERVOLTAGE_TRIP_V) { latch_fault(MOTOR_FAULT_UNDERVOLTAGE); return; }
    if (s.bus_v > APP_OVERVOLTAGE_TRIP_V) { latch_fault(MOTOR_FAULT_OVERVOLTAGE); return; }

    foc_ab_t current = foc_clarke(s.iu, s.iv);
    float angle = (s_state == MOTOR_RUN || s_state == MOTOR_HANDOVER) ? s_observer.angle_rad : s_forced_angle;
    foc_dq_t measured = foc_park(current, angle);
    s.id = measured.d;
    s.iq = measured.q;
    s_id_ref = 0.0f;
    s_iq_ref = 0.0f;
    bool pulse_l_inductance_test = false;

    if (s_state == MOTOR_ALIGN) {
        s_commission = COMM_ALIGN;
        s_id_ref = APP_ALIGNMENT_CURRENT_A;
        if (s_align_count < MC_ALIGN_SAMPLES) {
            ++s_align_count;
        } else if (!s_polarity_corrected) {
            if (s.id < 0.0f) {
                s_current_sign = -s_current_sign;
                s_polarity_corrected = true;
                s_request_align_done = true;
                return;
            }
            s_polarity_corrected = true;
            s_commission = COMM_RS;
            s_rs_voltage_sum = 0.0f;
            s_rs_current_sum = 0.0f;
            s_rs_sample_count = 0U;
        } else if (s_commission == COMM_RS) {
            if (s_rs_sample_count < MC_RS_SAMPLES) {
                s_rs_voltage_sum += fabsf(s.vd);
                s_rs_current_sum += fabsf(s.id);
                ++s_rs_sample_count;
            } else {
                float mean_i = s_rs_current_sum / (float)s_rs_sample_count;
                float mean_v = s_rs_voltage_sum / (float)s_rs_sample_count;
                float rs = mean_v / fmaxf(mean_i, 0.05f);
                if (isfinite(rs) && rs >= APP_MOTOR_RS_MIN_OHM && rs <= APP_MOTOR_RS_MAX_OHM) {
                    s.rs_ohm = rs;
                    s_rs_measured = true;
                } else {
                    s.rs_ohm = APP_MOTOR_RS_PHASE_OHM;
                    s_rs_measured = false;
                }
                s_commission = COMM_LS_STEP;
                s_ls_i_start = s.id;
                s_ls_v_step = MC_LS_VOLTAGE_V;
                s_ls_samples = 0U;
            }
        } else if (s_commission == COMM_LS_STEP) {
            /* Apply a known small d-axis voltage vector for eight PWM intervals. */
            pulse_l_inductance_test = true;
            s_id_ref = 0.0f;
            if (++s_ls_samples >= MC_LS_STEP_SAMPLES) {
                float di_dt = (s.id - s_ls_i_start) / ((float)s_ls_samples * MC_FAST_DT);
                float measured_ls = s_ls_v_step / fmaxf(fabsf(di_dt), 0.001f);
                if (!isfinite(measured_ls) || measured_ls < APP_MOTOR_LS_MIN_HENRY || measured_ls > APP_MOTOR_LS_MAX_HENRY) {
                    latch_fault(MOTOR_FAULT_LS);
                    return;
                }
                s.ls_h = measured_ls;
                s_ls_measured = true;
                s_request_startup = true;
            }
        }
    } else if (s_state == MOTOR_STARTUP) {
        float target_speed = (float)s_direction * APP_SENSORLESS_MIN_RPM * TWO_PI_F *
                             (float)APP_MOTOR_POLE_PAIRS / 60.0f;
        float accel = APP_ACCELERATION_RPM_PER_S * TWO_PI_F * (float)APP_MOTOR_POLE_PAIRS / 60.0f;
        s_forced_speed += clampf(target_speed - s_forced_speed, -accel * MC_FAST_DT, accel * MC_FAST_DT);
        s_forced_angle = foc_wrap_angle(s_forced_angle + s_forced_speed * MC_FAST_DT);
        s_iq_ref = 0.25f * APP_CURRENT_LIMIT_A * (float)s_direction;
    } else if (s_state == MOTOR_HANDOVER) {
        float blend = clampf((float)(s_now_ms - s_state_started_ms) / (float)APP_HANDOVER_TIME_MS, 0.0f, 1.0f);
        float delta = s_observer.angle_rad - s_forced_angle;
        while (delta > PI_F) delta -= TWO_PI_F;
        while (delta < -PI_F) delta += TWO_PI_F;
        angle = foc_wrap_angle(s_forced_angle + blend * delta);
        s_iq_ref = 0.25f * APP_CURRENT_LIMIT_A * (float)s_direction;
    } else if (s_state == MOTOR_RUN) {
        angle = s_observer.angle_rad;
        s.estimated_rpm = s_observer.speed_rad_s * 60.0f / (TWO_PI_F * (float)APP_MOTOR_POLE_PAIRS);
        float error = s.ramped_rpm - s.estimated_rpm;
        float candidate_i = s_speed_integral + APP_SPEED_PI_KI * error * 0.001f;
        float iq = APP_SPEED_PI_KP * error + candidate_i;
        if (s.bus_v > APP_OVERVOLTAGE_TRIP_V - 0.5f && iq * s.estimated_rpm < 0.0f) iq = 0.0f;
        s_iq_ref = clampf(iq, -APP_CURRENT_LIMIT_A, APP_CURRENT_LIMIT_A);
        if (fabsf(iq) < APP_CURRENT_LIMIT_A) s_speed_integral = candidate_i;
        if (!s_observer.locked) {
            if (++s_unlock_count >= APP_OBSERVER_UNLOCK_PERSIST_MS * (APP_PWM_FREQUENCY_HZ / 1000U)) latch_fault(MOTOR_FAULT_OBSERVER);
        } else s_unlock_count = 0U;
    } else if (s_state == MOTOR_STOPPING) {
        /* Coast rather than drive negative torque into the non-regenerative adapter. */
        s_iq_ref = 0.0f;
    }

    foc_output_t output = {0};
    if (pulse_l_inductance_test) {
        foc_dq_t test_voltage = {s_ls_v_step, 0.0f};
        foc_ab_t alpha_beta = foc_inverse_park(test_voltage, s_forced_angle);
        output.voltage = test_voltage;
        output.duty = foc_svpwm(alpha_beta, s.bus_v, APP_PWM_MIN_DUTY, APP_PWM_MAX_DUTY);
        output.measured = measured;
        output.electrical_speed = 0.0f;
    } else {
        foc_input_t input = {s.iu, s.iv, s.iw, s.bus_v, s_id_ref, s_iq_ref,
                             angle, MC_FAST_DT, s.rs_ohm, s.ls_h, s.flux_wb};
        if (!foc_step(&input, &s_id_pi, &s_iq_pi, &output)) { latch_fault(MOTOR_FAULT_ADC); return; }
    }
    s.id_ref = s_id_ref;
    s.iq_ref = s_iq_ref;
    s.vd = output.voltage.d;
    s.vq = output.voltage.q;
    s.duty_u = output.duty.u;
    s.duty_v = output.duty.v;
    s.duty_w = output.duty.w;
    if (io != NULL && io->write_duties != NULL) io->write_duties(output.duty.u, output.duty.v, output.duty.w);
    if (!pulse_l_inductance_test) {
        foc_ab_t applied = foc_applied_voltage_from_duty(output.duty, s.bus_v);
        foc_observer_step(&s_observer, current, applied, s.rs_ohm, s.ls_h, s.flux_wb, MC_FAST_DT);
    }
    s.electrical_angle = angle;
    s.observer_confidence = s_observer.confidence;
    s.observer_locked = s_observer.locked;

    if (s_state == MOTOR_STARTUP && s_observer.speed_rad_s * (float)s_direction >
        (float)APP_SENSORLESS_MIN_RPM * TWO_PI_F * (float)APP_MOTOR_POLE_PAIRS / 60.0f && s_observer.bemf_magnitude > 0.2f) {
        float omega = fabsf(s_observer.speed_rad_s);
        float flux = (s_observer.bemf_magnitude - s.rs_ohm * s.iq - omega * s.ls_h * s.id) / fmaxf(omega, 1.0f);
        if (isfinite(flux) && flux >= APP_MOTOR_FLUX_MIN_WB && flux <= APP_MOTOR_FLUX_MAX_WB) {
            s_flux_sum += flux;
            if (++s_flux_samples >= FLUX_AVERAGE_TARGET) { s.flux_wb = s_flux_sum / (float)s_flux_samples; s_flux_measured = true; }
        } else if (s_now_ms - s_state_started_ms > APP_STARTUP_TIMEOUT_MS / 2U) latch_fault(MOTOR_FAULT_FLUX);
    }
    if (s_state == MOTOR_STARTUP && s_flux_measured && s_observer.locked) {
        if (++s_lock_samples >= APP_OBSERVER_LOCK_PERSIST_MS * (APP_PWM_FREQUENCY_HZ / 1000U)) s_request_handover = true;
    } else s_lock_samples = 0U;
    if (s_state == MOTOR_HANDOVER && s_now_ms - s_state_started_ms >= APP_HANDOVER_TIME_MS && s_observer.locked) s_request_run = true;
    if (s_state == MOTOR_STOPPING && (!s_observer.locked || fabsf(s_observer.speed_rad_s) < 25.0f)) s_request_idle = true;

    uint32_t elapsed = (io != NULL && io->cycles != NULL) ? io->cycles() - cycle_start : 0U;
    s.control_cycles = elapsed;
    if (io != NULL && io->cycle_budget != NULL && elapsed > io->cycle_budget(APP_FAST_LOOP_BUDGET_US)) {
        if (++s_overrun_count >= 3U) latch_fault(MOTOR_FAULT_OVERRUN);
    }
    s.overrun_count = s_overrun_count;
}

void motor_control_fast_sample_callback(const motor_io_current_sample_t *sample)
{
    if (sample == NULL || !sample->valid) { motor_control_fast_sample(0U, 0U, 0U, false); return; }
    motor_control_fast_sample(sample->phase_u, sample->phase_v, sample->phase_w, true);
}
void motor_fast_adc_event(void) { }
void motor_control_set_platform_time(uint32_t now) { s_now_ms = now; }
void motor_control_set_cycle_counter(uint32_t cycles) { s_cycle_count = cycles; }

void motor_supervisor_tick_1ms(void)
{
    const motor_io_t *io = motor_io();
    if (!s_hw_initialized) {
        if (io == NULL || io->initialize == NULL || !io->initialize()) latch_fault(MOTOR_FAULT_CONFIG);
        else s_hw_initialized = true;
    }
    ++s_now_ms;
    s.timestamp_ms = s_now_ms;
    if (s_state == MOTOR_BOOT) {
        if (io != NULL && io->disable_bridge != NULL) io->disable_bridge();
        enter_state(MOTOR_SELF_TEST);
    }
    if (s_state == MOTOR_SELF_TEST) {
        float bus = 0.0f;
        if (io == NULL || io->read_bus_voltage == NULL || !io->read_bus_voltage(&bus)) latch_fault(MOTOR_FAULT_ADC);
        else {
            s.bus_v = bus;
            if (bus < APP_UNDERVOLTAGE_TRIP_V || bus > APP_SELFTEST_OVERVOLTAGE_V ||
                (io->driver_fault != NULL && io->driver_fault())) latch_fault(MOTOR_FAULT_CONFIG);
            else {
                if (io->start_pwm != NULL) io->start_pwm();
                if (io->start_uart_rx != NULL) (void)io->start_uart_rx();
                enter_state(MOTOR_CALIBRATION);
            }
        }
    }
    if (s_state == MOTOR_CALIBRATION && s_now_ms - s_state_started_ms > 250U) latch_fault(MOTOR_FAULT_ADC);
    if (s_state == MOTOR_IDLE) {
        if (io != NULL && io->disable_bridge != NULL) io->disable_bridge();
        if (s_reverse_pending && (!s_observer.locked || fabsf(s_observer.speed_rad_s) < 25.0f)) {
            s_direction = s_reverse_target_rpm < 0.0f ? -1 : 1;
            s.requested_rpm = fabsf(s_reverse_target_rpm);
            s_reverse_pending = false;
        }
        if (s_start_request) {
            s_start_request = false;
            if (!s_calibrated) latch_fault(MOTOR_FAULT_CONFIG);
            else if (fabsf(s.requested_rpm) < APP_SENSORLESS_MIN_RPM) { }
            else if (io != NULL && io->enable_bridge != NULL && (io->driver_fault == NULL || !io->driver_fault())) {
                io->enable_bridge();
                enter_state(MOTOR_ALIGN);
            } else latch_fault(MOTOR_FAULT_DRIVER);
        }
    }
    if ((s_state == MOTOR_ALIGN || s_state == MOTOR_STARTUP || s_state == MOTOR_HANDOVER) &&
        s_now_ms - s_state_started_ms > APP_STARTUP_TIMEOUT_MS) latch_fault(MOTOR_FAULT_STARTUP_TIMEOUT);
    if (s_state == MOTOR_RUN) {
        float step = APP_ACCELERATION_RPM_PER_S * 0.001f;
        s.ramped_rpm += clampf(s.requested_rpm - s.ramped_rpm, -step, step);
    }
    if (s_stop_request && (s_state == MOTOR_RUN || s_state == MOTOR_STARTUP || s_state == MOTOR_HANDOVER || s_state == MOTOR_ALIGN)) {
        s_stop_request = false;
        s.requested_rpm = 0.0f;
        enter_state(MOTOR_STOPPING);
    }
    if (s_request_startup) { s_request_startup = false; enter_state(MOTOR_STARTUP); }
    if (s_request_handover) { s_request_handover = false; enter_state(MOTOR_HANDOVER); }
    if (s_request_run) { s_request_run = false; enter_state(MOTOR_RUN); }
    if (s_request_idle) {
        s_request_idle = false;
        if (io != NULL && io->disable_bridge != NULL) io->disable_bridge();
        enter_state(MOTOR_IDLE);
    }
    apply_fault_if_pending();
    s.mode = s_state == MOTOR_RUN ? MOTOR_MODE_CLOSED_LOOP :
        ((s_state == MOTOR_ALIGN || s_state == MOTOR_STARTUP || s_state == MOTOR_HANDOVER) ? MOTOR_MODE_FORCED : MOTOR_MODE_OFF);
}

void motor_get_snapshot(motor_snapshot_t *out)
{
    if (out == NULL) return;
    *out = s;
    out->fault = s_fault;
    out->rs_measured = s_rs_measured;
    out->ls_measured = s_ls_measured;
    out->flux_measured = s_flux_measured;
    out->polarity_corrected = s_polarity_corrected;
    out->estimated_rpm = s_observer.speed_rad_s * 60.0f / (TWO_PI_F * (float)APP_MOTOR_POLE_PAIRS);
}

const char *motor_state_name(motor_state_t value)
{
    static const char *const names[] = {"BOOT", "SELF_TEST", "CALIBRATION", "IDLE", "ALIGN", "STARTUP", "HANDOVER", "RUN", "STOPPING", "FAULT"};
    return (unsigned)value < (sizeof(names) / sizeof(names[0])) ? names[value] : "INVALID";
}
const char *motor_fault_name(motor_fault_t value)
{
    static const char *const names[] = {"NONE", "CONFIG", "DRIVER", "ADC", "OVERCURRENT", "UNDERVOLTAGE", "OVERVOLTAGE", "STARTUP_TIMEOUT", "OBSERVER_UNLOCK", "STALL", "CONTROL_OVERRUN", "RS_MEASUREMENT", "LS_MEASUREMENT", "FLUX_MEASUREMENT"};
    return (unsigned)value < (sizeof(names) / sizeof(names[0])) ? names[value] : "INVALID";
}
