#include "foc.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

#define FOC_PI 3.14159265358979323846f
#define FOC_TWO_PI (2.0f * FOC_PI)
#define FOC_SQRT3 1.7320508075688772935f

float foc_wrap_angle(float angle)
{
    if (!isfinite(angle)) return 0.0f;
    while (angle >= FOC_TWO_PI) angle -= FOC_TWO_PI;
    while (angle < 0.0f) angle += FOC_TWO_PI;
    return angle;
}

/* Amplitude-invariant Clarke transform; phase order U,V,W; W is reconstructed from KCL. */
foc_ab_t foc_clarke(float iu, float iv)
{
    foc_ab_t out = {iu, (iu + 2.0f * iv) / FOC_SQRT3};
    return out;
}

/* Electrical angle increases in the U->V->W positive sequence. */
foc_dq_t foc_park(foc_ab_t ab, float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    foc_dq_t out = {ab.alpha * c + ab.beta * s, -ab.alpha * s + ab.beta * c};
    return out;
}

foc_ab_t foc_inverse_park(foc_dq_t dq, float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    foc_ab_t out = {dq.d * c - dq.q * s, dq.d * s + dq.q * c};
    return out;
}

float foc_pi_step(foc_pi_t *pi, float error, float dt, float output_limit)
{
    if (pi == NULL || !isfinite(error) || !isfinite(dt) || !isfinite(output_limit) ||
        dt <= 0.0f || output_limit < 0.0f) return 0.0f;
    float proportional = pi->kp * error;
    float candidate_integral = pi->integral + pi->ki * error * dt;
    float raw = proportional + candidate_integral;
    float saturated = raw;
    if (saturated > output_limit) saturated = output_limit;
    if (saturated < -output_limit) saturated = -output_limit;
    if (raw == saturated || (raw > output_limit && error < 0.0f) ||
        (raw < -output_limit && error > 0.0f)) pi->integral = candidate_integral;
    if (pi->integral > output_limit) pi->integral = output_limit;
    if (pi->integral < -output_limit) pi->integral = -output_limit;
    return saturated;
}

/* Centered SVPWM: common-mode injection and explicit minimum pulse bounds. */
foc_uvw_t foc_svpwm(foc_ab_t voltage, float bus_v, float min_duty, float max_duty)
{
    foc_uvw_t duty = {0.5f, 0.5f, 0.5f};
    if (!isfinite(bus_v) || !isfinite(voltage.alpha) || !isfinite(voltage.beta) ||
        !isfinite(min_duty) || !isfinite(max_duty) || bus_v < 1.0f ||
        min_duty < 0.0f || max_duty > 1.0f || min_duty > max_duty) return duty;
    float vu = voltage.alpha;
    float vv = -0.5f * voltage.alpha + 0.5f * FOC_SQRT3 * voltage.beta;
    float vw = -0.5f * voltage.alpha - 0.5f * FOC_SQRT3 * voltage.beta;
    float vmax = fmaxf(vu, fmaxf(vv, vw));
    float vmin = fminf(vu, fminf(vv, vw));
    float common = -0.5f * (vmax + vmin);
    duty.u = 0.5f + (vu + common) / bus_v;
    duty.v = 0.5f + (vv + common) / bus_v;
    duty.w = 0.5f + (vw + common) / bus_v;
    if (duty.u < min_duty) duty.u = min_duty;
    if (duty.u > max_duty) duty.u = max_duty;
    if (duty.v < min_duty) duty.v = min_duty;
    if (duty.v > max_duty) duty.v = max_duty;
    if (duty.w < min_duty) duty.w = min_duty;
    if (duty.w > max_duty) duty.w = max_duty;
    return duty;
}

void foc_observer_init(foc_observer_t *observer, float initial_flux)
{
    (void)initial_flux;
    if (observer != NULL) memset(observer, 0, sizeof(*observer));
}

/*
 * Back-EMF voltage-model observer with a PLL. The input voltage must be the
 * actually applied, saturated alpha/beta voltage, aligned with current samples.
 * At standstill/low speed voltage errors and dead time dominate, so no lock is
 * claimed below the configured sensorless speed floor. The flux parameter is
 * supplied by commissioning and must not be invented or silently adapted here.
 */
void foc_observer_step(foc_observer_t *observer, foc_ab_t current, foc_ab_t voltage,
                       float rs, float ls, float flux, float dt)
{
    if (observer == NULL) return;
    if (!isfinite(flux) || !isfinite(ls) || !isfinite(rs) || !isfinite(dt) ||
        !isfinite(current.alpha) || !isfinite(current.beta) ||
        !isfinite(voltage.alpha) || !isfinite(voltage.beta) ||
        flux <= 0.0f || ls <= 0.0f || dt <= 0.0f) {
        observer->locked = false;
        observer->confidence = 0.0f;
        return;
    }
    float e_alpha = voltage.alpha - rs * current.alpha -
                    ls * (current.alpha - observer->previous_alpha) / dt;
    float e_beta = voltage.beta - rs * current.beta -
                   ls * (current.beta - observer->previous_beta) / dt;
    observer->previous_alpha = current.alpha;
    observer->previous_beta = current.beta;

    float magnitude = sqrtf(e_alpha * e_alpha + e_beta * e_beta);
    observer->bemf_magnitude = magnitude;
    float expected = fabsf(observer->speed_rad_s) * flux;
    observer->residual = fabsf(magnitude - expected);
    float measured_angle = atan2f(-e_alpha, e_beta);
    float error = measured_angle - observer->angle_rad;
    while (error > FOC_PI) error -= FOC_TWO_PI;
    while (error < -FOC_PI) error += FOC_TWO_PI;
    observer->speed_rad_s += 30.0f * error * dt;
    observer->angle_rad = foc_wrap_angle(observer->angle_rad +
                                         (observer->speed_rad_s + 80.0f * error) * dt);

    bool quality_good = magnitude > 1.0f &&
                        observer->residual < fmaxf(2.0f, 0.35f * magnitude);
    if (quality_good) observer->confidence = fminf(1.0f, observer->confidence + dt * 4.0f);
    else observer->confidence = fmaxf(0.0f, observer->confidence - dt * 8.0f);
    float floor_electrical = (float)APP_OBSERVER_LOCK_MIN_SPEED_RPM * FOC_TWO_PI *
                             (float)APP_MOTOR_POLE_PAIRS / 60.0f;
    observer->locked = observer->confidence > 0.85f && fabsf(observer->speed_rad_s) > floor_electrical;
}

foc_ab_t foc_applied_voltage_from_duty(foc_uvw_t duty, float bus_v)
{
    if (!isfinite(bus_v) || bus_v <= 0.0f) return (foc_ab_t){0.0f, 0.0f};
    float neutral = (duty.u + duty.v + duty.w) / 3.0f;
    float vu = (duty.u - neutral) * bus_v;
    float vv = (duty.v - neutral) * bus_v;
    foc_ab_t applied = {vu, (vu + 2.0f * vv) / FOC_SQRT3};
    return applied;
}

bool foc_step(const foc_input_t *input, foc_pi_t *id_pi, foc_pi_t *iq_pi,
              foc_output_t *output)
{
    if (input == NULL || id_pi == NULL || iq_pi == NULL || output == NULL ||
        !isfinite(input->bus_v) || !isfinite(input->dt) ||
        !isfinite(input->iu) || !isfinite(input->iv) || !isfinite(input->angle) ||
        input->bus_v < APP_UNDERVOLTAGE_TRIP_V || input->dt <= 0.0f) return false;

    foc_ab_t current_ab = foc_clarke(input->iu, input->iv);
    output->measured = foc_park(current_ab, input->angle);
    float voltage_limit = input->bus_v * 0.45f;
    output->voltage.d = foc_pi_step(id_pi, input->id_ref - output->measured.d,
                                    input->dt, voltage_limit);
    output->voltage.q = foc_pi_step(iq_pi, input->iq_ref - output->measured.q,
                                    input->dt, voltage_limit);

    float magnitude = sqrtf(output->voltage.d * output->voltage.d +
                             output->voltage.q * output->voltage.q);
    if (magnitude > voltage_limit && magnitude > 0.0f) {
        float scale = voltage_limit / magnitude;
        output->voltage.d *= scale;
        output->voltage.q *= scale;
        id_pi->integral = output->voltage.d - id_pi->kp * (input->id_ref - output->measured.d);
        iq_pi->integral = output->voltage.q - iq_pi->kp * (input->iq_ref - output->measured.q);
    }
    foc_ab_t voltage_ab = foc_inverse_park(output->voltage, input->angle);
    output->duty = foc_svpwm(voltage_ab, input->bus_v, APP_PWM_MIN_DUTY, APP_PWM_MAX_DUTY);
    output->electrical_speed = 0.0f;
    return true;
}
