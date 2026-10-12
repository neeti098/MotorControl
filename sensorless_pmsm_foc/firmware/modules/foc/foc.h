#ifndef FOC_H
#define FOC_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { float alpha, beta; } foc_ab_t;
typedef struct { float d, q; } foc_dq_t;
typedef struct { float u, v, w; } foc_uvw_t;
typedef struct { float kp, ki, integral, limit; } foc_pi_t;

typedef struct {
    float angle_rad;
    float speed_rad_s;
    float confidence;
    float residual;
    float bemf_magnitude;
    float previous_alpha;
    float previous_beta;
    bool locked;
} foc_observer_t;

typedef struct {
    float iu, iv, iw;
    float bus_v;
    float id_ref, iq_ref;
    float angle;
    float dt;
    float rs, ls, flux;
} foc_input_t;

typedef struct {
    foc_dq_t measured;
    foc_dq_t voltage;
    foc_uvw_t duty;
    float electrical_speed;
} foc_output_t;

float foc_wrap_angle(float angle);
foc_ab_t foc_clarke(float iu, float iv);
foc_dq_t foc_park(foc_ab_t ab, float angle);
foc_ab_t foc_inverse_park(foc_dq_t dq, float angle);
float foc_pi_step(foc_pi_t *pi, float error, float dt, float output_limit);
foc_uvw_t foc_svpwm(foc_ab_t v, float bus_v, float min_duty, float max_duty);
void foc_observer_init(foc_observer_t *observer, float initial_flux);
void foc_observer_step(foc_observer_t *observer, foc_ab_t current, foc_ab_t voltage,
                       float rs, float ls, float flux, float dt);
bool foc_step(const foc_input_t *input, foc_pi_t *id_pi, foc_pi_t *iq_pi,
              foc_output_t *output);
foc_ab_t foc_applied_voltage_from_duty(foc_uvw_t duty, float bus_v);
#endif
