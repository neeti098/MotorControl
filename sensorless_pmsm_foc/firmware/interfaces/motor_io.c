#include "motor_io.h"

static const motor_io_t *s_io;
static motor_io_sample_callback_t s_callback;

void motor_io_bind(const motor_io_t *io) { s_io = io; }
const motor_io_t *motor_io(void) { return s_io; }
void motor_io_set_sample_callback(motor_io_sample_callback_t callback) { s_callback = callback; }
bool motor_io_get_current_sample(motor_io_current_sample_t *sample)
{
    return s_io && s_io->read_current_sample && sample && s_io->read_current_sample(sample);
}
void motor_io_clear_current_sample(void)
{
    if (s_io && s_io->read_current_sample) {
        motor_io_current_sample_t discard;
        (void)s_io->read_current_sample(&discard);
    }
}
void motor_io_fast_sample_ready(void)
{
    if (s_io && s_io->read_current_sample && s_callback) {
        motor_io_current_sample_t sample;
        if (s_io->read_current_sample(&sample)) s_callback(&sample);
    }
}
