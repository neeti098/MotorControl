#ifndef OSAL_H
#define OSAL_H

/* OS abstraction layer — the contract that lets services/ and modules/ start tasks, take
 * locks and sleep without naming an RTOS.
 *
 * DECLARATIONS ONLY, and on this target NOTHING IMPLEMENTS IT YET. Calling one of these
 * from app/, services/ or modules/ will compile and then fail at LINK. If you need one,
 * write the body in platforms/stm32/ (a SysTick counter covers osal_uptime_ms and
 * osal_delay_ms on a bare-metal build; tasks and mutexes need an RTOS). Do NOT respond to a
 * link error by moving your loop into platforms/ -- that is the layer no host unit test can
 * compile, and the timing logic is exactly what you want testable. Nothing in this header
 * may include a freertos/ header or any other vendor SDK header (writing the glob with a
 * star would open a nested comment and break Doxygen for the whole file); that is the
 * entire point of the layer.
 *
 * Call these from app/, services/ and modules/ instead of including freertos/ headers
 * there. That is not bureaucracy: a loop written with vTaskDelay has to live in
 * platforms/, and nothing in platforms/ can be compiled by a host unit test. Keep the decision portable --
 * `blink_tick(&state, now_ms)` -- and let this layer do the waiting.
 *
 * Delete this file if the project does not need an RTOS seam.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void *osal_task_t;
typedef void *osal_mutex_t;

typedef void (*osal_task_fn_t)(void *arg);

/* Sleep the calling task. */
void osal_delay_ms(uint32_t ms);

/* Monotonic milliseconds since boot. */
uint32_t osal_uptime_ms(void);
bool osal_scheduler_running(void);
uint32_t osal_task_count(void);
uint32_t osal_task_stack_high_water_words(osal_task_t task);

/* Task lifecycle. Returns NULL on failure — callers must check; a silently failed task
   creation is an unbootable device with no log line. */
osal_task_t osal_task_create(osal_task_fn_t fn,
                             const char *name,
                             size_t stack_bytes,
                             void *arg,
                             uint8_t priority);

/* Mutual exclusion. osal_mutex_create returns NULL on failure. */
osal_mutex_t osal_mutex_create(void);
bool osal_mutex_lock(osal_mutex_t mutex, uint32_t timeout_ms);
void osal_mutex_unlock(osal_mutex_t mutex);

#endif /* OSAL_H */
