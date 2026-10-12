#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "app_config.h"

static StaticTask_t s_task_tcb[6];
static StackType_t s_task_stack[6][512];
static uint32_t s_stack_words[6] = {APP_SUPERVISOR_STACK_BYTES / sizeof(StackType_t), APP_COMMAND_STACK_BYTES / sizeof(StackType_t), APP_UI_STACK_BYTES / sizeof(StackType_t), APP_TELEMETRY_STACK_BYTES / sizeof(StackType_t), APP_HEALTH_STACK_BYTES / sizeof(StackType_t), APP_TELEMETRY_STACK_BYTES / sizeof(StackType_t)};
static StaticSemaphore_t s_mutex_storage[4];
static StaticTask_t s_idle_tcb;
static StackType_t s_idle_stack[configMINIMAL_STACK_SIZE];
static uint8_t s_mutex_used[4];
static uint8_t s_task_used[6];
static uint32_t s_task_created;

void osal_delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
uint32_t osal_uptime_ms(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }
bool osal_scheduler_running(void) { return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED; }
uint32_t osal_task_count(void) { return s_task_created; }
uint32_t osal_task_stack_high_water_words(osal_task_t task)
{
    return task ? (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)task) : 0U;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, uint32_t *size)
{
    if (tcb) *tcb = &s_idle_tcb;
    if (stack) *stack = s_idle_stack;
    if (size) *size = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task, signed char *name)
{
    (void)task;
    (void)name;
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

osal_task_t osal_task_create(osal_task_fn_t fn, const char *name, size_t stack_bytes, void *arg, uint8_t priority)
{
    if (!fn) return NULL;
    for (size_t i = 0; i < 6U; ++i) {
        if (!s_task_used[i]) {
            uint32_t requested_words = (uint32_t)(stack_bytes / sizeof(StackType_t));
            if (requested_words == 0U || requested_words > s_stack_words[i]) return NULL;
            TaskHandle_t handle = xTaskCreateStatic(fn, name, requested_words, arg, priority,
                                                    s_task_stack[i], &s_task_tcb[i]);
            if (handle != NULL) {
                s_task_used[i] = 1U;
                ++s_task_created;
            }
            return (osal_task_t)handle;
        }
    }
    return NULL;
}
osal_mutex_t osal_mutex_create(void)
{
    for (size_t i = 0; i < 4U; ++i) {
        if (!s_mutex_used[i]) {
            s_mutex_used[i] = 1U;
            return (osal_mutex_t)xSemaphoreCreateMutexStatic(&s_mutex_storage[i]);
        }
    }
    return NULL;
}
bool osal_mutex_lock(osal_mutex_t mutex, uint32_t timeout_ms)
{
    return mutex && xSemaphoreTake((SemaphoreHandle_t)mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
void osal_mutex_unlock(osal_mutex_t mutex)
{
    if (mutex) (void)xSemaphoreGive((SemaphoreHandle_t)mutex);
}
