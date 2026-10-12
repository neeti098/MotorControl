#include "app.h"
#include "app_config.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "motor_io.h"
#include "motor_control.h"
#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CMD_TEXT_LEN APP_UART_LINE_MAX
#define TX_TEXT_LEN 384U
#define TELEMETRY_LINE_LEN 384U
#define TELEMETRY_BUF_LEN 384U
#define APP_TELEMETRY_QUEUE_LEN 4U
#define APP_STATUS_MESSAGE_QUEUE_LEN 3U
#define APP_RUN_TELEMETRY_MESSAGE_LEN 192U
#define APP_BOOT_TELEMETRY_MESSAGE_LEN 320U

typedef enum { CMD_START, CMD_STOP, CMD_DISABLE, CMD_CLEAR, CMD_SPEED, CMD_DIRECTION,
               CMD_SOURCE, CMD_HELP, CMD_STATUS, CMD_INVALID } command_kind_t;
typedef struct { command_kind_t kind; float number; int direction; motor_source_t source; } app_command_t;
typedef struct { char text[APP_BOOT_TELEMETRY_MESSAGE_LEN]; } tx_message_t;

static StaticQueue_t s_command_queue_storage;
static uint8_t s_command_queue_buffer[APP_COMMAND_QUEUE_LENGTH * sizeof(app_command_t)];
static QueueHandle_t s_command_queue;
static StaticQueue_t s_tx_queue_storage;
static uint8_t s_tx_queue_buffer[APP_TELEMETRY_QUEUE_LEN * sizeof(tx_message_t)];
static QueueHandle_t s_tx_queue;
static StaticQueue_t s_status_queue_storage;
static uint8_t s_status_queue_buffer[APP_STATUS_MESSAGE_QUEUE_LEN * sizeof(tx_message_t)];
static QueueHandle_t s_status_queue;
static volatile uint32_t s_command_drops;
static volatile uint32_t s_tx_drops;
static uint8_t s_button_state;
static uint8_t s_button_candidate;
static uint32_t s_button_change_ms;
static motor_snapshot_t s_snapshot;

static void queue_reply(const char *text)
{
    tx_message_t msg = {{0}};
    (void)snprintf(msg.text, sizeof(msg.text), "%s\r\n", text);
    if (xQueueSend(s_status_queue, &msg, 0U) != pdPASS) ++s_tx_drops;
}
static bool parse_command(char *line, app_command_t *cmd)
{
    char *save = NULL;
    char *word = strtok_r(line, " \t\r\n", &save);
    if (!word || !cmd) return false;
    memset(cmd, 0, sizeof(*cmd));
    if (!strcmp(word, "help")) cmd->kind = CMD_HELP;
    else if (!strcmp(word, "status")) cmd->kind = CMD_STATUS;
    else if (!strcmp(word, "start")) cmd->kind = CMD_START;
    else if (!strcmp(word, "stop")) cmd->kind = CMD_STOP;
    else if (!strcmp(word, "disable")) cmd->kind = CMD_DISABLE;
    else if (!strcmp(word, "clear")) cmd->kind = CMD_CLEAR;
    else if (!strcmp(word, "speed")) {
        char *arg = strtok_r(NULL, " \t\r\n", &save);
        char *end = NULL;
        if (!arg) return false;
        cmd->number = strtof(arg, &end);
        if (!end || *end || !isfinite(cmd->number)) return false;
        cmd->kind = CMD_SPEED;
    } else if (!strcmp(word, "direction")) {
        char *arg = strtok_r(NULL, " \t\r\n", &save);
        if (!arg) return false;
        if (!strcmp(arg, "forward")) cmd->direction = 1;
        else if (!strcmp(arg, "reverse")) cmd->direction = -1;
        else return false;
        cmd->kind = CMD_DIRECTION;
    } else if (!strcmp(word, "source")) {
        char *arg = strtok_r(NULL, " \t\r\n", &save);
        if (!arg) return false;
        if (!strcmp(arg, "console")) cmd->source = MOTOR_SOURCE_CONSOLE;
        else if (!strcmp(arg, "pot")) cmd->source = MOTOR_SOURCE_POT;
        else return false;
        cmd->kind = CMD_SOURCE;
    } else return false;
    if (strtok_r(NULL, " \t\r\n", &save) != NULL) return false;
    return true;
}

static void supervisor_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    (void)motor_hw_bind_platform();
    motor_control_init();
    motor_io_set_sample_callback(motor_control_fast_sample_callback);
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1U));
        motor_supervisor_tick_1ms();
    }
}
static void command_task(void *arg)
{
    (void)arg;
    static char line[CMD_TEXT_LEN];
    size_t len = 0U;
    const motor_io_t *io = motor_io();
    for (;;) {
        uint8_t byte;
        bool did_work = false;
        while (io && io->uart_rx_pop && io->uart_rx_pop(&byte)) {
            did_work = true;
            if (byte == '\r' || byte == '\n') {
                if (len) {
                    line[len] = '\0';
                    app_command_t cmd;
                    if (parse_command(line, &cmd)) {
                        if (xQueueSend(s_command_queue, &cmd, 0U) != pdPASS) ++s_command_drops;
                    } else queue_reply("ERR malformed command");
                    len = 0U;
                }
            } else if (len + 1U < sizeof(line) && isprint(byte)) line[len++] = (char)byte;
            else { len = 0U; queue_reply("ERR line too long or invalid byte"); }
        }
        app_command_t cmd;
        while (xQueueReceive(s_command_queue, &cmd, 0U) == pdPASS) {
            did_work = true;
            switch (cmd.kind) {
            case CMD_START:
                if (s_snapshot.state == MOTOR_FAULT) queue_reply("ERR fault latched");
                else if (fabsf(s_snapshot.requested_rpm) < APP_SENSORLESS_MIN_RPM) queue_reply("ERR speed below sensorless floor");
                else { motor_set_start(); queue_reply("OK start accepted"); }
                break;
            case CMD_STOP: motor_set_stop(); queue_reply("OK stopping requested"); break;
            case CMD_DISABLE: motor_emergency_disable(); queue_reply("OK bridge disabled; fault clear required"); break;
            case CMD_CLEAR: queue_reply(motor_clear_fault() ? "OK fault cleared; motor remains idle" : "ERR clear conditions not met"); break;
            case CMD_SPEED:
                if (fabsf(cmd.number) < APP_SENSORLESS_MIN_RPM) queue_reply("ERR speed below sensorless floor");
                else if (fabsf(cmd.number) > APP_SENSORLESS_MAX_RPM) queue_reply("ERR speed outside 300..1500 rpm");
                else { motor_set_speed(cmd.number); queue_reply("OK speed set"); }
                break;
            case CMD_DIRECTION: queue_reply(motor_set_direction(cmd.direction) ? "OK direction set; reversal uses stop then restart" : "ERR invalid direction"); break;
            case CMD_SOURCE: queue_reply(motor_set_source(cmd.source) ? "OK source set" : "ERR source can change only in IDLE"); break;
            case CMD_HELP: queue_reply("help status start stop disable fault clear speed <rpm> direction forward|reverse source console|pot"); break;
            case CMD_STATUS: queue_reply("OK status follows in telemetry CSV"); break;
            default: queue_reply("ERR unsupported command"); break;
            }
        }
        if (!did_work) vTaskDelay(pdMS_TO_TICKS(2U));
    }
}
static void ui_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint32_t pressed_ms = 0U;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_UI_PERIOD_MS));
        bool pressed = motor_io()->button_pressed();
        uint32_t now = osal_uptime_ms();
        if (pressed != (s_button_candidate != 0U)) { s_button_candidate = pressed ? 1U : 0U; s_button_change_ms = now; }
        if ((now - s_button_change_ms) >= APP_BUTTON_DEBOUNCE_MS && s_button_state != s_button_candidate) {
            s_button_state = s_button_candidate;
            if (s_button_state) pressed_ms = now;
            else if (now - pressed_ms >= APP_BUTTON_DEBOUNCE_MS) {
                motor_get_snapshot(&s_snapshot);
                if (s_snapshot.state == MOTOR_FAULT) (void)motor_clear_fault();
                else if (s_snapshot.state == MOTOR_IDLE) motor_set_start();
                else motor_set_stop();
            }
        }
        motor_get_snapshot(&s_snapshot);
        bool led_on = s_snapshot.state == MOTOR_RUN;
        bool blink = ((now / ((s_snapshot.state == MOTOR_FAULT) ? 100U : 500U)) & 1U) != 0U;
        if (s_snapshot.state == MOTOR_FAULT || s_snapshot.state == MOTOR_ALIGN || s_snapshot.state == MOTOR_STARTUP || s_snapshot.state == MOTOR_HANDOVER || s_snapshot.state == MOTOR_IDLE)
            led_on = blink;
        motor_io()->led_write(led_on);
    }
}
static void telemetry_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    tx_message_t msg;
    const motor_io_t *io = motor_io();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_TELEMETRY_PERIOD_MS));
        motor_get_snapshot(&s_snapshot);
        int n = snprintf(msg.text, sizeof(msg.text),
          "CSV,%lu,%s,%s,%s,req=%.0f,ramp=%.0f,est=%.0f,ang=%.2f,lock=%u,bus=%.2f,iu=%.2f,iv=%.2f,iw=%.2f,id=%.2f,iq=%.2f,iqr=%.2f,duty=%.2f:%.2f:%.2f,valid=%u,cyc=%lu,ovr=%lu\r\n",
          (unsigned long)s_snapshot.timestamp_ms, motor_state_name(s_snapshot.state),
          s_snapshot.mode == MOTOR_MODE_CLOSED_LOOP ? "CLOSED_LOOP" : (s_snapshot.mode == MOTOR_MODE_FORCED ? "FORCED_ANGLE" : "OFF"),
          motor_fault_name(s_snapshot.fault), s_snapshot.requested_rpm, s_snapshot.ramped_rpm, s_snapshot.estimated_rpm,
          s_snapshot.electrical_angle, s_snapshot.observer_locked, s_snapshot.bus_v, s_snapshot.iu, s_snapshot.iv, s_snapshot.iw,
          s_snapshot.id, s_snapshot.iq, s_snapshot.id_ref, s_snapshot.iq_ref, s_snapshot.vd, s_snapshot.vq,
          s_snapshot.duty_u, s_snapshot.duty_v, s_snapshot.duty_w, s_snapshot.current_valid,
          (unsigned long)s_snapshot.control_cycles, (unsigned long)s_snapshot.overrun_count,
          s_snapshot.rs_ohm, s_snapshot.rs_measured ? 'M' : 'E',
          s_snapshot.ls_h, s_snapshot.ls_measured ? 'M' : 'U',
          s_snapshot.flux_wb, s_snapshot.flux_measured ? 'M' : 'E',
          s_snapshot.polarity_corrected, (unsigned long)s_command_drops, (unsigned long)s_tx_drops);
        if (n > 0 && (size_t)n < sizeof(msg.text)) {
            if (xQueueSend(s_tx_queue, &msg, 0U) != pdPASS) ++s_tx_drops;
        }
        if (io && io->read_potentiometer && s_snapshot.state == MOTOR_IDLE) {
            uint16_t raw;
            if (io->read_potentiometer(&raw) && raw > 256U && raw < 3839U && s_snapshot.mode == MOTOR_MODE_OFF) {
                (void)motor_set_source(MOTOR_SOURCE_POT);
                float speed = APP_SENSORLESS_MIN_RPM + ((float)raw / 4095.0f) * (APP_SENSORLESS_MAX_RPM - APP_SENSORLESS_MIN_RPM);
                motor_set_speed(speed);
            }
        }
    }
}
static void health_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_HEALTH_PERIOD_MS));
        motor_get_snapshot(&s_snapshot);
        if (s_snapshot.state == MOTOR_FAULT) motor_emergency_disable();
        /* Watchdog intentionally not enabled until independent task and control health checks are commissioned. */
    }
}
static void tx_task(void *arg)
{
    (void)arg;
    tx_message_t msg;
    const motor_io_t *io = motor_io();
    for (;;) {
        QueueHandle_t source = (uxQueueMessagesWaiting(s_status_queue) > 0U) ? s_status_queue : s_tx_queue;
        if (xQueueReceive(source, &msg, pdMS_TO_TICKS(20U)) == pdPASS && io && io->uart_tx)
            (void)io->uart_tx((const uint8_t *)msg.text, strlen(msg.text));
    }
}

void app_start(void)
{
    if (!motor_hw_bind_platform()) for (;;) {}
    s_command_queue = xQueueCreateStatic(APP_COMMAND_QUEUE_LENGTH, sizeof(app_command_t), s_command_queue_buffer, &s_command_queue_storage);
    s_tx_queue = xQueueCreateStatic(APP_TELEMETRY_QUEUE_LEN, sizeof(tx_message_t), s_tx_queue_buffer, &s_tx_queue_storage);
    s_status_queue = xQueueCreateStatic(APP_STATUS_MESSAGE_QUEUE_LEN, sizeof(tx_message_t), s_status_queue_buffer, &s_status_queue_storage);
    if (!s_command_queue || !s_tx_queue || !s_status_queue) for (;;) {}
    if (!osal_task_create(supervisor_task, "supervisor", APP_SUPERVISOR_STACK_BYTES, NULL, APP_SUPERVISOR_TASK_PRIORITY) ||
        !osal_task_create(command_task, "commands", APP_COMMAND_STACK_BYTES, NULL, APP_COMMAND_TASK_PRIORITY) ||
        !osal_task_create(ui_task, "ui", APP_UI_STACK_BYTES, NULL, APP_UI_TASK_PRIORITY) ||
        !osal_task_create(telemetry_task, "telemetry", APP_TELEMETRY_STACK_BYTES, NULL, APP_TELEMETRY_TASK_PRIORITY) ||
        !osal_task_create(health_task, "health", APP_HEALTH_STACK_BYTES, NULL, APP_HEALTH_TASK_PRIORITY) ||
        !osal_task_create(tx_task, "tx", APP_TELEMETRY_STACK_BYTES, NULL, APP_TELEMETRY_TASK_PRIORITY)) for (;;) {}
    vTaskStartScheduler();
    for (;;) {}
}
