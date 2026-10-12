#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_io.h"
#include "main.h"

bool motor_hw_bind_platform(void);
uint32_t motor_hw_micros(void);
HAL_StatusTypeDef motor_hw_start_current_sampling(void);
ADC_HandleTypeDef *motor_hw_adc1_handle(void);
ADC_HandleTypeDef *motor_hw_adc2_handle(void);

void motor_hw_uart_irq(void);
void motor_hw_uart_rx_irq(void);

#endif
