#include "motor_hw.h"
#include "main.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "app_config.h"

static TIM_HandleTypeDef s_tim1;
static ADC_HandleTypeDef s_adc1;
static ADC_HandleTypeDef s_adc2;
static UART_HandleTypeDef s_uart2;
static volatile uint16_t s_current_u;
static volatile uint16_t s_current_v;
static volatile bool s_sample_valid;
static uint32_t s_adc_timestamp_us;
static volatile uint8_t s_adc_pair_mask;
static volatile uint8_t s_rx_byte;
static volatile uint8_t s_rx_data[128];
static volatile uint16_t s_rx_head, s_rx_tail;
static uint32_t s_tick_origin;

#include <stddef.h>
static void gpio_analog(GPIO_TypeDef *port, uint16_t pins)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pins; gpio.Mode = GPIO_MODE_ANALOG; gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &gpio);
}

static bool adc_configure_injected(ADC_HandleTypeDef *adc, uint32_t channel)
{
    ADC_InjectionConfTypeDef cfg = {0};
    cfg.InjectedChannel = channel;
    cfg.InjectedRank = ADC_INJECTED_RANK_1;
    cfg.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    cfg.InjectedSingleDiff = ADC_SINGLE_ENDED;
    cfg.InjectedOffsetNumber = ADC_OFFSET_NONE;
    cfg.InjectedOffset = 0U;
    cfg.InjectedNbrOfConversion = 1U;
    cfg.InjectedDiscontinuousConvMode = DISABLE;
    cfg.AutoInjectedConv = DISABLE;
    cfg.QueueInjectedContext = DISABLE;
    cfg.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
    cfg.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    return HAL_ADCEx_InjectedConfigChannel(adc, &cfg) == HAL_OK;
}

static bool adc_initialize(ADC_HandleTypeDef *adc, ADC_TypeDef *instance)
{
    adc->Instance = instance;
    adc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    adc->Init.Resolution = ADC_RESOLUTION_12B;
    adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc->Init.GainCompensation = 0U;
    adc->Init.ScanConvMode = ADC_SCAN_DISABLE;
    adc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    adc->Init.LowPowerAutoWait = DISABLE;
    adc->Init.ContinuousConvMode = DISABLE;
    adc->Init.NbrOfConversion = 1U;
    adc->Init.DiscontinuousConvMode = DISABLE;
    adc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc->Init.DMAContinuousRequests = DISABLE;
    adc->Init.Overrun = ADC_OVR_DATA_PRESERVED;
    adc->Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(adc) != HAL_OK) return false;
    return HAL_ADCEx_Calibration_Start(adc, ADC_SINGLE_ENDED) == HAL_OK;
}

static bool adc_read_regular(ADC_HandleTypeDef *adc, uint32_t channel, uint16_t *value)
{
    ADC_ChannelConfTypeDef cfg = {0};
    if (!value) return false;
    cfg.Channel = channel; cfg.Rank = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    cfg.SingleDiff = ADC_SINGLE_ENDED; cfg.OffsetNumber = ADC_OFFSET_NONE; cfg.Offset = 0U;
    bool was_injected_active = ((adc->Instance->JSQR & ADC_JSQR_JL) != 0U) || ((adc->Instance->IER & ADC_IER_JEOSIE) != 0U);
    if (was_injected_active && HAL_ADCEx_InjectedStop(adc) != HAL_OK) return false;
    if (HAL_ADC_ConfigChannel(adc, &cfg) != HAL_OK || HAL_ADC_Start(adc) != HAL_OK) return false;
    bool ok = HAL_ADC_PollForConversion(adc, 2U) == HAL_OK;
    if (ok) *value = (uint16_t)HAL_ADC_GetValue(adc);
    (void)HAL_ADC_Stop(adc);
    if (was_injected_active) {
        ADC_InjectionConfTypeDef inj = {0};
        inj.InjectedChannel = (adc->Instance == ADC1) ? ADC_CHANNEL_12 : ADC_CHANNEL_2;
        inj.InjectedRank = ADC_INJECTED_RANK_1;
        inj.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
        inj.InjectedSingleDiff = ADC_SINGLE_ENDED;
        inj.InjectedOffsetNumber = ADC_OFFSET_NONE;
        inj.InjectedNbrOfConversion = 1U;
        inj.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
        inj.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
        if (HAL_ADCEx_InjectedConfigChannel(adc, &inj) != HAL_OK || HAL_ADCEx_InjectedStart_IT(adc) != HAL_OK) return false;
    }
    return ok;
}

static bool hw_initialize(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_5; HAL_GPIO_Init(GPIOA, &gpio);

    /* Analog-only mapped pins including unused parallel routing pads. */
    gpio_analog(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_7);
    gpio_analog(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_11);
    gpio_analog(GPIOC, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
    /* PB12 is a high-impedance active-low TIM1 break input; external board pull-up. */
    gpio.Pin = GPIO_PIN_12; gpio.Mode = GPIO_MODE_AF_PP; gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW; gpio.Alternate = GPIO_AF6_TIM1; HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_11; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_13; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3; gpio.Alternate = GPIO_AF7_USART2; HAL_GPIO_Init(GPIOA, &gpio);

    s_tim1.Instance = TIM1;
    s_tim1.Init.Prescaler = 0U;
    s_tim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    s_tim1.Init.Period = APP_PWM_HALF_PERIOD_COUNTS;
    s_tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim1.Init.RepetitionCounter = 0U;
    s_tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    s_tick_origin = HAL_GetTick();
    __HAL_RCC_TIM1_CLK_ENABLE();
    if (HAL_TIM_PWM_Init(&s_tim1) != HAL_OK) return false;
    TIM_MasterConfigTypeDef master = {0};
    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_tim1, &master) != HAL_OK) return false;
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1; oc.Pulse = APP_PWM_HALF_PERIOD_COUNTS / 2U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH; oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE; oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&s_tim1, &oc, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&s_tim1, &oc, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&s_tim1, &oc, TIM_CHANNEL_3) != HAL_OK) return false;
    /* PWM2 on CH4 provides a recurring OC4REF edge for the paired injected ADC trigger. */
    oc.OCMode = TIM_OCMODE_PWM2;
    oc.Pulse = APP_PWM_SAMPLE_GUARD_COUNTS;
    if (HAL_TIM_PWM_ConfigChannel(&s_tim1, &oc, TIM_CHANNEL_4) != HAL_OK) return false;
    TIM_BreakDeadTimeConfigTypeDef bd = {0};
    bd.OffStateRunMode = TIM_OSSR_ENABLE; bd.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bd.LockLevel = TIM_LOCKLEVEL_OFF; bd.DeadTime = 0U; bd.BreakState = TIM_BREAK_ENABLE;
    bd.BreakPolarity = TIM_BREAKPOLARITY_LOW; bd.BreakFilter = 0U; bd.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    bd.Break2State = TIM_BREAK2_DISABLE; bd.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&s_tim1, &bd) != HAL_OK) return false;

    if (!adc_initialize(&s_adc1, ADC1) || !adc_initialize(&s_adc2, ADC2)) return false;
    if (!adc_configure_injected(&s_adc1, ADC_CHANNEL_12) || !adc_configure_injected(&s_adc2, ADC_CHANNEL_2)) return false;

    s_uart2.Instance = USART2; s_uart2.Init.BaudRate = APP_UART_BAUD;
    s_uart2.Init.WordLength = UART_WORDLENGTH_8B; s_uart2.Init.StopBits = UART_STOPBITS_1;
    s_uart2.Init.Parity = UART_PARITY_NONE; s_uart2.Init.Mode = UART_MODE_TX_RX;
    s_uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; s_uart2.Init.OverSampling = UART_OVERSAMPLING_16;
    s_uart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; s_uart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    s_uart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&s_uart2) != HAL_OK) return false;
    HAL_NVIC_SetPriority(ADC1_2_IRQn, APP_FAST_IRQ_PRIORITY, 0U); HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 7U, 0U); HAL_NVIC_EnableIRQ(USART2_IRQn);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_tick_origin = HAL_GetTick();
    return true;
}

static void hw_start_pwm(void)
{
    (void)motor_hw_start_current_sampling();
    __HAL_TIM_SET_COMPARE(&s_tim1, TIM_CHANNEL_1, APP_PWM_HALF_PERIOD_COUNTS / 2U);
    __HAL_TIM_SET_COMPARE(&s_tim1, TIM_CHANNEL_2, APP_PWM_HALF_PERIOD_COUNTS / 2U);
    __HAL_TIM_SET_COMPARE(&s_tim1, TIM_CHANNEL_3, APP_PWM_HALF_PERIOD_COUNTS / 2U);
    (void)HAL_TIM_PWM_Start(&s_tim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&s_tim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&s_tim1, TIM_CHANNEL_3);
    (void)HAL_TIM_OC_Start(&s_tim1, TIM_CHANNEL_4);
    /* No main-output enable (MOE) while idle; enables stay physically low. */
    __HAL_TIM_MOE_DISABLE(&s_tim1);
}
static bool hw_fault(void);
static void hw_enable(void)
{
    if (hw_fault()) return;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    __HAL_TIM_MOE_ENABLE(&s_tim1);
}
static void hw_disable(void)
{
    __HAL_TIM_MOE_DISABLE(&s_tim1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
}
static void hw_duties(float u, float v, float w)
{
    uint32_t top = APP_PWM_HALF_PERIOD_COUNTS;
    u = (u < APP_PWM_MIN_DUTY) ? APP_PWM_MIN_DUTY : ((u > APP_PWM_MAX_DUTY) ? APP_PWM_MAX_DUTY : u);
    v = (v < APP_PWM_MIN_DUTY) ? APP_PWM_MIN_DUTY : ((v > APP_PWM_MAX_DUTY) ? APP_PWM_MAX_DUTY : v);
    w = (w < APP_PWM_MIN_DUTY) ? APP_PWM_MIN_DUTY : ((w > APP_PWM_MAX_DUTY) ? APP_PWM_MAX_DUTY : w);
    __HAL_TIM_SET_COMPARE(&s_tim1, TIM_CHANNEL_1, (uint32_t)(u * (float)top));
    __HAL_TIM_SET_COMPARE(&s_tim1, TIM_CHANNEL_2, (uint32_t)(v * (float)top));
    __HAL_TIM_SET_COMPARE(&s_tim1, TIM_CHANNEL_3, (uint32_t)(w * (float)top));
}
static bool hw_fault(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET ||
           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET ||
           (__HAL_TIM_GET_FLAG(&s_tim1, TIM_FLAG_BREAK) != RESET);
}
static bool hw_button_pressed(void) { return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET; }
static void hw_led_write(bool on) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static bool hw_sample(motor_io_current_sample_t *sample)
{
    if (!sample || !s_sample_valid) return false;
    sample->phase_u = s_current_u;
    sample->phase_v = s_current_v;
    sample->phase_w = 0U;
    sample->timestamp_us = s_adc_timestamp_us;
    sample->valid = true;
    s_sample_valid = false;
    return true;
}
static bool hw_bus(float *volts)
{
    uint16_t raw;
    if (!volts || !adc_read_regular(&s_adc1, ADC_CHANNEL_1, &raw)) return false;
    *volts = (float)raw * 3.3f / 4095.0f / 0.0625f;
    return true;
}
static bool hw_pot(uint16_t *raw) { return adc_read_regular(&s_adc1, ADC_CHANNEL_8, raw); }
static bool hw_ntc(uint16_t *raw) { return adc_read_regular(&s_adc2, ADC_CHANNEL_5, raw); }
static bool hw_rx_start(void) { return HAL_UART_Receive_IT(&s_uart2, (uint8_t *)&s_rx_byte, 1U) == HAL_OK; }
static bool hw_rx_pop(uint8_t *byte)
{
    if (!byte || s_rx_tail == s_rx_head) return false;
    *byte = s_rx_data[s_rx_tail]; s_rx_tail = (uint16_t)((s_rx_tail + 1U) % sizeof(s_rx_data)); return true;
}
static bool hw_tx(const uint8_t *data, size_t len)
{
    return data && len <= UINT16_MAX && HAL_UART_Transmit_IT(&s_uart2, (uint8_t *)data, (uint16_t)len) == HAL_OK;
}
static uint32_t hw_cycles(void) { return DWT->CYCCNT; }
static uint32_t hw_cycle_budget(uint32_t us) { return us * 170U; }
static uint32_t hw_micros(void)
{
    return (HAL_GetTick() - s_tick_origin) * 1000U + (DWT->CYCCNT % 170000U) / 170U;
}

static const motor_io_t s_io = {
    hw_initialize, hw_start_pwm, hw_enable, hw_disable, hw_duties, hw_fault,
    hw_button_pressed, hw_led_write, hw_sample, hw_bus, hw_pot, hw_ntc, hw_rx_start, hw_rx_pop, hw_tx,
    hw_cycles, hw_cycle_budget, hw_micros
};

bool motor_hw_bind_platform(void) { motor_io_bind(&s_io); return true; }
HAL_StatusTypeDef motor_hw_start_current_sampling(void)
{
    if (HAL_ADCEx_InjectedStart_IT(&s_adc2) != HAL_OK) return HAL_ERROR;
    if (HAL_ADCEx_InjectedStart_IT(&s_adc1) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}
ADC_HandleTypeDef *motor_hw_adc1_handle(void) { return &s_adc1; }
ADC_HandleTypeDef *motor_hw_adc2_handle(void) { return &s_adc2; }
void motor_hw_uart_irq(void) { HAL_UART_IRQHandler(&s_uart2); }
void motor_hw_uart_rx_irq(void)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % sizeof(s_rx_data));
    if (next != s_rx_tail) { s_rx_data[s_rx_head] = s_rx_byte; s_rx_head = next; }
    (void)HAL_UART_Receive_IT(&s_uart2, (uint8_t *)&s_rx_byte, 1U);
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        s_current_v = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        s_adc_pair_mask |= 1U;
    } else if (hadc->Instance == ADC2) {
        s_current_u = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        s_adc_pair_mask |= 2U;
    }
    if (s_adc_pair_mask == 3U) {
        s_adc_pair_mask = 0U;
        s_adc_timestamp_us = hw_micros();
        s_sample_valid = true;
        motor_io_fast_sample_ready();
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) motor_hw_uart_rx_irq();
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    (void)htim;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
}
uint32_t motor_hw_micros(void) { return hw_micros(); }
