#include "esc_spi.h"
#include "spi_slave.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include <string.h>

static int16_t throttles[ESC_CHANNELS];
static uint32_t last_frame_ms = 0;

/* Debug variables */
volatile uint8_t debug_pwm_initialized = 0;

/* Simple CRC8 (poly 0x07) */
static uint8_t crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

void ESC_UpdateFromBuffer(uint8_t *buf, uint16_t len)
{
    if (len < 4) return;
    debug_esc_last_crc_ok = 0;
    debug_esc_last_frame_valid = 0;
    if (buf[0] != 0xAA) return;
    uint8_t cmd = buf[1];
    debug_esc_last_cmd = cmd;
    uint8_t dlen = buf[2];
    debug_esc_last_frame_len = len;
    if (dlen + 4 > len) return;
    uint8_t crc = buf[len-1];
    debug_esc_last_crc = crc;
    if (crc8(buf, len-1) != crc) return;
    debug_esc_last_crc_ok = 1;

    if (cmd == 0x01 && dlen == ESC_CHANNELS * 2) {
        for (int i = 0; i < ESC_CHANNELS; ++i) {
            int16_t v = (int16_t)(buf[3 + i*2] | (buf[3 + i*2 +1] << 8));
            throttles[i] = v;
            debug_esc_throttles[i] = v;
        }
        /* update last frame time (simple HAL_GetTick fallback if available) */
        last_frame_ms = HAL_GetTick();
        debug_esc_last_frame_valid = 1;
        debug_esc_last_frame_ms = last_frame_ms;
    }
}

int16_t ESC_GetThrottle(int idx)
{
    if (idx < 0 || idx >= ESC_CHANNELS) return 0;
    return throttles[idx];
}

/* Internal function to set PWM value */
static void Pwm_SetChannelInternal(uint8_t channel, uint32_t pulse_us)
{
    if (channel >= 8) return;
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;

    if (channel < 4) {
        uint32_t ch = (channel == 0) ? TIM_CHANNEL_1 : 
                      (channel == 1) ? TIM_CHANNEL_2 : 
                      (channel == 2) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
        __HAL_TIM_SET_COMPARE(&htim1, ch, pulse_us);
    } else {
        uint32_t ch = (channel == 4) ? TIM_CHANNEL_1 : 
                      (channel == 5) ? TIM_CHANNEL_2 : 
                      (channel == 6) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
        __HAL_TIM_SET_COMPARE(&htim4, ch, pulse_us);
    }
}

/* Initialize PWM */
void ESC_PWM_Init(void)
{
    debug_pwm_initialized = 0;

    /* Ensure GPIO clocks are enabled */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Disable any TIM4 remapping - use default pins (PB6-PB9) */
    __HAL_AFIO_REMAP_TIM4_DISABLE();

    /* Set initial pulse to mid (1500us) for all channels */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1500);

    /* Enable advanced timer main output (TIM1 only) */
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* Enable TIM1 counter first */
    __HAL_TIM_ENABLE(&htim1);

    /* Start TIM1 channels (PA8-PA11) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* Enable TIM4 counter */
    __HAL_TIM_ENABLE(&htim4);

    /* Start TIM4 channels (PB6-PB9) */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    debug_pwm_initialized = 1;
}

/* Test function: set fixed PWM value on all channels */
void ESC_PWM_Test_Fixed(uint32_t pulse_us)
{
    for (int i = 0; i < 8; i++) {
        Pwm_SetChannelInternal(i, pulse_us);
    }
}

void ESC_SPI_Init(void)
{
    for (int i = 0; i < ESC_CHANNELS; ++i) throttles[i] = 0;
}

/* Helper: apply throttle to TIM PWM outputs. Map throttles (assumed -100..100) to pulse width 1ms..2ms (1000..2000us) */
void ESC_ApplyToPWM(void)
{
    uint32_t pulse_us;
    for (int i = 0; i < ESC_CHANNELS; ++i)
    {
        int16_t v = throttles[i];
        /* Support both -100..100 percentage or 1000..2000us direct pulse width */
        if (v >= -100 && v <= 100)
        {
            pulse_us = (uint32_t)(1500 + (v * 5));
        }
        else
        {
            pulse_us = (uint32_t)v;
        }
        Pwm_SetChannelInternal(i, pulse_us);
    }
}

void ESC_TimeoutHandler(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    if (now - last_frame_ms > timeout_ms) {
        debug_esc_timeout_active = 1;
        /* reset to mid (0 -> 1500us mapping) */
        for (int i = 0; i < ESC_CHANNELS; ++i) throttles[i] = 0;
        ESC_ApplyToPWM();
    } else {
        debug_esc_timeout_active = 0;
    }
}
