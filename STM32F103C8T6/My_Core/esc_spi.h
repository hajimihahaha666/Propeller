/* esc_spi.h - SPI slave for receiving 8-channel throttle values */
#ifndef _ESC_SPI_H
#define _ESC_SPI_H

#include "stdint.h"

#define ESC_CHANNELS 8

void ESC_SPI_Init(void);
void ESC_UpdateFromBuffer(uint8_t *buf, uint16_t len);
int16_t ESC_GetThrottle(int idx);
void ESC_ApplyToPWM(void);
void ESC_TimeoutHandler(uint32_t timeout_ms);

/* PWM functions */
void ESC_PWM_Init(void);
void ESC_PWM_Test_Fixed(uint32_t pulse_us);

/* Debug variables for watch window */
extern volatile uint32_t debug_esc_last_frame_ms;
extern volatile uint16_t debug_esc_last_frame_len;
extern volatile uint8_t debug_esc_last_cmd;
extern volatile uint8_t debug_esc_last_crc;
extern volatile uint8_t debug_esc_last_crc_ok;
extern volatile uint8_t debug_esc_last_frame_valid;
extern volatile int16_t debug_esc_throttles[ESC_CHANNELS];
extern volatile uint8_t debug_esc_timeout_active;
extern volatile uint8_t debug_pwm_initialized;

/* Frame format (example):
   [0] HEADER 0xAA
   [1] CMD    0x01 (write channels)
   [2] LEN    N (should be 16 for 8x int16)
   [3..] DATA  8x int16_t little-endian (microseconds or percentage scaled)
   [..] CRC8
*/

#endif
