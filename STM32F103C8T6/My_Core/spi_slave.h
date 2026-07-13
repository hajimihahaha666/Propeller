#ifndef _SPI_SLAVE_H
#define _SPI_SLAVE_H

#include "stdint.h"

void SPI_Slave_Init(void);

/* Call this from HAL_SPI_IRQHandler if you integrate interrupts yourself */
void SPI_Slave_IRQHandler(void);

/* Polling/interrupt-driven start receive */
void SPI_Slave_StartReceive(void);

/* Call frequently from main loop to start SPI transfers after CS falling edge */
void SPI_Slave_Poll(void);

/* Debug variables for SPI receive state */
extern volatile uint16_t debug_spi_frame_len;
extern volatile uint32_t debug_spi_frame_count;
extern volatile uint8_t debug_spi_last_byte;
extern volatile uint8_t debug_spi_frame_ready;
extern volatile uint8_t debug_spi_error;

#endif
