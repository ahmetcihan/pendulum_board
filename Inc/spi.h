#ifndef __spi_H
#define __spi_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define TIMEOUT_HAL_SPI_TRANSMIT 	 (uint8_t)10

extern SPI_HandleTypeDef hspi2;

void MX_SPI2_Init( void );

#endif /*__ spi_H */
