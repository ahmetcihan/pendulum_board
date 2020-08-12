#ifndef __dma_H
#define __dma_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/
extern void _Error_Handler(char*, int);

void MX_DMA_Init( void );

#ifdef __cplusplus
}
#endif

#endif /* __dma_H */

