#ifndef NVIC_DRIVER_
#define NVIC_DRIVER_

#include "stm32f4xx.h"
#include "stdlib.h"
#include <stdint.h>

typedef struct
{
    uint32_t NVIC_IRQChannelCmd;
    uint32_t NVIC_IRQChannelPreemptionPriority;
    uint32_t NVIC_IRQChannelSubPriority;
    uint32_t NVIC_IRQChannel;

} NVIC_InitTypeDef;

void NVIC_Init(NVIC_InitTypeDef *NVIC_InitStruct);

#endif /* NVIC_DRIVER_ */
