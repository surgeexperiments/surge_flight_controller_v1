/**
 *	@file dma_driver.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains the functions to init and de-init the DMA for I2C,
 *	  	   and a general function to init any DMA stream.
 *
 * TODO:
 * - dma_init_i2c: Fix mem addr (dma_perh_base_addr), make this work for more than I2C1
 */

#include "dma_driver.h"
#include "nvic_driver.h"

/**
 * 	@brief Function to de-init a DMA stream
 *
 * 	@param DMAy_Streamx an instance of DMA_Stream_TypeDef representing a stream to be de-init
 */
static void dma_de_init(DMA_Stream_TypeDef *DMAy_Streamx)
{

    /* Disable the selected DMAy Streamx */
    DMAy_Streamx->CR &= ~((uint32_t)DMA_SxCR_EN);

    /* Reset DMAy Streamx control register */
    DMAy_Streamx->CR = 0;

    /* Reset DMAy Streamx Number of Data to Transfer register */
    DMAy_Streamx->NDTR = 0;

    /* Reset DMAy Streamx peripheral address register */
    DMAy_Streamx->PAR = 0;

    /* Reset DMAy Streamx memory 0 address register */
    DMAy_Streamx->M0AR = 0;

    /* Reset DMAy Streamx memory 1 address register */
    DMAy_Streamx->M1AR = 0;

    /* Reset DMAy Streamx FIFO control register */
    DMAy_Streamx->FCR = (uint32_t)0x00000021;

    /* Reset interrupt pending bits for the selected stream */
    if (DMAy_Streamx == DMA1_Stream0)
    {
        /* Reset interrupt pending bits for DMA1 Stream0 */
        DMA1->LIFCR = DMA_Stream0_IT_MASK;
    }
    else if (DMAy_Streamx == DMA1_Stream1)
    {
        /* Reset interrupt pending bits for DMA1 Stream1 */
        DMA1->LIFCR = DMA_Stream1_IT_MASK;
    }
    else if (DMAy_Streamx == DMA1_Stream2)
    {
        /* Reset interrupt pending bits for DMA1 Stream2 */
        DMA1->LIFCR = DMA_Stream2_IT_MASK;
    }
    else if (DMAy_Streamx == DMA1_Stream3)
    {
        /* Reset interrupt pending bits for DMA1 Stream3 */
        DMA1->LIFCR = DMA_Stream3_IT_MASK;
    }
    else if (DMAy_Streamx == DMA1_Stream4)
    {
        /* Reset interrupt pending bits for DMA1 Stream4 */
        DMA1->HIFCR = DMA_Stream4_IT_MASK;
    }
    else if (DMAy_Streamx == DMA1_Stream5)
    {
        /* Reset interrupt pending bits for DMA1 Stream5 */
        DMA1->HIFCR = DMA_Stream5_IT_MASK;
    }
    else if (DMAy_Streamx == DMA1_Stream6)
    {
        /* Reset interrupt pending bits for DMA1 Stream6 */
        DMA1->HIFCR = (uint32_t)DMA_Stream6_IT_MASK;
    }
    else if (DMAy_Streamx == DMA1_Stream7)
    {
        /* Reset interrupt pending bits for DMA1 Stream7 */
        DMA1->HIFCR = DMA_Stream7_IT_MASK;
    }
    else if (DMAy_Streamx == DMA2_Stream0)
    {
        /* Reset interrupt pending bits for DMA2 Stream0 */
        DMA2->LIFCR = DMA_Stream0_IT_MASK;
    }
    else if (DMAy_Streamx == DMA2_Stream1)
    {
        /* Reset interrupt pending bits for DMA2 Stream1 */
        DMA2->LIFCR = DMA_Stream1_IT_MASK;
    }
    else if (DMAy_Streamx == DMA2_Stream2)
    {
        /* Reset interrupt pending bits for DMA2 Stream2 */
        DMA2->LIFCR = DMA_Stream2_IT_MASK;
    }
    else if (DMAy_Streamx == DMA2_Stream3)
    {
        /* Reset interrupt pending bits for DMA2 Stream3 */
        DMA2->LIFCR = DMA_Stream3_IT_MASK;
    }
    else if (DMAy_Streamx == DMA2_Stream4)
    {
        /* Reset interrupt pending bits for DMA2 Stream4 */
        DMA2->HIFCR = DMA_Stream4_IT_MASK;
    }
    else if (DMAy_Streamx == DMA2_Stream5)
    {
        /* Reset interrupt pending bits for DMA2 Stream5 */
        DMA2->HIFCR = DMA_Stream5_IT_MASK;
    }
    else if (DMAy_Streamx == DMA2_Stream6)
    {
        /* Reset interrupt pending bits for DMA2 Stream6 */
        DMA2->HIFCR = DMA_Stream6_IT_MASK;
    }
    else
    {
        if (DMAy_Streamx == DMA2_Stream7)
        {
            /* Reset interrupt pending bits for DMA2 Stream7 */
            DMA2->HIFCR = DMA_Stream7_IT_MASK;
        }
    }
}

/**
 * 	@brief Function borrowed from the peripheral library for initing a DMA struct
 *
 * 	See the peripheral lib documentation for into about this function
 */
static void DMA_Init(DMA_Stream_TypeDef *DMAy_Streamx, DMA_InitTypeDef *DMA_InitStruct)
{
    uint32_t tmpreg = 0;

    /*------------------------- DMAy Streamx CR Configuration ------------------*/
    /* Get the DMAy_Streamx CR value */
    tmpreg = DMAy_Streamx->CR;

    /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
    tmpreg &= ((uint32_t) ~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST |
                            DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
                            DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC |
                            DMA_SxCR_DIR));

    /* Configure DMAy Streamx: */
    /* Set CHSEL bits according to DMA_CHSEL value */
    /* Set DIR bits according to DMA_DIR value */
    /* Set PINC bit according to DMA_PeripheralInc value */
    /* Set MINC bit according to DMA_MemoryInc value */
    /* Set PSIZE bits according to DMA_PeripheralDataSize value */
    /* Set MSIZE bits according to DMA_MemoryDataSize value */
    /* Set CIRC bit according to DMA_Mode value */
    /* Set PL bits according to DMA_Priority value */
    /* Set MBURST bits according to DMA_MemoryBurst value */
    /* Set PBURST bits according to DMA_PeripheralBurst value */
    tmpreg |= DMA_InitStruct->DMA_Channel | DMA_InitStruct->DMA_DIR |
              DMA_InitStruct->DMA_PeripheralInc | DMA_InitStruct->DMA_MemoryInc |
              DMA_InitStruct->DMA_PeripheralDataSize | DMA_InitStruct->DMA_MemoryDataSize |
              DMA_InitStruct->DMA_Mode | DMA_InitStruct->DMA_Priority |
              DMA_InitStruct->DMA_MemoryBurst | DMA_InitStruct->DMA_PeripheralBurst;

    /* Write to DMAy Streamx CR register */
    DMAy_Streamx->CR = tmpreg;

    /*------------------------- DMAy Streamx FCR Configuration -----------------*/
    /* Get the DMAy_Streamx FCR value */
    tmpreg = DMAy_Streamx->FCR;

    /* Clear DMDIS and FTH bits */
    tmpreg &= (uint32_t) ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

    /* Configure DMAy Streamx FIFO:
      Set DMDIS bits according to DMA_FIFOMode value
      Set FTH bits according to DMA_FIFOThreshold value */
    tmpreg |= DMA_InitStruct->DMA_FIFOMode | DMA_InitStruct->DMA_FIFOThreshold;

    /* Write to DMAy Streamx CR */
    DMAy_Streamx->FCR = tmpreg;

    /*------------------------- DMAy Streamx NDTR Configuration ----------------*/
    /* Write to DMAy Streamx NDTR register */
    DMAy_Streamx->NDTR = DMA_InitStruct->DMA_BufferSize;

    /*------------------------- DMAy Streamx PAR Configuration -----------------*/
    /* Write to DMAy Streamx PAR */
    DMAy_Streamx->PAR = DMA_InitStruct->DMA_PeripheralBaseAddr;

    /*------------------------- DMAy Streamx M0AR Configuration ----------------*/
    /* Write to DMAy Streamx M0AR */
    DMAy_Streamx->M0AR = DMA_InitStruct->DMA_Memory0BaseAddr;
}

/**
 * @author SurgeExperiments
 *
 * @brief Function to initialize I2C with DMA transfer capability.
 *
 * IMPORTANT: Only use this at the beginning of the program!
 *
 * @param Instance of type I2C_TypeDef representing the I2C channel. I2C1 for i2c number one ect.
 * @param target_array the array that the DMA should put received data into.
 * @param number_of_bytes the number of bytes that will be received from i2c and put into target_array
 */
void dma_init_i2c(I2C_TypeDef *I2Cx, volatile uint8_t *target_array, uint8_t number_of_bytes)
{

    /* Since this function runs @ the beginning of the program we don't de-init the stream as it'll be
     * inactive by default
     */
    uint32_t dma_perh_base_addr;
    uint32_t dma_channel = 0;
    DMA_Stream_TypeDef *dma_stream;
    IRQn_Type IRQ_stream;

    if (I2Cx == I2C1)
    {
        /* Set to i2c1 DR reg */
        dma_de_init(DMA1_Stream0);
        dma_perh_base_addr = 0x40005410;
        dma_stream = DMA1_Stream0; // this could also b stream  5 on the stm32f4
        dma_channel = DMA_Channel_1;
        IRQ_stream = DMA1_Stream0_IRQn;
    }
    else if (I2Cx == I2C2)
    {
        /* Set to i2c2 DR reg */
        dma_de_init(DMA1_Stream2);
        dma_perh_base_addr = 0x40005410;
        dma_stream = DMA1_Stream2; // This could also be stream 3 on the stm32f4
        dma_channel = 7;
        IRQ_stream = DMA1_Stream2_IRQn;
    }
    else if (I2Cx == I2C3)
    {
        /* Set to i2c3 DR reg */
        dma_de_init(DMA1_Stream1);
        dma_perh_base_addr = 0x40005410;
        dma_stream = DMA1_Stream1; // this could also be stream 2/channel 3 on the stm32f4
        dma_channel = 1;
        IRQ_stream = DMA1_Stream1_IRQn;
    }

    /* Set DMA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    DMA_InitTypeDef DMA_InitInstance;

    /* TODO: move to another function? */
    DMA_InitInstance.DMA_Channel = dma_channel;
    DMA_InitInstance.DMA_PeripheralBaseAddr = dma_perh_base_addr;
    DMA_InitInstance.DMA_Memory0BaseAddr = (uint32_t)target_array;
    DMA_InitInstance.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitInstance.DMA_BufferSize = number_of_bytes;
    DMA_InitInstance.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitInstance.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitInstance.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitInstance.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitInstance.DMA_Mode = DMA_Mode_Normal;
    DMA_InitInstance.DMA_Priority = DMA_Priority_Medium;
    DMA_InitInstance.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitInstance.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitInstance.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(dma_stream, &DMA_InitInstance);

    /* Allow transfer interrupt
     * DMA_IT_TC  & TRANSFER_IT_ENABLE_MASK failed?
     */
    dma_stream->CR |= (1 << 4);

    NVIC_InitTypeDef NVIC_init_instance;

    /*
     * this is DMA1_Stream0_IRQn= 11 (uint32_t)IRQ_stream
     * I2C1 connect to channel 7 of DMA1
     */
    NVIC_init_instance.NVIC_IRQChannel = IRQ_stream;
    NVIC_init_instance.NVIC_IRQChannelPreemptionPriority = 0x04;
    NVIC_init_instance.NVIC_IRQChannelSubPriority = 0x04;
    NVIC_init_instance.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_init_instance);
}
