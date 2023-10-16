/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "spi_flash.h"

#include "stm32f4xx_hal.h"

#define SPI_FLASH_CS_PIN GPIO_PIN_4
#define SPI_FLASH_SCK_PIN GPIO_PIN_5
#define SPI_FLASH_MISO_PIN GPIO_PIN_6
#define SPI_FLASH_MOSI_PIN GPIO_PIN_7

#define FLASH_DUMMY_BYTE 0xA5

#define FLASH_READY 0
#define FLASH_BUSY  1
#define FLASH_TIMEOUT 2

/* 1st addressing cycle */
#define ADDR_1st_CYCLE(ADDR) (uint8_t)((ADDR)& 0xFF)
/* 2nd addressing cycle */
#define ADDR_2nd_CYCLE(ADDR) (uint8_t)(((ADDR)& 0xFF00) >> 8)
/* 3rd addressing cycle */
#define ADDR_3rd_CYCLE(ADDR) (uint8_t)(((ADDR)& 0xFF0000) >> 16)
/* 4th addressing cycle */
#define ADDR_4th_CYCLE(ADDR) (uint8_t)(((ADDR)& 0xFF000000) >> 24)

#define UNDEFINED_CMD 0xFF

HAL_SPI_StateTypeDef state;
SPI_HandleTypeDef hspi1;

typedef struct __attribute__((__packed__))
{
    uint8_t page_offset;
    uint8_t read_cmd;
    uint8_t read_id_cmd;
    uint8_t write_cmd;
    uint8_t write_en_cmd;
    uint8_t erase_cmd;
    uint8_t status_cmd;
    uint8_t busy_bit;
    uint8_t busy_state;
    uint32_t freq;
} spi_conf_t;

static spi_conf_t spi_conf;

static void spi_flash_gpio_init()
{
    GPIO_InitTypeDef gpio_init;

 //   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Enable SPI peripheral clock */
   // RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
    /* Configure SPI SCK pin */
 //   hspi1.GPIO_Pin = SPI_FLASH_SCK_PIN;
  //  hspi1.GPIO_Speed = GPIO_Speed_50MHz;
 //   hspi1.GPIO_Mode = GPIO_Mode_AF_PP;
 //   GPIO_Init(GPIOA, &gpio_init);

    /* Configure SPI MOSI pin */
 //   hspi1.GPIO_Pin = SPI_FLASH_MOSI_PIN;
 //   GPIO_Init(GPIOA, &gpio_init);

    /* Configure SPI MISO pin */
 //  hspi1.GPIO_Pin = SPI_FLASH_MISO_PIN;
  //  hspi1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //  GPIO_Init(GPIOA, &gpio_init);
  
    /* Configure SPI CS pin */
  //  hspi1.GPIO_Pin = SPI_FLASH_CS_PIN;
 //   hspi1.GPIO_Mode = GPIO_Mode_Out_PP;
  //  GPIO_Init(GPIOA, &gpio_init);

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&gpio_init) != HAL_OK)
  {
    Error_Handler();
  }
}

static void spi_flash_gpio_uninit()
{
    GPIO_InitTypeDef gpio_init;

    /* Disable SPI peripheral clock */
  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);

    /* Disable SPI SCK pin */
  //  hspi1.GPIO_Pin = SPI_FLASH_SCK_PIN;
  //  hspi1.GPIO_Mode = GPIO_MODE_AF_INPUT
  //  GPIO_Init(GPIOA, &gpio_init);

    /* Disable SPI MISO pin */
  //  hspi1.GPIO_Pin = SPI_FLASH_MISO_PIN;
  //  GPIO_Init(GPIOA, &gpio_init);

    /* Disable SPI MOSI pin */
   // hspi1.GPIO_Pin = SPI_FLASH_MOSI_PIN;
   // GPIO_Init(GPIOA, &gpio_init);

    /* Disable SPI CS pin */
   // hspi1.GPIO_Pin = SPI_FLASH_CS_PIN;
   // GPIO_Init(GPIOA, &gpio_init);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_SPI1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#if defined (STM32F407xx)
   GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
#else
#endif
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&gpio_init) != HAL_OK)
  {
    Error_Handler();
  }
}

static inline void spi_flash_select_chip()
{
    //GPIO_ResetBits(GPIOA, SPI_FLASH_CS_PIN);
    HAL_GPIO_WritePin(GPIOA,SPI_FLASH_CS_PIN,0);
}

static inline void spi_flash_deselect_chip()
{
    //GPIO_SetBits(GPIOA, SPI_FLASH_CS_PIN);
     HAL_GPIO_WritePin(GPIOA,SPI_FLASH_CS_PIN,1);
}

static uint16_t spi_flash_get_baud_rate_prescaler(uint32_t spi_freq_khz)
{
    uint32_t system_clock_khz = SystemCoreClock / 1000;

    if (spi_freq_khz >= system_clock_khz / 2)
        return SPI_BAUDRATEPRESCALER_2 ;
    else if (spi_freq_khz >= system_clock_khz / 4)
        return SPI_BAUDRATEPRESCALER_4;
    else if (spi_freq_khz >= system_clock_khz / 8)
        return SPI_BAUDRATEPRESCALER_8;
    else if (spi_freq_khz >= system_clock_khz / 16)
        return SPI_BAUDRATEPRESCALER_16;
    else if (spi_freq_khz >= system_clock_khz / 32)
        return SPI_BAUDRATEPRESCALER_32;
    else if (spi_freq_khz >= system_clock_khz / 64)
        return SPI_BAUDRATEPRESCALER_64;
    else if (spi_freq_khz >= system_clock_khz / 128)
        return SPI_BAUDRATEPRESCALER_128;
    else
        return SPI_BAUDRATEPRESCALER_256;
}

static int spi_flash_init(void *conf, uint32_t conf_size)
{
    SPI_InitTypeDef spi_init;

    if (conf_size < sizeof(spi_conf_t))
        return -1; 
    spi_conf = *(spi_conf_t *)conf;

    spi_flash_gpio_init();

    spi_flash_deselect_chip();

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

   if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

    /* Configure SPI */
    /*
    spi_init.Direction = SPI_Direction_2Lines_FullDuplex;
    spi_init.Mode = SPI_Mode_Master;
    spi_init.DataSize = SPI_DataSize_8b;
    spi_init.CPOL = SPI_CPOL_High;
    spi_init.CPHA = SPI_CPHA_2Edge;
    spi_init.NSS = SPI_NSS_SOFT;
    spi_init.BaudRatePrescaler = 
    //spi_flash_get_baud_rate_prescaler(spi_conf.freq);
    spi_init.FirstBit = SPI_FIRSTBIT_MSB;
    spi_init.CRCPolynomial = 7;
    SPI_Init(SPI1, &spi_init);
*/
    /* Enable SPI */
   // SPI_Cmd(SPI1, ENABLE);

    return 0;
}

static void spi_flash_uninit()
{
    spi_flash_gpio_uninit();

    /* Disable SPI */
  //  SPI_Cmd(SPI1, DISABLE);
}

static uint8_t spi_flash_send_byte(uint8_t byte)
{
    uint16_t data;
    /* Loop while DR register in not emplty */
    //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    state = HAL_SPI_GetState(&hspi1);
    while (state ==  HAL_SPI_STATE_RESET );
    /* Send byte through the SPI1 peripheral to generate clock signal */
   // SPI_I2S_SendData(SPI1, byte);
    HAL_SPI_Transmit(&hspi1,byte,sizeof(byte),0);
    /* Wait to receive a byte */
    
    //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    state = HAL_SPI_GetState(&hspi1);
    while (state ==  HAL_SPI_STATE_RESET );
    /* Return the byte read from the SPI bus */
    HAL_SPI_Receive(&hspi1,data,sizeof(data),0);
    //return SPI_I2S_ReceiveData(SPI1);
    return data;
}

static inline uint8_t spi_flash_read_byte()
{
    return spi_flash_send_byte(FLASH_DUMMY_BYTE);
}

static uint32_t spi_flash_read_status()
{
    uint8_t status;
    uint32_t flash_status = FLASH_READY;

    spi_flash_select_chip();

    spi_flash_send_byte(spi_conf.status_cmd);

    status = spi_flash_read_byte();

    if (spi_conf.busy_state == 1 && (status & (1 << spi_conf.busy_bit)))
        flash_status = FLASH_BUSY;
    else if (spi_conf.busy_state == 0 && !(status & (1 << spi_conf.busy_bit)))
        flash_status = FLASH_BUSY;

    spi_flash_deselect_chip();

    return flash_status;
}

static uint32_t spi_flash_get_status()
{
    uint32_t status, timeout = 0x1000000;

    status = spi_flash_read_status();

    /* Wait for an operation to complete or a TIMEOUT to occur */
    while (status == FLASH_BUSY && timeout)
    {
        status = spi_flash_read_status();
        timeout --;
    }

    if (!timeout)
        status = FLASH_TIMEOUT;

    return status;
}

static void spi_flash_read_id(chip_id_t *chip_id)
{
    spi_flash_select_chip();

    spi_flash_send_byte(spi_conf.read_id_cmd);

    chip_id->maker_id = spi_flash_read_byte();
    chip_id->device_id = spi_flash_read_byte();
    chip_id->third_id = spi_flash_read_byte();
    chip_id->fourth_id = spi_flash_read_byte();

    spi_flash_deselect_chip();
}

static void spi_flash_write_enable()
{
    if (spi_conf.write_en_cmd == UNDEFINED_CMD)
        return;

    spi_flash_select_chip();
    spi_flash_send_byte(spi_conf.write_en_cmd);
    spi_flash_deselect_chip();
}

static void spi_flash_write_page_async(uint8_t *buf, uint32_t page,
    uint32_t page_size)
{
    uint32_t i;

    spi_flash_write_enable();

    spi_flash_select_chip();

    spi_flash_send_byte(spi_conf.write_cmd);

    page = page << spi_conf.page_offset;

    spi_flash_send_byte(ADDR_3rd_CYCLE(page));
    spi_flash_send_byte(ADDR_2nd_CYCLE(page));
    spi_flash_send_byte(ADDR_1st_CYCLE(page));

    for (i = 0; i < page_size; i++)
        spi_flash_send_byte(buf[i]);

    spi_flash_deselect_chip();
}

static uint32_t spi_flash_read_data(uint8_t *buf, uint32_t page,
    uint32_t page_offset, uint32_t data_size)
{
    uint32_t i, addr = (page << spi_conf.page_offset) + page_offset;

    spi_flash_select_chip();

    spi_flash_send_byte(spi_conf.read_cmd);

    spi_flash_send_byte(ADDR_3rd_CYCLE(addr));
    spi_flash_send_byte(ADDR_2nd_CYCLE(addr));
    spi_flash_send_byte(ADDR_1st_CYCLE(addr));

    /* AT45DB requires write of dummy byte after address */
    spi_flash_send_byte(FLASH_DUMMY_BYTE);

    for (i = 0; i < data_size; i++)
        buf[i] = spi_flash_read_byte();

    spi_flash_deselect_chip();

    return FLASH_READY;
}

static uint32_t spi_flash_read_page(uint8_t *buf, uint32_t page,
    uint32_t page_size)
{
    return spi_flash_read_data(buf, page, 0, page_size);
}

static uint32_t spi_flash_read_spare_data(uint8_t *buf, uint32_t page,
    uint32_t offset, uint32_t data_size)
{
    return FLASH_STATUS_INVALID_CMD;
}

static uint32_t spi_flash_erase_block(uint32_t page)
{
    uint32_t addr = page << spi_conf.page_offset;

    spi_flash_write_enable();

    spi_flash_select_chip();

    spi_flash_send_byte(spi_conf.erase_cmd);

    spi_flash_send_byte(ADDR_3rd_CYCLE(addr));
    spi_flash_send_byte(ADDR_2nd_CYCLE(addr));
    spi_flash_send_byte(ADDR_1st_CYCLE(addr));

    spi_flash_deselect_chip();

    return spi_flash_get_status();
}

static inline bool spi_flash_is_bb_supported()
{
    return false;
}

flash_hal_t hal_spi =
{
    .init = spi_flash_init,
    .uninit = spi_flash_uninit,
    .read_id = spi_flash_read_id,
    .erase_block = spi_flash_erase_block,
    .read_page = spi_flash_read_page,
    .read_spare_data = spi_flash_read_spare_data, 
    .write_page_async = spi_flash_write_page_async,
    .read_status = spi_flash_read_status,
    .is_bb_supported = spi_flash_is_bb_supported
};
