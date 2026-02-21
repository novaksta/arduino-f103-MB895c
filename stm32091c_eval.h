/**
  ******************************************************************************
  * @file    stm32091c_eval.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32091C_EVAL's Leds, push-buttons
  *          and COM port hardware resources.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32091C_EVAL_H
#define __STM32091C_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32091C_EVAL STM32091C-EVAL
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include <arduino.h>

#if !defined (USE_STM32091C_EVAL)
 #define USE_STM32091C_EVAL
#endif


#if defined(HAL_SPI_MODULE_ENABLED)
/**
  * @brief  Definition for SPI Interface pins (SPI1 used)
  */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define EVAL_SPIx_TIMEOUT_MAX                 1000

#endif /* HAL_SPI_MODULE_ENABLED */
/**
  * @}
  */


/** @defgroup STM32091C_EVAL_COMPONENT STM32091C-EVAL COMPONENT
  * @{
  */
/*##################### LCD ###################################*/  
/* Chip Select macro definition */
#define LCD_CS_LOW()                    HAL_GPIO_WritePin(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()                   HAL_GPIO_WritePin(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET)

/* Link function for LCD peripheral over SPI */
void                      LCD_IO_Init(void);
void                      LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void                      LCD_IO_WriteReg(uint8_t Reg);
uint16_t                  LCD_IO_ReadData(uint16_t RegValue);
void                      LCD_Delay (uint32_t delay);

/* Link functions for SD Card peripheral */
void                     SD_IO_Init(void);
void                     SD_IO_CSState(uint8_t state);
void                     SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
uint8_t                  SD_IO_WriteByte(uint8_t Data);

/**
  * @}
  */  

/**
  * @}
  */  
 
/** @defgroup STM32091C_EVAL_Exported_Functions Exported Functions
  * @{
  */
uint32_t                BSP_GetVersion(void);
/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __STM32091C_EVAL_H */

