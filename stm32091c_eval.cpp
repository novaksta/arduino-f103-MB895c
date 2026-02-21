#include "wiring_time.h"
/**
******************************************************************************
* @file    stm32091c_eval.c
* @author  MCD Application Team
* @brief   This file provides: a set of firmware functions to manage Leds,
*          push-button and COM ports
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

/* Includes ------------------------------------------------------------------*/
#include <arduino.h>
#include <SPI.h>
#include "stm32091c_eval.h"

#define CS_PIN (PB2)
#define LCD_CS_LOW() digitalWrite(CS_PIN, LOW)
#define   LCD_CS_HIGH() digitalWrite(CS_PIN, HIGH)

/** @addtogroup BSP
* @{
*/ 

/** @addtogroup STM32091C_EVAL
 * @brief This file provides firmware functions to manage Leds, push-buttons,  
 *        COM ports, SD card on SPI and temperature sensor (LM75) available on 
 *        STM32091C-EVAL evaluation board from STMicroelectronics.
 * @{
 */ 

/** @addtogroup STM32091C_EVAL_Common
  * @{
  */ 

/** @addtogroup STM32091C_EVAL_Private_Constants
 * @{
 */ 
/* LINK LCD */
#define START_BYTE         0x70
#define SET_INDEX          0x00
#define READ_STATUS        0x01
#define LCD_WRITE_REG      0x02
#define LCD_READ_REG       0x03

/**
* @brief STM32091C EVAL BSP Driver version number V2.0.8
*/
#define __STM32091C_EVAL_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32091C_EVAL_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32091C_EVAL_BSP_VERSION_SUB2   (0x08) /*!< [15:8]  sub2 version */
#define __STM32091C_EVAL_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __STM32091C_EVAL_BSP_VERSION         ((__STM32091C_EVAL_BSP_VERSION_MAIN << 24)\
|(__STM32091C_EVAL_BSP_VERSION_SUB1 << 16)\
  |(__STM32091C_EVAL_BSP_VERSION_SUB2 << 8 )\
    |(__STM32091C_EVAL_BSP_VERSION_RC))
/**
* @}
*/ 

#if defined(HAL_SPI_MODULE_ENABLED)
uint32_t SpixTimeout = EVAL_SPIx_TIMEOUT_MAX;    /*<! Value of Timeout when SPI communication fails */
SPIClass heval_Spi (PB15, PB14, PB13, PNUM_NOT_DEFINED); //MOSI, MISO, SCK,CS
//static SPI_HandleTypeDef heval_Spi;
#endif /* HAL_SPI_MODULE_ENABLED */


#if defined(HAL_SPI_MODULE_ENABLED)
/* SPIx bus function */
static void               SPIx_Init(void);
static void               SPIx_Write(uint8_t Value);

static uint32_t           SPIx_Read(void);
static void               SPIx_Error (void);

#endif /* HAL_SPI_MODULE_ENABLED */

/**
* @}
*/ 

/** @addtogroup STM32091C_EVAL_Exported_Functions
  * @{
  */ 

/**
* @brief  This method returns the STM32F091C EVAL BSP Driver revision
* @retval version : 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __STM32091C_EVAL_BSP_VERSION;
}

#if defined(HAL_SPI_MODULE_ENABLED)
/******************************* SPI Routines *********************************/

/**
  * @brief SPIx Bus initialization
  * @retval None
  */
static void SPIx_Init(void)
{
  pinMode(CS_PIN, OUTPUT);
  heval_Spi.begin();
  heval_Spi.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE3));
  return;
  /* Done above
  if(HAL_SPI_GetState(&heval_Spi) == HAL_SPI_STATE_RESET)
  {
    // SPI Config 
    heval_Spi.Instance = EVAL_SPIx;
    // SPI baudrate is set to 12 MHz (PCLK1/SPI_BaudRatePrescaler = 48/4 = 12 MHz) 
    // to verify these constraints:
    // HX8347D LCD SPI interface max baudrate is  50MHz for write and 6.66MHz for read
    // PCLK1 frequency is set to 48 MHz 
    // - SD card SPI interface max baudrate is 25MHz for write/read
    heval_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    heval_Spi.Init.Direction = SPI_DIRECTION_2LINES;
    heval_Spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    heval_Spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    heval_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    heval_Spi.Init.CRCPolynomial = 7;
    heval_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
    heval_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    heval_Spi.Init.NSS = SPI_NSS_SOFT;
    heval_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
    heval_Spi.Init.Mode = SPI_MODE_MASTER;
    
    HAL_SPI_MspInit(EVAL_SPIx);
    HAL_SPI_Init(&heval_Spi);
  }*/
}

/**
  * @brief SPI Read 4 bytes from device
  * @retval Read data
  */
static uint32_t SPIx_Read(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t value = 0xFFFFFFFFU;
  

  //status = HAL_SPI_TransmitReceive(&heval_Spi, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SpixTimeout);
  heval_Spi.transfer((byte *)&value, 4);
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }

  return value;
}


/**
  * @brief SPI Write a byte to device
  * @param Value value to be written
  * @retval None
  */
static void SPIx_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data;

  heval_Spi.transfer((byte) Value);
  //status = HAL_SPI_TransmitReceive(&heval_Spi, (uint8_t*) &Value, &data, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}


/**
  * @brief SPI error treatment function
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  //HAL_SPI_DeInit(&heval_Spi);

  /* Re- Initiaize the SPI communication BUS */
  SPIx_Init();
}


#endif /*HAL_SPI_MODULE_ENABLED*/

/**
  * @}
  */

/** @defgroup STM32091C_EVAL_LINK_OPERATIONS Link Operation functions
  * @{
  */

/******************************************************************************
LINK OPERATIONS
*******************************************************************************/


/********************************* LINK LCD ***********************************/

/**
* @brief  Configures the LCD_SPI interface.
* @retval None
*/
void LCD_IO_Init(void)
{

  /* Set or Reset the control line */
  LCD_CS_LOW();
  LCD_CS_HIGH();
  
  SPIx_Init();
}

/**
* @brief  Write register value.
* @param  pData Pointer on the register value
* @param  Size Size of byte to transmit to the register
* @retval None
*/
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
  uint32_t counter = 0;
  
  /* Reset LCD control line(/CS) and Send data */  
  LCD_CS_LOW();
  
  /* Send Start Byte */
  SPIx_Write(START_BYTE | LCD_WRITE_REG);

  if (Size == 1)
  {
    /* Only 1 byte to be sent to LCD - general interface can be used */
    /* Send Data */
    SPIx_Write(*pData);
  }
  else
  {
    for (counter = 0; counter < Size; counter += 2)
    {
      SPIx_Write(pData[counter +1]);
      SPIx_Write(pData[counter]);
      
      /*while(((heval_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }  
      // Need to invert bytes for LCD
      *((__IO uint8_t*)&heval_Spi.Instance->DR) = *(pData+1);
      
      while(((heval_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }  
      *((__IO uint8_t*)&heval_Spi.Instance->DR) = *pData;
      counter--;
      pData += 2;*/
    }
    
    /* Wait until the bus is ready before releasing Chip select */
    /*while(((heval_Spi.Instance->SR) & SPI_FLAG_BSY) != RESET)
    {
    }*/
  }

  /* Empty the Rx fifo */

  /* Reset LCD control line(/CS) and Send data */  
  LCD_CS_HIGH();
}

/**
* @brief  Writes address on LCD register.
* @param  Reg Register to be written
* @retval None
*/
void LCD_IO_WriteReg(uint8_t Reg) 
{
  /* Reset LCD control line(/CS) and Send command */
  LCD_CS_LOW();
  
  /* Send Start Byte */
  SPIx_Write(START_BYTE | SET_INDEX);
  
  /* Write 16-bit Reg Index (High Byte is 0) */
  SPIx_Write(0x00);
  SPIx_Write(Reg);
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
* @brief  Read data from LCD data register.
* @param  Reg Register to be read
* @retval readvalue
*/
uint16_t LCD_IO_ReadData(uint16_t Reg) 
{
  uint32_t readvalue = 0;
  
  /* Send Reg value to Read */
  LCD_IO_WriteReg(Reg);
  
  /* Reset LCD control line(/CS) and Send command */
  LCD_CS_LOW();
  
  /* Send Start Byte */
  SPIx_Write(START_BYTE | LCD_READ_REG);
  
  /* Read Upper Byte */
  SPIx_Write(0xFF);
  readvalue = SPIx_Read();
  readvalue = readvalue << 8;
  readvalue |= SPIx_Read();
  
  HAL_Delay(10);
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
  return readvalue;
}

/**
* @brief  Wait for loop in ms.
* @param  Delay in ms.
* @retval None
*/
void LCD_Delay (uint32_t Delay)
{
  delay(Delay);
  //HAL_Delay (Delay);
}



