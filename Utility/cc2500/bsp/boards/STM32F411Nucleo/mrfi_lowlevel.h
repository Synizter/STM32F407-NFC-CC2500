/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MRFI_LOWLEVEL_H
#define MRFI_LOWLEVEL_H

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Exported types ------------------------------------------------------------*/
typedef struct mrfiSignal
{
  int sem;
}mrfiSignal_t;

/* Exported constants --------------------------------------------------------*/
/*
 * @brief  MRFI SPI port macro for STM32 Std library.
 */
#define MRFI_SPI                           SPI3
#define MRFI_SPI_CLK                       RCC_APB1Periph_SPI3
#define MRFI_SPI_CLK_CMD                   RCC_APB1PeriphClockCmd
#define MRFI_SPI_IRQn                      SPI3_IRQn
#define MRFI_SPI_IRQHandler                SPI3_IRQHandler

#define MRFI_SPI_SCK_PIN                   GPIO_Pin_10
#define MRFI_SPI_SCK_GPIO_PORT             GPIOC
#define MRFI_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOC
#define MRFI_SPI_SCK_GPIO_CLK_CMD          RCC_AHB1PeriphClockCmd
#define MRFI_SPI_SCK_SOURCE                GPIO_PinSource10
#define MRFI_SPI_SCK_AF                    GPIO_AF_SPI3

#define MRFI_SPI_MISO_PIN                  GPIO_Pin_11
#define MRFI_SPI_MISO_GPIO_PORT            GPIOC
#define MRFI_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define MRFI_SPI_MISO_GPIO_CLK_CMD         RCC_AHB1PeriphClockCmd
#define MRFI_SPI_MISO_SOURCE               GPIO_PinSource11
#define MRFI_SPI_MISO_AF                   GPIO_AF_SPI3

#define MRFI_SPI_MOSI_PIN                  GPIO_Pin_12
#define MRFI_SPI_MOSI_GPIO_PORT            GPIOC
#define MRFI_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define MRFI_SPI_MOSI_GPIO_CLK_CMD         RCC_AHB1PeriphClockCmd
#define MRFI_SPI_MOSI_SOURCE               GPIO_PinSource12
#define MRFI_SPI_MOSI_AF                   GPIO_AF_SPI3
//WAIT EDITED
#define MRFI_CS_PIN                        GPIO_Pin_8
#define MRFI_CS_GPIO_PORT                  GPIOE
#define MRFI_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOE
#define MRFI_CS_GPIO_CLK_CMD               RCC_AHB1PeriphClockCmd

/* 
 * GDO0 Pin Configuration
 */
#define MRFI_GDO0_PIN                      GPIO_Pin_10
#define MRFI_GDO0_GPIO_PORT                GPIOE
#define MRFI_GDO0_GPIO_CLK                 RCC_AHB1Periph_GPIOE
#define MRFI_GDO0_GPIO_CLK_CMD             RCC_AHB1PeriphClockCmd
#define MRFI_GDO0_EXTI_LINE                EXTI_Line10
#define MRFI_GDO0_EXTI_PIN_SOURCE          EXTI_PinSource10
#define MRFI_GDO0_EXTI_PORT_SOURCE         EXTI_PortSourceGPIOE
#define MRFI_GDO0_EXTI_IRQn                EXTI15_10_IRQn
#define MRFI_GDO0_EXTI_IRQHandler          EXTI15_10_IRQHandler

///* ------------------------------------------------------------------------------------------------
// *                     GDO2 Pin Configuration
// * ------------------------------------------------------------------------------------------------
// */
//
//#define MRFI_GDO2_PIN                      GPIO_Pin_6                  // PE.06
//#define MRFI_GDO2_GPIO_PORT                GPIOD                       // GPIOD
//#define MRFI_GDO2_GPIO_CLK                 RCC_AHB1Periph_GPIOD
//#define MRFI_GDO2_GPIO_CLK_CMD             RCC_AHBPeriphClockCmd
//#define MRFI_GDO2_EXTI_LINE                EXTI_Line6
//#define MRFI_GDO2_EXTI_PIN_SOURCE          EXTI_PinSource6
//#define MRFI_GDO2_EXTI_PORT_SOURCE         EXTI_PortSourceGPIOD
//#define MRFI_GDO2_EXTI_IRQn                EXTI9_5_IRQn
//#define MRFI_GDO2_EXTI_IRQHandler          EXTI9_5_IRQHandler

/* Exported macro ------------------------------------------------------------*/
/*
 * GDO0 Macro
 */
#define MRFI_GDO0_PIN_IS_HIGH()               (GPIO_ReadInputDataBit(MRFI_GDO0_GPIO_PORT, MRFI_GDO0_PIN))

/* 
 * CS Pin macro.
 */
#define MRFI_SPI_DRIVE_CSN_HIGH()             GPIO_SetBits(MRFI_SPI_SCK_GPIO_PORT, MRFI_CS_PIN)
#define MRFI_SPI_DRIVE_CSN_LOW()              GPIO_ResetBits(MRFI_SPI_SCK_GPIO_PORT, MRFI_CS_PIN)
#define MRFI_SPI_CSN_IS_HIGH()                GPIO_ReadOutputDataBit(MRFI_SPI_SCK_GPIO_PORT, MRFI_CS_PIN)

/* 
 * MISO Pin macro.
 */
#define MRFI_SPI_SO_IS_HIGH()                 GPIO_ReadInputDataBit(MRFI_SPI_MISO_GPIO_PORT, MRFI_SPI_MISO_PIN)

/* Exported functions ------------------------------------------------------- */
/*
 * SPI Lowlevel function.
 */
void    MRFI_SPI_Init(void);
uint8_t MRFI_SPI_IsInit(void);
void    MRFI_SPI_IOInit(void);
void    MRFI_SPI_WriteByte(uint8_t Data);
uint8_t MRFI_SPI_ReadByte(void);
void    MRFI_SPI_WaitTransferDone(void);
void    MRFI_SPI_AcquireMutex(void);
void    MRFI_SPI_ReleaseMutex(void);

/* 
 * GDO0 Lowlevel function.
 */
void    MRFI_GDO0_ConfigAsInput(void);
void    MRFI_GDO0_EnableInterrupt(void);
void    MRFI_GDO0_DisableInterrupt(void);
void    MRFI_GDO0_ConfigInterruptRisingEdge(void);
void    MRFI_GDO0_ConfigInterruptFallingEdge(void);
uint8_t MRFI_GDO0_IsInterruptEnable(void);
void    MRFI_GDO0_ClearInterruptFlag(void);
uint8_t MRFI_GDO0_GetInterruptFlag(void);
void    MRFI_GDO0_SignalInit(mrfiSignal_t *signal);
uint8_t MRFI_GDO0_CheckSignal(mrfiSignal_t *signal);
void    MRFI_GDO0_ReleaseSignal(mrfiSignal_t *signal);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MRFI_LOWLEVEL_H */
