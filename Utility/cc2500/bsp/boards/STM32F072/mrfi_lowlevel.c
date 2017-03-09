/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "mrfi_lowlevel.h"
#include "mrfi_defs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t MRFI_SPI_InitState = 0;     /* SPI init state flag. */
static volatile uint8_t MRFI_SPI_GDO0IntState = 0;  /* GDO0 Interrupt init state flag. */
static volatile uint8_t MRFI_SPI_LastReceiveBytes;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
extern void MRFI_GpioIsr(void);

/*
 * @brief  Initializes the MRFI SPI Interface.
 *
 *         Radio SPI Specifications
 *         -----------------------------------------------
 *         SPI Mode        :  Master
 *         Max SPI Clock   :  1 MHz
 *         Data Order      :  MSB transmitted first
 *         Clock Polarity  :  low when idle
 *         Clock Phase     :  sample leading edge
 *         Data width      :  8-bit
 *
 * @param  None
 * @retval None
 */
void MRFI_SPI_Init(void)
{
  SPI_InitTypeDef   SPI_InitStructure;
  
  /*!< MRFI_SPI Periph clock enable */
  MRFI_SPI_CLK_CMD(MRFI_SPI_CLK, ENABLE); 

  /* Connect PXx to MRFI_SPI_SCK */
  GPIO_PinAFConfig(MRFI_SPI_SCK_GPIO_PORT, MRFI_SPI_SCK_SOURCE, MRFI_SPI_SCK_AF);

  /* Connect PXx to MRFI_SPI_MISO */
  GPIO_PinAFConfig(MRFI_SPI_MISO_GPIO_PORT, MRFI_SPI_MISO_SOURCE, MRFI_SPI_MISO_AF); 

  /* Connect PXx to MRFI_SPI_MOSI */
  GPIO_PinAFConfig(MRFI_SPI_MOSI_GPIO_PORT, MRFI_SPI_MOSI_SOURCE, MRFI_SPI_MOSI_AF);  
  
  /*!< MRFI_SPI Config */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(MRFI_SPI, &SPI_InitStructure);
  
  SPI_RxFIFOThresholdConfig(MRFI_SPI, SPI_RxFIFOThreshold_QF);
  
  SPI_Cmd(MRFI_SPI, ENABLE); /*!< MRFI_SPI enable */
  
  MRFI_SPI_InitState = 1;
}

/*
 * @brief  Check MRFI SPI Interface is enabled.
 * @param  None
 * @retval If MRFI SPI is already initialize, 1 is returned.
 *         If not, 0 is returned.
 */
uint8_t MRFI_SPI_IsInit(void)
{
  return MRFI_SPI_InitState;
}

/*
 * @brief  MRFI SPI IO Init.
 * @param  None
 * @retval None
 */
void MRFI_SPI_IOInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  MRFI_SPI_SCK_GPIO_CLK_CMD(MRFI_SPI_SCK_GPIO_CLK ,ENABLE);
  MRFI_SPI_MOSI_GPIO_CLK_CMD(MRFI_SPI_MOSI_GPIO_CLK ,ENABLE);
  MRFI_SPI_MISO_GPIO_CLK_CMD(MRFI_SPI_MISO_GPIO_CLK ,ENABLE);
  MRFI_CS_GPIO_CLK_CMD(MRFI_CS_GPIO_CLK ,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = MRFI_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(MRFI_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = MRFI_SPI_MOSI_PIN;
  GPIO_Init(MRFI_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = MRFI_SPI_MISO_PIN;
  GPIO_Init(MRFI_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = MRFI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_Init(MRFI_CS_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * @brief  Send SPI data byte.
 * @param  Data: 
 * @retval None.
 */
void MRFI_SPI_WriteByte(uint8_t Data)
{
  /* Wait until the transmit buffer is empty */
  while(SPI_I2S_GetFlagStatus(MRFI_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
  }
  
  /* Send the byte */
  SPI_SendData8(MRFI_SPI, Data);
}

/*
 * @brief  Receive SPI data byte.
 * @param  None
 * @retval Received data.
 */
uint8_t MRFI_SPI_ReadByte(void)
{
  /* Return the byte read from the SPI bus */
  return MRFI_SPI_LastReceiveBytes;
}

/*
 * @brief  Waiting for SPI transfer.
 * @param  None
 * @retval Received data.
 */
void MRFI_SPI_WaitTransferDone(void)
{
  /* Wait to receive a byte */
  while(SPI_I2S_GetFlagStatus(MRFI_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }
  
  /* Return the byte read from the SPI bus */
  MRFI_SPI_LastReceiveBytes = SPI_ReceiveData8(MRFI_SPI);
}

/*
 * @brief  MRFI Sync pin init (GDO0 Pin)
 * @param  None
 * @retval None
 */
void MRFI_GDO0_ConfigAsInput(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  MRFI_GDO0_GPIO_CLK_CMD(MRFI_GDO0_GPIO_CLK ,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = MRFI_GDO0_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(MRFI_GDO0_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * @brief  Initializes GDO0 external interrupt
 * @param  None
 * @retval None
 */
void MRFI_GDO0_EnableInterrupt(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(MRFI_GDO0_EXTI_PORT_SOURCE, MRFI_GDO0_EXTI_PIN_SOURCE);
  
  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = MRFI_GDO0_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = MRFI_GDO0_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  MRFI_SPI_GDO0IntState = 1;
}

/*
 * @brief  Deinitializes GDO0 external interrupt
 * @param  None
 * @retval None
 */
void MRFI_GDO0_DisableInterrupt(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = MRFI_GDO0_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = MRFI_GDO0_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  MRFI_SPI_GDO0IntState = 0;
}

/*
 * @brief  Change GDO0 external interrupt trigger to rising edge.
 * @param  None
 * @retval None
 */
void MRFI_GDO0_ConfigInterruptRisingEdge(void)
{
  MRFI_GDO0_EnableInterrupt();
}

/*
 * @brief  Change GDO0 external interrupt trigger to falling edge.
 * @param  None
 * @retval None
 */
void MRFI_GDO0_ConfigInterruptFallingEdge(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(MRFI_GDO0_EXTI_PORT_SOURCE, MRFI_GDO0_EXTI_PIN_SOURCE);
  
  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = MRFI_GDO0_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = MRFI_GDO0_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  MRFI_SPI_GDO0IntState = 1;
}

/**
  * @brief  Check GDO0 external interrupt is enabled.
  * @param  None
  * @retval GDO0 interrupt state.
  */
uint8_t MRFI_GDO0_IsInterruptEnable(void)
{
  return MRFI_SPI_GDO0IntState;
}

/**
  * @brief  Clear GDO0 external interrupt flag.
  * @param  None
  * @retval None
  */
void MRFI_GDO0_ClearInterruptFlag(void)
{
  EXTI_ClearFlag(MRFI_GDO0_EXTI_LINE);
}

/**
  * @brief  Check GDO0 external interrupt flag is set.
  * @param  None
  * @retval GDO0 interrupt flags.
  */
uint8_t MRFI_GDO0_GetInterruptFlag(void)
{
  if(EXTI_GetFlagStatus(MRFI_GDO0_EXTI_LINE) != RESET)
    return 1;
  else
    return 0;
}

/**************************************************************************************************
 * @fn          GDO0 Interrupt
 *
 * @brief       -
 *
 * @param       -
 *
 * @return      -
 **************************************************************************************************
 */
void MRFI_GDO0_EXTI_IRQHandler(void)
{
  /*
   *  This ISR is easily replaced.  The new ISR must simply
   *  include the following function call.
   */
  /*Check if interrupt come from line 0*/
  //EDITED
  if(EXTI_GetFlagStatus(MRFI_GDO0_EXTI_LINE) != RESET)
    MRFI_GpioIsr();
}


void MRFI_SPI_AcquireMutex(void)
{
  
}

void MRFI_SPI_ReleaseMutex(void)
{
  
}

void MRFI_GDO0_SignalInit(mrfiSignal_t *signal)
{
  signal->sem = 0;
}

uint8_t MRFI_GDO0_CheckSignal(mrfiSignal_t *signal)
{
  if(signal->sem)
  {
    signal->sem = 0;
    return 1;
  }
  else
    return 0;
}

void MRFI_GDO0_ReleaseSignal(mrfiSignal_t *signal)
{
  signal->sem = 1;
}

/**************************************************************************************************
 */


