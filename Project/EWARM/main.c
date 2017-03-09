#include "stm32f4xx.h"
/* 
Cast pointer to function                                ( <__ret_type__> (*)(<__param__> 
Create new type of pointer to function                  typedef <__ret_type__> (*<__name__>)(<__param__>)
*/

enum NFC_CmdType{
  NFC_ACTIVATE,
  NFC_DEACTIVATE,
  NFC_SET_RESP_RATE_LOW,
  NFC_GET_UID;
};

typedef void (*USART_RX_Callback)(uint8_t);
USART_RX_Callback usart_rx_callback = 0;
void CommandRetrival(uint8_t);
void CmdFrameCreate(NFC_CmdState);
void USART_SendCmd(uint8_t*, uint8_t);

uint8_t temp_buffer[64];
uint8_t buffer[16];
uint8_t cmd_comp = 0;

int main()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  
  USART_InitStructure.USART_BaudRate = 115200;   
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
  USART_InitStructure.USART_StopBits = USART_StopBits_1;   
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &USART_InitStructure);
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  usart_rx_callback = CommandRetrival;
  
  CmdFrameCreate(NFC_SET_RESP_RATE_LOW);
  while(1) {
    
  }
  
}

void CmdFrameCreate(NFC_CmdType cmd) {
  uint8_t* tmp_frame;
  uint8_t len = 0;
  switch (cmd) {
    case NFC_ACTIVATE:
      
    break;
    
    case NFC_DEACTIVATE:
    break;
    
    case NFC_ACTIVATE:
    break;
    
    case NFC_SET_RESP_RATE_LOW:
      uint8_t frame[8] = {0xAA, 0x00, 0x04, 0x00, 0x00, 0x0D, 0x00};
      frame[7] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6]; 
      len = 8;
    break;
    
    case NFC_GET_UID:
    break;
    
    default:
      return;
    break;
  }

}

void USART_SendCmd(uint8_t* data, uint8_t len) {
  uint8_t idx = 0;
  while(len) {
    USART_SendData(USART1, data[idx++]);
    while(USART_GetFlagStatus(USART_FLAG_TXE != SET);
    len--;
  }

}
void USART1_IRQHandler(void) {
  uint8_t TmpData;
  // USART RX Not Empty Interrupt
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    TmpData = USART_ReceiveData(ESP8266_USART);
    temp_buffer[tmp_idx++] = TmpData
    (tmp_idx == 64)?(tmp_idx = 0):(tmp_idx);
//    if(usart_rx_callback && cmd_comp){
//      usart_rx_callback(TmpData);
//    }
    
  }
}
