#include "stm32f4xx.h"
#include "string.h"
#include "network.h"
/* 
Cast pointer to function                                ( <__ret_type__> (*)(<__param__> 
Create new type of pointer to function                  typedef <__ret_type__> (*<__name__>)(<__param__>)
*/
#define CEN_PERIPH_ADDR         ((uint32_t) 0x00000001)
#define NFC_PERIPH_ADDR         ((uint32_t) 0x00000003)

typedef enum {
  NFC_ACTIVATE,
  NFC_DEACTIVATE,
  NFC_SETUP,
  NFC_TX_RX_SPEED_FAST,
  NFC_GET_UID
}NFC_CmdType;

//typedef void (*USART_RX_Callback)(uint8_t*);
//USART_RX_Callback usart_rx_callback = 0;

void CmdFrameSend(NFC_CmdType);
void USART_SendCmd(uint8_t*, uint8_t);
void NFC_Init();
void Button_Init();

uint8_t temp_buffer[64];
uint8_t tmp_idx = 0;
uint8_t len_resp_frame = 0;
uint8_t NFC_ENABLE_FLAG = 1;
uint8_t BtnPressed = 0;
uint16_t UID = 0;

NFC_CmdType curr_cmd;
MRFI_Packet packet;



int main()
{
  
  NetworkInit();
  if(isNetwork_init()){
    NFC_Init();
    Button_Init();
    packet.host_addr = NFC_PERIPH_ADDR;
    packet.dest_addr = CEN_PERIPH_ADDR;
  
    while(1) {
      MRFI_ServiceRun();
      switch (NFC_ENABLE_FLAG) {
        /*
         * NFC Enable command is NOT received
         */
        case 0:
          GPIO_ResetBits(GPIOE, GPIO_Pin_12);
          CmdFrameSend(NFC_DEACTIVATE);
        break;
        /*
         * NFC Enable command is received
         */
        case 1:
          GPIO_SetBits(GPIOE, GPIO_Pin_12);
          CmdFrameSend(NFC_ACTIVATE);
          
          if (BtnPressed) {
            CmdFrameSend(NFC_GET_UID);
          
            if (UID != 0) {
              packet.payload_length = 3;
              packet.payload[0] = (UID >> 8);
              packet.payload[1] = (UID);
              SendPacket(&packet);
            }
            memset(packet.payload, 0, MRFI_MAX_PAYLOAD_SIZE);
            UID = 0;
            packet.payload_length = 0;
            BtnPressed = 0;
          }
        break;
        
        default:
          NFC_ENABLE_FLAG = 0;
        break;
      
      
      }
    }
  }
}


/* Private Function ----------------------------------------------------------*/
void Button_Init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  /* Button EXTI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  /* LED */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);  
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
}

void NFC_Init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
  
  USART_InitStructure.USART_BaudRate = 115200;   
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
  USART_InitStructure.USART_StopBits = USART_StopBits_1;   
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART3, &USART_InitStructure);
  
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART3, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  CmdFrameSend(NFC_SETUP);
  CmdFrameSend(NFC_TX_RX_SPEED_FAST);
  CmdFrameSend(NFC_ACTIVATE);
}

void CmdFrameSend(NFC_CmdType cmd) {
  uint8_t len = 0;
  switch (cmd) {
    case NFC_ACTIVATE:
      uint8_t act_frame[8] = {0xAA, 0x0, 0x4, 0x0, 0x0, 0x01, 0x30, 0x35};
      len = 8;
      curr_cmd = NFC_ACTIVATE;
      USART_SendCmd(act_frame, len);
    break;
    
    case NFC_DEACTIVATE:
      uint8_t dact_frame[8] = {0xAA, 0x0, 0x04, 0x0, 0x0, 0x01, 0x31, 0x34};
      len = 8;
      curr_cmd = NFC_DEACTIVATE;
      USART_SendCmd(dact_frame, len);
    break;
    
    case NFC_SETUP:
      uint8_t setup_frame[8] ={0xAA, 0x0, 0x04, 0x0, 0x0, 0x0D, 0x0, 0x09};
      len = 8;
      curr_cmd = NFC_SETUP;
      USART_SendCmd(setup_frame, len);
    break;
    
    case NFC_GET_UID:
      uint8_t uid_get_frame[10] = {0xAA, 0x0, 0x06, 0x0, 0x0, 0x0D, 0x10, 0x0, 0x0, 0x1B}; 
      len = 10;
      curr_cmd = NFC_GET_UID;
      USART_SendCmd(uid_get_frame, len);
    break;
    
    case NFC_TX_RX_SPEED_FAST:
      uint8_t txrx_speed_frame[9] = {0xAA, 0x0, 05, 0x0, 0x0, 0x0D, 0x01, 0x11, 0x18};  
      len = 9;
      curr_cmd = NFC_TX_RX_SPEED_FAST;
      USART_SendCmd(txrx_speed_frame, len);    
    break;
      
    default:
      return;
    break;
  }
}

uint8_t UID_Retrival() {
  if(temp_buffer[9] == 0) {
    return 0;
  }
  else {
    UID = ((uint16_t)temp_buffer[9] << 8) | ((uint16_t)temp_buffer[10]);
    return 1;
  }
  
}

void USART_SendCmd(uint8_t* data, uint8_t len) {
  uint8_t idx = 0;
  while(len) {
    USART_SendData(USART3, data[idx]);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET);
    len--;
    idx++;
  }
  
  //Wait for complete response
  while(tmp_idx < len_resp_frame + 4  || (len_resp_frame == 0));
  if(curr_cmd == NFC_GET_UID) {
      UID_Retrival();
  }
  //Rest Buffer and index
  memset(temp_buffer, 0, 64);
  tmp_idx = 0;
  
}

/* Callback Function ---------------------------------------------------------*/
/*
*RF Receive event callback
*/
void MRFI_ReceiveCallback(mrfiPacket_t *packet) {
  
  uint32_t target_addr = (packet->frame[5] << 24) | (packet->frame[6] << 16) | (packet->frame[7] << 8) | (packet->frame[8]);
  
  if(target_addr == NFC_PERIPH_ADDR){
    (packet->frame[9] == 'O')?(NFC_ENABLE_FLAG = 1):(NFC_ENABLE_FLAG = 0);
    }
}
void EXTI0_IRQHandler(void) {
  if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
    BtnPressed = 1;
    EXTI_ClearITPendingBit(EXTI_Line0);    

  }
}
void USART3_IRQHandler(void) {
  uint8_t TmpData;

  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    TmpData = USART_ReceiveData(USART3);  
    
    temp_buffer[tmp_idx++] = TmpData;
    
    if(tmp_idx == 3) {
      len_resp_frame = (temp_buffer[tmp_idx - 2] << 8) | temp_buffer[tmp_idx - 1];
    }
    
  }
}
