#include "stm32f4xx.h"
#include "string.h"
#include "network.h"
/* 
Cast pointer to function                                ( <__ret_type__> (*)(<__param__> 
Create new type of pointer to function                  typedef <__ret_type__> (*<__name__>)(<__param__>)
*/
#define CEN_PERIPH_ADDR         ((uint32_t) 0x00000001)
#define BUZZ_PERIPH_ADDR        ((uint32_t) 0x00000002)
#define PERIOD                  30
#define DUTY                    50
#define PULSE_WIDTH             (int)(PERIOD * DUTY / 100)

void Buzzer_Init();

uint32_t PWM_Counter = 0;
uint8_t BUZZER_ENABLE_FLAG = 0;
MRFI_Packet packet;

int main()
{
  
  NetworkInit();
  /*Systick interrupt every 1ms*/
  SysTick_Config(SystemCoreClock/16800);
  if(isNetwork_init()){
    Buzzer_Init();
    packet.host_addr = BUZZ_PERIPH_ADDR;
    packet.dest_addr = CEN_PERIPH_ADDR;
  
    while(1) {
      MRFI_ServiceRun();
    }
  }
}

/* Private Function ----------------------------------------------------------*/
void Buzzer_Init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  /* BUZZER */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);  
  
}
/* Callback Function ---------------------------------------------------------*/
/*
*RF Receive event callback
*/
void MRFI_ReceiveCallback(mrfiPacket_t *packet) {
  
  uint32_t target_addr = (packet->frame[5] << 24) | (packet->frame[6] << 16) | (packet->frame[7] << 8) | (packet->frame[8]);
  
  if(target_addr == BUZZ_PERIPH_ADDR){
    (packet->frame[9] == '1')?(BUZZER_ENABLE_FLAG = 1):(BUZZER_ENABLE_FLAG = 0);
    }
}
void SysTick_Handler(){
  /*Reset Counter*/
  (PWM_Counter == PERIOD)?(PWM_Counter = 0):(PWM_Counter++);
  
  if(PWM_Counter <= PULSE_WIDTH && BUZZER_ENABLE_FLAG) 
    GPIO_SetBits(GPIOE, GPIO_Pin_9);
  else if(PWM_Counter > PULSE_WIDTH && BUZZER_ENABLE_FLAG)
    GPIO_ResetBits(GPIOE, GPIO_Pin_9);
  else
    return;
}