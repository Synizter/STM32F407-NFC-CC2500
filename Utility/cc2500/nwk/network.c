#include "network.h"

static mrfiPacket_t TX_buffer;
static uint8_t RF_STATE = 0x0;

void SendPacket(MRFI_Packet * pBuff)
{
    uint8_t i;
    uint8_t payload_offset;
    
    payload_offset = __mrfi_LENGTH_FIELD_SIZE__ + (2 * __mrfi_ADDR_SIZE__);
    
    TX_buffer.frame[0] = pBuff->payload_length + (2 * __mrfi_ADDR_SIZE__);
    
    TX_buffer.frame[1] = (pBuff->host_addr>>24) & 0xFF;
    TX_buffer.frame[2] = (pBuff->host_addr>>16) & 0xFF;
    TX_buffer.frame[3] = (pBuff->host_addr>>8) & 0xFF;
    TX_buffer.frame[4] = pBuff->host_addr & 0xFF;
    
    TX_buffer.frame[5] = (pBuff->dest_addr>>24) & 0xFF;
    TX_buffer.frame[6] = (pBuff->dest_addr>>16) & 0xFF;
    TX_buffer.frame[7] = (pBuff->dest_addr>>8) & 0xFF;
    TX_buffer.frame[8] = pBuff->dest_addr & 0xFF;
    
    for( i = 0; i < pBuff->payload_length; i++)
    {
      TX_buffer.frame[i + payload_offset] = pBuff->payload[i];
    }
    
    MRFI_Transmit( &TX_buffer, MRFI_TX_TYPE_CCA );
}

void NetworkInit(){
  BSP_Init();
  MRFI_Init();
  MRFI_RxOn(MRFI_ReceiveCallback);
  RF_STATE = 0x1;
}

uint8_t isNetwork_init(){
  return RF_STATE;
}


