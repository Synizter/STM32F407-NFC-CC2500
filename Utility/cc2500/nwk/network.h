#ifndef __NETWORK_H__
#define __NETWORK_H__

#include "bsp.h"
#include "mrfi.h"

#define PERIPH_EN           0xE
#define PERIPH_DIS          0xD

typedef struct
{
  uint32_t  host_addr;
  uint32_t  dest_addr;
  uint8_t   length;
  uint8_t   payload_length;
  uint8_t   payload[MRFI_MAX_PAYLOAD_SIZE];  
}MRFI_Packet;

void NetworkInit();
uint8_t isNetwork_init();
void SendPacket(MRFI_Packet * pBuff);
void PacketDecoder(MRFI_Packet* pBuff, mrfiPacket_t* packet);
void MRFI_ReceiveCallback(mrfiPacket_t *packet);

#endif