/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Include file for MRFI common code between
 *   radio family 1 and radio family 2.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_COMMON_H
#define MRFI_COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* ------------------------------------------------------------------------------------------------
 *                                       SmartRF Configuration
 * ------------------------------------------------------------------------------------------------
 */
#if (defined MRFI_CC1100)
#include "smartrf/smartrf_CC1100.h"
#elif (defined MRFI_CC1101)
#include "smartrf/smartrf_CC1101.h"
#elif (defined MRFI_CC2500)
#include "smartrf/smartrf_CC2500.h"
#elif (defined MRFI_CC1110)
#include "smartrf/smartrf_CC1110.h"
#elif (defined MRFI_CC1111)
#include "smartrf/smartrf_CC1111.h"
#elif (defined MRFI_CC2510)
#include "smartrf/smartrf_CC2510.h"
#elif (defined MRFI_CC2511)
#include "smartrf/smartrf_CC2511.h"
#else
#error "ERROR: A valid radio is not specified."
#endif

/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void MRFI_SetRxModeControl(void (*RxCtrlOff)(void), void (*RxCtrlOn)(void));
uint8_t MRFI_RxAddrIsFiltered(uint8_t * pAddr);

/**************************************************************************************************
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MRFI_COMMON_H */
