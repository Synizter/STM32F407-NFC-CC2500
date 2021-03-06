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

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS�
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

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Include file for core BSP services.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_H
#define BSP_H


/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp_macros.h"
#include "bsp_mcu_defs.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* ------------------------------------------------------------------------------------------------
 *                                          BSP Defines
 * ------------------------------------------------------------------------------------------------
 */
#define BSP
#define BSP_VER       100  /* BSP version 1.00a */
#define BSP_SUBVER    a


/* ------------------------------------------------------------------------------------------------
 *                                            Clock
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_CLOCK_MHZ   __bsp_CLOCK_MHZ__


/* ------------------------------------------------------------------------------------------------
 *                                            Memory
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_LITTLE_ENDIAN   __bsp_LITTLE_ENDIAN__

#define CODE    __bsp_CODE_MEMSPACE__
#define XDATA   __bsp_XDATA_MEMSPACE__

/* ------------------------------------------------------------------------------------------------
 *                                            Interrupts
 * ------------------------------------------------------------------------------------------------
 */
//#define BSP_ENABLE_INTERRUPTS()         __bsp_ENABLE_INTERRUPTS__()
//#define BSP_DISABLE_INTERRUPTS()        __bsp_DISABLE_INTERRUPTS__()
//#define BSP_INTERRUPTS_ARE_ENABLED()    __bsp_INTERRUPTS_ARE_ENABLED__()


/* ------------------------------------------------------------------------------------------------
 *                                         Critical Sections
 * ------------------------------------------------------------------------------------------------
 */
typedef bsp_mcu_istate_t bspIState_t;

//#define BSP_ENTER_CRITICAL_SECTION(x)   st( x = __bsp_GET_ISTATE__(); __bsp_DISABLE_INTERRUPTS__(); )
//#define BSP_EXIT_CRITICAL_SECTION(x)    __bsp_RESTORE_ISTATE__(x)
//#define BSP_CRITICAL_STATEMENT(x)       st( bspIState_t s;                    \
//                                            BSP_ENTER_CRITICAL_SECTION(s);    \
//                                            x;                                \
//                                            BSP_EXIT_CRITICAL_SECTION(s); )


/* ------------------------------------------------------------------------------------------------
 *                                           Asserts
 * ------------------------------------------------------------------------------------------------
 */

/*
 *  BSP_ASSERT( expression ) - The given expression must evaluate as "true" or else the assert
 *  handler is called.  From here, the call stack feature of the debugger can pinpoint where
 * the problem occurred.
 *
 *  BSP_FORCE_ASSERT() - If asserts are in use, immediately calls the assert handler.
 *
 *  BSP_ASSERTS_ARE_ON - can use #ifdef to see if asserts are enabled
 *
 *  Asserts can be disabled for optimum performance and minimum code size (ideal for
 *  finalized, debugged production code).
 */

#if (!defined BSP_NO_DEBUG)
#ifndef BSP_ASSERT_HANDLER
#define BSP_ASSERT_HANDLER()      st( BSP_DisableInterrupt();  while(1); )
#endif
#define BSP_ASSERT(expr)          st( if (!(expr)) BSP_ASSERT_HANDLER(); )
#define BSP_FORCE_ASSERT()        BSP_ASSERT_HANDLER()
#define BSP_ASSERTS_ARE_ON
#else
#define BSP_ASSERT(expr)          /* empty */
#define BSP_FORCE_ASSERT()        /* empty */
#endif

/* static assert */
#define BSP_STATIC_ASSERT(expr)   void bspDummyPrototype( char dummy[1/((expr)!=0)] );

/* ------------------------------------------------------------------------------------------------
 *                                           Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void BSP_Init(void);
void BSP_DisableInterrupt(void);
void BSP_EnableInterrupt(void);
void BSP_EnterCriticalSection(bspIState_t *IntState);
void BSP_ExitCriticalSection(bspIState_t *IntState);
void BSP_DelayUs(uint16_t usec);
void BSP_DelayMs(uint16_t ms);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BSP_H */
