#ifndef BSP_MCU_DEFS_H
#define BSP_MCU_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
#include "stm32f4xx.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

typedef int bsp_mcu_istate_t;

/* ------------------------------------------------------------------------------------------------
 *                                          Common
 * ------------------------------------------------------------------------------------------------
 */
#define __bsp_LITTLE_ENDIAN__   1
#define __bsp_CODE_MEMSPACE__   /* blank */
#define __bsp_XDATA_MEMSPACE__  /* blank */

#ifndef NULL
#define NULL 0
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

/**************************************************************************************************
 */
#endif /* BSP_MCU_DEFS_H */
