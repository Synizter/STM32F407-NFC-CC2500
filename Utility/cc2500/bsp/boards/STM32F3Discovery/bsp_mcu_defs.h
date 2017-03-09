#ifndef BSP_MCU_DEFS_H
#define BSP_MCU_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
#include "stm32f30x.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/*
 * Note: This implementation does not automatically hook an ISR defined using
 * __bsp_ISR_FUNCTION__ since the Stellaris vector table is typically stored
 * in flash.  As a result, it is vital that you install the function
 * BSP_GpioPort1Isr() in the vector table for the GPIO port which contains the
 * pin used as the asynchronous signal from the radio.
 *
 */
typedef int bsp_mcu_istate_t;
#define BSP_EARLY_INIT(void)

#define __bsp_ENABLE_INTERRUPTS__()		bsp_port_Enable_All_Interrupt()
#define __bsp_DISABLE_INTERRUPTS__()		bsp_port_Disable_All_Interrupt()
#define __bsp_INTERRUPTS_ARE_ENABLED__()	bsp_port_Interrupt_Are_Enable()

#define __bsp_GET_ISTATE__()			bsp_port_GET_Interrupt_State()
#define __bsp_RESTORE_ISTATE__(x)		do							\
						{							\
							if(x)						\
							{						\
                                                  		bsp_port_Disable_All_Interrupt();	\
							}						\
							else						\
							{						\
								bsp_port_Enable_All_Interrupt();	\
							}						\
						} while (0);

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
