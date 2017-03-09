/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
 * @brief       Initialize the board and drivers.
 * @param       none
 * @return      none
 */
void BSP_Init(void)
{
  
  /*-------------------------------------------------------------
   *  Run time integrity checks.  Perform only if asserts
   *  are enabled.
   */
#ifdef BSP_ASSERTS_ARE_ON
  /* verify endianess is correctly specified */
  {
    uint16_t test = 0x00AA; /* first storage byte of 'test' is non-zero for little endian */
    BSP_ASSERT(!(*((uint8_t *)&test)) == !BSP_LITTLE_ENDIAN); /* endianess mismatch */
  }
#endif
}

/*
 * @brief       Sleep for the requested amount of time.
 * @param       # of microseconds to sleep.
 * @return      none
 */
void BSP_Delay(uint16_t usec)
{
  volatile uint32_t v_usec = usec * ((336 / 8) - 2);
  while(v_usec--)
  {
    __NOP();
  }
}

/*
 * @brief       Enter critical section.
 * @param       none
 * @return      none
 */
void BSP_DisableInterrupt(void)
{
  __disable_interrupt();
}

/*
 * @brief       Enter critical section.
 * @param       none
 * @return      none
 */
void BSP_EnableInterrupt(void)
{
  __disable_interrupt();
}

/*
 * @brief       Enter critical section.
 * @param       none
 * @return      none
 */
void BSP_EnterCriticalSection(bspIState_t *IntState)
{
  __disable_interrupt();
  *IntState = 1;
}

/*
 * @brief       Exit critical section.
 * @param       none
 * @return      none
 */
void BSP_ExitCriticalSection(bspIState_t *IntState)
{
  if(*IntState)
    __enable_interrupt();
}

/*
 * @brief       Sleep for the requested amount of time.
 * @param       # of microseconds to sleep.
 * @return      none
 */
void BSP_DelayUs(uint16_t usec)
{
  unsigned long ulDelay;
  ulDelay = usec * 34;
  while(ulDelay--) {
    __NOP();
  }
}

/*
 * @brief       Delay the specified number of milliseconds.
 * @param       #milliseconds - delay time
 * @return      none
 */
void BSP_DelayMs(uint16_t ms)
{
  unsigned long ulDelay;
  ulDelay = ms * 3360;
  while(ulDelay--) {
    __NOP();
  }
}

/* ************************************************************************************************
 *                                   Compile Time Integrity Checks
 * ************************************************************************************************
 */
BSP_STATIC_ASSERT( sizeof(  uint8_t ) == 1 );
BSP_STATIC_ASSERT( sizeof(   int8_t ) == 1 );
BSP_STATIC_ASSERT( sizeof( uint16_t ) == 2 );
BSP_STATIC_ASSERT( sizeof(  int16_t ) == 2 );
BSP_STATIC_ASSERT( sizeof( uint32_t ) == 4 );
BSP_STATIC_ASSERT( sizeof(  int32_t ) == 4 );


/**************************************************************************************************
 */
