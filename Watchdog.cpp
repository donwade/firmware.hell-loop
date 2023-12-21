#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_iwdg.h"
#include "misc.h"

void KickWatchdog(void)
{
	IWDG_ReloadCounter();
}


// CAUTION we hard code the LED pin to be gpio port B /pin 1 (the ysf pin).
void SetupWatchdog(void)
{
  /* Enable the Clock*/

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

#if DWADE_DISABLED  
  /* Configure the YSF GPIO_LED pin */
  //GPIO_InitTypeDef  GPIO_InitStructure;

  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_Init(GPIOC, &GPIO_InitStructure);


  /* Enable LED if a WD happens the h/w reset kills it */
  //GPIO_SetBits(GPIOB, GPIO_Pin_1);
#endif


  /* Enable the LSI OSC */
  RCC_LSICmd(ENABLE);
  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}

  /* Enable Watchdog*/
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_32); // 4, 8, 16 ... 256
  IWDG_SetReload(0x0FFF);//This parameter must be a number between 0 and 0x0FFF.
  IWDG_ReloadCounter();
  IWDG_Enable();


}
