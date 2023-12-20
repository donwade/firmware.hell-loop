/*
 *   Copyright (C) 2020 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Jim McLaughlin KI6ZUM
 *   Copyright (C) 2016,2017,2018,2019,2020 by Andy Uribe CA6JAU
 *   Copyright (C) 2017 by Danilo DB4PLE 
  
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include "Config.h"

// Select initial release of ZUMspot RPi:
// #define PI_HAT_7021_REV_02

#if defined(STM32F10X_MD)

#include "Globals.h"
#include "IO.h"

#if defined(PI_HAT_7021_REV_02)

#define PIN_SDIO_CLK             GPIO_Pin_4
#define PORT_SDIO_CLK            GPIOB

#define PIN_SDIO_READFROM            GPIO_Pin_5
#define PORT_SDIO_READ_FROM           GPIOB

#define PIN_SDIO_WRITETO            GPIO_Pin_6
#define PORT_SDIO_WRITETO           GPIOB

#define PIN_SDIO_LATCH1              GPIO_Pin_7
#define PORT_SDIO_LATCH1             GPIOB

#define PIN_CE               GPIO_Pin_14
#define PORT_CE              GPIOC

#define PIN_MODEM_DATA1              GPIO_Pin_3
#define PORT_MODEM_DATA1             GPIOB

// TXD used in SPI Data mode of ADF7021
// TXD is TxRxCLK of ADF7021, standard TX/RX data interface
#define PIN_MODEM_CLK1              GPIO_Pin_15
#define PORT_MODEM_CLK1             GPIOA
#define PIN_MODEM_INT1          GPIO_PinSource15
#define PORT_MODEM_INT1         GPIO_PortSourceGPIOA

// CLKOUT used in SPI Data mode of ADF7021
#define PIN_CLKOUT           GPIO_Pin_14
#define PORT_CLKOUT          GPIOA
#define PIN_CLKOUT_INT       GPIO_PinSource14
#define PORT_CLKOUT_INT      GPIO_PortSourceGPIOA

#define PIN_RED_SERVICE              GPIO_Pin_13
#define PORT_RED_SERVICE             GPIOC

#define PIN_DEBUG_B9              GPIO_Pin_11
#define PORT_DEBUG_B9             GPIOA

#define PIN_DSTAR_GREEN_LED        GPIO_Pin_14
#define PORT_DSTAR_GREEN_LED       GPIOB

#define PIN_DMR_YLW_LED          GPIO_Pin_15
#define PORT_DMR_YLW_LED         GPIOB

#define PIN_YSF_AMBR_LED          GPIO_Pin_13
#define PORT_YSF_AMBR_LED         GPIOA

#define PIN_P25_RED_LED          GPIO_Pin_12
#define PORT_P25_RED_LED         GPIOA

#define PIN_NXDN_BLUE_LED         GPIO_Pin_8
#define PORT_NXDN_BLUE_LED        GPIOA

// XXX  FIXME
#define PIN_M17_BLUE_LED          GPIO_Pin_8
#define PORT_M17_BLUE_LED         GPIOA

#define PIN_POCSAG_BLUE_LED       GPIO_Pin_5
#define PORT_POCSAG_BLUE_LED      GPIOA

#define PIN_PTT_RED_LED          GPIO_Pin_12
#define PORT_PTT_RED_LED         GPIOB

#define PIN_COS_YELW_LED          GPIO_Pin_13
#define PORT_COS_YELW_LED         GPIOB

#elif defined(ZUMSPOT_ADF7021) || defined(SKYBRIDGE_HS) || defined(LONESTAR_USB)

#define PIN_SDIO_CLK             GPIO_Pin_5
#define PORT_SDIO_CLK            GPIOB

#define PIN_SDIO_READFROM            GPIO_Pin_6
#define PORT_SDIO_READ_FROM           GPIOB

#define PIN_SDIO_WRITETO            GPIO_Pin_7
#define PORT_SDIO_WRITETO           GPIOB

#define PIN_SDIO_LATCH1              GPIO_Pin_8
#define PORT_SDIO_LATCH1             GPIOB

#define PIN_SDIO_LATCH2             GPIO_Pin_6
#define PORT_SDIO_LATCH2            GPIOA

#define PIN_CE               GPIO_Pin_14
#define PORT_CE              GPIOC

#define PIN_MODEM_DATA1              GPIO_Pin_4
#define PORT_MODEM_DATA1             GPIOB

#define PIN_SGL_DBL          GPIO_Pin_12
#define PORT_SGL_DBL         GPIOA

#define PIN_DL_DPX           GPIO_Pin_15
#define PORT_DL_DPX          GPIOC

#define PIN_SET_BAND         GPIO_Pin_15
#define PORT_SET_BAND        GPIOA

// TXD used in SPI Data mode of ADF7021
// TXD is TxRxCLK of ADF7021, standard TX/RX data interface
#define PIN_MODEM_CLK1              GPIO_Pin_3
#define PORT_MODEM_CLK1             GPIOB
#define PIN_MODEM_INT1          GPIO_PinSource3
#define PORT_MODEM_INT1         GPIO_PortSourceGPIOB

#if defined(DUPLEX)
#define PIN_MODEM_DATA2             GPIO_Pin_11
#define PORT_MODEM_DATA2            GPIOA

// TXD2 is TxRxCLK of the second ADF7021, standard TX/RX data interface
#define PIN_MODEM_CLK2             GPIO_Pin_8
#define PORT_MODEM_CLK2            GPIOA
#define PIN_MODEM_INT2         GPIO_PinSource8
#define PORT_MODEM_INT        GPIO_PortSourceGPIOA
#endif

// CLKOUT used in SPI Data mode of ADF7021
#define PIN_CLKOUT           GPIO_Pin_15
#define PORT_CLKOUT          GPIOA
#define PIN_CLKOUT_INT       GPIO_PinSource15
#define PORT_CLKOUT_INT      GPIO_PortSourceGPIOA

#define PIN_RED_SERVICE              GPIO_Pin_13
#define PORT_RED_SERVICE             GPIOC

#define PIN_DEBUG_B9              GPIO_Pin_9
#define PORT_DEBUG_B9             GPIOB

#define PIN_DSTAR_GREEN_LED        GPIO_Pin_12
#define PORT_DSTAR_GREEN_LED       GPIOB

#define PIN_DMR_YLW_LED          GPIO_Pin_13
#define PORT_DMR_YLW_LED         GPIOB

#define PIN_YSF_AMBR_LED          GPIO_Pin_1
#define PORT_YSF_AMBR_LED         GPIOB

#define PIN_P25_RED_LED          GPIO_Pin_0
#define PORT_P25_RED_LED         GPIOB

#if defined(STM32_USB_HOST)
#define PIN_NXDN_BLUE_LED         GPIO_Pin_1
#else
#define PIN_NXDN_BLUE_LED         GPIO_Pin_7
#endif
#define PORT_NXDN_BLUE_LED        GPIOA

// XXX FIXME
#if defined(STM32_USB_HOST)
#define PIN_M17_BLUE_LED          GPIO_Pin_1
#else
#define PIN_M17_BLUE_LED          GPIO_Pin_7
#endif
#define PORT_M17_BLUE_LED         GPIOA

#define PIN_POCSAG_BLUE_LED       GPIO_Pin_5
#define PORT_POCSAG_BLUE_LED      GPIOA

#define PIN_PTT_RED_LED          GPIO_Pin_14
#define PORT_PTT_RED_LED         GPIOB

#define PIN_COS_YELW_LED          GPIO_Pin_15
#define PORT_COS_YELW_LED         GPIOB

#elif defined(LIBRE_KIT_ADF7021) || defined(MMDVM_HS_HAT_REV12) || defined(MMDVM_HS_DUAL_HAT_REV10) || defined(NANO_HOTSPOT) || defined(NANO_DV_REV11) || defined(D2RG_MMDVM_HS)

#define PIN_SDIO_CLK             GPIO_Pin_5
#define PORT_SDIO_CLK            GPIOB

#define PIN_SDIO_READFROM            GPIO_Pin_7
#define PORT_SDIO_READ_FROM           GPIOB

#define PIN_SDIO_WRITETO            GPIO_Pin_6
#define PORT_SDIO_WRITETO           GPIOB

#define PIN_SDIO_LATCH1              GPIO_Pin_8
#define PORT_SDIO_LATCH1             GPIOB

#define PIN_SDIO_LATCH2             GPIO_Pin_6
#define PORT_SDIO_LATCH2            GPIOA

#define PIN_CE               GPIO_Pin_14
#define PORT_CE              GPIOC

#define PIN_MODEM_DATA1              GPIO_Pin_4
#define PORT_MODEM_DATA1             GPIOB

#define PIN_MODEM_DATA2             GPIO_Pin_4
#define PORT_MODEM_DATA2            GPIOA

// TXD used in SPI Data mode of ADF7021
// TXD is TxRxCLK of ADF7021, standard TX/RX data interface
#define PIN_MODEM_CLK1              GPIO_Pin_3
#define PORT_MODEM_CLK1             GPIOB
#define PIN_MODEM_INT1          GPIO_PinSource3
#define PORT_MODEM_INT1         GPIO_PortSourceGPIOB

// TXD2 is TxRxCLK of the second ADF7021, standard TX/RX data interface
#define PIN_MODEM_CLK2             GPIO_Pin_5
#define PORT_MODEM_CLK2            GPIOA
#define PIN_MODEM_INT2         GPIO_PinSource5
#define PORT_MODEM_INT        GPIO_PortSourceGPIOA

// CLKOUT used in SPI Data mode of ADF7021
#define PIN_CLKOUT           GPIO_Pin_15
#define PORT_CLKOUT          GPIOA
#define PIN_CLKOUT_INT       GPIO_PinSource15
#define PORT_CLKOUT_INT      GPIO_PortSourceGPIOA

#define PIN_RED_SERVICE              GPIO_Pin_13
#define PORT_RED_SERVICE             GPIOC

#define PIN_DEBUG_B9              GPIO_Pin_9
#define PORT_DEBUG_B9             GPIOB

#define PIN_DSTAR_GREEN_LED        GPIO_Pin_12
#define PORT_DSTAR_GREEN_LED       GPIOB

#define PIN_DMR_YLW_LED          GPIO_Pin_13
#define PORT_DMR_YLW_LED         GPIOB

#define PIN_YSF_AMBR_LED          GPIO_Pin_1
#define PORT_YSF_AMBR_LED         GPIOB

#define PIN_P25_RED_LED          GPIO_Pin_0
#define PORT_P25_RED_LED         GPIOB

#define PIN_NXDN_BLUE_LED         GPIO_Pin_8
#define PORT_NXDN_BLUE_LED        GPIOA

// XXX FIXME
#define PIN_M17_BLUE_LED          GPIO_Pin_8
#define PORT_M17_BLUE_LED         GPIOA

#define PIN_POCSAG_BLUE_LED       GPIO_Pin_7
#define PORT_POCSAG_BLUE_LED      GPIOA

#define PIN_PTT_RED_LED          GPIO_Pin_14
#define PORT_PTT_RED_LED         GPIOB

#define PIN_COS_YELW_LED          GPIO_Pin_15
#define PORT_COS_YELW_LED         GPIOB

#else
#error "Either PI_HAT_7021_REV_02, ZUMSPOT_ADF7021, LONESTAR_USB, LIBRE_KIT_ADF7021, MMDVM_HS_HAT_REV12, MMDVM_HS_DUAL_HAT_REV10, NANO_HOTSPOT, NANO_DV_REV11, D2RG_MMDVM_HS or SKYBRIDGE_HS need to be defined"
#endif

extern "C" {
#if defined(PI_HAT_7021_REV_02)

#if defined(BIDIR_DATA_PIN)
  void EXTI15_10_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line15)!=RESET) {
      io.interrupt();
    EXTI_ClearITPendingBit(EXTI_Line15);
    }
  }
#else
  void EXTI15_10_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line14)!=RESET) {
      io.interrupt();
    EXTI_ClearITPendingBit(EXTI_Line14);
    }
  }
#endif

#elif defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(LIBRE_KIT_ADF7021) || defined(MMDVM_HS_HAT_REV12) || defined(MMDVM_HS_DUAL_HAT_REV10) || defined(NANO_HOTSPOT) || defined(NANO_DV_REV11) || defined(D2RG_MMDVM_HS) || defined(SKYBRIDGE_HS)

#if defined(BIDIR_DATA_PIN)
  void EXTI3_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line3)!=RESET) {
      io.interrupt();
    EXTI_ClearITPendingBit(EXTI_Line3);
    }
  }
#else
  void EXTI15_10_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line15)!=RESET) {
      io.interrupt();
    EXTI_ClearITPendingBit(EXTI_Line15);
    }
  }
#endif

#if defined(DUPLEX)
  void EXTI9_5_IRQHandler(void) {
    #if defined(ZUMSPOT_ADF7021) || defined(SKYBRIDGE_HS)
    if(EXTI_GetITStatus(EXTI_Line8)!=RESET) {
      io.interrupt2();
    EXTI_ClearITPendingBit(EXTI_Line8);
    }
    #else
    if(EXTI_GetITStatus(EXTI_Line5)!=RESET) {
      io.interrupt2();
    EXTI_ClearITPendingBit(EXTI_Line5);
    }
    #endif
  }
#endif

#endif
}

void CIO::Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);

  EXTI_InitTypeDef EXTI_InitStructure;
#if defined(DUPLEX)
  EXTI_InitTypeDef EXTI_InitStructure2;
#endif

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

#if defined(PI_HAT_7021_REV_02)
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
#elif defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(LIBRE_KIT_ADF7021) || defined(MMDVM_HS_HAT_REV12) || defined(MMDVM_HS_DUAL_HAT_REV10) || defined(NANO_HOTSPOT) || defined(NANO_DV_REV11) || defined(D2RG_MMDVM_HS) || defined(SKYBRIDGE_HS)
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#endif

#if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS)
  // Pin defines if the board has a single ADF7021 or double
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_SGL_DBL;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_Init(PORT_SGL_DBL, &GPIO_InitStruct);

  // Pin defines if the board is dual band or duplex
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_DL_DPX;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_Init(PORT_DL_DPX, &GPIO_InitStruct);

  // Pin will set UHF or VHF
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_SET_BAND;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SET_BAND, &GPIO_InitStruct);
  // TODO: Remove this line
  // GPIO_WriteBit(PORT_SET_BAND, PIN_SET_BAND, Bit_RESET);

#endif

#if defined(STM32_USB_HOST)
  // Pin PA11,PA12 = LOW, USB Reset
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_RESET);
  GPIO_WriteBit(GPIOA, GPIO_Pin_12, Bit_RESET);

#endif

#if defined(LONG_USB_RESET)
  // 10 ms delay
  delay_us(10000U);
#else
  volatile unsigned int delay;
  for(delay = 0;delay<512;delay++);
#endif

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // Pin SCLK
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_SDIO_CLK;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SDIO_CLK, &GPIO_InitStruct);

  // Pin SDATA
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_SDIO_WRITETO;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SDIO_WRITETO, &GPIO_InitStruct);

  // Pin SREAD
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_SDIO_READFROM;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(PORT_SDIO_READ_FROM, &GPIO_InitStruct);

  // Pin SLE
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_SDIO_LATCH1;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SDIO_LATCH1, &GPIO_InitStruct);

#if defined(DUPLEX)
  // Pin SLE2
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_SDIO_LATCH2;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_SDIO_LATCH2, &GPIO_InitStruct);

  // Pin RXD2
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_MODEM_DATA2;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(PORT_MODEM_DATA2, &GPIO_InitStruct);
#endif

  // Pin CE
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_CE;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_CE, &GPIO_InitStruct);

  // Pin RXD
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_MODEM_DATA1;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(PORT_MODEM_DATA1, &GPIO_InitStruct);

  // Pin TXD
  // TXD is TxRxCLK of ADF7021, standard TX/RX data interface
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_MODEM_CLK1;
#if defined(BIDIR_DATA_PIN)
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
#else
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
#endif
  GPIO_Init(PORT_MODEM_CLK1, &GPIO_InitStruct);
#if defined(DUPLEX)
  GPIO_InitStruct.GPIO_Pin   = PIN_MODEM_CLK2;
  GPIO_Init(PORT_MODEM_CLK2, &GPIO_InitStruct);
#endif

  // Pin TXRX_CLK
#if !defined(BIDIR_DATA_PIN)
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_CLKOUT;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(PORT_CLKOUT, &GPIO_InitStruct);
#endif

  // Pin LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_RED_SERVICE;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_RED_SERVICE, &GPIO_InitStruct);

  // Pin Debug
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_DEBUG_B9;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_DEBUG_B9, &GPIO_InitStruct);

  // D-Star LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_DSTAR_GREEN_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_DSTAR_GREEN_LED, &GPIO_InitStruct);

  // DMR LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_DMR_YLW_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_DMR_YLW_LED, &GPIO_InitStruct);

  // YSF LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_YSF_AMBR_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_YSF_AMBR_LED, &GPIO_InitStruct);

  // P25 LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_P25_RED_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_P25_RED_LED, &GPIO_InitStruct);

  // NXDN LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_NXDN_BLUE_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_NXDN_BLUE_LED, &GPIO_InitStruct);

  // POCSAG LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_POCSAG_BLUE_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_POCSAG_BLUE_LED, &GPIO_InitStruct);

  // PTT LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_PTT_RED_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_PTT_RED_LED, &GPIO_InitStruct);

  // COS LED
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_COS_YELW_LED;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(PORT_COS_YELW_LED, &GPIO_InitStruct);

#if defined(PI_HAT_7021_REV_02)

#if defined(BIDIR_DATA_PIN)
  // Connect EXTI15 Line
  GPIO_EXTILineConfig(PORT_MODEM_INT1, PIN_MODEM_INT1);
  // Configure EXTI15 line
  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
#else
  // Connect EXTI14 Line
  GPIO_EXTILineConfig(PORT_CLKOUT_INT, PIN_CLKOUT_INT);
  // Configure EXTI14 line
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
#endif

#elif defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(LIBRE_KIT_ADF7021) || defined(MMDVM_HS_HAT_REV12) || defined(MMDVM_HS_DUAL_HAT_REV10) || defined(NANO_HOTSPOT) || defined(NANO_DV_REV11) || defined(D2RG_MMDVM_HS) || defined(SKYBRIDGE_HS)

#if defined(BIDIR_DATA_PIN)
  // Connect EXTI3 Line
  GPIO_EXTILineConfig(PORT_MODEM_INT1, PIN_MODEM_INT1);
  // Configure EXTI3 line
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
#else
  // Connect EXTI15 Line
  GPIO_EXTILineConfig(PORT_CLKOUT_INT, PIN_CLKOUT_INT);
  // Configure EXTI15 line
  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
#endif

#if defined(DUPLEX)
  // Connect EXTI5 Line
  GPIO_EXTILineConfig(PORT_MODEM_INT, PIN_MODEM_INT2);
  // Configure EXT5 line
  #if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS)
  EXTI_InitStructure2.EXTI_Line = EXTI_Line8;
  #else
  EXTI_InitStructure2.EXTI_Line = EXTI_Line5;
  #endif
#endif

#endif

  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

#if defined(DUPLEX)
  EXTI_InitStructure2.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure2.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure2.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure2);
#endif
}

void CIO::startInt()
{
  NVIC_InitTypeDef NVIC_InitStructure;

#if defined(DUPLEX)
  NVIC_InitTypeDef NVIC_InitStructure2;
#endif

#if defined(PI_HAT_7021_REV_02)

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;

#elif defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(LIBRE_KIT_ADF7021) || defined(MMDVM_HS_HAT_REV12) || defined(MMDVM_HS_DUAL_HAT_REV10) || defined(NANO_HOTSPOT) || defined(NANO_DV_REV11) || defined(D2RG_MMDVM_HS) || defined(SKYBRIDGE_HS)

#if defined(BIDIR_DATA_PIN)
  // Enable and set EXTI3 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
#else
  // Enable and set EXTI15 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
#endif

#if defined(DUPLEX)
  NVIC_InitStructure2.NVIC_IRQChannel = EXTI9_5_IRQn;
#endif

#endif

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#if defined(DUPLEX)
  NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 15;
  NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure2);
#endif
}

#if defined(BIDIR_DATA_PIN)
// RXD pin is bidirectional in standard interfaces
void CIO::SET_PP_OR_FLOAT_MODE(bool dir)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin   = PIN_MODEM_DATA1;

  if(dir)
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  else
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;

  GPIO_Init(PORT_MODEM_DATA1, &GPIO_InitStruct);
}
#endif

void CIO::SDIO_SET_CLOCKBIT(bool on)
{
  GPIO_WriteBit(PORT_SDIO_CLK, PIN_SDIO_CLK, on ? Bit_SET : Bit_RESET);
}

void CIO::SDIO_WRITE_DATABIT(bool on)
{
  GPIO_WriteBit(PORT_SDIO_WRITETO, PIN_SDIO_WRITETO, on ? Bit_SET : Bit_RESET);
}

bool CIO::SDIO_READ_DATABIT()
{
  return GPIO_ReadInputDataBit(PORT_SDIO_READ_FROM, PIN_SDIO_READFROM) == Bit_SET;
}

void CIO::SDIO_LATCHCMD_U1(bool on)
{
  GPIO_WriteBit(PORT_SDIO_LATCH1, PIN_SDIO_LATCH1, on ? Bit_SET : Bit_RESET);
}

#if defined(DUPLEX)
void CIO::SDIO_LATCHCMD_U2(bool on)
{
  GPIO_WriteBit(PORT_SDIO_LATCH2, PIN_SDIO_LATCH2, on ? Bit_SET : Bit_RESET);
}

bool CIO::READ_MODEM_DATA2()
{
  return GPIO_ReadInputDataBit(PORT_MODEM_DATA2, PIN_MODEM_DATA2) == Bit_SET;
}
#endif

void CIO::RESET_ALL_MODEMS(bool on)
{
  GPIO_WriteBit(PORT_CE, PIN_CE, on ? Bit_SET : Bit_RESET);
}

bool CIO::READ_MODEM_DATA1()
{
  return GPIO_ReadInputDataBit(PORT_MODEM_DATA1, PIN_MODEM_DATA1) == Bit_SET;
}

bool CIO::READ_MODEM_CLOCK1()
{
#if defined(BIDIR_DATA_PIN)
  return GPIO_ReadInputDataBit(PORT_MODEM_CLK1, PIN_MODEM_CLK1) == Bit_SET;
#else
  return GPIO_ReadInputDataBit(PORT_CLKOUT, PIN_CLKOUT) == Bit_SET;
#endif
}

#if defined(BIDIR_DATA_PIN)
void CIO::WRITE_MODEM_DATA1(bool on)
{
  GPIO_WriteBit(PORT_MODEM_DATA1, PIN_MODEM_DATA1, on ? Bit_SET : Bit_RESET);
}
#endif

void CIO::WRITE_MODEM_CLOCK1(bool on)
{
  GPIO_WriteBit(PORT_MODEM_CLK1, PIN_MODEM_CLK1, on ? Bit_SET : Bit_RESET);
}

void CIO::LED_RED_service(bool on)
{
  GPIO_WriteBit(PORT_RED_SERVICE, PIN_RED_SERVICE, on ? Bit_SET : Bit_RESET);
}

void CIO::DEBUG_pin(bool on)
{
  GPIO_WriteBit(PORT_DEBUG_B9, PIN_DEBUG_B9, on ? Bit_SET : Bit_RESET);
}

void CIO::LED_DSTAR_GREEN(bool on)
{
  GPIO_WriteBit(PORT_DSTAR_GREEN_LED, PIN_DSTAR_GREEN_LED, on ? Bit_SET : Bit_RESET);
}

void CIO::LED_DMR_YELLOW(bool on)
{
  GPIO_WriteBit(PORT_DMR_YLW_LED, PIN_DMR_YLW_LED, on ? Bit_SET : Bit_RESET);
}

void CIO::LED_YSF_AMBER(bool on)
{
  GPIO_WriteBit(PORT_YSF_AMBR_LED, PIN_YSF_AMBR_LED, on ? Bit_SET : Bit_RESET);
}

void CIO::LED_P25_RED(bool on)
{
  GPIO_WriteBit(PORT_P25_RED_LED, PIN_P25_RED_LED, on ? Bit_SET : Bit_RESET);
}

void CIO::LED_NXDN_BLUE(bool on)
{
#if defined(USE_ALTERNATE_NXDN_LEDS)
  GPIO_WriteBit(PORT_YSF_AMBR_LED, PIN_YSF_AMBR_LED, on ? Bit_SET : Bit_RESET);
  GPIO_WriteBit(PORT_P25_RED_LED, PIN_P25_RED_LED, on ? Bit_SET : Bit_RESET);
#else
  GPIO_WriteBit(PORT_NXDN_BLUE_LED, PIN_NXDN_BLUE_LED, on ? Bit_SET : Bit_RESET);
#endif
}

void CIO::LED_M17_BLUE(bool on)
{
#if defined(USE_ALTERNATE_M17_LEDS)
  GPIO_WriteBit(PORT_DSTAR_GREEN_LED, PIN_DSTAR_GREEN_LED, on ? Bit_SET : Bit_RESET);
  GPIO_WriteBit(PORT_P25_RED_LED, PIN_P25_RED_LED, on ? Bit_SET : Bit_RESET);
#else
  GPIO_WriteBit(PORT_M17_BLUE_LED, PIN_M17_BLUE_LED, on ? Bit_SET : Bit_RESET);
#endif
}

void CIO::LED_POCSAG_BLUE(bool on)
{
#if defined(USE_ALTERNATE_POCSAG_LEDS)
  GPIO_WriteBit(PORT_DSTAR_GREEN_LED, PIN_DSTAR_GREEN_LED, on ? Bit_SET : Bit_RESET);
  GPIO_WriteBit(PORT_DMR_YLW_LED, PIN_DMR_YLW_LED, on ? Bit_SET : Bit_RESET);
#else
  GPIO_WriteBit(PORT_POCSAG_BLUE_LED, PIN_POCSAG_BLUE_LED, on ? Bit_SET : Bit_RESET);
#endif
}

void CIO::LED_PTT_RED(bool on)
{
  GPIO_WriteBit(PORT_PTT_RED_LED, PIN_PTT_RED_LED, on ? Bit_SET : Bit_RESET);
}

void CIO::LED_COS_YELW(bool on)
{
  GPIO_WriteBit(PORT_COS_YELW_LED, PIN_COS_YELW_LED, on ? Bit_SET : Bit_RESET);
}

#if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS)
void CIO::setBandVHF(bool vhf_on) {
  GPIO_WriteBit(PORT_SET_BAND, PIN_SET_BAND, vhf_on ? Bit_SET : Bit_RESET);
}

bool CIO::hasSingleADF7021() {
  return GPIO_ReadInputDataBit(PORT_SGL_DBL, PIN_SGL_DBL) == Bit_SET;
}

bool CIO::isDualBand() {
  return GPIO_ReadInputDataBit(PORT_DL_DPX, PIN_DL_DPX) == Bit_SET;
}
#endif

/**
 * Function delay_us() from stm32duino project
 *
 * @brief Delay the given number of microseconds.
 *
 * @param us Number of microseconds to delay.
 */
static inline void delay_us(uint32_t us) {
    us *= 12;

    /* fudge for function call overhead  */
    us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}

void CIO::delay_IFcal() {
  delay_us(10000);
}

void CIO::delay_reset() {
  delay_us(300);
}

void CIO::delay_us(uint32_t us) {
  ::delay_us(us);
}

static inline void delay_ns() {

    asm volatile("nop          \n\t"
                 "nop          \n\t"
                 "nop          \n\t"
                 );
}


void CIO::dlybit(void)
{
  delay_ns();
}
#endif
