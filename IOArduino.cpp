/*
 *   Copyright (C) 2015,2016 by Jonathan Naylor G4KLX
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
#include "Globals.h"
#include "IO.h"

#if defined(ARDUINO)

#if defined (__STM32F1__)

// STM32F1 pin definitions, using STM32duino

#if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS)

#define PIN_SDIO_CLK       PB5
#define PIN_SDIO_WRITETO      PB6
#define PIN_SDIO_READFROM      PB7
#define PIN_SDIO_LATCH1        PB8
#define PIN_SDIO_LATCH2       PA6
#define PIN_CE         PC14
#define PIN_MODEM_DATA1        PB4
#define PIN_MODEM_DATA12       PA11
#define PIN_MODEM_CLK1        PB3
#define PIN_MODEM_CLK2       PA8
#define PIN_CLKOUT     PA15
#define PIN_RED_SERVICE        PC13
#define PIN_DEBUG_B9        PB9
#define PIN_DSTAR_GREEN_LED  PB12
#define PIN_DMR_YLW_LED    PB13
#define PIN_YSF_AMBR_LED    PB1
#define PIN_P25_RED_LED    PB0
#if defined(STM32_USB_HOST)
#define PIN_NXDN_BLUE_LED   PA1
#else
#define PIN_NXDN_BLUE_LED   PA7
#endif
#define PIN_POCSAG_BLUE_LED PA5
#define PIN_PTT_RED_LED    PB14
#define PIN_COS_YELW_LED    PB15

#elif defined(LIBRE_KIT_ADF7021) || defined(MMDVM_HS_HAT_REV12) || defined(MMDVM_HS_DUAL_HAT_REV10) || defined(NANO_HOTSPOT) || defined(NANO_DV_REV10)

#define PIN_SDIO_CLK       PB5
#define PIN_SDIO_WRITETO      PB7
#define PIN_SDIO_READFROM      PB6
#define PIN_SDIO_LATCH1        PB8
#define PIN_SDIO_LATCH2       PA6
#define PIN_CE         PC14
#define PIN_MODEM_DATA1        PB4
#define PIN_MODEM_DATA12       PA4
#define PIN_MODEM_CLK1        PB3
#define PIN_MODEM_CLK2       PA5
#define PIN_CLKOUT     PA15
#define PIN_RED_SERVICE        PC13
#define PIN_DEBUG_B9        PB9
#define PIN_DSTAR_GREEN_LED  PB12
#define PIN_DMR_YLW_LED    PB13
#define PIN_YSF_AMBR_LED    PB1
#define PIN_P25_RED_LED    PB0
#define PIN_NXDN_BLUE_LED   PA8
#define PIN_POCSAG_BLUE_LED PA7
#define PIN_PTT_RED_LED    PB14
#define PIN_COS_YELW_LED    PB15

#else
#error "Either ZUMSPOT_ADF7021, LONESTAR_USB, LIBRE_KIT_ADF7021, MMDVM_HS_HAT_REV12, MMDVM_HS_DUAL_HAT_REV10, NANO_HOTSPOT, NANO_DV_REV10 or SKYBRIDGE_HS need to be defined"
#endif

#elif defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

// Teensy pin definitions
#define PIN_SDIO_CLK        2
#define PIN_SDIO_READFROM       3
#define PIN_SDIO_WRITETO       4
#define PIN_SDIO_LATCH1         5
#define PIN_CE          6
#define PIN_MODEM_DATA1         7
#define PIN_MODEM_CLK1         8
#define PIN_CLKOUT     22
#define PIN_RED_SERVICE        13
#define PIN_DEBUG_B9        23
#define PIN_DSTAR_GREEN_LED  16
#define PIN_DMR_YLW_LED    17
#define PIN_YSF_AMBR_LED    18
#define PIN_P25_RED_LED    19
#define PIN_NXDN_BLUE_LED   20
#define PIN_POCSAG_BLUE_LED 21
#define PIN_PTT_RED_LED    14
#define PIN_COS_YELW_LED    15

#else

// Arduino pin definitions (Due and Zero)
#define PIN_SDIO_CLK        3
#define PIN_SDIO_READFROM       4   // 2 in Arduino Zero Pro
#define PIN_SDIO_WRITETO       5
#define PIN_SDIO_LATCH1         6
#define PIN_CE         12
#define PIN_MODEM_DATA1         7
#define PIN_MODEM_CLK1         8
#define PIN_CLKOUT      2   // 4 in Arduino Zero Pro
#define PIN_RED_SERVICE        13
#define PIN_DEBUG_B9        11
#define PIN_DSTAR_GREEN_LED  14
#define PIN_DMR_YLW_LED    15
#define PIN_YSF_AMBR_LED    16
#define PIN_P25_RED_LED    17
#define PIN_NXDN_BLUE_LED   18
#define PIN_POCSAG_BLUE_LED 19
#define PIN_PTT_RED_LED     9
#define PIN_COS_YELW_LED    10

#endif

extern "C" {
  void EXT_IRQHandler(void) {
    io.interrupt();
  }
}

#if defined(DUPLEX)
extern "C" {
  void EXT_IRQHandler2(void) {
    io.interrupt2();
  }
}
#endif

void CIO::delay_IFcal() {
  delayMicroseconds(10000);
}

void CIO::delay_reset() {
  delayMicroseconds(300);
}

void CIO::Init()
{
#if defined (__STM32F1__)

#if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(LIBRE_KIT_ADF7021) || defined(MMDVM_HS_HAT_REV12) || defined(MMDVM_HS_DUAL_HAT_REV10) || defined(NANO_HOTSPOT) || defined(NANO_DV_REV10) || defined(SKYBRIDGE_HS)
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
#endif

#endif

  pinMode(PIN_SDIO_CLK, OUTPUT);
  pinMode(PIN_SDIO_READFROM, OUTPUT);
  pinMode(PIN_SDIO_WRITETO, INPUT);
  pinMode(PIN_SDIO_LATCH1, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_MODEM_DATA1, INPUT);
  pinMode(PIN_CLKOUT, INPUT);
  pinMode(PIN_RED_SERVICE, OUTPUT);
  pinMode(PIN_DEBUG_B9, OUTPUT);
  pinMode(PIN_DSTAR_GREEN_LED, OUTPUT);
  pinMode(PIN_DMR_YLW_LED, OUTPUT);
  pinMode(PIN_YSF_AMBR_LED, OUTPUT);
  pinMode(PIN_P25_RED_LED, OUTPUT);
  pinMode(PIN_NXDN_BLUE_LED, OUTPUT);
  pinMode(PIN_POCSAG_BLUE_LED, OUTPUT);
  pinMode(PIN_PTT_RED_LED, OUTPUT);
  pinMode(PIN_COS_YELW_LED, OUTPUT);

#if defined(DUPLEX)
  pinMode(PIN_SDIO_LATCH2, OUTPUT);
  pinMode(PIN_MODEM_DATA12, INPUT);
  pinMode(PIN_MODEM_CLK2, INPUT);
#endif

#if defined(BIDIR_DATA_PIN)
  pinMode(PIN_MODEM_CLK1, INPUT);
#else
  pinMode(PIN_MODEM_CLK1, OUTPUT);
#endif
}

void CIO::startInt()
{
#if defined(BIDIR_DATA_PIN)

// TXD pin is TxRxCLK of ADF7021, standard TX/RX data interface
#if defined (__STM32F1__)
  attachInterrupt(PIN_MODEM_CLK1, EXT_IRQHandler, CHANGE);
#else
  attachInterrupt(digitalPinToInterrupt(PIN_MODEM_CLK1), EXT_IRQHandler, CHANGE);
#endif

#else

#if defined (__STM32F1__)
  attachInterrupt(PIN_CLKOUT, EXT_IRQHandler, CHANGE);
#else
  attachInterrupt(digitalPinToInterrupt(PIN_CLKOUT), EXT_IRQHandler, CHANGE);
#endif

#endif

#if defined(DUPLEX)
  attachInterrupt(PIN_MODEM_CLK2, EXT_IRQHandler2, RISING);
#endif
}

#if defined(BIDIR_DATA_PIN)
// RXD pin is bidirectional in standard interfaces
void CIO::SET_PP_OR_FLOAT_MODE(bool dir) 
{
  if(dir)
    pinMode(PIN_MODEM_DATA1, OUTPUT);
  else
    pinMode(PIN_MODEM_DATA1, INPUT);
}
#endif

void CIO::SDIO_SET_CLOCKBIT(bool on) 
{
  digitalWrite(PIN_SDIO_CLK, on ? HIGH : LOW);
}

void CIO::SDIO_WRITE_DATABIT(bool on) 
{
  digitalWrite(PIN_SDIO_READFROM, on ? HIGH : LOW);
}

bool CIO::SDIO_READ_DATABIT()
{
  return digitalRead(PIN_SDIO_WRITETO) == HIGH;
}

void CIO::SDIO_LATCHCMD_U1(bool on) 
{
  digitalWrite(PIN_SDIO_LATCH1, on ? HIGH : LOW);
}

#if defined(DUPLEX)
void CIO::SDIO_LATCHCMD_U2(bool on) 
{
  digitalWrite(PIN_SDIO_LATCH2, on ? HIGH : LOW);
}

bool CIO::READ_MODEM_DATA2()
{
  return digitalRead(PIN_MODEM_DATA12) == HIGH;
}
#endif

void CIO::RESET_ALL_MODEMS(bool on) 
{
  digitalWrite(PIN_CE, on ? HIGH : LOW);
}

bool CIO::READ_MODEM_DATA1()
{
  return digitalRead(PIN_MODEM_DATA1) == HIGH;
}

bool CIO::READ_MODEM_CLOCK1()
{
  return digitalRead(PIN_MODEM_CLK1) == HIGH;
}

#if defined(BIDIR_DATA_PIN)
void CIO::WRITE_MODEM_DATA1(bool on)
{
  digitalWrite(PIN_MODEM_DATA1, on ? HIGH : LOW);
}
#endif

void CIO::WRITE_MODEM_CLOCK1(bool on) 
{
  digitalWrite(PIN_MODEM_CLK1, on ? HIGH : LOW);
}

void CIO::LED_RED_service(bool on) 
{
  digitalWrite(PIN_RED_SERVICE, on ? HIGH : LOW);
}

void CIO::DEBUG_pin(bool on) 
{
  digitalWrite(PIN_DEBUG_B9, on ? HIGH : LOW);
}

void CIO::LED_DSTAR_GREEN(bool on) 
{
  digitalWrite(PIN_DSTAR_GREEN_LED, on ? HIGH : LOW);
}

void CIO::LED_DMR_YELLOW(bool on) 
{
  digitalWrite(PIN_DMR_YLW_LED, on ? HIGH : LOW);
}

void CIO::LED_YSF_AMBER(bool on) 
{
  digitalWrite(PIN_YSF_AMBR_LED, on ? HIGH : LOW);
}

void CIO::LED_P25_RED(bool on) 
{
  digitalWrite(PIN_P25_RED_LED, on ? HIGH : LOW);
}

void CIO::LED_NXDN_BLUE(bool on) 
{
#if defined(USE_ALTERNATE_NXDN_LEDS)
  digitalWrite(PIN_YSF_AMBR_LED, on ? HIGH : LOW);
  digitalWrite(PIN_P25_RED_LED, on ? HIGH : LOW);
#else
  digitalWrite(PIN_NXDN_BLUE_LED, on ? HIGH : LOW);
#endif
}

void CIO::LED_POCSAG_BLUE(bool on)
{
#if defined(USE_ALTERNATE_POCSAG_LEDS)
  digitalWrite(PIN_DSTAR_GREEN_LED, on ? HIGH : LOW);
  digitalWrite(PIN_DMR_YLW_LED, on ? HIGH : LOW);
#else
  digitalWrite(PIN_POCSAG_BLUE_LED, on ? HIGH : LOW);
#endif
}

void CIO::LED_PTT_RED(bool on) 
{
  digitalWrite(PIN_PTT_RED_LED, on ? HIGH : LOW);
}

void CIO::LED_COS_YELW(bool on) 
{
  digitalWrite(PIN_COS_YELW_LED, on ? HIGH : LOW);
}

void CIO::delay_us(uint32_t us) {
  ::delayMicroseconds(us);
}
void CIO::dlybit(void)
{
    asm volatile("nop          \n\t"
                 "nop          \n\t"
                 "nop          \n\t"
                 );
}

#endif
