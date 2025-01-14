/*
 *   Copyright (C) 2015,2016,2020 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016,2017,2018,2019 by Andy Uribe CA6JAU
 *   Copyright (C) 2019 by Florian Wolters DF2ET
 *
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

#if !defined(GLOBALS_H)
#define  GLOBALS_H

#if defined(STM32F10X_MD)
#include <stm32f10x.h>
#include "string.h"
#elif defined(STM32F4XX)
#include "stm32f4xx.h"
#include "string.h"
#elif defined(STM32F7XX)
#include "stm32f7xx.h"
#include "string.h"
#else
#include <Arduino.h>
#endif

enum MMDVM_STATE {
  STATE_IDLE      = 0,
  STATE_DSTAR     = 1,
  STATE_DMR       = 2,
  STATE_YSF       = 3,
  STATE_P25       = 4,
  STATE_NXDN      = 5,
  STATE_POCSAG    = 6,
  STATE_M17       = 7,

  // Dummy states start at 90
  STATE_DMRDMO1K  = 92,
  STATE_RSSICAL   = 96,
  STATE_CWID      = 97,
  STATE_DMRCAL    = 98,
  STATE_DSTARCAL  = 99,
  STATE_INTCAL    = 100,
  STATE_POCSAGCAL = 101
};

const uint8_t  MARK_SLOT1 = 0x08U;
const uint8_t  MARK_SLOT2 = 0x04U;
const uint8_t  MARK_NONE  = 0x00U;

// Bidirectional Data pin (Enable Standard TX/RX Data Interface of ADF7021):
#define BIDIR_DATA_PIN

#include "IO.h"
#include "SerialPort.h"
#include "DMRDMORX.h"
#include "DMRDMOTX.h"

#if defined(DUPLEX)
#include "DMRIdleRX.h"
#include "DMRRX.h"
#include "DMRTX.h"
#endif

#include "DStarRX.h"
#include "DStarTX.h"
#include "YSFRX.h"
#include "YSFTX.h"
#include "P25RX.h"
#include "P25TX.h"
#include "M17RX.h"
#include "M17TX.h"
#include "NXDNRX.h"
#include "NXDNTX.h"
#include "POCSAGTX.h"
#include "CWIdTX.h"
#include "CalRSSI.h"
#include "CalDMR.h"
#include "Debug.h"
#include "Utils.h"
#include "I2CHost.h"

extern MMDVM_STATE m_modemState;
extern MMDVM_STATE m_calState;
extern MMDVM_STATE m_modemState_prev;

extern bool m_cwid_state;
extern bool m_pocsag_state;

extern uint8_t m_cwIdTXLevel;

extern uint32_t m_modeTimerCnt;

extern bool m_dstarEnable;
extern bool m_dmrEnable;
extern bool m_ysfEnable;
extern bool m_p25Enable;
extern bool m_nxdnEnable;
extern bool m_m17Enable;
extern bool m_pocsagEnable;

extern bool m_duplex;

extern bool m_tx;
extern bool m_dcd;

extern CIO io;
extern CSerialPort serial;

extern CDStarRX dstarRX;
extern CDStarTX dstarTX;

extern uint8_t m_control;

#if defined(DUPLEX)
extern CDMRIdleRX dmrIdleRX;
extern CDMRRX dmrRX;
extern CDMRTX dmrTX;
#endif

extern CDMRDMORX dmrDMORX;
extern CDMRDMOTX dmrDMOTX;

extern CYSFRX ysfRX;
extern CYSFTX ysfTX;

extern CP25RX p25RX;
extern CP25TX p25TX;

extern CM17RX m17RX;
extern CM17TX m17TX;

extern CNXDNRX nxdnRX;
extern CNXDNTX nxdnTX;

extern CPOCSAGTX pocsagTX;

extern CCalDMR  calDMR;

#if defined(SEND_RSSI_DATA)
extern CCalRSSI calRSSI;
#endif

extern CCWIdTX cwIdTX;

#if defined(STM32_I2C_HOST)
extern CI2CHost i2c;
#endif

#define _RESET   "\033[0m"
#define _BLACK   "\033[30m"      /* Black */
#define _RED     "\033[91m"      /* Red */
#define _GREEN   "\033[92m"      /* Green */
#define _YELLOW  "\033[93m"      /* Yellow */
#define _BLUE    "\033[94m"      /* Blue */
#define _MAGENTA "\033[95m"      /* Magenta */
#define _CYAN    "\033[96m"      /* Cyan */
#define _WHITE   "\033[97m"      /* White */
#define _BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define _BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define _BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define _BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define _BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define _BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define _BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define _BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
#define _BLINK_RED     "\033[5m\033[31m"      /* Red */
#define _BLINK_GREEN   "\033[5m\033[32m"      /* Green */
#define _BLINK_YELLOW  "\033[5m\033[33m"      /* Yellow */
#define _BLINK_BLUE    "\033[5m\033[34m"      /* Blue */
#define _BLINK_MAGENTA "\033[5m\033[35m"      /* Magenta */
#define _BLINK_CYAN    "\033[5m\033[36m"      /* Cyan */
#define _BLINK_WHITE   "\033[5m\033[37m"      /* White */

#endif

