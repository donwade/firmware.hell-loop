/*
 *   Copyright (C) 2015,2016,2020 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016,2017,2018,2019,2020 by Andy Uribe CA6JAU
 *   Copyright (C) 2017 by Danilo DB4PLE
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

#if !defined(CIO_H)
#define  CIO_H

#include "Config.h"
#include "Globals.h"
#include "BitRB.h"

#define LOW      0
#define HIGH     1

// HS frequency ranges
#define VHF1_MIN  144000000
#define VHF1_MAX  148000000
#define VHF2_MIN  219000000
#define VHF2_MAX  225000000
#define UHF1_MIN  420000000
#define UHF1_MAX  475000000
#define UHF2_MIN  842000000
#define UHF2_MAX  950000000

// Banned amateur frequency ranges (satellite only, ISS, etc)
#define BAN_OPPL  140990000
#define BAN_OPPH  143000000

#define BAN_OPSL  866000000
#define BAN_OPSH  868000000

#define BAN1_MIN  145800000
#define BAN1_MAX  146000000
#define BAN2_MIN  435000000
#define BAN2_MAX  438000000

#define SCAN_TIME  1920
#define SCAN_PAUSE 20000

#if defined(DUPLEX)
#if defined(STM32_USB_HOST)
#define CAL_DLY_LOOP 98950U
#else
#define CAL_DLY_LOOP 96100U
#endif
#else
#if defined(STM32_USB_HOST)
#define CAL_DLY_LOOP 110850U
#else
#define CAL_DLY_LOOP 104600U
#endif
#endif
//----------------------------------------
typedef union {
	struct {
		uint32_t  ADDRESS: 4;
		uint32_t  DEMODSCHEME : 3;
		uint32_t  DOTCROSS : 1;
		uint32_t  INVERTCTL : 2;
		uint32_t  DISCRIM_BW : 10;
		uint32_t  POST_DEMOD_BW :10;
		uint32_t  XIF_BW :2;
	};
	uint32_t ALL_32;
} REG4_DEMODULATOR_SETUP;

typedef union {
	struct {
		uint32_t  ADDRESS: 4;
        uint32_t  SET_ADC_MODE:2;
        uint32_t  READBACK_MODE:2;
        uint32_t  READBACK_ENABLED: 1;
	};
	uint32_t ALL_32;
} REG7_READBACK_SETUP;


typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  BBOS_CLK_REG : 2;
		uint32_t  DEMOD_CLK_REG : 4;
		uint32_t  CDR_CLK_REG : 8;
		uint32_t  SEQUENCER_CLK_REG : 8;
		uint32_t  AGC_CLK_REG : 6;
	};
	uint32_t ALL_32;
} REG3_TX_RX_CONTROL;

typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  SLICER_THRES_REG : 7;
		uint32_t  VITERBI_ON_REG : 1;
		uint32_t  PHASE_CRCTN_EN_REG : 1;
		uint32_t  VITERBI_MEM_REG : 2;
		uint32_t  FSK3_THRES_REG : 7;
		uint32_t  FSK3_TIME_REG : 4;
	};
	uint32_t ALL_32;
} REG13_FSK_CONTROL;

typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  AFC_ENABLED_REG : 1;
		uint32_t  AFC_SCALING_REG : 12;
		uint32_t  AFC_KI_REG : 4;
		uint32_t  AFC_KP_REG : 3;
		uint32_t  AFC_MAX_RANGE_REG : 8;
	};
	uint32_t ALL_32;
} REG10_AFC_CONTROL;

typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  MOD_SCHEME_REG : 3;
		uint32_t  PA_ENABLE_REG : 1;
		uint32_t  PA_RAMP_REG : 3;
		uint32_t  PA_BIAS : 2;
		uint32_t  POWER_AMP_REG : 6;
		uint32_t  TX_DEVIATON_REG : 9;
		uint32_t  TX_INVERT_REG : 2;
		uint32_t  TX_RCOSINE_REG : 1;
	};
	uint32_t ALL_32;
} REG2_TX_CONTROL;

typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  RF_R_DIVIDER : 3;
		uint32_t  CLKOUT_DIV_REG : 4;
		uint32_t  XTAL_DOUBLER : 1;
		uint32_t  XTAL_OSC_EN : 1;
		uint32_t  XTAL_BIAS : 2;
		uint32_t  CP_CURRENT : 2;
		uint32_t  VCO_EN : 1;
		uint32_t  RF_DIV2_EN : 1;
		uint32_t  VCO_BIAS : 4;
		uint32_t  VCO_ADJ : 2;
		uint32_t  VCO_EXTERN_EN : 1;
	};
	uint32_t ALL_32;
} REG1_VCO_CONTROL;

typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  FRACTIONAL_N : 15;
		uint32_t  INTEGER_N : 8;
		uint32_t  RECIEVE_ON : 1;
		uint32_t  UART_MODE_ON : 1;
		uint32_t  MUXOUT_SEL : 3;
	};
	uint32_t ALL_32;
} REG0_N_REGISTER;

typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  SYNC_LENGTH : 2;
		uint32_t  TOLERANCE : 2;
		uint32_t  SEQUENCE : 24;
	};
	uint32_t ALL_32;
} REG11_SYNCWORD_REGISTER;

typedef union {
	struct {
		uint32_t  ADDRESS : 4;
		uint32_t  LOCK_MODE : 2;
		uint32_t  SWD_MODE : 2;
		uint32_t  PAYLOAD_LEN : 8;
	};
	uint32_t ALL_32;
} REG12_SWD_REGISTER;


typedef enum { down, stay, up } knob;

extern uint32_t  m_frequency_rx;
extern uint32_t  m_frequency_tx;
extern uint32_t  m_pocsag_freq_tx;
extern uint8_t   m_power;

class CIO {

public:
  CIO();

  // Platform API
  void      Init(void);
  void      SDIO_SET_CLOCKBIT(bool on);
  void      SDIO_WRITE_DATABIT(bool on);
  bool      SDIO_READ_DATABIT(void);
  void      SDIO_LATCHCMD_U1(bool on);
#if defined(DUPLEX)
  void      SDIO_LATCHCMD_U2(bool on);
  bool      READ_MODEM_DATA2(void);
#endif
  void      RESET_ALL_MODEMS(bool on);
  bool      READ_MODEM_DATA1(void);
  bool      READ_MODEM_CLOCK1(void);

#if defined(BIDIR_DATA_PIN)
  void      WRITE_MODEM_DATA1(bool on);
#endif

  void      WRITE_MODEM_CLOCK1(bool on);
  void      LED_PTT_RED(bool on);
  void      LED_RED_service(bool on);
  void      DEBUG_pin(bool on);
  void      LED_DSTAR_GREEN(bool on);
  void      LED_DMR_YELLOW(bool on);
  void      LED_YSF_AMBER(bool on);
  void      LED_P25_RED(bool on);
  void      LED_NXDN_BLUE(bool on);
  void      LED_M17_BLUE(bool on);
  void      LED_POCSAG_BLUE(bool on);
  void      LED_COS_YELW(bool on);
  void      interrupt(void);
#if defined(DUPLEX)
  void      interrupt2(void);
#endif

#if defined(BIDIR_DATA_PIN)
  void      SET_PP_OR_FLOAT_MODE(bool dir);
#endif
  void     monitorADF7021(uint8_t reg, uint32_t value);
  void     summaryADF7021(void);
  // IO API
  void      write(uint8_t* data, uint16_t length, const uint8_t* control = NULL);
  uint16_t  getSpace(void) const;
  void      process(void);
  bool      hasTXOverflow(void);
  bool      hasRXOverflow(void);
  uint8_t   setFreq(uint32_t frequency_rx, uint32_t frequency_tx, uint8_t rf_power, uint32_t pocsag_freq_tx);
  void      setPower(uint8_t power);
  void      setMode(MMDVM_STATE modemState);
  void      setDecode(bool dcd);
  void      resetWatchdog(void);
  uint32_t  getWatchdog(void);
  void      getIntCounter(uint16_t &int1, uint16_t &int2);
  void      selfTest(void);
#if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS)
  void      checkBand(uint32_t frequency_rx, uint32_t frequency_tx);
  uint8_t   checkZUMspot(uint32_t frequency_rx, uint32_t frequency_tx);
  void      setBandVHF(bool vhf_on);
  bool      hasSingleADF7021(void);
  bool      isDualBand(void);
#endif

  // RF interface API
  void      setTX(void);
  void      setRX(bool doSle = true);
  void      ifConf(MMDVM_STATE modemState, bool reset);
#if defined(DUPLEX)
  void      ifConf2(MMDVM_STATE modemState);
#endif
  void      start(void);
  void      startInt(void);
  void      setDeviations(uint8_t dmrTXLevel, uint8_t p25TXLevel, uint8_t pocsagTXLevel);
  void      updateCal(void);

#if defined(SEND_RSSI_DATA)
  uint16_t  readRSSI(void);
#endif

  // Misc functions
  void      dlybit(void);
  void      delay_IFcal(void);
  void      delay_reset(void);
  void      delay_us(uint32_t us);

#if defined(ENABLE_DEBUG)
  uint32_t  RXfreq(void);
  uint32_t  TXfreq(void);
  uint16_t  devDSTAR(void);
  uint16_t  devDMR(void);
  uint16_t  devP25(void);
  uint16_t  devPOCSAG(void);
  void      printConf();
#endif

private:
  uint8_t            m_RX_N_divider;
  uint16_t           m_RX_F_divider;
  uint8_t            m_TX_N_divider;
  uint16_t           m_TX_F_divider;

  bool               m_started;
  CBitRB             m_rxBuffer;
  CBitRB             m_txBuffer;
  uint32_t           m_ledCount;
  bool               m_scanEnable;
  uint32_t           m_scanPauseCnt;
  uint8_t            m_scanPos;
  uint8_t            m_TotalModes;
  MMDVM_STATE        m_Modes[6];
  bool               m_ledValue;
  volatile uint32_t  m_watchdog;
  volatile uint16_t  m_int1counter;
  volatile uint16_t  m_int2counter;

  bool               m_bTransmitAllowed;
  uint8_t   setFreqHw(uint32_t frequency_rx, uint32_t frequency_tx, uint8_t rf_power, uint32_t pocsag_freq_tx);
};

#endif
