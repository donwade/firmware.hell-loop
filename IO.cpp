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

#include "Config.h"
#include "Globals.h"
#include "IO.h"

uint32_t    m_frequency_rx;
uint32_t    m_frequency_tx;
uint32_t    m_pocsag_freq_tx;
uint8_t     m_power;

CIO::CIO():
m_started(false),
m_rxBuffer(1024U),
m_txBuffer(1024U),
m_ledCount(0U),
m_scanEnable(false),
m_scanPauseCnt(0U),
m_scanPos(0U),
m_ledValue(true),
m_watchdog(0U),
m_int1counter(0U),
m_int2counter(0U)
{
  Init();

  RESET_ALL_MODEMS(HIGH);
  LED_RED_service(HIGH);
  LED_PTT_RED(LOW);
  LED_DSTAR_GREEN(LOW);
  LED_DMR_YELLOW(LOW);
  LED_YSF_AMBER(LOW);
  LED_P25_RED(LOW);
  LED_NXDN_BLUE(LOW);
  LED_M17_BLUE(LOW);
  LED_POCSAG_BLUE(LOW);
  LED_COS_YELW(LOW);
  DEBUG_pin(LOW);

#if !defined(BIDIR_DATA_PIN)
  WRITE_MODEM_CLOCK1(LOW);
#endif

  SDIO_SET_CLOCKBIT(LOW);
  SDIO_WRITE_DATABIT(LOW);
  SDIO_LATCHCMD_U1(LOW);

  selfTest();

  m_modeTimerCnt = 0U;
}

void CIO::selfTest()
{
  bool ledValue = false;
  uint32_t ledCount = 0U;
  uint32_t blinks = 0U;

  while(true) {
    ledCount++;
    delay_us(1000U);

    if(ledCount >= 125U) {
      ledCount = 0U;
      ledValue = !ledValue;

      LED_RED_service(!ledValue);
      LED_PTT_RED(ledValue);
      LED_DSTAR_GREEN(ledValue);
      LED_DMR_YELLOW(ledValue);
      LED_YSF_AMBER(ledValue);
      LED_P25_RED(ledValue);
      LED_NXDN_BLUE(ledValue);
      LED_M17_BLUE(ledValue);
      LED_POCSAG_BLUE(ledValue);
      LED_COS_YELW(ledValue);

      blinks++;

      if(blinks > 5U)
        break;
    }
  }
}

void CIO::process()
{
  uint8_t bit;
  uint32_t scantime;
  uint8_t  control;

  m_ledCount++;

  if (m_started) {
    // Two seconds timeout
    if (m_watchdog >= 19200U) {
      if (m_modemState == STATE_DMR || m_modemState == STATE_P25 ) {
        m_modemState = STATE_IDLE;
        setMode(m_modemState);
      }

      m_watchdog = 0U;
    }

#if defined(CONSTANT_SRV_LED)
    LED_RED_service(HIGH);
#elif defined(CONSTANT_SRV_LED_INVERTED)
    LED_RED_service(LOW);
#elif defined(DISCREET_SRV_LED)
    if (m_ledCount == 10000U) LED_RED_service(LOW);
    if (m_ledCount >= 480000U) {
      m_ledCount = 0U;
      LED_RED_service(HIGH);
    };
#elif defined(DISCREET_SRV_LED_INVERTED)
    if (m_ledCount == 10000U) LED_RED_service(HIGH);
    if (m_ledCount >= 480000U) {
      m_ledCount = 0U;
      LED_RED_service(LOW);
    };
#else
    if (m_ledCount >= 24000U) {
      m_ledCount = 0U;
      m_ledValue = !m_ledValue;
      LED_RED_service(m_ledValue);
    }
#endif
  } else {
    if (m_ledCount >= 240000U) {
      m_ledCount = 0U;
      m_ledValue = !m_ledValue;
      LED_RED_service(m_ledValue);
    }
    return;
  }

  // Switch off the transmitter if needed
  if (m_txBuffer.getData() == 0U && m_tx) {
    if(m_cwid_state) { // check for CW ID end of transmission
      m_cwid_state = false;
      // Restoring previous mode
      if (m_TotalModes)
        io.ifConf(m_modemState_prev, true);
    }
    if(m_pocsag_state) { // check for POCSAG end of transmission
      m_pocsag_state = false;
      // Restoring previous mode
      if (m_TotalModes)
        io.ifConf(m_modemState_prev, true);
    }
    setRX(false);
  }

  if(m_modemState_prev == STATE_DMR)
    scantime = SCAN_TIME * 2U;
  else if(m_modemState_prev == STATE_P25)
    scantime = SCAN_TIME;
  else
    scantime = SCAN_TIME;

  if(m_modeTimerCnt >= scantime) {
    m_modeTimerCnt = 0U;
    if( (m_modemState == STATE_IDLE) && (m_scanPauseCnt == 0U) && m_scanEnable && !m_cwid_state && !m_pocsag_state) {
      m_scanPos = (m_scanPos + 1U) % m_TotalModes;
      #if !defined(QUIET_MODE_LEDS)
      setMode(m_Modes[m_scanPos]);
      #endif
      io.ifConf(m_Modes[m_scanPos], true);
    }
  }

  if (m_rxBuffer.getData() >= 1U) {
    m_rxBuffer.get(bit, control);

    switch (m_modemState_prev) {
      case STATE_DMR:
#if defined(DUPLEX)
        if (m_duplex) {
          if (m_tx)
            dmrRX.databit(bit, control);
          else
            dmrIdleRX.databit(bit);
        } else
          dmrDMORX.databit(bit);
#else
        dmrDMORX.databit(bit);
#endif
        break;
      case STATE_P25:
        p25RX.databit(bit);
        break;
      default:
        break;
    }

  }
}

void CIO::start()
{
  m_TotalModes = 0U;

  if(m_dmrEnable) {
    m_Modes[m_TotalModes] = STATE_DMR;
    m_TotalModes++;
  }
  if(m_p25Enable) {
    m_Modes[m_TotalModes] = STATE_P25;
    m_TotalModes++;
  }

#if defined(ENABLE_SCAN_MODE)
  if(m_TotalModes > 1U)
    m_scanEnable = true;
  else {
    m_scanEnable = false;
    setMode(m_modemState);
  }
#else
  m_scanEnable = false;
  setMode(m_modemState);
#endif

  if (m_started)
    return;

  startInt();

  m_started = true;
}

void CIO::write(uint8_t* data, uint16_t length, const uint8_t* control)
{
  if (!m_started)
    return;

  for (uint16_t i = 0U; i < length; i++) {
    if (control == NULL)
      m_txBuffer.put(data[i], MARK_NONE);
    else
      m_txBuffer.put(data[i], control[i]);
  }

  // Switch the transmitter on if needed
  if (!m_tx) {
    setTX();
    m_tx = true;
  }
}

uint16_t CIO::getSpace() const
{
  return m_txBuffer.getSpace();
}

bool CIO::hasTXOverflow()
{
  return m_txBuffer.hasOverflowed();
}

bool CIO::hasRXOverflow()
{
  return m_rxBuffer.hasOverflowed();
}

#if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS)
void CIO::checkBand(uint32_t frequency_rx, uint32_t frequency_tx) {
  if (!(io.hasSingleADF7021())) {
    // There are two ADF7021s on the board
    if (io.isDualBand()) {
      // Dual band
      if ((frequency_tx <= VHF2_MAX) && (frequency_rx <= VHF2_MAX)) {
        // Turn on VHF side
        io.setBandVHF(true);
      } else if ((frequency_tx >= UHF1_MIN) && (frequency_rx >= UHF1_MIN)) {
        // Turn on UHF side
        io.setBandVHF(false);
      }
    }
  }
}

uint8_t CIO::checkZUMspot(uint32_t frequency_rx, uint32_t frequency_tx) {
  if (!(io.hasSingleADF7021())) {
    // There are two ADF7021s on the board
    if (io.isDualBand()) {
      // Dual band
      if ((frequency_tx <= VHF2_MAX) && (frequency_rx <= VHF2_MAX)) {
        // Turn on VHF side
        io.setBandVHF(true);
      } else if ((frequency_tx >= UHF1_MIN) && (frequency_rx >= UHF1_MIN)) {
        // Turn on UHF side
        io.setBandVHF(false);
      }
    } else if (!io.isDualBand()) {
      // Duplex board
      if ((frequency_tx < UHF1_MIN) || (frequency_rx < UHF1_MIN)) {
        // Reject VHF frequencies
        return 4U;
      }
    }
  }
  return 0U;
}
#endif

uint8_t CIO::setFreq(uint32_t frequency_rx, uint32_t frequency_tx, uint8_t rf_power, uint32_t pocsag_freq_tx)
{
  // Configure power level
  setPower(rf_power);

  bool tx_enabled = true;
#if !defined(DISABLE_FREQ_CHECK)
  // Check frequency ranges
  if( !(
          ((frequency_tx >= VHF1_MIN)&&(frequency_tx < VHF1_MAX)) ||
          ((frequency_tx >= UHF1_MIN)&&(frequency_tx < UHF1_MAX)) ||
          ((frequency_tx >= VHF2_MIN)&&(frequency_tx < VHF2_MAX)) ||
          ((frequency_tx >= UHF2_MIN)&&(frequency_tx < UHF2_MAX))
        )
    )
    tx_enabled = false;

  if( !( ((pocsag_freq_tx >= VHF1_MIN)&&(pocsag_freq_tx < VHF1_MAX)) ||
          ((pocsag_freq_tx >= UHF1_MIN)&&(pocsag_freq_tx < UHF1_MAX)) ||
          ((pocsag_freq_tx >= VHF2_MIN)&&(pocsag_freq_tx < VHF2_MAX)) ||
          ((pocsag_freq_tx >= UHF2_MIN)&&(pocsag_freq_tx < UHF2_MAX))
        )
    )
     tx_enabled = false;
#endif

#if  1 // !defined(DISABLE_FREQ_BAN)
  // Always Check banned frequency ranges
  // receive on any frequency you wish.
  // ((frequency_rx >= BAN2_MIN)&&(frequency_rx <= BAN2_MAX))
  // ((frequency_rx >= BAN1_MIN)&&(frequency_rx <= BAN1_MAX))

  if(  ((frequency_tx >= BAN1_MIN)&&(frequency_tx <= BAN1_MAX)) ||
 	   ((frequency_tx >= BAN2_MIN)&&(frequency_tx <= BAN2_MAX)) ||
       ((frequency_tx >= BAN_OPPL)&&(frequency_tx <= BAN_OPPH)) ||
	   ((frequency_tx >= BAN_OPSL)&&(frequency_tx <= BAN_OPSH))
    )
	  tx_enabled = false;

  if(   ((pocsag_freq_tx >= BAN1_MIN)&&(pocsag_freq_tx <= BAN1_MAX)) ||
       ((pocsag_freq_tx >= BAN_OPPL)&&(pocsag_freq_tx <= BAN_OPPH)) ||
       ((pocsag_freq_tx >= BAN_OPSL)&&(pocsag_freq_tx <= BAN_OPSH)) ||
  	    ((pocsag_freq_tx >= BAN2_MIN)&&(pocsag_freq_tx <= BAN2_MAX))
    )
	  tx_enabled = false;
#endif

	m_bTransmitAllowed = tx_enabled;

	ALWAYS("TX IS %s on freq %lu", m_bTransmitAllowed ? _BOLDRED "ALLOWED - DANGER" _RESET: _GREEN "INHIBITED - SAFE" _RESET, frequency_tx );

 	return setFreqHw( frequency_rx, frequency_tx, m_bTransmitAllowed ? rf_power : 0, pocsag_freq_tx);

}

uint8_t CIO::setFreqHw(uint32_t frequency_rx, uint32_t frequency_tx, uint8_t rf_power, uint32_t pocsag_freq_tx)
{
// Check if we have a single, dualband or duplex board
#if defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS)
  if (checkZUMspot(frequency_rx, frequency_tx) > 0) {
    return 4U;
  }
#endif

  // Configure frequency
  m_frequency_rx = frequency_rx;
  m_frequency_tx = frequency_tx;
  m_pocsag_freq_tx = pocsag_freq_tx;

  return 0U;
}

void CIO::setMode(MMDVM_STATE modemState)
{
    LED_P25_RED(modemState == STATE_P25);
    LED_DMR_YELLOW(modemState  == STATE_DMR);
    LED_POCSAG_BLUE(modemState == STATE_POCSAG);
}

void CIO::setDecode(bool dcd)
{
  if (dcd != m_dcd) {
    m_scanPauseCnt = 1U;
    LED_COS_YELW(dcd ? true : false);
  }

  m_dcd = dcd;
}

void CIO::resetWatchdog()
{
  m_watchdog = 0U;
}

uint32_t CIO::getWatchdog()
{
  return m_watchdog;
}

void CIO::getIntCounter(uint16_t &int1, uint16_t &int2)
{
  int1 = m_int1counter;
  int2 = m_int2counter;
  m_int1counter = 0U;
  m_int2counter = 0U;
}
