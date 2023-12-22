/*
 *   Copyright (C) 2020,2021 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Jim McLaughlin KI6ZUM
 *   Copyright (C) 2016,2017,2018,2019,2020 by Andy Uribe CA6JAU
 *   Copyright (C) 2017 by Danilo DB4PLE
 *
 *   Some of the code is based on work of Guus Van Dooren PE1PLM:
 *   https://github.com/ki6zum/gmsk-dstar/blob/master/firmware/dvmega/dvmega.ino
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
#include <cassert>

#include "Config.h"
#include "Debug.h"

#include "Globals.h"
#include "IO.h"
#include "ADF7021.h"
#include <math.h>

volatile bool totx_request = false;
volatile bool torx_request = false;
volatile bool even = true;
static uint32_t last_clk = 2U;

volatile uint32_t  AD7021_control_word;

uint32_t           ADF7021_RX_REG0;
uint32_t           ADF7021_TX_REG0;
uint32_t           ADF7021_REG1;
uint32_t           div2;
uint32_t           f_div;

uint16_t           m_dstarDev;
uint16_t           m_dmrDev;
uint16_t           m_p25Dev;
uint16_t           m_m17Dev;
uint16_t           m_pocsagDev;

uint32_t		   mirror[2][16] = {0};
const REG4_DEMODULATOR_SETUP REG4_ALLONES = { .ALL_32 = 0xFFFFFFFF};
const REG3_TX_RX_CONTROL REG3_ALLONES = { .ALL_32 = 0xFFFFFFFF };
void diffDump_AD7021(void)
{
	int i;
	ALWAYS(_GREEN "changes" _RESET);
	for (i = 0; i < 16; i++)
	{
	  uint32_t delta = mirror[0][i] ^ mirror[1][i];
	  if (delta)
	  	ALWAYS(_YELLOW "!= ADF7021_REG%02d 0x%08X // 0x%08X" _RESET, i, mirror[0][i], delta);
	  else
	  	ALWAYS(_GREEN  "== ADF7021_REG%02d 0x%08X // 0x%08X" _RESET, i, mirror[0][i], delta);
	}
	memcpy(mirror[1], mirror[0], sizeof mirror[0]);
	ALWAYS(" ");
}
static void Send_AD7021_control_shift()
{
  int index = AD7021_control_word & 0x0F;
  mirror [0][index] = AD7021_control_word;
  for (int AD7021_counter = 31; AD7021_counter >= 0; AD7021_counter--) {
    if (bitRead(AD7021_control_word, AD7021_counter) == HIGH)
      io.SDIO_WRITE_DATABIT(HIGH);
    else
      io.SDIO_WRITE_DATABIT(LOW);

    io.dlybit();
    io.SDIO_SET_CLOCKBIT(HIGH);
    io.dlybit();
    io.SDIO_SET_CLOCKBIT(LOW);
  }

  // to keep SDATA signal at defined level when idle (not required)
  io.SDIO_WRITE_DATABIT(LOW);
}

static void Send_AD7021_control_slePulse()
{
  io.SDIO_LATCHCMD_U1(HIGH);
  io.dlybit();
  io.SDIO_LATCHCMD_U1(LOW);
}

void Send_AD7021_control(bool doSle)
{
  Send_AD7021_control_shift();

  if (doSle)
    Send_AD7021_control_slePulse();
}

#if defined(DUPLEX)
static  void Send_AD7021_control_sle2Pulse()
{
  io.SDIO_LATCHCMD_U2(HIGH);
  io.dlybit();
  io.SDIO_LATCHCMD_U2(LOW);
}

void Send_AD7021_control2(bool doSle)
{
  Send_AD7021_control_shift();

  if (doSle)
    Send_AD7021_control_sle2Pulse();
}
#if defined(ADF7021_14_7456)
	#define XTAL  14745600
#else
	#define XTAL  12288000
	#error ok really?
#endif
inline void CIO::monitorADF7021(uint8_t reg, uint32_t value)
{
	mirror[0][reg] = value;
}

void CIO::summaryADF7021(void)
{
    REG0_N_REGISTER 		REG0;
	REG1_VCO_CONTROL 		REG1;
    REG2_TX_CONTROL 		REG2;
	REG3_TX_RX_CONTROL 		REG3;
    REG4_DEMODULATOR_SETUP 	REG4;
    REG7_READBACK_SETUP		REG7;
    REG12_SWD_REGISTER		REG12;
    REG13_FSK_CONTROL 		REG13;
    REG10_AFC_CONTROL 		REG10;
    REG11_SYNCWORD_REGISTER REG11;
	readRSSI();  // testing reg 7
	REG0.ALL_32 = mirror[0][0];
	REG1.ALL_32 = mirror[0][1];
	REG2.ALL_32 = mirror[0][2];
	REG3.ALL_32 = mirror[0][3];
	REG4.ALL_32 = mirror[0][4];
	REG7.ALL_32 = mirror[0][7];
	REG10.ALL_32 = mirror[0][10];
	REG12.ALL_32 = mirror[0][10];
	REG11.ALL_32 = mirror[0][11];
	REG13.ALL_32 = mirror[0][13];
	uint32_t PFD = XTAL /REG1.RF_R_DIVIDER; //used many places
	if (REG1.XTAL_DOUBLER) PFD = 2 * PFD;
	ALWAYS("note: PFD = %d Hz", PFD);
    ALWAYS("-------------------");
	ALWAYS("REG0.ALL = 0x%08X\n", REG0.ALL_32);
	ALWAYS("FRACTIONAL_N =\t%d", REG0.FRACTIONAL_N);
	ALWAYS("INTEGER_N =\t%d", REG0.INTEGER_N);
	ALWAYS("RECIEVE ON=\t%d", REG0.RECIEVE_ON);
	ALWAYS("UART_ENABLED=\t%d", REG0.UART_MODE_ON);
	static const char *mux[8] = {"Ready", "cal-done", "lock-det", "rssi-rdy", "tx-rx", "ZERO", "FLOAT", "ONE" };
	ALWAYS("MUXOUT_SEL=\t%s", mux[REG0.MUXOUT_SEL]);
	uint32_t rfOut = PFD * (REG0.INTEGER_N + REG0.FRACTIONAL_N/(1<<15));
	if (REG1.RF_R_DIVIDER) rfOut /= 2;
	ALWAYS("*** rfout =\t\t%d Hz", rfOut);
    ALWAYS("-------------------");

    ALWAYS("REG1.ALL = 0x%08X\n", REG1.ALL_32);
	ALWAYS("REG1.RF_R_DIVIDER =\t%d", REG1.RF_R_DIVIDER);
	ALWAYS("REG1.CLKOUT_DIV_REG =\t%d", REG1.CLKOUT_DIV_REG);
	ALWAYS("REG1.XTAL_DOUBLER =\t%d", REG1.XTAL_DOUBLER );
	ALWAYS("REG1.XTAL_OSC_EN =\t%d", REG1.XTAL_OSC_EN);

	ALWAYS("REG1.XTAL_BIAS =\t%d uA", REG1.XTAL_BIAS * 5 + 20);

	static const char *current[4]={ ".3", ".9", "1.5", "2.1"};
	ALWAYS("REG1.CP_CURRENT =\t%s mA", current[REG1.CP_CURRENT]);

	ALWAYS("REG1.VCO_BIAS =\t%d uA", 250 * REG1.VCO_BIAS );


	ALWAYS("REG1.VCO_ADJ =\t%d", REG1.VCO_ADJ);
	ALWAYS("REG1.VCO_EXTERN_EN =\t%d", REG1.VCO_EXTERN_EN);


    ALWAYS("-------------------");
    ALWAYS("REG2.ALL = 0x%08X\n", REG2.ALL_32);

    static const char *MOS[8] = {"2FSK", "G2FSK", "3FSK", "4FSK", "OS2FSK", "RS2FSK", "RC3FSK", "RC4FSK"};
    ALWAYS("REG2.MOD_SCHEME =\t%s", MOS[REG2.MOD_SCHEME_REG]);
    ALWAYS("REG2.PA_ENABLE =\t%d", REG2.PA_ENABLE_REG);
	ALWAYS("REG2.PA_RAMP =\t%d c/bit", REG2.PA_RAMP_REG);

	static const uint8_t bias[4] = { 5, 7, 9, 11};
	ALWAYS("REG2.PA_BIAS =\t%d uA", bias[REG2.PA_BIAS]);

	ALWAYS("REG2.POWER_AMP_REG =\t%d dBm", REG2.POWER_AMP_REG);

	uint32_t txFdev = REG2.TX_DEVIATON_REG * PFD / (1 << 16);
	if (REG1.RF_DIV2_EN) txFdev /=2;  //pg 49

	ALWAYS("REG2.TX_DEVIATION =\t%d Hz", txFdev);

	static const char *ivert[4] = { "norm", "i-clk", "i-data", "i-clkNdata"};
	ALWAYS("REG2.TX_INVERT =\t%s", ivert[REG2.TX_INVERT_REG]);
	static const char *rcosine[2] = { ".5", ".7" };
	ALWAYS("REG2.TX_RCOSINE =\t%s uS",rcosine[REG2.TX_RCOSINE_REG]);

    ALWAYS("-------------------");

	char const *scheme[] = { "2FSK linear\0", "2FSK correl8r\0", "3FSK\0", "4FSK\0"};
	char const *ifbw[]   = { "12.5", "18.75", "25.0", "ERROR" };
    char const *invert[] = { "normal clk", "invert clk", "invert data", "invert clk/data" };
	char const *dotcross[] = { "cross product", "dot product", "ERROR" };


	// quickly recover demod clock from xtal and demod reg.
    uint32_t demod_clk = XTAL / REG3.DEMOD_CLK_REG;
    uint32_t cdr_clock = demod_clk / REG3.CDR_CLK_REG;
    uint32_t baud = cdr_clock / 32;

    ALWAYS("REG3.ALL = 0x%08X\n", REG3.ALL_32);
    ALWAYS("DEMOD_CLK\t= %d Hz", demod_clk);
    ALWAYS("CDR_CLK\t= %d Hz", cdr_clock);
    ALWAYS("BAUD\t\t= %d bps", baud);

	float K = (REG4.DISCRIM_BW * 400000.) / (float)demod_clk;

	unsigned k_multiplier;  	// used for K value determination
	float	 fCutoffHz;

	switch (REG4.DEMODSCHEME)
	{
		case 0:  //2fsk linear
		case 1:  //2fsk corel8d
			k_multiplier = 1;
			fCutoffHz = baud * .75;
			break;

		case 2:  //3fsk
			k_multiplier = 2;
			fCutoffHz = baud;
	    	break;
	    case 3:  //4fsk
	    	k_multiplier = 4;
			fCutoffHz = baud * 1.6;
	    	break;
	    default:
	    	assert(0);
	    	break;
	}

	float fDev;
	fDev = 100000. / (K * k_multiplier);

    float fDiscBandwidth;
    fDiscBandwidth = (float)demod_clk * K / 400000.0;

    ALWAYS("Symbol dev =\t%d hz(?)", (uint32_t)fDiscBandwidth);
    ALWAYS("K = %d \tfDev = %d \tfCutoff = %d",(uint32_t) K,(uint32_t) fDev, (uint32_t)fCutoffHz);

    ALWAYS("-------------------");
	ALWAYS("REG4.ALL 0x%08X\n", REG4.ALL_32);
    ALWAYS("REG4.X_IF_BW =\t0x%X\t%s khz", REG4.XIF_BW, ifbw[REG4.XIF_BW]);
    ALWAYS("REG4.POST_DEMOD_BW =\t0x%X", REG4.POST_DEMOD_BW);
    ALWAYS("REG4.DISCRIM_BW =\t0x%X", REG4.DISCRIM_BW);
    ALWAYS("REG4.INVERTCTL =\t0x%X\t%s", REG4.INVERTCTL, invert[REG4.INVERTCTL]);
    ALWAYS("REG4.DOTCROSS =\t0x%X\t%s", REG4.DOTCROSS, dotcross[REG4.DOTCROSS]);
    ALWAYS("REG4.DEMODSCHEME =\t0x%X\t%s", REG4.DEMODSCHEME, scheme[REG4.DEMODSCHEME]);
    ALWAYS("REG4.ADDRESS_REGISTER=\t0x%X", REG4.ADDRESS);
    ALWAYS("-------------------");

   	ALWAYS("REG7.ALL = 0x%08X\n", REG7.ALL_32);

   	static const char *adcmd[4] = {"rssi", "voltage", "tempC", "extPin" };
	ALWAYS("REG7.SET_ADC_MODE =\t%s", adcmd[REG7.SET_ADC_MODE]);

	// select value 1 below to read adc value above
	// see page 45 for sequence to read back the data.
	// see uint16_t CIO::readRSSI() for how command is sent and data recvd

	static const char *rdbck[4] = {"afc", "adc_out", "cal", "sil-rev" };
	ALWAYS("REG7.READBACK_MODE =\t%s val", rdbck[REG7.READBACK_MODE]);
	ALWAYS("REG7.READBACK_ENABLED =\t%d", REG7.READBACK_ENABLED);

    ALWAYS("-------------------");
	ALWAYS("REG11.ALL = 0x%08X\n", REG11.ALL_32);
	ALWAYS("REG11.SYNC_LENGTH =\t%d bits", REG11.SYNC_LENGTH * 4 + 12);
	ALWAYS("REG11.TOLERANCE =\t%d bit errors", REG11.TOLERANCE);
	ALWAYS("REG11.SEQUENCE =\t0x%08X", REG11.SEQUENCE);
    ALWAYS("-------------------");

	// lock AFC and AGC thresholds when?
    ALWAYS("REG12.ALL = 0x%08X\n", REG12.ALL_32);
    ALWAYS("(AFC/AGG locks when ...");
	static const char* lck[4] = { "neverlock", "lockafter-sync", "lockafter-payload", "lock-NOW"};
	ALWAYS("REG12.LOCK_MODE =\t%s", lck[REG12.LOCK_MODE]);
	static const char* lmode[4] = { "swd_LO", "swd=sync-det", "swd=payload-done", "swd-HI"};
	ALWAYS("REG12.SWD_MODE (swd pin)=\t%s", lmode[REG12.SWD_MODE]);
	ALWAYS("REG12.PAYLOAD_LENGTH =\t%d bytes", REG12.PAYLOAD_LEN);

    ALWAYS("-------------------");
    ALWAYS("REG13.ALL = 0x%08X\n", REG13.ALL_32);
    ALWAYS("REG13.SLICER_THRHLD =\t%d", REG13.SLICER_THRES_REG);
    ALWAYS("REG13.VITERBI EN\t= %d", REG13.VITERBI_ON_REG);
    ALWAYS("REG13.PHASE_CORR_EN= %d", REG13.PHASE_CRCTN_EN_REG);
    ALWAYS("REG13.PHASE_CORR_MEMSIZE= %d", REG13.VITERBI_MEM_REG);
    ALWAYS("REG13.PHASE_FSK_THRSHLD= %d", REG13.FSK3_THRES_REG);
    ALWAYS("REG13.PHASE_FSK_TIME= %d", REG13.FSK3_TIME_REG);

    ALWAYS("-------------------");
	ALWAYS("REG10.AFC_ENABLED =\t%d",REG10.AFC_ENABLED_REG);
	ALWAYS("REG10.AFC_SCALING =\t%d", REG10.AFC_SCALING_REG);
	ALWAYS("REG10.AFC_KI_REG =\t%d", REG10.AFC_KI_REG);
	ALWAYS("REG10.AFC_KP_REG =\t%d", REG10.AFC_KP_REG);
	ALWAYS("REG10.AFC_MAX_RNG =\t%d", REG10.AFC_MAX_RANGE_REG);

    ALWAYS("\n-------------------");

}
//-------------------------------------------------------------

#if defined(SEND_RSSI_DATA)
uint16_t CIO::readRSSI()
{
  uint32_t AD7021_RB;
  uint16_t RB_word = 0U;
  uint8_t RB_code, gain_code, gain_corr;

  // Register 7, readback enable, ADC RSSI mode
  AD7021_RB = 0x0147;

  // Send control register
  for (int AD7021_counter = 8; AD7021_counter >= 0; AD7021_counter--) {
    if (bitRead(AD7021_RB, AD7021_counter) == HIGH)
      SDIO_WRITE_DATABIT(HIGH);
    else
      SDIO_WRITE_DATABIT(LOW);

    dlybit();
    SDIO_SET_CLOCKBIT(HIGH);
    dlybit();
    SDIO_SET_CLOCKBIT(LOW);
  }

  SDIO_WRITE_DATABIT(LOW);

#if defined(DUPLEX)
  if (m_duplex || m_calState == STATE_RSSICAL)
    SDIO_LATCHCMD_U2(HIGH);
  else
    SDIO_LATCHCMD_U1(HIGH);
#else
  SDIO_LATCHCMD_U1(HIGH);
#endif

  dlybit();

  // Read SREAD pin
  for (int AD7021_counter = 17; AD7021_counter >= 0; AD7021_counter--) {
    SDIO_SET_CLOCKBIT(HIGH);
    dlybit();

    if ((AD7021_counter != 17) && (AD7021_counter != 0))
      RB_word |= ((SDIO_READ_DATABIT() & 0x01) << (AD7021_counter - 1));

    SDIO_SET_CLOCKBIT(LOW);
    dlybit();
  }

#if defined(DUPLEX)
  if (m_duplex || m_calState == STATE_RSSICAL)
    SDIO_LATCHCMD_U2(LOW);
  else
    SDIO_LATCHCMD_U1(LOW);
#else
  SDIO_LATCHCMD_U1(LOW);
#endif

  // Process RSSI code
  RB_code = RB_word & 0x7f;
  gain_code = (RB_word >> 7) & 0x0f;

  switch(gain_code) {
    case 0b1010:
      gain_corr = 0U;
      break;
    case 0b0110:
      gain_corr = 24U;
      break;
    case 0b0101:
      gain_corr = 38U;
      break;
    case 0b0100:
      gain_corr = 58U;
      break;
    case 0b0000:
      gain_corr = 86U;
      break;
    default:
      gain_corr = 0U;
      break;
  }

  return ( 130 - (RB_code + gain_corr)/2 );

}
#endif

void CIO::ifConf(MMDVM_STATE modemState, bool reset)
{
  float    divider;

  uint32_t ADF7021_REG2  = 0U;
  uint32_t ADF7021_REG3  = 0U;
  uint32_t ADF7021_REG4  = 0U;
  uint32_t ADF7021_REG10 = 0U;
  uint32_t ADF7021_REG13 = 0U;
  int32_t AFC_OFFSET = 0;

  uint32_t frequency_tx_tmp, frequency_rx_tmp;

  if (modemState != STATE_CWID && modemState != STATE_POCSAG)
    m_modemState_prev = modemState;

  // Change frequency for POCSAG mode, store a backup of DV frequencies
  if (modemState == STATE_POCSAG) {
    frequency_tx_tmp = m_frequency_tx;
    frequency_rx_tmp = m_frequency_rx;
    m_frequency_tx   = m_pocsag_freq_tx;
    m_frequency_rx   = m_pocsag_freq_tx;
  }

  #if defined (ZUMSPOT_ADF7021) || defined(SKYBRIDGE_HS)
  io.checkBand(m_frequency_rx, m_frequency_tx);
  #endif

  // Toggle CE pin for ADF7021 reset
  if (reset) {
    RESET_ALL_MODEMS(LOW);
    delay_reset();
    RESET_ALL_MODEMS(HIGH);
    delay_reset();
  }

  // Check frequency band
  if ((m_frequency_tx >= VHF1_MIN) && (m_frequency_tx < VHF1_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_VHF1;         // VHF1, external VCO
    div2 = 1U;
  } else if ((m_frequency_tx >= VHF2_MIN) && (m_frequency_tx < VHF2_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_VHF2;         // VHF1, external VCO
    div2 = 1U;
  } else if ((m_frequency_tx >= UHF1_MIN)&&(m_frequency_tx < UHF1_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_UHF1;         // UHF1, internal VCO
    div2 = 1U;
  } else if ((m_frequency_tx >= UHF2_MIN)&&(m_frequency_tx < UHF2_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_UHF2;         // UHF2, internal VCO
    div2 = 2U;
  } else {
    ADF7021_REG1 = ADF7021_REG1_UHF1;         // UHF1, internal VCO
    div2 = 1U;
  }

  if (div2 == 1U)
    f_div = 2U;
  else
    f_div = 1U;

  switch (modemState) {
    case STATE_POCSAG:
      AFC_OFFSET = 0;
      break;
    case STATE_DMR:
    case STATE_CWID:
      AFC_OFFSET = AFC_OFFSET_DMR;
      break;
    case STATE_P25:
      AFC_OFFSET = AFC_OFFSET_P25;
      break;
    default:
      break;
  }

  if (div2 == 1U)
    divider = (m_frequency_rx - 100000 + AFC_OFFSET) / (ADF7021_PFD / 2U);
  else
    divider = (m_frequency_rx - 100000 + (2 * AFC_OFFSET)) / ADF7021_PFD;

  m_RX_N_divider = floor(divider);
  divider = (divider - m_RX_N_divider) * 32768;
  m_RX_F_divider = floor(divider + 0.5);

  ADF7021_RX_REG0  = (uint32_t) 0b0000;

#if defined(BIDIR_DATA_PIN)
  ADF7021_RX_REG0 |= (uint32_t) 0b01001   << 27;   // mux regulator/receive
#else
  ADF7021_RX_REG0 |= (uint32_t) 0b01011   << 27;   // mux regulator/uart-spi enabled/receive
#endif

  ADF7021_RX_REG0 |= (uint32_t) m_RX_N_divider << 19;   // frequency;
  ADF7021_RX_REG0 |= (uint32_t) m_RX_F_divider << 4;    // frequency;

  if (div2 == 1U)
    divider = m_frequency_tx / (ADF7021_PFD / 2U);
  else
    divider = m_frequency_tx / ADF7021_PFD;

  m_TX_N_divider = floor(divider);
  divider = (divider - m_TX_N_divider) * 32768;
  m_TX_F_divider = floor(divider + 0.5);

  ADF7021_TX_REG0  = (uint32_t) 0b0000;            // register 0

#if defined(BIDIR_DATA_PIN)
  ADF7021_TX_REG0 |= (uint32_t) 0b01000   << 27;   // mux regulator/transmit
#else
  ADF7021_TX_REG0 |= (uint32_t) 0b01010   << 27;   // mux regulator/uart-spi enabled/transmit
#endif

  ADF7021_TX_REG0 |= (uint32_t) m_TX_N_divider << 19;   // frequency;
  ADF7021_TX_REG0 |= (uint32_t) m_TX_F_divider << 4;    // frequency;

  switch (modemState) {
    case STATE_CWID:
      // CW ID base configuration: DMR
      // Dev: +1 symb (variable), symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_DMR;
      ADF7021_REG10 = ADF7021_REG10_DMR;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_DMR       << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_DMR       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                      << 30;  // IF filter (25 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_DMR    << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_cwIdTXLevel / div2)    << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
      break;

    case STATE_POCSAG:
      // Dev: 4500 Hz, symb rate = 1200

      ADF7021_REG3 = ADF7021_REG3_POCSAG;
      ADF7021_REG10 = ADF7021_REG10_POCSAG;

      ADF7021_REG4  = (uint32_t) 0b0100                     << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b000                      << 4;   // 2FSK linear demodulator
      ADF7021_REG4 |= (uint32_t) 0b1                        << 7;
      ADF7021_REG4 |= (uint32_t) 0b10                       << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_POCSAG     << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_POCSAG     << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                       << 30;  // IF filter (25 kHz)

      // Register 13 not used with 2FSK
      ADF7021_REG13 = (uint32_t) 0b1101                     << 0;   // register 13

      ADF7021_REG2  = (uint32_t) 0b10                       << 28;  // inverted data, clock normal
      ADF7021_REG2 |= (uint32_t) (m_pocsagDev / div2)       << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b000                      << 4;   // modulation (2FSK)
      break;
/*
    case STATE_DSTAR:
      // Dev: 1200 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_DSTAR;
      ADF7021_REG10 = ADF7021_REG10_DSTAR;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b001                     << 4;   // mode, GMSK
      ADF7021_REG4 |= (uint32_t) 0b1                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b10                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_DSTAR     << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_DSTAR     << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b00                      << 30;  // IF filter (12.5 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_DSTAR  << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b00                       << 28;  // clock normal
      ADF7021_REG2 |= (uint32_t) (m_dstarDev / div2)       << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b001                     << 4;   // modulation (GMSK)
      break;
*/

    case STATE_DMR:
      // Dev: +1 symb 648 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_DMR;
      ADF7021_REG10 = ADF7021_REG10_DMR;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_DMR       << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_DMR       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                      << 30;  // IF filter (25 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_DMR    << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_dmrDev / div2)         << 19;  // deviation
#if defined(ADF7021_DISABLE_RC_4FSK)
      ADF7021_REG2 |= (uint32_t) 0b011                     << 4;   // modulation (4FSK)
#else
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
#endif
      break;

/*
    case STATE_YSF:
      // Dev: +1 symb 900/450 Hz, symb rate = 4800

      ADF7021_REG3 = (m_LoDevYSF ? ADF7021_REG3_YSF_L : ADF7021_REG3_YSF_H);
      ADF7021_REG10 = ADF7021_REG10_YSF;

      // K=28
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) (m_LoDevYSF ? ADF7021_DISC_BW_YSF_L : ADF7021_DISC_BW_YSF_H) << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_YSF       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                      << 30;  // IF filter (25 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) (m_LoDevYSF ? ADF7021_SLICER_TH_YSF_L : ADF7021_SLICER_TH_YSF_H) << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_ysfDev / div2)         << 19;  // deviation
#if defined(ADF7021_DISABLE_RC_4FSK)
      ADF7021_REG2 |= (uint32_t) 0b011                     << 4;   // modulation (4FSK)
#else
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
#endif
      break;
*/
    case STATE_P25:
      // Dev: +1 symb 600 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_P25;
      ADF7021_REG10 = ADF7021_REG10_P25;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_P25       << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_P25       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b00                      << 30;  // IF filter (12.5 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_P25    << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_p25Dev / div2)         << 19;  // deviation
#if defined(ENABLE_P25_WIDE) || defined(ADF7021_DISABLE_RC_4FSK)
      ADF7021_REG2 |= (uint32_t) 0b011                     << 4;   // modulation (4FSK)
#else
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
#endif
      break;
/*
    case STATE_NXDN:
      // Dev: +1 symb 350 Hz, symb rate = 2400

      ADF7021_REG3 = ADF7021_REG3_NXDN;
      ADF7021_REG10 = ADF7021_REG10_NXDN;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_NXDN      << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_NXDN      << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b00                      << 30;  // IF filter (12.5 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_NXDN   << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_nxdnDev / div2)        << 19;  // deviation
#if defined(ADF7021_DISABLE_RC_4FSK)
      ADF7021_REG2 |= (uint32_t) 0b011                     << 4;   // modulation (4FSK)
#else
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
#endif
      break;
*/

/*
    case STATE_M17:
      // Dev: +1 symb 800 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_M17;
      ADF7021_REG10 = ADF7021_REG10_M17;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_M17       << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_M17       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                      << 30;  // IF filter (25 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_M17    << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_m17Dev / div2)         << 19;  // deviation
#if defined(ADF7021_DISABLE_RC_4FSK)
      ADF7021_REG2 |= (uint32_t) 0b011                     << 4;   // modulation (4FSK)
#else
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
#endif
      break;
*/

    default:
      break;
  }

  // VCO/OSCILLATOR (REG1)
  AD7021_control_word = ADF7021_REG1;
  Send_AD7021_control();

  // TX/RX CLOCK (3)
  AD7021_control_word = ADF7021_REG3;
  Send_AD7021_control();

  // DEMOD (4)
  AD7021_control_word = ADF7021_REG4;
  Send_AD7021_control();

  // IF fine cal (6)
  AD7021_control_word = ADF7021_REG6;
  Send_AD7021_control();

  // IF coarse cal (5)
  AD7021_control_word = ADF7021_REG5;
  Send_AD7021_control();

  // Delay for filter calibration
  delay_IFcal();

  // Frequency RX (0)
  setRX();

  // MODULATION (2)
  ADF7021_REG2 |= (uint32_t) 0b0010;               // register 2
  ADF7021_REG2 |= m_bTransmitAllowed ? (uint32_t) m_power << 13 : 0;  // power level
  ADF7021_REG2 |= (uint32_t) 0b110001      << 7;   // PA
  AD7021_control_word = ADF7021_REG2;
  Send_AD7021_control();

  // TEST DAC (14)
#if defined(TEST_DAC)
  AD7021_control_word = 0x0000001E;
#else
  AD7021_control_word = 0x0000000E;
#endif
  Send_AD7021_control();

  // AGC (auto, defaults) (9)
#if defined(AD7021_GAIN_AUTO)
  AD7021_control_word = 0x000231E9; // AGC ON, normal operation
#elif defined(AD7021_GAIN_AUTO_LIN)
  AD7021_control_word = 0x100231E9; // AGC ON, LNA high linearity
#elif defined(AD7021_GAIN_LOW)
  AD7021_control_word = 0x120631E9; // AGC OFF, low gain, LNA high linearity
#elif defined(AD7021_GAIN_HIGH)
  AD7021_control_word = 0x00A631E9; // AGC OFF, high gain
#endif
  Send_AD7021_control();

  // AFC (10)
  AD7021_control_word = ADF7021_REG10;
  Send_AD7021_control();

  // SYNC WORD DET (11)
  AD7021_control_word = 0x0000003B;
  Send_AD7021_control();

  // SWD/THRESHOLD (12)
  AD7021_control_word = 0x0000010C;
  Send_AD7021_control();

  // 3FSK/4FSK DEMOD (13)
  AD7021_control_word = ADF7021_REG13;
  Send_AD7021_control();

#if defined(TEST_TX)
  LED_PTT_RED(HIGH);
  AD7021_control_word = ADF7021_TX_REG0;
  Send_AD7021_control();
  // TEST MODE (TX carrier only) (15)
  AD7021_control_word = 0x000E010F;
#else
  // TEST MODE (disabled) (15)
  AD7021_control_word = 0x000E000F;
#endif
  Send_AD7021_control();

  // Restore normal DV frequencies
  if (modemState == STATE_POCSAG) {
    m_frequency_tx = frequency_tx_tmp;
    m_frequency_rx = frequency_rx_tmp;
  }

#if defined(DUPLEX)
if (m_duplex && (modemState != STATE_CWID && modemState != STATE_POCSAG))
  ifConf2(modemState);
#endif
}

#if defined(DUPLEX)
void CIO::ifConf2(MMDVM_STATE modemState)
{
  uint32_t ADF7021_REG2  = 0U;
  uint32_t ADF7021_REG3  = 0U;
  uint32_t ADF7021_REG4  = 0U;
  uint32_t ADF7021_REG10 = 0U;
  uint32_t ADF7021_REG13 = 0U;

  switch (modemState) {
/*
    case STATE_DSTAR:
      // Dev: 1200 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_DSTAR;
      ADF7021_REG10 = ADF7021_REG10_DSTAR;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b001                     << 4;   // mode, GMSK
      ADF7021_REG4 |= (uint32_t) 0b1                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b10                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_DSTAR     << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_DSTAR     << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b00                      << 30;  // IF filter (12.5 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_DSTAR  << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b00                       << 28;  // clock normal
      ADF7021_REG2 |= (uint32_t) (m_dstarDev / div2)<< 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b001                     << 4;   // modulation (GMSK)
      break;
*/
    case STATE_DMR:
      // Dev: +1 symb 648 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_DMR;
      ADF7021_REG10 = ADF7021_REG10_DMR;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_DMR       << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_DMR       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                      << 30;  // IF filter (25 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_DMR    << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_dmrDev / div2)  << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
      break;
/*
    case STATE_YSF:
      // Dev: +1 symb 900/450 Hz, symb rate = 4800

      ADF7021_REG3 = (m_LoDevYSF ? ADF7021_REG3_YSF_L : ADF7021_REG3_YSF_H);
      ADF7021_REG10 = ADF7021_REG10_YSF;

      // K=28
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) (m_LoDevYSF ? ADF7021_DISC_BW_YSF_L : ADF7021_DISC_BW_YSF_H) << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_YSF       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                      << 30;  // IF filter (25 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) (m_LoDevYSF ? ADF7021_SLICER_TH_YSF_L : ADF7021_SLICER_TH_YSF_H) << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_ysfDev / div2)  << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
      break;
*/
    case STATE_P25:
      // Dev: +1 symb 600 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_P25;
      ADF7021_REG10 = ADF7021_REG10_P25;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_P25       << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_P25       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b00                      << 30;  // IF filter (12.5 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_P25    << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_p25Dev / div2)  << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
      break;
/*
    case STATE_NXDN:
      // Dev: +1 symb 350 Hz, symb rate = 2400

      ADF7021_REG3 = ADF7021_REG3_NXDN;
      ADF7021_REG10 = ADF7021_REG10_NXDN;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_NXDN      << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_NXDN      << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b00                      << 30;  // IF filter (12.5 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_NXDN   << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_nxdnDev / div2) << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
      break;
*/

/*
    case STATE_M17:
      // Dev: +1 symb 800 Hz, symb rate = 4800

      ADF7021_REG3 = ADF7021_REG3_M17;
      ADF7021_REG10 = ADF7021_REG10_M17;

      // K=32
      ADF7021_REG4  = (uint32_t) 0b0100                    << 0;   // register 4
      ADF7021_REG4 |= (uint32_t) 0b011                     << 4;   // mode, 4FSK
      ADF7021_REG4 |= (uint32_t) 0b0                       << 7;
      ADF7021_REG4 |= (uint32_t) 0b11                      << 8;
      ADF7021_REG4 |= (uint32_t) ADF7021_DISC_BW_M17       << 10;  // Disc BW
      ADF7021_REG4 |= (uint32_t) ADF7021_POST_BW_M17       << 20;  // Post dem BW
      ADF7021_REG4 |= (uint32_t) 0b10                      << 30;  // IF filter (25 kHz)

      ADF7021_REG13 = (uint32_t) 0b1101                    << 0;   // register 13
      ADF7021_REG13 |= (uint32_t) ADF7021_SLICER_TH_M17    << 4;   // slicer threshold

      ADF7021_REG2 = (uint32_t) 0b10                       << 28;  // invert data (and RC alpha = 0.5)
      ADF7021_REG2 |= (uint32_t) (m_m17Dev / div2)  << 19;  // deviation
      ADF7021_REG2 |= (uint32_t) 0b111                     << 4;   // modulation (RC 4FSK)
      break;
*/
    default:
      break;
  }

  // VCO/OSCILLATOR (1)
  AD7021_control_word = ADF7021_REG1;
  Send_AD7021_control2();

  // TX/RX CLOCK (3)
  AD7021_control_word = ADF7021_REG3;
  Send_AD7021_control2();

  // DEMOD (4)
  AD7021_control_word = ADF7021_REG4;
  Send_AD7021_control2();

  // IF fine cal (6)
  AD7021_control_word = ADF7021_REG6;
  Send_AD7021_control2();

  // IF coarse cal (5)
  AD7021_control_word = ADF7021_REG5;
  Send_AD7021_control2();

  // Delay for coarse IF filter calibration
  delay_IFcal();

  // Frequency RX (0) and set to RX only
  AD7021_control_word = ADF7021_RX_REG0;
  Send_AD7021_control2();

  // MODULATION (2)
  ADF7021_REG2 |= (uint32_t) 0b0010;                  // register 2
  ADF7021_REG2 |= m_bTransmitAllowed ? (uint32_t) (m_power & 0x3F) << 13 : 0;  // power level
  ADF7021_REG2 |= (uint32_t) 0b110001         << 7;   // PA
  AD7021_control_word = ADF7021_REG2;
  Send_AD7021_control2();

  // TEST DAC (14)
  AD7021_control_word = 0x0000000E;
  Send_AD7021_control2();

  // AGC (auto, defaults) (9)
  AD7021_control_word = 0x000231E9;
  Send_AD7021_control2();

  // AFC (10)
  AD7021_control_word = ADF7021_REG10;
  Send_AD7021_control2();

  // SYNC WORD DET (11)
  AD7021_control_word = 0x0000003B;
  Send_AD7021_control2();

  // SWD/THRESHOLD (12)
  AD7021_control_word = 0x0000010C;
  Send_AD7021_control2();

  // 3FSK/4FSK DEMOD (13)
  AD7021_control_word = ADF7021_REG13;
  Send_AD7021_control2();

  // TEST MODE (disabled) (15)
  AD7021_control_word = 0x000E000F;
  Send_AD7021_control2();
}
#endif

void CIO::interrupt()
{
  uint8_t bit = 0U;

  if (!m_started)
    return;

  uint8_t clk = READ_MODEM_CLOCK1();

  // this is to prevent activation by spurious interrupts
  // which seem to happen if you send out an control word
  // needs investigation
  // this workaround will fail if only rising or falling edge
  // is used to trigger the interrupt !!!!
  // TODO: figure out why sending the control word seems to issue interrupts
  // possibly this is a design problem of the RF7021 board or too long wires
  // on the breadboard build
  // but normally this will not hurt too much
  if (clk == last_clk)
    return;
  else
    last_clk = clk;

  // we set the TX bit at TXD low, sampling of ADF7021 happens at rising clock
  if (m_tx && clk == 0U) {
    m_txBuffer.get(bit, m_control);
    even = !even;

#if defined(BIDIR_DATA_PIN)
    if (bit)
      WRITE_MODEM_DATA1(HIGH);
    else
      WRITE_MODEM_DATA1(LOW);
#else
    if (bit)
      WRITE_MODEM_CLOCK1(HIGH);
    else
      WRITE_MODEM_CLOCK1(LOW);
#endif

    // wait a brief period before raising SLE
    if (totx_request == true) {
      asm volatile("nop          \n\t"
                   "nop          \n\t"
                   "nop          \n\t"
                   );

      // SLE Pulse, should be moved out of here into class method
      // according to datasheet in 4FSK we have to deliver this before 1/4 tbit == 26uS
      SDIO_LATCHCMD_U1(HIGH);
      asm volatile("nop          \n\t"
                   "nop          \n\t"
                   "nop          \n\t"
                   );
      SDIO_LATCHCMD_U1(LOW);
      SDIO_WRITE_DATABIT(LOW);

      // now do housekeeping
      totx_request = false;
      // first tranmittted bit is always the odd bit
      even = ADF7021_EVEN_BIT;
    }
  }

  // we sample the RX bit at rising TXD clock edge, so TXD must be 1 and we are not in tx mode
  if (!m_tx && clk == 1U && !m_duplex) {
    if (READ_MODEM_DATA1())
      bit = 1U;
    else
      bit = 0U;

    m_rxBuffer.put(bit, m_control);
  }

  if (torx_request && even == ADF7021_EVEN_BIT && m_tx && clk == 0U) {
      // that is absolutely crucial in 4FSK, see datasheet:
      // enable sle after 1/4 tBit == 26uS when sending MSB (even == false) and clock is low
      delay_us(26U);

      // SLE Pulse, should be moved out of here into class method
      SDIO_LATCHCMD_U1(HIGH);
      asm volatile("nop          \n\t"
                   "nop          \n\t"
                   "nop          \n\t"
                   );
      SDIO_LATCHCMD_U1(LOW);
      SDIO_WRITE_DATABIT(LOW);

      // now do housekeeping
      m_tx = false;
      torx_request = false;
      // last tranmittted bit is always the even bit
      // since the current bit is a transitional "don't care" bit, never transmitted
      even = !ADF7021_EVEN_BIT;
  }

  m_watchdog++;
  m_modeTimerCnt++;
  m_int1counter++;

  if (m_scanPauseCnt >= SCAN_PAUSE)
    m_scanPauseCnt = 0U;

  if (m_scanPauseCnt != 0U)
    m_scanPauseCnt++;
}

#if defined(DUPLEX)
void CIO::interrupt2()
{
  uint8_t bit = 0U;

  if (m_duplex) {
    if (READ_MODEM_DATA2())
      bit = 1U;
    else
      bit = 0U;

    m_rxBuffer.put(bit, m_control);
  }

  m_int2counter++;
}
#endif

void CIO::setTX()
{
  // PTT pin on (doing it earlier helps to measure timing impact)
  LED_PTT_RED(HIGH);

  // Send register 0 for TX operation, but do not activate yet.
  // This is done in the interrupt at the correct time
  AD7021_control_word = ADF7021_TX_REG0;
  Send_AD7021_control(false);

#if defined(BIDIR_DATA_PIN)
  SET_PP_OR_FLOAT_MODE(true);  // Data pin output mode
#endif

  totx_request = true;
  while(READ_MODEM_CLOCK1());
}

void CIO::setRX(bool doSle)
{
  // PTT pin off (doing it earlier helps to measure timing impact)
  LED_PTT_RED(LOW);

  // Send register 0 for RX operation, but do not activate yet.
  // This is done in the interrupt at the correct time
  AD7021_control_word = ADF7021_RX_REG0;
  Send_AD7021_control(doSle);

#if defined(BIDIR_DATA_PIN)
  SET_PP_OR_FLOAT_MODE(false);  // Data pin input mode
#endif

  if (!doSle) {
    torx_request = true;
    while(torx_request) { asm volatile ("nop"); }
  }
}

void CIO::setPower(uint8_t power)
{
  m_power = power >> 2;
}

void CIO::setDeviations(uint8_t dmrTXLevel, uint8_t p25TXLevel, uint8_t pocsagTXLevel)
{
  m_dmrDev = uint16_t((ADF7021_DEV_DMR * uint16_t(dmrTXLevel)) / 128U);
  m_p25Dev = uint16_t((ADF7021_DEV_P25 * uint16_t(p25TXLevel)) / 128U);
  m_pocsagDev = uint16_t((ADF7021_DEV_POCSAG * uint16_t(pocsagTXLevel)) / 128U);
}

void CIO::updateCal()
{
  uint32_t ADF7021_REG2;
  float    divider;

  // Check frequency band
  if ((m_frequency_tx >= VHF1_MIN) && (m_frequency_tx < VHF1_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_VHF1;         // VHF1, external VCO
    div2 = 1U;
  } else if ((m_frequency_tx >= VHF2_MIN) && (m_frequency_tx < VHF2_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_VHF2;         // VHF1, external VCO
    div2 = 1U;
  } else if ((m_frequency_tx >= UHF1_MIN)&&(m_frequency_tx < UHF1_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_UHF1;         // UHF1, internal VCO
    div2 = 1U;
  } else if ((m_frequency_tx >= UHF2_MIN)&&(m_frequency_tx < UHF2_MAX)) {
    ADF7021_REG1 = ADF7021_REG1_UHF2;         // UHF2, internal VCO
    div2 = 2U;
  } else {
    ADF7021_REG1 = ADF7021_REG1_UHF1;         // UHF1, internal VCO
    div2 = 1U;
  }

  if (div2 == 1U)
    f_div = 2U;
  else
    f_div = 1U;

  // VCO/OSCILLATOR (REG1)
  AD7021_control_word = ADF7021_REG1;
  Send_AD7021_control();

  ADF7021_REG2  = (uint32_t) 0b10              << 28;  // invert data (and RC alpha = 0.5)

  if (m_modemState == STATE_DMR) {
    ADF7021_REG2 |= (uint32_t) (m_dmrDev / div2)    << 19;  // DMR deviation
    ADF7021_REG2 |= (uint32_t) 0b111                << 4;   // modulation (RC 4FSK)
  } else if (m_modemState == STATE_POCSAG) {
    ADF7021_REG2 |= (uint32_t) (m_pocsagDev / div2) << 19;  // POCSAG deviation
    ADF7021_REG2 |= (uint32_t) 0b000                << 4;   // modulation (2FSK)
  }

  ADF7021_REG2 |= (uint32_t) 0b0010;                   // register 2
  ADF7021_REG2 |= m_bTransmitAllowed ? (uint32_t) m_power << 13 : 0;  // power level
  ADF7021_REG2 |= (uint32_t) 0b110001          << 7;   // PA

  AD7021_control_word = ADF7021_REG2;
  Send_AD7021_control();

  if (div2 == 1U)
    divider = m_frequency_tx / (ADF7021_PFD / 2U);
  else
    divider = m_frequency_tx / ADF7021_PFD;

  m_TX_N_divider = floor(divider);
  divider = (divider - m_TX_N_divider) * 32768;
  m_TX_F_divider = floor(divider + 0.5);

  ADF7021_TX_REG0  = (uint32_t) 0b0000;            // register 0

#if defined(BIDIR_DATA_PIN)
  ADF7021_TX_REG0 |= (uint32_t) 0b01000   << 27;   // mux regulator/transmit
#else
  ADF7021_TX_REG0 |= (uint32_t) 0b01010   << 27;   // mux regulator/uart-spi enabled/transmit
#endif

  ADF7021_TX_REG0 |= (uint32_t) m_TX_N_divider << 19;   // frequency;
  ADF7021_TX_REG0 |= (uint32_t) m_TX_F_divider << 4;    // frequency;

  if (m_tx)
    setTX();
  else
    setRX();
}

#if defined(ENABLE_DEBUG)

uint32_t CIO::RXfreq()
{
  return (uint32_t)((float)(ADF7021_PFD / f_div) * ((float)((32768 * m_RX_N_divider) + m_RX_F_divider) / 32768.0)) + 100000;
}

uint32_t CIO::TXfreq()
{
  return (uint32_t)((float)(ADF7021_PFD / f_div) * ((float)((32768 * m_TX_N_divider) + m_TX_F_divider) / 32768.0));
}

uint16_t CIO::devDMR()
{
  return (uint16_t)((ADF7021_PFD * m_dmrDev) / (f_div * 65536));
}

uint16_t CIO::devP25()
{
  return (uint16_t)((ADF7021_PFD * m_p25Dev) / (f_div * 65536));
}

uint16_t CIO::devPOCSAG()
{
  return (uint16_t)((ADF7021_PFD * m_pocsagDev) / (f_div * 65536));
}

void CIO::printConf()
{
  DEBUG1("MMDVM_HS FW configuration:");
  diffDump_AD7021();
  summaryADF7021();
  DEBUG2I("TX freq (Hz):", TXfreq());
  DEBUG2I("RX freq (Hz):", RXfreq());
  DEBUG2("Power REQESTED:", m_power);
  DEBUG2("Can transmit:", m_bTransmitAllowed);
  DEBUG2("DMR +1 sym dev (Hz):", devDMR());
  DEBUG2("P25 +1 sym dev (Hz):", devP25());
  DEBUG2("POCSAG dev (Hz):", devPOCSAG());
}

#endif

#endif
