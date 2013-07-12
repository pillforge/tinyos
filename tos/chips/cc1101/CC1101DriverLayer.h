/*
 * Copyright (c) 2013, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Author: Janos Sallai
 * Author: Addisu Z. Taddese (Port to CC1101)
 */

#ifndef __CC1101DRIVERLAYER_H__
#define __CC1101DRIVERLAYER_H__


typedef nx_struct cc1101_header_t
{
    nxle_uint8_t length;
} cc1101_header_t;

typedef struct cc1101_metadata_t
{
    uint8_t lqi;
    union
    {
        uint8_t power;
        uint8_t rssi;
    };
} cc1101_metadata_t;

enum cc1101_timing_enums {
    CC1101_SYMBOL_TIME = 16, // 16us
    IDLE_2_RX_ON_TIME = 12 * CC1101_SYMBOL_TIME,
    PD_2_IDLE_TIME = 860, // .86ms
    STROBE_TO_TX_ON_TIME = 12 * CC1101_SYMBOL_TIME,
    // TX SFD delay is computed as follows:
    // a.) STROBE_TO_TX_ON_TIME is required for preamble transmission to
    // start after TX strobe is issued
    // b.) the SFD byte is the 5th byte transmitted (10 symbol periods)
    // c.) there's approximately a 25us delay between the strobe and reading
    // the timer register
    TX_SFD_DELAY = STROBE_TO_TX_ON_TIME + 10 * CC1101_SYMBOL_TIME - 25,
    // TX SFD is captured in hardware
    RX_SFD_DELAY = 0,
};

enum cc1101_reg_access_enums {
    CC1101_CMD_REGISTER_MASK = 0x3f,
    CC1101_CMD_REGISTER_READ = 0x40,
    CC1101_CMD_REGISTER_WRITE = 0x00,
    CC1101_CMD_TXRAM_WRITE = 0x80,
};

typedef union cc1101_status {
    uint16_t value;
    struct {
        unsigned  reserved0:1;
        unsigned  rssi_valid:1;
        unsigned  lock:1;
        unsigned  tx_active:1;

        unsigned  enc_busy:1;
        unsigned  tx_underflow:1;
        unsigned  xosc16m_stable:1;
        unsigned  reserved7:1;
    };
} cc1101_status_t;

typedef union cc1101_iocfg0 {
    uint16_t value;
    struct {
        unsigned  fifop_thr:7;
        unsigned  cca_polarity:1;
        unsigned  sfd_polarity:1;
        unsigned  fifop_polarity:1;
        unsigned  fifo_polarity:1;
        unsigned  bcn_accept:1;
        unsigned  reserved:4; // write as 0
    } f;
} cc1101_iocfg0_t;

// TODO: make sure that we avoid wasting RAM
static const cc1101_iocfg0_t cc1101_iocfg0_default = {.f.fifop_thr = 64, .f.cca_polarity = 0, .f.sfd_polarity = 0, .f.fifop_polarity = 0, .f.fifo_polarity = 0, .f.bcn_accept = 0, .f.reserved = 0};

typedef union cc1101_iocfg1 {
    uint16_t value;
    struct {
        unsigned  ccamux:5;
        unsigned  sfdmux:5;
        unsigned  hssd_src:3;
        unsigned  reserved:3; // write as 0
    } f;
} cc1101_iocfg1_t;

static const cc1101_iocfg1_t cc1101_iocfg1_default = {.value = 0};

typedef union cc1101_fsctrl {
    uint16_t value;
    struct {
        unsigned  freq:10;
        unsigned  lock_status:1;
        unsigned  lock_length:1;
        unsigned  cal_running:1;
        unsigned  cal_done:1;
        unsigned  lock_thr:2;
    } f;
} cc1101_fsctrl_t;

static const cc1101_fsctrl_t cc1101_fsctrl_default = {.f.lock_thr = 1, .f.freq = 357, .f.lock_status = 0, .f.lock_length = 0, .f.cal_running = 0, .f.cal_done = 0};

typedef union cc1101_mdmctrl0 {
    uint16_t value;
    struct {
        unsigned  preamble_length:4;
        unsigned  autoack:1;
        unsigned  autocrc:1;
        unsigned  cca_mode:2;
        unsigned  cca_hyst:3;
        unsigned  adr_decode:1;
        unsigned  pan_coordinator:1;
        unsigned  reserved_frame_mode:1;
        unsigned  reserved:2;
    } f;
} cc1101_mdmctrl0_t;

static const cc1101_mdmctrl0_t cc1101_mdmctrl0_default = {.f.preamble_length = 2, .f.autocrc = 1, .f.cca_mode = 3, .f.cca_hyst = 2, .f.adr_decode = 1};

typedef union cc1101_txctrl {
    uint16_t value;
    struct {
        unsigned  pa_level:5;
        unsigned reserved:1;
        unsigned pa_current:3;
        unsigned txmix_current:2;
        unsigned txmix_caparray:2;
        unsigned tx_turnaround:1;
        unsigned txmixbuf_cur:2;
    } f;
} cc1101_txctrl_t;

static const cc1101_txctrl_t cc1101_txctrl_default = {.f.pa_level = 31, .f.reserved = 1, .f.pa_current = 3, .f.tx_turnaround = 1, .f.txmixbuf_cur = 2};


#ifndef CC1101_DEF_CHANNEL
#define CC1101_DEF_CHANNEL 11
#endif

#ifndef CC1101_DEF_RFPOWER
#define CC1101_DEF_RFPOWER 31
#endif

enum {
    CC1101_TX_PWR_MASK = 0x1f,
    CC1101_CHANNEL_MASK = 0x1f,
};

enum cc1101_config_reg_enums {
    CC1101_SRES = 0x30,
    CC1101_SNOP = 0x3d,
    //---------------------- not modified after copying ----
    CC1101_SXOSCON = 0x01,
    CC1101_STXCAL = 0x02,
    CC1101_SRXON = 0x03,
    CC1101_STXON = 0x04,
    CC1101_STXONCCA = 0x05,
    CC1101_SRFOFF = 0x06,
    CC1101_SXOSCOFF = 0x07,
    CC1101_SFLUSHRX = 0x08,
    CC1101_SFLUSHTX = 0x09,
    CC1101_SACK = 0x0a,
    CC1101_SACKPEND = 0x0b,
    CC1101_SRXDEC = 0x0c,
    CC1101_STXENC = 0x0d,
    CC1101_SAES = 0x0e,
    CC1101_MAIN = 0x10,
    CC1101_MDMCTRL0 = 0x11,
    CC1101_MDMCTRL1 = 0x12,
    CC1101_RSSI = 0x13,
    CC1101_SYNCWORD = 0x14,
    CC1101_TXCTRL = 0x15,
    CC1101_RXCTRL0 = 0x16,
    CC1101_RXCTRL1 = 0x17,
    CC1101_FSCTRL = 0x18,
    CC1101_SECCTRL0 = 0x19,
    CC1101_SECCTRL1 = 0x1a,
    CC1101_BATTMON = 0x1b,
    CC1101_IOCFG0 = 0x1c,
    CC1101_IOCFG1 = 0x1d,
    CC1101_MANFIDL = 0x1e,
    CC1101_MANFIDH = 0x1f,
    CC1101_FSMTC = 0x20,
    CC1101_MANAND = 0x21,
    CC1101_MANOR = 0x22,
    CC1101_AGCCTRL = 0x23,
    CC1101_AGCTST0 = 0x24,
    CC1101_AGCTST1 = 0x25,
    CC1101_AGCTST2 = 0x26,
    CC1101_FSTST0 = 0x27,
    CC1101_FSTST1 = 0x28,
    CC1101_FSTST2 = 0x29,
    CC1101_FSTST3 = 0x2a,
    CC1101_RXBPFTST = 0x2b,
    CC1101_FSMSTATE = 0x2c,
    CC1101_ADCTST = 0x2d,
    CC1101_DACTST = 0x2e,
    CC1101_TOPTST = 0x2f,
    CC1101_TXFIFO = 0x3e,
    CC1101_RXFIFO = 0x3f,
};

// CC1101 Initial Configuration

uint8_t cc1101_init_config[] {

}

#endif // __CC1101DRIVERLAYER_H__
