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
    // Ref CC1101 datasheet Table 34: Overall State Transition Times
    // These values assume Fxosc of 26MHz and no PA ramping
    IDLE_2_RX_ON_TIME = 800, // with calibration
};

// Maximum time to wait for sending and receiving data
#define TX_DATA_TIME ((12+64)*16e6/(CC1101_BAUD_RATE))

enum cc1101_reg_access_enums {
    CC1101_CMD_REGISTER_MASK = 0x3f,
    CC1101_CMD_REGISTER_READ = 0x80,
    CC1101_CMD_REGISTER_WRITE = 0x00,
    CC1101_CMD_BURST_MODE = 0x40,
};

typedef union cc1101_status {
    uint8_t value;
    struct {
        unsigned  fifo_byte_avail:4;
        unsigned  state:3;
        unsigned  chip_rdyn:1;
    };
} cc1101_status_t;

// Enums for the state bits in the cc1101_status union
enum cc1101_status_state { // Comments from datasheet
    CC1101_STATE_IDLE             = 0, // IDLE state
    CC1101_STATE_RX               = 1, // Receive mode
    CC1101_STATE_TX               = 2, // Transmit mode
    CC1101_STATE_FSTXON           = 3, // Fast TX ready
    CC1101_STATE_CALIBRATE        = 4, // Frequency synthesizer calibration is running
    CC1101_STATE_SETTLING         = 5, // PLL is settling
    CC1101_STATE_RXFIFO_OVERFLOW  = 6, // RX FIFO has overflowed. Read out any useful data, then flush the FIFO with SFRX
    CC1101_STATE_TXFIFO_UNDERFLOW = 7  // TX FIFO has underflowed. Acknowledge with SFTX
};

//enum {
//    CC1101_TX_PWR_MASK = 0x1f,
//    CC1101_CHANNEL_MASK = 0x1f,
//};

enum cc1101_config_reg_enums {
    // Configuration Registers
    CC1101_IOCFG2              = 0x00,       // GDO2 output pin configuration
    CC1101_IOCFG1              = 0x01,       // GDO1 output pin configuration
    CC1101_IOCFG0              = 0x02,       // GDO0 output pin configuration
    CC1101_FIFOTHR             = 0x03,       // RX FIFO and TX FIFO thresholds
    CC1101_SYNC1               = 0x04,       // Sync word, high byte
    CC1101_SYNC0               = 0x05,       // Sync word, low byte
    CC1101_PKTLEN              = 0x06,       // Packet length
    CC1101_PKTCTRL1            = 0x07,       // Packet automation control
    CC1101_PKTCTRL0            = 0x08,       // Packet automation control
    CC1101_ADDR                = 0x09,       // Device address
    CC1101_CHANNR              = 0x0A,       // Channel number
    CC1101_FSCTRL1             = 0x0B,       // Frequency synthesizer control
    CC1101_FSCTRL0             = 0x0C,       // Frequency synthesizer control
    CC1101_FREQ2               = 0x0D,       // Frequency control word, high byte
    CC1101_FREQ1               = 0x0E,       // Frequency control word, middle byte
    CC1101_FREQ0               = 0x0F,       // Frequency control word, low byte
    CC1101_MDMCFG4             = 0x10,       // Modem configuration
    CC1101_MDMCFG3             = 0x11,       // Modem configuration
    CC1101_MDMCFG2             = 0x12,       // Modem configuration
    CC1101_MDMCFG1             = 0x13,       // Modem configuration
    CC1101_MDMCFG0             = 0x14,       // Modem configuration
    CC1101_DEVIATN             = 0x15,       // Modem deviation setting
    CC1101_MCSM2               = 0x16,       // Main Radio Cntrl State Machine config
    CC1101_MCSM1               = 0x17,       // Main Radio Cntrl State Machine config
    CC1101_MCSM0               = 0x18,       // Main Radio Cntrl State Machine config
    CC1101_FOCCFG              = 0x19,       // Frequency Offset Compensation config
    CC1101_BSCFG               = 0x1A,       // Bit Synchronization configuration
    CC1101_AGCCTRL2            = 0x1B,       // AGC control
    CC1101_AGCCTRL1            = 0x1C,       // AGC control
    CC1101_AGCCTRL0            = 0x1D,       // AGC control
    CC1101_WOREVT1             = 0x1E,       // High byte Event 0 timeout
    CC1101_WOREVT0             = 0x1F,       // Low byte Event 0 timeout
    CC1101_WORCTRL             = 0x20,       // Wake On Radio control
    CC1101_FREND1              = 0x21,       // Front end RX configuration
    CC1101_FREND0              = 0x22,       // Front end TX configuration
    CC1101_FSCAL3              = 0x23,       // Frequency synthesizer calibration
    CC1101_FSCAL2              = 0x24,       // Frequency synthesizer calibration
    CC1101_FSCAL1              = 0x25,       // Frequency synthesizer calibration
    CC1101_FSCAL0              = 0x26,       // Frequency synthesizer calibration
    CC1101_RCCTRL1             = 0x27,       // RC oscillator configuration
    CC1101_RCCTRL0             = 0x28,       // RC oscillator configuration
    CC1101_FSTEST              = 0x29,       // Frequency synthesizer cal control
    CC1101_PTEST               = 0x2A,       // Production test
    CC1101_AGCTEST             = 0x2B,       // AGC test
    CC1101_TEST2               = 0x2C,       // Various test settings
    CC1101_TEST1               = 0x2D,       // Various test settings
    CC1101_TEST0               = 0x2E,       // Various test settings

    // Strobe commands
    CC1101_SRES                = 0x30,       // Reset chip.
    CC1101_SFSTXON             = 0x31,       // Enable/calibrate freq synthesizer
    CC1101_SXOFF               = 0x32,       // Turn off crystal oscillator.
    CC1101_SCAL                = 0x33,       // Calibrate freq synthesizer & disable
    CC1101_SRX                 = 0x34,       // Enable RX.
    CC1101_STX                 = 0x35,       // Enable TX.
    CC1101_SIDLE               = 0x36,       // Exit RX / TX
    CC1101_SAFC                = 0x37,       // AFC adjustment of freq synthesizer
    CC1101_SWOR                = 0x38,       // Start automatic RX polling sequence
    CC1101_SPWD                = 0x39,       // Enter pwr down mode when CSn goes hi
    CC1101_SFRX                = 0x3A,       // Flush the RX FIFO buffer.
    CC1101_SFTX                = 0x3B,       // Flush the TX FIFO buffer.
    CC1101_SWORRST             = 0x3C,       // Reset real time clock.
    CC1101_SNOP                = 0x3D,       // No operation.

    // Status registers
    CC1101_PARTNUM             = 0x30,       // Part number
    CC1101_VERSION             = 0x31,       // Current version number
    CC1101_FREQEST             = 0x32,       // Frequency offset estimate
    CC1101_LQI                 = 0x33,       // Demodulator estimate for link quality
    CC1101_RSSI                = 0x34,       // Received signal strength indication
    CC1101_MARCSTATE           = 0x35,       // Control state machine state
    CC1101_WORTIME1            = 0x36,       // High byte of WOR timer
    CC1101_WORTIME0            = 0x37,       // Low byte of WOR timer
    CC1101_PKTSTATUS           = 0x38,       // Current GDOx status and packet status
    CC1101_VCO_VC_DAC          = 0x39,       // Current setting from PLL cal module
    CC1101_TXBYTES             = 0x3A,       // Underflow and # of bytes in TXFIFO
    CC1101_RXBYTES             = 0x3B,       // Overflow and # of bytes in RXFIFO
    CC1101_NUM_RXBYTES         = 0x7F,       // Mask "# of bytes" field in _RXBYTES

    // Other memory locations
    CC1101_PATABLE             = 0x3E,
    CC1101_TXFIFO              = 0x3F,
    CC1101_RXFIFO              = 0x3F,
};

// TODO: The following code was adapted from the CC1100.h file of the Blaze radio stack. Figure out how to incorporate 
// license of that file into this.
//
/*
 * Copyright (c) 2005-2006 Rincon Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Rincon Research Corporation nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * RINCON RESEARCH OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE
 */
 
/**
 * @author Jared Hill
 * @author David Moss
 * @author Roland Hendel
 */
/** 
 * This helps calculate new FREQx register settings at runtime 
 * The frequency is in Hz
 */
#define CC1101_CRYSTAL_HZ 26000000

#define CC1101_315_MHZ 0
#define CC1101_433_MHZ 1
#define CC1101_868_MHZ 2
#define CC1101_915_MHZ 3

/**
 * You can change the matching network at compile time
 */
#ifndef CC1101_MATCHING_NETWORK
#define CC1101_MATCHING_NETWORK CC1101_433_MHZ
#endif

#define CC1101_1_2K  1
#define CC1101_2_4K  2
#define CC1101_10K   3
#define CC1101_26K   4
#define CC1101_38_4K 5
#define CC1101_76_8K 6
#define CC1101_100K  7
#define CC1101_150K  8
#define CC1101_250K  9
#define CC1101_500K  10

#ifndef CC1101_BAUD
//#define CC1101_BAUD CC1101_2_4K
//#define CC1101_BAUD CC1101_10K
//#define CC1101_BAUD CC1101_100K
#define CC1101_BAUD CC1101_250K
//#define CC1101_BAUD CC1101_500K
#endif

#if (CC1101_BAUD == CC1101_1_2K)
  #define CC1101_BAUD_RATE 1200
#elif (CC1101_BAUD == CC1101_2_4K)
  #define CC1101_BAUD_RATE 2400
#elif (CC1101_BAUD == CC1101_10K) 
  #define CC1101_BAUD_RATE 10000
#elif (CC1101_BAUD == CC1101_26K) 
  #define CC1101_BAUD_RATE 26000
#elif (CC1101_BAUD == CC1101_38_4K) 
  #define CC1101_BAUD_RATE 38400
#elif (CC1101_BAUD == CC1101_76_8K) 
  #define CC1101_BAUD_RATE 76800
#elif (CC1101_BAUD == CC1101_100K) 
  #define CC1101_BAUD_RATE 100000
#elif (CC1101_BAUD == CC1101_150K) 
  #define CC1101_BAUD_RATE 150000
#elif (CC1101_BAUD == CC1101_250K) 
  #define CC1101_BAUD_RATE 250000
#elif (CC1101_BAUD == CC1101_500K) 
  #define CC1101_BAUD_RATE 500000
#endif

/**
 * All default channels and FREQx registers obtained from SmartRF studio. We
 * are not trying to define channel frequencies to match up with any sort of
 * specification; instead, we want flexibility.  If you want to align with 
 * specs, then go for it.
 *
 * Note you can setup the CC1101 to match your antenna characteristics.
 * Maybe your antenna is tuned to +/- 5 MHz with a center frequency of 315 MHz.
 * You want your center frequency to be 314.996 MHz, and your lower edge to be 
 * 310 MHz and your upper edge to be 320 MHz. 
 *
 *   Lower Channel Calculation:
 *      CC1101_CHANNEL_MIN = [(310000 desired kHz) - (CC1101_LOWEST_FREQ)]
 *                           ---------------------------------------------
 *                                     199 kHz channel spacing
 *
 *         Where CC1101_LOWEST_FREQ is defined for each band and 199 kHz is 
 *         approximately the channel spacing, CC1101_CHANNEL_WIDTH
 *
 *      CC1101_CHANNEL_MIN ~= 45
 *
 *  
 *   Upper Channel Calculation:
 *      CC1101_CHANNEL_MAX = [(320000 desired kHz) - (CC1101_LOWEST_FREQ)]
 *                           ---------------------------------------------
 *                                     199 kHz channel spacing
 * 
 *      CC1101_CHANNEL_MAX ~= 95
 * 
 * Incidentally, (95+45)/2 = 70, which is our default center channel.
 * 
 * When you apply the MAX and MIN values, the radio stack will automatically 
 * make sure you're within bounds when you set the frequency or channel during
 * runtime.
 *
 * We defined the minimum and maximum channels for the various bands below
 * so they generally stay within the limits of the CC1101 radio defined in the
 * datasheet.
 */
 
 

#if (CC1101_MATCHING_NETWORK == CC1101_433_MHZ)
/***************** 433 MHz Matching Network ****************/

// Default channel is at 433.191833 MHz
#ifndef CC1101_DEFAULT_CHANNEL
#define CC1101_DEFAULT_CHANNEL 161
#endif

#ifndef CC1101_CHANNEL_MIN
#define CC1101_CHANNEL_MIN 0
#endif

#ifndef CC1101_CHANNEL_MAX
#define CC1101_CHANNEL_MAX 255
#endif

enum {  
  CC1101_LOWEST_FREQ = 400998, 
  CC1101_DEFAULT_FREQ2 = 0x0F,
  CC1101_DEFAULT_FREQ1 = 0x6C,
  CC1101_DEFAULT_FREQ0 = 0x4E,
};  

/** 
 * These values calculated using TI smart RF studio
 */
enum{
  CC1101_PA_PLUS_10 = 0xC0,
  CC1101_PA_PLUS_5 = 0x84,
  CC1101_PA_PLUS_0 = 0x60,
  CC1101_PA_MINUS_10 = 0x34,	
  CC1101_PA_MINUS_15 = 0x1d,
  CC1101_PA_MINUS_20 = 0x0e,
  CC1101_PA_MINUS_30 = 0x12,
};

#ifndef CC1101_PA
//#define CC1101_PA CC1101_PA_PLUS_10
#define CC1101_PA CC1101_PA_PLUS_0
//#define CC1101_PA CC1101_PA_MINUS_30
#endif
// End CC1101_433_MHz
#elif (CC1101_MATCHING_NETWORK == CC1101_868_MHZ)
/***************** 868 MHz Matching Network ****************/

#warning "Using 868 MHz radio"
// Default channel is at 868.192749 MHz
#ifndef CC1101_DEFAULT_CHANNEL
#define CC1101_DEFAULT_CHANNEL 141
#endif

#ifndef CC1101_CHANNEL_MIN
#define CC1101_CHANNEL_MIN 0
#endif

#ifndef CC1101_CHANNEL_MAX
#define CC1101_CHANNEL_MAX 255
#endif

enum {
  CC1101_LOWEST_FREQ = 839998,
  CC1101_DEFAULT_FREQ2 = 0x20,
  CC1101_DEFAULT_FREQ1 = 0x4E,
  CC1101_DEFAULT_FREQ0 = 0xC4,
};

/**
 * These values calculated using TI smart RF studio
 */
enum{
  CC1101_PA_PLUS_10 = 0xC3,
  CC1101_PA_PLUS_5 = 0x85,
  CC1101_PA_PLUS_0 = 0x8E,
  CC1101_PA_MINUS_5 = 0x57,
  CC1101_PA_MINUS_10 = 0x34,
};

#ifndef CC1101_PA
#define CC1101_PA CC1101_PA_PLUS_0
#endif

#endif // End CC1101_868_MHz
/**
 * These are used for calculating channels at runtime
 */
#define CC1101_CHANNEL_WIDTH 199 // kHz : Do not edit

#if (CC1101_BAUD == CC1101_1_2K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  CC1101_CONFIG_FIFOTHR = 0x0F,
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x06,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0xF5,
  CC1101_CONFIG_MDMCFG3 = 0x83,
  CC1101_CONFIG_MDMCFG2 = 0x03,  // 0x03 = no manchester
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x15,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x16,
  CC1101_CONFIG_BSCFG = 0x6C,
  CC1101_CONFIG_AGCTRL2 = 0x03,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0x91, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0x56,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xE9,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x81,
  CC1101_CONFIG_TEST1 = 0x35,
  CC1101_CONFIG_TEST0 = 0x0B,
   
}; // end 1.2

#elif (CC1101_BAUD == CC1101_2_4K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  CC1101_CONFIG_FIFOTHR = 0x0F,
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x06,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0xF5,
  CC1101_CONFIG_MDMCFG3 = 0x83,
  CC1101_CONFIG_MDMCFG2 = 0x03,  // 0x03 = no manchester
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x15,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x16,
  CC1101_CONFIG_BSCFG = 0x6C,
  CC1101_CONFIG_AGCTRL2 = 0x03,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0x91, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0x56,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xE9,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x81,
  CC1101_CONFIG_TEST1 = 0x35,
  CC1101_CONFIG_TEST0 = 0x0B,
   
}; // end 2.4

#elif (CC1101_BAUD == CC1101_10K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts at the end of a received packet */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  /** FIFO Threshold is maxed so we don't try downloading incomplete pkts */
  CC1101_CONFIG_FIFOTHR = 0x0F,
  
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x06,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0xC8,
  CC1101_CONFIG_MDMCFG3 = 0x93,
  CC1101_CONFIG_MDMCFG2 = 0x1B,  // GFSK. 0x13 = no manchester / 0x1B = with manch.
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x34,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x16,
  CC1101_CONFIG_BSCFG = 0x6C,
  CC1101_CONFIG_AGCTRL2 = 0x43,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0x91, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0x56,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xE9,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x81,
  CC1101_CONFIG_TEST1 = 0x35,
  CC1101_CONFIG_TEST0 = 0x0B,
   
}; // end 10k baud

#elif (CC1101_BAUD == CC1101_26K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts at the end of a received packet */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  /** FIFO Threshold is maxed so we don't try downloading incomplete pkts */
  CC1101_CONFIG_FIFOTHR = 0x0F,
  
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x06,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0xFA,
  CC1101_CONFIG_MDMCFG3 = 0x06,
  CC1101_CONFIG_MDMCFG2 = 0x73,
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x00,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x16,
  CC1101_CONFIG_BSCFG = 0x6C,
  CC1101_CONFIG_AGCTRL2 = 0x03,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0x91, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0x56,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xE9,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x81,
  CC1101_CONFIG_TEST1 = 0x35,
  CC1101_CONFIG_TEST0 = 0x09,
   
}; // end 26

#elif (CC1101_BAUD == CC1101_38_4K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  CC1101_CONFIG_FIFOTHR = 0x0F,
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x06,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0xCA,
  CC1101_CONFIG_MDMCFG3 = 0x83,
  CC1101_CONFIG_MDMCFG2 = 0x0B,  // 0x03 = no manchester / 0x0B = with manch.
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x34,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x16,
  CC1101_CONFIG_BSCFG = 0x6C,
  CC1101_CONFIG_AGCTRL2 = 0x43,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0x91, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0x56,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xE9,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x81,
  CC1101_CONFIG_TEST1 = 0x35,
  CC1101_CONFIG_TEST0 = 0x0B,
   
}; // end 38.4

#elif (CC1101_BAUD == CC1101_76_8K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  CC1101_CONFIG_FIFOTHR = 0x0F,
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x06,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0x7B,
  CC1101_CONFIG_MDMCFG3 = 0x83,
  CC1101_CONFIG_MDMCFG2 = 0x0B,  // 0x03 = no manchester / 0x0B = with manch.
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x42,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x1D,
  CC1101_CONFIG_BSCFG = 0x1C,
  CC1101_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0xB2, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0xB6,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xEA,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x81,
  CC1101_CONFIG_TEST1 = 0x35,
  CC1101_CONFIG_TEST0 = 0x0B,
   
}; // end 76.8

#elif (CC1101_BAUD == CC1101_100K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  CC1101_CONFIG_FIFOTHR = 0x0F,
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x06,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0x5B,
  CC1101_CONFIG_MDMCFG3 = 0xF8,
  CC1101_CONFIG_MDMCFG2 = 0x1B,  // 0x13 = no manchester
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x47,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x1D,
  CC1101_CONFIG_BSCFG = 0x1C,
  CC1101_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0xB2, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0xB6,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xEA,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x81,
  CC1101_CONFIG_TEST1 = 0x35,
  CC1101_CONFIG_TEST0 = 0x0B,
   
}; // end 100

#elif (CC1101_BAUD == CC1101_150K)

enum CC1101_config_reg_state_enums {
    CC1101_CONFIG_IOCFG2    = 0x29,     // 0x00, GDO2 Output Pin Config
    CC1101_CONFIG_IOCFG1    = 0x2E,     // 0x01, GDO1 Output Pin Config
    CC1101_CONFIG_IOCFG0    = 0x06,     // 0x02, GDO0 Output Pin Config
    CC1101_CONFIG_FIFOTHR   = 0x0F,     // 0x03, RX FIFO and TX FIFO Thresholds
    CC1101_CONFIG_SYNC1     = 0xD3,		// 0x04, Sync Word, High Byte
    CC1101_CONFIG_SYNC0     = 0x91,		// 0x05, Sync Word, Low Byte
    CC1101_CONFIG_PKTLEN    = 0x3D,     // 0x06, Packet Length
    CC1101_CONFIG_PKTCTRL1  = 0x24,     // 0x07, Packet Automation Control
    CC1101_CONFIG_PKTCTRL0  = 0x45,     // 0x08, Packet Automation Control
    CC1101_CONFIG_ADDR      = 0x00,		// 0x09, Device Address
    CC1101_CONFIG_CHANNR    = CC1101_DEFAULT_CHANNEL,	// 0x0A, Channel Number
    CC1101_CONFIG_FSCTRL1   = 0x0E,     // 0x0B, Frequency Synthesizer Control
    CC1101_CONFIG_FSCTRL0   = 0x00,     // 0x0C, Frequency Synthesizer Control
    CC1101_CONFIG_FREQ2     = 0x0F,     // 0x0D, Frequency Control Word, High
    CC1101_CONFIG_FREQ1     = 0x6C,     // 0x0E, Frequency Control Word, Middle
    CC1101_CONFIG_FREQ0     = 0x4E,     // 0x0F, Frequency Control Word, Low
    CC1101_CONFIG_MDMCFG4   = 0x0C,     // 0x10, Modem Config
    CC1101_CONFIG_MDMCFG3   = 0x3B,     // 0x11, Modem Config
    CC1101_CONFIG_MDMCFG2   = 0x73,     // 0x12, Modem Config
    CC1101_CONFIG_MDMCFG1   = 0x42,     // 0x13, Modem Config
    CC1101_CONFIG_MDMCFG0   = 0xF8,     // 0x14, Modem Config
    CC1101_CONFIG_DEVIATN   = 0x00,     // 0x15, Modem Deviation Setting
    CC1101_CONFIG_MCSM2     = 0x07,     // 0x16, Main Radio Control State Machine Config
    CC1101_CONFIG_MCSM1     = 0x3F,     // 0x17, Main Radio Control State Machine Config
    CC1101_CONFIG_MCSM0     = 0x18,     // 0x18, Main Radio Control State Machine Config
    CC1101_CONFIG_FOCCFG    = 0x1D,     // 0x19, Frequency Offset Compensation Config
    CC1101_CONFIG_BSCFG     = 0x1C,     // 0x1A, Bit Sync Config
    CC1101_CONFIG_AGCTRL2   = 0xC7,     // 0x1B, AGC Control
    CC1101_CONFIG_AGCTRL1   = 0x00,     // 0x1C, AGC Control
    CC1101_CONFIG_AGCTRL0   = 0xB0,     // 0x1D, AGC Control
    CC1101_CONFIG_WOREVT1   = 0x87,     // 0x1E, High Byte Event0 Timeout
    CC1101_CONFIG_WOREVT0   = 0x6B,     // 0x1F, Low Byte Event0 Timeout
    CC1101_CONFIG_WORCTRL   = 0xF8,     // 0x20, Wake On Radio Control
    CC1101_CONFIG_FREND1    = 0xB6,     // 0x21, Front End RX Config
    CC1101_CONFIG_FREND0    = 0x10,     // 0x22, Front End TX Config
    CC1101_CONFIG_FSCAL3    = 0xEA,     // 0x23, Frequency Synthesizer Calibration
    CC1101_CONFIG_FSCAL2    = 0x2A,     // 0x24, Frequency Synthesizer Calibration
    CC1101_CONFIG_FSCAL1    = 0x00,     // 0x25, Frequency Synthesizer Calibration
    CC1101_CONFIG_FSCAL0    = 0x1F,     // 0x26, Frequency Synthesizer Calibration
    CC1101_CONFIG_RCCTRL1   = 0x41,     // 0x27, RC Oscillator Config
    CC1101_CONFIG_RCCTRL0   = 0x00,     // 0x28, RC Oscillator Config
    CC1101_CONFIG_FSTEST    = 0x59,     // 0x29, Frequency Synthesizer Calibration Control
    CC1101_CONFIG_PTEST     = 0x7F,     // 0x2A, Production Test
    CC1101_CONFIG_AGCTST    = 0x3F,     // 0x2B, AGC Test
    CC1101_CONFIG_TEST2     = 0x88,     // 0x2D, Varoius Test Setting
    CC1101_CONFIG_TEST1     = 0x31,     // 0x2E, Varoius Test Setting
    CC1101_CONFIG_TEST0     = 0x0B      // 0x2F, Varoius Test Setting
}; // end 150

#elif (CC1101_BAUD == CC1101_250K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  CC1101_CONFIG_FIFOTHR = 0x0F,
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x0C,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0x2D,
  CC1101_CONFIG_MDMCFG3 = 0x3B,
  CC1101_CONFIG_MDMCFG2 = 0x0B,  // 0x03 = no manchester
  CC1101_CONFIG_MDMCFG1 = 0x22,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x62,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x1D,
  CC1101_CONFIG_BSCFG = 0x1C,
  CC1101_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0xB0, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0xB6,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xEA,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x88,
  CC1101_CONFIG_TEST1 = 0x31,
  CC1101_CONFIG_TEST0 = 0x0B,
   
}; // end 250

#elif (CC1101_BAUD == CC1101_500K)

enum CC1101_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1101_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1101_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1101_CONFIG_IOCFG0 = 0x06, 
  
  CC1101_CONFIG_FIFOTHR = 0x0F,
  CC1101_CONFIG_SYNC1 = 0xD3,
  CC1101_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1101_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1101_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1101_CONFIG_PKTCTRL0 = 0x45,
  
  CC1101_CONFIG_ADDR = 0x00,
  
  CC1101_CONFIG_CHANNR = CC1101_DEFAULT_CHANNEL,
  
  CC1101_CONFIG_FSCTRL1 = 0x0E,
  CC1101_CONFIG_FSCTRL0 = 0x00,
  
  CC1101_CONFIG_FREQ2 = CC1101_DEFAULT_FREQ2,
  CC1101_CONFIG_FREQ1 = CC1101_DEFAULT_FREQ1,
  CC1101_CONFIG_FREQ0 = CC1101_DEFAULT_FREQ0,
  
  CC1101_CONFIG_MDMCFG4 = 0x0E,
  CC1101_CONFIG_MDMCFG3 = 0x3B,
  CC1101_CONFIG_MDMCFG2 = 0x73,
  CC1101_CONFIG_MDMCFG1 = 0x42,
  CC1101_CONFIG_MDMCFG0 = 0xF8,
  CC1101_CONFIG_DEVIATN = 0x00,
  CC1101_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1101_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1101_CONFIG_MCSM0 = 0x18,
  
  CC1101_CONFIG_FOCCFG = 0x1D,
  CC1101_CONFIG_BSCFG = 0x1C,
  CC1101_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1101_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1101_CONFIG_AGCTRL0 = 0xB0, 
  
  CC1101_CONFIG_WOREVT1 = 0x87,
  CC1101_CONFIG_WOREVT0 = 0x6B,
  CC1101_CONFIG_WORCTRL = 0xF8,
  CC1101_CONFIG_FREND1 = 0xB6,
  CC1101_CONFIG_FREND0 = 0x10,
  CC1101_CONFIG_FSCAL3 = 0xEA,
  CC1101_CONFIG_FSCAL2 = 0x2A,
  CC1101_CONFIG_FSCAL1 = 0x00,
  CC1101_CONFIG_FSCAL0 = 0x1F,
  
  CC1101_CONFIG_RCCTRL1 = 0x41,
  CC1101_CONFIG_RCCTRL0 = 0x00,
  CC1101_CONFIG_FSTEST = 0x59,
  CC1101_CONFIG_PTEST = 0x7F,
  CC1101_CONFIG_AGCTST = 0x3F,
  CC1101_CONFIG_TEST2 = 0x88,
  CC1101_CONFIG_TEST1 = 0x31,
  CC1101_CONFIG_TEST0 = 0x0B,
   
};
#else
#error "Baud rate not defined"
#endif

#endif // __CC1101DRIVERLAYER_H__
