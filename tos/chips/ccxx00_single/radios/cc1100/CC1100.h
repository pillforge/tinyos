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
 * 10 kBaud Manchester
 */
 
/**
 * All frequency settings assume a 26 MHz crystal.
 * If you have a 27 MHz crystal, you'll need to fix the defined FREQ registers
 * 
 * @author Jared Hill
 * @author David Moss
 * @author Roland Hendel
 */
 
#ifndef CC1100_H
#define CC1100_H

#include "Blaze.h"

enum {
  CC1100_RADIO_ID = unique( UQ_BLAZE_RADIO ),
};



/** 
 * This helps calculate new FREQx register settings at runtime 
 * The frequency is in Hz
 */
#define CC1100_CRYSTAL_HZ 26000000

#define CC1100_315_MHZ 0
#define CC1100_433_MHZ 1
#define CC1100_868_MHZ 2
#define CC1100_915_MHZ 3

/**
 * You can change the matching network at compile time
 */
#ifndef CC1100_MATCHING_NETWORK
#define CC1100_MATCHING_NETWORK CC1100_433_MHZ
#endif

#define CC1100_1_2K  1
#define CC1100_2_4K  2
#define CC1100_10K   3
#define CC1100_26K   4
#define CC1100_38_4K 5
#define CC1100_76_8K 6
#define CC1100_100K  7
#define CC1100_150K  8
#define CC1100_250K  9
#define CC1100_500K  10

#ifndef CC1100_BAUD
#define CC1100_BAUD CC1100_250K
#endif


/**
 * All default channels and FREQx registers obtained from SmartRF studio. We
 * are not trying to define channel frequencies to match up with any sort of
 * specification; instead, we want flexibility.  If you want to align with 
 * specs, then go for it.
 *
 * Note you can setup the CC1100 to match your antenna characteristics.
 * Maybe your antenna is tuned to +/- 5 MHz with a center frequency of 315 MHz.
 * You want your center frequency to be 314.996 MHz, and your lower edge to be 
 * 310 MHz and your upper edge to be 320 MHz. 
 *
 *   Lower Channel Calculation:
 *      CC1100_CHANNEL_MIN = [(310000 desired kHz) - (CC1100_LOWEST_FREQ)]
 *                           ---------------------------------------------
 *                                     199 kHz channel spacing
 *
 *         Where CC1100_LOWEST_FREQ is defined for each band and 199 kHz is 
 *         approximately the channel spacing, CC1100_CHANNEL_WIDTH
 *
 *      CC1100_CHANNEL_MIN ~= 45
 *
 *  
 *   Upper Channel Calculation:
 *      CC1100_CHANNEL_MAX = [(320000 desired kHz) - (CC1100_LOWEST_FREQ)]
 *                           ---------------------------------------------
 *                                     199 kHz channel spacing
 * 
 *      CC1100_CHANNEL_MAX ~= 95
 * 
 * Incidentally, (95+45)/2 = 70, which is our default center channel.
 * 
 * When you apply the MAX and MIN values, the radio stack will automatically 
 * make sure you're within bounds when you set the frequency or channel during
 * runtime.
 *
 * We defined the minimum and maximum channels for the various bands below
 * so they generally stay within the limits of the CC1100 radio defined in the
 * datasheet.
 */
 
 

/***************** 433 MHz Matching Network ****************/

// Default channel is at 433.191833 MHz
#ifndef CC1100_DEFAULT_CHANNEL
#define CC1100_DEFAULT_CHANNEL 161
#endif

#ifndef CC1100_CHANNEL_MIN
#define CC1100_CHANNEL_MIN 0
#endif

#ifndef CC1100_CHANNEL_MAX
#define CC1100_CHANNEL_MAX 255
#endif

enum {  
  CC1100_LOWEST_FREQ = 400998, 
  CC1100_DEFAULT_FREQ2 = 0x0F,
  CC1100_DEFAULT_FREQ1 = 0x6C,
  CC1100_DEFAULT_FREQ0 = 0x4E,
};  

/** 
 * These values calculated using TI smart RF studio
 */
enum{
  CC1100_PA_PLUS_10 = 0xC0,
  CC1100_PA_PLUS_5 = 0x85,
  CC1100_PA_PLUS_0 = 0x60,
  CC1100_PA_MINUS_5 = 0x57,
  CC1100_PA_MINUS_10 = 0x26,	
};

#ifndef CC1100_PA
#define CC1100_PA CC1100_PA_PLUS_0
#endif


/**
 * These are used for calculating channels at runtime
 */
#define CC1100_CHANNEL_WIDTH 199 // kHz : Do not edit

/** SLOW THINGS DOWN FOR THIS DATA RATE */
#define TRANSMITTER_QUALITY_THRESHOLD 200
#define BLAZE_MIN_INITIAL_BACKOFF 5000
#define BLAZE_MIN_BACKOFF 500
#define BLAZE_BACKOFF_PERIOD 100

#if (CC1100_BAUD == CC1100_1_2K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  CC1100_CONFIG_FIFOTHR = 0x0F,
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x06,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0xF5,
  CC1100_CONFIG_MDMCFG3 = 0x83,
  CC1100_CONFIG_MDMCFG2 = 0x03,  // 0x03 = no manchester
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x15,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x16,
  CC1100_CONFIG_BSCFG = 0x6C,
  CC1100_CONFIG_AGCTRL2 = 0x03,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0x91, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0x56,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xE9,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x81,
  CC1100_CONFIG_TEST1 = 0x35,
  CC1100_CONFIG_TEST0 = 0x0B,
   
}; // end 1.2

#elif (CC1100_BAUD == CC1100_2_4K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  CC1100_CONFIG_FIFOTHR = 0x0F,
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x06,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0xF5,
  CC1100_CONFIG_MDMCFG3 = 0x83,
  CC1100_CONFIG_MDMCFG2 = 0x03,  // 0x03 = no manchester
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x15,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x16,
  CC1100_CONFIG_BSCFG = 0x6C,
  CC1100_CONFIG_AGCTRL2 = 0x03,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0x91, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0x56,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xE9,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x81,
  CC1100_CONFIG_TEST1 = 0x35,
  CC1100_CONFIG_TEST0 = 0x0B,
   
}; // end 2.4

#elif (CC1100_BAUD == CC1100_10K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts at the end of a received packet */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  /** FIFO Threshold is maxed so we don't try downloading incomplete pkts */
  CC1100_CONFIG_FIFOTHR = 0x0F,
  
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x06,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0xC8,
  CC1100_CONFIG_MDMCFG3 = 0x93,
  CC1100_CONFIG_MDMCFG2 = 0x1B,  // GFSK. 0x13 = no manchester / 0x1B = with manch.
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x34,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x16,
  CC1100_CONFIG_BSCFG = 0x6C,
  CC1100_CONFIG_AGCTRL2 = 0x43,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0x91, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0x56,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xE9,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x81,
  CC1100_CONFIG_TEST1 = 0x35,
  CC1100_CONFIG_TEST0 = 0x0B,
   
}; // end 10k baud

#elif (CC1100_BAUD == CC1100_26K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts at the end of a received packet */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  /** FIFO Threshold is maxed so we don't try downloading incomplete pkts */
  CC1100_CONFIG_FIFOTHR = 0x0F,
  
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x06,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0xFA,
  CC1100_CONFIG_MDMCFG3 = 0x06,
  CC1100_CONFIG_MDMCFG2 = 0x73,
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x00,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x16,
  CC1100_CONFIG_BSCFG = 0x6C,
  CC1100_CONFIG_AGCTRL2 = 0x03,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0x91, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0x56,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xE9,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x81,
  CC1100_CONFIG_TEST1 = 0x35,
  CC1100_CONFIG_TEST0 = 0x09,
   
}; // end 26

#elif (CC1100_BAUD == CC1100_38_4K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  CC1100_CONFIG_FIFOTHR = 0x0F,
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x06,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0xCA,
  CC1100_CONFIG_MDMCFG3 = 0x83,
  CC1100_CONFIG_MDMCFG2 = 0x0B,  // 0x03 = no manchester / 0x0B = with manch.
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x34,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x16,
  CC1100_CONFIG_BSCFG = 0x6C,
  CC1100_CONFIG_AGCTRL2 = 0x43,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x40,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0x91, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0x56,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xE9,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x81,
  CC1100_CONFIG_TEST1 = 0x35,
  CC1100_CONFIG_TEST0 = 0x0B,
   
}; // end 38.4

#elif (CC1100_BAUD == CC1100_76_8K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  CC1100_CONFIG_FIFOTHR = 0x0F,
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x06,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0x7B,
  CC1100_CONFIG_MDMCFG3 = 0x83,
  CC1100_CONFIG_MDMCFG2 = 0x0B,  // 0x03 = no manchester / 0x0B = with manch.
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x42,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x1D,
  CC1100_CONFIG_BSCFG = 0x1C,
  CC1100_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0xB2, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0xB6,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xEA,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x81,
  CC1100_CONFIG_TEST1 = 0x35,
  CC1100_CONFIG_TEST0 = 0x0B,
   
}; // end 76.8

#elif (CC1100_BAUD == CC1100_100K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  CC1100_CONFIG_FIFOTHR = 0x0F,
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x06,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0x5B,
  CC1100_CONFIG_MDMCFG3 = 0xF8,
  CC1100_CONFIG_MDMCFG2 = 0x1B,  // 0x13 = no manchester
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x47,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x1D,
  CC1100_CONFIG_BSCFG = 0x1C,
  CC1100_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0xB2, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0xB6,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xEA,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x81,
  CC1100_CONFIG_TEST1 = 0x35,
  CC1100_CONFIG_TEST0 = 0x0B,
   
}; // end 100

#elif (CC1100_BAUD == CC1100_150K)

enum CC1100_config_reg_state_enums {
    CC1100_CONFIG_IOCFG2    = 0x29,     // 0x00, GDO2 Output Pin Config
    CC1100_CONFIG_IOCFG1    = 0x2E,     // 0x01, GDO1 Output Pin Config
    CC1100_CONFIG_IOCFG0    = 0x01,     // 0x02, GDO0 Output Pin Config
    CC1100_CONFIG_FIFOTHR   = 0x0F,     // 0x03, RX FIFO and TX FIFO Thresholds
    CC1100_CONFIG_SYNC1     = 0xD3,		// 0x04, Sync Word, High Byte
    CC1100_CONFIG_SYNC0     = 0x91,		// 0x05, Sync Word, Low Byte
    CC1100_CONFIG_PKTLEN    = 0x3D,     // 0x06, Packet Length
    CC1100_CONFIG_PKTCTRL1  = 0x24,     // 0x07, Packet Automation Control
    CC1100_CONFIG_PKTCTRL0  = 0x45,     // 0x08, Packet Automation Control
    CC1100_CONFIG_ADDR      = 0x00,		// 0x09, Device Address
    CC1100_CONFIG_CHANNR    = CC1100_DEFAULT_CHANNEL,	// 0x0A, Channel Number
    CC1100_CONFIG_FSCTRL1   = 0x0E,     // 0x0B, Frequency Synthesizer Control
    CC1100_CONFIG_FSCTRL0   = 0x00,     // 0x0C, Frequency Synthesizer Control
    CC1100_CONFIG_FREQ2     = 0x0F,     // 0x0D, Frequency Control Word, High
    CC1100_CONFIG_FREQ1     = 0x6C,     // 0x0E, Frequency Control Word, Middle
    CC1100_CONFIG_FREQ0     = 0x4E,     // 0x0F, Frequency Control Word, Low
    CC1100_CONFIG_MDMCFG4   = 0x0C,     // 0x10, Modem Config
    CC1100_CONFIG_MDMCFG3   = 0x3B,     // 0x11, Modem Config
    CC1100_CONFIG_MDMCFG2   = 0x73,     // 0x12, Modem Config
    CC1100_CONFIG_MDMCFG1   = 0x42,     // 0x13, Modem Config
    CC1100_CONFIG_MDMCFG0   = 0xF8,     // 0x14, Modem Config
    CC1100_CONFIG_DEVIATN   = 0x00,     // 0x15, Modem Deviation Setting
    CC1100_CONFIG_MCSM2     = 0x07,     // 0x16, Main Radio Control State Machine Config
    CC1100_CONFIG_MCSM1     = 0x3F,     // 0x17, Main Radio Control State Machine Config
    CC1100_CONFIG_MCSM0     = 0x18,     // 0x18, Main Radio Control State Machine Config
    CC1100_CONFIG_FOCCFG    = 0x1D,     // 0x19, Frequency Offset Compensation Config
    CC1100_CONFIG_BSCFG     = 0x1C,     // 0x1A, Bit Sync Config
    CC1100_CONFIG_AGCTRL2   = 0xC7,     // 0x1B, AGC Control
    CC1100_CONFIG_AGCTRL1   = 0x00,     // 0x1C, AGC Control
    CC1100_CONFIG_AGCTRL0   = 0xB0,     // 0x1D, AGC Control
    CC1100_CONFIG_WOREVT1   = 0x87,     // 0x1E, High Byte Event0 Timeout
    CC1100_CONFIG_WOREVT0   = 0x6B,     // 0x1F, Low Byte Event0 Timeout
    CC1100_CONFIG_WORCTRL   = 0xF8,     // 0x20, Wake On Radio Control
    CC1100_CONFIG_FREND1    = 0xB6,     // 0x21, Front End RX Config
    CC1100_CONFIG_FREND0    = 0x10,     // 0x22, Front End TX Config
    CC1100_CONFIG_FSCAL3    = 0xEA,     // 0x23, Frequency Synthesizer Calibration
    CC1100_CONFIG_FSCAL2    = 0x2A,     // 0x24, Frequency Synthesizer Calibration
    CC1100_CONFIG_FSCAL1    = 0x00,     // 0x25, Frequency Synthesizer Calibration
    CC1100_CONFIG_FSCAL0    = 0x1F,     // 0x26, Frequency Synthesizer Calibration
    CC1100_CONFIG_RCCTRL1   = 0x41,     // 0x27, RC Oscillator Config
    CC1100_CONFIG_RCCTRL0   = 0x00,     // 0x28, RC Oscillator Config
    CC1100_CONFIG_FSTEST    = 0x59,     // 0x29, Frequency Synthesizer Calibration Control
    CC1100_CONFIG_PTEST     = 0x7F,     // 0x2A, Production Test
    CC1100_CONFIG_AGCTST    = 0x3F,     // 0x2B, AGC Test
    CC1100_CONFIG_TEST2     = 0x88,     // 0x2D, Varoius Test Setting
    CC1100_CONFIG_TEST1     = 0x31,     // 0x2E, Varoius Test Setting
    CC1100_CONFIG_TEST0     = 0x0B      // 0x2F, Varoius Test Setting
}; // end 150

#elif (CC1100_BAUD == CC1100_250K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  CC1100_CONFIG_FIFOTHR = 0x0F,
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x0C,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0x2D,
  CC1100_CONFIG_MDMCFG3 = 0x3B,
  CC1100_CONFIG_MDMCFG2 = 0x0B,  // 0x03 = no manchester
  CC1100_CONFIG_MDMCFG1 = 0x22,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x62,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x1D,
  CC1100_CONFIG_BSCFG = 0x1C,
  CC1100_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0xB0, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0xB6,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xEA,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x88,
  CC1100_CONFIG_TEST1 = 0x31,
  CC1100_CONFIG_TEST0 = 0x0B,
   
}; // end 250

#elif (CC1100_BAUD == CC1100_500K)

enum CC1100_config_reg_state_enums {
  /** GDO2 is CHIP_RDY, even when the chip is first powered */
  CC1100_CONFIG_IOCFG2 = 0x29,
  
  /** GDO1 is High Impedance */
  CC1100_CONFIG_IOCFG1 = 0x2E,
  
  /** GDO0 asserts when there is data in the RX FIFO */
  CC1100_CONFIG_IOCFG0 = 0x01, 
  
  CC1100_CONFIG_FIFOTHR = 0x0F,
  CC1100_CONFIG_SYNC1 = 0xD3,
  CC1100_CONFIG_SYNC0 = 0x91,
  
  /** Maximum variable packet length is 61 per Errata */
  CC1100_CONFIG_PKTLEN = 0x3D,
  
  /** No hw address recognition for better ack rate, append 2 status bytes */
  CC1100_CONFIG_PKTCTRL1 = 0x24,
  
  /** CRC appending, variable length packets */
  CC1100_CONFIG_PKTCTRL0 = 0x45,
  
  CC1100_CONFIG_ADDR = 0x00,
  
  CC1100_CONFIG_CHANNR = CC1100_DEFAULT_CHANNEL,
  
  CC1100_CONFIG_FSCTRL1 = 0x0E,
  CC1100_CONFIG_FSCTRL0 = 0x00,
  
  CC1100_CONFIG_FREQ2 = CC1100_DEFAULT_FREQ2,
  CC1100_CONFIG_FREQ1 = CC1100_DEFAULT_FREQ1,
  CC1100_CONFIG_FREQ0 = CC1100_DEFAULT_FREQ0,
  
  CC1100_CONFIG_MDMCFG4 = 0x0E,
  CC1100_CONFIG_MDMCFG3 = 0x3B,
  CC1100_CONFIG_MDMCFG2 = 0x73,
  CC1100_CONFIG_MDMCFG1 = 0x42,
  CC1100_CONFIG_MDMCFG0 = 0xF8,
  CC1100_CONFIG_DEVIATN = 0x00,
  CC1100_CONFIG_MCSM2 = 0x07,
  
  /** TX on CCA; Stay in Rx after Rx and Tx */
  CC1100_CONFIG_MCSM1 = 0x3F,
  
  /** I experimented with a cal every 4th time, but I never saw any, ever..? */
  CC1100_CONFIG_MCSM0 = 0x18,
  
  CC1100_CONFIG_FOCCFG = 0x1D,
  CC1100_CONFIG_BSCFG = 0x1C,
  CC1100_CONFIG_AGCTRL2 = 0xC7,   // If no Tx, lower LNA's (look at AGC)
  CC1100_CONFIG_AGCTRL1 = 0x00,   // CCA thresholds
  CC1100_CONFIG_AGCTRL0 = 0xB0, 
  
  CC1100_CONFIG_WOREVT1 = 0x87,
  CC1100_CONFIG_WOREVT0 = 0x6B,
  CC1100_CONFIG_WORCTRL = 0xF8,
  CC1100_CONFIG_FREND1 = 0xB6,
  CC1100_CONFIG_FREND0 = 0x10,
  CC1100_CONFIG_FSCAL3 = 0xEA,
  CC1100_CONFIG_FSCAL2 = 0x2A,
  CC1100_CONFIG_FSCAL1 = 0x00,
  CC1100_CONFIG_FSCAL0 = 0x1F,
  
  CC1100_CONFIG_RCCTRL1 = 0x41,
  CC1100_CONFIG_RCCTRL0 = 0x00,
  CC1100_CONFIG_FSTEST = 0x59,
  CC1100_CONFIG_PTEST = 0x7F,
  CC1100_CONFIG_AGCTST = 0x3F,
  CC1100_CONFIG_TEST2 = 0x88,
  CC1100_CONFIG_TEST1 = 0x31,
  CC1100_CONFIG_TEST0 = 0x0B,
   
};
#else
#error "Baud rate not defined"
#endif

#ifndef CCXX00_RADIO_DEFINED
#define CCXX00_RADIO_DEFINED
#endif

#endif

