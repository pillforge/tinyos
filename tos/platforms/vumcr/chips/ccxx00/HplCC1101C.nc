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

configuration HplCC1101C {
  provides {
    interface Resource as SpiResource;
    interface SpiByte;
    interface SpiBlock;
    interface GeneralIO as CSN;
    interface GeneralIO as GDO0;
    interface GeneralIO as GDO2;
    interface GeneralIO as RADIO_EN;
    interface GpioCapture as Gdo0Capture;
    /*interface GpioCapture as Gdo2Capture;*/

    interface LocalTime<TRadio> as LocalTimeRadio;
    interface Init;
    interface Alarm<TRadio,uint16_t>;
  }
}
implementation {

  components HplMsp430GeneralIOC as IO, HplRadioSpiC as SpiC;

  // pins
  components new Msp430GpioC() as CSNM;
  components new Msp430GpioC() as GDO0M;
  components new Msp430GpioC() as GDO2M;
  components new Msp430GpioC() as RADIO_ENM;

#ifdef OLD_BOARD
  CSNM -> IO.Port26;
#else
  CSNM -> IO.Port21;
#endif
  GDO0M -> IO.Port23;
  GDO2M -> IO.Port24;
  RADIO_ENM -> IO.Port17;

  CSN = CSNM;
  GDO0 = GDO0M;
  GDO2 = GDO2M;
  RADIO_EN = RADIO_ENM;

  // spi	
  SpiResource = SpiC.Resource;
  SpiByte = SpiC;
  SpiBlock = SpiC;

  // capture
  /*components Msp430TimerC as TimerC;*/
  /*components new GpioCaptureC();*/
  /*GpioCaptureC.Msp430TimerControl -> TimerC.Control2_A0;*/
  /*GpioCaptureC.Msp430Capture -> TimerC.Capture2_A0;*/
  /*GpioCaptureC.GeneralIO -> IO.Port23;*/
  /*Gdo0Capture = GpioCaptureC;*/
  // SoftCapture is used instead since we don't have stringent timestamping requirements
  // and because we are running out of timers (Timer2_A0 can now be used for TMilli)
  components new SoftCaptureC();
  components new Msp430InterruptC(), HplMsp430InterruptC as IOIntC;

  Msp430InterruptC -> IOIntC.Port23;
  SoftCaptureC.GpioInterrupt -> Msp430InterruptC;
  Gdo0Capture = SoftCaptureC.GpioCapture;

  // alarm
  components HplRadioAlarmC as AlarmC;
  Alarm = AlarmC;
  Init = AlarmC;

  // localTime
  components LocalTime32khzC;
  LocalTimeRadio = LocalTime32khzC.LocalTime;

}
