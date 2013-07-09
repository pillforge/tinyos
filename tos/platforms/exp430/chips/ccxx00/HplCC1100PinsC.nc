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

configuration HplCC1100PinsC {
  provides {
    interface Resource as SpiResource;
    /*interface FastSpiByte;*/
    interface SpiByte as FastSpiByte;
    interface GeneralIO as CSN;
    interface GeneralIO as GDO0;
    interface GeneralIO as GDO2;
    /*interface GpioCapture as Gdo0Capture;*/
    /*interface GpioCapture as Gdo2Capture;*/

    // For ccxx00 driver, in case we use that
    interface GeneralIO as Power;
    interface GeneralIO as Csn;
    interface GeneralIO as Gdo0_io;
    interface GeneralIO as Gdo2_io;
    interface GpioInterrupt as Gdo0_int;
    interface GpioInterrupt as Gdo2_int;

    /*interface LocalTime<TRadio> as LocalTimeRadio;*/
    /*interface Init;*/
    /*interface Alarm<TRadio,uint16_t>;*/
  }
}
implementation {

  components HplMsp430GeneralIOC as IO, new Msp430UsciSpiB0C() as SpiC;

  // pins
  components new Msp430GpioC() as CSNM;
  components new Msp430GpioC() as GDO0M;
  components new Msp430GpioC() as GDO2M;

  CSNM -> IO.Port26;
  GDO0M -> IO.Port23;
  GDO2M -> IO.Port24;

  CSN = CSNM;
  GDO0 = GDO0M;
  GDO2 = GDO2M;

  Csn = CSNM;
  Gdo0_io = GDO0M;
  Gdo2_io = GDO2M;

  components DummyIoP;
  Power = DummyIoP;

  // spi	
  SpiResource = SpiC.Resource;
  FastSpiByte = SpiC;

  // capture
/*
 *  components Msp430TimerC as TimerC;
 *  components new GpioCaptureC();
 *  GpioCaptureC.Msp430TimerControl -> TimerC.ControlA1;
 *  GpioCaptureC.Msp430Capture -> TimerC.CaptureA1;
 *  GpioCaptureC.GeneralIO -> IO.Port12;
 *  SfdCapture = GpioCaptureC;
 *
 */
  // interrupts
  components HplMsp430InterruptC;
  components new Msp430InterruptC() as Gdo0_intm;
  Gdo0_intm.HplInterrupt -> HplMsp430InterruptC.Port23;
  Gdo0_int = Gdo0_intm.Interrupt; 

  components new Msp430InterruptC() as Gdo2_intm;
  Gdo2_intm.HplInterrupt -> HplMsp430InterruptC.Port24;
  Gdo2_int = Gdo2_intm.Interrupt; 


  // alarm
/*
 *  components new Alarm32khz16C() as AlarmC;
 *  Alarm = AlarmC;
 *  Init = AlarmC;
 *
 *  // localTime
 *  components LocalTime32khzC;
 *  LocalTimeRadio = LocalTime32khzC.LocalTime;
 */

}
