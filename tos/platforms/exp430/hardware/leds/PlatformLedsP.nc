/*
 * Copyright (c) 2009-2010 People Power Co.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This module provides the general Led interface.
 *
 * The advantage to doing it this way is we can now create a platforms
 * that provide more or less than 3 LED's, and the LED's can be pull-up or
 * pull-down enabled.
 *
 * @author David Moss
 */

generic module PlatformLedsP(){
  provides {
    interface Init;
    interface Leds;
  }

  uses {
    interface GeneralIO as Led0;
    interface GeneralIO as Led1;
    interface GeneralIO as Led2;
    interface GeneralIO as Led3;
    interface GeneralIO as Led4;
    interface GeneralIO as Led5;
    interface GeneralIO as Led6;
    interface GeneralIO as Led7;
  }
}

implementation {

  /***************** Init Commands ****************/
  command error_t Init.init() {
    atomic {
      call Led0.makeOutput();
      call Led1.makeOutput();
      call Led2.makeOutput();
      call Led3.makeOutput();
      call Led4.makeOutput();
      call Led5.makeOutput();
      call Led6.makeOutput();
      call Led7.makeOutput();
      call Led0.clr();
      call Led1.clr();
      call Led2.clr();
      call Led3.clr();
      call Led4.clr();
      call Led5.clr();
      call Led6.clr();
      call Led7.clr();
    }
    return SUCCESS;
  }

  /***************** Leds Commands ****************/
  async command void Leds.led0On() {
    call Led0.set();
  }

  async command void Leds.led0Off() {
    call Led0.clr();
  }

  async command void Leds.led0Toggle() {
    call Led0.toggle();
  }

  async command void Leds.led1On() {
    call Led1.set();
  }

  async command void Leds.led1Off() {
    call Led1.clr();
  }

  async command void Leds.led1Toggle() {
    call Led1.toggle();
  }

 async command void Leds.led2On() {
    call Led2.set();
  }

  async command void Leds.led2Off() {
    call Led2.clr();
  }

  async command void Leds.led2Toggle() {
    call Led2.toggle();
  }
  
  async command void Leds.led3On() {
    call Led3.set();
  }
  
  async command void Leds.led3Off() {
    call Led3.clr();
  }
  
  async command void Leds.led3Toggle() {
    call Led3.toggle();
  } 
  
  
  async command void Leds.led4On() {
    call Led4.set();
  }
  
  async command void Leds.led4Off() {
    call Led4.clr();
  }
  
  async command void Leds.led4Toggle() {
    call Led4.toggle();
  }

  
  async command void Leds.led5On() {
    call Led5.set();
  }
  
  async command void Leds.led5Off() {
    call Led5.clr();
  }
  
  async command void Leds.led5Toggle() {
    call Led5.toggle();
  }

  
  async command void Leds.led6On() {
    call Led6.set();
  }
  
  async command void Leds.led6Off() {
    call Led6.clr();
  }
  
  async command void Leds.led6Toggle() {
    call Led6.toggle();
  }

  
  async command void Leds.led7On() {
    call Led7.set();
  }
  
  async command void Leds.led7Off() {
    call Led7.clr();
  }
  
  async command void Leds.led7Toggle() {
    call Led7.toggle();
  }

  async command uint8_t Leds.get() {
    uint8_t rval;
    atomic {
      rval = 0;
      if (call Led0.get()) {
        rval |= LEDS_LED0;
      }
      if (call Led1.get()) {
        rval |= LEDS_LED1;
      }
      if (call Led2.get()) {
        rval |= LEDS_LED2;
      }
      if (call Led3.get()) {
        rval |= LEDS_LED3;
      }
      if (call Led4.get()) {
        rval |= LEDS_LED4;
      }
      if (call Led5.get()) {
        rval |= LEDS_LED5;
      }
      if (call Led6.get()) {
        rval |= LEDS_LED6;
      }
      if (call Led7.get()) {
        rval |= LEDS_LED7;
      }
    }
    return rval;
  }

  async command void Leds.set(uint8_t val) {
    atomic {
      if (val & LEDS_LED0) {
        call Leds.led0On();
      } else {
        call Leds.led0Off();
      }
      if (val & LEDS_LED1) {
        call Leds.led1On();
      } else {
        call Leds.led1Off();
      }
      if (val & LEDS_LED2) {
        call Leds.led2On();
      } else {
        call Leds.led2Off();
      }
      if (val & LEDS_LED3) {
        call Leds.led3On();
      } else {
        call Leds.led3Off();
      }
      if (val & LEDS_LED4) {
        call Leds.led4On();
      } else {
        call Leds.led4Off();
      }
      if (val & LEDS_LED5) {
        call Leds.led5On();
      } else {
        call Leds.led5Off();
      }
      if (val & LEDS_LED6) {
        call Leds.led6On();
      } else {
        call Leds.led6Off();
      }
      if (val & LEDS_LED7) {
        call Leds.led7On();
      } else {
        call Leds.led7Off();
      }
    }
  }
}
