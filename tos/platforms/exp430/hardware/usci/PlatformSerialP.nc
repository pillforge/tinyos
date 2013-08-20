/*
 * Copyright (c) 2009-2010 People Power Company
 * All rights reserved.
 *
 * This open source code was developed with funding from People Power Company
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
 * @author David Moss
 * @author Peter A. Bigot <pab@peoplepowerco.com>
 * @author Addisu Z. Taddese <addisu.z.taddese@vanderbilt.edu>
 */

/*
 * This table assumes UART clock input (SMCLK) is 8MHz
 */
msp430_usci_config_t msp430_usci_uart_exp430_config = {
  /* N81 UART mode driven by SMCLK */
  ctl0 : 0,
  ctl1 : UCSSEL__SMCLK,

  /* SLAU208 Table 34-4 8MHz 9600: UBR=833, BRS=2, BRF=0 */
  /*brw : 833, // 9600*/
  /*br0 : 65,*/
  br0 : 8,
  /*br1 : 3,*/
  mctl : UCBRF_0 | UCBRS_2
};

module PlatformSerialP {
  provides interface StdControl;
  provides interface Msp430UsciConfigure;
  uses interface Resource;
}

implementation {

  command error_t StdControl.start(){
    return call Resource.immediateRequest();
  }

  command error_t StdControl.stop(){
    return call Resource.release();
  }

  event void Resource.granted() { }

  async command const msp430_usci_config_t* Msp430UsciConfigure.getConfiguration(){
    return &msp430_usci_uart_exp430_config;
  }
}
