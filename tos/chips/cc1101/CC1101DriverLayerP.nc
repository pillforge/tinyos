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

#include <CC1101DriverLayer.h>
#include <Tasklet.h>
#include <RadioAssert.h>
#include <TimeSyncMessageLayer.h>
#include <RadioConfig.h>
module CC1101DriverLayerP
{
  provides
  {
    interface Init as SoftwareInit @exactlyonce();

    interface RadioState;
    interface RadioSend;
    interface RadioReceive;
    interface RadioCCA;
    interface RadioPacket;

    interface PacketField<uint8_t> as PacketTransmitPower;
    interface PacketField<uint8_t> as PacketRSSI;
    interface PacketField<uint8_t> as PacketTimeSyncOffset;
    interface PacketField<uint8_t> as PacketLinkQuality;
    interface LinkPacketMetadata;
  }

  uses
  {
    interface Resource as SpiResource;
    interface BusyWait<TMicro, uint16_t>;
    interface LocalTime<TRadio>;
    interface CC1101DriverConfig as Config;

    interface SpiByte;
    interface SpiBlock;
    interface GeneralIO as CSN;
    interface GeneralIO as GDO0;
    interface GeneralIO as GDO2;
    interface GpioCapture as Gdo0Capture;

    interface PacketFlag as TransmitPowerFlag;
    interface PacketFlag as RSSIFlag;
    interface PacketFlag as TimeSyncFlag;

    interface PacketTimeStamp<TRadio, uint32_t>;

    interface Tasklet;
    interface RadioAlarm;

#ifdef RADIO_DEBUG
    interface DiagMsg;
#endif
    //for apps/PPPSniffer:
#ifdef PPPSNIFFER
    interface SplitControl as PppSplitControl;
    interface LcpAutomaton as Ipv6LcpAutomaton;
    interface PppIpv6;
#endif
    interface Leds;
  }
}

implementation
{
  cc1101_header_t* getHeader(message_t* msg)
  {
    return ((void*)msg) + call Config.headerLength(msg);
  }

  void* getPayload(message_t* msg)
  {
    return ((void*)msg);
  }

  cc1101_metadata_t* getMeta(message_t* msg)
  {
    return ((void*)msg) + sizeof(message_t) - call RadioPacket.metadataLength(msg);
  }

  /*----------------- STATE -----------------*/

  enum
  {
    STATE_POR = 0,
    STATE_PD = 1,
    STATE_IDLE = 3,
    STATE_IDLE_2_RX_ON = 4,
    STATE_RX_ON = 5,
    STATE_RX_WAIT_END_PKT = 6,
    STATE_BUSY_TX_2_RX_ON = 7,
    STATE_IDLE_2_TX_ON = 8,
    STATE_TX_ON = 9,
    STATE_RX_DOWNLOAD = 10,
    STATE_RX_INVALID = 11,
  };
  norace uint8_t state = STATE_POR;

  enum
  {
    CMD_NONE = 0,     // the state machine has stopped
    CMD_TURNOFF = 1,    // goto SLEEP state
    CMD_STANDBY = 2,    // goto TRX_OFF state
    CMD_TURNON = 3,     // goto RX_ON state
    CMD_TRANSMIT = 4,   // currently transmitting a message
    CMD_RECEIVE = 5,    // currently receiving a message
    CMD_CCA = 6,      // performing clear chanel assesment
    CMD_CHANNEL = 7,    // changing the channel
    CMD_SIGNAL_DONE = 8,  // signal the end of the state transition
    CMD_DOWNLOAD = 9,   // download the received message
    CMD_RX_FLUSH = 10,   // Fluch RX FIFO
  };
  tasklet_norace uint8_t cmd = CMD_NONE;

  //for apps/PPPSniffer
#ifdef PPPSNIFFER
  enum {
    PPP_QUEUE_LEN = 10,
  };

  message_t  pppQueueBufs[PPP_QUEUE_LEN];
  message_t  *pppQueue[PPP_QUEUE_LEN];
  uint8_t    pppIn, pppOut;
  bool       pppBusy, pppFull;

#endif

  // flag: RX GDO0 was captured, but not yet processed
  norace bool rxGdo0 = 0;
  // flag: end of TX event (falling GDO0 edge) was captured, but not yet processed
  norace bool txEnd = 0;

  tasklet_norace uint8_t txPower;
  tasklet_norace uint8_t channel;

  tasklet_norace message_t* rxMsg;
#ifdef RADIO_DEBUG_MESSAGES
  tasklet_norace message_t* txMsg;
#endif
  message_t rxMsgBuffer;


  // initial CC1101 configuration

  uint8_t configRegs[] = {
    CC1101_CONFIG_IOCFG2,
    CC1101_CONFIG_IOCFG1,
    CC1101_CONFIG_IOCFG0,
    CC1101_CONFIG_FIFOTHR,
    CC1101_CONFIG_SYNC1,
    CC1101_CONFIG_SYNC0,
    CC1101_CONFIG_PKTLEN,
    CC1101_CONFIG_PKTCTRL1,
    CC1101_CONFIG_PKTCTRL0,
    CC1101_CONFIG_ADDR,
    CC1101_CONFIG_CHANNR,
    CC1101_CONFIG_FSCTRL1,
    CC1101_CONFIG_FSCTRL0,
    CC1101_CONFIG_FREQ2,
    CC1101_CONFIG_FREQ1,
    CC1101_CONFIG_FREQ0,
    CC1101_CONFIG_MDMCFG4,
    CC1101_CONFIG_MDMCFG3,
    CC1101_CONFIG_MDMCFG2,
    CC1101_CONFIG_MDMCFG1,
    CC1101_CONFIG_MDMCFG0,
    CC1101_CONFIG_DEVIATN,
    CC1101_CONFIG_MCSM2,
    CC1101_CONFIG_MCSM1,
    CC1101_CONFIG_MCSM0,
    CC1101_CONFIG_FOCCFG,
    CC1101_CONFIG_BSCFG,
    CC1101_CONFIG_AGCTRL2,
    CC1101_CONFIG_AGCTRL1,
    CC1101_CONFIG_AGCTRL0,
    CC1101_CONFIG_WOREVT1,
    CC1101_CONFIG_WOREVT0,
    CC1101_CONFIG_WORCTRL,
    CC1101_CONFIG_FREND1,
    CC1101_CONFIG_FREND0,
    CC1101_CONFIG_FSCAL3,
    CC1101_CONFIG_FSCAL2,
    CC1101_CONFIG_FSCAL1,
    CC1101_CONFIG_FSCAL0,
    CC1101_CONFIG_RCCTRL1,
    CC1101_CONFIG_RCCTRL0,
    CC1101_CONFIG_FSTEST,
    CC1101_CONFIG_PTEST,
    CC1101_CONFIG_AGCTST,
    CC1101_CONFIG_TEST2,
    CC1101_CONFIG_TEST1,
    CC1101_CONFIG_TEST0
  };

  uint8_t configRegSize = 47;


  norace uint16_t capturedTime; // time when the last GDO0 rising edge was captured

  inline cc1101_status_t getStatus();
  inline cc1101_status_t enableReceiveGdo();

  /*----------------- ALARM -----------------*/
  tasklet_async event void RadioAlarm.fired()
  {
    cc1101_status_t status;
    if( state == STATE_IDLE_2_RX_ON ) {
      status = getStatus();
#ifdef RADIO_DEBUG_STATE
      if( call DiagMsg.record() )
      {
        call DiagMsg.uint16(call RadioAlarm.getNow());
        call DiagMsg.str("alrm");
        call DiagMsg.str("s=");
        call DiagMsg.uint8(state);
        call DiagMsg.str("c=");
        call DiagMsg.uint8(cmd);
        if(rxGdo0)
          call DiagMsg.str("rxGdo0");
        if(txEnd)
          call DiagMsg.str("txEnd");
        if(call GDO0.get())
          call DiagMsg.str("GDO0");
        call DiagMsg.str("st=");
        call DiagMsg.uint8(status.value);

        call DiagMsg.send();
      }
#endif
      if (status.state == CC1101_STATE_RX){
        state = STATE_RX_ON;
        cmd = CMD_SIGNAL_DONE;
        // in receive mode, enable GDO0 capture
        enableReceiveGdo();
      } else {
        call RadioAlarm.wait(IDLE_2_RX_ON_TIME); // 12 symbol periods
      }
    } else
      RADIO_ASSERT(FALSE);

    // make sure the rest of the command processing is called
    call Tasklet.schedule();
  }

  /*----------------- REGISTER -----------------*/

  inline uint16_t readRegister(uint8_t reg)
  {
    uint16_t value = 0;

    RADIO_ASSERT( call SpiResource.isOwner() );
    RADIO_ASSERT( reg == (reg & CC1101_CMD_REGISTER_MASK) );

    call CSN.set();
    call CSN.clr();

    call SpiByte.write(CC1101_CMD_REGISTER_WRITE | reg);
    value = call SpiByte.write(0);
    call CSN.set();

    return value;
  }

  inline cc1101_status_t strobe(uint8_t reg)
  {
    cc1101_status_t status;

    RADIO_ASSERT( call SpiResource.isOwner() );
    RADIO_ASSERT( reg == (reg & CC1101_CMD_REGISTER_MASK) );

    call CSN.set();
    call CSN.clr();

    status.value = call SpiByte.write(CC1101_CMD_REGISTER_WRITE | reg);

    call CSN.set();
    return status;

  }

  inline cc1101_status_t getStatus() {
    int8_t value1 = 0, value2 = 0;
    cc1101_status_t status;


    RADIO_ASSERT( call SpiResource.isOwner() );

    call CSN.set();
    call CSN.clr();
    value1 = call SpiByte.write(CC1101_CMD_REGISTER_WRITE | CC1101_SNOP);
    do {
      value2 = value1;
      value1 = call SpiByte.write(CC1101_CMD_REGISTER_WRITE | CC1101_SNOP);
    }while(value1 != value2);
    call CSN.set();

    status.value = value1;
    return status;
  }

  inline cc1101_status_t writeRegister(uint8_t reg, uint8_t value)
  {
    cc1101_status_t status;

    RADIO_ASSERT( call SpiResource.isOwner() );
    RADIO_ASSERT( reg == (reg & CC1101_CMD_REGISTER_MASK) );

    call CSN.set();
    call CSN.clr();

    call SpiByte.write(CC1101_CMD_REGISTER_WRITE | reg);
    status.value = call SpiByte.write(value);

    call CSN.set();
    return status;
  }

  inline cc1101_status_t burstWrite(uint8_t addr,uint8_t* data, uint8_t length)
  {
    cc1101_status_t status;
    RADIO_ASSERT( call SpiResource.isOwner() );

    status.value = call SpiByte.write(CC1101_CMD_REGISTER_WRITE | CC1101_CMD_BURST_MODE  | addr);
    call SpiBlock.transfer(data, NULL, length);

    return status;
  }
  inline cc1101_status_t burstRead(uint8_t addr,uint8_t* data, uint8_t length)
  {
    cc1101_status_t status;
    RADIO_ASSERT( call SpiResource.isOwner() );

    status.value = call SpiByte.write(CC1101_CMD_REGISTER_READ | CC1101_CMD_BURST_MODE  | addr);
    call SpiBlock.transfer(NULL, data, length);

    return status;
  }

  inline cc1101_status_t writeTxFifo(uint8_t* data, uint8_t length)
  {
    return burstWrite(CC1101_TXFIFO, data, length);
  }
  inline cc1101_status_t readRxFifo(uint8_t* data, uint8_t length)
  {
    return burstRead(CC1101_RXFIFO, data, length);
  }


  inline cc1101_status_t readLengthFromRxBytes(uint8_t* lengthPtr)
  {
    cc1101_status_t status;
    uint8_t pkt_status;
    uint8_t length1, length2;

    RADIO_ASSERT( call SpiResource.isOwner() );
    RADIO_ASSERT( call CSN.get() == 1 );


    call CSN.set(); // set CSN, just in clase it's not set
    call CSN.clr(); // clear CSN, starting a multi-byte SPI command

    // For debugging using Logic Analyzer
    burstRead(CC1101_PKTSTATUS, &pkt_status, 1);

    /*status.value = call SpiByte.write(CC1101_CMD_REGISTER_READ | CC1101_CMD_BURST_MODE | CC1101_RXBYTES);*/
    /**lengthPtr = call SpiByte.write(0);*/

    status = burstRead(CC1101_RXBYTES, &length1, 1);
    do {
      length2 = length1;
      status = burstRead(CC1101_RXBYTES, &length1, 1);
    }while(length1 != length2);

    RADIO_ASSERT(status.chip_rdyn == 0);
    RADIO_ASSERT(status.state == CC1101_STATE_RX);

    *lengthPtr = length1;

    return status;
  }
  // The first byte in the RX fifo contains the length
  // This assumes that this is called first before reading any of the bytes in the RX FIFO
  inline cc1101_status_t readLengthFromRxFifo(uint8_t* lengthPtr){
    cc1101_status_t status;
    call CSN.set(); // set CSN, just in clase it's not set
    call CSN.clr(); // clear CSN, starting a multi-byte SPI command
    status = readRxFifo(lengthPtr,1);
    /*call CSN.set(); // set CSN, just in clase it's not set*/
    /*call CSN.clr(); // clear CSN, starting a multi-byte SPI command*/
    return status;
  }

  inline void readPayloadFromRxFifo(uint8_t* data, uint8_t length)
  {
    // readLengthFromRxFifo was called before, so CSN is cleared and spi is ours
    RADIO_ASSERT( call CSN.get() == 0 );

    call SpiBlock.transfer(NULL, data, length);
  }

  inline void readRssiFromRxFifo(uint8_t* rssiPtr)
  {
    // readLengthFromRxFifo was called before, so CSN is cleared and spi is ours
    RADIO_ASSERT( call CSN.get() == 0 );

    *rssiPtr = call SpiByte.write(0);
  }

  inline void readCrcOkAndLqiFromRxFifo(uint8_t* crcOkAndLqiPtr)
  {
    // readLengthFromRxFifo was called before, so CSN is cleared and spi is ours
    RADIO_ASSERT( call CSN.get() == 0 );

    *crcOkAndLqiPtr = call SpiByte.write(0);

    // end RxFifo read operation
    call CSN.set();
  }

  inline cc1101_status_t waitForState(uint8_t st, uint8_t timeout) {
    cc1101_status_t status;
    uint8_t counter = 0;
    do {
      status = getStatus();
      counter++;
    }while(status.state != st && counter < timeout);
    return status;
  }
  inline cc1101_status_t flushRxFifo() {
    strobe(CC1101_SFRX);
    strobe(CC1101_SRX);
    return waitForState(CC1101_STATE_RX, 0xff);
  }

  /*----------------- INIT -----------------*/

  command error_t SoftwareInit.init()
  {
    //for apps/PPPSniffer
#ifdef PPPSNIFFER
    uint8_t i;
#endif

    /*uint16_t timeout;*/

    // set pin directions
    call CSN.makeOutput();
    call GDO0.makeInput();
    call GDO2.makeInput(); // CHIP_RDYn by default

    call Gdo0Capture.disable();

    rxMsg = &rxMsgBuffer;

    state = STATE_POR;

    //for apps/PPPSniffer
#ifdef PPPSNIFFER
    call Ipv6LcpAutomaton.open();
    call PppSplitControl.start();

    for (i = 0; i < PPP_QUEUE_LEN; i++)
      pppQueue[i] = &pppQueueBufs[i];

    pppIn = pppOut = 0;
    pppBusy = FALSE;
    pppFull = FALSE;
#endif

    // request SPI, rest of the initialization will be done from
    // the granted event
    return call SpiResource.request();
  }

  inline void resetRadio() {
    uint16_t timeout;

    // Go through reset procedure
    call CSN.set();
    call BusyWait.wait(30);
    call CSN.clr();
    call BusyWait.wait(30);
    call CSN.set();
    call BusyWait.wait(45);

    call CSN.clr();
    timeout = 0;
    // wait for CHIP_RDYn pin to go low
    while(call GDO2.get()) {
      timeout++;
      if(timeout > 10000) {
        strobe(CC1101_SRES);
        call CSN.clr();
        timeout = 0;
      }
    }
    // The chip is ready. XOSC is stable.

    // Strobe SRES
    strobe(CC1101_SRES);
    call CSN.clr();
    while(call GDO2.get());

    // Power down
    strobe(CC1101_SIDLE);
    strobe(CC1101_SPWD);

    state = STATE_PD;
  }


  void initRadio()
  {
    resetRadio();

    /*txPower = CC1101_DEF_RFPOWER & CC1101_TX_PWR_MASK;*/
    /*channel = CC1101_DEF_CHANNEL & CC1101_CHANNEL_MASK;*/

  }

  /*----------------- SPI -----------------*/

  event void SpiResource.granted()
  {

    call CSN.makeOutput();
    call CSN.set();

    if( state == STATE_POR )
    {
      initRadio();
      call SpiResource.release();
    }
    else
      call Tasklet.schedule();
  }

  bool isSpiAcquired()
  {
    if( call SpiResource.isOwner() )
      return TRUE;

    if( call SpiResource.immediateRequest() == SUCCESS )
    {
      call CSN.makeOutput();
      call CSN.set();

      return TRUE;
    }

    call SpiResource.request();
    return FALSE;
  }

  /*----------------- CHANNEL -----------------*/

  tasklet_async command uint8_t RadioState.getChannel()
  {
    return channel;
  }

  tasklet_async command error_t RadioState.setChannel(uint8_t c)
  {
/*
 *    c &= CC1101_CHANNEL_MASK;
 *
 *    if( cmd != CMD_NONE )
 *      return EBUSY;
 *    else if( channel == c )
 *      return EALREADY;
 *
 *    channel = c;
 *    cmd = CMD_CHANNEL;
 *    call Tasklet.schedule();
 *
 *    return SUCCESS;
 */
  }

  //TODO: Support setting channels
  inline void setChannel()
  {
/*
 *    cc1101_fsctrl_t fsctrl;
 *    // set up freq
 *    fsctrl= cc1101_fsctrl_default;
 *    fsctrl.f.freq = 357+5*(channel - 11);
 *
 *    writeRegister(CC1101_FSCTRL, fsctrl.value);
 */
  }

  inline void changeChannel()
  {
    RADIO_ASSERT( cmd == CMD_CHANNEL );
    RADIO_ASSERT( state == STATE_PD || state == STATE_IDLE || ( state == STATE_RX_ON && call RadioAlarm.isFree()));

    if( isSpiAcquired() )
    {
      setChannel();

      if( state == STATE_RX_ON ) {
        call RadioAlarm.wait(IDLE_2_RX_ON_TIME); // 12 symbol periods
        state = STATE_IDLE_2_RX_ON;
      }
      else
        cmd = CMD_SIGNAL_DONE;
    }
  }

  /*----------------- TURN ON/OFF -----------------*/

  inline void changeState()
  {

#ifdef RADIO_DEBUG_STATE
    if( call DiagMsg.record() )
    {
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.str("cs");
      call DiagMsg.str("s=");
      call DiagMsg.uint8(state);
      call DiagMsg.str("c=");
      call DiagMsg.uint8(cmd);
      call DiagMsg.str("l=");
      call DiagMsg.send();
    }
#endif

    if( (cmd == CMD_STANDBY || cmd == CMD_TURNON)
        && state == STATE_PD  && isSpiAcquired() && call RadioAlarm.isFree() )
    {
      call CSN.clr();
      while(call GDO2.get());
      // Go back to IDLE state
      strobe(CC1101_SIDLE);
      // Send configuration registers
      call CSN.clr();
      burstWrite(0x0, configRegs, configRegSize);
      call CSN.set();
      state = STATE_IDLE;
      call Tasklet.schedule();
    }
    else if( cmd == CMD_TURNON && state == STATE_IDLE && isSpiAcquired() && call RadioAlarm.isFree())
    {
      // setChannel was ignored in SLEEP because the SPI was not working, so do it here
      /*setChannel();*/

      // Flush anything that might be in the RX FIFO
      strobe(CC1101_SFRX);
      // start receiving
      strobe(CC1101_SRX);
      call RadioAlarm.wait(IDLE_2_RX_ON_TIME); // 12 symbol periods
      state = STATE_IDLE_2_RX_ON;
    }
    else if( (cmd == CMD_TURNOFF || cmd == CMD_STANDBY)
        && state == STATE_RX_ON && isSpiAcquired() )
    {
      // disable GDO0 capture
      call Gdo0Capture.disable();

      // stop receiving
      strobe(CC1101_SIDLE);
      state = STATE_IDLE;
    }

    if( cmd == CMD_TURNOFF && state == STATE_IDLE  && isSpiAcquired() )
    {
      // stop oscillator
      strobe(CC1101_SPWD);

      // do a reset
      initRadio();
      state = STATE_PD;
      cmd = CMD_SIGNAL_DONE;
    }
    else if( cmd == CMD_STANDBY && state == STATE_IDLE )
      cmd = CMD_SIGNAL_DONE;
  }

  tasklet_async command error_t RadioState.turnOff()
  {
    if( cmd != CMD_NONE )
      return EBUSY;
    else if( state == STATE_PD )
      return EALREADY;

#ifdef RADIO_DEBUG_STATE
    if( call DiagMsg.record() )
    {
      call DiagMsg.str("turnOff");
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.send();
    }
#endif

    cmd = CMD_TURNOFF;
    call Tasklet.schedule();

    return SUCCESS;
  }

  tasklet_async command error_t RadioState.standby()
  {
    if( cmd != CMD_NONE || (state == STATE_PD && ! call RadioAlarm.isFree()) )
      return EBUSY;
    else if( state == STATE_IDLE )
      return EALREADY;

#ifdef RADIO_DEBUG_STATE
    if( call DiagMsg.record() )
    {
      call DiagMsg.str("standBy");
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.send();
    }
#endif

    cmd = CMD_STANDBY;
    call Tasklet.schedule();

    return SUCCESS;
  }

  tasklet_async command error_t RadioState.turnOn()
  {
    if( cmd != CMD_NONE || (state == STATE_PD && ! call RadioAlarm.isFree()) )
      return EBUSY;
    else if( state == STATE_RX_ON )
      return EALREADY;

#ifdef RADIO_DEBUG_STATE
    if( call DiagMsg.record() )
    {
      call DiagMsg.str("turnOn");
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.send();
    }
#endif

    cmd = CMD_TURNON;
    call Tasklet.schedule();

    return SUCCESS;
  }

  default tasklet_async event void RadioState.done() { }

  /*----------------- TRANSMIT -----------------*/

  inline cc1101_status_t enableTransmitGdo()
  {
    cc1101_status_t status;
    atomic {
      // Finished sending message
      call Gdo0Capture.captureFallingEdge();
    }
    return status;
  }
  tasklet_async command error_t RadioSend.send(message_t* msg)
  {
    /*uint16_t time;*/
    /*uint8_t p;*/
    uint8_t length;
    uint8_t* data;
    /*
     *uint8_t header;
     *uint32_t time32;
     *void* timesync;
     *timesync_relative_t timesync_relative;
     *uint32_t sfdTime;
     */
    cc1101_status_t status;
#ifdef RADIO_DEBUG
    uint8_t sfd1, sfd2, sfd3, sfd4;
#endif
    if( cmd != CMD_NONE || (state != STATE_IDLE && state != STATE_RX_ON) || ! isSpiAcquired() || rxGdo0 || txEnd )
      return EBUSY;

    status = getStatus();
    if (status.state == CC1101_STATE_TXFIFO_UNDERFLOW){
      RADIO_ASSERT(FALSE);
      // flush tx fifo
      strobe(CC1101_SFTX);
      return EBUSY;
    }
    // start transmission
    status = strobe(CC1101_STX);

    // Do other useful things while we wait for TX state
    data = getPayload(msg);
    // The length byte does not count itself so add one here in order to send the
    // right number of bytes to the TXFIFO
    length = call RadioPacket.payloadLength(msg) + 1;

    // Fill up the FIFO
    call CSN.set();
    call CSN.clr();
    writeTxFifo(data, length);
    call CSN.set();

    // Check if in TX mode. If not in TX mode, CCA has failed
    status = getStatus();
    if (status.state != CC1101_STATE_TX)
      return EBUSY;

    atomic {
#ifdef RADIO_DEBUG
      sfd1 = call GDO0.get();
#endif
#ifdef RADIO_DEBUG
      sfd2 = call GDO0.get();
#endif
      // get a timestamp right after strobe returns
      /*time = call RadioAlarm.getNow();*/

      cmd = CMD_TRANSMIT;
      state = STATE_BUSY_TX_2_RX_ON;
#ifdef RADIO_DEBUG
      sfd3 = call GDO0.get();
#endif
      /*call Gdo0Capture.captureFallingEdge();*/
#ifdef RADIO_DEBUG
      sfd4 = call GDO0.get();
#endif
    }
    waitForState(CC1101_STATE_RX, 0xff);
    txEnd = TRUE;

#ifdef RADIO_DEBUG
    RADIO_ASSERT(sfd1 == 0);
    RADIO_ASSERT(sfd2 == 0);
    RADIO_ASSERT(sfd3 == 0);
    RADIO_ASSERT(sfd4 == 0);
#endif

    //TODO: implement timesync
/*
 *    timesync = call PacketTimeSyncOffset.isSet(msg) ? ((void*)msg) + call PacketTimeSyncOffset.get(msg) : 0;
 *
 *    if( timesync == 0 ) {
 *      // no timesync: write the entire payload to the fifo
 *      if(length>0){
 *        call CSN.set();
 *        call CSN.clr();
 *        writeTxFifo(data+header, length - 1);
 *        state = STATE_BUSY_TX_2_RX_ON;
 *        call CSN.set();
 *      }
 *    } else {
 *      // timesync required: write the payload before the timesync bytes to the fifo
 *      // TODO: we're assuming here that the timestamp is at the end of the message
 *      call CSN.set();
 *      call CSN.clr();
 *      writeTxFifo(data+header, length - sizeof(timesync_relative) - 1);
 *      call CSN.set();
 *    }
 *
 *
 *    // compute timesync
 *    sfdTime = time;
 *
 *    // read both clocks
 *    atomic {
 *      time = call RadioAlarm.getNow();
 *      time32 = call LocalTime.get();
 *    }
 *
 *    // adjust time32 with the time elapsed since the GDO0 event
 *    time -= sfdTime;
 *    time32 -= time;
 *
 *    // adjust for delay between the STXON strobe and the transmission of the GDO0
 *    time32 += TX_SFD_DELAY;
 *
 *    call PacketTimeStamp.set(msg, time32);
 *
 *    if( timesync != 0 ) {
 *      // read and adjust the timestamp field
 *      timesync_relative = (*(timesync_absolute_t*)timesync) - time32;
 *
 *      // write it to the fifo
 *      // TODO: we're assuming here that the timestamp is at the end of the message
 *      call CSN.set();
 *      call CSN.clr();
 *      writeTxFifo((uint8_t*)(&timesync_relative), sizeof(timesync_relative));
 *      call CSN.set();
 *      state = STATE_BUSY_TX_2_RX_ON;
 *    }
 */


#ifdef RADIO_DEBUG_MESSAGES
    txMsg = msg;

    if( call DiagMsg.record() )
    {
      length = call RadioPacket.payloadLength(msg) + 1;

      call DiagMsg.chr('t');
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.uint32(call PacketTimeStamp.isValid(msg) ? call PacketTimeStamp.timestamp(msg) : 0);
      call DiagMsg.int8(length);
      call DiagMsg.hex8s(getPayload(msg), length);
      call DiagMsg.send();
    }
#endif

    // GDO0 capture interrupt will be triggered: we'll reenable interrupts from there
    // and clear the rx fifo -- should something have arrived in the meantime
    /*call Leds.led2Off();*/
    call Tasklet.schedule();
    return SUCCESS;
  }

  default tasklet_async event void RadioSend.sendDone(error_t error) { }
  default tasklet_async event void RadioSend.ready() { }

  /*----------------- CCA -----------------*/

  // The CC1101 has TX-if-CCA. This feature allows for the CC1101 to enter its TX state only if clear channel
  // requirements are met. Thus, we always return SUCCESS here and re-evaluate CCA when trying to transmit.
  tasklet_async command error_t RadioCCA.request()
  {
    if( cmd != CMD_NONE || state != STATE_RX_ON )
      return EBUSY;

    signal RadioCCA.done(SUCCESS);
    return SUCCESS;
  }

  default tasklet_async event void RadioCCA.done(error_t error) { }

  /*----------------- RECEIVE -----------------*/

  inline cc1101_status_t enableReceiveGdo()
  {
    cc1101_status_t status;
    atomic {
      // ready to receive new message: enable receive GDO0 capture
      call Gdo0Capture.captureRisingEdge();
      /*call Gdo0Capture.captureFallingEdge();*/
    }
    return status;
  }


  //for apps/PPPSniffer
#ifdef PPPSNIFFER
  task void ppptransmit()
  {
    uint8_t len;
    message_t* msg;

    atomic {
      if (pppIn == pppOut && !pppFull) {
        pppBusy = FALSE;
        return;
      }

      msg = pppQueue[pppOut];
      len = getHeader(msg)->length; // separate FCS/CRC
    }

    call Leds.led1Toggle();
    //if (call UartSend.send(uartQueue[uartOut], len) == SUCCESS) {
    if (call PppIpv6.transmit(getPayload(msg)+1,
          len) == SUCCESS) {
      //call Leds.led2Toggle();
      atomic {
        if (msg == pppQueue[pppOut]) {
          if (++pppOut >= PPP_QUEUE_LEN)
            pppOut = 0;
          if (pppFull)
            pppFull = FALSE;
        }
      }
      post ppptransmit();
    } else {
      post ppptransmit();
    }
  }

#endif

  inline void downloadMessage()
  {
    uint8_t fifo_length;
    uint16_t crc = 1;
    uint8_t* data;
    uint8_t rssi;
    uint8_t crc_ok_lqi;
    uint16_t sfdTime;

    cc1101_status_t status;

    status = getStatus();

    state = STATE_RX_DOWNLOAD;

    sfdTime = capturedTime;

    // data starts after the length field
    data = getPayload(rxMsg) + sizeof(cc1101_header_t);

    // read the length of bytes in the RX FIFO
    status = readLengthFromRxFifo(&fifo_length);

#ifdef RADIO_DEBUG_STATE
    if( call DiagMsg.record() )
    {
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.str("dwnld");
      call DiagMsg.str("s=");
      call DiagMsg.uint8(state);
      call DiagMsg.str("c=");
      call DiagMsg.uint8(cmd);
      call DiagMsg.str("l=");
      call DiagMsg.uint8(fifo_length);
      call DiagMsg.send();
    }
#endif
    if (fifo_length < 3 || fifo_length > call RadioPacket.maxPayloadLength() + 2 ) {
      // bad length: bail out
      state = STATE_RX_ON;
      cmd = CMD_NONE;
      call CSN.set();
      enableReceiveGdo();
      return;
    }

    // if we're here, length must be correct
    RADIO_ASSERT(fifo_length >= 3 && fifo_length <= call RadioPacket.maxPayloadLength() + 2);

    // The length does not include the length byte
    getHeader(rxMsg)->length = fifo_length;

    // TODO: The payload contains the length byte, thus setting the length byte in the previous line of code is useless.
    // Investigate if we should consider the length byte as part of the payload or not.
    // download the whole payload
    readPayloadFromRxFifo(data, fifo_length );

    // the last two bytes are not the fsc, but RSSI(8), CRC_ON(1)+LQI(7)
    readRssiFromRxFifo(&rssi);
    readCrcOkAndLqiFromRxFifo(&crc_ok_lqi);

    /* UNCOMMENT THIS CODE IF THERE ARE TIMESTAMPING ERRORS
    // there are still bytes in the fifo or if there's an overflow, flush rx fifo
    if (call FIFOP.get() == 1 || call FIFO.get() == 1 || call GDO0.get() == 1) {
    RADIO_ASSERT(FALSE);

    state = STATE_RX_ON;
    cmd = CMD_NONE;

    RADIO_ASSERT(call GDO0.get() == 0);
    enableReceiveGdo();
    return;
    }
     */

    state = STATE_RX_ON;
    cmd = CMD_NONE;

    // ready to receive new message: enable GDO0 interrupts
    // If RX FIFO overflowed, we have to flush it before we enable interrupts
    if(status.state != CC1101_STATE_RXFIFO_OVERFLOW)
      enableReceiveGdo();

#ifdef RADIO_DEBUG_MESSAGES
    if( call DiagMsg.record() )
    {
      call DiagMsg.str("rmsg");
      call DiagMsg.uint16(call RadioAlarm.getNow() - (uint16_t)call PacketTimeStamp.timestamp(rxMsg) );
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.uint16(call PacketTimeStamp.isValid(rxMsg) ? call PacketTimeStamp.timestamp(rxMsg) : 0);
      call DiagMsg.int8(fifo_length);
      call DiagMsg.hex8s(getPayload(rxMsg), fifo_length);
      call DiagMsg.send();
    }
#endif

    // bail out if we're not interested in this message
    if( !signal RadioReceive.header(rxMsg) )
      return;

    // set RSSI, CRC and LQI only if we're accepting the message
    call PacketRSSI.set(rxMsg, rssi);
    call PacketLinkQuality.set(rxMsg, crc_ok_lqi & 0x7f);
    crc = (crc_ok_lqi > 0x7f) ? 0 : 1;


    //for apps/PPPSniffer
#ifdef PPPSNIFFER
    call Leds.led0Toggle();
    atomic {
      if (!pppFull) {
        //ret = pppQueue[pppIn];
        pppQueue[pppIn] = rxMsg;

        pppIn = (pppIn + 1) % PPP_QUEUE_LEN;

        if (pppIn == pppOut)
          pppFull = TRUE;

        if (!pppBusy) {
          post ppptransmit();
          pppBusy = TRUE;
        }
      }
    }
    //call PppIpv6.transmit(getPayload(rxMsg)+1,
    //          fifo_length+4);
    //fifo_length-1+ sizeof(ieee154_header_t));
    //          fifo_length-1+ sizeof(cc1101packet_header_t));
    //call PppIpv6.transmit(rxMsg+1,
    //          fifo_length -1 + sizeof(cc1101packet_header_t));
#endif

    // signal reception only if it has passed the CRC check
    if( crc == 0 ) {
      uint32_t time32;
      uint16_t time;
      atomic {
        time = call RadioAlarm.getNow();
        time32 = call LocalTime.get();
      }

      time -= sfdTime;
      time32 -= time;

      call PacketTimeStamp.set(rxMsg, time32);

#ifdef RADIO_DEBUG_MESSAGES
      if( call DiagMsg.record() )
      {
        call DiagMsg.str("r");
        call DiagMsg.uint16(call RadioAlarm.getNow() - (uint16_t)call PacketTimeStamp.timestamp(rxMsg) );
        call DiagMsg.uint16(call RadioAlarm.getNow());
        call DiagMsg.uint16(call PacketTimeStamp.isValid(rxMsg) ? call PacketTimeStamp.timestamp(rxMsg) : 0);
        call DiagMsg.int8(fifo_length);
        call DiagMsg.hex8s(getPayload(rxMsg), fifo_length);
        call DiagMsg.send();
      }
#endif
      rxMsg = signal RadioReceive.receive(rxMsg);


    }
    // TODO: Handle RX FIFO overflow
    if(status.state == CC1101_STATE_RXFIFO_OVERFLOW){
      // flush rx fifo
      flushRxFifo();
      enableReceiveGdo();
    }

  }


  /*----------------- IRQ -----------------*/

  // RX GDO0 (rising edge) or end of TX (falling edge)
  async event void Gdo0Capture.captured( uint16_t time )
  {
    uint8_t gdo0_val;
    call Gdo0Capture.disable();
    call Leds.led2On();
    RADIO_ASSERT( ! rxGdo0 ); // assert that there's no nesting
    RADIO_ASSERT( ! txEnd ); // assert that there's no nesting
  
    gdo0_val = call GDO0.get();

    if(state == STATE_RX_ON) {
      if(gdo0_val){
        capturedTime = time;
        call Gdo0Capture.captureFallingEdge();
        call Leds.led0On();
        call Leds.led0Off();
      }else{
        rxGdo0 = TRUE;
        call Leds.led0On();
        call Leds.led0Off();
        call Leds.led0On();
        call Leds.led0Off();
      }
    } else {
      // received capture interrupt in an invalid state
      RADIO_ASSERT(FALSE);
    }

#ifdef RADIO_DEBUG_IRQ
    if( call DiagMsg.record() )
    {
      if(rxGdo0)
        call DiagMsg.str("rxGdo0");
      if(txEnd)
        call DiagMsg.str("txEnd");
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.str("s=");
      call DiagMsg.uint8(state);
      if(call GDO0.get())
        call DiagMsg.str("GDO0");
      call DiagMsg.send();
    }
#endif

    // do the rest of the processing
    call Tasklet.schedule();
    call Leds.led2Off();
  }


  default tasklet_async event bool RadioReceive.header(message_t* msg)
  {
    return TRUE;
  }

  default tasklet_async event message_t* RadioReceive.receive(message_t* msg)
  {
    return msg;
  }

  /*----------------- TASKLET -----------------*/

  task void releaseSpi()
  {
    call SpiResource.release();
  }

  tasklet_async event void Tasklet.run()
  {
#ifdef RADIO_DEBUG_TASKLET
    if( call DiagMsg.record() )
    {
      call DiagMsg.str("tsk_str");
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.str("s=");
      call DiagMsg.uint8(state);
      call DiagMsg.str("c=");
      call DiagMsg.uint8(cmd);
      if(rxGdo0)
        call DiagMsg.str("rxGdo0");
      if(txEnd)
        call DiagMsg.str("txEnd");
      if(call GDO0.get())
        call DiagMsg.str("GDO0");

      call DiagMsg.send();
    }
#endif

    if( txEnd ) {
      // end of transmission
      if( isSpiAcquired() )
      {
        cc1101_status_t status;

        txEnd = FALSE;

        RADIO_ASSERT(state == STATE_TX_ON || state == STATE_BUSY_TX_2_RX_ON);
        RADIO_ASSERT(cmd == CMD_TRANSMIT);

        state = STATE_RX_ON;
        cmd = CMD_NONE;

        // a packet might have been received since the end of the transmission
        enableReceiveGdo();

        status = getStatus();

#if defined(RADIO_DEBUG_IRQ) && defined(RADIO_DEBUG_MESSAGES)
        if( call DiagMsg.record() )
        {
          call DiagMsg.str("txdone");
          call DiagMsg.uint16(call RadioAlarm.getNow());
          // TODO: captured time is not set for tx end
          //call DiagMsg.uint16(capturedTime - (uint16_t)call PacketTimeStamp.timestamp(txMsg));
          if(call GDO0.get())
            call DiagMsg.str("GDO0");

          call DiagMsg.send();
        }
#endif

        // check for tx underflow
        if ( status.state == CC1101_STATE_TXFIFO_UNDERFLOW ) {
          RADIO_ASSERT(FALSE);
          // flush tx fifo
          strobe(CC1101_SFTX);
          signal RadioSend.sendDone(FAIL);
        } else {
          signal RadioSend.sendDone(SUCCESS);
        }
      }
      else
        RADIO_ASSERT(FALSE);
    }

    if( rxGdo0 ) {
      // incoming packet
      if( isSpiAcquired() )
      {
        cc1101_status_t status;
        status = getStatus();
        rxGdo0 = FALSE;
        RADIO_ASSERT(cmd == CMD_NONE);

        if (status.state == CC1101_STATE_RXFIFO_OVERFLOW)
          cmd = CMD_RX_FLUSH;
        else{
          cmd = CMD_DOWNLOAD;
          state = STATE_RX_ON;
        }

      }
      else
        RADIO_ASSERT(FALSE);
    }


    if( cmd != CMD_NONE )
    {
      if( cmd == CMD_DOWNLOAD ) {
        RADIO_ASSERT(state == STATE_RX_ON);
        /*do{*/
          downloadMessage();
        /*}while(call GDO0.get());*/
      }
      else if (cmd == CMD_RX_FLUSH){
        flushRxFifo();
        state = STATE_RX_ON;
        enableReceiveGdo();
        cmd = CMD_NONE;
      }
      else if( CMD_TURNOFF <= cmd && cmd <= CMD_TURNON )
        changeState();
      else if( cmd == CMD_CHANNEL )
        changeChannel();

      if( cmd == CMD_SIGNAL_DONE )
      {
        cmd = CMD_NONE;
        signal RadioState.done();
      }
    }

    if( cmd == CMD_NONE && state == STATE_RX_ON && ! rxGdo0 && ! txEnd )
      signal RadioSend.ready();

    if( cmd == CMD_NONE )
      post releaseSpi();

#ifdef RADIO_DEBUG_TASKLET
    if( call DiagMsg.record() )
    {
      call DiagMsg.uint16(call RadioAlarm.getNow());
      call DiagMsg.str("tsk_end");
      call DiagMsg.str("s=");
      call DiagMsg.uint8(state);
      call DiagMsg.str("c=");
      call DiagMsg.uint8(cmd);
      if(rxGdo0)
        call DiagMsg.str("rxGdo0");
      if(txEnd)
        call DiagMsg.str("txEnd");
      if(call GDO0.get())
        call DiagMsg.str("GDO0");

      call DiagMsg.send();
    }
#endif
  }

  /*----------------- RadioPacket -----------------*/

  async command uint8_t RadioPacket.headerLength(message_t* msg)
  {
    return call Config.headerLength(msg) + sizeof(cc1101_header_t);
  }

  async command uint8_t RadioPacket.payloadLength(message_t* msg)
  {
    return getHeader(msg)->length;
  }

  async command void RadioPacket.setPayloadLength(message_t* msg, uint8_t length)
  {
    RADIO_ASSERT( 1 <= length && length <= 125 );
    RADIO_ASSERT( call RadioPacket.headerLength(msg) + length + call RadioPacket.metadataLength(msg) <= sizeof(message_t) );

    // exclude the length byte
    getHeader(msg)->length = length;
  }

  async command uint8_t RadioPacket.maxPayloadLength()
  {
    RADIO_ASSERT( call Config.maxPayloadLength() - sizeof(cc1101_header_t) <= 125 );

    return call Config.maxPayloadLength() - sizeof(cc1101_header_t);
  }

  async command uint8_t RadioPacket.metadataLength(message_t* msg)
  {
    return call Config.metadataLength(msg) + sizeof(cc1101_metadata_t);
  }

  async command void RadioPacket.clear(message_t* msg)
  {
    // all flags are automatically cleared
  }

  /*----------------- PacketTransmitPower -----------------*/

  async command bool PacketTransmitPower.isSet(message_t* msg)
  {
    return call TransmitPowerFlag.get(msg);
  }

  async command uint8_t PacketTransmitPower.get(message_t* msg)
  {
    return getMeta(msg)->power;
  }

  async command void PacketTransmitPower.clear(message_t* msg)
  {
    call TransmitPowerFlag.clear(msg);
  }

  async command void PacketTransmitPower.set(message_t* msg, uint8_t value)
  {
    call TransmitPowerFlag.set(msg);
    getMeta(msg)->power = value;
  }

  /*----------------- PacketRSSI -----------------*/

  async command bool PacketRSSI.isSet(message_t* msg)
  {
    return call RSSIFlag.get(msg);
  }

  async command uint8_t PacketRSSI.get(message_t* msg)
  {
    return getMeta(msg)->rssi;
  }

  async command void PacketRSSI.clear(message_t* msg)
  {
    call RSSIFlag.clear(msg);
  }

  async command void PacketRSSI.set(message_t* msg, uint8_t value)
  {
    // just to be safe if the user fails to clear the packet
    call TransmitPowerFlag.clear(msg);

    call RSSIFlag.set(msg);
    getMeta(msg)->rssi = value;
  }

  /*----------------- PacketTimeSyncOffset -----------------*/

  async command bool PacketTimeSyncOffset.isSet(message_t* msg)
  {
    return call TimeSyncFlag.get(msg);
  }

  async command uint8_t PacketTimeSyncOffset.get(message_t* msg)
  {
    return call RadioPacket.headerLength(msg) + call RadioPacket.payloadLength(msg) - sizeof(timesync_absolute_t);
  }

  async command void PacketTimeSyncOffset.clear(message_t* msg)
  {
    call TimeSyncFlag.clear(msg);
  }

  async command void PacketTimeSyncOffset.set(message_t* msg, uint8_t value)
  {
    // we do not store the value, the time sync field is always the last 4 bytes
    RADIO_ASSERT( call PacketTimeSyncOffset.get(msg) == value );

    call TimeSyncFlag.set(msg);
  }

  /*----------------- PacketLinkQuality -----------------*/

  async command bool PacketLinkQuality.isSet(message_t* msg)
  {
    return TRUE;
  }

  async command uint8_t PacketLinkQuality.get(message_t* msg)
  {
    return getMeta(msg)->lqi;
  }

  async command void PacketLinkQuality.clear(message_t* msg)
  {
  }

  async command void PacketLinkQuality.set(message_t* msg, uint8_t value)
  {
    getMeta(msg)->lqi = value;
  }

  /*----------------- LinkPacketMetadata -----------------*/

  async command bool LinkPacketMetadata.highChannelQuality(message_t* msg)
  {
    return call PacketLinkQuality.get(msg) > 105;
  }

  //for apps/PPPSniffer
#ifdef PPPSNIFFER
  event void Ipv6LcpAutomaton.transitionCompleted (LcpAutomatonState_e lcpstate) { }
  event void Ipv6LcpAutomaton.thisLayerUp () { }
  event void Ipv6LcpAutomaton.thisLayerDown () { }
  event void Ipv6LcpAutomaton.thisLayerStarted () { }
  event void Ipv6LcpAutomaton.thisLayerFinished () { }
  event void PppIpv6.linkUp () {}
  event void PppIpv6.linkDown () {}
  event error_t PppIpv6.receive (const uint8_t* message, unsigned int len) {
    return SUCCESS;
  }

  event void PppSplitControl.startDone (error_t error) { }
  event void PppSplitControl.stopDone (error_t error) { }
#endif
  }
