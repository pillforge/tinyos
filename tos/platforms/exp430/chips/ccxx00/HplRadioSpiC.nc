


configuration HplRadioSpiC {
  
  provides interface Resource;
  provides interface SpiByte;
  provides interface SpiPacket;
  
}

implementation {

  components new Msp430UsciSpiB0C() as SpiC;
  components HplRadioSpiP;
  Resource = SpiC;
  SpiByte = SpiC;
  SpiPacket = SpiC;
  SpiC.Msp430UsciConfigure -> HplRadioSpiP;
}

