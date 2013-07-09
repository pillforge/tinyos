module HplRadioSpiP {
  provides interface Msp430UsciConfigure;
}
implementation {
  const msp430_usci_config_t msp430_usci_spi_cc1101 = {
    /* Inactive high MSB-first 8-bit 3-pin master driven by SMCLK */

       ctl0 : UCCKPH | UCMSB | UCMST | UCSYNC,
       ctl1 : UCSSEL__SMCLK,
       br0  : 2,			/* 2x Prescale, 1*2^19 (512 KiHz) */
       br1  : 0,
       mctl : 0,
       i2coa: 0
  };
  async command const msp430_usci_config_t*
    Msp430UsciConfigure.getConfiguration () {
      return &msp430_usci_spi_cc1101;
  }
}
