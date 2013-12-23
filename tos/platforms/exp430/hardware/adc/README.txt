The MSP430 ADC12 implementation uses Timer_A as the timing source of the ADC for modes that require a timer (repeated
samples). The use of Timer_A conflicts with other parts of the system (system time, radio). On this platform, we want
ADC12 to use Timer_B. To do so, the files Msp430Adc12P.h, Msp430Adc12P.nc and Msp430Adc12ImplP.nc have been copied from
tos/chips/msp430/adc12. The files have been modified to user Timer_B, which is available on the MSP430F552x chips.
