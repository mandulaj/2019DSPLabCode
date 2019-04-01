/*
Program sinegen is a simple dual channel sine wave generator, which uses a look-up table for the coefficients.
In the example below, it produces frequencies of 6 kHz and 12 kHz.

Author: Patrick Gaydecki (modified by Haseeb Malik, Jakub Mandula, Tarig Mustafa, Kevin Golan)
Date  : 02.10.2017 (modified 5/3/19)
*/

.section L1_data_a;  // Linker places 12 kHz LUT starting at 0x11800000
.BYTE4/r32 lut1[]=0.0r,0.999r,0.0r,-0.999r;										//Hard coded 12 kHz values from LUT
.section L1_data_b;  // Linker places 6 kHz LUT starting at 0x11900000
.BYTE4/r32 lut2[]=0.0r,0.7071r,0.999r,0.7071r,0.0r,-0.7071r,-0.999r,-0.7071r;	//Hard coded 6 kHz values from LUT

.section program;
.global _main;
.align 4;
# include <defBF706.h>

_main:
call codec_configure;
call sport_configure;

P0=length(lut1)*4;							//Initializing a circular buffer to hold LUT for 12 kHz
I0=0x11800000;B0=I0;L0=P0;					//Populating buffer 1 with LUT data for 12 kHz
P0=length(lut2)*4;							//Initializing another circular buffer to hold LUT for 6 kHz
I1=0x11900000;B1=I1;L1=P0;					//Populating buffer 2 with LUT data for 6 kHz

get_audio:
wait_left:
// Wait for left data then dummy read
R0=[REG_SPORT0_CTL_B];
CC=BITTST(R0, 31);
if !CC jump wait_left;
R0=[REG_SPORT0_RXPRI_B];

R0=[I0++]; 									//Writing value from LUT for 12 kHz to left output and then moving to next value
R0=R0>>8;									//Bit-shifting to accomodate 24 Bit DAC in codec

[REG_SPORT0_TXPRI_A]=R0;
wait_right:
// Wait for right data then dummy read
R0=[REG_SPORT0_CTL_B];
CC=BITTST(R0, 31);
if !CC jump wait_right;
R0=[REG_SPORT0_RXPRI_B];

R0=[I1++];									//Writing value from LUT for 6 kHz to right output, then moving to next value
R0=R0>>8;									//Bit-shifting to accomodate 24 bit DAC in codec

[REG_SPORT0_TXPRI_A]=R0;
jump get_audio;
._main.end:

// Function codec_configure initialises the ADAU1761 codec. Refer to the control register
// descriptions, page 51 onwards of the ADAU1761 data sheet.
codec_configure:
[--SP] = RETS;                            // Push stack (only for nested calls)
R1=0x01(X); R0=0x4000(X); call TWI_write; // Enable master clock, disable PLL
R1=0x7f(X); R0=0x40f9(X); call TWI_write; // Enable all clocks
R1=0x03(X); R0=0x40fa(X); call TWI_write; // Enable all clocks
R1=0x01(X); R0=0x4015(X); call TWI_write; // Set serial port master mode
R1=0x13(X); R0=0x4019(X); call TWI_write; // Set ADC to on, both channels
R1=0x21(X); R0=0x401c(X); call TWI_write; // Enable left channel mixer
R1=0x41(X); R0=0x401e(X); call TWI_write; // Enable right channel mixer
R1=0x03(X); R0=0x4029(X); call TWI_write; // Turn on power, both channels
R1=0x03(X); R0=0x402a(X); call TWI_write; // Set both DACs on
R1=0x01(X); R0=0x40f2(X); call TWI_write; // DAC gets L, R input from serial port
R1=0x01(X); R0=0x40f3(X); call TWI_write; // ADC sends L, R input to serial port
R1=0x0b(X); R0=0x400a(X); call TWI_write; // Set left line-in gain to 0 dB
R1=0x0b(X); R0=0x400c(X); call TWI_write; // Set right line-in gain to 0 dB
R1=0xe7(X); R0=0x4023(X); call TWI_write; // Set left headphone volume to 0 dB
R1=0xe7(X); R0=0x4024(X); call TWI_write; // Set right headphone volume to 0 dB
R1=0x00(X); R0=0x4017(X); call TWI_write; // Set codec default sample rate, 48 kHz
NOP;
RETS = [SP++];                            // Pop stack (only for nested calls)
RTS;
codec_configure.end:

// Function sport_configure initialises the SPORT0. Refer to pages 26-59, 26-67,
// 26-75 and 26-76 of the ADSP-BF70x Blackfin+ Processor Hardware Reference manual.
sport_configure:
R0=0x3F0(X); [REG_PORTC_FER]=R0;          // Set up Port C in peripheral mode
R0=0x3F0(X); [REG_PORTC_FER_SET]=R0;      // Set up Port C in peripheral mode
R0=0x2001973; [REG_SPORT0_CTL_A]=R0;      // Set up SPORT0 (A) as TX to codec, 24 bits
R0=0x0400001; [REG_SPORT0_DIV_A]=R0;      // 64 bits per frame, clock divisor of 1
R0=0x1973(X); [REG_SPORT0_CTL_B]=R0;      // Set up SPORT0 (B) as RX from codec, 24 bits
R0=0x0400001; [REG_SPORT0_DIV_B]=R0;      // 64 bits per frame, clock divisor of 1
RTS;
sport_configure.end:

// Function TWI_write is a simple driver for the TWI. Refer to page 24-15 onwards
// of the ADSP-BF70x Blackfin+ Processor Hardware Reference manual.
TWI_write:
R3=R0 <<0x8; R0=R0 >>>0x8; R2=R3|R0;      // Reverse low order and high order bytes
R0=0x3232(X); [REG_TWI0_CLKDIV]=R0;       // Set duty cycle
R0=0x008c(X); [REG_TWI0_CTL]=R0;          // Set prescale and enable TWI
R0=0x0038(X); [REG_TWI0_MSTRADDR]=R0;     // Address of codec
[REG_TWI0_TXDATA16]=R2;                   // Address of register to set, LSB then MSB
R0=0x00c1(X); [REG_TWI0_MSTRCTL]=R0;      // Command to send three bytes and enable tx
[--SP] = RETS; call delay; RETS = [SP++]; // Delay
[REG_TWI0_TXDATA8]=R1;                    // Data to write
[--SP] = RETS; call delay; RETS = [SP++]; // Delay
R0=0x050; [REG_TWI0_ISTAT]=R0;            // Clear TXERV interrupt
[--SP] = RETS; call delay; RETS = [SP++]; // Delay
R0=0x010; [REG_TWI0_ISTAT]=R0;            // Clear MCOMP interrupt
rts;
TWI_write.end:

// Function delay introduces a delay to allow TWI communication
delay:
P0=0x8000;
loop LC0=P0;
NOP; NOP; NOP;
loop_end;
RTS;
delay.end:
