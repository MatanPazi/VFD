/* 32 sample sine wave look up table.
   16Mhz CPU clk, & clkI/O
   1 OVF scenario will occur every 256 * 2 = 512 samples.
   1/16MHz * 512 = 32[us]
   going over the 32 sample look-up table will take 32us * 32 = 1.024[ms] -> 976.5625[Hz]
   I'm aiming for 100[Hz] and below.
   So to achieve a 100[Hz] sine wave, I will need to increment in the sine table every
   976.5625 / 100 = 9.765625 OVF scenarios
   for 90[Hz] -> 976.5625 / 90 = 10.850694 OVF scenarios etc.
   Lower frequency resolution at higher frequencies since using byte data type (Rounded to a whole number)
   The amplitude will be attenuated by multiplying the sine table values by
   the desired frequency divided by the base frequency (60 OR 50 [Hz]) in order to maintain a constant V/Hz value.
   For 40 [Hz], the amplitude will be attenuated by 40/60 = 2/3.
   Due to low resolution of compare registers (only 1 byte), attenuating by dividing the sine table values
   will eventually lead to a distorted sine wave.
   Since each sine index is repeated at least 10 times (10 OVF scenarios for max freq of 100 [Hz])
   I can simply turn all the transistors off for some of the OVF scenarios, depending on the desired attenuation.
   //
   REMINDER: Charge low side mosfets for at least 10[ms] at 50% duty cycle prior to normal operation (App note AN4043, P. 34)***************************
*/
#define _DISABLE_ARDUINO_TIMER0_INTERRUPT_HANDLER_  //These 2 lines were added to be able to compile. Also changed wiring.c file. Disables the previous overflow handles used for millis(), micros(), delay() etc.
#include <wiring.c>                                 //Reference: https://stackoverflow.com/questions/46573550/atmel-arduino-isrtimer0-ovf-vect-wont-compile-first-defined-in-vector/48779546
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define BASE_FREQ 1552       //Hz

volatile uint8_t count = 0;
volatile uint8_t count120 = 7;
volatile uint8_t count240 = 14;
volatile uint8_t Desired_Freq = 1;
volatile uint32_t Freq_Counter = 0;
volatile float   Amp = 1.0;
const unsigned char DT = 1; //Dead time to prevent short-circuit betweem high & low mosfets
const unsigned char Sine_Len = 21;
const unsigned char Sine[] = {0x7f,0xb2,0xdd,0xf7,0xfc,0xec,0xc9,0x99,0x64,0x34,0x11,0x1,0x6,0x20,0x4b,0x7f};

void setup()
{
  DDRD = (1 << PORTD6) | (1 << PORTD5) | (1 << PORTD3); //Sets the OC0A, OC0B and OC2B pins to outputs
  DDRB = (1 << PORTB3) | (1 << PORTB2) | (1 << PORTB1); //Sets the OC2A, OC1B and OC1A pins to outputs
  cli();                      //Disable interrupts
  CLKPR = (1 << CLKPS0)       //System clock prescaler of 2
  //Timer 0
  TCNT0 = 0;                  //Zero counter of timer 0
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
  TCCR0B = (1 << CS00);       //No prescaler
  TIMSK0 = (1 << TOIE0);      //Timer/Counter0 Overflow Interrupt Enable
  OCR0A = Sine[0] - DT;   //Sign determined by set or clear at count-up
  OCR0B = Sine[0] + 2*DT;   //Sign determined by set or clear at count-up
  // Timer 1
  TCNT1 = 0;                  //Zero counter of timer 0
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10); // Clear OC1A and set OC1B counting up. Waveform mode 1 (Table 14-8)
  TCCR1B = (1 << CS10);       //No prescaler
  OCR1A = Sine[count120] - DT;   //Sign determined by set or clear at count-up
  OCR1B = Sine[count120] + 2*DT;   //Sign determined by set or clear at count-up
  // Timer 2
  TCNT2 = 0;                  //Zero counter of timer 0
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
  TCCR2B = (1 << CS20);       //No prescaler
  OCR2A = Sine[count240] - DT;   //Sign determined by set or clear at count-up
  OCR2B = Sine[count240] + 2*DT;   //Sign determined by set or clear at count-up
  sei();
  while (1)  {}
}



ISR (TIMER0_OVF_vect)
{
  Desired_Freq = 100;
  Freq_Counter++;
  if (Freq_Counter >= (BASE_FREQ / Desired_Freq))
  {
    count++;
    count120++;
    count240++;
    if (count == Sine_Len) count = 0;
    if (count120 == Sine_Len) count120 = 0;
    if (count240 == Sine_Len) count240 = 0;    
    //  
    if ((0.1*Sine[count] - DT) < 0) OCR0A = 0;
    else  OCR0A = 0.1*Sine[count] - DT;  //Sign determined by set or clear at count-up
    OCR0B = 0.1*Sine[count] + 2*DT;  //Sign determined by set or clear at count-up
    //
    OCR1A = Sine[count120] - DT;  //Sign determined by set or clear at count-up
    OCR1B = Sine[count120] + 2*DT;  //Sign determined by set or clear at count-up
    OCR2A = Sine[count240] - DT;  //Sign determined by set or clear at count-up
    OCR2B = Sine[count240] + 2*DT;  //Sign determined by set or clear at count-up
    Freq_Counter = 0;
  }
}
