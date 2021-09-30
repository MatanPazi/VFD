/* 32 sample sine wave look up table.
   16Mhz CPU clk & clkI/O -> Prescaler of 2 -> 8Mhz
   1 OVF scenario will occur every 256 * 2 = 512 samples.
   1/8MHz * 512 = 64[us]
   going over the 15 sample look-up table will take 64us * 15 = 0.96[ms] -> 1,041.66[Hz]
   I'm aiming for 100[Hz] and below.
   So to achieve a 100[Hz] sine wave, I will need to increment in the sine table every
   1,041.66 / 100 = 10.41 OVF scenarios
   for 90[Hz] -> 1,041.66 / 90 = 11.57 OVF scenarios etc.
   Lower frequency resolution at higher frequencies since using byte data type (Rounded to a whole number)
   The amplitude will be attenuated by multiplying the sine table values by
   the desired frequency divided by the base frequency (60 OR 50 [Hz]) in order to maintain a constant V/Hz value.
   For 40 [Hz], the amplitude will be attenuated by 40/60 = 2/3.
   Due to low resolution of compare registers (only 1 byte), attenuating by dividing the sine table values
   will eventually lead to a distorted sine wave.
   Consider the following to overcome the distorted sine wave issue:
   Since each sine index is repeated at least 10 times (10 OVF scenarios for max freq of 100 [Hz])
   I can simply turn all the transistors off for some of the OVF scenarios, depending on the desired attenuation.
   For now, after testing, seems like even at an amplitude of 10%, the sine wave form still remains. So will continue using division for now.
   //
   REMINDER: Charge low side mosfets for at least 10[ms] at 50% duty cycle prior to normal operation (App note AN4043, P. 34)***************************
   //
   If operating a 3 phase motor, the 3 sine waves need to be 120 def apart
   If operating a single phase motor, we have 2 options depending on the wiring:
   1. With the capacitors removed, the phase shift between the main and auxiliary windings is achieved by connecting all three phases and the 3 sine waves can still be 120 degrees apart
   2. With the capacitor/s installed, A single sine wave will be used connecting only 2 phases, so the outputs need to be inverted. 2 sine waves 180 degrees phase shifted (Simply invert PWM logic for relevant timer).
   // Setting the LED display
   https://lastminuteengineers.com/tm1637-arduino-tutorial/
   //Atmega328 pin numbers:
   http://www.learningaboutelectronics.com/Articles/Atmega328-pinout.php
*/
#define _DISABLE_ARDUINO_TIMER0_INTERRUPT_HANDLER_  //These 2 lines were added to be able to compile. Also changed wiring.c file. Disables the previous overflow handles used for millis(), micros(), delay() etc.
#include <wiring.c>                                 //Reference: https://stackoverflow.com/questions/46573550/atmel-arduino-isrtimer0-ovf-vect-wont-compile-first-defined-in-vector/48779546
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <EEPROM.h>
#include <TM1637Display.h>

// Module connection pins (Digital Pins)
#define CLK1 3
#define DIO1 2
#define CLK2 13
#define DIO2 6
#define CURR_INPUT A0
#define POT_INPUT A2
//
#define BASE_FREQ 1041                 //Hz
#define PHASE_CONFIG_ADDRESS 0
#define V_F_CONFIG_ADDRESS 1
#define EEPROM_PRIOR_SAVE_ADDRESS 256
#define EEPROM_PRIOR_SAVE_VALUE 123
#define MILLI_A_RESOLUTION 125
#define LED_DELAY_MICRO 65000
#define MAX_V_F 20.0
#define MIN_V_F 2.0
//millis() etc. disabled(?) due to use of timers in PWM.
//Approx. time of loop ~
#define ONE_MS   16
#define HALF_SECOND   8000

const uint8_t ONE_PHASE[] = {
    SEG_B | SEG_C,                                    // 1
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,            // P
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G             // H
    };
const uint8_t THREE_PHASE[] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,            // 3
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,            // P
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G             // H
    };


uint16_t Curr_Value = 0;
uint32_t Timer = 0;
uint32_t Timer_Temp = 0;
uint32_t Timer_Temp1 = 0;
uint32_t Init_PWM_Counter = 0;
uint32_t Click_Timer = 0;
uint32_t Config_Set_Rdy_Timer = 0;
uint32_t Config_Change_Rdy_Timer = 0;
uint8_t  Click_Type = 0;                  //1: Short, 2: Long
bool  Phase_Config = 0;                   //0: 3 phase, 1: 1 phase
bool  Config_Displayed = 0;               //What configuration is seen at the moment. 0: Phase #, 1:V/f
bool  Config_Set_Rdy_Flag = 0;            //Flag to determine if V/f & phase configurations were set
bool  Config_Change_Rdy_Flag = 0;         //Flag to determine if V/f & phase configurations are ready to be set (If potentiometer switch is low long enough)
bool  PWM_Running = 0;                    //Indicates if the PWM is operating or not
bool  Config_Editable = 0;                //Is the configuration editable or not (Between 2 long clicks)
float V_f = MIN_V_F;                      //Voltage to frequency value of the motor
//Sine wave index variables
volatile uint8_t Sine_Index = 0;
volatile uint8_t Sine_Index_120 = 5;
volatile uint8_t Sine_Index_240 = 10;
//
volatile uint8_t Desired_Freq = 100;      //Desired freq [Hz]
volatile uint32_t OVF_Counter = 0;        //Increments every overflow
const unsigned char DT = 1;               //Dead time to prevent short-circuit betweem high & low mosfets
const unsigned char Sine_Len = 15;        //Sine table length
const unsigned char Sine[] = {0x7f,0xb5,0xe1,0xfa,0xfa,0xe1,0xb5,0x7f,0x48,0x1c,0x3,0x3,0x1c,0x48,0x7f};

TM1637Display Display1(CLK1, DIO1, LED_DELAY_MICRO);
TM1637Display Display2(CLK2, DIO2, LED_DELAY_MICRO);

void setup()
{
   Display1.setBrightness(0x02);
   Display2.setBrightness(0x02);
   Display1.clear();
   Display2.clear();
   PORTD = (1 << PORTD2);     //Activates internal pull up for PD2. Default pin state is input. Potentiometer switch
   PORTB = (1 << PORTB4);     //Activates internal pull up for PB4. Default pin state is input. Tactile switch
   DDRB = (1 << DDB0);       //Sets PB0 pin to output (Default is LOW). Commands the relay
   if (EEPROM.read(EEPROM_PRIOR_SAVE_ADDRESS) == EEPROM_PRIOR_SAVE_VALUE)
   {
     EEPROM.get(PHASE_CONFIG_ADDRESS, Phase_Config);
     EEPROM.get(V_F_CONFIG_ADDRESS, V_f);      
     Config_Set_Rdy_Flag = 1;
   }
}
void loop()
{   
   Curr_Value = (analogRead(CURR_INPUT) >> 6);     //A value of 1023 (5V) -> 8000[mA], so 1023 << 3. Gives a resolution of 8[mA] allegedly.
   Pot_Switch_State_Check();
   if (Config_Change_Rdy_Flag)
   {
      Button_Click();
   }
   //function: If PIND2==LOW for more than half second, on a rising edge of PINB4 (low and high states are long enough) due the following:
   //If the low state was longer than 1 sec, turn to config state
   //Otherwise, cycle between possible configurations
   Timer++;
   
   
  
}


void Display(bool PWM_Running, uint8_t Display_Num, bool Blink, uint16_t Delay)
{
   if (PWM_Running)
   {
      if (Display_Num == 1)
      {
         if (Freq > 999) Display1.showNumberDec(Freq, false, 4, 0);
         else if (Freq > 99) Display1.showNumberDec(Freq, false, 3, 0);
         else (Freq > 9) Display1.showNumberDec(Freq, false, 2, 0);         
      }
      else if (Display_Num == 2)
      {
         if (val > 999) Display2.showNumberDec(val, false, 4, 0);
         else if (val > 99) Display2.showNumberDec(val, false, 3, 0);
         else if (val > 9) Display2.showNumberDec(val, false, 2, 0);
         else Display2.showNumberDec(val, false, 1, 0);
      }   
   }
   else
   {
      while (Delay != 0) Delay--;
      if (Config_Displayed)
      {
         uint8_t V_f_Display = V_f * 10.0;
         if (Blink) Display1.clear();
         if (V_f_Display > 99) Display1.showNumberDec(V_f_Display, false, 3, 0);
         else (V_f_Display > 9) Display1.showNumberDec(V_f_Display, false, 2, 0);         
      }
      else
      {
      }

      
   }
}

void Button_Click()
{
   if (!PINB4)
   {
      if (Timer - Timer_Temp1  > 1) Click_Timer = 0;    //To make sure these increments are consecutive
      else Click_Timer++;
      Timer_Temp1 = Timer;
   }
   else
   {
      if (Click_Timer > 10 * ONE_MS && Click_Timer < HALF_SECOND) Click_Type = 1;
      else if (Click_Timer > 2 * HALF_SECOND) Click_Type = 2;         
      Click_Timer = 0;
   }   
   if (Click_Type == 2)
   {
      Config_Editable = !Config_Editable;    //Toggle
      Click_Type = 0;
   }
   else if (Click_Type == 1)
   {
      if (Config_Editable)
      {
         if (Config_Displayed)
         {
            V_f += 0.1;
            if (V_f > MAX_V_F) V_f = MIN_V_F;
         }
         else Phase_Config = !Phase_Config;
      }
      else
      {
         Config_Displayed = !Config_Displayed;
      } 
      Click_Type = 0;
   }
   Display(bool PWM_Running, uint8_t Display_Num, bool Blink)
}


void Pot_Switch_State_Check()
{
   if (PIND2)     //Potnetiometer switch ON
   {      
      if (Config_Set_Rdy_Flag && (!PWM_Running))
      {
         if (Timer - Timer_Temp  > 1) Config_Set_Rdy_Timer = 0;    //To make sure these increments are consecutive
         else  Config_Set_Rdy_Timer++;      
         Timer_Temp = Timer;
      }
      Config_Change_Rdy_Timer = 0;
      Config_Change_Rdy_Flag = 0;
   }
   else           //Potentiometer switch OFF
   {
      if (Timer - Timer_Temp  > 1) Config_Change_Rdy_Timer = 0;    //To make sure these increments are consecutive
      else  Config_Change_Rdy_Timer++;      
      Timer_Temp = Timer;
      Config_Set_Rdy_Timer = 0;
   }
   if (Config_Set_Rdy_Timer > HALF_SECOND)
   {
      Pwm_Config();      
      Config_Set_Rdy_Timer = 0;
   }   
   if (Config_Change_Rdy_Timer > HALF_SECOND)
   {
      Pwm_Disable();
      Config_Change_Rdy_Flag = 1;
      Config_Set_Rdy_Timer = 0;
   }  
}


void Pwm_Disable()
{
   PWM_Running = 0;
   cli();
   TCCR0A = 0;
   TCCR0B = 0;
   TCCR1A = 0;
   TCCR1B = 0;
   TCCR2A = 0;
   TCCR2B = 0;
   sei();         
}

void Pwm_Config()
{
   //Need to make sure the pins are LOW prior to and after setting them to outputs so don't accidentally cause short in IPM.
   PWM_Running = 1;
   DDRD = (1 << DDRD6) | (1 << DDRD5) | (1 << DDRD3); //Sets the OC0A, OC0B and OC2B pins to outputs
   DDRB = (1 << DDRB3) | (1 << DDRB2) | (1 << DDRB1); //Sets the OC2A, OC1B and OC1A pins to outputs
   if (Phase_Config == 1)
   {
      cli();                      //Disable interrupts
      CLKPR = (1 << CLKPCE);      //Enable change of the clock prescaler
      CLKPR = (1 << CLKPS0);      //Set system clock prescaler to 2
      //Timer 0
      TCNT0 = 0;                  //Zero counter of timer 0
      TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
      TCCR0B = (1 << CS00);       //No prescaler
      TIMSK0 = (1 << TOIE0);      //Timer/Counter0 Overflow Interrupt Enable
      OCR0A = 0;   //Sign determined by set or clear at count-up
      OCR0B = 127;   //Sign determined by set or clear at count-up
      // Timer 1
      TCNT1 = 0;                  //Zero counter of timer 0
      TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10); // Clear OC1A and set OC1B counting up. Waveform mode 1 (Table 14-8)
      TCCR1B = (1 << CS10);       //No prescaler
      OCR1A = 0;   //Sign determined by set or clear at count-up
      OCR1B = 127;   //Sign determined by set or clear at count-up
      // Timer 2
      TCNT2 = 0;                  //Zero counter of timer 0
      TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
      TCCR2B = (1 << CS20);       //No prescaler
      OCR2A = 0;   //Sign determined by set or clear at count-up
      OCR2B = 127;   //Sign determined by set or clear at count-up
      sei();
   }
   else if (Phase_Config == 2)      //Change relevent to 1 phase*************************
   {
      cli();                      //Disable interrupts
      CLKPR = (1 << CLKPCE);      //Enable change of the clock prescaler
      CLKPR = (1 << CLKPS0);      //Set system clock prescaler to 2
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
   }

}



ISR (TIMER0_OVF_vect)
{   
   if (Init_PWM_Counter < 3 * ONE_MS) Init_PWM_Counter++;
   else
   {
      OVF_Counter++;   
      else if (OVF_Counter >= (BASE_FREQ / Desired_Freq))
      {
         if (Sine_Index == Sine_Len) Sine_Index = 0;
         if (Sine_Index_120 == Sine_Len) Sine_Index_120 = 0;
         if (Sine_Index_240 == Sine_Len) Sine_Index_240 = 0;    
         //  
         if ((Sine[Sine_Index] - DT) < 0) OCR0A = 0;
         else  OCR0A = Sine[Sine_Index] - DT;  //Sign determined by set or clear at count-up
         OCR0B = Sine[Sine_Index] + 2*DT;  //Sign determined by set or clear at count-up
         //
         OCR1A = Sine[Sine_Index_120] - DT;  //Sign determined by set or clear at count-up
         OCR1B = Sine[Sine_Index_120] + 2*DT;  //Sign determined by set or clear at count-up
         OCR2A = Sine[Sine_Index_240] - DT;  //Sign determined by set or clear at count-up
         OCR2B = Sine[Sine_Index_240] + 2*DT;  //Sign determined by set or clear at count-up
         OVF_Counter = 0;
         Sine_Index++;
         Sine_Index_120++;
         Sine_Index_240++;
      }
   }
}
