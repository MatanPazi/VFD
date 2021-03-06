/* 15 sample sine wave look up table.
   16Mhz CPU clk & clkI/O -> Prescaler of 2 -> 8Mhz
   1 OVF scenario will occur every 256 * 2 = 512 samples.
   1/8MHz * 512 = 64[us]
   going over the 15 sample look-up table will take 64us * 15 = 0.96[ms] -> 1,041.66[Hz]
   I'm aiming for ~100[Hz] and below.
   So to achieve a 100[Hz] sine wave, I will need to increment the sine table every
   1,041.66 / 100 = 10.41 OVF scenarios
   for 90[Hz] -> 1,041.66 / 90 = 11.57 OVF scenarios etc.
   Lower resolution at higher frequencies (Rounded to a whole number)
   The amplitude will be derived from the desired frequency and the V/f value (230[VAC]/60[Hz]=3.8333), with a minimum and maximum value.
   For example, w/ a desired freq of 30[Hz], the amplitude will be (3.8333 * 30) / 230 = 0.5
   Due to low resolution of compare registers (only 1 byte), attenuating by dividing the sine table values
   will eventually lead to a distorted sine wave.
   Consider the following to overcome the distorted sine wave issue:
   Since each sine index is repeated at least 10 times (10 OVF scenarios for max freq of 100 [Hz])
   I can simply turn all the transistors off for some of the OVF scenarios, depending on the desired attenuation.
   For now, after testing, seems like even at an amplitude of 10%, the sine wave form still remains. So will continue using division for now.
   If operating a 3 phase motor, the 3 sine waves need to be 120 def apart
   If operating a single phase motor, we have 2 options depending on the wiring: (Source: https://youtu.be/FyeJJ3Tp6iY?t=58)
   1. With the capacitors removed, the phase shift between the main and auxiliary windings is achieved by connecting all three phases and the 3 sine waves can still be 120 degrees apart.
      Won't be 90 deg phase shifted, but at 120 degrees, will still be reasonable.
   2. With the capacitor/s installed, the outputs will be inverted. 2 sine waves 180 degrees phase shifted (Simply invert PWM logic for relevant timer).
   // Setting the LED display
   https://lastminuteengineers.com/tm1637-arduino-tutorial/
   //Atmega328 pin numbers:
   http://www.learningaboutelectronics.com/Articles/Atmega328-pinout.php
   Dead-time:
   Datasheet recommends 1 [us] for each input signal. However, when looking at the datasheet, the turn-on time is 290[ns] and turn off time is 515[ns], so I need a minimum of
   515-290 = 225[ns] between PWM signals. Each count takes 125[ns], so I'll take 125*4 = 500[ns] dead time.
   //
   //To-do *****************************************************************************
   Save last phase configuration to EEPROM. ********************************************
   *************************************************************************************
*/
#define _DISABLE_ARDUINO_TIMER0_INTERRUPT_HANDLER_  //These 2 lines were added to be able to compile. Also changed wiring.c file. Disables the previous overflow handles used for millis(), micros(), delay() etc.
#include <wiring.c>                                 //Reference: https://stackoverflow.com/questions/46573550/atmel-arduino-isrtimer0-ovf-vect-wont-compile-first-defined-in-vector/48779546
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <EEPROM.h>
#include <TM1637Display.h>

// Module connection pins (Digital Pins)
#define CLK1 12
#define DIO1 2
#define CLK2 7
#define DIO2 4
#define CURR_INPUT A0
#define POT_INPUT A2
//
#define THREE_PH 0
#define ONE_PH 1
#define PWM_RUNNING 2
#define PWM_NOT_RUNNING 1
#define PWM_NOT_SET 0
//millis(), delay() don't work as expected due to use of timers in PWM.
//Approx. time of loop ~
#define SHORT_CLICK     3
#define LONG_CLICK      25
#define POT_SWITCH_SAMPLES  2
#define TEN_MS_OVF      160
#define SHORT_WAIT      500
#define DISPLAY_BLINK   25

const uint8_t ONE_PHASE[] = {
    SEG_B | SEG_C,                                    // 1
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,            // P
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,            // H
    0,                                                // space
    };
const uint8_t THREE_PHASE[] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,            // 3
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,            // P
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,            // H
    0,                                                // space
    };
const uint8_t SPACE[] = {
    0,                                                // space
    }; 

// Generated using: https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
const uint8_t Sine[] = {0x7f,0xb5,0xe1,0xfa,0xfa,0xe1,0xb5,0x7f,0x48,0x1c,0x3,0x3,0x1c,0x48,0x7f};
const uint8_t Sine_Len = 15;              //Sine table length
const uint8_t Min_Freq = 20;              //Minimal demanded sine wave frequency
const uint8_t Max_Freq = 100;             //Maximal demanded sine wave frequency
const uint8_t DT = 1;                     //Dead time to prevent short-circuit betweem high & low mosfets
const uint16_t Base_Freq = 1041;          //[Hz] Maximal frequency if the sine wave array index is incremented every OVF occurance 
const float Min_Amp = 0.1;                //Minimal allowed sine wave amplitude
const float Max_Amp = 1.0;                //Maximal allowed sine wave amplitude
const float V_f = 3.8333;                 //V/f value. ~230[VAC] w/ 60[Hz]
const float VBus = 230.0;                 //AC voltage [VAC]
bool  Phase_Config = 0;                   //0: 3 phase, 1: 1 phase
bool  Config_Editable = 0;                //Is the configuration editable or not (Between 2 long clicks). 0: No, 1: Yes
uint8_t  Click_Type = 0;                  //1: Short, 2: Long
uint8_t  PWM_Running = PWM_NOT_SET;       //Indicates if the PWM is operating or not. 2 is running, 1 is not, initialized to 0 to indicate not yet set.
uint16_t Curr_Value = 0;                  //Current value measured in [mA]
uint32_t Timer = 0;                       //Timer, increments every loop
uint32_t Click_Timer = 0;                 //Increments while button is clicked
uint32_t Pot_Switch_Off_Timer = 0;        //Increments while potentiometer switch is OFF
uint32_t Pot_Switch_On_Timer = 0;         //Increments while potentiometer switch is ON
uint32_t Display_Timer = 0;               //Used for delay of the blinking display (When configurable)
uint32_t Timer_Temp = 0;                  //Used to make sure of consecutive executions
uint32_t Timer_Temp1 = 0;                 //Used to make sure of consecutive executions
uint32_t Init_PWM_Counter = 0;            //Used for charging the bootstrap capacitors, source below
float Amp = Min_Amp;                      //Sine wave amplitude
volatile uint8_t Desired_Freq = Min_Freq; //Desired sine wave freq [Hz]
volatile uint8_t Sine_Index = 0;          //3 sine wave indices are used to allow for phase shifted sine waves.
volatile uint8_t Sine_Index_120 = Sine_Len / 3;
volatile uint8_t Sine_Index_240 = (Sine_Len * 2) / 3;       //Sine_Len must be lower than 128, otherwise, change eq. 
volatile uint32_t OVF_Counter = 0;        //Increments every Timer0 overflow

TM1637Display Display1(CLK1, DIO1);
TM1637Display Display2(CLK2, DIO2);

void setup()
{
  cli();                                      //Disable interrupts
  CLKPR = (1 << CLKPCE);                      //Enable change of the clock prescaler
  CLKPR = (1 << CLKPS0);                      //Set system clock prescaler to 2. Beforehand DT had to be increased to a large value due to IPM, and at low amplitudes distorted sine wave. When reducing the prescaler, this allows for the DT value to be small.
  sei();
  //Serial.begin(19200);                        //Set the baud rate to double that which is set in "Serial Monitor" due to the prescaler being 2 instead of 1.
  Display1.setBrightness(0x02);
  Display2.setBrightness(0x02);
  Display1.clear();
  Display2.clear();
  PORTC = (1 << PORTC3) | (1 << PORTC4);      //Activates internal pull up for PC3 (ADC3) and PC4 (ADC4). Default pin state is input. Potentiometer switch and button respectively   
  DDRB = (1 << DDB0);                         //Sets PB0 pin to output (Default is LOW). Commands the relay
  Pwm_Config();
}
void loop()
{   
   /*
   //Calculate variables
   Curr_Value = ((analogRead(CURR_INPUT) >> 4) << 7);        //A value of 1023 (5V) -> 8000[mA]. First divide by 16 then multiply by 128. Gives resolution of 128[mA]
   Desired_Freq = (analogRead(POT_INPUT) >> 3);              //A value of 1023 (5V) -> 128[Hz]. Divide result by 8 to get value in Hz. Gives resolution of 1[Hz]
   if (PWM_Running != PWM_RUNNING) Wait_A_Bit(SHORT_WAIT);   //Using this function since delay() doesn't seem to work correctly when ISR is activated. Not needed to delay if interrupts are enabled.
   if (Desired_Freq < Min_Freq) Desired_Freq = Min_Freq;
   else if (Desired_Freq > Max_Freq) Desired_Freq = Max_Freq;
   Amp = (float(Desired_Freq) * V_f) / VBus;                 //Calculating the sine wave amplitude based on the desired frequency and the V/f value.
   if (Amp < Min_Amp) Amp = Min_Amp;      
   else if (Amp > Max_Amp) Amp = Max_Amp;
   //Run functions
   Pot_Switch_State_Check();
   if (PWM_Running != PWM_RUNNING) Button_Click();
   if (PWM_Running != PWM_NOT_SET) Display(PWM_Running, Config_Editable);
   Timer++;    
   */
   Amp = (float(Desired_Freq) * V_f) / VBus;                 //Calculating the sine wave amplitude based on the desired frequency and the V/f value.
   //if (PWM_Running != PWM_RUNNING) Pwm_Config();
}



/* Pot_Switch_State_Check(): Detects the state of the potentiometer switch
   If it is ON a sufficiently long time (Avoid false triggers) PWM starts running with defined configuration and displays the measured current and the desired frequency.
   Otherwise, if it is OFF a sufficiently long time (Avoid false triggers) PWM is disabled and the configuration
   is displayed (1PH/3PH) and allowed to be altered using the button.
*/
void Pot_Switch_State_Check()    
{
   if (!((PINC >> PINC3) & 1))     //Potentiometer switch ON (when ON, PINC3 is pulled LOW).
   {      
      if (PWM_Running != PWM_RUNNING)    //If PWM isn't running (If it was already running, no need to update anything).
      {
        if (Timer - Timer_Temp  > 1) Pot_Switch_On_Timer = 0;    //To make sure these increments are consecutive
        else  Pot_Switch_On_Timer++;      
        Timer_Temp = Timer;
        Pot_Switch_Off_Timer = 0;
        if (Pot_Switch_On_Timer > POT_SWITCH_SAMPLES)      //If potentiometer switch was ON a sufficient amount of time
        {
          Pwm_Config();      
          Pot_Switch_On_Timer = 0;
          Pot_Switch_Off_Timer = 0;      
          Display1.clear();
          Display2.clear();
        }         
      }
   }
   else           //Potentiometer switch OFF
   {
      if (PWM_Running != PWM_NOT_RUNNING)     //If PWM is running (If it was already not running, no need to update anything).
      {
        if (Timer - Timer_Temp  > 1) Pot_Switch_Off_Timer = 0;      //To make sure these increments are consecutive
        else  Pot_Switch_Off_Timer++;      
        Timer_Temp = Timer;
        Pot_Switch_On_Timer = 0;
        if (Pot_Switch_Off_Timer > POT_SWITCH_SAMPLES)     //If potentiometer switch was OFF a sufficient amount of time
        {
          Pwm_Disable();
          Pot_Switch_On_Timer = 0;
          Pot_Switch_Off_Timer = 0;
          Display1.clear();
          Display2.clear();
        } 
      }
   }    
}


/* Button_Click(): Detects a button click. Determines if it was a short click:
   switch between configuration if multiple are available or alter the configuration.
   Or a long click: Enables and disables the ability to alter the configuration.
*/
void Button_Click()
{
  if (!((PINC >> PINC4) & 1))     //Button being pushed (when button is pushed, PINC4 is pulled LOW).
  {
    if (Timer - Timer_Temp1  > 1) Click_Timer = 0;    //To make sure these increments are consecutive
    else Click_Timer++;
    Timer_Temp1 = Timer;
    if (Click_Timer == LONG_CLICK)
    {
      Config_Editable = !Config_Editable;    //Toggle
    }
  } 
  else if (Click_Timer > SHORT_CLICK && Click_Timer < LONG_CLICK)
  {
    if (Config_Editable)
    {
       Phase_Config = !Phase_Config;        //Toggle
    }
    Click_Timer = 0;
  }
  else Click_Timer = 0;
}


/* Display(): Chooses what and how to display on the 2 available LED displays.
   2 inputs:
      1. PWM_Running, whether or not the PWM is active or not.
      2. Blink, whether to cause the display to blink or not. signifies if the configuration is editable or not.
         Blinking: Editable, Not blinking: Not editable
*/
void Display(uint8_t PWM_Running, bool Blink)
{
  if (PWM_Running == PWM_RUNNING)
  {
    //Display 1, displays desired frequency in [Hz]
    if (Desired_Freq > 99) Display1.showNumberDec(Desired_Freq, false, 3, 1);
    else 
    {
      Display1.setSegments(SPACE, 1, 1);
      Display1.showNumberDec(Desired_Freq, false, 2, 2);         
    }
    //Display 2, displays measured current in [mA]
    if (Curr_Value > 999) Display2.showNumberDec(Curr_Value, false, 4, 0);
    else 
    {
      Display2.setSegments(SPACE, 1, 0);
      Display2.showNumberDec(Curr_Value, false, 3, 1);         
    }
  }
  else
  {
    Display_Timer++;
    if (Blink && (Display_Timer == DISPLAY_BLINK))
    {
      Display1.clear();
      Display2.clear();
    }
    else if (Display_Timer > (2*DISPLAY_BLINK))
    {
      if (Phase_Config) Display1.setSegments(ONE_PHASE);
      else Display1.setSegments(THREE_PHASE); 
      Display_Timer = 0;   
    }
  }
}



/* Wait_A_Bit(): Delays for a specified amount of executions
*/
void Wait_A_Bit(uint32_t Executions_To_Wait)
{
  volatile uint32_t Timer_Temp2 = 0;
  while (Timer_Temp2 < Executions_To_Wait)
  {
    Timer_Temp2 = Timer_Temp2 + 1;
  }
  Timer_Temp2 = 0;
}




/* Pwm_Disable(): Disables the PWM and zeros some parameters
   PWM_Running indicates the PWM is disabled
   Init_PWM_Counter is used to charge the bootstrap capacitors everytime the PWM is enabled
*/
void Pwm_Disable()
{
   PWM_Running = PWM_NOT_RUNNING;
   cli();
   Init_PWM_Counter = 0;
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
   //***Check in scope. Need to make sure the pins are LOW prior to and after setting them to outputs so don't accidentally cause short in IPM.
   PWM_Running = PWM_RUNNING;
   if (Phase_Config == THREE_PH)
   {
      DDRD = (1 << DDD6) | (1 << DDD5) | (1 << DDD3); //Sets the OC0A, OC0B and OC2B pins to outputs
      DDRB = (1 << DDB3) | (1 << DDB2) | (1 << DDB1); //Sets the OC2A, OC1B and OC1A pins to outputs
      cli();                      //Disable interrupts
      //Synchronising all 3 timers 1st segment. Source: http://www.openmusiclabs.com/learning/digital/synchronizing-timers/index.html
      GTCCR = (1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC); //Halt all timers
      //Timer 0
      TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
      TCCR0B = (1 << CS00);       //No prescaler
      TIMSK0 = (1 << TOIE0);      //Timer/Counter0 Overflow Interrupt Enable
      OCR0A = 0;     //Sign determined by set or clear at count-up. High-side IGBT OFF.
      OCR0B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
      // Timer 1
      TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10); // Clear OC1A and set OC1B counting up. Waveform mode 1 (Table 14-8)
      TCCR1B = (1 << CS10);       //No prescaler
      OCR1A = 0;     //Sign determined by set or clear at count-up. High-side IGBT OFF.
      OCR1B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
      // Timer 2
      TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
      TCCR2B = (1 << CS20);       //No prescaler
      OCR2A = 0;     //Sign determined by set or clear at count-up. High-side IGBT OFF.
      OCR2B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
      //Synchronising all 3 timers 2nd segment. Source: http://www.openmusiclabs.com/learning/digital/synchronizing-timers/index.html
      TCNT0 = 0;     //Set timer0 to 0
      TCNT1H = 0;    //Set timer1 high byte to 0
      TCNT1L = 0;    //Set timer1 low byte to 0
      TCNT2 = 0;     //Set timer2 to 0      
      GTCCR = 0;     //Release all timers
      sei();      
   }
   else if (Phase_Config == ONE_PH)
   {
      DDRD = (1 << DDD6) | (1 << DDD5) | (1 << DDD3); //Sets the OC0A, OC0B and OC2B pins to outputs
      DDRB = (1 << DDB3);                             //Sets the OC2A pin to output
      cli();                      //Disable interrupts
      //Synchronising all 3 timers 1st segment. Source: http://www.openmusiclabs.com/learning/digital/synchronizing-timers/index.html
      GTCCR = (1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC); //Halt all timers
      //Timer 0
      TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
      TCCR0B = (1 << CS00);       //No prescaler
      TIMSK0 = (1 << TOIE0);      //Timer/Counter0 Overflow Interrupt Enable
      OCR0A = 0;     //Sign determined by set or clear at count-up. High-side IGBT OFF.
      OCR0B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
      // Timer 2
      TCCR2A = (1 << COM2A1) | (1 << COM2A0) | (1 << COM2B1) | (1 << WGM20); // Set OC2A and clear OC2B counting up. Waveform mode 1 (Table 14-8)
      TCCR2B = (1 << CS20);       //No prescaler
      OCR2A = 255;   //Sign determined by set or clear at count-up. High-side IGBT OFF.
      OCR2B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
      //Synchronising all 3 timers 2nd segment. Source: http://www.openmusiclabs.com/learning/digital/synchronizing-timers/index.html
      TCNT0 = 0;     //Set timer0 to 0
      TCNT1H = 0;    //Set timer1 high byte to 0
      TCNT1L = 0;    //Set timer1 low byte to 0
      TCNT2 = 0;     //Set timer2 to 0      
      GTCCR = 0;     //Release all timers
      sei();
   }
}



ISR (TIMER0_OVF_vect)
{   
   if (Init_PWM_Counter < TEN_MS_OVF) Init_PWM_Counter++;           //Charge bootstrap cap as recommended by "Initial bootstrap capacitor charging" section in AN4840 by ST
   else
   {
      OVF_Counter++;   
      if (OVF_Counter >= (Base_Freq / Desired_Freq))
      {
         if (Sine_Index == Sine_Len) Sine_Index = 0;
         if (Sine_Index_120 == Sine_Len) Sine_Index_120 = 0;
         if (Sine_Index_240 == Sine_Len) Sine_Index_240 = 0;    
         //  
         if ((Amp * Sine[Sine_Index] - 2*DT) < 0) OCR0A = 0;
         else  OCR0A = Amp * Sine[Sine_Index] - 2*DT;
         OCR0B = Amp * Sine[Sine_Index] + 2*DT;                // if/else not needed for +DT because Amp <= 1.0
         //
         if (Phase_Config == ONE_PH)
         {
            if ((Amp * Sine[Sine_Index] - 2*DT) < 0) OCR2B = 0;
            else  OCR2B = Amp * Sine[Sine_Index] - 2*DT;
            OCR2A = Amp * Sine[Sine_Index] + 2*DT;  
         }
         else if (Phase_Config == THREE_PH)
         {
            if ((Amp * Sine[Sine_Index_120] - 2*DT) < 0) OCR1A = 0;
            else OCR1A = Amp * Sine[Sine_Index_120] - 2*DT;
            OCR1B = Amp * Sine[Sine_Index_120] + 2*DT;   
            if ((Amp * Sine[Sine_Index_240] - 2*DT) < 0) OCR2A = 0;
            else OCR2A = Amp * Sine[Sine_Index_240] - 2*DT;
            OCR2B = Amp * Sine[Sine_Index_240] + 2*DT;   
         }
         OVF_Counter = 0;
         Sine_Index++;
         Sine_Index_120++;
         Sine_Index_240++;
      }
   }
}
