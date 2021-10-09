#include <xc.h>
#include "config.h"
#include <stdint.h>
#include "pic16f886.h"

#define _XTAL_FREQ 4000000
#define TMR2PRESCALE 4

long PWM_freq = 5000;
uint16_t init_pulse_width = 250; //initial value (0-1023)


void PWM_Initialize(long PWM_freq, uint16_t init_pulse_width );
void set_PWM_duty_cycle(uint16_t duty);

void ADC_Initialize();
uint16_t ADC_Read(uint8_t channel);

void main(void) {
  uint16_t adc_value;
  uint16_t pulse_width = init_pulse_width;
  
  ANSEL = 0x00;
  ANSELH = 0x00;
  
  TRISB = 0x00;
  TRISC = 0x00;
  
  ADC_Initialize(); //Initializes ADC Module
  PWM_Initialize(PWM_freq, init_pulse_width );  //This sets the PWM frequency of PWM1

  
  while (1){
    adc_value = ADC_Read(0);
    if (adc_value != pulse_width){
        pulse_width = adc_value;
        set_PWM_duty_cycle(pulse_width);
    }
    __delay_ms(50);
  }

}


void PWM_Initialize(long PWM_freq, uint16_t init_pulse_width ){
    
    PR2 = (unsigned)((_XTAL_FREQ/(PWM_freq*4*TMR2PRESCALE)) - 1); //Setting the PR2 formulae using Datasheet // Makes the PWM work in 5KHZ
    CCP1CONbits.P1M = 0b10;  // half bridge PWM 
    CCP1CONbits.CCP1M = 0b1101; // PWM with active high & active low (same phase difference)
//    CCP1CONbits.CCP1M = 0b1100; // PWM with active high & active high (80 phase difference)
    
    T2CONbits.T2CKPS = 0b01; //prescaler = 4
    T2CONbits.TMR2ON = 0b1; // start timer
    
//    PWM1CONbits.PDC = 10; // test and find a suitable value (useful with 180 phase difference)

    set_PWM_duty_cycle(init_pulse_width);
    
    TRISC2 = 0; // make port pin RC2 (P1A) as output
    TRISB2 = 0; // make port pin RB2 (P1B) as output
    
    return;    
}

void set_PWM_duty_cycle(uint16_t duty){
    if(duty<1023){
        duty = (uint16_t)(((float)duty/1023)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE))); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);
        
        CCP1CONbits.DC1B = duty & 0b00000011; // store lsb 2 bits
        CCPR1L = duty>>2;// Store the remining 8 bit
    }
    return;
}

void ADC_Initialize()
{
  ADCON0 = 0b01000001; //ADC ON and Fosc/8 is selected
  ADCON1 = 0b11000000; // Internal reference voltage is selected
  ANSELbits.ANS0 = 1; // make RA0 analog pin
  TRISAbits.TRISA0 = 1; // make RA0 input pin
  return;
}

uint16_t ADC_Read(uint8_t channel)
{
//  ANSEL = 0x01 << channel; // make the pin analog read pin
//  ANSELH = 0x00 ; // do not use channels from AN8 to AN12
  ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
  ADCON0 |= (unsigned)channel<<3; //Setting the required Bits
  __delay_ms(2); //Acquisition time to charge hold capacitor
  GO_nDONE = 1; //Initializes A/D Conversion
  while(GO_nDONE); //Wait for A/D Conversion to complete
  return (uint16_t)((ADRESH<<8)+ADRESL); //Returns Result
}

