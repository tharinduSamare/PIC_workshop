#include <xc.h>
#include "config.h"
#include <stdint.h>
#include "pic16f886.h"

#define _XTAL_FREQ 4000000

void ADC_Initialize();
uint16_t ADC_Read(uint8_t channel);

uint16_t ADC_limits[8] = {0,128,256,384,512,640,768,896}; // ADC valus limits

void main(void) {
  uint16_t adc_value;
  
  ANSEL = 0x00; //set all analog input capable pins to digital inputs
  ANSELH = 0x00;
  
  TRISB = 0x00; // make port B as outputs 
  PORTB = 0x00;
  
  ADC_Initialize(); //Initializes ADC Module

  while (1){
    adc_value = ADC_Read(0);
    
    for (uint8_t i = 0; i< 8;i++){
        if (adc_value > ADC_limits[i]){
            PORTB = PORTB | (1U<<i); // LED on
        }
        else{
            PORTB = PORTB & ~(1U<<i) ; // LED off
        }
    }
  __delay_ms(50);
  }
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
  ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
  ADCON0 |= (unsigned)channel<<3; //Setting the required Bits
  __delay_ms(2); //Acquisition time to charge hold capacitor
  GO_nDONE = 1; //Initializes A/D Conversion
  while(GO_nDONE); //Wait for A/D Conversion to complete
  return (uint16_t)((ADRESH<<8)+ADRESL); //Returns Result
}