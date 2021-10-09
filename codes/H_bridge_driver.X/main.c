#include <xc.h>
#include "config.h"
#include <stdint.h>
#include "pic16f886.h"
#include <stdio.h>
#include <stdbool.h>

#define _XTAL_FREQ 4000000
#define TMR2PRESCALE 4

///  LCD pins  ///  
#define RS RC4
#define EN RC5
#define D4 RC6
#define D5 RC7
#define D6 RB6
#define D7 RB7

#define direction_set_pin PORTCbits.RC0
#define LED0 PORTBbits.RB0
#define LED1 PORTBbits.RB1
#define direction_0 RA1
#define direction_1 RA2

#include "lcd.h"

long PWM_freq = 5000;
uint16_t init_pulse_width = 250; //initial value (0-1023)
bool current_direction = 0; // used for LCD
float duty_LCD;  // used for LCD

void PWM_Initialize(long PWM_freq, uint16_t init_pulse_width );
void set_PWM_duty_cycle(uint16_t duty);
void ADC_Initialize();
uint16_t ADC_Read(uint8_t channel);
void set_H_bridge_pins();

void main(){

  uint16_t adc_value;
  
  uint16_t pulse_width = init_pulse_width;
  
  ANSEL = 0x00;
  ANSELH = 0x00;
  
  TRISB = 0x00;
  TRISC = 0x00;
  
  ADC_Initialize(); //Initializes ADC Module
  PWM_Initialize(PWM_freq, init_pulse_width );  //This sets the PWM frequency of PWM1
  
  Lcd_Init();
 

  TRISAbits.TRISA1 = 0;
  TRISAbits.TRISA2 = 0;
  LED0 = 0;	// Keep the LED0 off
  LED1 = 0;    // Keep the LED1 off
 
  current_direction = direction_set_pin;
  set_H_bridge_pins();   // initialize H_bridge inputs (PWM signals & digital inputs)
  
  char line1[16];
  char line2[16];
  
  duty_LCD = (float)(pulse_width)/1023;
  int freq = PWM_freq / 1000; // frequency in kHz

  sprintf(line1, "Inverter freq=%dk", freq);
  sprintf(line2, "dir=%d duty=%0.3f", current_direction,duty_LCD);
  
  Lcd_Clear();
  Lcd_Set_Cursor(1,1);
  Lcd_Write_String(line1);
  Lcd_Set_Cursor(2,1);
  Lcd_Write_String(line2);

  
  while (1){

    adc_value = ADC_Read(0);
    
    if (adc_value != pulse_width){
        pulse_width = adc_value;
        set_PWM_duty_cycle(pulse_width);
      
        char duty_str[5];
        duty_LCD = (float)(pulse_width)/1023;
        sprintf(duty_str, "%0.3f", duty_LCD);
        Lcd_Set_Cursor(2,12);
        Lcd_Write_String(duty_str);
    }
    
    if (direction_set_pin != current_direction){
        current_direction = direction_set_pin;
        set_H_bridge_pins();   // initialize H_bridge inputs (PWM signals & digital inputs)

        char dir_str;
        sprintf(dir_str, "%d", current_direction);
        Lcd_Set_Cursor(2,5);
        Lcd_Write_String(dir_str);
    }
    
    
    __delay_ms(50);
    
  }

}


void PWM_Initialize(long PWM_freq, uint16_t init_pulse_width ){
    
    PR2 = (unsigned)((_XTAL_FREQ/(PWM_freq*4*TMR2PRESCALE)) - 1); //Setting the PR2 formulae using Datasheet // Makes the PWM work in 5KHZ
    CCP1CONbits.P1M = 0b10;  // half bridge PWM 
    CCP1CONbits.CCP1M = 0b1101; // PWM with active high & active low (same phase)
    
    T2CONbits.T2CKPS = 0b01; //prescaler = 4
    T2CONbits.TMR2ON = 0b1; // start timer
    
//    PWM1CONbits.PDC = 10; // test and find a suitable value **********

    set_PWM_duty_cycle(init_pulse_width);
    
    TRISC2 = 0; // make port pin RC2 (P1A) as output
    TRISB2 = 0; // make port pin RB2 (P1B) as output
    
    return;    
}

void set_PWM_duty_cycle(uint16_t duty){
    if(duty<1023){
        duty = (uint16_t)(((float)duty/1023)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE))); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);
        
        CCP1CONbits.DC1B = duty & 0x03; // store lsb 2 bits
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

uint16_t ADC_Read(uint8_t channel){
  ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
  ADCON0 |= (unsigned)channel<<3; //Setting the required Bits
  __delay_ms(2); //Acquisition time to charge hold capacitor
  GO_nDONE = 1; //Initializes A/D Conversion
  while(GO_nDONE); //Wait for A/D Conversion to complete
  return (uint16_t)((ADRESH<<8)+ADRESL); //Returns Result
}

void set_H_bridge_pins(){
    if (current_direction == 0){
        direction_0 = 1;
        TRISCbits.TRISC2 = 0;
        LED0 = 1;
        direction_1 = 0; 
        TRISBbits.TRISB2 = 1;
        LED1 = 0;
    }
    else{
        direction_1 = 1;
        TRISBbits.TRISB2 = 0;
        LED1 = 1;
        direction_0 = 0;
        TRISCbits.TRISC2 = 1;
        LED0 = 0;
    }
    return;
}

