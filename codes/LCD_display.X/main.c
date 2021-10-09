#include <xc.h>
#include "config.h"
#include <stdint.h>
#include "pic16f886.h"
#include <stdio.h>
#include <stdbool.h>

#define _XTAL_FREQ 4000000

///  LCD pins  ///  
#define RS RC4
#define EN RC5
#define D4 RC6
#define D5 RC7
#define D6 RB6
#define D7 RB7

#include "lcd.h"


void main(){

  
  ANSEL = 0x00;
  ANSELH = 0x00;
  
  TRISB = 0x00;
  TRISC = 0x00;
   
  Lcd_Init();  //Initialize the LCD
  
  char line1[16] = "abcdefghijklmnop";
  char line2[16] = "1234567890123456";
  
  int freq = 5;
  int current_direction = 1;
  float duty_LCD = 0.5;
  
  sprintf(line1, "Inverter freq=%ik", freq);
  sprintf(line2, "dir=%d duty=%0.3f", current_direction,duty_LCD);
  
  Lcd_Clear();
  Lcd_Set_Cursor(1,1);
  Lcd_Write_String(line1);
  Lcd_Set_Cursor(2,1);
  Lcd_Write_String(line2);

  
  while (1){
   
//    __delay_ms(50);
  }

}

