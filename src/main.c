#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "serial_printf.h"
#include "pins.h"
#include "pwm.h"
#include "adc.h"
#include "ui.h"
#include "ir.h"
#include "util.h"

#define	baud 57600  // baud rate
#define baudgen ((F_CPU / (16 * baud)) -1)  //baud divider

/*
*/

void init_USART(void) {
  UBRR0 = baudgen;
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void init_hw(void) {
  pwm_init(PWM_0A, PRESCALER_64);
  pwm_init(PWM_0B, PRESCALER_64);

  lcd_init(LCD_DISP_ON);

  sei();
}

/*

 IR LISTENERS

*/

void ir_listener_on_ok(IR_Packet packet) {
  printf("Hello OK! %d\n", packet.repeat);
}

/*



*/

int main(void) {
  // Initialization 
  init_USART();
  init_serial();

  init_hw();
  init_adc();
  init_ir();

  init_util();
  init_ui();

  ir_add_listener(IR_OK, ir_listener_on_ok);

  /* pinMode(D, 4, OUTPUT);
  pinMode(D, 3, OUTPUT);
  pinMode(D, 2 , OUTPUT);
  for(uint8_t channel = 0; channel < 8; channel++) {
    digitalWrite(D, 4, (channel & (1 << 0)));
    digitalWrite(D, 3, (channel & (1 << 1)));
    digitalWrite(D, 2, (channel & (1 << 2)));

    uint16_t value = analogRead(ADC_CHANNEL_0);
    value = analogRead(ADC_CHANNEL_0);
  }*/

  while(1) {
    ir_run();
  }
}