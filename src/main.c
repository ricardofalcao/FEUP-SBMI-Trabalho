#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "main.h"
#include "serial_printf.h"
#include "pins.h"
#include "pwm.h"
#include "adc.h"
#include "ui.h"
#include "ir.h"
#include "util.h"

#define	baud 57600  // baud rate
#define baudgen ((F_CPU / (16 * baud)) -1)  //baud divider

static Car_State state = IDLE;

/*
  Automatic car variables
*/

static float Kp = 2; // proportional

static float Ki = 2; // integral

static float Kd = 2; // derivative

static Line_Sensor sensors[SENSORS_NUM];

/*
  Manual car variables
*/

static float speed = 0.4f; // 0 - parado | 1 - velocidade máxima
static float direction = 0.0f; // -1 esquerda | 0 - centro | 1 - direita
static float directionStep = 0.8f;

static uint64_t last_direction_change_time = 0;

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

  pin_mode(B, 4, OUTPUT);
  pin_mode(B, 3, OUTPUT);
  pin_mode(B, 2, OUTPUT);
}

uint16_t analog_mux_read(uint8_t channel) {
  digital_write(B, 4, (channel & (1 << 0)));
  digital_write(B, 3, (channel & (1 << 1)));
  digital_write(B, 2, (channel & (1 << 2)));

  uint16_t value = analogRead(ADC_CHANNEL_0);
  value = analogRead(ADC_CHANNEL_0);
  return value;
}

/*

 IR LISTENERS

*/

void ir_listener_on_ok(IR_Packet packet) {
  if(packet.repeat == 0) {
    if(state == IDLE) {
      state = MANUAL;
    } else if (state == MANUAL) {
      state = IDLE;
    }
  }
}

void ir_listener_on_up(IR_Packet packet) {
  if(packet.repeat == 0) {
    speed = clamp_f(speed + 0.25f, 0.0f, 1.0f);
  }
}

void ir_listener_on_down(IR_Packet packet) {
  if(packet.repeat == 0) {
    speed = clamp_f(speed - 0.25f, 0.0f, 1.0f);
  }
}

void ir_listener_on_left(IR_Packet packet) {
  direction = - directionStep;
  last_direction_change_time = get_millis();
}

void ir_listener_on_right(IR_Packet packet) {
  direction = directionStep;
  last_direction_change_time = get_millis();
}

/*



*/
Line_Sensor_Read read_sensor(Line_Sensor * sensor) {
  uint16_t read = analog_mux_read(sensor->index);
  
  #if LINE_TYPE == 1 // white line
    return read >= sensor->line_threshold ? LINE : read <= sensor->floor_threshold ? FLOOR : UNKNOWN;
  #else // black line
    return read <= sensor->line_threshold ? LINE : read >= sensor->floor_threshold ? FLOOR : UNKNOWN;
  #endif
}

int main(void) {
  // Initialization 
  for(uint8_t i = 0; i < SENSORS_NUM; i++) {
    sensors[i].index = i;
  }

  init_USART();
  init_serial();

  init_hw();
  init_adc();
  init_ir();

  init_util();
  init_ui();

  ir_add_listener(IR_UP, ir_listener_on_up);
  ir_add_listener(IR_DOWN, ir_listener_on_down);

  ir_add_listener(IR_OK, ir_listener_on_ok);
  ir_add_listener(IR_LEFT, ir_listener_on_left);
  ir_add_listener(IR_RIGHT, ir_listener_on_right);

  pwm_init(PWM_0A, PRESCALER_64); // Motor esquerdo
  pwm_init(PWM_0B, PRESCALER_64); // Motor direito

  sei();

  while(1) {
    uint64_t current_millis = get_millis();
    IR_Packet packet = ir_run();

    if(state == MANUAL) {
      switch(packet.command) {
        case IR_NUMBER_ONE:
          directionStep = 0.1f;
          break; 
        case IR_NUMBER_TWO:
          directionStep = 0.2f;
          break; 
        case IR_NUMBER_THREE:
          directionStep = 0.3f;
          break; 
        case IR_NUMBER_FOUR:
          directionStep = 0.4f;
          break; 
        case IR_NUMBER_FIVE:
          directionStep = 0.5f;
          break; 
        case IR_NUMBER_SIX:
          directionStep = 0.6f;
          break; 
        case IR_NUMBER_SEVEN:
          directionStep = 0.7f;
          break; 
        case IR_NUMBER_EIGHT:
          directionStep = 0.8f;
          break; 
        case IR_NUMBER_NINE:
          directionStep = 0.9f;
          break; 
        case IR_NUMBER_ZERO:
          directionStep = 1.0f;
          break; 
      }

      if(current_millis - last_direction_change_time > 400) {
        direction = 0.0f;
      } 

      pwm_write(PWM_0A, speed * (direction < 0.0f ? 1.0f + direction : 1.0f));
      pwm_write(PWM_0B, speed * (direction > 0.0f ? 1.0f - direction : 1.0f));

    } else if(state == IDLE) {
      pwm_write(PWM_0A, 0.0f);
      pwm_write(PWM_0B, 0.0f);

    }
  }
}