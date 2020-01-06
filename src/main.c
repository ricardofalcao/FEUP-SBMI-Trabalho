#include <stdlib.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/eeprom.h>
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

#define CROSS_SECTION_NUM 4

#define MAX_BATTERY_VOLTAGE 8300
#define MIN_BATTERY_VOLTAGE 6400

#define MOTOR_MAX_TURN 1.0f

#define DEBUG

static uint16_t EEMEM minimum_sensor_value[SENSORS_NUM];
static uint16_t EEMEM maximum_sensor_value[SENSORS_NUM];

static Car_State state = AUTOMATIC;

/*
  Automatic car variables
*/

static float Kp = 0.5; // proportional

static float Ki = 0; // integral

static float Kd = 0; // derivative

static float Kr = 0; // reduction

static float base_velocity = .6f;

static Line_Sensor sensors[SENSORS_NUM];

static float current_line_position;

static float last_line_position;

static uint8_t last_detected_sensors = 0;

static uint8_t laps = 0;
/*
  Manual car variables
*/

static float speed = 1.0f; // 0 - parado | 1 - velocidade máxima
static float direction = 0.0f; // -1 esquerda | 0 - centro | 1 - direita
static float directionStep = 0.5f;

static uint64_t last_direction_change_time = 0;

/*
*/
static uint64_t last_ui_update = 0;
static uint64_t last_clock = 0;

void init_USART(void) {
  UBRR0 = baudgen;
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void init_hw(void) {
  pwm_init(PWM_0A, PRESCALER_8);
  pwm_init(PWM_0B, PRESCALER_8);

  sei();

  pin_mode(D, 7, OUTPUT); // C
  pin_mode(D, 4, OUTPUT); // B
  pin_mode(D, 3, OUTPUT); // A
}

uint16_t analog_mux_read(uint8_t channel) {
  digital_write(D, 3, (channel & (1 << 0)));
  digital_write(D, 4, (channel & (1 << 1)));
  digital_write(D, 7, (channel & (1 << 2)));
  
  uint16_t value = analog_read(ADC_CHANNEL_2);
  //value = analog_read(ADC_CHANNEL_2);
  return value;
}

/*

 IR LISTENERS

*/

void ir_listener_on_ok(IR_Packet packet) {
  if(packet.repeat == 0) {
    if(state == IDLE) {
      state = AUTOMATIC;
    } else {
      state = IDLE;
    }

    draw_state(state);
  }
}

void ir_listener_on_up(IR_Packet packet) {
  if(state == MANUAL) {
    if(packet.repeat == 0) {
      speed = clamp_f(speed + 0.25f, 0.0f, 1.0f);
    }
  }
}

void ir_listener_on_down(IR_Packet packet) {
  if(state == MANUAL) {
    if(packet.repeat == 0) {
      speed = clamp_f(speed - 0.25f, 0.0f, 1.0f);
    }
  }
}

void ir_listener_on_left(IR_Packet packet) {
  if(state == MANUAL) {
    direction = - directionStep;
    last_direction_change_time = get_millis();
  }
}

void ir_listener_on_right(IR_Packet packet) {
  if(state == MANUAL) {
    direction = directionStep;
    last_direction_change_time = get_millis();
  }
}

void ir_listener_on_1(IR_Packet packet) { // chao
  if(state == AUTOMATIC) {
    for(uint8_t i = 0; i < SENSORS_NUM; i++) {
      sensors[i].minimum_value = analog_mux_read(sensors[i].pin);
      eeprom_update_word(&minimum_sensor_value[i], sensors[i].minimum_value);
      printf("Saving sensor %d min value: %d\n", i + 1, sensors[i].minimum_value);
    }
  }
}

void ir_listener_on_2(IR_Packet packet) { // linha
  if(state == AUTOMATIC) {
    for(uint8_t i = 0; i < SENSORS_NUM; i++) {
      sensors[i].maximum_value = analog_mux_read(sensors[i].pin);
      eeprom_update_word(&maximum_sensor_value[i], sensors[i].maximum_value);
      printf("Saving sensor %d max value: %d\n", i + 1, sensors[i].maximum_value);
    }
  }
}

/*



*/

// 0 - Chão | 1 - Linha
float read_sensor(Line_Sensor * sensor) {
  uint16_t read = 0;

  for(uint8_t i = 0; i < SENSORS_SAMPLES; i++) {
    read += analog_mux_read(sensor->pin);
  }

  read = read / SENSORS_SAMPLES;

  float value = clamp_f( (float) ((int16_t) read - (int16_t) sensor->minimum_value) / (float) (sensor->maximum_value - sensor->minimum_value), 0.0f, 1.0f);
  
  //printf("%d %d (%d) [%d-%d]\n", sensor->index, read, (uint8_t) (value * 100), sensor->minimum_value, sensor->maximum_value);

  #if LINE_TYPE == 0 // white line
    return value;
  #else // black line
    return 1.0f - value;
  #endif
}

void loop(uint32_t current_millis) {
  uint8_t redraw_lcd = current_millis - last_ui_update > 500;

  IR_Packet * packet = ir_run();

  if(packet != NULL) {
    switch(packet->command) {
    case IR_NUMBER_FOUR: 
      Kp += 0.01f;
      last_ui_update = 0;
      break;
    case IR_NUMBER_SEVEN: 
      Kp -= 0.01f;
      last_ui_update = 0;
      break;
    case IR_NUMBER_FIVE: 
      Ki += 0.01f;
      break;
    case IR_NUMBER_EIGHT: 
      Ki -= 0.01f;
      break;
    case IR_NUMBER_SIX: 
      Kd += 0.01f;
      last_ui_update = 0;
      break;
    case IR_NUMBER_NINE: 
      Kd -= 0.01f;
      last_ui_update = 0;
      break;
    case IR_ASTERISK: 
      base_velocity += 0.01f;
      last_ui_update = 0;
      break;
    case IR_HASHTAG: 
      base_velocity -= 0.01f;
      last_ui_update = 0;
      break;
    case IR_NUMBER_THREE: 
      Kr += 0.01f;
      last_ui_update = 0;
      break;
    case IR_NUMBER_ZERO: 
      if(packet->repeat == 0) {
        if(state == IDLE) {
          state = MANUAL;
          draw_state(state);
          break;
        }
      }

      Kr -= 0.01f;
      last_ui_update = 0;
      break;
  }
  }

  if(state == MANUAL) {
    if(current_millis - last_direction_change_time > 150) {
      direction = 0.0f;
    } 

    pwm_write(PWM_0B, speed * (direction < 0.0f ? 1.0f + direction : 1.0f));
    pwm_write(PWM_0A, speed * (direction > 0.0f ? 1.0f - direction : 1.0f));

  } else if(state == AUTOMATIC) {
    float weighted_sum = 0.0f;
    float sum = 0.0f;

    uint8_t detected_sensors = 0;
    uint8_t aux = 0;

    for(uint8_t i = 0; i < SENSORS_NUM; i++) {
      float value = read_sensor(&sensors[i]);

      float weight = (i + 1) * 1000;

      if (value >= LINE_THRESHOLD) { // Line detected
        detected_sensors++;
      } 
      
      if((value < LINE_THRESHOLD && detected_sensors > 0) || detected_sensors > 2) {
        aux = 1;
      }

      if(!aux) {
        weighted_sum += value * weight;
        sum += value;
      }
    }

    if(detected_sensors < 3) {
      if(sum > 0.0f) {
        float pid_correction = 0.0f;
        last_line_position = current_line_position;
        current_line_position = (weighted_sum / sum - 3000) / 2000.0; // Entre 1000 e 5000
        
        pid_correction =  Kp * current_line_position + 
                                Ki * (last_line_position + current_line_position) + 
                                Kd * (current_line_position - last_line_position);
        // positivo significa que é preciso virar à direita

        float reduction = clamp_f(Kr * (fabs(last_line_position) + fabs(current_line_position)), 0.0f, base_velocity);

        pwm_write(PWM_0A, base_velocity - pid_correction - reduction);
        pwm_write(PWM_0B, base_velocity + pid_correction - reduction);

        #ifdef DEBUG
        if(redraw_lcd) {
          char *text2 = calloc(sizeof(char), 24);
          sprintf(text2, "%.2f    %d  ", pid_correction, detected_sensors);
          lcd_gotoxy(0, 6);
          lcd_puts(text2);
          free(text2);
        }
        #endif
      }

    } else if(last_detected_sensors < 4 && detected_sensors >= 4) {
      laps++;
      lcd_gotoxy(14, 6);

      char *text = calloc(sizeof(char), 8);
      sprintf(text, "V: %d  ", laps);
      lcd_puts(text);
      free(text);
    }

    
    last_detected_sensors = detected_sensors;
    draw_sensors(0b00000000, current_line_position);


  } else if(state == IDLE) {
    pwm_write(PWM_0A, 0.0f);
    pwm_write(PWM_0B, 0.0f);

  }

  if(redraw_lcd) {
    #ifdef DEBUG
      char *text = calloc(sizeof(char), 18);
      sprintf(text, "%.2f %.2f %.2f %.2f", Kp, Kd, Kr, base_velocity);
      draw_info(text);
      free(text);
    #endif

    uint16_t battery_read = analog_read(ADC_CHANNEL_1);
    uint16_t voltage = battery_read / 1023.0f * 5000 * 2;
    float battery = calculate_battery(voltage, MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE);
    draw_battery(battery);

    last_ui_update = current_millis;
    lcd_display();
  }
}

int main(void) {
  // Initialization 
  sensors[0].pin = 6;
  sensors[1].pin = 7;
  sensors[2].pin = 5;
  sensors[3].pin = 0;
  sensors[4].pin = 3;

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

  ir_add_listener(IR_NUMBER_ONE, ir_listener_on_1);
  ir_add_listener(IR_NUMBER_TWO, ir_listener_on_2);

  pwm_init(PWM_0A, PRESCALER_8); // Motor esquerdo
  pwm_init(PWM_0B, PRESCALER_8); // Motor direito

  sei();

  for(uint8_t i = 0; i < SENSORS_NUM; i++) {
    sensors[i].index = i;
    sensors[i].minimum_value = eeprom_read_word(&minimum_sensor_value[i]);
    sensors[i].maximum_value = eeprom_read_word(&maximum_sensor_value[i]);

    printf("Restoring sensor %d values: (%d, %d)\n", i + 1, sensors[i].minimum_value, sensors[i].maximum_value);
  }

  draw_state(state);

  while(1) {
    uint64_t current_millis = get_millis();
    
    if(current_millis - last_clock > 10) {
      loop(current_millis);
    }
  }
}