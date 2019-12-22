#include "ui.h"
#include "main.h"
#include <stdio.h>

#define BATTERY_POSITION_X 58
#define BATTERY_POSITION_Y 15
#define BATTERY_WIDTH 34
#define BATTERY_HEIGHT 9

#define SENSORS_POSITION_X 58
#define SENSORS_POSITION_Y 31
#define SENSORS_WIDTH 32
#define SENSORS_HEIGHT 9

static const char *Car_State_Name[] = {
    "Em espera", "Calibrando", "Manual", "Automatico",
};

void draw_battery(float battery) {
    lcd_gotoxy(0, 2);

    lcd_puts("Bateria:");

    lcd_drawRect(BATTERY_POSITION_X, BATTERY_POSITION_Y, BATTERY_POSITION_X + BATTERY_WIDTH, BATTERY_POSITION_Y + BATTERY_HEIGHT, WHITE);
    lcd_fillRect(BATTERY_POSITION_X + 1, BATTERY_POSITION_Y + 1, BATTERY_POSITION_X + 1 + (BATTERY_WIDTH - 2) * battery, BATTERY_POSITION_Y + BATTERY_HEIGHT - 1, WHITE);
    lcd_fillRect(BATTERY_POSITION_X + 1 + (BATTERY_WIDTH - 2) * battery + 1, BATTERY_POSITION_Y + 1, BATTERY_POSITION_X + BATTERY_WIDTH - 1, BATTERY_POSITION_Y + BATTERY_HEIGHT - 1, BLACK);

    lcd_gotoxy(16, 2);
    char *str = calloc(5, sizeof(char));
    sprintf(str, "%d%%  ", (uint8_t) (battery * 100));
    lcd_puts(str);
    free(str);
}

void draw_state(uint8_t state) {
    const char *text = Car_State_Name[state];

    char str[24];
    uint8_t ended = 0;
    for(uint8_t i = 0; i < 24; i++) {
        char c = text[i];

        if(c == '\0') {
            ended = 1;
        }

        if(ended) {
            str[i] = ' ';
            continue;
        }

        str[i] = c;
    }

    str[23] = '\0';

    lcd_gotoxy(0, 7);
    lcd_puts(str);
}

void draw_sensors(uint8_t mask, float weighted_position) {
    lcd_gotoxy(0, 4);

    lcd_puts("Sensores:");

    lcd_drawRect(SENSORS_POSITION_X, SENSORS_POSITION_Y, SENSORS_POSITION_X + SENSORS_WIDTH, SENSORS_POSITION_Y + SENSORS_HEIGHT, WHITE);

    uint8_t pixelsPerSensor = (SENSORS_WIDTH - 2) / SENSORS_NUM;

    for(uint8_t i = 0; i < SENSORS_NUM; i++) {
        uint8_t active = mask & (1 << i);

        lcd_fillRect(
            SENSORS_POSITION_X + 1 + pixelsPerSensor * i + (i > 0 ? 1 : 0), 
            SENSORS_POSITION_Y + 1, 
            SENSORS_POSITION_X + pixelsPerSensor * (i + 1) + 1, 
            SENSORS_POSITION_Y + SENSORS_HEIGHT - 1, 
            active ? BLACK : WHITE
        );
    }

    float x = SENSORS_POSITION_X + SENSORS_WIDTH * (weighted_position + 1.0f) / 2.0f;

    lcd_fillRect(SENSORS_POSITION_X, SENSORS_POSITION_Y + SENSORS_HEIGHT + 3, SENSORS_POSITION_X + SENSORS_WIDTH, SENSORS_POSITION_Y + SENSORS_HEIGHT + 3 + 6, BLACK);
    lcd_drawLine(x, SENSORS_POSITION_Y + SENSORS_HEIGHT + 3, x, SENSORS_POSITION_Y + SENSORS_HEIGHT + 3 + 6, WHITE);
}

/*
*/

void init_ui() {
    lcd_init(LCD_DISP_ON);    // init lcd and turn on
    lcd_clrscr();

    draw_battery(0.0f);
}