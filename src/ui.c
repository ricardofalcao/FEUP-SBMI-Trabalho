#include "ui.h"

void draw_battery(float battery) {
    uint8_t width = 40, height = 12;
    uint8_t startX = 1, startY = 13;

    lcd_gotoxy(0, 0);

    lcd_puts("Bateria:");
    lcd_drawRect(startX, startY, startX + width, startY + height, WHITE);
    lcd_fillRect(startX, startY, startX + width * battery, startY + height, WHITE);
    lcd_fillRect(startX + width * battery + 1, startY + 1, startX + width - 1, startY + height - 1, BLACK);

    lcd_gotoxy(8, 2);
    char str[5];
    sprintf(str, "%d%%  ", (uint8_t) (battery * 100));
    lcd_puts(str);
    lcd_display();
}

void draw_sensors(uint8_t mask) {
    lcd_gotoxy(0, 4);
    lcd_puts("Sensores:");

    lcd_drawCircle(4, 59, 3, WHITE);
    lcd_drawCircle(14, 55, 3, WHITE);
    lcd_drawCircle(24, 52, 3, WHITE);
    lcd_drawCircle(34, 55, 3, WHITE);
    lcd_drawCircle(44, 59, 3, WHITE);

    lcd_fillCircle(4, 59, 2, (mask & (1 << 4) ? WHITE : BLACK));
    lcd_fillCircle(14, 55, 2, (mask & (1 << 3) ? WHITE : BLACK));
    lcd_fillCircle(24, 52, 2, (mask & (1 << 2) ? WHITE : BLACK));
    lcd_fillCircle(34, 55, 2, (mask & (1 << 1) ? WHITE : BLACK));
    lcd_fillCircle(44, 59, 2, (mask & (1 << 0) ? WHITE : BLACK));
    lcd_display();
}

/*
*/

void init_ui() {
    lcd_init(LCD_DISP_ON);    // init lcd and turn on

    draw_battery(0.0f);
    draw_sensors(0b00000100);
}