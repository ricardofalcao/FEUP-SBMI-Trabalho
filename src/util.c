#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "util.h"

// Timer compare value to make the timer calculate milliseconds very well
#define TIMER_PRESCALER_ONE_MS 64 // check the datasheet, we use TCCR1B to set it
// The timer compare value which corresponds to the 1 ms (approx)
#define TIMER_COMPARE_VALUE_ONE_MS (uint8_t) ( F_CPU / ((uint32_t) 1000 * TIMER_PRESCALER_ONE_MS)) 

volatile uint64_t millis = 0;

ISR(TIMER2_COMPA_vect) {
    millis++;
}

void init_util(void) {
    TCCR2B = 0;
    TIFR2 |= (7 << TOV2);
	TCCR2A = (1 << WGM21); // CTC mode -> TCNT2 = OCR2A    
	OCR2A = TIMER_COMPARE_VALUE_ONE_MS; 

	TIMSK2 |= (1 << OCIE2A); // allow interrupts compa
	TCCR2B |= (1 << CS22); // prescaler 64

    sei();
}

uint16_t clamp_16(uint16_t value, uint16_t min, uint16_t max) {
    if(value > max) {
        value = max;
    } else if(value < min)  {
        value = min;
    }

    return value;
}

uint8_t clamp_8(uint8_t value, uint8_t min, uint8_t max) {
    if(value > max) {
        value = max;
    } else if(value < min)  {
        value = min;
    }

    return value;
}

uint64_t get_millis(void){
    uint64_t out;
    cli();
    out = millis;
    sei();
    return out;
}
