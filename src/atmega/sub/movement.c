#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include "movement.h"

void InitializeMovement(void) {
    DDRC &= ~(1 << DDC0);
    
    PRR0 &= ~(1 << PRADC); // Clear power reduction bit
    ADMUX |= (1 << REFS0); // Select Vref = AVcc
    ADMUX &= ~(1 << REFS1); // Select Vref = AVcc
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
    ADMUX &= ~(1 << MUX0) & ~(1 << MUX1) & ~(1 << MUX2) & ~(1 << MUX3);
    ADCSRA |= (1 << ADATE);
    ADCSRB &= ~(1 << ADTS0) & ~(1 << ADTS1) & ~(1 << ADTS2);
    DIDR0 |= (1 << ADC0D);
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
    
    // Setup timer2: 
    // Set OC2B (PD3) as output
    DDRD |= (1 << DDD3);

    // Fast PWM mode with OCR2A as TOP (Mode 7)
    TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2B1);
    TCCR2B = (1 << WGM22);

    // Prescaler = 8
    TCCR2B |= (1 << CS21);

    // Set TOP for ~38 kHz
    OCR2A = 52;

    // 50% duty cycle
    OCR2B = OCR2A / 2;
    
    
    // GPIO input for address reading
    DDRD &= ~(1 << DDD4);
    
    // Outputs for 
    DDRC |= (1 << DDC0);
    PORTC &= ~(1 << PORTC0);
   
    DDRC |= (1 << DDC1);
    PORTC &= ~(1 << PORTC1);
   
    DDRC |= (1 << DDC2);
    PORTC &= ~(1 << PORTC2);
   
    DDRC |= (1 << DDC3);
    PORTC &= ~(1 << PORTC3);
}

void turn_cw() {
    PORTC |=  (1 << PORTC0);
    PORTC &= ~(1 << PORTC1);
    PORTC &= ~(1 << PORTC2);
    PORTC |=  (1 << PORTC3);
}

void turn_ccw() {
    PORTC &= ~(1 << PORTC0);
    PORTC |=  (1 << PORTC1);
    PORTC |=  (1 << PORTC2);
    PORTC &= ~(1 << PORTC3);
}

void move_forward() {
    PORTC &= ~(1 << PORTC0);
    PORTC |=  (1 << PORTC1);
    PORTC &= ~(1 << PORTC2);
    PORTC |=  (1 << PORTC3);
}

void move_backward() {
    PORTC |=  (1 << PORTC0);
    PORTC &= ~(1 << PORTC1);
    PORTC |=  (1 << PORTC2);
    PORTC &= ~(1 << PORTC3);
}

void stop_movement() {
    PORTC &= ~((1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3));
}

int main(void) {
   uart_init();
   Initialize();

   while (1) {
       printf("ADC: %d\n", ADC);
       _delay_ms(50);
       if (!(PIND & (1 << PD4))) {
           printf("ON");
       }
       // your application logic
   }
}