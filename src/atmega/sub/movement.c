#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include "uart.h"

#define DDRMOTOR DDRD
#define PORTMOTOR PORTD
#define MOTOR_SETUP (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);
#define MOTOR_LEFT_ONE PD4
#define MOTOR_LEFT_TWO PD5
#define MOTOR_RIGHT_ONE PD6
#define MOTOR_RIGHT_TWO PD7

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
    
    DDRMOTOR |= MOTOR_SETUP;
}

void move_backward() {
    PORTMOTOR |=  (1 << MOTOR_LEFT_ONE);
    PORTMOTOR &= ~(1 << MOTOR_LEFT_TWO);
    PORTMOTOR &= ~(1 << MOTOR_RIGHT_ONE);
    PORTMOTOR |=  (1 << MOTOR_RIGHT_TWO);
}

void move_forward() {
    PORTMOTOR &= ~(1 << MOTOR_LEFT_ONE);
    PORTMOTOR |=  (1 << MOTOR_LEFT_TWO);
    PORTMOTOR |=  (1 << MOTOR_RIGHT_ONE);
    PORTMOTOR &= ~(1 << MOTOR_RIGHT_TWO);
}

void turn_cw() {
    PORTMOTOR &= ~(1 << MOTOR_LEFT_ONE);
    PORTMOTOR |=  (1 << MOTOR_LEFT_TWO);
    PORTMOTOR &= ~(1 << MOTOR_RIGHT_ONE);
    PORTMOTOR |=  (1 << MOTOR_RIGHT_TWO);
}

void turn_ccw() {
    PORTMOTOR |=  (1 << MOTOR_LEFT_ONE);
    PORTMOTOR &= ~(1 << MOTOR_LEFT_TWO);
    PORTMOTOR |=  (1 << MOTOR_RIGHT_ONE);
    PORTMOTOR &= ~(1 << MOTOR_RIGHT_TWO);
}

void stop_movement() {
    PORTMOTOR &= ~((1 << MOTOR_LEFT_ONE) | (1 << MOTOR_LEFT_TWO) | 
            (1 << MOTOR_RIGHT_ONE) | (1 << MOTOR_RIGHT_TWO));
}

//int main(void) {
//   uart_init();
//   InitializeMovement();
//
//   while (1) {
////        // Coast first
////        PORTMOTOR &= ~(1 << PORTB1);
////        PORTMOTOR &= ~(1 << PORTB2);
////        PORTMOTOR &= ~(1 << PORTB3);
////        PORTMOTOR &= ~(1 << PORTB4);
////        _delay_ms(50);
////
////        // Forward
////        PORTMOTOR &= ~(1 << PORTB1);
////        PORTMOTOR |=  (1 << PORTB2);
////        PORTMOTOR &= ~(1 << PORTB3);
////        PORTMOTOR |=  (1 << PORTB4);
////        _delay_ms(500);
////
////        // Coast
////        PORTMOTOR &= ~(1 << PORTB1);
////        PORTMOTOR &= ~(1 << PORTB2);
////        PORTMOTOR &= ~(1 << PORTB3);
////        PORTMOTOR &= ~(1 << PORTB4);
////        _delay_ms(50);
////
////        // Reverse
////        PORTMOTOR |=  (1 << PORTB1);
////        PORTMOTOR &= ~(1 << PORTB2);
////        PORTMOTOR |=  (1 << PORTB3);
////        PORTMOTOR &= ~(1 << PORTB4);
////        _delay_ms(500);
//       printf("ADC START: %d\n", ADC);
//       _delay_ms(500);
////       // your application logic
////       turn_cw();
////       _delay_ms(250);
////       stop_movement();
////       _delay_ms(1000);
////       printf("ADC END: %d\n", ADC);
//   }
//}