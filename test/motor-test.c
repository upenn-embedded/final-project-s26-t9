#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

/*
 * ATmega328PB PWM Pin Mapping:
 * Timer0 (8-bit): OC0A = PD6, OC0B = PD5
 * Timer1 (16-bit): OC1A = PB1, OC1B = PB2
 *
 * Mode: Fast PWM 8-bit
 * Freq @ 16MHz, prescaler=64: ~977 Hz — suitable for vibration motors
 */

#define F_CPU 16000000UL

void
timer0_pwm_init(void) {
    // Set OC0A (PD6) and OC0B (PD5) as outputs
    DDRD |= (1 << DDD6) | (1 << DDD5);

    // Fast PWM 8-bit: WGM02:0 = 011 (TOP = 0xFF)
    // Non-inverting on OC0A and OC0B: COM0A1=1, COM0B1=1
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);   // Prescaler = 64

    // Initial duty cycle: 0 (motors off)
    OCR0A = 255;
    OCR0B = 0;
}

void
timer1_pwm_init(void) {
    // Set OC1A (PB1) and OC1B (PB2) as outputs
    DDRB |= (1 << DDB1) | (1 << DDB2);

    // Fast PWM 8-bit: WGM13:0 = 0101 (TOP = 0xFF)
    // Non-inverting on OC1A and OC1B: COM1A1=1, COM1B1=1
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);   // Prescaler = 64

    // Initial duty cycle: 0 (motors off)
    OCR1A = 255;
    OCR1B = 0;
}

// Set motor speed: duty = 0 (off) to 255 (full speed)
void
set_motor_0A(uint8_t duty) {
    OCR0A = duty;
}
void
set_motor_0B(uint8_t duty) {
    OCR0B = duty;
}
void
set_motor_1A(uint8_t duty) {
    OCR1A = duty;
}
void
set_motor_1B(uint8_t duty) {
    OCR1B = duty;
}

void
motor_A_on(uint8_t duty) {
    OCR0A = duty;
}
void
motor_A_off(void) {
    OCR0A = 0;
}
void
motor_B_on(uint8_t duty) {
    OCR0B = duty;
}
void
motor_B_off(void) {
    OCR0B = 0;
}
void
motor_1A_on(uint8_t duty) {
    OCR1A = duty;
}
void
motor_1A_off(void) {
    OCR1A = 0;
}
void
motor_1B_on(uint8_t duty) {
    OCR1B = duty;
}
void
motor_1B_off(void) {
    OCR1B = 0;
}

int
main(void) {
    timer0_pwm_init();

    uint8_t duty_cycle = 90;

    while (1) {
        motor_A_on(duty_cycle);
        _delay_ms(1000);
        motor_A_off();

        _delay_ms(1000);

        motor_B_on(duty_cycle);
        _delay_ms(1000);
        motor_B_off();
    }
}