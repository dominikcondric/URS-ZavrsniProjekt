#include <avr/io.h>
#define F_CPU 7372800UL
#include <avr/delay.h>
#include <avr/interrupt.h>

#define NOISE_PIN 3
#define ULTRASONIC_TRIG_PIN 1
#define ULTRASONIC_ECHO_PIN 2
#define STEPPER_PORT PORTB
#define MAX_STEPS 700
#define FIRST_LEDS 12
#define SECOND_LEDS 7
#define THIRD_LEDS 4

static uint16_t distance = 0;
static uint8_t doorOpened = 0;
static uint8_t noiseDetected = 0;

void moveDoor() {
	uint8_t sequence = 0;
	for (uint16_t i = 0; i < MAX_STEPS; ++i) {
		if (doorOpened) {
			sequence = (sequence << 1) & 0x0f;
			if (sequence == 0) sequence = 1;
		} else if (!doorOpened) {
			sequence >>= 1;
			if (sequence == 0) sequence = 0x08;
		}

		STEPPER_PORT = ~sequence;
		_delay_ms(3);
	}

	if (!doorOpened) {
		doorOpened = 1;
		PORTA |= _BV(3);
	} else {
		doorOpened = 0;
		PORTA &= ~0x07;
	}
}

void readUltrasonicSensorInput() {
	uint16_t time = 0;
	PORTD &= ~_BV(ULTRASONIC_TRIG_PIN);
	_delay_us(2);
	PORTD |= _BV(ULTRASONIC_TRIG_PIN);
	_delay_us(10);
	PORTD &= ~_BV(ULTRASONIC_TRIG_PIN);

	while (bit_is_clear(PIND, ULTRASONIC_ECHO_PIN)); // Wait for the echo pin to go high,
	while (bit_is_set(PIND, ULTRASONIC_ECHO_PIN)) { // then start measuring until pin goes low
		time++;
		_delay_us(1);
	}

	distance = (time / 2) / 29; // datasheet distance formula
	if (distance <= FIRST_LEDS) {
		PORTA |= _BV(2);
	}
	if (distance <= SECOND_LEDS) {
		PORTA |= _BV(1);
	}
	if (distance <= THIRD_LEDS) {
		PORTA |= _BV(0);
		TIMSK &= ~_BV(OCIE1A);
		if (noiseDetected == 1) noiseDetected = 0;
		moveDoor();
	}
}

ISR(TIMER1_COMPA_vect) {
	readUltrasonicSensorInput();
}

void setupPins() {
	DDRB = 0x0f; // Stepper motor port - DDRB, pins 0, 1, 2, 3 set to output

	DDRA = 0xff; // Led port set to output
	PORTA = 0x00;

	// Interrupt setup
	TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
	OCR1A = 7760;
	TIMSK = _BV(OCIE1A);
	sei();

	DDRD &= ~_BV(NOISE_PIN); // Noise sensor pin set to input

	DDRD |= _BV(ULTRASONIC_TRIG_PIN); // Ultrasonic sensor trigger pin set to output
	DDRD &= ~_BV(ULTRASONIC_ECHO_PIN); // Ultrasonic sensor echo pin set to input
}

void readNoiseSensorInput() {
	if (bit_is_clear(PIND, NOISE_PIN) && doorOpened) {
		noiseDetected = 1;
		moveDoor();
		TIMSK |= _BV(OCIE1A);
	} else if (bit_is_clear(PIND, NOISE_PIN) && !doorOpened && noiseDetected == 1) {
		noiseDetected = 0;
		PORTA &= ~_BV(3);
	}
}

int main(void) {
	setupPins();
	while (1) {
		readNoiseSensorInput();
	}
}