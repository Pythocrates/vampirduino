/*
 * Vampire diorama control project.
 * Detect a cat using distance sensor with digital output.
 * Actuate one door.
 * Actuate one person?
 * Actuate one garage door?
 * Switch on/off light?
 * Use timers to switch off stuff.
 * Use interrupt to react to cat detection.
 * Play a melody with a piezo?
 */

#include <Stepper.h>

const int STEPS_PER_MOTOR_REVOLUTION = 32;  // change this to fit the number of steps per revolution
const int GEAR_REDUCTION_FACTOR = 64;
const int STEPS_PER_REVOLUTION = 2048;  // change this to fit the number of steps per revolution
const int STEPS_PER_ITERATION = 128;
Stepper myStepper(STEPS_PER_MOTOR_REVOLUTION, 9, 11, 10, 12);
int stepCount = 0;

// door motion
int door_moving = false;
int door_direction = 1;

// Define pin connections.
int cat_detector_pin = 2;
int red_light_pin = 8;


void setup() {
    // Set up pins.
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(cat_detector_pin, INPUT); 
    pinMode(red_light_pin, OUTPUT);
    
    // The detection means output going low.
    attachInterrupt(digitalPinToInterrupt(cat_detector_pin), on_cat_detected, FALLING);

    noInterrupts();
    TCCR1A = 0; // clear registers
    TCCR1B = 0;
    TCNT1 = 0;
  
    OCR1A = 31250; // set 2sec timer
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS12); // prescaler 1024 and CTC mode
    TIMSK1 = 0; // Disable interrupt
    //TIMSK1 = bit(OCIE1A); // Set interrupt
    interrupts();

    Serial.begin(9600);
    myStepper.setSpeed(400);
}

ISR(TIMER1_COMPA_vect) {
        Serial.println("ISR");
    digitalWrite(LED_BUILTIN, LOW);
    TIMSK1 = 0; // Disable interrupt
    door_moving = true;
}

void on_cat_detected() {
    digitalWrite(LED_BUILTIN, HIGH);
    //TIMSK1 = 0; // Disable interrupt
    TCNT1 = 0;
    OCR1A = 31250; // set 0.5sec timer
    TIMSK1 = bit(OCIE1A); // Set interrupt
    door_moving = true;
    digitalWrite(red_light_pin, HIGH);
}

void loop() {
    if (door_moving) { 
        myStepper.step(door_direction * STEPS_PER_ITERATION);
        stepCount += STEPS_PER_ITERATION;
        Serial.println(stepCount);
        if (stepCount > .25 * STEPS_PER_REVOLUTION) {
            door_moving = false;
            door_direction = -door_direction;
            stepCount = 0;

if (door_direction < 0) {
// Start timer.
Serial.println("Starting timer");
    noInterrupts();
    TIMSK1 = 0;
    TCNT1 = 0;
    TCCR1A = 0;
    //TCCR1B = 0;
TCCR1B &= ~(1 << CS10);
TCCR1B &= ~(1 << CS12);

TIFR1 = bit (OCF2A); // clear any pending interrupt: https://arduino.stackexchange.com/questions/22432/irregularly-triggering-an-isr-using-timers

    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS12); // prescaler 1024 and CTC mode
    OCR1A = 31250; // set 0.5sec timer
    Serial.println(TCNT1);
    TIMSK1 = bit(OCIE1A); // Set interrupt
    interrupts();
} else {
  digitalWrite(red_light_pin, LOW);
  }
        }
        
    }
    
    delay(20);
}

