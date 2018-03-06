/*
 * Vampire diorama control project.
 * Detect a cat using distance sensor with digital output.
 * Actuate one door.
 * Actuate one person.
 * Switch on/off light.
 * Use timers to switch off stuff.
 * Use interrupt to react to cat detection.
 */

class Door {
public:
    class Motion {
    public:
        typedef int Type;
        static const Type NONE = 0;
        static const Type CW = 1 << 0;
        static const Type CCW = 1 << 1;
    };

    Motion::Type motion = Motion::NONE;

    void move(const Motion::Type & motion) {
        this->motion = motion;
        Serial.println("move");
    }

    void stop() {
        motion = Motion::NONE;
        Serial.println("stop");
    }

    boolean isMoving() const {
        return Motion::NONE != motion;
    };
};


#include <Stepper.h>

const int STEPS_PER_MOTOR_REVOLUTION = 32;  // change this to fit the number of steps per revolution
const int GEAR_REDUCTION_FACTOR = 64;
const int STEPS_PER_REVOLUTION = 2048;  // change this to fit the number of steps per revolution
const int STEPS_PER_ITERATION = 128;
Stepper myStepper(STEPS_PER_MOTOR_REVOLUTION, 9, 11, 10, 12);
int stepCount = 0;

// door motion
Door door = Door();

// Define pin connections.
int cat_detector_pin = 2;
int red_light_pin = 8;

void setup() {
    // Set up pins.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(cat_detector_pin, INPUT); 
    pinMode(red_light_pin, OUTPUT);
    
    // The detection means output going low.

    Serial.begin(9600);
    Serial.println("Delay...");
    delay(2000);
    Serial.println("... done!");
    noInterrupts();
    EIFR |= (1 << INTF0); // Clear pending interrupts.
    attachInterrupt(digitalPinToInterrupt(cat_detector_pin), on_cat_detected, FALLING);

    TCCR1A = 0; // clear registers
    TCCR1B = 0;
    TCNT1 = 0;
  
    OCR1A = 31250; // set 2sec timer
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS12); // prescaler 1024 and CTC mode
    TIMSK1 = 0; // Disable interrupt
    //TIMSK1 = bit(OCIE1A); // Set interrupt
    interrupts();

    myStepper.setSpeed(400);
}

ISR(TIMER1_COMPA_vect) {
    digitalWrite(LED_BUILTIN, LOW);
    TIMSK1 = 0; // Disable interrupt
    door.move(Door::Motion::CCW);
    Serial.println("TMR");
}

void on_cat_detected() {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(red_light_pin, HIGH);
    if (! door.isMoving()) {
        door.move(Door::Motion::CW);
        //noInterrupts();
        Serial.println("Cat");
    }
}

void loop() {
    if (door.isMoving()) {
        //myStepper.step(door_direction * STEPS_PER_ITERATION);
        myStepper.step(door.motion == Door::Motion::CW ? STEPS_PER_ITERATION : -STEPS_PER_ITERATION);
        stepCount += STEPS_PER_ITERATION;
        Serial.println(stepCount);
        if (stepCount >= .5 * STEPS_PER_REVOLUTION) {
            //door_moving = false;
            boolean open_reached = (door.motion == Door::Motion::CW);
            door.stop();
            //door_direction = -door_direction;
            
            stepCount = 0;

            //if (door_direction < 0) {
            if (open_reached) {
                // Start timer.
                Serial.println("Starting timer");

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

                //interrupts();
            } else {
                digitalWrite(red_light_pin, LOW);
                //interrupts();
            }
        } else {
            // While moving, do not react to cat.
            //noInterrupts();
        }
    }
    
    delay(20);
}

