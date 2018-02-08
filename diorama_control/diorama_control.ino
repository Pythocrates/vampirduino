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

// Define pin connections.
int cat_detector_pin = 2;


void setup() {
    // Set up pins.
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(cat_detector_pin, INPUT); 
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
}

ISR (TIMER1_COMPA_vect) {
    digitalWrite(LED_BUILTIN, LOW);
    TIMSK1 = 0; // Disable interrupt
}

void on_cat_detected() {
    digitalWrite(LED_BUILTIN, HIGH);
    //TIMSK1 = 0; // Disable interrupt
    TCNT1 = 0;
    OCR1A = 31250; // set 0.5sec timer
    TIMSK1 = bit(OCIE1A); // Set interrupt
}

void loop() {
}

