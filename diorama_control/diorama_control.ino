/* Infrared obstacle sensor module test project
 */

int cat_detector_pin = 2;
int cat_found;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(cat_detector_pin, INPUT); 
}

bool is_cat_close() {
  return (! digitalRead(cat_detector_pin));
}

void loop() {
    cat_found = is_cat_close();
    digitalWrite(LED_BUILTIN, cat_found);
    delay(100);
}
