const unsigned int sevenSegmentNumber[10] = { B11111100,
                                              B01100000,
                                              B11011010,
                                              B11110010,
                                              B01100110,
                                              B10110110,
                                              B10111110,
                                              B11100000,
                                              B11111110,
                                              B11100110
                                            };
const unsigned int sevenSegmentS = B10110111;
const unsigned int sevenSegmentC = B10011101;
const unsigned int sevenSegmentO = B11000111;

enum OperationMode {SPEEDOMETER = 0, ODOMETER, CHRONOMETER, BATTERY} operationMode = BATTERY;

enum ChronometerMode {RUNNING, PAUSED} chronometerMode = PAUSED;

const int dataPin = 4;
const int clockPin = 5;
const int hallPin = 2;
const int latchPin = 7;
const int buzzerPin = 10;

const int buttonPins[2] = {A1, A0};
const int multiplexDigitPins[4] = {6, 9, 10, 11};

boolean buttonPressed[2] = {false, false};

int currentDigit = 0x01;

int magnetTicks = 0;

unsigned long longPressTime = 0;

boolean recentlyChangedOperation = false;

unsigned long letterDisplayTime = 0;

boolean chronometerJustReset = false;

unsigned long chronometerStartTime = 0;
unsigned long chronometerLastTime = 0;
unsigned long chronometerActualTime = 0;

unsigned long debounceTime = 0;

boolean currentMultiplexedPin = false;
unsigned long multiplexDelay = 50000;

const int numberOfTeeth = 8;
const unsigned int maxMagnetTicks = 60000;

const int samplePeriod = 1000;

const int numberOfReadings = 1;
int smoothingReadings[numberOfReadings];
unsigned long readingTotal = 0;
int readingIndex = 0;

boolean magnetDot = false;

unsigned long multiplexingTimer = 0;

unsigned long extraChronometerTime = 0;


unsigned long speedValue = 0;
unsigned long chronometerValue = 0;
double odometerValue = 0;

unsigned long eepromTimer = 0;
unsigned long eepromDelay = 5000;

const int debounceDelay = 250;

int circleIterator = 0;

unsigned long batteryTimer = 0;

int lastVoltage = 0;

unsigned long lastDifferentialTime = millis();
unsigned long currentDifferentialTime = lastDifferentialTime;
unsigned int differentialDelta = 0;
unsigned long differentialSpeed = 0;
unsigned long differentialReadings[5];



void setup() {

  Serial.begin(9600);

  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  //digitalWrite(hallPin, HIGH);

  for (int i = 0; i < 4; i++) {
    pinMode(multiplexDigitPins[i], OUTPUT);
  }

  for (int i = 0; i < 2; i++) {
    digitalWrite(buttonPins[i], HIGH);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(hallPin), magnetDetected, CHANGE);

  cli();
  //Timer
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  // 5[Hz]
  OCR1A = 15624;//3214; //3214; //15624 = 1 Hz, 3214 = 5Hz
  //CTC Mode
  TCCR1B |= (1 << WGM12);
  //1024 Prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  //Comparation INterrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();

  //  Serial.print("loaded odometer:");
  //  Serial.println(EEPROM.read(0) * 256 + EEPROM.read(1));
  //  odometerValue = EEPROM.read(0) * 256 + EEPROM.read(1);

  //  Serial.print("loaded chronometer:");
  //
  //  extraChronometerTime = ((unsigned long)(EEPROM.read(2) * 256 + EEPROM.read(3)) * 1000);
  //  Serial.println(extraChronometerTime);
}

void loop() {

  intro();

  core();
}

void intro() {
  while (multiplexDelay > 4400) {
    delayMicroseconds(multiplexDelay);
    multiplexDelay -= 240;
    multiplexSevenSegment(15, 0x04);
  }
  multiplexDelay = 4400;
}

void core() {

  while (true) {
    pollOperationButton();

    switch (operationMode) {
      case SPEEDOMETER:
        pollSpeedometerButton();
        multiplexSevenSegment(differentialSpeed/*speedValue*/, 0x04 | (magnetDot ? 0x08 : 0x00));
        break;
      case ODOMETER:
        pollOdometerButton();
        multiplexSevenSegment(((int)(odometerValue) / 10), 0x02);
        break;
      case CHRONOMETER:
        pollChronometerButton();
        multiplexSevenSegment(getChronometerValue(), getChronometerDecimals());
        break;
      case BATTERY:
        lastVoltage = readVcc();
        multiplexSevenSegment(0.76923*lastVoltage - 2307.7 , 0x04);
        //Serial.println(lastVoltage);

    }
  }
}

long readVcc() {
  if (millis() - batteryTimer > 1000) {
    batteryTimer = millis();

    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

    result = 1105508 / result; // Calculate Vcc (in mV);1105508 =1,1*(3,91/3,98)*1023*1000
    return result; // Vcc in millivolts
  }
}

//void eepromSave() {
//
//  Serial.println("start");
//  unsigned char firstByteOdometer = ((int)(odometerValue)) / 256;
//  unsigned char secondByteOdometer = ((int)(odometerValue)) % 256;
//  EEPROM.write(0, firstByteOdometer);
//  EEPROM.write(1, secondByteOdometer);
//
//  unsigned long chronometerSeconds = ((chronometerLastTime - chronometerStartTime) + extraChronometerTime) / 1000;
//
//  unsigned char firstByteChronometer = ((int)(chronometerSeconds) / 256);
//  unsigned char secondByteChronometer = ((int)(chronometerSeconds) % 256);
//  EEPROM.write(2, firstByteChronometer);
//  EEPROM.write(3, secondByteChronometer);
//
//  Serial.println((unsigned long)(chronometerSeconds));
//  Serial.println((int)firstByteChronometer);
//  Serial.println((int)secondByteChronometer);
//  Serial.println((firstByteChronometer * 256 + secondByteChronometer));
//
//  eepromTimer = millis();
//}

void magnetDetected() {
  magnetTicks = (magnetTicks + 1) % maxMagnetTicks;
  circleIterator = (circleIterator + 1) % 6;
  magnetDot = !magnetDot;
  odometerValue += 0.267035; //(2 * pi * 0.34) /8 [m]
  calculateDeltaSpeed();
}

ISR(TIMER1_COMPA_vect) {

  //Removes the oldest value from the total and then adds the newest value
  readingTotal -= smoothingReadings[readingIndex];
  smoothingReadings[readingIndex] = magnetTicks;
  readingTotal += smoothingReadings[readingIndex];

  readingIndex = ((readingIndex + 1) % numberOfReadings);

  //Total is multiplied by 10 for a 1 extra decimal precision.
  //Then, the total number is divided by the number of readings, so that we can get the average readings.
  //The average is then multiplied by a 1s period multiplier, so that we can get the average per second.
  //This frequency is then divided by the number of teeth so that we can get the rotation frequency.
  double doubleReading = ((double)readingTotal * 10) * 7.690619 / numberOfReadings * (1000 / samplePeriod) / numberOfTeeth;

  speedValue = ((int)doubleReading);

  magnetTicks = 0;
}

void calculateDeltaSpeed() {
    currentDifferentialTime = millis();
  
    differentialDelta = currentDifferentialTime - lastDifferentialTime;

    lastDifferentialTime = currentDifferentialTime;

    differentialSpeed = ((10000.0 * 2 * 3.14 * 0.34 / 8) / (differentialDelta)) * 3.6;

    Serial.println(differentialDelta);
    //Serial.println(differentialSpeed);
    
}

void pollOperationButton() {
  if (!buttonPressed[0] && !digitalRead(buttonPins[0])) {

    buttonPressed[0] = true;
    debounceTime = millis();
  }
  else if (buttonPressed[0] && digitalRead(buttonPins[0]) && (millis() - debounceTime) > debounceDelay) {
    changeOperationMode();
    buttonPressed[0] = false;
  }
}

void pollSpeedometerButton() {
  if (!buttonPressed[1] && !digitalRead(buttonPins[1])) {
    buttonPressed[1] = true;
    debounceTime = millis();
  }
  else if (buttonPressed[1] && digitalRead(buttonPins[1]) && (millis() - debounceTime) > debounceDelay) {
    circleIterator = (circleIterator + 1) % 6;
    //Serial.println(circleIterator);
    buttonPressed[1] = false;
  }
  //  else if (buttonPressed[1] && digitalRead(buttonPins[1])) {
  //    buttonPressed[1] = false;
  //    if (millis() - eepromTimer > eepromDelay) {
  //      //eepromSave();
  //      circleIterator = (circleIterator + 1) % 6;
  //      Serial.println(circleIterator);
  //      eepromTimer = millis();
  //    }
  //  }
}

void pollChronometerButton() {

  if (!buttonPressed[1] && !digitalRead(buttonPins[1])) {
    longPressTime = millis();
    buttonPressed[1] = true;
    debounceTime = millis();
  }
  else if (buttonPressed[1] && digitalRead(buttonPins[1]) && (millis() - debounceTime) > debounceDelay ) {
    switch (chronometerMode) {
      case PAUSED:
        if (!chronometerJustReset) {
          chronometerStartTime += (millis() - chronometerLastTime);
          chronometerMode = RUNNING;
        }
        else {
          chronometerJustReset = false;
        }
        break;
      case RUNNING:
        chronometerMode = PAUSED;
        break;
    }
    buttonPressed[1] = false;
  }
  else if (buttonPressed[1] && !digitalRead(buttonPins[1])) {
    if (millis() - longPressTime >= 2000) {
      if (chronometerMode == PAUSED) {
        extraChronometerTime = 0;
        chronometerStartTime = chronometerLastTime;
        chronometerJustReset = true;
      }
    }
  }
}

void pollOdometerButton() {
  if (!buttonPressed[1] && !digitalRead(buttonPins[1])) {
    longPressTime = millis();
    buttonPressed[1] = true;
    debounceTime = millis();
  }
  else if (buttonPressed[1] && digitalRead(buttonPins[1]) && (millis() - debounceTime) > debounceDelay ) {
    buttonPressed[1] = false;
  }
  else if (buttonPressed[1] && !digitalRead(buttonPins[1])) {
    if (millis() - longPressTime >= 2000) {
      odometerValue = 0.0;
    }

  }
}


int getChronometerValue() {
  unsigned long chronometerRawValue = extraChronometerTime;
  switch (chronometerMode) {
    case PAUSED:
      chronometerRawValue += chronometerLastTime - chronometerStartTime;
      break;
    case RUNNING:
      chronometerRawValue += millis() - chronometerStartTime;
      chronometerLastTime = millis();
      break;
  }

  unsigned long minutes = chronometerRawValue / 60000;

  chronometerValue = minutes * 100;

  unsigned long seconds = (chronometerRawValue / 1000) % 60;

  chronometerValue += seconds;

  return chronometerValue;
}

int getChronometerDecimals() {
  return 0x02 | (chronometerMode == RUNNING ? 0x08 : 0x00);
}
void multiplexSevenSegment(short numberToDisplay, char decimals) {


  if ((micros() - multiplexingTimer) < multiplexDelay) {
    return;
  } else {
    multiplexingTimer = micros();
  }



  if (numberToDisplay > 9999) numberToDisplay = 9999;

  digitalWrite(latchPin, LOW);

  for (int i = 0; i < 4; i++) {
    digitalWrite(multiplexDigitPins[i], LOW);
  }

  if ((currentDigit & 0x01) == 0x01) {
    if (recentlyChangedOperation && (millis() - letterDisplayTime < 1000)) {
      if (operationMode == SPEEDOMETER) {
        shiftOut(dataPin, clockPin, LSBFIRST, ~sevenSegmentS);
      }
      else if (operationMode == ODOMETER) {
        shiftOut(dataPin, clockPin, LSBFIRST, ~sevenSegmentO);
      }
      else if (operationMode == CHRONOMETER) {
        shiftOut(dataPin, clockPin, LSBFIRST, ~sevenSegmentC);
      }
      else if (operationMode == BATTERY) {
        shiftOut(dataPin, clockPin, LSBFIRST, 0xFF);
      }
    } else {
      recentlyChangedOperation = false;
      if (operationMode == SPEEDOMETER) {
        shiftOut(dataPin, clockPin, LSBFIRST, ~(1 << (circleIterator + 2)));
      } else {
        shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[numberToDisplay / 1000] | ((decimals & 0x01) == 0x01 ? 0x01 : 0x00)));
      }
    }

  }
  else if ((currentDigit & 0x02) == 0x02) {
    shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[(numberToDisplay / 100) % 10] | ((decimals & 0x02) == 0x02 ? 0x01 : 0x00)));
  }
  else if ((currentDigit & 0x04) == 0x04) {
    shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[(numberToDisplay / 10) % 10] | ((decimals & 0x04) == 0x04 ? 0x01 : 0x00)));
  }
  else if ((currentDigit & 0x08) == 0x08) {
    shiftOut(dataPin, clockPin, LSBFIRST, ~(sevenSegmentNumber[numberToDisplay % 10] | ((decimals & 0x08) == 0x08 ? 0x01 : 0x00)));
  }

  digitalWrite(latchPin, HIGH);
  for (int i = 0; i < 4; i++) {
    digitalWrite(multiplexDigitPins[i], (currentDigit & (1 << i)) == (1 << i));
  }

  currentDigit = (currentDigit << 1);
  if (currentDigit > 0x08) currentDigit = 0x01;
  //delay(multiplexDelay);
}

void multiplexSevenSegmentIntro() {
  if ((micros() - multiplexingTimer) < multiplexDelay) {
    return;
  } else {
    multiplexingTimer = micros();
  }

  digitalWrite(latchPin, LOW);

  for (int i = 0; i < 4; i++) {
    digitalWrite(multiplexDigitPins[i], LOW);
  }

  if ((currentDigit & 0x01) == 0x01) {
    shiftOut(dataPin, clockPin, LSBFIRST, ~(0B10011100));
  }
  else if ((currentDigit & 0x02) == 0x02) {
    shiftOut(dataPin, clockPin, LSBFIRST, ~(10010000));
  }
  else if ((currentDigit & 0x04) == 0x04) {
    shiftOut(dataPin, clockPin, LSBFIRST, ~(10010000));
  }
  else if ((currentDigit & 0x08) == 0x08) {
    shiftOut(dataPin, clockPin, LSBFIRST, ~(11110000));
  }

  digitalWrite(latchPin, HIGH);
  for (int i = 0; i < 4; i++) {
    digitalWrite(multiplexDigitPins[i], (currentDigit & (1 << i)) == (1 << i));
  }

  currentDigit = (currentDigit << 1);
  if (currentDigit > 0x08) currentDigit = 0x01;
}

void changeOperationMode() {

  recentlyChangedOperation = true;
  letterDisplayTime = millis();

  switch (operationMode) {
    case SPEEDOMETER:
      operationMode = ODOMETER;
      break;
    case ODOMETER:
      operationMode = CHRONOMETER;
      break;
    case CHRONOMETER:
      operationMode = BATTERY;
      break;
    case BATTERY:
      operationMode = SPEEDOMETER;
      break;
  }
}

