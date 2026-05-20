#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// =====================
// PIN MAP
// =====================
// D3  = Emergency button, INT1
// D4  = Ultrasonic TRIG
// D8  = Ultrasonic ECHO, ICP1
// D10 = Servo, OC1B
// D11 = Buzzer, OC2A
// D12 = Red LED
// D13 = Green LED
// A0  = Potentiometer

// =====================
// SYSTEM STATES
// =====================

enum CrossingState {
  IDLE,
  CROSSING_ACTIVE,
  SAFE_TO_OPEN,
  EMERGENCY_STOP,
  FAULT
};

volatile CrossingState currentState = IDLE;

// =====================
// GLOBAL VARIABLES
// =====================

volatile unsigned long msTicks = 0;

volatile bool emergencyInterruptFlag = false;
volatile bool emergencyToggleRequest = false;

bool trainPresent = false;
bool emergencyActive = false;

unsigned long lastTrainSeenTime = 0;
unsigned long lastFlashTime = 0;
unsigned long lastDebugTime = 0;
unsigned long lastUltrasonicTrigger = 0;
unsigned long lastEmergencyPressTime = 0;

bool redLedState = false;

unsigned int adcValue = 0;
unsigned long safetyDelay = 3000;

volatile char rxCommand = '\0';

#define TX_BUFFER_SIZE 64
volatile char txBuffer[TX_BUFFER_SIZE];
volatile uint8_t txHead = 0;
volatile uint8_t txTail = 0;

volatile uint16_t captureStart = 0;
volatile uint16_t captureEnd = 0;
volatile uint16_t captureTicks = 0;
volatile bool captureDone = false;
volatile bool waitingForFallingEdge = false;

float distanceCm = 999.0;

// =====================
// FUNCTION PROTOTYPES
// =====================

void setupPins();
void setupTimer0_CTC();
void setupTimer1_FastPWM_ServoAndInputCapture();
void setupTimer2_PhaseCorrectPWM_Buzzer();
void setupExternalInterrupts();
void setupADC();
void setupUSART();

unsigned long getMillis();
unsigned int readADC();

void raiseGate();
void lowerGate();
void setServoPulseMicroseconds(int us);
void startBuzzer();
void stopBuzzer();

void triggerUltrasonic();
void processInputCapture();

void readInputs();
void updateCrossingState();
void updateOutputs();
void flashRedLED();

void handleSerialCommand();
void printDebugInfo();

void usartSendChar(char c);
void usartSendString(const char *str);
void usartSendNumber(int num);

// =====================
// MAIN PROGRAM
// =====================

int main(void) {
  cli();

  setupPins();
  setupTimer0_CTC();
  setupTimer1_FastPWM_ServoAndInputCapture();
  setupTimer2_PhaseCorrectPWM_Buzzer();
  setupExternalInterrupts();
  setupADC();
  setupUSART();

  sei();

  usartSendString("Railway Crossing System Started\r\n");

  while (1) {
    triggerUltrasonic();
    processInputCapture();

    readInputs();
    updateCrossingState();
    updateOutputs();

    handleSerialCommand();
    printDebugInfo();
  }

  return 0;
}

// =====================
// TIME FUNCTION
// =====================

unsigned long getMillis() {
  unsigned long t;

  cli();
  t = msTicks;
  sei();

  return t;
}

// =====================
// PIN SETUP
// =====================

void setupPins() {
  // D3 = PD3 = emergency button input
  DDRD &= ~(1 << PD3);
  PORTD |= (1 << PD3);   // internal pull-up enabled

  // D4 = PD4 = ultrasonic trigger
  DDRD |= (1 << PD4);
  PORTD &= ~(1 << PD4);

  // D8 = PB0 = ultrasonic echo / ICP1
  DDRB &= ~(1 << PB0);

  // D10 = PB2 = servo output
  DDRB |= (1 << PB2);

  // D11 = PB3 = buzzer output
  DDRB |= (1 << PB3);

  // D12 = PB4 = red LED output
  DDRB |= (1 << PB4);

  // D13 = PB5 = green LED output
  DDRB |= (1 << PB5);
}

// =====================
// TIMER0 CTC MODE
// 1 ms system tick
// =====================

void setupTimer0_CTC() {
  TCCR0A = 0;
  TCCR0B = 0;

  TCCR0A |= (1 << WGM01);
  OCR0A = 249;

  TCCR0B |= (1 << CS01) | (1 << CS00);
  TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
  msTicks++;
}

// =====================
// TIMER1 FAST PWM + INPUT CAPTURE
// Servo = D10 / OC1B
// Echo  = D8 / ICP1
// =====================

void setupTimer1_FastPWM_ServoAndInputCapture() {
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;

  // Fast PWM mode 15, TOP = OCR1A
  TCCR1A |= (1 << WGM10) | (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // Non-inverting PWM on OC1B
  TCCR1A |= (1 << COM1B1);

  // Prescaler = 8
  TCCR1B |= (1 << CS11);

  // 50 Hz servo PWM
  // 16 MHz / 8 = 2 MHz
  // 20 ms = 40000 counts
  OCR1A = 40000;

  // Input capture starts on rising edge
  TCCR1B |= (1 << ICES1);
  TIMSK1 |= (1 << ICIE1);

  raiseGate();
}

void setServoPulseMicroseconds(int us) {
  OCR1B = us * 2;
}

void raiseGate() {
  setServoPulseMicroseconds(1400);
}

void lowerGate() {
  setServoPulseMicroseconds(600);
}

ISR(TIMER1_CAPT_vect) {
  if (!waitingForFallingEdge) {
    captureStart = ICR1;
    TCCR1B &= ~(1 << ICES1);
    waitingForFallingEdge = true;
  } 
  else {
    captureEnd = ICR1;

    if (captureEnd >= captureStart) {
      captureTicks = captureEnd - captureStart;
    } 
    else {
      captureTicks = (40000 - captureStart) + captureEnd;
    }

    captureDone = true;

    TCCR1B |= (1 << ICES1);
    waitingForFallingEdge = false;
  }
}

// =====================
// TIMER2 PHASE-CORRECT PWM
// Buzzer = D11 / OC2A
// =====================

void setupTimer2_PhaseCorrectPWM_Buzzer() {
  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2A |= (1 << WGM20);
  TCCR2A |= (1 << COM2A1);

  TCCR2B |= (1 << CS22);

  OCR2A = 0;
}

void startBuzzer() {
  OCR2A = 128;
}

void stopBuzzer() {
  OCR2A = 0;
}

// =====================
// EXTERNAL INTERRUPT
// INT1 = D3 emergency button
// =====================

void setupExternalInterrupts() {
  EICRA = 0;

  // INT1 falling edge
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);

  EIMSK |= (1 << INT1);
}

ISR(INT1_vect) {
  emergencyInterruptFlag = true;
}

// =====================
// ADC A0
// =====================

void setupADC() {
  ADMUX = 0;
  ADMUX |= (1 << REFS0);

  ADCSRA = 0;
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

unsigned int readADC() {
  ADCSRA |= (1 << ADSC);

  while (ADCSRA & (1 << ADSC));

  return ADC;
}

// =====================
// USART INTERRUPT RX/TX
// =====================

void setupUSART() {
  uint16_t ubrr = 103;

  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;

  UCSR0B = 0;
  UCSR0C = 0;

  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

ISR(USART_RX_vect) {
  rxCommand = UDR0;
}

ISR(USART_UDRE_vect) {
  if (txHead == txTail) {
    UCSR0B &= ~(1 << UDRIE0);
  } 
  else {
    UDR0 = txBuffer[txTail];
    txTail = (txTail + 1) % TX_BUFFER_SIZE;
  }
}

void usartSendChar(char c) {
  uint8_t nextHead = (txHead + 1) % TX_BUFFER_SIZE;

  while (nextHead == txTail);

  txBuffer[txHead] = c;
  txHead = nextHead;

  UCSR0B |= (1 << UDRIE0);
}

void usartSendString(const char *str) {
  while (*str) {
    usartSendChar(*str++);
  }
}

void usartSendNumber(int num) {
  char buffer[12];
  itoa(num, buffer, 10);
  usartSendString(buffer);
}

// =====================
// ULTRASONIC SENSOR
// =====================

void triggerUltrasonic() {
  unsigned long now = getMillis();

  if (now - lastUltrasonicTrigger >= 200) {
    lastUltrasonicTrigger = now;

    PORTD |= (1 << PD4);
    _delay_us(10);
    PORTD &= ~(1 << PD4);
  }
}

void processInputCapture() {
  if (captureDone) {
    cli();
    uint16_t ticks = captureTicks;
    captureDone = false;
    sei();

    float pulseUs = ticks * 0.5;
    distanceCm = pulseUs / 58.0;
  }
}

// =====================
// INPUT READING
// =====================

void readInputs() {
  adcValue = readADC();

  // Potentiometer maps safety delay from 1s to 6s
  safetyDelay = 1000 + ((unsigned long)adcValue * 5000UL / 1023UL);

  // Ultrasonic train detection threshold
  trainPresent = (distanceCm > 2.0 && distanceCm < 20.0);

  // Emergency button toggle with debounce
  if (emergencyInterruptFlag) {
    emergencyInterruptFlag = false;

    unsigned long now = getMillis();

    if (now - lastEmergencyPressTime > 300) {
      lastEmergencyPressTime = now;
      emergencyToggleRequest = true;
    }
  }

  if (emergencyToggleRequest) {
    emergencyToggleRequest = false;

    if (!emergencyActive) {
      emergencyActive = true;
      currentState = EMERGENCY_STOP;
      usartSendString("EMERGENCY STOP ACTIVE\r\n");
    } 
    else {
      if (!trainPresent) {
        emergencyActive = false;
        currentState = IDLE;
        usartSendString("EMERGENCY STOP RESET\r\n");
      } 
      else {
        usartSendString("RESET BLOCKED: TRAIN DETECTED\r\n");
      }
    }
  }
}

// =====================
// STATE MACHINE
// =====================

void updateCrossingState() {
  unsigned long now = getMillis();

  switch (currentState) {

    case IDLE:
      if (emergencyActive) {
        currentState = EMERGENCY_STOP;
      } 
      else if (trainPresent) {
        currentState = CROSSING_ACTIVE;
        lastTrainSeenTime = now;
      }
      break;

    case CROSSING_ACTIVE:
      if (emergencyActive) {
        currentState = EMERGENCY_STOP;
      } 
      else if (trainPresent) {
        lastTrainSeenTime = now;
      } 
      else if (now - lastTrainSeenTime >= safetyDelay) {
        currentState = SAFE_TO_OPEN;
      }
      break;

    case SAFE_TO_OPEN:
      currentState = IDLE;
      break;

    case EMERGENCY_STOP:
      if (!emergencyActive && !trainPresent) {
        currentState = IDLE;
      }
      break;

    case FAULT:
      break;
  }
}

// =====================
// OUTPUT CONTROL
// =====================

void updateOutputs() {
  switch (currentState) {

    case IDLE:
      raiseGate();
      stopBuzzer();

      PORTB |= (1 << PB5);    // green ON
      PORTB &= ~(1 << PB4);   // red OFF
      break;

    case CROSSING_ACTIVE:
      lowerGate();
      startBuzzer();

      PORTB &= ~(1 << PB5);   // green OFF
      flashRedLED();
      break;

    case SAFE_TO_OPEN:
      raiseGate();
      stopBuzzer();

      PORTB |= (1 << PB5);    // green ON
      PORTB &= ~(1 << PB4);   // red OFF
      break;

    case EMERGENCY_STOP:
    case FAULT:
      lowerGate();
      startBuzzer();

      PORTB &= ~(1 << PB5);   // green OFF
      PORTB |= (1 << PB4);    // red ON
      break;
  }
}

void flashRedLED() {
  unsigned long now = getMillis();

  if (now - lastFlashTime >= 500) {
    lastFlashTime = now;
    redLedState = !redLedState;

    if (redLedState) {
      PORTB |= (1 << PB4);
    } 
    else {
      PORTB &= ~(1 << PB4);
    }
  }
}

// =====================
// SERIAL COMMANDS
// =====================

void handleSerialCommand() {
  if (rxCommand == '\0') return;

  char cmd = rxCommand;
  rxCommand = '\0';

  if (cmd == 'R' || cmd == 'r') {
    if (!trainPresent) {
      emergencyActive = false;
      currentState = IDLE;
      usartSendString("RESET\r\n");
    } 
    else {
      usartSendString("RESET BLOCKED: TRAIN DETECTED\r\n");
    }
  }

  else if (cmd == 'C' || cmd == 'c') {
    currentState = CROSSING_ACTIVE;
    lastTrainSeenTime = getMillis();
    usartSendString("MANUAL CLOSE\r\n");
  }

  else if (cmd == 'O' || cmd == 'o') {
    if (!trainPresent && !emergencyActive) {
      currentState = SAFE_TO_OPEN;
      usartSendString("MANUAL OPEN\r\n");
    } 
    else {
      usartSendString("OPEN BLOCKED\r\n");
    }
  }

  else if (cmd == 'E' || cmd == 'e') {
    emergencyActive = true;
    currentState = EMERGENCY_STOP;
    usartSendString("EMERGENCY STOP\r\n");
  }
}

// =====================
// DEBUG PRINTING
// =====================

void printDebugInfo() {
  unsigned long now = getMillis();

  if (now - lastDebugTime >= 1000) {
    lastDebugTime = now;

    usartSendString("State: ");

    switch (currentState) {
      case IDLE:
        usartSendString("IDLE");
        break;

      case CROSSING_ACTIVE:
        usartSendString("CROSSING_ACTIVE");
        break;

      case SAFE_TO_OPEN:
        usartSendString("SAFE_TO_OPEN");
        break;

      case EMERGENCY_STOP:
        usartSendString("EMERGENCY_STOP");
        break;

      case FAULT:
        usartSendString("FAULT");
        break;
    }

    usartSendString(" | Train: ");
    usartSendString(trainPresent ? "YES" : "NO");

    usartSendString(" | Emergency: ");
    usartSendString(emergencyActive ? "YES" : "NO");

    usartSendString(" | Distance(cm): ");
    usartSendNumber((int)distanceCm);

    usartSendString(" | ADC: ");
    usartSendNumber(adcValue);

    usartSendString(" | Delay(ms): ");
    usartSendNumber((int)safetyDelay);

    usartSendString("\r\n");
  }
}
