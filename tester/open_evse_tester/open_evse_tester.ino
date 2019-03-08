#define pinB 13
#define pinC 10
#define pinD 7

#define pinPilot A0
#define pinButton 8

#define stateA 0
#define stateB 1
#define stateC 2
#define stateD 3

#define samplesAverage 20

uint8_t state = stateA;

volatile unsigned long lastChange = 0;
volatile unsigned long risingEdges[samplesAverage];
volatile unsigned long fallingEdges[samplesAverage];
volatile uint8_t i = 0;
volatile uint8_t inited = 0;
volatile uint8_t rised = 0;

volatile unsigned long freq = 0;
volatile unsigned long timeUp = 0;
volatile unsigned long timeDown = 0;

void initArrays(){
  for(i = 0; i < samplesAverage; i++){
    risingEdges[i] = 0;
    fallingEdges[i] = 0;
  }
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT1_vect) {
  lastChange = millis();                      // Not interrupt safe, non critical though
  unsigned long eventTime = micros();
  uint8_t pinState = digitalRead(pinPilot);   // Could be done faster if required
  if(pinState == HIGH){
    // Rising edge
    freq = 1000000 * samplesAverage / (eventTime - risingEdges[i]);
    if(!rised){
      rised = 1;
    }
    //timeDown += (eventTime - fallingEdges[(i - 1) % samplesAverage]);
    //timeUp -= (fallingEdges[i] - risingEdges[i]);
    //timeDown -= (fallingEdges[i] - risingEdges[i]);
    //timeUp -= (risingEdges[i] - fallingEdges[(i - 1) % samplesAverage]); 
    risingEdges[i] = eventTime;
  } else {
    if(rised){
      //timeUp += (eventTime - risingEdges[i]);
      //timeDown -= (risingEdges[(i + 1) & samplesAverage] - fallingEdges[i]);
      //timeUp -= (risingEdges[(i + 1) % samplesAverage] - fallingEdges[i]);
      //timeDown -= (fallingEdges[(i + 1) % samplesAverage] - risingEdges[(i + 1) % samplesAverage]);
      // Update time up and down here
      timeUp += (eventTime - risingEdges[i]);
      timeDown += (risingEdges[i] - fallingEdges[(i - 1) % samplesAverage]);
      if(inited){
        timeUp -= (fallingEdges[(i + 1) % samplesAverage] - risingEdges[(i + 1) % samplesAverage]);
        timeDown -= (risingEdges[(i + 1) % samplesAverage] - fallingEdges[i]); 
      } else if(i == (samplesAverage - 1)){
        inited = 1;
      }
      
      fallingEdges[i] = eventTime;
      i = (i + 1) % samplesAverage;
    }    
  }
}

void setup() {
  cli();        // Disable global interrupts
  initArrays();
  // Set state pins as output and set to low
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinPilot, INPUT);
  pinMode(pinButton, INPUT);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, LOW);

  // Setup serial
  Serial.begin(115200);

  pciSetup(pinPilot);

  sei();    // Enable interrupts
}

void process_state(uint8_t state){
  if(state == stateA){
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, LOW);
  } else if(state == stateB){
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, LOW);
    digitalWrite(pinB, HIGH);
  } else if(state == stateC){
    digitalWrite(pinB, LOW);
    digitalWrite(pinD, LOW);
    digitalWrite(pinC, HIGH);
  } else if(state == stateD){
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, HIGH);
  }
}

uint8_t lastButtonState = LOW;
uint8_t buttonState = LOW;
unsigned long buttonPressTime = 0;
unsigned long lastFreq = 0;
unsigned long freqLastSent = 0;
unsigned long lastTimeUp = 0;
unsigned long lastTimeDown = 0;

void loop() { 
  buttonState = digitalRead(pinButton);
  if(buttonState == HIGH && buttonState != lastButtonState && (millis() - buttonPressTime) > 300){
    buttonPressTime = millis();
    state++;
    if(state > stateD){
      state = stateA;
    }
    Serial.println('Changing state to: ');
    Serial.println(state);
    process_state(state);
  }
  lastButtonState = buttonState;
  
  cli();
  lastFreq = freq;
  lastTimeUp = timeUp;
  lastTimeDown = timeDown;
  sei();
  if((millis() - lastChange) > 100 && freq > 0){
    cli();
    freq = 0;
    inited = 0;
    i = 0;
    timeUp = 0;
    timeDown = 0;
    rised = 0;
    initArrays();
    sei();
  }

  if((millis() - freqLastSent) > 3000){
    Serial.println(lastFreq);
    Serial.println(lastTimeUp * 10000 / (lastTimeUp + lastTimeDown));
    Serial.println(lastTimeUp);
    Serial.println(lastTimeDown);
    Serial.println(inited);
    Serial.println();
    freqLastSent = millis();
  }
}
