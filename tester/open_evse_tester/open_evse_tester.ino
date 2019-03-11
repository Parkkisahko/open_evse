#define pinB 13
#define pinC 10
#define pinD 7

#define pinPilot A0
#define pinRelay1 A1
#define pinButton 8

#define stateA 0
#define stateB 1
#define stateC 2
#define stateD 3

#define samplesAverage 20
#define msgLen 6

#define debug 0

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
volatile unsigned long totalTime = 0;

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
  unsigned long eventTime = micros();
  lastChange = millis();                      // Not interrupt safe, non critical though
  uint8_t pinState = digitalRead(pinPilot);   // Could be done faster if required
  if(pinState == HIGH){
    // Rising edge
    freq = 1000000 * samplesAverage / (eventTime - risingEdges[i]);
    totalTime = eventTime - risingEdges[i];//(i + 1) % samplesAverage];
    //timeDown = eventTime - fallingEdges[(i - 1) % samplesAverage];
    if(!rised){
      rised = 1;
    }
    risingEdges[i] = eventTime;
  } else {
    if(rised){
      // Update time up and down here
      timeUp += (eventTime - risingEdges[i]);
      if(inited){
        timeUp -= (fallingEdges[(i + 1) % samplesAverage] - risingEdges[(i + 1) % samplesAverage]);
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
  pinMode(pinRelay1, INPUT);
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
#if debug
  unsigned long freqLastSent = 0;
#endif
unsigned long stateLastSent = 0;
unsigned long lastTimeUp = 0;
unsigned long lastTimeDown = 0;
unsigned long lastTotalTime = 0;

uint8_t incomingByte[msgLen];
uint8_t bufIdx = 0;
bool received = false;

uint8_t relayState = 0;

void process_incoming_data(){
  if(received){
    switch(incomingByte[0]){
      case 'S':
        switch(incomingByte[1]){
          case 'S':   // Set State (SS)
            switch(incomingByte[3]){
              case 'A':
                state = stateA;
                break;
              case 'B':
                state = stateB;
                break;
              case 'C':
                state = stateC;
                break;
              case 'D':
                state = stateD;
                break;
            }
            #if debug
              Serial.print("Setting state to: ");
              Serial.println(state);
            #endif
            process_state(state);
            break;
        }
        break;
    }
    received = false;
    bufIdx = 0;
  }
}

void loop() { 
  buttonState = digitalRead(pinButton);
  if(buttonState == HIGH && buttonState != lastButtonState && (millis() - buttonPressTime) > 300){
    buttonPressTime = millis();
    state++;
    if(state > stateD){
      state = stateA;
    }
    Serial.println("Changing state to: ");
    Serial.println(state);
    process_state(state);
  }
  lastButtonState = buttonState;

  relayState = digitalRead(pinRelay1);
  
  cli();
  lastFreq = freq;
  lastTimeUp = timeUp;
  lastTimeDown = timeDown;
  lastTotalTime = totalTime;
  sei();
  if((millis() - stateLastSent) > 200){
    Serial.print("{\"relay\": ");
    Serial.print(relayState);
    Serial.print(", \"freq\": ");
    Serial.print(lastFreq);
    Serial.print(", \"PWM\": ");
    Serial.print(lastTimeUp * 10000 / (lastTotalTime));
    #if debug
      Serial.print(", \"lastTimeUp\": ");
      Serial.print(lastTimeUp);
      Serial.print(", \"lastTimeDown\": ");
      Serial.print(lastTimeDown);
      Serial.print(", \"totalTime\": ");
      Serial.print(lastTotalTime);
    #endif
    Serial.print(", \"state\": ");
    Serial.print(state);
    Serial.print(", \"time\": ");
    Serial.print(millis());
    Serial.println("}");
    stateLastSent = millis();
  }
  if((millis() - lastChange) > 100 && freq > 0){
    cli();
    freq = 0;
    inited = 0;
    i = 0;
    timeUp = 0;
    rised = 0;
    initArrays();
    sei();
  }

  #if debug
    if((millis() - freqLastSent) > 3000){
      Serial.print("Relay state: ");
      Serial.println(relayState);
      Serial.print("Freq: ");
      Serial.println(lastFreq);
      Serial.print("PWM duty cycle: ");
      Serial.println(lastTimeUp * 10000 / lastTotalTime);
      Serial.print("Time up: ");
      Serial.println(lastTimeUp);
      Serial.print("Total time: ");
      Serial.println(lastTotalTime);
      Serial.println();
      freqLastSent = millis();
    }
  #endif
  

  process_incoming_data();
    
  while (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte[bufIdx++] = Serial.read();
    if(incomingByte[bufIdx-1] == '\n'){
      received = true;
      #if debug
        Serial.println("Receive finished");
      #endif
    }
    #if debug
      Serial.print("Received byte: ");
      Serial.println(incomingByte[bufIdx - 1]);
      Serial.print("At idx: ");
      Serial.println(bufIdx);
    #endif
  }
}
