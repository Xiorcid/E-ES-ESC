// HALLS
#define H1  5
#define H2  6
#define H3  7

// PHASES
#define PH1H 9
#define PH1L 2
#define PH2H 10
#define PH2L 3
#define PH3H 11
#define PH3L 4

// PHASE STATES
#define PH_H  1
#define PH_L  0
#define PH_Z  -1

#define POT A0

uint8_t DUTY = 0; // PWM DUTY

uint32_t bt_tmr; // BLUETOOH TIMER
uint32_t pulses; // PULSE COUNTER

bool hardFault = false; 

bool halls[3]; // HALL STATES
bool old_halls[3]; // OLD HALL STATES

void setup() {
  // PIN CONFIGURATION
  pinMode(H1, INPUT_PULLUP);
  pinMode(H2, INPUT_PULLUP);
  pinMode(H3, INPUT_PULLUP);

  pinMode(PH1H, OUTPUT);
  pinMode(PH1L, OUTPUT);
  pinMode(PH2H, OUTPUT);
  pinMode(PH2L, OUTPUT);
  pinMode(PH3H, OUTPUT);
  pinMode(PH3L, OUTPUT);

  pinMode(A0, INPUT);

  // PWM FREQUENCY
  TCCR1A = 0b00000001;  
  TCCR1B = 0b00000001;  
  TCCR2B = 0b00000001;  
  TCCR2A = 0b00000001;  

  // DISABLE ALL PHASES
  setPhases(PH_Z, PH_Z, PH_Z);

  // SAY HELLO
  Serial.begin(9600);
  Serial.println("Hello, world!");

  // RESET TIMERS
  bt_tmr = millis();
}

void loop() {
  // RESET STATES
  hardFault = false;

  // COPY HALL VALUES
  old_halls[0] = halls[0];
  old_halls[1] = halls[1];
  old_halls[2] = halls[2];

  // UPDATE HALLS
  updHalls();

  // UPDATE POT
  updPot();

  // COUNT PULSES
  if(old_halls[0] != halls[0] || old_halls[1] != halls[1] || old_halls[2] != halls[2]){
    pulses++;
  }
  
  // SET CORRECT CONFIG
  if(!halls[0] && !halls[1] && halls[2]){
    setPhases(PH_Z, PH_L, PH_H);
  }else if(halls[0] && !halls[1] && halls[2]){
    setPhases(PH_H, PH_L, PH_Z);
  }else if(halls[0] && !halls[1] && !halls[2]){
    setPhases(PH_H, PH_Z, PH_L);
  }else if(halls[0] && halls[1] && !halls[2]){
    setPhases(PH_Z, PH_H, PH_L);
  }else if(!halls[0] && halls[1] && !halls[2]){
    setPhases(PH_L, PH_H, PH_Z);
  }else if(!halls[0] && halls[1] && halls[2]){
    setPhases(PH_L, PH_Z, PH_H);
  }else{ // UNEXPECTED VALUE
    setPhases(PH_Z, PH_Z, PH_Z); // DISABLE PHASES
    hardFault = true;
  }

  // SEND DATA
  if(millis() - bt_tmr > 1000){
    if(hardFault){
      Serial.println("HARD FAULT!");
    }else{
      Serial.println(pulses);
      pulses = 0;
    }
    bt_tmr = millis();
  }
}

void setPhases(int8_t PH1, int8_t PH2, int8_t PH3){
  // PHASE 1
  if (PH1 != -1){
    // SET VALUE
    //digitalWrite(PH1H, PH1);
    analogWrite(PH1H, DUTY*PH1);
    digitalWrite(PH1L, PH1);
  }else{
    // DISABLE
    //digitalWrite(PH1H, HIGH);
    analogWrite(PH1H, 255);
    digitalWrite(PH1L, LOW);
  }

  if (PH2 != -1){
    //digitalWrite(PH1H, PH2);
    analogWrite(PH2H, DUTY*PH2);
    digitalWrite(PH2L, PH2);
  }else{
    //digitalWrite(PH2H, HIGH);
    analogWrite(PH2H, 255);
    digitalWrite(PH2L, LOW);
  }

  if (PH3 != -1){
    //digitalWrite(PH3H, PH3);
    analogWrite(PH3H, DUTY*PH3);
    digitalWrite(PH3L, PH3);
  }else{
    //digitalWrite(PH3H, HIGH);
    analogWrite(PH3H, 255);
    digitalWrite(PH3L, LOW);
  } 
}

void updHalls(){
  // SAVE HALLS VALUES
  halls[0] = digitalRead(H1);
  halls[1] = digitalRead(H2);
  halls[2] = digitalRead(H3);
}

void updPot(){
  uint16_t val = analogRead(POT);
  DUTY = map(val, 0, 1023, 0, 255);
}


