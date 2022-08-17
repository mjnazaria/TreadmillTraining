#include <Servo.h>

#define REWARD_DURATION 100   // msec
#define DISTANCE_COVERED 1400 // number of pulses ~50cm
#define START_DISTANCE_COVERED 500
#define INCREMENT_DISTANCE 300
#define TOTAL_TRIALS_NB 10000  // msec
#define INTERTRIAL_DURATION 4000 // msec
#define SPEED_THRESHOLD 7    // cm/sec

Servo myservo;
int servo_pos_NB = 0;
int servo_pos_B = 19;  // Break angle
int servo_pos_R = 70;  // Release angle
int pin_chA = 2; // Interrupt pin for encoder channel A
int pin_chB = 3; // Interrupt pin for encoder channel B
int pin_mode = 6;

int pin_rwd = 8; // Digital output pin for delivering reward
int pin_ave = 21; // Digital output pin for aversive stimulus
int pin_light = 20; // Digital output pin for light stimulus
int pin_photo_sensor = 16;  // Photosensor detects one full lap

long encoder_count = 0;
long last_encoder_count = 0;

int phase = 50;
unsigned int ts = 0;
unsigned int dur = 0;
unsigned int ts1 = 0;
unsigned int dur1 = 0;
unsigned int trials = 0;
unsigned int trialDistance = START_DISTANCE_COVERED;
unsigned int tsD = 0;
unsigned int durD = 0;
float mouse_speed = 0;

char serialRead = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(pin_rwd, OUTPUT);
  pinMode(pin_ave, OUTPUT);
  pinMode(pin_light, OUTPUT);

  pinMode(pin_chA, INPUT);
  pinMode(pin_chB, INPUT);
  pinMode(pin_mode, INPUT_PULLUP);
  pinMode(pin_photo_sensor,INPUT);

  digitalWrite(pin_rwd, LOW);
  digitalWrite(pin_ave, LOW);
  digitalWrite(pin_light, LOW);
  attachInterrupt(digitalPinToInterrupt(pin_chA), encoder_funcAR, RISING); //
  attachInterrupt(digitalPinToInterrupt(pin_chB), encoder_funcBR, RISING); //
  attachInterrupt(digitalPinToInterrupt(pin_chA), encoder_funcAF, FALLING); //
  attachInterrupt(digitalPinToInterrupt(pin_chB), encoder_funcBF, FALLING); //
  Serial.begin(115200);
  ts = millis();
}

void loop() {
  // start of the trial
  if (digitalRead(pin_mode) == 0) {        
    // No brake loop
    if (phase == 60) {
      trials++;
      if (trials <= TOTAL_TRIALS_NB)
        phase = 100;
      else {
        trials = 0;
        phase = 1000;
        ts = millis();
      }
    }
    if (phase == 100) {
      myservo.write(servo_pos_NB);
      delay(100);
      phase = 200;
    }
    if (phase == 200) {
      Serial.println(trialDistance);
      digitalWrite(pin_ave, HIGH);
      encoder_count = 0;
      tsD = millis();
      phase = 320;
    }
    if (phase == 320) {
        if (encoder_count > trialDistance){
        digitalWrite(pin_ave, LOW);
        phase = 600;
        ts = millis();
        durD = millis()-tsD;
        mouse_speed = (trialDistance * 2*3.141592654/1000 * 5)*1000/durD; // speed in cm/sec
        Serial.println(mouse_speed);
        myservo.write(servo_pos_B);
        }
    }
  if (phase == 400) {
      dur = millis() - ts;
      if (dur > 10) {
        digitalWrite(pin_rwd, HIGH);
        phase = 500;
        ts1 = millis();
      }
    }
    if (phase == 500) {
      dur = millis() - ts1;
      if (dur > REWARD_DURATION) {
        digitalWrite(pin_rwd, LOW);
        phase = 600;
      }
    }
    if (phase == 600) {
      dur = millis() - ts;
      if (dur > INTERTRIAL_DURATION) {
        phase = 60;
        if (mouse_speed>=SPEED_THRESHOLD)
          trialDistance = trialDistance + INCREMENT_DISTANCE;
        if (trialDistance>DISTANCE_COVERED)
          trialDistance = DISTANCE_COVERED;
        if (trialDistance<START_DISTANCE_COVERED)
          trialDistance = START_DISTANCE_COVERED;
      }
    }
  }
  if (digitalRead(pin_mode) == 1) {
    trials = 0;
    trialDistance = START_DISTANCE_COVERED;
    digitalWrite(pin_rwd, LOW);
    digitalWrite(pin_ave, LOW);
    digitalWrite(pin_light, LOW);
    phase = 60;
    
  
    delay(15);
    ts = millis();

    if (digitalRead(pin_photo_sensor) == 0) {
      if (encoder_count > 3000){
      digitalWrite(pin_rwd, LOW);
      delay(REWARD_DURATION);
      digitalWrite(pin_rwd, LOW);
      delay(500);
      encoder_count = 0;
      }
    }
  }
}

void encoder_funcAR() {
  if (digitalRead(pin_chB) == 1)
    encoder_count++;
  else
    encoder_count--;
}

void encoder_funcAF() {
  if (digitalRead(pin_chB) == 0)
    encoder_count++;
  else
    encoder_count--;
}

void encoder_funcBR() {
  if (digitalRead(pin_chA) == 0)
    encoder_count++;
  else
    encoder_count--;
}

void encoder_funcBF() {
  if (digitalRead(pin_chA) == 1)
    encoder_count++;
  else
    encoder_count--;
}
