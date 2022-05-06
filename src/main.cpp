#include "Arduino.h"
#include "PID_v1.h"
#include "time.h"

#ifndef DEBUG
#define DEBUG
#endif

// define port numer of each pin to mannually control the registor
#define UH 0
#define UL 1
#define VH 2
#define VL 3
#define WH 4
#define WL 5

// define output pin
#define PWM_H_PIN 5 // OC0B
#define PWM_L_PIN 6 // OC0A

#define UH_PIN 8 // PB0
#define UL_PIN 9
#define VH_PIN 10
#define VL_PIN 11
#define WH_PIN 12
#define WL_PIN 13 // PB5

#define CAP_SW 7 //Controling the capacitor


// define input pin
// from switch
#define DRIVE_SW_PIN 2 // INT0
#define BRAKE_SW_PIN 3 // INT1
#define DRIVE_LED 4
// from hall sensor
#define HALL_U_PIN 14 // PC0
#define HALL_V_PIN 15 // PC1
#define HALL_W_PIN 16 // PC2

//from voltage sensor
#define BAT_VO_PIN A3
#define BAT_CURR_PIN A4

bool drive_flag;
bool brake_flag;
uint8_t hall_sen;
uint8_t switch_patt;
uint8_t vol_in;
uint8_t curr_in;
double bat_vol;
double bat_curr;
double target_curr;
double target_voltage;
double PIDinput;
double PIDoutput;
int hall_c=0;

unsigned long time_0=0;
unsigned long time_1=0;

PID myPID(&PIDinput, &PIDoutput, &target_voltage, 0.02, 0.01, 0.01, DIRECT);

void drive_sw_interrupt();
void brake_sw_interrupt();
void switch_pattn();
void switch_pattn2();
void switch_pattn3();


void setup() {
  // setup pin mode
  pinMode(PWM_H_PIN, OUTPUT);
  pinMode(PWM_L_PIN, OUTPUT);
  pinMode(CAP_SW, OUTPUT);
  /*
  pinMode(UH_PIN, OUTPUT);
  pinMode(UL_PIN, OUTPUT);
  pinMode(VH_PIN, OUTPUT);
  pinMode(VL_PIN, OUTPUT);
  pinMode(WH_PIN, OUTPUT);
  pinMode(WL_PIN, OUTPUT);
  */
  // configure PB0-PB5
  DDRB = 0b00111111;

  // from control swtiches
  pinMode(DRIVE_SW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DRIVE_SW_PIN), drive_sw_interrupt, CHANGE);
  // pinMode(BRAKE_SW_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(BRAKE_SW_PIN), brake_sw_interrupt, CHANGE);
  pinMode(DRIVE_LED, OUTPUT); //
  // from sensor
  pinMode(BAT_VO_PIN, INPUT);
  pinMode(BAT_CURR_PIN, INPUT);
  // Enable pin change interrupt
  PCICR |= 1 << PCIE1;
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
  PORTC |= (1<<PC0) | (1<<PC1) | (1<<PC2); // pinMode()
  // from voltage
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  // Set up PID
  PIDinput = 0;
  PIDoutput = 0;
  target_voltage = 42;
  target_curr = -1;
  myPID.SetMode(AUTOMATIC);

  switch_pattn();
  
}

void loop() {
  //drive_flag = true;
  brake_flag = false;
  vol_in = analogRead(BAT_VO_PIN);
  // hall_sen = PINC & 0b000111;
  switch_pattn();
  if(drive_flag)
  {
    analogWrite(PWM_H_PIN, 255);
    analogWrite(PWM_L_PIN, 255);
  }
}

void drive_sw_interrupt()
{
  time_1 = millis();
  if (time_1 - time_0>=10)
  {
    drive_flag = digitalRead(DRIVE_SW_PIN);
    #ifdef DEBUG
    Serial.print("Detected drive switch:");
    Serial.println(drive_flag);
    Serial.print("\t Time:");
    Serial.println(time_0);
    Serial.print("\n");
    #endif
    digitalWrite(DRIVE_LED, drive_flag);
  }
  
}

void brake_sw_interrupt()
{
  time_1 = millis();
  if (time_1-time_0>=10)
  {
    time_0=time_1;
    brake_flag = digitalRead(BRAKE_SW_PIN);
    #ifdef DEBUG
    Serial.print("Detected drive switch:");
    // Serial.println(drive_flag);
    Serial.print("\t Time:");
    Serial.println(time_0);
    Serial.print("\n");
    #endif
  }
}

ISR(PCINT1_vect)
{
  hall_c ++;
  hall_sen = PINC & 0b00000111;
  switch_pattn();
  #ifdef DEBUG
  Serial.write("Detect hall sensor");
  Serial.println(hall_c);
  Serial.print("\t");
  Serial.println(hall_sen,BIN);
  Serial.print("\n");
  Serial.print("Switch pattern:");
  Serial.println(switch_patt, BIN);
  Serial.print("\n");
  #endif   
}

void switch_pattn()
{
    //hall_sen = ~(PINC & 0b00000111) & 0b00000111; // W||V||U
    switch (hall_sen)
    {
      case 0b001: //0b001
        switch_patt = (1<<WH) | (1<<VL);
        break;
      case 0b011: //0b011
        switch_patt = (1<<UH) | (1<<VL);
        break;
      case 0b010: //0b010
        switch_patt = (1<<UH) | (1<<WL);
        break;
      case 0b110: //0b110
        switch_patt = (1<<VH) | (1<<WL);
        break;
      case 0b100: //0b100
        switch_patt = (1<<VH) | (1<<UL);
        break;
      case 0b101: //0b101
        switch_patt = (1<<WH) | (1<<UL);
        break;
      default:
        switch_patt = 0;
        break;
    }
    PORTB = switch_patt;
}

void switch_pattn2()
{
    hall_sen = ~(PINC & 0b00000111) & 0b00000111; // W||V||U
    switch (hall_sen)
    {
      case 0b001: //0b001
        switch_patt = (1<<WH) | (1<<UL);
        break;
      case 0b011: //0b011
        switch_patt = (1<<WH) | (1<<VL);
        break;
      case 0b010: //0b010
        switch_patt = (1<<UH) | (1<<VL);
        break;
      case 0b110: //0b110
        switch_patt = (1<<UH) | (1<<WL);
        break;
      case 0b100: //0b100
        switch_patt = (1<<VH) | (1<<WL);
        break;
      case 0b101: //0b101
        switch_patt = (1<<VH) | (1<<UL);
        break;
      default:
        switch_patt = 0;
        break;
    }
    PORTB = switch_patt;
}

void switch_pattn3()
{
    hall_sen = ~(PINC & 0b00000111) & 0b00000111; // W||V||U
    switch (hall_sen)
    {
      case 0b001: //0b001
        switch_patt = (1<<UH) | (1<<WL);
        break;
      case 0b011: //0b011
        switch_patt = (1<<VH) | (1<<WL);
        break;
      case 0b010: //0b010
        switch_patt = (1<<VH) | (1<<UL);
        break;
      case 0b110: //0b110
        switch_patt = (1<<WH) | (1<<UL);
        break;
      case 0b100: //0b100
        switch_patt = (1<<WH) | (1<<VL);
        break;
      case 0b101: //0b101
        switch_patt = (1<<UH) | (1<<VL);
        break;
      default:
        switch_patt = 0;
        break;
    }
    
    PORTB = switch_patt;
}
