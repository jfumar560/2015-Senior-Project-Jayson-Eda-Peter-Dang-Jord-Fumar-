/*
Water Control System 2015
by: Peter Dang, Jayson Eda, Jord Fumar

Written by: Peter and Jayson
Calculation provided by: Jord

v2: state correction
v3: added state: goes to GROUP_1 when no flow for 1min
v4: lcd display added
v5: calculating water flow is done by timer0
*/

/////////////////////////////////////////////////////////////////
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include "Platform.h"
#include "SoftwareSerial.h"

#ifndef CDC_ENABLED
// Shield Jumper on SW
SoftwareSerial port(12, 13);

#else
// Shield Jumper on HW (for Leonardo)
#define port Serial1
#endif

#else // Arduino 0022 - use modified NewSoftSerial
#include "WProgram.h"
#include "NewSoftSerial.h"
NewSoftSerial port(12, 13);
#endif

#include "EasyVR.h"

#define SND_SMART_WATER_UPDATE_1  1  // system activated
#define SND_SMART_WATER_UPDATE_2  2  // system continuing
#define SND_SMART_WATER_UPDATE_3  3  // system paused
#define SND_SMART_WATER_UPDATE_4  4  // system ready to be continued
#define SND_SMART_WATER_UPDATE_5  5  // temperature decreased
#define SND_SMART_WATER_UPDATE_6  6  // temperature increased

EasyVR easyvr(port);

//Groups and Commands
enum Groups
{
  GROUP_0  = 0,
  GROUP_1  = 1,
  GROUP_2  = 2,
  GROUP_3  = 3,
};

enum Group0
{
  G0_SMART_SHOWER = 0,
};

enum Group1
{
  G1_BREAK = 0,
  G1_HOTTER = 1,
  G1_COLDER = 2,
};

enum Group2
{
  G2_ACTIVATE = 0,
};

enum Group3
{
  G3_CONTINUE = 0,
};

EasyVRBridge bridge;
volatile int8_t group, idx;

/////////////////////////////////////////////////////////////////
//Include Servo library
#include <Servo.h>
Servo servo; //create servo (connected to PIN45)
volatile int servo_val = 5;
volatile int servo_mapped; //variable for servo

//Include GLCD library
#include <glcd.h>
#include <fonts/allFonts.h>

//Define Input Control Pins
#define button1 5 //digital pin18 (interrupt 5)
#define button2 4 //digital pin19 (interrupt 4)
volatile boolean button = false;

int temp_knob = A8; //led_bar + temp control (servo) input
int knob; //global variable for temperature pot

//Define Output Control Pins
#define valve1 42 //valve1 control
#define valve2 41 //valve2 control
#define motor 40 //motor relay control

//Define Shift Register Pins
#define data 50 //data pin for shift reg
#define latch 49 //latch pin for shift reg
#define clock 48 //clock pin for shift reg
int shift_reg1, shift_reg2; //shift reg variables

//Define Sensor Pins
//for temperature sensor
//global variable for temp sensing
int sample = 10; //sampling
int temp1 = A9; //temperature sensor 1
int temp2 = A10; //temperature sensor 2
double temp1_voltage, temp2_voltage, temp_fahr1, temp_fahr2;

const double a = 0.0008800;
const double b = 0.0002530;
const double c = 0.0000002;

//for water flow sensor
#define flow_sense1 3 //digital Pin 20 (interrupt 3)
#define flow_sense2 2 //digital Pin 21 (interrupt 2)

volatile int flow_pulse1 = 0; //pulse counter for flow sensor 1
volatile int flow_pulse2 = 0; //pulse counter for flow sensor 2

double flow1 = 0; //flow rate 1
double flow2 = 0; //flow rate 2
unsigned long oldTime = 0; //time tracking for sensor

double tflow_rate, total_water;
int time_duration = 0;
/////////////////////////////////////////////////////////////////
volatile int state, last_state;
#define led 15 //led indicator for state
volatile int counter;

unsigned long oldTime2 = 0;
int time_duration2 = 0;
/////////////////////////////////////////////////////////////////
void setup()
{
  // put your setup code here, to run once:
  noInterrupts(); // disable all interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2  = 0; //initialize counter value to 0

  OCR2A = 156;// set compare match register (every .01sec)
  TCCR2A = 0b00000010;// turn on CTC mode
  TCCR2B = 0b00000111;// Set prescaler 1024
  TIMSK2 = 0b00000010;// enable timer compare interrupt
  interrupts(); // enable all interrupts

  //initialize servo (connect pin D45)
  servo.attach(45);

  //initialize GLCD
  GLCD.Init();
  GLCD.SelectFont(System5x7); // Select the font for the default text area

  GLCD.CursorTo(0, 3); // (column, row)
  GLCD.print("Temperature 1:");
  GLCD.CursorTo(0, 4); // (column, row)
  GLCD.print("Temperature 2:");
  GLCD.CursorTo(0, 6); // (column, row)
  GLCD.print("Flow Rate:"); //flow rate is not necessary...
  GLCD.CursorTo(0, 7); // (column, row)
  GLCD.print("Total Water:");

  //initialize output pins
  pinMode(valve1, OUTPUT);
  pinMode(valve2, OUTPUT);
  pinMode(motor, OUTPUT);
  digitalWrite(valve1, LOW);
  digitalWrite(valve2, LOW);
  digitalWrite(motor, LOW);

  //initialize shift reg
  pinMode(data, OUTPUT);
  pinMode(latch, OUTPUT);
  pinMode(clock, OUTPUT);

  /////////////////////////////////////////////////////////////////
  pinMode(led, OUTPUT); //led indicator for HOT water
  /////////////////////////////////////////////////////////////////

  //button interrupt ISR
  attachInterrupt(button1, button1_int, RISING); //if button 1 is press, do button_int action
  attachInterrupt(button2, button2_int, RISING); //if button 2 is press, do button2_int action

  //flow sensors interrupt
  attachInterrupt(flow_sense1, pulseCounter, RISING); //for counting pulse (water flow sensor 1)
  attachInterrupt(flow_sense2, pulseCounter2, RISING); //for counting pulse (water flow sensor 2)

  //taking initial temperature reading from temperature 1
  temp_sensors();
  if (temp_fahr1 <= 90)
  {
    last_state = 0; //taking last_state;
    state0(); //indicate state 0
  }
  else
  {
    last_state = 1;
    state1(); //indicate state 1
  }
  digitalWrite(motor, LOW); //turn relay off
  digitalWrite(valve1, LOW); //turn valve1 off
  digitalWrite(valve2, LOW); //turn valve2 off

#ifndef CDC_ENABLED

  // bridge mode?
  if (bridge.check())
  {
    cli();
    bridge.loop(0, 1, 12, 13);
  }
  // run normally
  Serial.begin(9600);

#else
  // bridge mode?
  if (bridge.check())
  {
    port.begin(9600);
    bridge.loop(port);
  }
#endif
  port.begin(9600);

  easyvr.setPinOutput(EasyVR::IO1, LOW);
  easyvr.setTimeout(0);
  easyvr.setLanguage(0);
  easyvr.setLevel(1);


  easyvr.playSound(0, EasyVR::VOL_FULL);
  group = EasyVR::TRIGGER; //<-- start group (customize)
}

ISR(TIMER2_COMPA_vect) // timer compare interrupt service routine
{
  counter++;
  if (counter == 100)
  {
    //taking the pulse from flow sensors
    time_duration++;
    time_duration2++;

    flow1 = (flow_pulse1 * 2.25) / 1000;
    flow2 = (flow_pulse2 * 2.25) / 1000;

    flow_pulse1 = 0;
    flow_pulse2 = 0;
    counter = 0;
  }
  tflow_rate = flow1 + flow2;
  total_water = (tflow_rate * time_duration) + total_water;
}

void loop()
{
  easyvr.setPinOutput(EasyVR::IO1, HIGH); // LED on (listening)
  Serial.println(group);
  easyvr.recognizeCommand(group);
  do
  {
    temp_sensors(); //read all temperature sensors
    knob = analogRead(temp_knob);
    //Serial.println(knob); //testing purpose
    temp_control(knob); //read temperature control
    //Serial.println(knob);
    if (temp_fahr1 <= 90)
    {
      state0(); //go to state 0...water in hot water line is not HOT
      if (last_state == 1)
      {
        group = GROUP_0;
        last_state = 0;
      }
    }
    else
    {
      state1(); //got to state 1...water is HOT and ready
      digitalWrite(motor, LOW); //turn off relay, turning the motor off
      if (last_state == 0)
      {
        last_state = 1;
        digitalWrite(valve1, LOW);
        digitalWrite(valve2, LOW);
      }
    }

    if (tflow_rate == 0)
    {
      time_duration = 0;
      if (time_duration2 == 300)
      {
        group = EasyVR::TRIGGER;
        time_duration2 = 0;
        total_water = 0;
      }
    }
    else
    {
      time_duration2 = 0;
    }

    //display sensors
    GLCD.CursorTo(15, 3); // (column, row)
    GLCD.EraseTextLine();
    GLCD.CursorTo(15, 3); // (column, row)
    GLCD.print(temp_fahr1, 2);

    GLCD.CursorTo(15, 4); // (column, row)
    GLCD.EraseTextLine();
    GLCD.CursorTo(15, 4); // (column, row)
    GLCD.print(temp_fahr2, 2);

    GLCD.CursorTo(13, 6); // (column, row)
    GLCD.EraseTextLine();
    GLCD.CursorTo(13, 6); // (column, row)
    GLCD.print(tflow_rate, 2);

    GLCD.CursorTo(13, 7); // (column, row)
    GLCD.EraseTextLine();
    GLCD.CursorTo(13, 7); // (column, row)
    GLCD.print(total_water, 2);
  } while (!easyvr.hasFinished());
  easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off

  idx = easyvr.getWord();
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    // print debug message
    uint8_t train = 0;
    char name[32];
    action();
  }
}

void action()
{
  switch (group)
  {
    case GROUP_0:
      switch (idx)
      {
        case G0_SMART_SHOWER:
          easyvr.playSound(4, EasyVR::VOL_FULL);
          group = GROUP_2;
          break;
      }
      break;
    case GROUP_1:
      switch (idx)
      {
        case G1_BREAK:
          button = HIGH;
          digitalWrite(valve1, button);
          digitalWrite(valve2, button);
          easyvr.playSound(3, EasyVR::VOL_FULL);
          group = GROUP_3; //<-- or jump to another group X for composite commands
          break;
        case G1_HOTTER:
          servo_val++;
          if (servo_val >= 10)
          {
            servo_val = 10;
          }
          servo_mapped = map(servo_val, 1, 10, 1, 180);
          servo.write(servo_mapped);
          easyvr.playSound(6, EasyVR::VOL_FULL);
          // group = GROUP_X; <-- or jump to another group X for composite commands
          break;
        case G1_COLDER:
          servo_val--;
          if (servo_val <= 0)
          {
            servo_val = 1;
          }
          servo_mapped = map(servo_val, 1, 10, 1, 180);
          servo.write(servo_mapped);
          easyvr.playSound(5, EasyVR::VOL_FULL);
          // group = GROUP_X; <-- or jump to another group X for composite commands
          break;
      }
      break;
    case GROUP_2:
      switch (idx)
      {
        case G2_ACTIVATE:
          if (state == 0)
          {
            state0();
            digitalWrite(motor, HIGH);
            digitalWrite(valve1, HIGH);
            digitalWrite(valve2, HIGH);
            easyvr.playSound(1, EasyVR::VOL_FULL);
            group = GROUP_1; //<-- or jump to another group X for composite commands
          }
          else
          {
            state1();
            easyvr.playSound(0, EasyVR::VOL_FULL);
            group = GROUP_1;
          }
          break;
      }
      break;
    case GROUP_3:
      switch (idx)
      {
        case G3_CONTINUE:
          button = LOW;
          digitalWrite(valve1, button);
          digitalWrite(valve2, button);
          easyvr.playSound(2, EasyVR::VOL_FULL);
          group = GROUP_1; //<-- or jump to another group X for composite commands
          break;
      }
      break;
  }
}

/////////////////////////////////////////////////////////////////
//Interrupt Service Routine for switch buttons
void button1_int() //button 1 ISR, this happens only if button 1 is press
{
  if (state == 0) //when button 1 is press when at state 0, turn motor on and close valves
  {
    group = GROUP_1;
    digitalWrite(motor, HIGH); //turn on water pump
    digitalWrite(valve1, HIGH);
    digitalWrite(valve2, HIGH);
  }
  else //state 1
  {
    //toggle both valves
    button = !button;
    digitalWrite(valve1, button); //toogle on and off valve 2 (COLD water)
    digitalWrite(valve2, button); //toggle on and off valve 1 (HOT water)

    if (button == HIGH)
    {
      idx = G1_BREAK;
      group = GROUP_3;
    }
    else
    {
      idx = G3_CONTINUE;
      group = GROUP_1;
    }
  }
}
void button2_int() //button 2 ISR
{
  if (state == 0) //turn off pump and open valves
  {
    digitalWrite(motor, LOW); //turn off water pump
    digitalWrite(valve1, LOW); //open valve 1
    digitalWrite(valve2, LOW); //open valve 2
  }
  else //state 1
  {
    //servo turns only when button is press...
    //this happens when button 2 is press while at state 1
    servo_mapped = map(knob, 1023, 1, 1, 180); //read temperature pot and map value for servo
    servo.write(servo_mapped); //turn servo
  }
}

/////////////////////////////////////////////////////////////////
//Insterrupt Service Routine for Water Flow
void pulseCounter()
{
  // Increment the pulse counter
  flow_pulse1++;
}

void pulseCounter2()
{
  // Increment the pulse counter
  flow_pulse2++;
}

/////////////////////////////////////////////////////////////////
//function for reading temperature sensors
void temp_sensors()
{
  double temp_val1, Rtherm1, temp_val2, Rtherm2;

  double temp1_array[10]; //array of temperature sensor 1 reading
  double temp2_array[10]; //array of temperature sensor 2 reading

  //initial temperature values
  double temp1_readings = 0; //temperature reading from temperature sensor 1
  double temp2_readings = 0; //temperature reading from temperature sensor 2
  double temp1_ave = 0; //final value reading for temperature 1
  double temp2_ave = 0; //final value reading for temperature 2

  for (int index = 0; index < sample; index++) //taking readings of input voltages every 1sec
  {
    temp1_array[index] = analogRead(temp1); //reading analog pin A8
    temp2_array[index] = analogRead(temp2); //reading analog pin A10

    temp1_readings = temp1_readings + temp1_array[index]; //adding samples for temperature sensor
    temp2_readings = temp2_readings + temp2_array[index]; //adding samples for temperature sensor 2
  }
  temp1_ave = temp1_readings / sample; //taking average temperature 1
  temp2_ave = temp2_readings / sample; //taking average temperature 2

  temp1_voltage = temp1_ave * (5.0 / 1023.0);
  temp2_voltage = temp2_ave * (5.0 / 1023.0);

  Rtherm1 = (50000 - temp1_voltage * 10000) / temp1_voltage;
  Rtherm2 = (50000 - temp2_voltage * 10000) / temp2_voltage;
  Rtherm1 = log(Rtherm1);
  Rtherm2 = log(Rtherm2);
  temp_val1 = 1 / (a + b * Rtherm1 + c * (Rtherm1 * Rtherm1 * Rtherm1));
  temp_val2 = 1 / (a + b * Rtherm2 + c * (Rtherm2 * Rtherm2 * Rtherm2));
  temp_fahr1 = (temp_val1 - 273.15) * 1.8 + 32;
  temp_fahr2 = (temp_val2 - 273.15) * 1.8 + 32;
}
/////////////////////////////////////////////////////////////////
//led bar graph + shift register
void led_temp(int index)
{
  switch (index)
  {
    case 0:
      shift_reg1 = 0b00000000; shift_reg2 = 0b00000000;
      break;
    case 1:
      shift_reg1 = 0b00000001; shift_reg2 = 0b00000000;
      break;
    case 2:
      shift_reg1 = 0b00000011; shift_reg2 = 0b00000000;
      break;
    case 3:
      shift_reg1 = 0b00000111; shift_reg2 = 0b00000000;
      break;
    case 4:
      shift_reg1 = 0b00001111; shift_reg2 = 0b00000000;
      break;
    case 5:
      shift_reg1 = 0b00011111; shift_reg2 = 0b00000000;
      break;
    case 6:
      shift_reg1 = 0b00111111; shift_reg2 = 0b00000000;
      break;
    case 7:
      shift_reg1 = 0b01111111; shift_reg2 = 0b00000000;
      break;
    case 8:
      shift_reg1 = 0b11111111; shift_reg2 = 0b00000000;
      break;
    case 9:
      shift_reg1 = 0b11111111; shift_reg2 = 0b00000001;
      break;
    case 10:
      shift_reg1 = 0b11111111; shift_reg2 = 0b00000011;
      break;
  }
  digitalWrite(latch, LOW);
  //shiftOut(dataPin, clockPin, bitOrder, value)
  shiftOut(data, clock, MSBFIRST, shift_reg2);
  shiftOut(data, clock, MSBFIRST, shift_reg1);
  digitalWrite(latch, HIGH);
}

/////////////////////////////////////////////////////////////////
//servo control + bar graph
void temp_control(int temp)
{
  int temp_val = map(temp, 1023, 1, 1, 10);
  led_temp(temp_val);
}

/////////////////////////////////////////////////////////////////
//State + led indicator
void state0()
{
  state = 0; //go to state 0
  digitalWrite(led, HIGH); //turn led indicator on for NOT READY
  GLCD.CursorTo(0, 0); // (column, row)
  GLCD.print("Water is not ready! ");
}
void state1()
{
  state = 1; //go to state 1
  digitalWrite(led, LOW); //turn led indicator on for READY
  GLCD.CursorTo(0, 0); // (column, row)
  GLCD.print("Water is ready!     ");
}
