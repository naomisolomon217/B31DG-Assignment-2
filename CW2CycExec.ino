#include <stdio.h>
#include <stdint.h>
#include "B31DGMonitor.h"
#include "Ticker.h"

B31DGCyclicExecutiveMonitor monitor(1925);
Ticker ticker;

#define redOutput_T1 21  // red LED connected to GPIO35
#define orangeOutput_T2 19  // orange LED connected to GPIO32

#define input_T3 2   // Frequency input (F1)
#define input_T4 15   // Frequency input (F2)

#define yellowLED_T6 18  // yellow LED connected to GPIO33
#define greenLED_T7 25  // green LED connected to GPIO25

#define button_T7 26 // button connected to GPIO26



#define FRAME_DURATION_MS 2 // 2ms

uint8_t frameCounter;
const uint8_t debounceDelay = 50;

bool ledT7_state;

uint32_t freq_T3;
uint32_t freq_T4;
uint32_t freqTotal_T6;
uint32_t pulseHigh;
const uint32_t freqTimeout_T3 = 1500;
const uint32_t freqTimeout_T4 = 1200;

void frame();
void signal_T1();
void signal_T2();
void poll_T3();
void poll_T4();
void call_T5();
void sum_T6();

void IRAM_ATTR buttonT7_ISR() {
  // delay for debouncing
  delayMicroseconds(debounceDelay);
  if (digitalRead(button_T7)){
    ledT7_state = !ledT7_state;
    digitalWrite(greenLED_T7, ledT7_state);
    monitor.doWork();
  }
}

void setup() {

  Serial.begin(115200);

  // define button as input pins with pullup enabled, LEDs as outputs
  pinMode(button_T7, INPUT_PULLUP);
  pinMode(redOutput_T1, OUTPUT);
  pinMode(orangeOutput_T2, OUTPUT);
  pinMode(yellowLED_T6, OUTPUT);
  pinMode(greenLED_T7, OUTPUT);
  pinMode(input_T3, INPUT);
  pinMode(input_T4, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(button_T7), buttonT7_ISR, RISING);
  ticker.attach_ms(FRAME_DURATION_MS, frame);
  monitor.startMonitoring();
}

void signal_T1() 
{
  /* Task 1: Output waveform visualization
            measured to take 606us */
  monitor.jobStarted(1);
  digitalWrite(redOutput_T1, HIGH);
  delayMicroseconds(250);
  digitalWrite(redOutput_T1, LOW);
  delayMicroseconds(50);
  digitalWrite(redOutput_T1, HIGH);
  delayMicroseconds(300);
  digitalWrite(redOutput_T1, LOW);
  monitor.jobEnded(1);
} 


void signal_T2() 
{
  /* Task 2: Output waveform visualization 
            measured to take 356us*/
  monitor.jobStarted(2);
  digitalWrite(orangeOutput_T2, HIGH);
  delayMicroseconds(100);
  digitalWrite(orangeOutput_T2, LOW);
  delayMicroseconds(50);
  digitalWrite(orangeOutput_T2, HIGH);
  delayMicroseconds(200);
  digitalWrite(orangeOutput_T2, LOW);
  monitor.jobEnded(2); 
} 

// Task 3, takes 1ms
void poll_T3() 
{
  /* Task 3: Poll frequency of input square wave 
            measured to take 1501us at 666Hz (worst case execution), 
            1000us at 1000Hz (best case execution)*/

  monitor.jobStarted(3);
  long tempT3 = (pulseIn(input_T3, !digitalRead(input_T3), freqTimeout_T3));
  if(tempT3 < 760 && tempT3 > 490) freq_T3 = 1000000 / (tempT3 * 2);
  monitor.jobEnded(3);
  
} 


// Task 4, takes 2ms
void poll_T4() 
{
  /* Task 4: Poll frequency of input square wave 
            measured to take 1200us at 833Hz (worst case execution), 
            666us at 1500Hz (best case execution)
*/

  monitor.jobStarted(4);
  long tempT4 = (pulseIn(input_T4, !digitalRead(input_T4), freqTimeout_T4)); 
  if(tempT4 < 610 && tempT4 > 323) freq_T4 = 1000000 / (tempT4 * 2);
  monitor.jobEnded(4);
} 

void call_T5(){
  // Task 5: Call monitor.doWork()
  monitor.jobStarted(5);
  monitor.doWork();
  monitor.jobEnded(5);
}

void sum_T6(){
  // Task 6: LED control based on frequency sum
  freqTotal_T6 = freq_T3 + freq_T4;
  freqTotal_T6 = constrain(freqTotal_T6, 1499, 2500);
  if (freqTotal_T6 > 1500) digitalWrite(yellowLED_T6, HIGH);
  else digitalWrite(yellowLED_T6, LOW);
}

void frame() {
  switch(frameCounter){
    case(0):  signal_T1(); signal_T2(); call_T5();  break;
    case(1):  poll_T3();                            break;
    case(2):  signal_T1(); signal_T2();             break;
    case(3):  signal_T2(); poll_T4(); call_T5();    break;
    case(4):  signal_T1();                          break;
    case(5):  signal_T2(); poll_T3();               break;
    case(6):  signal_T1(); call_T5();               break;
    case(7):  signal_T2(); poll_T4();               break;
    case(8):  signal_T1(); signal_T2(); call_T5();  break;
    case(9):  signal_T2();                          break;
    case(10): signal_T1(); call_T5();               break;
    case(11): signal_T2(); poll_T3();               break;
    case(12): signal_T1(); signal_T2();             break;
    case(13): poll_T4(); call_T5();                 break;
    case(14): signal_T1(); signal_T2();             break;
    case(15): signal_T2(); poll_T3();               break;
    case(16): signal_T1(); call_T5();               break;
    case(17): signal_T2(); poll_T4();               break;
    case(18): signal_T1(); call_T5();               break;
    case(19): signal_T2(); sum_T6();                break;
    case(20): signal_T1(); signal_T2(); call_T5();  break;
    case(21): signal_T2(); poll_T3();               break;
    case(22): signal_T1(); poll_T4();               break;
    case(23): signal_T2(); call_T5();               break;
    case(24): signal_T1(); signal_T2();             break;
    case(25): poll_T3(); call_T5();                 break;
    case(26): signal_T1(); signal_T2();             break;
    case(27): signal_T2(); poll_T4();               break;
    case(28): signal_T1(); call_T5();               break;
    case(29): signal_T2();                          break;
    default:  Serial.print("Error: invalid frame reference");
  }

  frameCounter++;
  if(frameCounter >= 30) frameCounter = 0;
}

 
void loop(void) 
{
  /*
  unsigned long bT = micros();
  for (int i=0; i<1000; i++) 
  {
    signal_T1();
  }
  unsigned long timeItTook = micros()-bT / 1000;
  Serial.println("Duration SerialOutput Job = ");
  Serial.println(timeItTook);
  exit(0);
  frame();
*/
}


