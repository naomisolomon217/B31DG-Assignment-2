#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "B31DGMonitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define INCLUDE__vTaskDelayUntil 1

B31DGCyclicExecutiveMonitor monitor(2100);

// define pins
#define redOutput_T1 21  // red LED connected to GPIO21
#define orangeOutput_T2 19  // orange LED connected to GPIO19

#define input_T3 2   // Frequency input (Task 3)
#define input_T4 15   // Frequency input (Task 4)

#define yellowLED_T6 18  // yellow LED connected to GPIO18
#define greenLED_T7 25  // green LED connected to GPIO25

#define button_T7 26 // button connected to GPIO26

const uint8_t debounceDelay = 50;
uint32_t prevDebounceTime;
const uint32_t freqTimeout_T3 = 1500;
const uint32_t freqTimeout_T4 = 1200;
bool ledT6_state;
bool ledT7_state;

uint32_t freq_T3;
uint32_t freq_T4;
uint32_t freqTotal_T6;

SemaphoreHandle_t freqMutex_T3, freqMutex_T4, ledMutex;

void IRAM_ATTR buttonT7_ISR() {
  unsigned long debounceTime = millis();
  // delay for debouncing
  if ((debounceTime - prevDebounceTime) > debounceDelay){
    prevDebounceTime = debounceTime;
    xSemaphoreGiveFromISR(ledMutex, NULL);
  }
}

void signal_T1(void * pvParameters){

  
  //Task 1: 

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreq = pdMS_TO_TICKS(4);
  
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFreq);
    monitor.jobStarted(1);
    digitalWrite(redOutput_T1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(0.25));
    digitalWrite(redOutput_T1, LOW);
    vTaskDelay(pdMS_TO_TICKS(0.05));
    digitalWrite(redOutput_T1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(0.3));
    digitalWrite(redOutput_T1, LOW);
    monitor.jobEnded(1);
    
  }
}

void signal_T2(void*){
  
  
  //Task 2: 
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreq = pdMS_TO_TICKS(3);

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFreq);
    monitor.jobStarted(2);
    digitalWrite(orangeOutput_T2, HIGH);
    vTaskDelay(pdMS_TO_TICKS(0.1));
    digitalWrite(orangeOutput_T2, LOW);
    vTaskDelay(pdMS_TO_TICKS(0.05));
    digitalWrite(orangeOutput_T2, HIGH);
    vTaskDelay(pdMS_TO_TICKS(0.2));
    digitalWrite(orangeOutput_T2, LOW);
    monitor.jobEnded(2);
  }
}

void poll_T3(void*){

  
  //Task 3: 
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreq = pdMS_TO_TICKS(10);
  
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFreq);

    monitor.jobStarted(3);
    
    xSemaphoreTake(freqMutex_T3, portMAX_DELAY);
    
    long tempT3 = (pulseIn(input_T3, !digitalRead(input_T3), freqTimeout_T3));
    if(tempT3 < 760 && tempT3 > 490) freq_T3 = 1000000 / (tempT3 * 2);
    
    xSemaphoreGive(freqMutex_T3);
    
    monitor.jobEnded(3);
  }
}

void poll_T4(void*){
  
  //Task 4: 

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreq = pdMS_TO_TICKS(10);

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFreq);

    monitor.jobStarted(4);
    
    xSemaphoreTake(freqMutex_T4, portMAX_DELAY);
    
    long tempT4 = (pulseIn(input_T4, !digitalRead(input_T4), freqTimeout_T4)); 
    if(tempT4 < 610 && tempT4 > 323) freq_T4 = 1000000 / (tempT4 * 2);
    
    xSemaphoreGive(freqMutex_T4);
    
    monitor.jobEnded(4);
  }
}

void call_T5(void*){

  // Task 5: Call monitor.doWork() method
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreq = pdMS_TO_TICKS(5);

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFreq);
    monitor.jobStarted(5);
    monitor.doWork();
    monitor.jobEnded(5);
    
  }
}

void sum_T6(void*){

  // Task 6: 

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreq = pdMS_TO_TICKS(100);

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFreq);

    xSemaphoreTake(freqMutex_T3, portMAX_DELAY);
    xSemaphoreTake(freqMutex_T4, portMAX_DELAY);

    freqTotal_T6 = freq_T3 + freq_T4;
    freqTotal_T6 = constrain(freqTotal_T6, 1499, 2500);
    if (freqTotal_T6 > 1500) digitalWrite(yellowLED_T6, HIGH);
    else digitalWrite(yellowLED_T6, LOW);

    xSemaphoreGive(freqMutex_T3);
    xSemaphoreGive(freqMutex_T4);
  }
}

void toggle_T7(void*){

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreq = pdMS_TO_TICKS(50);

  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFreq);

    if(xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE){

      ledT7_state = !ledT7_state;
      digitalWrite(greenLED_T7, ledT7_state);
      monitor.doWork();

    }
  }
}

void setup() {

  Serial.begin(115200);

  pinMode(redOutput_T1, OUTPUT);
  pinMode(orangeOutput_T2, OUTPUT);
  pinMode(input_T3, INPUT);
  pinMode(input_T4, INPUT);
  pinMode(yellowLED_T6, OUTPUT);
  pinMode(greenLED_T7, OUTPUT);
  pinMode(button_T7, INPUT_PULLUP);

  freqMutex_T3 = xSemaphoreCreateMutex();
  freqMutex_T4 = xSemaphoreCreateMutex();
  ledMutex = xSemaphoreCreateBinary();

  xSemaphoreGive(ledMutex);

  attachInterrupt(digitalPinToInterrupt(button_T7), buttonT7_ISR, RISING);

  monitor.startMonitoring();

  xTaskCreate(signal_T1, "Task 1", 4096, NULL, 3, &T1_handle);
  xTaskCreate(signal_T2, "Task 2", 4096, NULL, 3, &T2_handle);
  xTaskCreate(poll_T3, "Task 3", 8192, NULL, 2, &T3_handle);
  xTaskCreate(poll_T4, "Task 4", 8192, NULL, 2, &T4_handle);
  xTaskCreate(call_T5, "Task 5", 4096, NULL, 3, &T5_handle);
  xTaskCreate(sum_T6, "Task 6", 4096, NULL, 1, &T6_handle);
  xTaskCreate(toggle_T7, "Task 7 LED", 2048, NULL, 1, NULL);
}

// Main loop unused
void loop() {}