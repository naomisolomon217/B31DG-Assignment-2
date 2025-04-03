#include <stdio.h>
#include <stdint.h>
#include "B31DGMonitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Define monitor with the provided ID
B31DGCyclicExecutiveMonitor monitor(0);

// Pin definitions
#define redOutput_T1 21
#define orangeOutput_T2 19  
#define input_T3 2
#define input_T4 15 
#define yellowLED_T6 18
#define greenLED_T7 25  
#define button_T7 26

// Timeout constants
const uint32_t freqTimeout_T3 = 1500;
const uint32_t freqTimeout_T4 = 1200;
const uint8_t debounceDelay = 20;

// Global variables with mutex protection
SemaphoreHandle_t freqMutex;
SemaphoreHandle_t buttonSemaphore;
volatile uint32_t freq_T3 = 0;
volatile uint32_t freq_T4 = 0;

// Task handles
TaskHandle_t task1Handle = NULL;
TaskHandle_t task2Handle = NULL;
TaskHandle_t task3Handle = NULL;
TaskHandle_t task4Handle = NULL;
TaskHandle_t task5Handle = NULL;
TaskHandle_t task6Handle = NULL;
TaskHandle_t task7Handle = NULL;

// Task function declarations
void signal_T1_task(void *pvParameters);
void signal_T2_task(void *pvParameters);
void poll_T3_task(void *pvParameters);
void poll_T4_task(void *pvParameters);
void call_T5_task(void *pvParameters);
void sum_T6_task(void *pvParameters);
void button_T7_task(void *pvParameters);

// Button ISR (task 7)
void IRAM_ATTR b_T7_ISR() {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > debounceDelay)
  {
    lastDebounceTime = currentTime;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR();
    }
  }
}

void setup() {
  
  Serial.begin(115200);

  pinMode(button_T7, INPUT_PULLUP); 
  pinMode(redOutput_T1, OUTPUT);
  pinMode(orangeOutput_T2, OUTPUT);
  pinMode(yellowLED_T6, OUTPUT);
  pinMode(greenLED_T7, OUTPUT);
  pinMode(input_T3, INPUT);
  pinMode(input_T4, INPUT);
  
  // Set initial state for outputs
  digitalWrite(redOutput_T1, LOW);
  digitalWrite(orangeOutput_T2, LOW);
  digitalWrite(yellowLED_T6, LOW);
  digitalWrite(greenLED_T7, LOW);
  
  // Create mutex for shared frequency values
  freqMutex = xSemaphoreCreateMutex();
  
  // Create semaphore for button events
  buttonSemaphore = xSemaphoreCreateBinary();
  
  // Attach interrupt for button
  attachInterrupt(digitalPinToInterrupt(button_T7), b_T7_ISR, FALLING);

  
  monitor.startMonitoring();
  
  delayMicroseconds(100);
  Serial.println("Free RTOS implementation running");
  
  // Create tasks with their priorities
  // Task 1 - 250Hz (4ms period)
  xTaskCreate(signal_T1_task, "T1_Signal", 2048, NULL, 3, &task1Handle);
  
  // Task 2 - 333Hz (3ms period)
  xTaskCreate(signal_T2_task, "T2_Signal", 2048, NULL, 4, &task2Handle);
  
  // Task 3 - 100Hz (10ms period)
  xTaskCreate(poll_T3_task, "T3_Poll", 2048, NULL, 2, &task3Handle);
  
  // Task 4 - 100Hz (10ms period)
  xTaskCreate(poll_T4_task, "T4_Poll", 2048, NULL, 2, &task4Handle);
  
  // Task 5 - 200Hz (5ms period)
  xTaskCreate(call_T5_task, "T5_DoWork", 2048, NULL, 3, &task5Handle);
  
  // Task 6 - checks frequency sum (no hard deadline, lower priority)
  xTaskCreate(sum_T6_task, "T6_Sum", 2048, NULL, 1, &task6Handle);
  
  // Task 7 - handles button presses (event-driven)
  xTaskCreate(button_T7_task, "T7_Button", 1024, NULL, 1, &task7Handle);
}

void signal_T1_task(void *pvParameters) {
  // Get current tick count for vTaskDelayUntil
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(4); 

  /* Task 1: Output digital signal 
     high 250us - low 50us - high 300us - low 
     measured to take 602us */

  while (1) {
    
    monitor.jobStarted(1);
    
    digitalWrite(redOutput_T1, HIGH);
    delayMicroseconds(250);
    digitalWrite(redOutput_T1, LOW);
    delayMicroseconds(50);
    digitalWrite(redOutput_T1, HIGH);
    delayMicroseconds(300);
    digitalWrite(redOutput_T1, LOW);
    
  
    monitor.jobEnded(1);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void signal_T2_task(void *pvParameters) {
  // Get current tick count for vTaskDelayUntil
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(3); // 3ms period = 333Hz

  /* Task 2: Output digital signal 
  high 100us - low 50us - high 200us - low 
  measured to take 353us */

  while (1) {
    
    monitor.jobStarted(2);
    
    digitalWrite(orangeOutput_T2, HIGH);
    delayMicroseconds(100);
    digitalWrite(orangeOutput_T2, LOW);
    delayMicroseconds(50);
    digitalWrite(orangeOutput_T2, HIGH);
    delayMicroseconds(200);
    digitalWrite(orangeOutput_T2, LOW);
    
    
    monitor.jobEnded(2);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void poll_T3_task(void *pvParameters) {
  // Get current tick count for vTaskDelayUntil
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms period = 100Hz
  uint32_t local_freq;

  /* Task 3: Poll frequency of input 3.3v square wave signal
            measured between 666Hz (worst case execution), 
            and 1000Hz (best case execution)*/

  while (1) {
    
    monitor.jobStarted(3);
    
    // Poll frequency of first signal
    long tempT3 = (pulseIn(input_T3, !digitalRead(input_T3), freqTimeout_T3));
    
    // Calculate frequency if within valid range
    if(tempT3 < 760 && tempT3 > 490) {
      local_freq = 1000000 / (tempT3 * 2);
      
      // Update shared frequency with mutex protection
      if (xSemaphoreTake(freqMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        freq_T3 = local_freq;
        xSemaphoreGive(freqMutex);
      }
    }
    
    monitor.jobEnded(3);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void poll_T4_task(void *pvParameters) {
  // Get current tick count for vTaskDelayUntil
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms period = 100Hz
  uint32_t local_freq;

  /* Task 4: Poll frequency of input 3.3v square wave signal
            measured between 833Hz (worst case execution), 
            and 1500Hz (best case execution)
*/

  while (1) {
    
    monitor.jobStarted(4);
    
    // Poll frequency of second signal
    long tempT4 = (pulseIn(input_T4, !digitalRead(input_T4), freqTimeout_T4));
    
    // Calculate frequency if within valid range
    if(tempT4 < 610 && tempT4 > 323) {
      local_freq = 1000000 / (tempT4 * 2);
      
      // Update shared frequency with mutex protection
      if (xSemaphoreTake(freqMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        freq_T4 = local_freq;
        xSemaphoreGive(freqMutex);
      }
    }
  
    monitor.jobEnded(4);
    
    // Wait for the next cycle using vTaskDelayUntil for precise timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void call_T5_task(void *pvParameters) {
  // Get current tick count for vTaskDelayUntil
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms period = 200Hz

  while (1) {
    
    monitor.jobStarted(5);
    
    // Task 5: Call doWork method
    monitor.doWork();
  
    monitor.jobEnded(5);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sum_T6_task(void *pvParameters) {
  uint32_t local_freq_T3, local_freq_T4, freqTotal_T6;
  
  const TickType_t xDelay = pdMS_TO_TICKS(20);
   
   // Task 6: LED control based on frequency sum (of task 3 and 4)

  while (1) {
    // Read frequencies with mutex protection
    if (xSemaphoreTake(freqMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      local_freq_T3 = freq_T3;
      local_freq_T4 = freq_T4;
      xSemaphoreGive(freqMutex);
    }
    
    // Calculate total frequency
    freqTotal_T6 = local_freq_T3 + local_freq_T4;
    
    // Set LED based on frequency threshold
    if (freqTotal_T6 > 1500) {
      digitalWrite(yellowLED_T6, HIGH);
    } else {
      digitalWrite(yellowLED_T6, LOW);
    }
    
    // Run this task at lower priority, with a small delay
    vTaskDelay(xDelay);
  }
}

void button_T7_task(void *pvParameters) {
  
  static bool led_state_T7 = false; // green LED state
  
  while (1) {
    // Wait for notification from ISR
    if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
      // Toggle LED state
      led_state_T7 = !led_state_T7;
      digitalWrite(greenLED_T7, led_state_T7);
      
      // Call doWork method 
      monitor.doWork();
    }
  }
}

void loop() {
 
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
