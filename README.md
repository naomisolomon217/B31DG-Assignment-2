# B31DG-Assignment-2
Problem
You need to develop two different implementations of a machine monitor system using your ESP32 kit.

First program

Your first program needs to implement a cyclic executive (see slides and other material published on Unit 6 in Canvas). You cannot use FreeRTOS’s tasks or other FreeRTOS’s features, but you are allowed to use a single Ticker object (you need to include the Arduino library, i.e. Ticker.h) to implement the major cycle of your cyclic executive. Note that it will be difficult to satisfy RT requirements without accurate timing from a Ticker.


Second program

Your second program must be implemented using FreeRTOS. 
You can use any of the FreeRTOS features (tasks, timers, queues, semaphores/mutexes…) you consider useful for your program.
It is your choice whether you implement tasks using actual FreeRTOS Tasks or callback functions called by FreeRTOS software timers, or a combination of the two approaches.


1.	Output a digital signal. This should be HIGH for 250μs, then LOW for 50μs, then HIGH again for 300μs, then LOW again. You should use spin wait loops (e.g. using calls to standard delay()) to implement the short time intervals you need. Do not use interrupts. 
2.	Output a second digital signal. This should be HIGH for 100μs, then LOW for 50μs, then HIGH again for 200μs, then LOW again. You should use spin wait loops (e.g. using calls to standard delay()) to implement the short time intervals you need. Do not use interrupts.
3.	Measure the frequency of a 3.3v square wave signal. The frequency of the wave signal in input will be in the range 666Hz to 1000Hz and the signal will be a standard square wave (50% duty cycle). Let’s call this frequency F1 and measure it in Hz. You should measure F1 by polling the signal. For this exercise, do not use interrupts (which would be the more efficient method).

4.	Measure the frequency of a second 3.3v square wave signal. The frequency of the wave signal in input will be in the range 833Hz to 1500Hz and the signal will be a standard square wave (50% duty cycle). Let’s call this frequency F2 and measure it in Hz. You should measure F2 by polling the signal. For this exercise, do not use interrupts (which would be the more efficient method).

5.	Call the monitor’s method doWork().

6.	Use a LED to indicate whether the sum of the two frequencies F1 and F2 is greater than 1500, i.e. when F1+F2 > 1500. 

7.	Monitor a pushbutton. Toggle the state of a second LED and call the monitor’s method doWork() whenever the pushbutton is pressed.

You should consider that requirements 1-5 must be repeated, and that they have hard real-time periodic deadlines. 

Specifically, from the time you call the monitor’s method startMonitoring() method, this will assume:
 
-	A deadline for (1) every 4ms   [Rate = 250Hz] 
-	A deadline for (2) every 3ms   [Rate = 333Hz] 
-	A deadline for (3) every 10ms [Rate = 100Hz]
-	A deadline for (4) every 10ms [Rate = 100Hz]  
-	A deadline for (5) every 5ms.  [Rate = 200Hz]  

