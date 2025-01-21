/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"


// Blinking rate in milliseconds
#define BLINKING_RATE     500ms


Timer t;
Ticker flipper;

using namespace std::chrono;

// Initialise the digital pin LED1 as an output
#ifdef LED1
    DigitalOut led(LED1);
#else
    bool led;
#endif

void chgmt1(){
    t.start();
	led=1;
}
void chgmt2(){
    led=0;
    t.stop();
}

void flip()
{
    led = !led;
}

//DigitalIn boutton(BUTTON1);
InterruptIn boutton(BUTTON1);

int main()
{

//printf("Holaaaaaaa\n");



boutton.rise(&chgmt1);
boutton.fall(&chgmt2);
flipper.attach(&flip, 1.0);

    while (true) {
    /*
        led = !led;
        ThisThread::sleep_for(BLINKING_RATE);
	printf("Holaaaaaaa\n");
	
	*/
	printf("The time taken was %llu milliseconds\n", duration_cast<milliseconds>(t.elapsed_time()).count());
	
	
	//printf("l'état du boutton est : %d\n", boutton.read());
    }
    
}

/// rise fall pour les interuptions
