#include "mbed.h"


#define FREQUENCY1   0.5
#define FREQUENCY2   1.0

Timer t;
Ticker flipper;

using namespace std::chrono;


#ifdef LED1
    DigitalOut led(LED1);
#endif


void flip()
{
    led = !led;
}


InterruptIn button(BUTTON1);


bool toggle_frequency = false;

void change_frequency()
{
    toggle_frequency = !toggle_frequency;
    if (toggle_frequency) {
        flipper.attach(&flip, FREQUENCY1); 
    } else {
        flipper.attach(&flip, FREQUENCY2); 
}
}

int main()
{
    // Configuration initiale
    flipper.attach(&flip, FREQUENCY2); // Démarrer avec la fréquence par défaut
    button.rise(&change_frequency);   // Attacher une interruption pour le bouton

    while (true) {
        // Mesurer le temps écoulé si nécessaire
        printf("The time taken was %llu milliseconds\n", 
               duration_cast<milliseconds>(t.elapsed_time()).count());
        
    }
}
