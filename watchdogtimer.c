#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define WDT_TIMEOUT 4              // Durée de timeout du Watchdog (en secondes)
#define LED_PIN PB0                // Définir la broche de la LED
#define BTN_PIN PD2                // Définir la broche du bouton

// Initialisation du Watchdog Timer
void WDT_Init(uint8_t prescaler) {
    wdt_reset();                  // Réinitialiser le Watchdog Timer
    MCUSR &= ~(1 << WDRF);         // Réinitialiser le flag du Watchdog Reset
    WDTCSR |= (1 << WDCE) | (1 << WDE); // Activer la modification des bits du WDT
    WDTCSR = (1 << WDIE) | prescaler;  // Activer l'interruption et définir le prescaler
}

// Réinitialisation du Watchdog Timer
void WDT_Reset() {
    wdt_reset();                  // Réinitialiser le Watchdog Timer
}

// Initialisation du Timer0 (utilisé pour une action rapide)
void Timer0_Init() {
    TCCR0A = (1 << WGM01);            // Mode CTC pour Timer0
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler de 1024
    TIMSK0 = (1 << TOIE0);             // Activer l'interruption de débordement
}

// Initialisation du Timer1 (utilisé pour une action moins fréquente)
void Timer1_Init() {
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // Mode CTC, prescaler de 1024
    TIMSK1 = (1 << OCIE1A);               // Activer l'interruption de comparaison
    OCR1A = 15624;                        // Valeur de comparaison pour une interruption toutes les 1 seconde
}

// Initialisation du bouton
void Button_Init() {
    DDRD &= ~(1 << BTN_PIN);             // Configurer la broche du bouton comme entrée
    PORTD |= (1 << BTN_PIN);             // Activer la résistance de tirage interne
}

// Initialisation de la LED
void LED_Init() {
    DDRB |= (1 << LED_PIN);              // Configurer la LED comme sortie
    PORTB &= ~(1 << LED_PIN);            // Éteindre la LED au départ
}

// Fonction pour alterner l'état de la LED
void Toggle_LED() {
    PORTB ^= (1 << LED_PIN);             // Changer l'état de la LED (allumer/éteindre)
}

// Interruption du Timer0 : Clignotement rapide de la LED
ISR(TIMER0_OVF_vect) {
    static uint8_t count = 0;           // Compteur pour contrôler la fréquence du clignotement
    count++;
    if (count >= 10) {
        Toggle_LED();                   // Clignoter la LED après 10 débordements
        count = 0;
    }
    WDT_Reset();                         // Réinitialiser le Watchdog Timer
}

// Interruption du Timer1 : Clignotement plus lent de la LED
ISR(TIMER1_COMPA_vect) {
    static uint8_t blink_count = 0;     // Compteur pour un clignotement plus lent
    blink_count++;
    if (blink_count >= 5) {
        Toggle_LED();                   // Clignoter la LED après 5 comparaisons
        blink_count = 0;
    }
}

// Interruption du Watchdog Timer : action lorsque le Watchdog expire
ISR(WDT_vect) {
    PORTB |= (1 << LED_PIN);            // Allumer la LED pour signaler un timeout
    _delay_ms(1000);                    // Attendre pendant 1 seconde
    PORTB &= ~(1 << LED_PIN);           // Éteindre la LED après l'attente
}

// Fonction principale
int main() {
    WDT_Init((1 << WDP3) | (1 << WDP0));  // Initialiser le Watchdog Timer avec un prescaler pour 4 secondes
    Timer0_Init();                        // Initialiser le Timer0
    Timer1_Init();                        // Initialiser le Timer1
    Button_Init();                        // Initialiser le bouton
    LED_Init();                           // Initialiser la LED
    sei();                                // Activer les interruptions globales

    while (1) {
        if (PIND & (1 << BTN_PIN)) {      // Vérifier si le bouton est pressé
            WDT_Reset();                  // Réinitialiser le Watchdog Timer si le bouton est pressé
        }
        _delay_ms(100);                   // Petit délai pour la gestion du bouton
    }

    return 0; // Ce point n'est jamais atteint
}
