/*  Arduino Motor Controller Firmware */
#include "Arduino.h"

// This must be set to 0 in order to run motor
#define DEBUG_MODE 0
#define HALL_DEBUG_MODE 0
#define HI_LO_DEBUG_MODE 0
#define THROTTLE_DEBUG_MODE 0


#define PIN_MOSFET_A_LO D11 // 11
#define PIN_MOSFET_A_HI D10 // 10

#define PIN_MOSFET_B_LO D5 // 9
#define PIN_MOSFET_B_HI D3 // 6

#define PIN_MOSFET_C_LO D9 // 5
#define PIN_MOSFET_C_HI D6 // 3

#define PIN_HALL_A A3
#define PIN_HALL_B A2
#define PIN_HALL_C A1

#define PIN_TEST_DRIVE 7

#define PIN_THROTTLE A4

void setup() {

    pinMode(PIN_MOSFET_A_LO, OUTPUT);
    pinMode(PIN_MOSFET_A_HI, OUTPUT);

    pinMode(PIN_MOSFET_B_LO, OUTPUT);
    pinMode(PIN_MOSFET_B_HI, OUTPUT);

    pinMode(PIN_MOSFET_C_LO, OUTPUT);
    pinMode(PIN_MOSFET_C_HI, OUTPUT);

    analogWriteFrequency(800000);

    analogWrite(PIN_MOSFET_A_LO, 0);
    analogWrite(PIN_MOSFET_A_HI, 0);

    analogWrite(PIN_MOSFET_B_HI, 0);
    analogWrite(PIN_MOSFET_B_LO, 0);

    analogWrite(PIN_MOSFET_C_LO, 0);
    analogWrite(PIN_MOSFET_C_HI, 0);

    pinMode(PIN_HALL_A, INPUT_PULLUP);
    pinMode(PIN_HALL_B, INPUT_PULLUP);
    pinMode(PIN_HALL_C, INPUT_PULLUP);

    pinMode(PIN_TEST_DRIVE, INPUT_PULLUP);

    if (DEBUG_MODE) {
        Serial.begin(9600);
    }
    if (HI_LO_DEBUG_MODE) {
        Serial.begin(9600);
    }
}

int counter = 0;

void loop() {
    bool hallA = digitalRead(PIN_HALL_A);
    bool hallB = digitalRead(PIN_HALL_B);
    bool hallC = digitalRead(PIN_HALL_C); 
  
    // Reading from potentiometer
    int reading = analogRead(PIN_THROTTLE);

    int dutyCycle = (reading/4);

    // if (digitalRead(PIN_TEST_DRIVE) == LOW){
    //     dutyCycle = 70;
    // }

    if (DEBUG_MODE){
        counter++;
        if (counter == 100){
            counter = 0;
            if (HALL_DEBUG_MODE) {
                Serial.print("hallA: ");
                Serial.print(hallA);
                Serial.print(" - hallB: ");
                Serial.print(hallB);
                Serial.print(" - hallC: ");
                Serial.println(hallC);
            }
            if (HI_LO_DEBUG_MODE){
                int A_HI = analogRead(PIN_MOSFET_A_HI);
                int A_LO = analogRead(PIN_MOSFET_A_LO);
                int B_HI = analogRead(PIN_MOSFET_B_HI);
                int B_LO = analogRead(PIN_MOSFET_B_LO);
                int C_HI = analogRead(PIN_MOSFET_C_HI);
                int C_LO = analogRead(PIN_MOSFET_C_LO);

                Serial.print("Pin Mosfet A Hi: ");
                Serial.print(A_HI);
                Serial.print("  -  Pin Mosfet A LO: ");
                Serial.print(A_LO);
                Serial.print("  Pin Mosfet B Hi: ");
                Serial.print(B_HI);
                Serial.print("  -  Pin Mosfet B LO: ");
                Serial.print(B_LO);
                Serial.print("  Pin Mosfet C Hi: ");
                Serial.print(C_HI);
                Serial.print("  -  Pin Mosfet C LO: ");
                Serial.println(C_LO);
            }
            if (THROTTLE_DEBUG_MODE){
                Serial.print("Throttle output: ");
                Serial.print(reading);
                Serial.print(" - Duty Cycle: ");
                Serial.println(dutyCycle);
            }
        }
    }
    
    // Combine hall sensor states into a single integer value
    int hallState = (hallA << 2) | (hallB << 1) | hallC; // Creates a 3-bit number representing hall sensor states

    switch (hallState) {
        case 0b111: // hallA, hallB, hallC 
        case 0b000: // !hallA, !hallB, !hallC
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_A_HI, 0); //ALL WORKING
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
            break;
            
        case 0b101: // hallA && !hallB && hallC
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_B_LO, 255); // YES
            analogWrite(PIN_MOSFET_C_HI, dutyCycle); // YES
            break;

        case 0b100: // hallA && !hallB && !hallC
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
            analogWrite(PIN_MOSFET_A_HI, dutyCycle); // YES
            analogWrite(PIN_MOSFET_B_LO, 255); // YES
            break;

        case 0b110: // hallA && hallB && !hallC
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
            analogWrite(PIN_MOSFET_A_HI, dutyCycle); // YES
            analogWrite(PIN_MOSFET_C_LO, 255); // YES
            break;

        case 0b010: // !hallA && hallB && !hallC
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
            analogWrite(PIN_MOSFET_B_HI, dutyCycle); // YES
            analogWrite(PIN_MOSFET_C_LO, 255); // YES
            break;

        case 0b011: // !hallA && hallB && hallC
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
            analogWrite(PIN_MOSFET_A_LO, 255); // YES
            analogWrite(PIN_MOSFET_B_HI, dutyCycle); // YES
            break; 

        case 0b001: // !hallA && !hallB && hallC
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_A_LO, 255); // YES
            analogWrite(PIN_MOSFET_C_HI, dutyCycle); // YES
            break;
    }
}
