#ifndef Melodies_H
#define Melodies_H
#include "Notes.h"

class Melodies {
public:
    static inline int Astronomia_tempo = 126;
    static inline int LionSleeps_tempo = 122;
    static inline int HarryPotter_tempo = 144;
    static inline int LowBattery_tempo = 120;

    static inline int Astronomia[212] = {
        REST, 8, REST, 8,
        NOTE_AS4, 8, NOTE_AS4, 8, NOTE_AS4, 8, NOTE_AS4, 8,
        NOTE_AS4, 8, NOTE_AS4, 8, NOTE_AS4, 8, NOTE_AS4, 8,
        NOTE_AS4, 8, NOTE_AS4, 8, NOTE_AS4, 8, NOTE_AS4, 8,
        NOTE_D5, 8, NOTE_D5, 8, NOTE_D5, 8, NOTE_D5, 8,
        NOTE_C5, 8, NOTE_C5, 8, NOTE_C5, 8, NOTE_C5, 8,
        NOTE_F5, 8, NOTE_F5, 8, NOTE_F5, 8, NOTE_F5, 8,
        NOTE_G5, 8, NOTE_G5, 8, NOTE_G5, 8, NOTE_G5, 8,
        NOTE_G5, 8, NOTE_G5, 8, NOTE_G5, 8, NOTE_G5, 8,
        NOTE_G5, 8, NOTE_G5, 8, NOTE_G5, 8, NOTE_G5, 8,
        NOTE_C5, 8, NOTE_AS4, 8, NOTE_A4, 8, NOTE_F4, 8,
        NOTE_G4, 8, 0, 8, NOTE_G4, 8, NOTE_D5, 8,
        NOTE_C5, 8, 0, 8, NOTE_AS4, 8, 0, 8,
        NOTE_A4, 8, 0, 8, NOTE_A4, 8, NOTE_A4, 8,
        NOTE_C5, 8, 0, 8, NOTE_AS4, 8, NOTE_A4, 8,
        NOTE_G4, 8, 0, 8, NOTE_G4, 8, NOTE_AS5, 8,
        NOTE_A5, 8, NOTE_AS5, 8, NOTE_A5, 8, NOTE_AS5, 8,
        NOTE_G4, 8, 0, 8, NOTE_G4, 8, NOTE_AS5, 8,
        NOTE_A5, 8, NOTE_AS5, 8, NOTE_A5, 8, NOTE_AS5, 8,
        NOTE_G4, 8, 0, 8, NOTE_G4, 8, NOTE_D5, 8,
        NOTE_C5, 8, 0, 8, NOTE_AS4, 8, 0, 8,
        NOTE_A4, 8, 0, 8, NOTE_A4, 8, NOTE_A4, 8,
        NOTE_C5, 8, 0, 8, NOTE_AS4, 8, NOTE_A4, 8,
        NOTE_G4, 8, 0, 8, NOTE_G4, 8, NOTE_AS5, 8,
        NOTE_A5, 8, NOTE_AS5, 8, NOTE_A5, 8, NOTE_AS5, 8,
        NOTE_G4, 8, 0, 8, NOTE_G4, 8, NOTE_AS5, 8,
        NOTE_A5, 8, NOTE_AS5, 8, NOTE_A5, 8, NOTE_AS5, 8};

    static inline int LionSleeps[604] = {
        REST, 2, NOTE_F4, 4, NOTE_G4, 4, NOTE_A4, 8, NOTE_G4, 4, NOTE_A4, 8, // 1
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_C4, 8, NOTE_C4, 4, NOTE_C4, 8, NOTE_C4, 4,
        NOTE_C4, 1, // 1st ending

        NOTE_F4, 4, NOTE_G4, 4, NOTE_A4, 8, NOTE_G4, 4, NOTE_A4, 8, // repeats from 1
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_C4, 8, NOTE_C4, 4, NOTE_C4, 8, NOTE_C4, 4,
        NOTE_C4, -2, REST, -8, NOTE_A4, 16, // 2nd ending

        NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, // 6
        NOTE_AS4, -8, NOTE_AS4, 16, NOTE_AS4, -8, NOTE_AS4, 16, NOTE_AS4, -8, NOTE_AS4, 16, NOTE_AS4, -8, NOTE_AS4, 16,
        NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16,
        NOTE_G4, -8, NOTE_G4, 16, NOTE_G4, -8, NOTE_G4, 16, NOTE_G4, -8, NOTE_G4, 16, NOTE_G4, -8, NOTE_G4, 16,

        NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, // 10
        NOTE_AS4, -8, NOTE_AS4, 16, NOTE_AS4, -8, NOTE_AS4, 16, NOTE_AS4, -8, NOTE_AS4, 16, NOTE_AS4, -8, NOTE_AS4, 16,
        NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16, NOTE_A4, -8, NOTE_A4, 16,
        NOTE_G4, -8, NOTE_G4, 16, NOTE_G4, -8, NOTE_G4, 16, NOTE_G4, -8, NOTE_G4, 16, NOTE_G4, -8, NOTE_G4, 16,

        NOTE_F4, 4, NOTE_G4, 4, NOTE_A4, 8, NOTE_G4, 4, NOTE_A4, 8, // 14
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_G4, 4, NOTE_F4, 4, NOTE_A4, 4,
        NOTE_G4, 1,
        NOTE_C5, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_A4, 4, NOTE_C5, 8,
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_G4, 4, NOTE_F4, 4, NOTE_A4, 4,
        NOTE_G4, 1,

        NOTE_C5, 1, // 22
        NOTE_C5, 4, NOTE_AS4, 8, NOTE_C5, 8, NOTE_AS4, 2,
        NOTE_A4, 4, NOTE_C4, 8, NOTE_C4, 4, NOTE_C4, 8, NOTE_C4, 4,
        NOTE_C4, 1,

        REST, 4, NOTE_A4, 8, NOTE_G4, 8, NOTE_F4, 8, NOTE_E4, 8, NOTE_D4, 8, NOTE_C4, 8,
        NOTE_D4, 1,
        REST, 4, NOTE_A4, 8, NOTE_G4, 8, NOTE_F4, 8, NOTE_E4, 8, NOTE_D4, 8, NOTE_C4, 8,
        NOTE_D4, 1,

        NOTE_F4, 4, NOTE_G4, 4, NOTE_A4, 8, NOTE_G4, 4, NOTE_A4, 8, // repeats from 14
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_G4, 4, NOTE_F4, 4, NOTE_A4, 4,
        NOTE_G4, 1,
        NOTE_C5, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_A4, 4, NOTE_C5, 8,
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_G4, 4, NOTE_F4, 4, NOTE_A4, 4,
        NOTE_G4, 1,

        NOTE_C5, 1, // 22
        NOTE_C5, 4, NOTE_AS4, 8, NOTE_C5, 8, NOTE_AS4, 2,
        NOTE_A4, 4, NOTE_C4, 8, NOTE_C4, 4, NOTE_C4, 8, NOTE_C4, 4,
        NOTE_C4, 1,

        REST, 4, NOTE_A4, 8, NOTE_G4, 8, NOTE_F4, 8, NOTE_E4, 8, NOTE_D4, 8, NOTE_C4, 8,
        NOTE_D4, 1,
        REST, 4, NOTE_A4, 8, NOTE_G4, 8, NOTE_F4, 8, NOTE_E4, 8, NOTE_D4, 8, NOTE_C4, 8,
        NOTE_D4, 1,

        NOTE_F4, 4, NOTE_G4, 4, NOTE_A4, 8, NOTE_G4, 4, NOTE_A4, 8, // 30
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_C4, 8, NOTE_C4, 4, NOTE_C4, 8, NOTE_C4, 4,
        NOTE_C4, 1,

        NOTE_F4, 4, NOTE_G4, 4, NOTE_A4, 8, NOTE_G4, 4, NOTE_A4, 8, // repeats from 14 (again)
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_G4, 4, NOTE_F4, 4, NOTE_A4, 4,
        NOTE_G4, 1,
        NOTE_C5, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_A4, 4, NOTE_C5, 8,
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_G4, 4, NOTE_F4, 4, NOTE_A4, 4,
        NOTE_G4, 1,

        NOTE_C5, 1, // 22
        NOTE_C5, 4, NOTE_AS4, 8, NOTE_C5, 8, NOTE_AS4, 2,
        NOTE_A4, 4, NOTE_C4, 8, NOTE_C4, 4, NOTE_C4, 8, NOTE_C4, 4,
        NOTE_C4, 1,

        REST, 4, NOTE_A4, 8, NOTE_G4, 8, NOTE_F4, 8, NOTE_E4, 8, NOTE_D4, 8, NOTE_C4, 8,
        NOTE_D4, 1,
        REST, 4, NOTE_A4, 8, NOTE_G4, 8, NOTE_F4, 8, NOTE_E4, 8, NOTE_D4, 8, NOTE_C4, 8,
        NOTE_D4, 1,

        NOTE_F4, 4, NOTE_G4, 4, NOTE_A4, 8, NOTE_G4, 4, NOTE_A4, 8, // 30
        NOTE_AS4, 4, NOTE_A4, 4, NOTE_G4, 8, NOTE_F4, 4, NOTE_G4, 8,
        NOTE_A4, 4, NOTE_C4, 8, NOTE_C4, 4, NOTE_C4, 8, NOTE_C4, 4,
        NOTE_C4, 1};
    static inline int HarryPotter[124] = {
        // Hedwig's theme fromn the Harry Potter Movies // Socre from https://musescore.com/user/3811306/scores/4906610

        REST, 2, NOTE_D4, 4,
        NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4,
        NOTE_G4, 2, NOTE_D5, 4,
        NOTE_C5, -2,
        NOTE_A4, -2,
        NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4,
        NOTE_F4, 2, NOTE_GS4, 4,
        NOTE_D4, -1,
        NOTE_D4, 4,

        NOTE_G4, -4, NOTE_AS4, 8, NOTE_A4, 4, // 10
        NOTE_G4, 2, NOTE_D5, 4,
        NOTE_F5, 2, NOTE_E5, 4,
        NOTE_DS5, 2, NOTE_B4, 4,
        NOTE_DS5, -4, NOTE_D5, 8, NOTE_CS5, 4,
        NOTE_CS4, 2, NOTE_B4, 4,
        NOTE_G4, -1,
        NOTE_AS4, 4,

        NOTE_D5, 2, NOTE_AS4, 4, // 18
        NOTE_D5, 2, NOTE_AS4, 4,
        NOTE_DS5, 2, NOTE_D5, 4,
        NOTE_CS5, 2, NOTE_A4, 4,
        NOTE_AS4, -4, NOTE_D5, 8, NOTE_CS5, 4,
        NOTE_CS4, 2, NOTE_D4, 4,
        NOTE_D5, -1,
        REST, 4, NOTE_AS4, 4,

        NOTE_D5, 2, NOTE_AS4, 4, // 26
        NOTE_D5, 2, NOTE_AS4, 4,
        NOTE_F5, 2, NOTE_E5, 4,
        NOTE_DS5, 2, NOTE_B4, 4,
        NOTE_DS5, -4, NOTE_D5, 8, NOTE_CS5, 4,
        NOTE_CS4, 2, NOTE_AS4, 4,
        NOTE_G4, -1};
    static inline int LowBattery[4] = {1000, 16, 0, 16};
};
#endif