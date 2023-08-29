#ifndef SongPlayer_H
#define SongPlayer_H

#include <Metro.h>
#include "Melodies.h"
#include "Arduino.h"

#define SONG_COUNT 4

struct Song {
    int *melody;
    int tempo;
    int noteCount;
};

class SongPlayer {
public:
    // Constructor
    SongPlayer(int pin);

    // Methods
    void update();
    void setSong(int index);
    void setPin(int pin);
    int getPin();
    void setRepeat(bool repeat);

private:
    void play(int selectedSongIndex);
    int buzzerPin;
    Metro noteTimer = Metro(1);
    int selectedSongIndex = 0;
    Song songs[SONG_COUNT + 1];
    Song selectedSong;
    int noteToPlay = 0;
    int tempo = 1;
    int noteCount = 1;
    int thisNote = 0;
    int noteDuration = 0;
    int wholenote = (60000 * 4) / tempo; // Duration of a whole note
    int divider = 1;
    bool repeat = true;
};
#endif
