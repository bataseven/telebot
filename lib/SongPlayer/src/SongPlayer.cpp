#include "SongPlayer.h"
#include <Metro.h>

SongPlayer::SongPlayer(int BuzzerPin) {
    buzzerPin = BuzzerPin;
    pinMode(buzzerPin, OUTPUT);

    songs[0].melody = NULL;
    songs[0].tempo = 1;
    songs[0].noteCount = 1;

    songs[1].melody = Melodies::Astronomia;
    songs[1].tempo = Melodies::Astronomia_tempo;
    songs[1].noteCount = sizeof(Melodies::Astronomia) / sizeof(Melodies::Astronomia[0]) / 2;

    songs[2].melody = Melodies::LionSleeps;
    songs[2].tempo = Melodies::LionSleeps_tempo;
    songs[2].noteCount = sizeof(Melodies::LionSleeps) / sizeof(Melodies::LionSleeps[0]) / 2;

    songs[3].melody = Melodies::HarryPotter;
    songs[3].tempo = Melodies::HarryPotter_tempo;
    songs[3].noteCount = sizeof(Melodies::HarryPotter) / sizeof(Melodies::HarryPotter[0]) / 2;

    songs[4].melody = Melodies::LowBattery;
    songs[4].tempo = Melodies::LowBattery_tempo;
    songs[4].noteCount = sizeof(Melodies::LowBattery) / sizeof(Melodies::LowBattery[0]) / 2;
}

void SongPlayer::setSong(int song) {
    song = constrain(song, 0, SONG_COUNT);
    if (song == selectedSongIndex)
        return;
    selectedSongIndex = song;
    selectedSong = songs[selectedSongIndex];

    thisNote = 0;

    tempo = selectedSong.tempo;
    wholenote = (60000 * 4) / tempo;
    noteCount = selectedSong.noteCount;
}

void SongPlayer::update() {
    play(selectedSongIndex);
}

void SongPlayer::play(int selectedSongIndex) {
    if (!noteTimer.check())
        return;

    if (selectedSongIndex == 0) {
        noTone(buzzerPin);
        return;
    }

    divider = selectedSong.melody[thisNote + 1];
    if (divider > 0) {
        noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
        noteDuration = (wholenote) / abs(divider);
        noteDuration *= 1.5;
    } else {
        noteDuration = 0;
    }

    noteToPlay = selectedSong.melody[thisNote];
    if (noteToPlay == 0) {
        noTone(buzzerPin);
    } else {
        tone(buzzerPin, noteToPlay, noteDuration * 0.95);
    }
    noteTimer.interval(noteDuration);
    thisNote += 2;
    thisNote %= (noteCount * 2);

    if (thisNote == 0 && !repeat) {
        selectedSongIndex = 0;
    }
}

void SongPlayer::setPin(int BuzzerPin) {
    buzzerPin = BuzzerPin;
    pinMode(buzzerPin, OUTPUT);
}

int SongPlayer::getPin() {
    return buzzerPin;
}

void SongPlayer::setRepeat(bool Repeat) {
    repeat = Repeat;
}