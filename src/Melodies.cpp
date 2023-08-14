#include "Melodies.h"
#include <Metro.h>

Melodies::Melodies(int BuzzerPin) {
  _BuzzerPin = BuzzerPin; 
}

void Melodies::play(int selectedSong) {
  if (!_started)
    _start_Counter++;

  if (_start_Counter == _start_Threshold) {
    _started = true;
    _Selected_Song = selectedSong;

  }
  else
    _Selected_Song = 0;

  if (_noteTimer.check()) {

    noTone(_BuzzerPin);

    if (_Song_Changed) {
      Serial.println("Changed");
      _Song_Changed = false;
      _thisNote = 0;
      _noteToPlay = 0;
      switch (_Selected_Song) {
        case 0:
        default:
          _noteToPlay = 0;
          _noteDuration = 100;
          noTone(_BuzzerPin);
          break;
        case 1:
          _tempo = Song_Tempos[_Selected_Song];
          _wholenote = (60000 * 4) / _tempo;
          _notes = sizeof(HarryPotter) / sizeof(HarryPotter[0]) / 2;
          break;
        case 2:
          _tempo = Song_Tempos[_Selected_Song];
          _wholenote = (60000 * 4) / _tempo;
          _notes = sizeof(LionSleeps) / sizeof(LionSleeps[0]) / 2;
          break;
        case 3:
          _tempo = Song_Tempos[_Selected_Song];
          _wholenote = (60000 * 4) / _tempo;
          _notes = sizeof(Astronomia) / sizeof(Astronomia[0]) / 2;
          break;
        case 4:
          _tempo = Song_Tempos[_Selected_Song];
          _wholenote = (60000 * 4) / _tempo;
          _notes = sizeof(LowBattery) / sizeof(LowBattery[0]) / 2;
          break;
      }
    }
    switch (_Selected_Song) {
      case 0:
      default:
        _noteToPlay = 0;
        _noteDuration = 100;
        noTone(_BuzzerPin);
        break;
      case 1:
        _divider = HarryPotter[_thisNote + 1];
        if (_divider > 0) {
          _noteDuration = (_wholenote) / _divider;
        } else if (_divider < 0) {
          _noteDuration = (_wholenote) / abs(_divider);
          _noteDuration *= 1.5;
        }
        _noteToPlay = HarryPotter[_thisNote];
        break;
      case 2:
        _divider = LionSleeps[_thisNote + 1];
        if (_divider > 0) {
          _noteDuration = (_wholenote) / _divider;
        } else if (_divider < 0) {
          _noteDuration = (_wholenote) / abs(_divider);
          _noteDuration *= 1.5;
        }
        _noteToPlay = LionSleeps[_thisNote];
        break;
      case 3:
        _divider = Astronomia[_thisNote + 1];
        if (_divider > 0) {
          _noteDuration = (_wholenote) / _divider;
        } else if (_divider < 0) {
          _noteDuration = (_wholenote) / abs(_divider);
          _noteDuration *= 1.5;
        }
        _noteToPlay = Astronomia[_thisNote];
        break;
      case 4:
        _divider = LowBattery[_thisNote + 1];
        if (_divider > 0) {
          _noteDuration = (_wholenote) / _divider;
        } else if (_divider < 0) {
          _noteDuration = (_wholenote) / abs(_divider);
          _noteDuration *= 1.5;
        }
        _noteToPlay = LowBattery[_thisNote];
        break;
    }
    tone(_BuzzerPin, _noteToPlay , _noteDuration * 0.95);
    _noteTimer.interval(_noteDuration);
    _thisNote += 2;
    _thisNote %= (_notes * 2);
  }

  if (_Selected_Song != _Selected_Song_Prev) {
    _Song_Changed = true;
  }
  _Selected_Song_Prev = _Selected_Song;

}
