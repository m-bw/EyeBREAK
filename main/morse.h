#ifndef MAIN_MORSE_H

#define MAIN_MORSE_H

#include <string_view>
#include <utility>
#include <vector>
#include "esp_timer.h"

enum class Morse : uint8_t {
  Dot = 0,
  Dash = 1,
  Invalid = 255,
};

class MorseCode {
 public:
  MorseCode();
  // Updates the internal Morse code state with the current 1/0 signal and time.
  // Returns a character if the update results in one.
  // No result is '\0', and backspace is '\b'.
  // NOTE: signal is defined as whether the eye is closed, so the classifier
  // output needs to be inverted.
  std::pair<Morse, char> Update(bool signal, int64_t start_time);

 private:
  // Decode the character using the binary tree encoding
  char GetMorse();

  // Whether the character/word end has been handled for the current eye open phase
  bool end_char_handled{}, end_word_handled{}, started{};
  // TODO use StackVector/static_vector/array?
  std::vector<Morse> morse_state;
  // The last timestamp the eye has been closed/open
  int64_t last_open_time{}, last_close_time{};
};

#endif // MAIN_MORSE_H
