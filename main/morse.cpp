#include "morse.h"

// length of morse unit in us
constexpr int MORSE_UNIT = 300000;
constexpr int MORSE_FARNSWORTH_UNIT = 2 * MORSE_UNIT;
// time to delete previous character
constexpr int MORSE_DELETE_TIME = 7 * MORSE_UNIT;

constexpr std::string_view MorseEncoded
    = "**ETIANMSURWDKGOHVF*L*PJBXCYZQ**54*3***2&*+****16=/"
      "***(*7***8*90************?_****\"**.****@***\'**-********;!*)*****,****:****************$***********************"
      "***********************************************************************************************";

MorseCode::MorseCode() { last_open_time = esp_timer_get_time(); }

/*
 * Morse Code timing basics:
 * Dit (.) = 1 unit
 * Dah (-) = 3 units
 * Intra-character space (within character): 1 unit
 * Inter-character space (between character): 3 (Farnsworth) units
 * Word space: 7 (Farnsworth) units
 * Examples:
 * ---_-___---_---_--- = -. --- = NO
 *
 *
 *
 */
// TODO also be a bit lenient with the timing (allow 0.5-1.5x MORSE_UNIT?)

std::pair<Morse, char> MorseCode::Update(bool signal, int64_t start_time) {
  Morse retmorse = Morse::Invalid;
  char retchar = '\0';
  if (!signal) {
    int64_t time_since_last_open = start_time - last_open_time;
    if (started && time_since_last_open > MORSE_DELETE_TIME) {
      morse_state.clear();
      retchar = '\b';
    } else if (time_since_last_open > 3 * MORSE_UNIT) {
      morse_state.emplace_back(Morse::Dash);
      retmorse = Morse::Dash;
      started = true;
    } else if (time_since_last_open > MORSE_UNIT) {
      morse_state.emplace_back(Morse::Dot);
      retmorse = Morse::Dot;
      started = true;
    }
    int64_t time_since_last_close = start_time - last_close_time;
    bool is_word_done = false, is_char_done = false;
    // don't end the word if no characters have been entered yet
    if (!end_word_handled && started && time_since_last_close > 7 * MORSE_FARNSWORTH_UNIT) {
      is_word_done = true;
      end_word_handled = true;
      // this will be done first, so we don't have to handle end of character later
      // TODO handle maximum character length
    } else if (!end_char_handled && !morse_state.empty() && time_since_last_close > 3 * MORSE_FARNSWORTH_UNIT) {
      is_char_done = true;
      end_char_handled = true;
    }
    // technically should check whether MORSE_UNIT has elapsed after a dot/dash

    if (is_char_done) {
      retchar = GetMorse();
      morse_state.clear();
    }

    if (is_word_done) {
      retchar = ' ';
    }

    last_open_time = start_time;
  } else {
    end_char_handled = false;
    end_word_handled = false;
    last_close_time = start_time;
  }
  return {retmorse, retchar};
}

char MorseCode::GetMorse() {
  int idx = 1;
  for (auto&& c : morse_state) {
    idx = 2 * idx + static_cast<int>(c);
  }
  return MorseEncoded[idx];
}
