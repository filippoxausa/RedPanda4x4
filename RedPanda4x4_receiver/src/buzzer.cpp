#include "buzzer.h"
#include "helpers.h"

// =======================================================
//                  Pin / LEDC config
// =======================================================
static const int PIN_BUZZER = 5;
static const int BUZZ_CH    = 7;        // canale LEDC libero
static const int BUZZ_RES   = 10;
static const int BUZZ_FREQ  = 2000;     // Hz (passivo)

static const uint32_t BEEP_ON_MS  = 120;
static const uint32_t BEEP_OFF_MS = 120;

// =======================================================
//                     State
// =======================================================
static bool     buzOn = false;
static uint32_t buzT0 = 0;

static void buzzerSet(bool on) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (on) ledcWriteTone(PIN_BUZZER, BUZZ_FREQ);
  else    ledcWriteTone(PIN_BUZZER, 0);
#else
  ledcWrite(BUZZ_CH, on ? (1 << (BUZZ_RES - 1)) : 0);
#endif
}

// =======================================================
//                   Public API
// =======================================================

void buzzerInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_BUZZER, BUZZ_FREQ, BUZZ_RES, BUZZ_CH);
#else
  ledcSetup(BUZZ_CH, BUZZ_FREQ, BUZZ_RES);
  ledcAttachPin(PIN_BUZZER, BUZZ_CH);
#endif

  buzOn = false;
  buzT0 = millis();

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIN_BUZZER, 0);
#else
  ledcWrite(BUZZ_CH, 0);
#endif
}

void buzzerUpdate(bool wantBeep) {
  uint32_t now = millis();

  if (!wantBeep) {
    if (buzOn) { buzOn = false; buzzerSet(false); }
    buzT0 = now;
    return;
  }

  if (!buzOn) {
    if (now - buzT0 >= BEEP_OFF_MS) {
      buzOn = true;
      buzT0 = now;
      buzzerSet(true);
    }
  } else {
    if (now - buzT0 >= BEEP_ON_MS) {
      buzOn = false;
      buzT0 = now;
      buzzerSet(false);
    }
  }
}
