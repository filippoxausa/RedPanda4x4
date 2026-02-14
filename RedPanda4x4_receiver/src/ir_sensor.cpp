#include "ir_sensor.h"

static const int  PIN_IR_BACK   = 23;
static const bool IR_ACTIVE_LOW = true;

void irInit() {
  pinMode(PIN_IR_BACK, IR_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
}

bool irBackObstacle() {
  int v = digitalRead(PIN_IR_BACK);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}
