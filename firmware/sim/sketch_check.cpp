// Compiles the Arduino sketch as plain C++ against arduino_stub/ and runs
// setup() once. Catches C++ mistakes on a laptop; it cannot catch a wrong
// ESP32 API, which is what the CI job with the real toolchain is for.
#include "emg_orthosis.ino"

int main() {
  setup();
  loop();
  return 0;
}
