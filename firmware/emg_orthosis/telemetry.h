// Snapshot the 1 kHz control task publishes for the serial/telemetry task.
#pragma once
#include <cstdint>

#include "orthosis_core.h"

struct Telemetry {
  uint32_t t_ms;
  orth::State state;
  float env[2], act[2];
  float cmd, ref, vel, pos, duty;
  int32_t raw_count;
  uint32_t enc_errors;
  bool kill_closed, mot_ok, zeroed;
  const char* msg;
  uint32_t msg_seq;
  const char* fault;
  orth::IdentResult ident;
  uint32_t loop_us_max, overruns;
  float k, friction, tau, kp, ki;   // loop model: 'j' can change these at run time
};
