/* =============================================================================
 * OpenEREMG v2 - EMG-driven orthosis motor control
 *
 * Target : ESP32-S3-WROOM-1 (3.3 V flash/PSRAM variants only: GPIO47 is the
 *          encoder, and on the 1.8 V "...V" modules it is a 1.8 V pin)
 * Core   : Arduino-ESP32 3.x (ESP-IDF 5). Tools -> USB CDC On Boot: Enabled,
 *          because the board's USB-C goes to the S3's native USB on IO19/IO20.
 * Board  : openeremg v2 - 4 ch INA333 front end, ISO7763 barrier, DPDT kill
 * Motor  : goBILDA 5203 Yellow Jacket 5.2:1 with 28 CPR quadrature encoder
 * Driver : BTS7960 (RPWM / LPWM / EN), on the far side of the isolator
 *
 * This file is only the hardware layer. Every decision - EMG processing,
 * intent, the velocity loop, the safety supervisor - lives in orthosis_core.h,
 * which is compiled unchanged into the host simulator and its tests
 * (firmware/sim). See firmware/README.md for the design and the test results.
 *
 * Execution model
 *   ctrl task, core 1, highest app priority, exactly 1 kHz (xTaskDelayUntil):
 *     read 4 ADC channels + encoder snapshot + kill/driver status
 *     -> Controller::step() -> BTS7960 -> feed the task watchdog
 *   encoder ISR: timestamps every quadrature edge (for the speed estimate)
 *   loop(), Arduino's task: serial commands and telemetry only. It posts
 *     requests to the ctrl task and reads a snapshot; it never touches the
 *     controller, so printing can never delay the control loop.
 *
 * SAFETY - read before powering a motor with this.
 *   - Nothing moves until you calibrate ('c'), zero ('z') and arm ('a').
 *   - The DPDT kill switch is the primary interlock: it pulls EN_ISO to MOT_GND
 *     in hardware. The firmware reads its second pole, but never relies on it.
 *   - If the ctrl task stops, the task watchdog resets the chip within 200 ms.
 *     A reset releases every pin, and the board's pull-downs (R67, R68 on the
 *     PWM lines, R69 on MOT_EN, R70 on EN_ISO) turn the driver off.
 *   - Bench-test with the motor unloaded and off the body first. Every time.
 * ============================================================================= */

#include <Arduino.h>
#include <atomic>

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#include "orthosis_core.h"
#include "telemetry.h"

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Needs Arduino-ESP32 core 3.x (Boards Manager -> esp32 by Espressif -> 3.x)"
#endif
static_assert(configTICK_RATE_HZ == 1000, "the 1 kHz control task assumes a 1 ms FreeRTOS tick");

// =============================================================================
// PIN MAP (matches the v2 schematic, tmayeremgboard.kicad_sch)
// =============================================================================
static const uint8_t PIN_EMG[4]     = {4, 5, 6, 7};  // IO4..IO7 = EMGC1..4, all on ADC1
static const uint8_t PIN_MOT_RPWM   = 15;  // -> ISO7763 INA -> OUTA -> RPWM_ISO
static const uint8_t PIN_MOT_LPWM   = 16;  // -> INB -> OUTB -> LPWM_ISO
static const uint8_t PIN_MOT_EN     = 17;  // -> INC -> OUTC -> kill switch -> EN_ISO
static const uint8_t PIN_KILL_SENSE = 18;  // <- kill switch pole B. LOW = closed (armed)
static const uint8_t PIN_MOT_FAULT  = 14;  // <- OUTF <- INF <- MOT_OK. HIGH = driver OK
static const uint8_t PIN_ENC_A      = 21;  // <- OUTD <- IND <- ENC_A_ISO
static const uint8_t PIN_ENC_B      = 47;  // <- OUTE <- INE <- ENC_B_ISO
static_assert(PIN_ENC_A < 32 && PIN_ENC_B >= 32, "readAB() reads A from GPIO_IN and B from GPIO_IN1");

static const uint32_t PWM_FREQ_HZ = 20000;  // above hearing, inside the BTS7960's range
static const uint8_t PWM_BITS = 10;
static const uint32_t PWM_MAX = (1u << PWM_BITS) - 1;

// =============================================================================
// CONFIGURATION - defaults live in orthosis_core.h (struct Config). Override
// here once you have measured them. 'j' prints the motor values to paste in.
// =============================================================================
static orth::Config makeConfig() {
  orth::Config c;
  // c.pulley_dia_mm = 47.0f;      // VERIFY: notebook uses 47 early, 65 late
  // c.gear_ratio = 5.2f;          // VERIFY with 'm'
  // c.encoder_sign = -1;          // if 'j' says the sign is reversed
  // c.dps_per_duty = ...;         // from 'j'
  // c.friction_duty = ...;        // from 'j'
  // c.plant_tau_s = ...;          // from 'j'
  // c.joint_max_deg = 30.0f;      // widen only after watching the full range unloaded
  return c;
}

static orth::Controller ctl(makeConfig());

// =============================================================================
// ENCODER - every edge is timestamped for the speed estimate
// =============================================================================
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static orth::EdgeTracker enc;

// Direct register reads: IRAM-safe (digitalRead() is not guaranteed to be) and
// both channels sampled back to back.
static inline uint8_t IRAM_ATTR readAB() {
  const uint32_t a = (REG_READ(GPIO_IN_REG) >> PIN_ENC_A) & 1u;
  const uint32_t b = (REG_READ(GPIO_IN1_REG) >> (PIN_ENC_B - 32)) & 1u;
  return static_cast<uint8_t>((a << 1) | b);
}

static void IRAM_ATTR encISR() {
  const uint32_t t = static_cast<uint32_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&encMux);
  enc.onChange(readAB(), t);
  portEXIT_CRITICAL_ISR(&encMux);
}

static orth::EdgeTracker encSnapshot() {
  portENTER_CRITICAL(&encMux);
  const orth::EdgeTracker e = enc;
  portEXIT_CRITICAL(&encMux);
  return e;
}

// =============================================================================
// MOTOR - BTS7960
//   duty > 0: PWM on RPWM, LPWM low.  duty < 0: the reverse.
//   enable with |duty| = 0: both low-side FETs on = the loop holds/brakes.
//   enable = false: EN low, bridge off, motor coasts. Used for every fault,
//   because releasing the limb is the right failure mode.
// =============================================================================
static void driveMotor(const orth::Outputs& o) {
  if (!o.enable) {
    digitalWrite(PIN_MOT_EN, LOW);  // EN first: the bridge is off before the PWMs change
    ledcWrite(PIN_MOT_RPWM, 0);
    ledcWrite(PIN_MOT_LPWM, 0);
    return;
  }
  const uint32_t mag = static_cast<uint32_t>(fabsf(o.duty) * PWM_MAX + 0.5f);
  if (o.duty >= 0.0f) {
    ledcWrite(PIN_MOT_LPWM, 0);
    ledcWrite(PIN_MOT_RPWM, mag);
  } else {
    ledcWrite(PIN_MOT_RPWM, 0);
    ledcWrite(PIN_MOT_LPWM, mag);
  }
  digitalWrite(PIN_MOT_EN, HIGH);
}

// =============================================================================
// SHARED STATE between the ctrl task and loop()
// =============================================================================
static std::atomic<uint8_t> pendingRequest{static_cast<uint8_t>(orth::Request::None)};

// struct Telemetry lives in telemetry.h: the Arduino IDE generates prototypes
// above the sketch body, so any type a function returns must be declared in a header.
static portMUX_TYPE telMux = portMUX_INITIALIZER_UNLOCKED;
static Telemetry tel;

static Telemetry telemetry() {
  portENTER_CRITICAL(&telMux);
  const Telemetry t = tel;
  portEXIT_CRITICAL(&telMux);
  return t;
}

// =============================================================================
// THE 1 kHz CONTROL TASK
// =============================================================================
static void controlTask(void*) {
  esp_task_wdt_add(nullptr);
  TickType_t wake = xTaskGetTickCount();
  uint32_t loop_us_max = 0, overruns = 0;
  for (;;) {
    if (xTaskDelayUntil(&wake, 1) == pdFALSE) ++overruns;   // the previous tick ran long
    const uint32_t t0 = static_cast<uint32_t>(esp_timer_get_time());

    orth::Inputs in;
    for (int i = 0; i < 4; ++i) in.adc[i] = analogRead(PIN_EMG[i]);
    in.enc = encSnapshot();
    in.now_us = static_cast<uint32_t>(esp_timer_get_time());
    in.kill_closed = digitalRead(PIN_KILL_SENSE) == LOW;
    in.mot_ok = digitalRead(PIN_MOT_FAULT) == HIGH;
    const auto req = static_cast<orth::Request>(
        pendingRequest.exchange(static_cast<uint8_t>(orth::Request::None)));

    const orth::Outputs out = ctl.step(in, req);
    driveMotor(out);

    const uint32_t dt = static_cast<uint32_t>(esp_timer_get_time()) - t0;
    if (dt > loop_us_max) loop_us_max = dt;

    Telemetry t;
    t.t_ms = millis();
    t.state = ctl.state();
    for (int i = 0; i < 2; ++i) { t.env[i] = ctl.channel(i).envelope(); t.act[i] = ctl.channel(i).activation; }
    t.cmd = ctl.speedCommand();
    t.ref = ctl.speedRef();
    t.vel = ctl.jointDps();
    t.pos = ctl.jointDeg();
    t.duty = out.enable ? out.duty : 0.0f;
    t.raw_count = in.enc.count;
    t.enc_errors = in.enc.errors;
    t.kill_closed = in.kill_closed;
    t.mot_ok = in.mot_ok;
    t.zeroed = ctl.zeroed();
    t.msg = ctl.lastMessage();
    t.msg_seq = ctl.messageSeq();
    t.fault = ctl.faultReason();
    t.ident = ctl.ident();
    const orth::Config& cfg = ctl.config();
    t.k = cfg.dps_per_duty;
    t.friction = cfg.friction_duty;
    t.tau = cfg.plant_tau_s;
    t.kp = cfg.kp();
    t.ki = cfg.ki();
    t.loop_us_max = loop_us_max;
    t.overruns = overruns;
    portENTER_CRITICAL(&telMux);
    tel = t;
    portEXIT_CRITICAL(&telMux);

    esp_task_wdt_reset();
  }
}

// =============================================================================
// SERIAL INTERFACE (loop task)
// =============================================================================
static bool streaming = false;
static int32_t ratioMark = 0;

static void printHelp() {
  Serial.println(F(
      "\ncommands:\n"
      "  c  calibrate: relax 4 s, flexor max 4 s, extensor max 4 s\n"
      "  z  zero the joint here (do it at the lower limit)\n"
      "  a  arm - the motor goes live. needs: calibrated, zeroed, kill switch closed, MOT_OK\n"
      "  d  disarm / clear a fault\n"
      "  j  identify the motor (limb OUT, joint mid-range): open-loop steps, prints K, friction,\n"
      "     tau and whether the encoder sign is right, and loads them into the loop\n"
      "  m  gear-ratio check: mark here, hand-turn the gearbox output N revs, then '?'\n"
      "  s  toggle 50 Hz CSV telemetry\n"
      "  ?  status      h  this help"));
}

static void printStatus() {
  const Telemetry t = telemetry();
  const orth::Config& c = ctl.config();   // transmission constants only: never changed at run time
  Serial.printf("\nstate=%s  kill=%s  driver=%s  zeroed=%s\n", orth::stateName(t.state),
                t.kill_closed ? "closed" : "OPEN", t.mot_ok ? "ok" : "NOT OK", t.zeroed ? "yes" : "no");
  Serial.printf("joint %+.2f deg  %+.1f deg/s  (cmd %+.1f, ref %+.1f)  duty %+.3f\n", t.pos, t.vel, t.cmd,
                t.ref, t.duty);
  Serial.printf("EMG  flexor env %.1f act %.2f | extensor env %.1f act %.2f\n", t.env[0], t.act[0], t.env[1],
                t.act[1]);
  Serial.printf("loop  worst %lu us of 1000, %lu overruns | encoder errors %lu\n",
                static_cast<unsigned long>(t.loop_us_max), static_cast<unsigned long>(t.overruns),
                static_cast<unsigned long>(t.enc_errors));
  const int32_t since = t.raw_count - ratioMark;
  Serial.printf("counts since 'm': %ld = %.3f motor revs = %.4f gearbox-output revs at %.2f:1\n",
                static_cast<long>(since), since / c.counts_per_motor_rev,
                since / (c.counts_per_motor_rev * c.gear_ratio), c.gear_ratio);
  Serial.printf("loop model  K %.1f deg/s/duty  friction %.3f  tau %.1f ms  -> Kp %.5f  Ki %.4f\n",
                t.k, t.friction, 1e3f * t.tau, t.kp, t.ki);
  if (t.state == orth::State::Fault) Serial.printf("FAULT: %s  ('d' to clear)\n", t.fault);
}

static void printIdent(const orth::IdentResult& r) {
  if (!r.sign_ok) return;
  Serial.printf("   K = %.1f deg/s per duty, friction = %.3f duty, tau = %.1f ms\n", r.dps_per_duty,
                r.friction_duty, 1e3f * r.tau_s);
  Serial.printf("   paste into makeConfig():  c.dps_per_duty = %.1ff; c.friction_duty = %.3ff; "
                "c.plant_tau_s = %.4ff;\n", r.dps_per_duty, r.friction_duty, r.tau_s);
}

static void post(orth::Request r) { pendingRequest.store(static_cast<uint8_t>(r)); }

static void handleSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'c': post(orth::Request::Calibrate); break;
      case 'z': post(orth::Request::Zero); break;
      case 'a': post(orth::Request::Arm); break;
      case 'd': post(orth::Request::Disarm); break;
      case 'j': post(orth::Request::Identify); break;
      case 'm':
        ratioMark = telemetry().raw_count;
        Serial.println(F("\n>> marked. Hand-turn the gearbox OUTPUT exactly N revs (5 or 10), then '?'.\n"
                         "   gear ratio = counts / 28 / N"));
        break;
      case 's': streaming = !streaming;
        if (streaming) Serial.println(F("t_ms,state,env_flex,env_ext,act_flex,act_ext,cmd,ref,vel,pos,duty"));
        break;
      case '?': printStatus(); break;
      case 'h': printHelp(); break;
      default: break;
    }
  }
}

// =============================================================================
// SETUP / LOOP
// =============================================================================
void setup() {
  // Motor pins first and safe, before anything else can take time.
  pinMode(PIN_MOT_EN, OUTPUT);
  digitalWrite(PIN_MOT_EN, LOW);
  ledcAttach(PIN_MOT_RPWM, PWM_FREQ_HZ, PWM_BITS);
  ledcAttach(PIN_MOT_LPWM, PWM_FREQ_HZ, PWM_BITS);
  ledcWrite(PIN_MOT_RPWM, 0);
  ledcWrite(PIN_MOT_LPWM, 0);

  pinMode(PIN_KILL_SENSE, INPUT_PULLUP);  // pole B shorts it to GND when closed
  pinMode(PIN_MOT_FAULT, INPUT);          // driven by the isolator; R66 defaults it LOW (= not OK)
  pinMode(PIN_ENC_A, INPUT);              // driven by the isolator
  pinMode(PIN_ENC_B, INPUT);

  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);         // ~0-3.1 V: VREF (1.65 V) sits mid-scale

  enc.reset(readAB());
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);

  uint16_t adc[4];
  for (int i = 0; i < 4; ++i) adc[i] = analogRead(PIN_EMG[i]);
  ctl.prime(adc);

  // Task watchdog: 200 ms, panic (= reset) on timeout, watching only our task.
  esp_task_wdt_config_t wdt;
  wdt.timeout_ms = 200;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);

  const orth::Config& c = ctl.config();
  Serial.println(F("\nOpenEREMG v2 - EMG orthosis control"));
  Serial.printf("transmission %.1f:1, %.1f mm screw, %.0f mm pulley = %.5f deg/count\n", c.gear_ratio,
                c.screw_pitch_mm, c.pulley_dia_mm, c.degPerCount());
  Serial.printf("limits %.0f..%.0f deg, %.0f deg/s max | envelope %.1f Hz (%.0f ms mean delay)\n",
                c.joint_min_deg, c.joint_max_deg, c.joint_max_dps, c.envelope_hz,
                2000.0f / (2.0f * orth::kPi * c.envelope_hz));
  printHelp();

  xTaskCreatePinnedToCore(controlTask, "ctrl", 6144, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
}

void loop() {
  static uint32_t lastSeq = 0, nextTelMs = 0;
  static bool identPrinted = true;
  handleSerial();

  const Telemetry t = telemetry();
  if (t.msg_seq != lastSeq) {
    lastSeq = t.msg_seq;
    Serial.printf("\n>> %s\n", t.msg);
    if (t.state == orth::State::Identify) identPrinted = false;
  }
  if (!identPrinted && t.ident.done) {
    printIdent(t.ident);
    identPrinted = true;
  }
  if (streaming && static_cast<int32_t>(millis() - nextTelMs) >= 0) {
    nextTelMs = millis() + 20;
    Serial.printf("%lu,%s,%.1f,%.1f,%.3f,%.3f,%+.2f,%+.2f,%+.2f,%+.2f,%+.3f\n",
                  static_cast<unsigned long>(t.t_ms), orth::stateName(t.state), t.env[0], t.env[1], t.act[0],
                  t.act[1], t.cmd, t.ref, t.vel, t.pos, t.duty);
  }
  delay(2);
}
