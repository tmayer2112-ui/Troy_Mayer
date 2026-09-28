/* =============================================================================
 * OpenEREMG v2 - EMG-driven orthosis motor control
 *
 * Target : ESP32-S3-WROOM-1  (Arduino-ESP32 core 2.x or 3.x)
 * Board  : openeremg v2 - 4ch INA333 front end, ISO7761FDW barrier, DPDT kill
 * Motor  : goBILDA 5203 Yellow Jacket planetary gearmotor w/ quadrature encoder
 * Driver : BTS7960 half-bridge pair (see MOTOR INTERFACE note below)
 *
 * Control scheme: two-site proportional velocity control.
 *   A flexor channel and an extensor channel each drive an envelope detector.
 *   Their normalised difference commands motor velocity. Simultaneous
 *   contraction of both ("co-contraction") commands a stop - the standard
 *   myoelectric idiom, and the user's fastest voluntary brake.
 *
 * SAFETY - read before powering a motor with this.
 *   - Nothing moves until you explicitly arm it over serial. No auto-arm.
 *   - The DPDT kill switch is a HARDWARE interlock. This firmware also reads
 *     its second pole, but the firmware is not what stops the motor - the
 *     switch shorting EN_ISO to MOT_GND is. Never remove that.
 *   - Soft position limits, stall detection and a command watchdog are all
 *     backstops, not primary protection.
 *   - Bench-test with the motor unloaded and off the body first. Every time.
 * ============================================================================= */

#include <Arduino.h>
#include <math.h>

// =============================================================================
// SECTION 1 - THINGS YOU MUST SET FOR YOUR HARDWARE
// =============================================================================

/* --- Gearbox ---------------------------------------------------------------
 * The 5203 encoder produces 7 pulses per channel per MOTOR revolution.
 * In 4x quadrature decoding that is 28 counts per motor revolution.
 * Counts per OUTPUT revolution = 28 * gear ratio.
 *
 * Read the exact ratio off your motor's label / goBILDA part number. It is
 * NOT a round number. Common ones:
 *     3.7:1  -> 103.8      26.9:1 ->  751.8      99.5:1 -> 2786.2
 *     5.2:1  -> 145.1      43.7:1 -> 1223.6     139:1   -> 3895.9
 *    13.7:1  -> 384.5      50.9:1 -> 1425.1     188:1   -> 5264.0
 *    19.2:1  -> 537.7      71.2:1 -> 1993.6
 *
 * Both of your reference docs used CPR values (12, and 728 for a 26:1) that
 * do not match this motor. 28 * ratio is the one to trust - goBILDA's own
 * published 537.7 counts/rev for the 19.2:1 confirms it.
 */
static const float GEAR_RATIO        = 19.2f;                        // <<< SET ME
static const float COUNTS_PER_MOTOR_REV = 28.0f;                     // 7 PPR x4
static const float COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
static const float DEG_PER_COUNT     = 360.0f / COUNTS_PER_OUTPUT_REV;

/* --- Joint range of motion ------------------------------------------------
 * Measured from the zero position you set with the 'z' command.
 * Positive = whatever direction POSITIVE encoder counts correspond to.
 * Start these CONSERVATIVE and open them up once you've watched it move.
 */
static const float JOINT_MIN_DEG     = -5.0f;                        // <<< SET ME
static const float JOINT_MAX_DEG     = 60.0f;                        // <<< SET ME

/* --- Which EMG channel is which ------------------------------------------- */
static const uint8_t CH_FLEXOR       = 0;   // EMGC1 -> drives positive motion
static const uint8_t CH_EXTENSOR     = 1;   // EMGC2 -> drives negative motion
// channels 2 and 3 are acquired and logged but not used in the control law

/* --- Motor limits ---------------------------------------------------------- */
static const float DUTY_MAX          = 0.55f;  // fraction of full scale. START LOW.
static const float DUTY_MIN          = 0.12f;  // below this the gearbox won't break stiction
static const float DUTY_SLEW_PER_S   = 2.0f;   // max duty change per second

// =============================================================================
// SECTION 2 - PIN MAP  (matches the v2 schematic)
// =============================================================================

// EMG channel outputs from the four analogue front ends (ADC1)
static const uint8_t PIN_EMG[4]      = { 4, 5, 6, 7 };   // IO4..IO7 = EMGC1..EMGC4

// Isolated motor interface - board side of the ISO7761FDW
static const uint8_t PIN_MOT_RPWM    = 15;   // IO15 -> INA -> OUTA -> RPWM_ISO
static const uint8_t PIN_MOT_LPWM    = 16;   // IO16 -> INB -> OUTB -> LPWM_ISO
static const uint8_t PIN_MOT_EN      = 17;   // IO17 -> INC -> OUTC -> kill switch -> EN_ISO
static const uint8_t PIN_KILL_SENSE  = 18;   // IO18 <- kill switch pole B. LOW = armed.
static const uint8_t PIN_MOT_FAULT   = 14;   // IO14 <- OUTF <- INF <- MOT_OK (motor side)

// Encoder - NOT PRESENT ON THE v2 BOARD YET. See the note in the reply.
#define USE_ENCODER 1
static const uint8_t PIN_ENC_A       = 21;   // IO21
static const uint8_t PIN_ENC_B       = 47;   // IO47

// =============================================================================
// SECTION 3 - SIGNAL PROCESSING AND TIMING CONSTANTS
// =============================================================================

static const uint32_t FS_HZ          = 1000;    // EMG sample rate per channel
static const uint32_t CTRL_HZ        = 100;     // control law rate
static const uint32_t TELEM_HZ       = 20;      // serial telemetry rate

static const float DC_BLOCK_FC_HZ    = 0.5f;    // tracks VREF drift out of the signal
static const float ENVELOPE_FC_HZ    = 3.0f;    // two cascaded poles -> ~55 ms effective

/* Activation mapping.
 * Full motor command at MVC_FRACTION of the calibrated maximum, so the user
 * does not have to contract maximally to get full speed. */
static const float MVC_FRACTION      = 0.60f;
static const float ONSET_K_SIGMA     = 6.0f;    // onset threshold = rest_mean + k*rest_sd
static const float ONSET_MIN_MVC     = 0.08f;   // ...but never below 8% of MVC
static const float RELEASE_RATIO     = 0.70f;   // hysteresis: release at 70% of onset
static const float DEADZONE          = 0.08f;   // |flex - ext| below this = no command
static const float COCONTRACT_LEVEL  = 0.35f;   // both channels above this = commanded stop

/* Safety timers */
static const uint32_t STALL_TIME_MS      = 400;   // commanded hard, not moving -> fault
static const float    STALL_DUTY         = 0.30f;
static const float    STALL_SPEED_DPS    = 3.0f;
static const uint32_t MAX_CONTINUOUS_MS  = 15000; // longest single uninterrupted drive
static const uint32_t CMD_WATCHDOG_MS    = 250;   // control loop must tick this often

/* Calibration durations */
static const uint32_t CAL_REST_MS    = 4000;
static const uint32_t CAL_MVC_MS     = 4000;

// PWM
static const uint32_t PWM_FREQ_HZ    = 20000;   // above audible, within BTS7960 spec
static const uint8_t  PWM_BITS       = 10;      // 0..1023
static const uint16_t PWM_MAX        = (1u << PWM_BITS) - 1;

// =============================================================================
// SECTION 4 - STATE
// =============================================================================

enum State : uint8_t {
  ST_BOOT,
  ST_CAL_REST,
  ST_CAL_FLEX,
  ST_CAL_EXT,
  ST_IDLE,        // calibrated, disarmed
  ST_ARMED,       // motor live
  ST_FAULT
};
static State state = ST_BOOT;
static const char* stateName(State s) {
  switch (s) {
    case ST_BOOT:     return "BOOT";
    case ST_CAL_REST: return "CAL_REST";
    case ST_CAL_FLEX: return "CAL_FLEX";
    case ST_CAL_EXT:  return "CAL_EXT";
    case ST_IDLE:     return "IDLE";
    case ST_ARMED:    return "ARMED";
    case ST_FAULT:    return "FAULT";
  }
  return "?";
}
static const char* faultReason = "";

struct Channel {
  float dc      = 2048.0f;  // tracked DC level in ADC counts
  float env1    = 0.0f;     // first envelope pole
  float env     = 0.0f;     // second pole - this is the envelope
  float restMean = 0.0f;
  float restSd   = 1.0f;
  float mvc      = 1.0f;
  float onset    = 0.0f;
  float release  = 0.0f;
  bool  active   = false;   // hysteresis latch
  float activation = 0.0f;  // 0..1
};
static Channel ch[4];

static float aDc  = 0.0f;   // DC blocker coefficient
static float aEnv = 0.0f;   // envelope coefficient

static float dutyCmd    = 0.0f;   // -1..1, post slew limit
static float dutyTarget = 0.0f;   // -1..1, from the control law

static uint32_t driveStartMs   = 0;
static uint32_t stallStartMs   = 0;
static uint32_t lastCtrlMs     = 0;

#if USE_ENCODER
static volatile int32_t encCount = 0;
static volatile uint8_t encState = 0;
static int32_t  encZero    = 0;
static float    jointDeg   = 0.0f;
static float    jointDps   = 0.0f;
static int32_t  lastEncSnapshot = 0;
#endif

// =============================================================================
// SECTION 5 - ENCODER
// =============================================================================

#if USE_ENCODER
/* 4x quadrature state table. Index = (previous 2 bits << 2) | current 2 bits.
 * Zero entries are "no change" or an illegal double transition. */
static const int8_t QTAB[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

static void IRAM_ATTR encISR() {
  uint8_t s = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
  encCount += QTAB[(encState << 2) | s];
  encState = s;
}

static int32_t encRead() {
  noInterrupts();
  int32_t c = encCount;
  interrupts();
  return c;
}
#endif

// =============================================================================
// SECTION 6 - MOTOR INTERFACE
// =============================================================================
/*
 * MOTOR INTERFACE NOTE
 *
 * This assumes the three isolated forward channels carry RPWM, LPWM and EN,
 * which maps 1:1 onto a BTS7960:
 *     PWM_ISO (OUTA) -> RPWM
 *     DIR_ISO (OUTB) -> LPWM
 *     EN_ISO  (OUTC, through the kill switch) -> R_EN and L_EN tied together
 *
 * One of RPWM/LPWM carries the PWM; the other is held low. That is the
 * standard BTS7960 drive and it needs no logic on the motor side at all.
 *
 * Your schematic currently names those nets MOT_PWM / MOT_DIR / MOT_EN. With a
 * DIR-style driver (L298N: IN1/IN2/ENA) you would need an inverter on the motor
 * side to derive IN2 from IN1, because a single DIR line cannot drive both.
 * Renaming the nets to RPWM/LPWM/EN costs nothing and removes that part.
 *
 * Do NOT drive this motor with an L298N. The 5203 stalls near 9 A; the L298N
 * is rated 2 A continuous. Your own reference doc flags this twice.
 */

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin)      ledcAttach((pin), PWM_FREQ_HZ, PWM_BITS)
  #define PWM_WRITE(pin, v)   ledcWrite((pin), (v))
#else
  static const uint8_t LEDC_CH_R = 0;
  static const uint8_t LEDC_CH_L = 1;
  static uint8_t ledcChanFor(uint8_t pin) {
    return (pin == PIN_MOT_RPWM) ? LEDC_CH_R : LEDC_CH_L;
  }
  #define PWM_SETUP(pin)      do { ledcSetup(ledcChanFor(pin), PWM_FREQ_HZ, PWM_BITS); \
                                   ledcAttachPin((pin), ledcChanFor(pin)); } while (0)
  #define PWM_WRITE(pin, v)   ledcWrite(ledcChanFor(pin), (v))
#endif

static void motorCoast() {
  PWM_WRITE(PIN_MOT_RPWM, 0);
  PWM_WRITE(PIN_MOT_LPWM, 0);
  digitalWrite(PIN_MOT_EN, LOW);
  dutyCmd = 0.0f;
}

/* duty in -1..1. Positive drives the joint toward JOINT_MAX. */
static void motorDrive(float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  uint16_t mag = (uint16_t)(fabsf(duty) * PWM_MAX);
  if (duty >= 0.0f) {
    PWM_WRITE(PIN_MOT_LPWM, 0);
    PWM_WRITE(PIN_MOT_RPWM, mag);
  } else {
    PWM_WRITE(PIN_MOT_RPWM, 0);
    PWM_WRITE(PIN_MOT_LPWM, mag);
  }
  digitalWrite(PIN_MOT_EN, mag > 0 ? HIGH : LOW);
}

static bool killSwitchArmed() {
  // Pole B: closed -> tied to GND -> LOW -> armed. Open -> pull-up -> HIGH -> killed.
  return digitalRead(PIN_KILL_SENSE) == LOW;
}

static void enterFault(const char* why) {
  motorCoast();
  faultReason = why;
  state = ST_FAULT;
  Serial.printf("\n!! FAULT: %s\n", why);
}

// =============================================================================
// SECTION 7 - EMG ACQUISITION
// =============================================================================

static void emgSample() {
  for (uint8_t i = 0; i < 4; i++) {
    float x = (float)analogRead(PIN_EMG[i]);

    // Track and remove DC. The analogue front end high-passes at ~23 Hz, so
    // this is really just following VREF and any slow electrode drift.
    ch[i].dc += aDc * (x - ch[i].dc);
    float ac = x - ch[i].dc;

    // Rectify, then two cascaded one-pole low passes = the envelope.
    float r = fabsf(ac);
    ch[i].env1 += aEnv * (r - ch[i].env1);
    ch[i].env  += aEnv * (ch[i].env1 - ch[i].env);
  }
}

/* Map envelope -> 0..1 activation, with onset/release hysteresis. */
static void emgActivation() {
  for (uint8_t i = 0; i < 4; i++) {
    Channel& c = ch[i];
    if (c.active) {
      if (c.env < c.release) c.active = false;
    } else {
      if (c.env > c.onset)   c.active = true;
    }
    if (!c.active) {
      c.activation = 0.0f;
    } else {
      float span = (c.mvc * MVC_FRACTION) - c.onset;
      if (span < 1.0f) span = 1.0f;
      c.activation = constrain((c.env - c.onset) / span, 0.0f, 1.0f);
    }
  }
}

// =============================================================================
// SECTION 8 - CALIBRATION
// =============================================================================

struct Accum { double sum = 0; double sumSq = 0; uint32_t n = 0; float peak = 0; };
static Accum acc[4];
static uint32_t calStartMs = 0;

static void calBegin(State s, const char* prompt) {
  for (uint8_t i = 0; i < 4; i++) acc[i] = Accum();
  calStartMs = millis();
  state = s;
  Serial.printf("\n>> %s\n", prompt);
}

static void calAccumulate() {
  for (uint8_t i = 0; i < 4; i++) {
    float e = ch[i].env;
    acc[i].sum   += e;
    acc[i].sumSq += (double)e * e;
    acc[i].n++;
    if (e > acc[i].peak) acc[i].peak = e;
  }
}

static void calFinishRest() {
  for (uint8_t i = 0; i < 4; i++) {
    float mean = (float)(acc[i].sum / acc[i].n);
    float var  = (float)(acc[i].sumSq / acc[i].n) - mean * mean;
    ch[i].restMean = mean;
    ch[i].restSd   = (var > 0.0f) ? sqrtf(var) : 1.0f;
    Serial.printf("   ch%u rest: mean %.1f  sd %.1f\n", i, ch[i].restMean, ch[i].restSd);
  }
}

/* Take the peak envelope as MVC. Peak is noisy but the envelope is already
 * heavily smoothed, so it is a reasonable stand-in for a 90th percentile. */
static void calFinishMvc(uint8_t idx) {
  ch[idx].mvc = acc[idx].peak;
  Serial.printf("   ch%u MVC peak: %.1f\n", idx, ch[idx].mvc);
}

static void calComputeThresholds() {
  for (uint8_t i = 0; i < 4; i++) {
    Channel& c = ch[i];
    float t = c.restMean + ONSET_K_SIGMA * c.restSd;
    float floorT = c.restMean + ONSET_MIN_MVC * (c.mvc - c.restMean);
    c.onset   = max(t, floorT);
    c.release = c.restMean + RELEASE_RATIO * (c.onset - c.restMean);
    c.active  = false;
  }
  Serial.printf("   thresholds: flex on %.1f off %.1f | ext on %.1f off %.1f\n",
                ch[CH_FLEXOR].onset,   ch[CH_FLEXOR].release,
                ch[CH_EXTENSOR].onset, ch[CH_EXTENSOR].release);
}

// =============================================================================
// SECTION 9 - CONTROL LAW
// =============================================================================

static void controlStep(float dt) {
  lastCtrlMs = millis();

  // --- Hard gates, checked every tick, highest priority first ---------------
  if (!killSwitchArmed()) {
    motorCoast();
    if (state == ST_ARMED) {
      state = ST_IDLE;
      Serial.println("\n>> Kill switch opened - disarmed.");
    }
    return;
  }
  if (state != ST_ARMED) { motorCoast(); return; }

#if USE_ENCODER
  int32_t c = encRead();
  jointDeg = (float)(c - encZero) * DEG_PER_COUNT;
  jointDps = (float)(c - lastEncSnapshot) * DEG_PER_COUNT / dt;
  lastEncSnapshot = c;
#endif

  // --- Control law ----------------------------------------------------------
  float aFlex = ch[CH_FLEXOR].activation;
  float aExt  = ch[CH_EXTENSOR].activation;

  if (aFlex > COCONTRACT_LEVEL && aExt > COCONTRACT_LEVEL) {
    dutyTarget = 0.0f;                       // co-contraction = voluntary stop
  } else {
    float net = aFlex - aExt;
    if (fabsf(net) < DEADZONE) {
      dutyTarget = 0.0f;
    } else {
      float mag = (fabsf(net) - DEADZONE) / (1.0f - DEADZONE);
      dutyTarget = copysignf(DUTY_MIN + mag * (DUTY_MAX - DUTY_MIN), net);
    }
  }

  // --- Soft position limits -------------------------------------------------
#if USE_ENCODER
  if (jointDeg >= JOINT_MAX_DEG && dutyTarget > 0.0f) dutyTarget = 0.0f;
  if (jointDeg <= JOINT_MIN_DEG && dutyTarget < 0.0f) dutyTarget = 0.0f;
#endif

  // --- Slew limit -----------------------------------------------------------
  float maxStep = DUTY_SLEW_PER_S * dt;
  float err = dutyTarget - dutyCmd;
  dutyCmd += constrain(err, -maxStep, maxStep);
  if (fabsf(dutyCmd) < 1e-3f) dutyCmd = 0.0f;

  // --- Duration limit -------------------------------------------------------
  if (dutyCmd != 0.0f) {
    if (driveStartMs == 0) driveStartMs = millis();
    else if (millis() - driveStartMs > MAX_CONTINUOUS_MS) {
      enterFault("continuous drive limit exceeded");
      return;
    }
  } else {
    driveStartMs = 0;
  }

  // --- Stall detection ------------------------------------------------------
#if USE_ENCODER
  if (fabsf(dutyCmd) > STALL_DUTY && fabsf(jointDps) < STALL_SPEED_DPS) {
    if (stallStartMs == 0) stallStartMs = millis();
    else if (millis() - stallStartMs > STALL_TIME_MS) {
      enterFault("stall detected");
      return;
    }
  } else {
    stallStartMs = 0;
  }
#endif

  motorDrive(dutyCmd);
}

// =============================================================================
// SECTION 10 - SERIAL INTERFACE
// =============================================================================

static void printHelp() {
  Serial.println(F(
    "\ncommands:\n"
    "  c  recalibrate (rest, then flexor MVC, then extensor MVC)\n"
    "  a  arm   - motor goes live. kill switch must be closed.\n"
    "  d  disarm\n"
    "  z  zero the encoder at the current position\n"
    "  ?  status\n"));
}

static void printStatus() {
  Serial.printf("\nstate=%s  kill=%s  duty=%+.2f",
                stateName(state), killSwitchArmed() ? "ARMED" : "OPEN", dutyCmd);
#if USE_ENCODER
  Serial.printf("  joint=%+.1f deg (%.0f dps)", jointDeg, jointDps);
#endif
  Serial.printf("  motOK=%d\n", digitalRead(PIN_MOT_FAULT));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("  ch%u env %7.1f  on %7.1f  mvc %7.1f  act %.2f%s\n",
                  i, ch[i].env, ch[i].onset, ch[i].mvc, ch[i].activation,
                  ch[i].active ? "  *" : "");
  }
  if (state == ST_FAULT) Serial.printf("  fault: %s\n", faultReason);
}

static void handleSerial() {
  if (!Serial.available()) return;
  int c = Serial.read();
  switch (c) {
    case 'c':
      motorCoast();
      state = ST_IDLE;
      calBegin(ST_CAL_REST, "CALIBRATION - relax completely for 4 s");
      break;
    case 'a':
      if (state == ST_IDLE) {
        if (!killSwitchArmed()) {
          Serial.println("\n!! cannot arm: kill switch is open");
        } else if (ch[CH_FLEXOR].active || ch[CH_EXTENSOR].active) {
          Serial.println("\n!! cannot arm: muscle active - relax first");
        } else {
          dutyCmd = dutyTarget = 0.0f;
          driveStartMs = stallStartMs = 0;
          state = ST_ARMED;
          Serial.println("\n>> ARMED");
        }
      } else {
        Serial.printf("\n!! cannot arm from %s\n", stateName(state));
      }
      break;
    case 'd':
      motorCoast();
      if (state == ST_ARMED || state == ST_FAULT) state = ST_IDLE;
      Serial.println("\n>> disarmed");
      break;
#if USE_ENCODER
    case 'z':
      encZero = encRead();
      lastEncSnapshot = encZero;
      Serial.println("\n>> encoder zeroed");
      break;
#endif
    case '?': printStatus(); break;
    case 'h': printHelp();   break;
    default: break;
  }
}

// =============================================================================
// SECTION 11 - SETUP / LOOP
// =============================================================================

void setup() {
  // Motor pins first and safe, before anything else can take time.
  pinMode(PIN_MOT_EN, OUTPUT);
  digitalWrite(PIN_MOT_EN, LOW);
  PWM_SETUP(PIN_MOT_RPWM);
  PWM_SETUP(PIN_MOT_LPWM);
  PWM_WRITE(PIN_MOT_RPWM, 0);
  PWM_WRITE(PIN_MOT_LPWM, 0);

  pinMode(PIN_KILL_SENSE, INPUT_PULLUP);
  pinMode(PIN_MOT_FAULT,  INPUT);

  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);          // full 0..3.3 V span

#if USE_ENCODER
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encState = (uint8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encISR, CHANGE);
#endif

  // Filter coefficients for a one-pole IIR at the given corner.
  aDc  = 1.0f - expf(-2.0f * (float)M_PI * DC_BLOCK_FC_HZ  / (float)FS_HZ);
  aEnv = 1.0f - expf(-2.0f * (float)M_PI * ENVELOPE_FC_HZ / (float)FS_HZ);

  // Prime the DC trackers so the envelope does not start with a huge step.
  for (uint8_t i = 0; i < 4; i++) ch[i].dc = (float)analogRead(PIN_EMG[i]);
  for (int n = 0; n < FS_HZ / 2; n++) { emgSample(); delayMicroseconds(1000000 / FS_HZ); }

  Serial.println(F("\nOpenEREMG v2 - EMG orthosis control"));
  Serial.printf("gear ratio %.1f:1  -> %.1f counts/output rev  (%.4f deg/count)\n",
                GEAR_RATIO, COUNTS_PER_OUTPUT_REV, DEG_PER_COUNT);
  printHelp();
  calBegin(ST_CAL_REST, "CALIBRATION - relax completely for 4 s");
}

void loop() {
  static uint32_t nextSampleUs = 0;
  static uint32_t nextCtrlUs   = 0;
  static uint32_t nextTelemUs  = 0;
  uint32_t now = micros();

  // --- 1 kHz: acquire and filter -------------------------------------------
  if ((int32_t)(now - nextSampleUs) >= 0) {
    nextSampleUs = now + 1000000UL / FS_HZ;
    emgSample();
    emgActivation();

    // Calibration states consume samples rather than running the control law.
    switch (state) {
      case ST_CAL_REST:
        calAccumulate();
        if (millis() - calStartMs > CAL_REST_MS) {
          calFinishRest();
          calBegin(ST_CAL_FLEX, "contract the FLEXOR as hard as you can for 4 s");
        }
        break;
      case ST_CAL_FLEX:
        calAccumulate();
        if (millis() - calStartMs > CAL_MVC_MS) {
          calFinishMvc(CH_FLEXOR);
          calBegin(ST_CAL_EXT, "contract the EXTENSOR as hard as you can for 4 s");
        }
        break;
      case ST_CAL_EXT:
        calAccumulate();
        if (millis() - calStartMs > CAL_MVC_MS) {
          calFinishMvc(CH_EXTENSOR);
          calComputeThresholds();
          state = ST_IDLE;
          Serial.println("\n>> calibrated. type 'a' to arm.");
        }
        break;
      default: break;
    }
  }

  // --- 100 Hz: control ------------------------------------------------------
  if ((int32_t)(now - nextCtrlUs) >= 0) {
    nextCtrlUs = now + 1000000UL / CTRL_HZ;
    controlStep(1.0f / (float)CTRL_HZ);
  }

  // --- Command watchdog -----------------------------------------------------
  if (state == ST_ARMED && millis() - lastCtrlMs > CMD_WATCHDOG_MS) {
    enterFault("control loop watchdog");
  }

  // --- 20 Hz: telemetry -----------------------------------------------------
  if ((int32_t)(now - nextTelemUs) >= 0) {
    nextTelemUs = now + 1000000UL / TELEM_HZ;
    if (state == ST_ARMED || state == ST_IDLE) {
      Serial.printf("%lu,%s,%.1f,%.1f,%.2f,%.2f,%+.3f",
                    (unsigned long)millis(), stateName(state),
                    ch[CH_FLEXOR].env, ch[CH_EXTENSOR].env,
                    ch[CH_FLEXOR].activation, ch[CH_EXTENSOR].activation,
                    dutyCmd);
#if USE_ENCODER
      Serial.printf(",%+.2f,%+.1f", jointDeg, jointDps);
#endif
      Serial.println();
    }
  }

  handleSerial();
}
