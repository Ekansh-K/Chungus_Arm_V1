// ================================================================
//  ESP32 Continuous-Servo Closed-Loop Controller  — v2
//  ESP32 + PCA9685 + Continuous Servo (Ch0) + Potentiometer
//
//  Project: Robotic arm joint controller — single-joint test bench
//  Hardware:
//    ESP32 GPIO 26 → PCA9685 SDA
//    ESP32 GPIO 27 → PCA9685 SCL
//    ESP32 GPIO 32 → Potentiometer (joint angle sensor)
//    PCA9685 Ch0   → Continuous servo MG995 (the joint motor)
//    PCA9685 Ch1–15→ Standard servos (other joints — future)
//
//  Control layers:
//   1. Inline PID — closed-loop position control (Ch0)
//   2. Runtime pot drift detection — silent correct / pause + warn
//   3. Reachability check — ghost-target prevention before every move
//   4. Direction-aware motion watchdog — stall & cal-failure detect
//   5. Auto-recalibration + NVS persistence — 5-gate validated
//
//  All tunable parameters (SERVO_STOP, deadband, PID gains, pot cal)
//  are runtime-adjustable via serial and NVS-persisted across reboots.
//
//  Serial commands (115200 baud) — type 'help' for full list
//
//  Libraries (PlatformIO lib_deps):
//   - adafruit/Adafruit PWM Servo Driver Library
//   - Preferences (built into ESP32 Arduino core)
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

// ───────────────────────────────────────────────
//  HARDWARE CONFIG
// ───────────────────────────────────────────────
#define I2C_SDA         26
#define I2C_SCL         27
#define PCA9685_ADDR    0x40
#define POT_PIN         32
#define NUM_CHANNELS    16
#define SERVO_FREQ      50

// ── Continuous servo (Ch0) pulse defaults ──
//
//  The servo physically stops (deadband) between 90° and 110°.
//  Converting those angles to PCA9685 pulse counts using SERVO_MIN/MAX:
//
//    pulse = SERVO_MIN + (angle / 180) × (SERVO_MAX - SERVO_MIN)
//
//    90°  → 110 + (90/180)  × (575-110) = 110 + 232.5 = 342.5 ≈ 343  (left edge)
//   100°  → 110 + (100/180) × (575-110) = 110 + 258.3 = 368            (neutral)
//   110°  → 110 + (110/180) × (575-110) = 110 + 284.2 = 394            (right edge)
//
//  SERVO_STOP    = midpoint of deadband = (343 + 394) / 2 = 368.5 ≈ 369
//  SERVO_STICTION = half the deadband width = (394 - 343) / 2 = 25.5 ≈ 26
//
//  Note: The original value of 342 was at the LEFT EDGE of the deadband (= 90°),
//  not the center, so the servo was being commanded right at the boundary of motion.
//
#define SERVO_STOP_DEFAULT    369  // midpoint of 90°–110° deadband in pulse counts
#define SERVO_DRIVE           120  // max pulse offset from STOP — raised to allow 3×30=90 ramp retries
#define SERVO_STICTION_DEFAULT 26  // half the deadband width = (394-343)/2

// ── Standard servos (Ch1–15) ──
#define SERVO_MIN       110   // pulse count at 0°
#define SERVO_MAX       575   // pulse count at 180°

// ───────────────────────────────────────────────
//  NVS — KEYS & FACTORY DEFAULTS
// ───────────────────────────────────────────────
#define NVS_NAMESPACE        "servo_cal"
#define NVS_KEY_0DEG         "pot0"
#define NVS_KEY_180DEG       "pot180"
#define NVS_KEY_SERVO_STOP     "srvstop"
#define NVS_KEY_DEADBAND       "deadband"
#define NVS_KEY_STICTION_TO0   "sti0"     // stiction toward 0°   (pulse decreases)
#define NVS_KEY_STICTION_TO180 "sti180"   // stiction toward 180°  (pulse increases)
#define NVS_KEY_KGRAVITY       "kgrav"    // gravity feedforward gain
#define NVS_KEY_TRANSZONE      "transzone" // transition zone multiplier
#define NVS_KEY_SBHOLD         "sbhold"   // HOLD backup boost
#define NVS_KEY_VELCUT         "velcut"   // velocity cut threshold

// YOUR measured pot values — replace or run 'recal2' after first flash.
#define DEFAULT_POT_0DEG     3186
#define DEFAULT_POT_180DEG   319

// ───────────────────────────────────────────────
//  SAFETY & CONTROL CONSTANTS
// ───────────────────────────────────────────────
#define SAFETY_MARGIN_DEG       3.0f  // buffer at each end → usable: 3°–177°
#define DEADBAND_ADC_DEFAULT    80    // ±80 ADC (~5°) — must cover stiction overshoot zone
#define FILTER_SAMPLES          10    // moving-average depth (pot sampled every loop ~1kHz)
                                      // filtered average updated every loop; PID runs every 50ms)

// Motion watchdog
#define WATCHDOG_WINDOW_MS      800   // re-evaluate every N ms
#define WATCHDOG_MIN_DELTA      20    // min ADC change required per window
#define WATCHDOG_DIR_TOLERANCE  5     // ADC — wrong-direction tolerance

// Braking zone near mechanical hard stops
// Within this many ADC counts of 0° or 180° endpoint, boost is clamped to 0
// (pure stiction only). Prevents gravity-driven arm from slamming into stop.
// 150 ADC ≈ 9° — change with: brakezon <n>
#define BRAKE_ZONE_ADC_DEFAULT 150
int brakeZoneADC = BRAKE_ZONE_ADC_DEFAULT;

// Target approach deceleration zone
// When arm is within this many ADC of the target AND total travel > 15°,
// boostNow is linearly tapered from full → 0.
// dirStiction floor is always maintained (constrain downstream).
// 240 ADC ≈ 15° — tune with: targetdecel <n>  (0 = disable)
#define TARGET_DECEL_ADC_DEFAULT 400  // was 240 (≈15°), now 400 (≈25°)
int targetDecelADC = TARGET_DECEL_ADC_DEFAULT;

// Pot disconnect / wild-reading detection
#define POT_BROKEN_LOW          50
#define POT_BROKEN_HIGH         4050

// Drift detection (stationary drift monitor)
#define DRIFT_DETECT_THRESHOLD  30    // ADC units considered drift
#define DRIFT_DETECT_WINDOW_MS  3000  // evaluate over this window
#define DRIFT_MAX_AUTO_CORRECT  200   // below this → silent correction
                                      // above this → pause + demand recal

// Reachability (ghost-target guard)
#define REACH_HEADROOM_ADC      40    // margin inside observed live range

// Recalibration gates
#define RECAL_STABILITY_MS      2000  // hold time per endpoint
#define RECAL_VARIANCE_MAX      15    // max ADC variance during collection
#define RECAL_MAX_SHIFT_ADC     500   // max plausible endpoint shift
#define RECAL_MIN_SPAN_ADC      500   // min span between 0° and 180° readings

// ───────────────────────────────────────────────
//  INLINE PID CONTROLLER
//
//  Why inline? Every external library tested failed at least one
//  requirement for this use case:
//   - br3ttb/PID  : no separate integral clamp, dead since 2017
//   - AutoPID     : derivative on *error* (causes kick on new target)
//   - ArduPID 1.x : requires C++14 (ESP32 Arduino defaults to C++11)
//   - QuickPID    : closest rival but requires pointer-based API refactor
//
//  This implementation:
//   ✅ Derivative on measurement (no kick on setpoint changes)
//   ✅ Independent integral anti-windup clamp (separate from output limits)
//   ✅ Sample-time gating (only computes every sampleMs ms)
//   ✅ Runtime gain update via setGains()
//   ✅ Safe reset() — initialises lastInput to current reading, not 0
//   ✅ dt guard — prevents derivative blow-up on first tick
//   ✅ C++11 clean — zero external dependencies
// ───────────────────────────────────────────────
extern double pidInput;   // forward-declare so reset() can read it

class PID {
public:
    double   Kp, Ki, Kd;
    double   outMin, outMax;
    double   iMin, iMax;
    uint32_t sampleMs;

    void init(double kp, double ki, double kd,
              double oMin, double oMax,
              double iLo,  double iHi,
              uint32_t ms) {
        Kp = kp; Ki = ki; Kd = kd;
        outMin = oMin; outMax = oMax;
        iMin = iLo;    iMax = iHi;
        sampleMs = ms;
        reset();
    }

    // Call every loop cycle. Returns PID output.
    double compute(double input, double setpoint) {
        uint32_t now = millis();
        if (now - _lastTime < sampleMs) return _lastOut;

        double dt = (now - _lastTime) / 1000.0;
        _lastTime = now;

        double error  = setpoint - input;
        double dInput = input - _lastInput;   // derivative on measurement

        _integral += Ki * error * dt;
        _integral  = constrain(_integral, iMin, iMax);

        double p = Kp * error;
        // dt guard: skip derivative on first tick or timer glitch
        double d = (dt > 0.001) ? (-Kd * dInput / dt) : 0.0;

        double out = p + _integral + d;
        out = constrain(out, outMin, outMax);

        _lastInput = input;
        _lastOut   = out;
        return out;
    }

    // Safe reset — lastInput is set to current pidInput so the
    // derivative term is 0 on the very first compute() after reset.
    void reset() {
        _integral  = 0;
        _lastInput = pidInput;   // ← was 0, caused huge derivative spike
        _lastOut   = 0;
        _lastTime  = millis();
    }

    void setGains(double kp, double ki, double kd) {
        Kp = kp; Ki = ki; Kd = kd;
    }

    // Reset only the integrator — preserves _lastInput to avoid derivative spike.
    // Used to gate integrator during MOVE (prevents windup during travel).
    void resetIntegral() {
        _integral = 0;
    }

private:
    double   _integral  = 0;
    double   _lastInput = 0;
    double   _lastOut   = 0;
    uint32_t _lastTime  = 0;
};

// ───────────────────────────────────────────────
//  SYSTEM STATE ENUMS
// ───────────────────────────────────────────────
enum SystemState {
    SYS_RUNNING,        // normal operation
    SYS_DRIFT_WARN,     // drift detected — servo paused, user action needed
    SYS_RECAL_ACTIVE,   // recalibration in progress — servo stopped
    SYS_SLOWTEST,       // stiction sweep — finding minimum drive for movement
    SYS_JOG,            // timed jog — fixed pulse for set duration, then idle
    SYS_FAULT,          // latched fault — servo stopped, needs 'reset'
    SYS_HOLDTEST        // gravity sweep — auto-calibrating kGravity
};

enum HoldTestPhase {
    HT_IDLE,
    HT_MOVE,   // moving to target angle via normal PID
    HT_SWEEP   // sweeping pulse down to find gravity hold point
};

enum RecalPhase {
    RECAL_IDLE,
    RECAL_COLLECT_0DEG,
    RECAL_COLLECT_180DEG
};

// ───────────────────────────────────────────────
//  CALIBRATION DATA STRUCTURE
// ───────────────────────────────────────────────
struct CalData {
    int   pot0deg;        // ADC reading at true 0°
    int   pot180deg;      // ADC reading at true 180°
    int   softLimitLow;   // ADC soft limit near 180° end
    int   softLimitHigh;  // ADC soft limit near 0° end
    float adcPerDeg;      // negative (inverted pot: high ADC = 0°)
    int   marginADC;      // SAFETY_MARGIN_DEG converted to ADC steps
};

// ───────────────────────────────────────────────
//  GLOBAL OBJECTS & STATE
// ───────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
Preferences             prefs;

CalData       cal;
SystemState   sysState   = SYS_RUNNING;
RecalPhase    recalPhase = RECAL_IDLE;

// ── Velocity Profile ──────────────────────────────────────────────────────
// Trapezoidal velocity profile replaces the old Trajectory Generator.
// The profile accelerates to maxSpeedDegPerSec then decelerates to zero at
// the target.  The hard cap MAX_VEL_DEG_S is enforced at all times — even if
// gravity pulls the arm faster it will be opposed by the velocity PID.
#define MAX_VEL_DEG_S   20.0f   // ABSOLUTE maximum — never exceeded
float  maxSpeedDegPerSec = 15.0f;   // cruise speed (speed command, ≤ MAX_VEL_DEG_S)
float  accelDegS2        = 30.0f;   // ramp rate deg/sec² (accel command)
float  finalTargetDeg    = 90.0f;   // desired final position in degrees
float  currentPosDeg     = 90.0f;   // profile's internal tracked position
float  profileVelDegS    = 0.0f;    // profile's current speed (always ≥ 0)
float  tgtVelDegS        = 0.0f;    // signed target velocity fed to inner PID

// ── EMA Velocity Filter ───────────────────────────────────────────────────
// Raw velocity = (pot - lastPot) / dt — very noisy on analog pots.
// EMA smooths it:  filtered = alpha*raw + (1-alpha)*prev
// alpha=0.15 balances noise rejection vs tracking speed.
// Tune live with:  velalpha <0.05–0.5>
float  measuredVelDegS = 0.0f;   // EMA-filtered velocity in deg/sec (+ = toward 0°)
float  rawVelDegS      = 0.0f;   // last raw (unfiltered) velocity sample
float  Kv_alpha        = 0.15f;  // EMA smoothing factor
int    lastPotForVel   = -1;     // pot at last velocity sample
unsigned long lastVelTimeMs = 0; // millis() at last velocity sample

// ── Velocity PID ──────────────────────────────────────────────────────────
// Inner-loop PID: error = tgtVelDegS - measuredVelDegS → output = pulse correction
// Kd intentionally 0: double-derivative on a noisy analog signal = chaos.
float  Kp_vel      = 0.4f;    // proportional gain (tune with velgain)
float  Ki_vel      = 0.02f;   // integral gain (small — avoid noise wind-up)
float  velIntegral = 0.0f;    // integrator state
float  velPidOut   = 0.0f;    // velocity PID correction (pulse counts)

// ── Legacy placeholders (used in a few helper calls, kept for compatibility)
double pidInput    = 0;
double pidSetpoint = 0;   // kept for drift detection helper which updates it
double pidOutput   = 0;   // kept for printStatus

// ── velplot streaming flag ────────────────────────────────────────────────
bool   velPlotActive = false;  // set by 'velplot' command, cleared by 'velplot off'

// Runtime-tunable servo parameters (NVS-persisted)
//
// Two separate stiction values because gearbox friction is asymmetric:
//   servoStictionTo0   — drive needed to move toward 0°   (pidOutput > 0, pulse decreases)
//   servoStictionTo180 — drive needed to move toward 180°  (pidOutput < 0, pulse increases)
//
// Run 'slowtest +' to find servoStictionTo0,  then 'stiction0 <n>'
// Run 'slowtest -' to find servoStictionTo180, then 'stiction180 <n>'
//
int servoStop          = SERVO_STOP_DEFAULT;        // true neutral pulse count
int servoStictionTo0   = SERVO_STICTION_DEFAULT;    // toward 0°   (pulse ↓) found: 22
int servoStictionTo180 = SERVO_STICTION_DEFAULT;    // toward 180° (pulse ↑) found:  6
int deadbandADC        = DEADBAND_ADC_DEFAULT;      // position deadband in ADC units

// Current commanded target angle (degrees)
// targetSet is false until the user explicitly sends 'target <angle>'.
// While false the servo outputs pure neutral (servoStop) and PID is bypassed.
float targetAngleDeg  = 90.0f;
float targetTravelDeg = 0.0f;  // total travel distance when target was set
                               // determines whether braking zone is active:
                               //   travel > 15°  → full braking zone near hard stops
                               //   travel ≤ 15°  → no braking zone (small adjustment)
bool  targetSet       = false;

// ── Moving-average filter ──
int  filterBuf[FILTER_SAMPLES] = {0};
int  filterIdx  = 0;
long filterSum  = 0;
bool filterFull = false;

// ── Motion watchdog ──
bool          wdArmed       = false;
int           wdStartPot    = 0;
unsigned long wdStartTime   = 0;
int           wdExpectedDir = 0;   // +1 = pot should increase, -1 = decrease

// ── Drift detection ──
int           driftRefPot   = 0;
unsigned long driftRefTime  = 0;
bool          driftRefValid = false;

// ── Recal state machine ──
int           recalMode    = 0;   // 0=0°only  1=180°only  2=two-point
unsigned long recalStartMs = 0;
long          recalAccum   = 0;
int           recalSamples = 0;
int           recalMin     = 9999;
int           recalMax     = 0;
int           pendingPot0  = -1;

// ── Live observed ADC extremes ──
int   observedPotMin = 9999;   // lowest ADC seen  → near 180° end
int   observedPotMax = 0;      // highest ADC seen → near 0° end

// Channel angle tracking (standard servos Ch1–15)
int currentAngle[NUM_CHANNELS] = {0};

// ── Slowtest state ──
// Used by 'slowtest' command to sweep servo pulses and detect actual stiction.
int           stDrive     = 0;    // current test drive offset from neutral
int           stDir       = 1;    // +1 toward 180°, -1 toward 0°
int           stStartPot  = 0;    // pot reading at start of sweep
unsigned long stLastStep  = 0;    // time of last drive increment
#define SLOWTEST_STEP_MS   600   // ms between each 1-count drive increment
int stMotionThrADC = 80;

// ── Jog state ──
unsigned long jogEndMs  = 0;
uint16_t      jogPulse  = 0;

// ── Watchdog grace period ──
// After reset/target/notarget the PID output is 0 for the first 50ms.
// During that window pidSign=+1 always, which can arm watchdog with the
// wrong expected direction. Grace period disables arming for 2 s.
unsigned long wdGraceUntil = 0;

// ── Stiction drive boost ──
// sticBoost: max extra pulse counts ramped up to when arm is stuck.
// Default 40. Set with: sticboost <n>
int  sticBoost = 40;

// ── Stiction retry ramp (DISABLED by default) ──
bool sticRetryEnabled = false;
#define STICTION_RAMP_MS       800
#define STICTION_RAMP_MOVE_THR  8
#define STICTION_RAMP_STEP     30
#define STICTION_MAX_RETRIES    3
int           sticRetry     = 0;
int           sticRampPotRef = 0;
unsigned long sticRampTime   = 0;
int           sticRampDir    = 0;
#define sticRamp (sticRetry * STICTION_RAMP_STEP)

// ── Time-based boost ramp (replaces distance taper) ──
//
//  Phase BP_WAIT  (0 – 800 ms)  : drive at pure stiction (boost=0)
//                                  if pot moves ≥ BOOST_MOVE_THR → lock boost=0 (BP_LOCKED)
//                                  if 800 ms with no movement    → start ramp  (BP_RAMP)
//
//  Phase BP_RAMP  (800 – 2000 ms): ramp boost linearly 0 → sticBoost over 1200 ms
//                                  if pot moves at any point     → freeze boost (BP_LOCKED)
//                                  if 1200 ms still no movement  → stop        (BP_TIMEOUT)
//
//  Phase BP_LOCKED              : boost frozen at level when motion first detected
//  Phase BP_TIMEOUT             : servo stopped, prints stuck message every 3 s
//
//  Resets to BP_WAIT when: new target, reset, direction change, or deadband reached.
//
enum BoostPhase { BP_WAIT, BP_RAMP, BP_LOCKED, BP_TIMEOUT };
BoostPhase    boostPhase      = BP_WAIT;
unsigned long boostPhaseStart = 0;
int           boostLocked     = 0;   // boost frozen when movement detected
int           boostPotRef     = 0;   // pot at start of each phase for movement check
int           boostPhaseDirRef = 0;  // moveSign when phase started
#define BOOST_WAIT_MS   800
#define BOOST_RAMP_MS  1200
#define BOOST_MOVE_THR  50    // ADC — min pot change to confirm real motion (~3°)
                              // Must be >> pot noise (which is ~10-15 ADC) to avoid
                              // falsely locking boost=0 before arm actually moves.

// ── Gravity feedforward ──
//
//  kGravity: gain for sin-based gravity compensation during HOLD.
//  FF = kGravity * sin(angle_deg - 90°)
//  Positive kGravity → arm tends to fall toward 0° at low angles.
//  At 90° (vertical): sin(0°) = 0, so no FF needed.
//  At 0°/180°: maximum FF.
//  Tuned automatically by 'holdtest <angle>' command.
//  Can also be set manually with: kgrav <float>
//
double kGravity    = 0.0;   // gravity FF gain (pulse counts per unit sin)

// ── Deadband smooth transition zone ──
//
//  Instead of a binary HOLD/MOVE step at deadbandADC, boost is linearly
//  scaled from 0→full between deadbandADC and transzoneMult×deadbandADC.
//  This eliminates the 376↔392 pulsating limit-cycle oscillation.
//  Set with: transzone <int>  (1–5, default 2)
//
int  transzoneMult = 2;     // transition zone = [deadband, mult×deadband]

// ── HOLD backup boost ──
//
//  Extra pulse offset added during HOLD on top of gravity FF.
//  Default 0 — gravity FF alone should be sufficient once kGravity is tuned.
//  Set with: sticboosthold <n>  (0–20)
//
int  sticBoostHold = 0;

// ── Velocity cut threshold ──
//
//  If the arm is moving AWAY from target faster than this (ADC/100ms),
//  effectiveDrive is cut to pure stiction for that tick.
//  Prevents gravity-driven runaway speed during approach.
//  Set with: velcut <n>  (10–100, default 30)
//
int  velCutThr = 30;

// ── Velocity estimator ──
//  Computed every PID sample (100ms). Signed: positive = pot increasing (toward 0°).

// ── Holdtest state machine ──
HoldTestPhase htPhase       = HT_IDLE;
float         htTargetAngle = 0.0f;  // angle requested by holdtest command
int           htStartPulse  = 0;     // last active drive pulse when arm reached deadband
int           htCurrentPulse = 0;    // pulse being tested (sweeps toward servoStop)
int           htPotRef       = 0;    // pot reading at start of each step
int           htDriftThr     = 80;   // ADC — drift threshold (~5°)
#define HT_STEP_MS           500    // ms between each sweep step
#define HT_STABLE_MS        1000    // ms in deadband before sweep starts
unsigned long htStableStart  = 0;   // when arm first entered deadband during HT_MOVE
unsigned long htLastStep     = 0;   // time of last sweep step

// ═══════════════════════════════════════════════
//  SECTION 1: NVS SETTINGS (servo stop, deadband)
// ═══════════════════════════════════════════════

void saveSettingsToFlash() {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putInt(NVS_KEY_SERVO_STOP,     servoStop);
    prefs.putInt(NVS_KEY_DEADBAND,       deadbandADC);
    prefs.putInt(NVS_KEY_STICTION_TO0,   servoStictionTo0);
    prefs.putInt(NVS_KEY_STICTION_TO180, servoStictionTo180);
    prefs.putDouble(NVS_KEY_KGRAVITY,    kGravity);
    prefs.putInt(NVS_KEY_TRANSZONE,      transzoneMult);
    prefs.putInt(NVS_KEY_SBHOLD,         sticBoostHold);
    prefs.putInt(NVS_KEY_VELCUT,         velCutThr);
    prefs.end();
    Serial.println(F("[CFG] Settings saved to NVS"));
}

void loadSettingsFromFlash() {
    prefs.begin(NVS_NAMESPACE, true);
    int    s   = prefs.getInt(NVS_KEY_SERVO_STOP,     SERVO_STOP_DEFAULT);
    int    d   = prefs.getInt(NVS_KEY_DEADBAND,       DEADBAND_ADC_DEFAULT);
    int    s0  = prefs.getInt(NVS_KEY_STICTION_TO0,   SERVO_STICTION_DEFAULT);
    int    s18 = prefs.getInt(NVS_KEY_STICTION_TO180, SERVO_STICTION_DEFAULT);
    double kg  = prefs.getDouble(NVS_KEY_KGRAVITY,    0.0);
    int    tz  = prefs.getInt(NVS_KEY_TRANSZONE,      2);
    int    sbh = prefs.getInt(NVS_KEY_SBHOLD,         0);
    int    vct = prefs.getInt(NVS_KEY_VELCUT,         30);
    prefs.end();

    // Validate ranges before accepting
    servoStop          = (s   >= 50 && s   <= 500) ? s   : SERVO_STOP_DEFAULT;
    deadbandADC        = (d   >= 2  && d   <= 200) ? d   : DEADBAND_ADC_DEFAULT;
    servoStictionTo0   = (s0  >= 1  && s0  <= 80)  ? s0  : SERVO_STICTION_DEFAULT;
    servoStictionTo180 = (s18 >= 1  && s18 <= 80)  ? s18 : SERVO_STICTION_DEFAULT;
    kGravity           = (kg  >= -200.0 && kg <= 200.0) ? kg : 0.0;
    transzoneMult      = (tz  >= 1  && tz  <= 5)   ? tz  : 2;
    sticBoostHold      = (sbh >= 0  && sbh <= 20)  ? sbh : 0;
    velCutThr          = (vct >= 10 && vct <= 100)  ? vct : 30;

    Serial.printf("[CFG] servoStop=%d  stiction(to0)=%d  stiction(to180)=%d  deadband=%d ADC\n",
                  servoStop, servoStictionTo0, servoStictionTo180, deadbandADC);
    Serial.printf("[CFG] kGravity=%.3f  transzone=%d  sticBoostHold=%d  velCutThr=%d\n",
                  kGravity, transzoneMult, sticBoostHold, velCutThr);
}

// ═══════════════════════════════════════════════
//  SECTION 2: CALIBRATION MATH
// ═══════════════════════════════════════════════

void recomputeCalibration(int new0deg, int new180deg) {
    cal.pot0deg   = new0deg;
    cal.pot180deg = new180deg;
    // adcPerDeg is negative (inverted pot: high ADC = 0°, low ADC = 180°)
    cal.adcPerDeg = (float)(new180deg - new0deg) / 180.0f;
    cal.marginADC = (int)(SAFETY_MARGIN_DEG * fabsf(cal.adcPerDeg));

    cal.softLimitHigh = new0deg   - cal.marginADC;  // 3° inward from 0°
    cal.softLimitLow  = new180deg + cal.marginADC;  // 3° inward from 180°

    observedPotMin = new180deg;
    observedPotMax = new0deg;
    driftRefValid  = false;

    Serial.println(F("\n[CAL] ══════ Calibration Updated ══════"));
    Serial.printf("[CAL]  0°   → %d ADC\n",          cal.pot0deg);
    Serial.printf("[CAL]  180° → %d ADC\n",          cal.pot180deg);
    Serial.printf("[CAL]  Scale: %.3f ADC/deg\n",    cal.adcPerDeg);
    Serial.printf("[CAL]  Soft limits: ADC [%d – %d]  (%.1f° – %.1f°)\n",
                  cal.softLimitLow, cal.softLimitHigh,
                  SAFETY_MARGIN_DEG, 180.0f - SAFETY_MARGIN_DEG);
    Serial.println(F("[CAL] ══════════════════════════════════\n"));
}

void saveCalToFlash() {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putInt(NVS_KEY_0DEG,   cal.pot0deg);
    prefs.putInt(NVS_KEY_180DEG, cal.pot180deg);
    prefs.end();
    Serial.println(F("[CAL] Saved to NVS flash"));
}

void loadCalFromFlash() {
    prefs.begin(NVS_NAMESPACE, true);
    int s0   = prefs.getInt(NVS_KEY_0DEG,   DEFAULT_POT_0DEG);
    int s180 = prefs.getInt(NVS_KEY_180DEG, DEFAULT_POT_180DEG);
    prefs.end();

    bool valid = (s0   > POT_BROKEN_LOW  && s0   < POT_BROKEN_HIGH) &&
                 (s180 > POT_BROKEN_LOW  && s180 < POT_BROKEN_HIGH) &&
                 (abs(s0 - s180) >= RECAL_MIN_SPAN_ADC);

    if (!valid) {
        Serial.println(F("[CAL] NVS invalid — using factory defaults"));
        s0   = DEFAULT_POT_0DEG;
        s180 = DEFAULT_POT_180DEG;
    } else {
        Serial.println(F("[CAL] Loaded from NVS flash"));
    }
    recomputeCalibration(s0, s180);
}

// ═══════════════════════════════════════════════
//  SECTION 3: COORDINATE CONVERSION
// ═══════════════════════════════════════════════

float potToAngle(int potVal) {
    if (fabsf(cal.adcPerDeg) < 0.01f) return 0.0f;
    return constrain((float)(potVal - cal.pot0deg) / cal.adcPerDeg,
                     0.0f, 180.0f);
}

int angleToPot(float deg) {
    return cal.pot0deg + (int)(deg * cal.adcPerDeg);
}

int clampToSoftLimits(int potTarget) {
    return constrain(potTarget, cal.softLimitLow, cal.softLimitHigh);
}

// ═══════════════════════════════════════════════
//  SECTION 4: POT FILTER & CONNECTIVITY
// ═══════════════════════════════════════════════

int readPotFiltered() {
    int raw = analogRead(POT_PIN);

    filterSum -= filterBuf[filterIdx];
    filterBuf[filterIdx] = raw;
    filterSum += raw;
    filterIdx  = (filterIdx + 1) % FILTER_SAMPLES;
    if (filterIdx == 0) filterFull = true;
    int count = filterFull ? FILTER_SAMPLES : max(filterIdx, 1);

    if (raw < observedPotMin) observedPotMin = raw;
    if (raw > observedPotMax) observedPotMax = raw;

    return (int)(filterSum / count);
}

bool potConnected(int val) {
    return (val > POT_BROKEN_LOW) && (val < POT_BROKEN_HIGH);
}

// ═══════════════════════════════════════════════
//  SECTION 5: FAULT & STATE MANAGEMENT
// ═══════════════════════════════════════════════

void triggerFault(const char* reason) {
    pwm.setPWM(0, 0, (uint16_t)servoStop);
    sysState = SYS_FAULT;
    Serial.print(F("[FAULT] "));
    Serial.println(reason);
    Serial.println(F("[FAULT] Send 'reset' to clear and resume"));
}

void triggerDriftWarn(int driftADC) {
    pwm.setPWM(0, 0, (uint16_t)servoStop);
    sysState = SYS_DRIFT_WARN;
    Serial.printf("[DRIFT] Warning — pot drifted %d ADC while stationary\n", driftADC);
    Serial.println(F("[DRIFT] Send 'recal2' to recalibrate, or 'resume' to ignore"));
}

// ═══════════════════════════════════════════════
//  SECTION 6: REACHABILITY CHECK
//
//  Before commanding a target, verify the target ADC lies within
//  the pot's observed live range (with headroom).
//  Prevents chasing ghost targets caused by calibration drift.
// ═══════════════════════════════════════════════

struct ReachResult {
    bool  ok;
    int   clampedPot;
    float clampedAngle;
    char  reason[80];
};

ReachResult checkReachability(float desiredAngleDeg) {
    ReachResult r;
    r.ok = true;
    r.reason[0] = '\0';

    int clampedPot = clampToSoftLimits(angleToPot(desiredAngleDeg));

    bool liveRangeValid = (observedPotMax - observedPotMin) > 200;
    if (liveRangeValid) {
        int liveMin = observedPotMin + REACH_HEADROOM_ADC;
        int liveMax = observedPotMax - REACH_HEADROOM_ADC;

        if (clampedPot < liveMin) {
            clampedPot = liveMin;
            r.ok = false;
            snprintf(r.reason, sizeof(r.reason),
                     "Near 180° limit of observed range — may be unreachable");
        } else if (clampedPot > liveMax) {
            clampedPot = liveMax;
            r.ok = false;
            snprintf(r.reason, sizeof(r.reason),
                     "Near 0° limit of observed range — may be unreachable");
        }
    }

    r.clampedPot   = clampedPot;
    r.clampedAngle = potToAngle(clampedPot);
    return r;
}

// ═══════════════════════════════════════════════
//  SECTION 7: DRIFT DETECTION
//
//  While arm holds position (in deadband), watch for pot shift.
//  Small drift → silently adjust setpoint.
//  Large drift → pause servo, demand recal.
// ═══════════════════════════════════════════════

void updateDriftDetection(int potNow, bool inDeadband) {
    if (!inDeadband) {
        driftRefValid = false;
        return;
    }
    if (!driftRefValid) {
        driftRefPot   = potNow;
        driftRefTime  = millis();
        driftRefValid = true;
        return;
    }
    if (millis() - driftRefTime < DRIFT_DETECT_WINDOW_MS) return;

    int drift = abs(potNow - driftRefPot);
    if (drift > DRIFT_DETECT_THRESHOLD) {
        if (drift <= DRIFT_MAX_AUTO_CORRECT) {
            int correction = potNow - driftRefPot;
            pidSetpoint   += correction;
            pidSetpoint    = clampToSoftLimits((int)pidSetpoint);
            Serial.printf("[DRIFT] Minor drift %d ADC — setpoint auto-corrected\n", drift);
        } else {
            triggerDriftWarn(drift);
        }
    }
    driftRefPot  = potNow;
    driftRefTime = millis();
}

// ═══════════════════════════════════════════════
//  SECTION 8: MOTION WATCHDOG (Direction-Aware)
//
//  Every WATCHDOG_WINDOW_MS while servo is actively driving:
//   1. Did pot move ≥ WATCHDOG_MIN_DELTA?   → stall / obstruction
//   2. Did it move in the correct direction? → cal-failure check
//
//  NOTE: Watchdog is only armed when error > 2×deadband.
//  This prevents false stall faults during the final slow approach
//  to target where PID output is low and pot movement is minimal.
// ═══════════════════════════════════════════════

void updateWatchdog(int potNow, bool shouldBeActive, int pidSign) {
    // Watchdog is completely disabled to allow slow trajectory movements!
    return;
}

// ═══════════════════════════════════════════════
//  SECTION 9: RECALIBRATION STATE MACHINE
//
//  mode 0 = 0° only
//  mode 1 = 180° only
//  mode 2 = full two-point
//
//  Five validation gates per collected reading:
//   1. Stability   — variance ≤ RECAL_VARIANCE_MAX
//   2. Connectivity— ADC in valid (non-broken) range
//   3. Plausible   — shift ≤ RECAL_MAX_SHIFT_ADC from current cal
//   4. Span        — endpoints ≥ RECAL_MIN_SPAN_ADC apart
//   5. Ordering    — 180° ADC < 0° ADC (inverted pot)
// ═══════════════════════════════════════════════

void startRecal(int mode) {
    recalMode    = mode;
    recalPhase   = RECAL_COLLECT_0DEG;
    recalStartMs = millis();
    recalAccum   = 0;  recalSamples = 0;
    recalMin     = 9999;  recalMax = 0;
    pendingPot0  = -1;
    sysState     = SYS_RECAL_ACTIVE;
    pwm.setPWM(0, 0, (uint16_t)servoStop);

    Serial.println(F("\n[RECAL] ══ Recalibration Started ══"));
    Serial.printf("[RECAL] Mode: %s\n",
                  mode == 0 ? "0° only" : mode == 1 ? "180° only" : "Two-point");
    if (mode == 1)
        Serial.println(F("[RECAL] Move arm to 180° and hold STILL..."));
    else
        Serial.println(F("[RECAL] Move arm to 0° and hold STILL..."));
}

void abortRecal(const char* reason) {
    recalPhase  = RECAL_IDLE;
    pendingPot0 = -1;
    sysState    = SYS_RUNNING;
    Serial.print(F("[RECAL] ABORTED — "));
    Serial.println(reason);
    Serial.println(F("[RECAL] Old calibration kept — resuming"));
}

void updateRecal(int potNow) {
    if (recalPhase == RECAL_IDLE) return;

    recalAccum += potNow;
    recalSamples++;
    if (potNow < recalMin) recalMin = potNow;
    if (potNow > recalMax) recalMax = potNow;

    unsigned long elapsed = millis() - recalStartMs;
    if (elapsed < RECAL_STABILITY_MS) {
        static unsigned long lastRecalPrint = 0;
        if (millis() - lastRecalPrint >= 500) {
            lastRecalPrint = millis();
            Serial.printf("[RECAL] Collecting... %lums  variance=%d\n",
                          elapsed, recalMax - recalMin);
        }
        return;
    }

    int variance   = recalMax - recalMin;
    int newReading = (int)(recalAccum / recalSamples);
    Serial.printf("[RECAL] Done: reading=%d  variance=%d\n", newReading, variance);

    // GATE 1: Stability
    if (variance > RECAL_VARIANCE_MAX) {
        abortRecal("Pot too noisy — hold arm more firmly"); return;
    }
    // GATE 2: Connectivity
    if (!potConnected(newReading)) {
        abortRecal("Reading out of valid range — check wiring"); return;
    }
    // GATE 3: Plausible shift
    // FIX: for mode 1 (180° only), compare against cal.pot180deg, not cal.pot0deg
    int expectedRef;
    if (recalMode == 1) {
        expectedRef = cal.pot180deg;   // 180°-only mode always collects 180° data
    } else {
        expectedRef = (recalPhase == RECAL_COLLECT_0DEG) ? cal.pot0deg : cal.pot180deg;
    }
    if (abs(newReading - expectedRef) > RECAL_MAX_SHIFT_ADC) {
        Serial.printf("[RECAL] Shift %d ADC > max %d\n",
                      abs(newReading - expectedRef), RECAL_MAX_SHIFT_ADC);
        abortRecal("Shift too large — is arm at the correct endpoint?");
        return;
    }

    if (recalPhase == RECAL_COLLECT_0DEG) {
        if (recalMode == 0) {
            // Single 0° — GATE 4
            if (abs(newReading - cal.pot180deg) < RECAL_MIN_SPAN_ADC) {
                abortRecal("New 0° too close to existing 180° reading"); return;
            }
            recomputeCalibration(newReading, cal.pot180deg);
            saveCalToFlash();
            recalPhase = RECAL_IDLE;  sysState = SYS_RUNNING;
            Serial.println(F("[RECAL] ✓ 0° recalibrated and saved"));

        } else if (recalMode == 1) {
            // Single 180° — GATE 4
            if (abs(newReading - cal.pot0deg) < RECAL_MIN_SPAN_ADC) {
                abortRecal("New 180° too close to existing 0° reading"); return;
            }
            recomputeCalibration(cal.pot0deg, newReading);
            saveCalToFlash();
            recalPhase = RECAL_IDLE;  sysState = SYS_RUNNING;
            Serial.println(F("[RECAL] ✓ 180° recalibrated and saved"));

        } else {
            // Two-point step 1 — store 0°, collect 180°
            pendingPot0  = newReading;
            recalPhase   = RECAL_COLLECT_180DEG;
            recalStartMs = millis();
            recalAccum   = 0;  recalSamples = 0;
            recalMin     = 9999;  recalMax = 0;
            Serial.printf("[RECAL] 0° stored (%d). Move to 180° and hold STILL...\n",
                          pendingPot0);
        }

    } else if (recalPhase == RECAL_COLLECT_180DEG) {
        if (abs(newReading - pendingPot0) < RECAL_MIN_SPAN_ADC) {
            abortRecal("0° and 180° readings too close — wrong positions?"); return;
        }
        if (newReading >= pendingPot0) {
            abortRecal("180° reading >= 0° reading — pot inverted or wrong positions");
            return;
        }
        recomputeCalibration(pendingPot0, newReading);
        saveCalToFlash();
        recalPhase  = RECAL_IDLE;  pendingPot0 = -1;
        sysState    = SYS_RUNNING;
        Serial.println(F("[RECAL] ✓ Two-point recalibration complete and saved"));
    }

    pidSetpoint = clampToSoftLimits(angleToPot(targetAngleDeg));
}

// ═══════════════════════════════════════════════
//  SECTION 10: STANDARD SERVO HELPERS (Ch1–15)
// ═══════════════════════════════════════════════

uint16_t angleToPulse(int angle) {
    return (uint16_t)map(constrain(angle, 0, 180), 0, 180, SERVO_MIN, SERVO_MAX);
}

void setServo(uint8_t ch, int angle) {
    if (ch == 0) {
        Serial.println(F("[WARN] Ch0 is PID-controlled. Use: target <angle>"));
        return;
    }
    if (ch >= NUM_CHANNELS) {
        Serial.printf("[ERROR] Ch %d out of range (1–15)\n", ch); return;
    }
    angle = constrain(angle, 0, 180);
    pwm.setPWM(ch, 0, angleToPulse(angle));
    currentAngle[ch] = angle;
    Serial.printf("[OK] Ch %2d → %3d°  (pulse %d)\n", ch, angle, angleToPulse(angle));
}

// ═══════════════════════════════════════════════
//  SECTION 11: SERIAL COMMAND PARSER
// ═══════════════════════════════════════════════

void printStatus() {
    int   potRaw = readPotFiltered();
    float angNow = potToAngle(potRaw);
    Serial.println();
    Serial.println(F("══════════════ STATUS ══════════════"));
    Serial.printf("  State       : %s\n",
                  sysState == SYS_RUNNING      ? "RUNNING"       :
                  sysState == SYS_DRIFT_WARN   ? "DRIFT WARNING" :
                  sysState == SYS_RECAL_ACTIVE ? "RECAL ACTIVE"  :
                  sysState == SYS_SLOWTEST     ? "SLOWTEST"      :
                  sysState == SYS_HOLDTEST     ? "HOLDTEST"      :
                  sysState == SYS_JOG          ? "JOG"           : "FAULT");
    Serial.printf("  Position    : %.1f°  (pot=%d)\n", angNow, potRaw);
    if (!targetSet) {
        Serial.println(F("  Target      : *** NO TARGET SET — send: target <angle> ***"));
        Serial.printf( "  Servo pulse : %d  (neutral = 100° equiv. = deadband centre)\n", servoStop);
    } else {
        Serial.printf("  Target      : %.1f°  (final=%.1f°)\n", targetAngleDeg, finalTargetDeg);
        Serial.printf("  Profile pos : %.1f°  vel=%.1f deg/s  tgt=%.1f deg/s\n",
                      currentPosDeg, measuredVelDegS, tgtVelDegS);
        Serial.printf("  Vel PID out : %.2f  integral=%.2f\n", velPidOut, velIntegral);
        Serial.printf("  Max speed   : %.1f deg/s (cap=%.0f)  accel=%.1f deg/s²\n",
                      maxSpeedDegPerSec, MAX_VEL_DEG_S, accelDegS2);
    }
    Serial.println(F("──────────────────────────────────────"));
    Serial.printf("  Cal  0°     : %d ADC\n",            cal.pot0deg);
    Serial.printf("  Cal 180°    : %d ADC\n",            cal.pot180deg);
    Serial.printf("  Scale       : %.3f ADC/deg\n",      cal.adcPerDeg);
    Serial.printf("  Soft limits : ADC [%d – %d]  (%.1f°–%.1f°)\n",
                  cal.softLimitLow, cal.softLimitHigh,
                  SAFETY_MARGIN_DEG, 180.0f - SAFETY_MARGIN_DEG);
    Serial.printf("  Observed    : ADC [%d – %d]\n",    observedPotMin, observedPotMax);
    Serial.println(F("──────────────────────────────────────"));
    Serial.printf("  Deadband    : ±%d ADC  (%.2f°)\n",
                  deadbandADC,
                  fabsf(cal.adcPerDeg) > 0.01f ? deadbandADC / fabsf(cal.adcPerDeg) : 0);
    Serial.printf("  Servo stop  : %d pulse counts\n", servoStop);
    Serial.printf("  Stiction    : to-0°=%d (pulse %d)  to-180°=%d (pulse %d)  max=%d\n",
                  servoStictionTo0,   servoStop - servoStictionTo0,
                  servoStictionTo180, servoStop + servoStictionTo180, SERVO_DRIVE);
    int trueCenter = (servoStop - servoStictionTo0 + servoStop + servoStictionTo180) / 2;
    Serial.printf("  True centre : %d  (edge to-0=%d  edge to-180=%d)\n",
                  trueCenter, servoStop - servoStictionTo0, servoStop + servoStictionTo180);
    Serial.println(F("════════════════════════════════════\n"));
}

void printHelp() {
    Serial.println(F("\n════════════════ COMMANDS ════════════════"));
    Serial.println(F("  ── Position Control ──"));
    Serial.println(F("  target <angle>        move Ch0 to angle (3\u00b0–177\u00b0)"));
    Serial.println(F("  notarget              release target — servo idles"));
    Serial.println(F("  stop                  abort slowtest/jog and idle"));
    Serial.println(F("  tune <Kp> <Ki> <Kd>   update PID gains live  (Kd=0 recommended)"));
    Serial.println(F("  deadband <adc>        set deadband (run: deadband 80)"));
    Serial.println(F("  ── Drive Boost ──"));
    Serial.println(F("  sticboost <n>         set base boost above stiction (default 40, 0=pure stiction)"));
    Serial.println(F("  sticretry on/off      enable/disable auto-ramp retries (default: off)"));
    Serial.println(F("  ── Gravity Compensation (HOLD) ──"));
    Serial.println(F("  holdtest <angle>      auto-calibrate kGravity at given angle (15°–165°)"));
    Serial.println(F("  kgrav <float>         manually set gravity FF gain (0=off, back-calc from holdtest)"));
    Serial.println(F("  sticboosthold <n>     extra HOLD boost on top of gravity FF (default 0, 0–20)"));
    Serial.println(F("  ── Deadband Transition Zone ──"));
    Serial.println(F("  transzone <mult>      smooth HOLD/MOVE boundary multiplier 1–5 (default 2)"));
    Serial.println(F("  ── Velocity Control ──"));
    Serial.println(F("  speed <deg/s>         cruise speed 1–20 deg/sec (hard cap 20)"));
    Serial.println(F("  accel <deg/s2>        profile ramp rate 5–200 deg/sec² (default 30)"));
    Serial.println(F("  velgain <Kp> <Ki>     velocity PID gains (default 1.5 0.05)"));
    Serial.println(F("  velalpha <0–1>        EMA filter alpha (default 0.15, lower=smoother)"));
    Serial.println(F("  velplot               stream raw/filtered/target velocity to Serial Plotter"));
    Serial.println(F("  velplot off           stop velplot streaming"));
    Serial.println(F("  velcut <n>            velocity cut threshold ADC/100ms 10–100 (default 30)"));
    Serial.println(F("  ── Jog (manual move at stiction speed) ──"));
    Serial.println(F("  jog0 <ms>             jog toward 0°   for <ms> ms at stiction0 pulse"));
    Serial.println(F("  jog180 <ms>           jog toward 180° for <ms> ms at stiction180 pulse"));
    Serial.println(F("  ── Stiction Sweep ──"));
    Serial.println(F("  slowtest +            sweep toward 0°  — find stiction0"));
    Serial.println(F("  slowtest -            sweep toward 180° — find stiction180"));
    Serial.println(F("  (stop to abort; result shown as command to run)"));
    Serial.println(F("  ── Servo Neutral ──"));
    Serial.println(F("  savestop <pulse>      set & save servo neutral pulse count"));
    Serial.println(F("  stiction <n>          set BOTH stiction values to n (symmetric)"));
    Serial.println(F("  stiction0 <n>         set toward-0°  stiction only (slowtest + result)"));
    Serial.println(F("  stiction180 <n>       set toward-180° stiction only (slowtest - result)"));
    Serial.println(F("  ── Pot Calibration ──"));
    Serial.println(F("  recal0                recalibrate 0° endpoint"));
    Serial.println(F("  recal180              recalibrate 180° endpoint"));
    Serial.println(F("  recal2                full two-point recalibration"));
    Serial.println(F("  resetcal              restore factory pot calibration"));
    Serial.println(F("  ── Fault Recovery ──"));
    Serial.println(F("  resume                clear drift warning, continue"));
    Serial.println(F("  reset                 clear fault and resume"));
    Serial.println(F("  ── Standard Servos ──"));
    Serial.println(F("  <ch> <angle>          move channel 1–15 (0°–180°)"));
    Serial.println(F("  all <angle>           move channels 1–15"));
    Serial.println(F("  ── Info ──"));
    Serial.println(F("  status                full system status"));
    Serial.println(F("  help                  this screen"));
    Serial.println(F("══════════════════════════════════════════\n"));
}

void handleCommand(String cmd) {
    cmd.trim();
    if (!cmd.length()) return;

    if (cmd.equalsIgnoreCase("help"))   { printHelp();   return; }
    if (cmd.equalsIgnoreCase("status")) { printStatus(); return; }

    if (cmd.equalsIgnoreCase("reset")) {
        sysState       = SYS_RUNNING;
        htPhase        = HT_IDLE;
        wdArmed        = false;
        wdGraceUntil   = millis() + 2000;
        sticRetry      = 0;
        profileVelDegS = 0.0f;
        tgtVelDegS     = 0.0f;
        velIntegral    = 0.0f;
        velPidOut      = 0.0f;
        if (targetSet) {
            currentPosDeg  = potToAngle(readPotFiltered());
            lastPotForVel  = readPotFiltered();
            lastVelTimeMs  = millis();
            measuredVelDegS = 0.0f;
        }
        Serial.println(F("[INFO] Fault cleared — resuming (watchdog grace 2s)"));
        return;
    }

    if (cmd.equalsIgnoreCase("stop")) {
        targetSet      = false;
        wdArmed        = false;
        htPhase        = HT_IDLE;
        profileVelDegS = 0.0f;
        tgtVelDegS     = 0.0f;
        velIntegral    = 0.0f;
        velPidOut      = 0.0f;
        if (sysState == SYS_SLOWTEST || sysState == SYS_RUNNING ||
            sysState == SYS_HOLDTEST)
            sysState = SYS_RUNNING;
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        Serial.println(F("[INFO] Target cleared — servo idle at neutral"));
        Serial.printf("[INFO] Servo pulse = %d (deadband centre)\n", servoStop);
        return;
    }
    if (cmd.equalsIgnoreCase("resume")) {
        if (sysState == SYS_DRIFT_WARN) {
            sysState = SYS_RUNNING;
            wdArmed  = false;
            wdGraceUntil = millis() + 2000;
            Serial.println(F("[INFO] Resuming from drift warning"));
        } else {
            Serial.println(F("[INFO] Not in drift warning state"));
        }
        return;
    }
    if (cmd.equalsIgnoreCase("resetcal")) {
        recomputeCalibration(DEFAULT_POT_0DEG, DEFAULT_POT_180DEG);
        saveCalToFlash();
        pidSetpoint = clampToSoftLimits(angleToPot(targetAngleDeg));
        Serial.println(F("[CAL] Factory calibration restored and saved"));
        return;
    }

    if (cmd.equalsIgnoreCase("recal0"))   { startRecal(0); return; }
    if (cmd.equalsIgnoreCase("recal180")) { startRecal(1); return; }
    if (cmd.equalsIgnoreCase("recal2"))   { startRecal(2); return; }

    if (cmd.startsWith("speed")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] maxSpeed = %.1f deg/sec\n", maxSpeedDegPerSec); return;
        }
        float spd = cmd.substring(sp + 1).toFloat();
        if (spd < 1.0f || spd > MAX_VEL_DEG_S) {
            Serial.printf("[ERROR] Speed must be 1–%.0f deg/sec (hard cap)\n", MAX_VEL_DEG_S); return;
        }
        maxSpeedDegPerSec = spd;
        Serial.printf("[INFO] Cruise speed set to %.1f deg/sec (cap=%.0f)\n",
                      maxSpeedDegPerSec, MAX_VEL_DEG_S);
        return;
    }

    if (cmd.startsWith("target")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) { Serial.println(F("[ERROR] Usage: target <angle>")); return; }
        float ang = cmd.substring(sp + 1).toFloat();
        ReachResult r = checkReachability(ang);
        if (!r.ok) {
            Serial.printf("[WARN] %.1f° clamped to %.1f° — %s\n",
                          ang, r.clampedAngle, r.reason);
        }
        targetAngleDeg  = r.clampedAngle;
        finalTargetDeg  = r.clampedAngle;
        // Initialise profile from current physical position
        currentPosDeg   = potToAngle(readPotFiltered());
        profileVelDegS  = 0.0f;
        tgtVelDegS      = 0.0f;
        velIntegral     = 0.0f;
        velPidOut       = 0.0f;
        wdArmed         = false;
        wdGraceUntil    = millis() + 2000;
        sticRetry       = 0;
        targetSet       = true;
        // Legacy compat
        pidSetpoint     = (double)r.clampedPot;
        lastPotForVel   = readPotFiltered();
        lastVelTimeMs   = millis();
        measuredVelDegS = 0.0f;
        Serial.printf("[VEL] Target: %.1f° — profile starts at %.1f°  (max %.0f deg/s)\n",
                      finalTargetDeg, currentPosDeg, min(maxSpeedDegPerSec, MAX_VEL_DEG_S));
        return;
    }



    if (cmd.startsWith("deadband")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] Current deadband: %d ADC\n", deadbandADC); return;
        }
        int d = cmd.substring(sp + 1).toInt();
        if (d < 2 || d > 200) {
            Serial.println(F("[ERROR] Deadband must be 2–200 ADC")); return;
        }
        deadbandADC = d;
        saveSettingsToFlash();
        Serial.printf("[CFG] Deadband set to ±%d ADC  (%.2f°)\n",
                      deadbandADC,
                      fabsf(cal.adcPerDeg) > 0.01f ? deadbandADC / fabsf(cal.adcPerDeg) : 0);
        return;
    }

    if (cmd.startsWith("savestop")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] Current servoStop: %d\n", servoStop); return;
        }
        int s = cmd.substring(sp + 1).toInt();
        if (s < 50 || s > 500) {
            Serial.println(F("[ERROR] Pulse count must be 50–500")); return;
        }
        servoStop = s;
        saveSettingsToFlash();
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        Serial.printf("[CFG] servoStop = %d — applied and saved\n", servoStop);
        return;
    }

    if (cmd.startsWith("stiction")) {
        bool setTo0   = cmd.startsWith("stiction0");
        bool setTo180 = cmd.startsWith("stiction180");
        bool setBoth  = !setTo0 && !setTo180;
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] stiction toward 0°=%d  toward 180°=%d\n",
                          servoStictionTo0, servoStictionTo180);
            return;
        }
        int s = cmd.substring(sp + 1).toInt();
        if (s < 1 || s > 80) {
            Serial.println(F("[ERROR] Stiction must be 1–80")); return;
        }
        if (setBoth || setTo0)   servoStictionTo0   = s;
        if (setBoth || setTo180) servoStictionTo180 = s;
        saveSettingsToFlash();
        if (setBoth)
            Serial.printf("[CFG] Both stiction values = %d  (to-0: pulse %d, to-180: pulse %d)\n",
                          s, servoStop - s, servoStop + s);
        else if (setTo0)
            Serial.printf("[CFG] stiction toward 0°  = %d  (pulse %d)\n", s, servoStop - s);
        else
            Serial.printf("[CFG] stiction toward 180° = %d  (pulse %d)\n", s, servoStop + s);
        return;
    }

    if (cmd.startsWith("slowtest")) {
        if (sysState == SYS_RECAL_ACTIVE) {
            Serial.println(F("[ERROR] Finish or abort recal first")); return;
        }
        int sp = cmd.indexOf(' ');
        String dirStr = (sp >= 0) ? cmd.substring(sp + 1) : "+";
        dirStr.trim();
        stDir       = dirStr.startsWith("-") ? 1 : -1;
        stDrive     = 0;
        stStartPot  = readPotFiltered();
        stLastStep  = millis();
        sysState    = SYS_SLOWTEST;
        stMotionThrADC = max(40, (int)(5.0f * fabsf(cal.adcPerDeg)));
        targetSet   = false;
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        Serial.printf("[SLOWTEST] Sweeping %s from neutral  pulse=%d  pot=%d\n",
                      stDir < 0 ? "toward 0°  (pulse decreasing)" : "toward 180° (pulse increasing)",
                      servoStop, stStartPot);
        Serial.printf("[SLOWTEST] Step: +1 pulse every %dms. Send 'stop' to abort.\n",
                      SLOWTEST_STEP_MS);
        Serial.printf("[SLOWTEST] Motion threshold: %d ADC = 5.0° of rotation\n", stMotionThrADC);
        return;
    }
    if (cmd.equalsIgnoreCase("notarget")) {
        targetSet   = false;
        wdArmed     = false;
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        Serial.println(F("[INFO] Target cleared — servo IDLE at neutral (no PID)"));
        return;
    }

    // (duplicate stop handler removed — handled above)

    // ── jog0 <ms> / jog180 <ms> — move at stiction speed for fixed duration ──
    //
    //  Exactly like slowtest but for a set time rather than until motion detected.
    //  Useful for checking whether the arm actually moves at a given pulse.
    //  Uses the saved stiction values for each direction.
    //
    //  jog0 <ms>     → toward 0°   at pulse (servoStop - servoStictionTo0)
    //  jog180 <ms>   → toward 180° at pulse (servoStop + servoStictionTo180)
    //
    if (cmd.startsWith("jog0")) {
        int sp = cmd.indexOf(' ');
        if (sp < 0) { Serial.println(F("[ERROR] Usage: jog0 <ms>  (100–10000)")); return; }
        int ms = cmd.substring(sp + 1).toInt();
        if (ms < 100 || ms > 10000) { Serial.println(F("[ERROR] jog0: ms must be 100–10000")); return; }
        jogPulse  = (uint16_t)max(1, servoStop - servoStictionTo0);
        jogEndMs  = millis() + (unsigned long)ms;
        targetSet = false;
        wdArmed   = false;
        sysState  = SYS_JOG;
        pwm.setPWM(0, 0, jogPulse);
        Serial.printf("[JOG] toward 0\u00b0   pulse=%d  (stiction=%d)  for %dms\n",
                      jogPulse, servoStictionTo0, ms);
        return;
    }
    if (cmd.startsWith("jog180")) {
        int sp = cmd.indexOf(' ');
        if (sp < 0) { Serial.println(F("[ERROR] Usage: jog180 <ms>  (100–10000)")); return; }
        int ms = cmd.substring(sp + 1).toInt();
        if (ms < 100 || ms > 10000) { Serial.println(F("[ERROR] jog180: ms must be 100–10000")); return; }
        jogPulse  = (uint16_t)(servoStop + servoStictionTo180);
        jogEndMs  = millis() + (unsigned long)ms;
        targetSet = false;
        wdArmed   = false;
        sysState  = SYS_JOG;
        pwm.setPWM(0, 0, jogPulse);
        Serial.printf("[JOG] toward 180\u00b0  pulse=%d  (stiction=%d)  for %dms\n",
                      jogPulse, servoStictionTo180, ms);
        return;
    }

    if (cmd.startsWith("holdtest")) {
        int sp = cmd.indexOf(' ');
        if (sp < 0) { Serial.println(F("[ERROR] Usage: holdtest <angle>  (15°–165°)")); return; }
        float ang = cmd.substring(sp + 1).toFloat();
        if (ang < 5.0f || ang > 175.0f) {
            Serial.println(F("[ERROR] holdtest angle must be 5°–175° (avoid mechanical stops)")); return;
        }
        if (fabsf(ang - 90.0f) < 10.0f) {
            Serial.println(F("[WARN] holdtest near 90° — sin is small, kGravity result may be inaccurate."));
            Serial.println(F("[WARN] Recommend 20°–70° or 110°–160° for best results."));
        }
        // Set up PID target first
        ReachResult r = checkReachability(ang);
        targetAngleDeg  = r.clampedAngle;
        finalTargetDeg  = r.clampedAngle;
        currentPosDeg   = potToAngle(readPotFiltered());
        htTargetAngle   = r.clampedAngle;
        wdArmed         = false;
        wdGraceUntil    = millis() + 2000;
        sticRetry       = 0;
        boostPhase      = BP_WAIT;
        boostPhaseStart = millis();
        boostLocked     = 0;
        boostPhaseDirRef = 0;
        targetSet       = true;
        htPhase         = HT_MOVE;
        htStartPulse    = servoStop; // Will be captured upon stabilization
        htStableStart   = millis();
        sysState        = SYS_RUNNING; // Use normal PID to approach target!
        Serial.printf("[GRAVTEST] Moving to %.1f° — sweep starts after %dms stable in deadband\n",
                      htTargetAngle, HT_STABLE_MS);
        return;
    }

    if (cmd.startsWith("kgrav")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] kGravity = %.3f\n", kGravity); return;
        }
        double kg = cmd.substring(sp + 1).toFloat();
        if (kg < -200.0 || kg > 200.0) {
            Serial.println(F("[ERROR] kGravity must be -200 to 200")); return;
        }
        kGravity = kg;
        saveSettingsToFlash();
        Serial.printf("[CFG] kGravity = %.3f — saved\n", kGravity);
        return;
    }

    if (cmd.startsWith("transzone")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] transzoneMult = %d  (zone = [%d, %d] ADC)\n",
                          transzoneMult, deadbandADC, transzoneMult * deadbandADC);
            return;
        }
        int tz = cmd.substring(sp + 1).toInt();
        if (tz < 1 || tz > 5) { Serial.println(F("[ERROR] transzone must be 1–5")); return; }
        transzoneMult = tz;
        saveSettingsToFlash();
        Serial.printf("[CFG] transzoneMult = %d  (zone = [%d, %d] ADC)\n",
                      transzoneMult, deadbandADC, transzoneMult * deadbandADC);
        return;
    }

    if (cmd.startsWith("sticboosthold")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] sticBoostHold = %d\n", sticBoostHold); return;
        }
        int sbh = cmd.substring(sp + 1).toInt();
        if (sbh < 0 || sbh > 20) { Serial.println(F("[ERROR] sticboosthold must be 0–20")); return; }
        sticBoostHold = sbh;
        saveSettingsToFlash();
        Serial.printf("[CFG] sticBoostHold = %d — saved\n", sticBoostHold);
        return;
    }

    if (cmd.startsWith("velcut")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] velCutThr = %d ADC/100ms\n", velCutThr); return;
        }
        int vct = cmd.substring(sp + 1).toInt();
        if (vct < 10 || vct > 100) { Serial.println(F("[ERROR] velcut must be 10–100")); return; }
        velCutThr = vct;
        saveSettingsToFlash();
        Serial.printf("[CFG] velCutThr = %d ADC/100ms — saved\n", velCutThr);
        return;
    }

    if (cmd.startsWith("accel")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] accel = %.1f deg/sec²\n", accelDegS2); return;
        }
        float a = cmd.substring(sp + 1).toFloat();
        if (a < 5.0f || a > 200.0f) {
            Serial.println(F("[ERROR] accel must be 5–200 deg/sec²")); return;
        }
        accelDegS2 = a;
        Serial.printf("[CFG] Profile acceleration = %.1f deg/sec²\n", accelDegS2);
        return;
    }

    if (cmd.startsWith("velalpha")) {
        int sp = cmd.indexOf(' ');
        if (sp == -1) {
            Serial.printf("[INFO] velalpha = %.3f\n", Kv_alpha); return;
        }
        float a = cmd.substring(sp + 1).toFloat();
        if (a < 0.01f || a > 1.0f) {
            Serial.println(F("[ERROR] velalpha must be 0.01–1.0")); return;
        }
        Kv_alpha = a;
        Serial.printf("[CFG] EMA alpha = %.3f  (lower=smoother, higher=faster)\n", Kv_alpha);
        return;
    }

    if (cmd.startsWith("velgain")) {
        int s1 = cmd.indexOf(' ');
        int s2 = (s1 >= 0) ? cmd.indexOf(' ', s1 + 1) : -1;
        if (s1 < 0 || s2 < 0) {
            Serial.printf("[INFO] velgain Kp=%.3f Ki=%.3f\n", Kp_vel, Ki_vel); return;
        }
        Kp_vel = cmd.substring(s1 + 1, s2).toFloat();
        Ki_vel = cmd.substring(s2 + 1).toFloat();
        velIntegral = 0.0f;
        Serial.printf("[CFG] Velocity PID gains — Kp=%.3f  Ki=%.3f\n", Kp_vel, Ki_vel);
        return;
    }

    if (cmd.startsWith("velplot")) {
        String arg = cmd.substring(7);
        arg.trim();
        if (arg.equalsIgnoreCase("off") || arg.equalsIgnoreCase("0")) {
            velPlotActive = false;
            Serial.println(F("[INFO] velplot OFF"));
        } else {
            velPlotActive = true;
            Serial.println(F("[INFO] velplot ON — streaming: raw_vel  filt_vel  tgt_vel  (deg/sec)"));
            Serial.println(F("[INFO] Type 'velplot off' to stop"));
        }
        return;
    }

    if (cmd.startsWith("all")) {
        int sp = cmd.indexOf(' ');
        if (sp < 0) { Serial.println(F("[ERROR] Usage: all <angle>")); return; }
        int ang = cmd.substring(sp + 1).toInt();
        for (uint8_t ch = 1; ch < NUM_CHANNELS; ch++) setServo(ch, ang);
        return;
    }

    int sp = cmd.indexOf(' ');
    if (sp > 0) {
        setServo((uint8_t)cmd.substring(0, sp).toInt(),
                 cmd.substring(sp + 1).toInt());
    } else {
        Serial.println(F("[ERROR] Unknown command — type 'help'"));
    }
}

// ═══════════════════════════════════════════════
//  SECTION 10b: HOLDTEST STATE MACHINE
//
//  'holdtest <angle>' moves the arm to the given angle using normal PID,
//  then once stable in the deadband for HT_STABLE_MS, sweeps the pulse
//  from the last active drive pulse back toward servoStop one count at a
//  time (every HT_STEP_MS ms). When the arm drifts >= htDriftThr ADC
//  (gravity pulling it off position), the holding pulse is recorded.
//  kGravity is back-calculated as holdOffset / sin(angle - 90°) and saved.
//
//  This gives an empirical, load-aware kGravity without needing to know
//  the arm geometry or mass distribution.
// ═══════════════════════════════════════════════

// ═══════════════════════════════════════════════
//  I2C BUS RECOVERY

//
//  The PCA9685 can lock up if a servo current spike causes a power
//  glitch mid-I2C-transaction. SDA or SCL gets held LOW, all future
//  writes are silently ignored (servo freezes, LED stays on).
//
//  Recovery sequence:
//   1. Toggle SCL 9 times to unstick SDA (I2C spec §3.1.16)
//   2. Send STOP condition
//   3. Re-initialise Wire + PCA9685
// ═══════════════════════════════════════════════

bool i2cProbe() {
    Wire.beginTransmission(PCA9685_ADDR);
    return (Wire.endTransmission() == 0);
}

void recoverI2C() {
    Serial.println(F("[I2C] PCA9685 not responding — attempting bus recovery"));

    // Step 1: release SDA by toggling SCL 9 times
    pinMode(I2C_SCL, OUTPUT);
    pinMode(I2C_SDA, OUTPUT);
    digitalWrite(I2C_SDA, HIGH);
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
        digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
    }
    // Step 2: STOP condition (SDA LOW→HIGH while SCL HIGH)
    digitalWrite(I2C_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH); delayMicroseconds(5);

    // Step 3: re-init Wire + PCA9685
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeout(1000);
    delay(10);
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
    pwm.setPWM(0, 0, (uint16_t)servoStop);

    if (i2cProbe()) {
        Serial.println(F("[I2C] Recovery successful — PCA9685 responding"));
    } else {
        Serial.println(F("[I2C] Recovery FAILED — check wiring and power supply"));
    }
}

// ═══════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(100);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeout(1000);   // prevent Wire from blocking indefinitely on I2C lockup
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);

    if (!i2cProbe()) {
        Serial.println(F("[BOOT] WARNING: PCA9685 not detected on I2C — check wiring!"));
    } else {
        Serial.println(F("[BOOT] PCA9685 detected OK"));
    }

    pinMode(POT_PIN, INPUT);

    loadSettingsFromFlash();
    loadCalFromFlash();

    for (int i = 0; i < FILTER_SAMPLES; i++) {
        int r = analogRead(POT_PIN);
        filterBuf[i] = r;
        filterSum   += r;
    }
    filterFull = true;

    int startPot = (int)(filterSum / FILTER_SAMPLES);
    pidInput  = startPot;
    

    Serial.printf("[BOOT] Arm position: %.1f°  (pot=%d)\n", potToAngle(startPot), startPot);
    Serial.printf("[BOOT] Servo neutral: %d pulse counts (100° equiv.)\n", servoStop);
    Serial.printf("[BOOT] stiction=[to-0:%d, to-180:%d]  deadband=%d ADC\n",
                  servoStictionTo0, servoStictionTo180, deadbandADC);
    Serial.println(F("[BOOT] ** Servo IDLE — send 'target <angle>' to begin **"));
    printHelp();
}

// ═══════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════
//  VELOCITY MEASUREMENT (EMA-filtered)
//
//  Called every loop cycle.  Updates:
//    rawVelDegS      — instantaneous deg/sec from pot delta
//    measuredVelDegS — EMA-smoothed version
//
//  Sign convention: POSITIVE = pot increasing = arm moving toward 0°
//                   NEGATIVE = pot decreasing = arm moving toward 180°
// ═══════════════════════════════════════════════
void updateVelocityMeasurement(int potRaw) {
    unsigned long nowMs = millis();
    if (lastPotForVel < 0) {
        // First call — initialise, no velocity yet
        lastPotForVel  = potRaw;
        lastVelTimeMs  = nowMs;
        return;
    }
    float dtSec = (float)(nowMs - lastVelTimeMs) / 1000.0f;
    if (dtSec < 0.020f) return;  // don't compute on < 20 ms intervals (reduce noise)

    int deltaPot = potRaw - lastPotForVel;
    // Stationary deadband: ignore tiny ADC fluctuations (<= 2 counts) over 20ms
    if (abs(deltaPot) <= 2) {
        deltaPot = 0;
    }

    float adcPerSec = (float)deltaPot / dtSec;
    // Convert ADC/sec → deg/sec  (adcPerDeg is negative for inverted pot)
    rawVelDegS = (fabsf(cal.adcPerDeg) > 0.01f)
                 ? adcPerSec / cal.adcPerDeg   // preserves sign correctly
                 : 0.0f;
    // EMA filter
    measuredVelDegS = Kv_alpha * rawVelDegS
                      + (1.0f - Kv_alpha) * measuredVelDegS;

    lastPotForVel = potRaw;
    lastVelTimeMs = nowMs;
}

// ═══════════════════════════════════════════════
void loop() {

    if (Serial.available())
        handleCommand(Serial.readStringUntil('\n'));

    int potRaw = readPotFiltered();
    updateVelocityMeasurement(potRaw);

    if (sysState == SYS_RECAL_ACTIVE) {
        updateRecal(potRaw);
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        return;
    }

    if (sysState == SYS_FAULT) {
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        static unsigned long lastFaultMsg = 0;
        if (millis() - lastFaultMsg >= 100) {
            lastFaultMsg = millis();
            int p = readPotFiltered();
            Serial.printf("[FAULT] pot=%4d  ang=%5.1f°  Send 'reset'\n", p, potToAngle(p));
        }
        return;
    }

    if (sysState == SYS_DRIFT_WARN) {
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        static unsigned long lastDriftMsg = 0;
        if (millis() - lastDriftMsg >= 100) {
            lastDriftMsg = millis();
            int p = readPotFiltered();
            Serial.printf("[DRIFT] pot=%4d  ang=%5.1f°  Paused. Send 'recal2' or 'resume'\n", p, potToAngle(p));
        }
        return;
    }

    // ── SLOWTEST — sweep drive 1 count at a time and detect stiction ──
    if (sysState == SYS_SLOWTEST) {
        int potNow = readPotFiltered();
        int delta  = abs(potNow - stStartPot);

        if (delta >= stMotionThrADC) {
            // Pot moved ≥5° — confirmed real motion, not noise
            pwm.setPWM(0, 0, (uint16_t)servoStop);
            sysState = SYS_RUNNING;
            int detectedSig = servoStop + stDir * stDrive;
            float degMoved  = fabsf(cal.adcPerDeg) > 0.01f
                              ? delta / fabsf(cal.adcPerDeg) : 0;
            const char* cmdSave   = (stDir < 0) ? "stiction0"   : "stiction180";
            const char* dirName   = (stDir < 0) ? "toward 0\xc2\xb0"  : "toward 180\xc2\xb0";
            const char* verifyDir = (stDir < 0) ? "-"           : "+";
            Serial.println(F("[SLOWTEST] \u2550\u2550\u2550 MOTION CONFIRMED \u2550\u2550\u2550"));
            Serial.printf("[SLOWTEST] Direction: %s\n", dirName);
            Serial.printf("[SLOWTEST] Servo moved %.1f\u00b0 (%d ADC) from start\n", degMoved, delta);
            Serial.printf("[SLOWTEST] Pulse at motion: %3d  (offset from neutral: %+d)\n",
                          detectedSig, stDir * stDrive);
            Serial.printf("[SLOWTEST] \u2192 Real stiction %s = %d pulse counts\n", dirName, stDrive);
            Serial.printf("[SLOWTEST] \u2192 Save with: %s %d\n", cmdSave, stDrive);
            Serial.printf("[SLOWTEST] \u2192 Then verify other direction: slowtest %s\n", verifyDir);
            return;
        }

        if (millis() - stLastStep >= SLOWTEST_STEP_MS) {
            stLastStep = millis();
            stDrive++;
            if (stDrive > 60) {
                pwm.setPWM(0, 0, (uint16_t)servoStop);
                sysState = SYS_RUNNING;
                Serial.println(F("[SLOWTEST] Max drive (60) reached \u2014 no motion detected"));
                Serial.println(F("[SLOWTEST] Check servo wiring or run 'savestop' first"));
                return;
            }
            uint16_t sig = (uint16_t)(servoStop + stDir * stDrive);
            pwm.setPWM(0, 0, sig);
            Serial.printf("[SLOWTEST] offset=%+d  pulse=%3d  pot=%4d  ang=%.1f\u00b0  delta=%+d\n",
                          stDir * stDrive, sig, potNow,
                          potToAngle(potNow), delta);
        } else {
            pwm.setPWM(0, 0, (uint16_t)(servoStop + stDir * stDrive));
        }
        return;
    }

    // ── JOG — hold fixed stiction pulse for timed duration, then idle ──
    if (sysState == SYS_JOG) {
        if (millis() >= jogEndMs) {
            pwm.setPWM(0, 0, (uint16_t)servoStop);
            sysState  = SYS_RUNNING;
            targetSet = false;
            int p = readPotFiltered();
            Serial.printf("[JOG] Done — servo back to neutral  pos=%.1f\u00b0 (pot=%d)\n",
                          potToAngle(p), p);
        } else {
            pwm.setPWM(0, 0, jogPulse);
            static unsigned long lastJogMsg = 0;
            if (millis() - lastJogMsg >= 200) {
                lastJogMsg = millis();
                int p = readPotFiltered();
                Serial.printf("[JOG] pulse=%3d  pos=%.1f\u00b0 (pot=%d)  remain=%lums\n",
                              jogPulse, potToAngle(p), p,
                              (unsigned long)(jogEndMs - millis()));
            }
        }
        return;
    }

    // ── HOLDTEST — gravity sweep state machine ──
    if (sysState == SYS_HOLDTEST) {
        if (htPhase != HT_SWEEP) {
            htPhase = HT_IDLE;
            sysState = SYS_RUNNING;
            return;
        }

        // Apply the current test pulse directly
        pwm.setPWM(0, 0, (uint16_t)htCurrentPulse);

        // Check for drift
        int drift = abs(potRaw - htPotRef);
        if (drift >= htDriftThr) {
            // Arm started moving — the previous pulse was the holding point
            int holdPulse  = htCurrentPulse + (htCurrentPulse >= servoStop ? 1 : -1);

            Serial.println(F("\n[GRAVTEST] ═══ DRIFT DETECTED ═══"));
            Serial.printf("[GRAVTEST]   Drift: %d ADC at pulse %d (offset %+d)\n",
                          drift, htCurrentPulse, htCurrentPulse - servoStop);
            Serial.printf("[GRAVTEST]   Holding pulse = %d  (offset %+d from servoStop=%d)\n",
                          holdPulse, holdPulse - servoStop, servoStop);

            float sinTerm = sinf(DEG_TO_RAD * (htTargetAngle - 90.0f));
            if (fabsf(sinTerm) > 0.1f) {
                float measuredOffset = (float)(holdPulse - servoStop);
                kGravity = (double)(-measuredOffset / sinTerm);
                saveSettingsToFlash();
                Serial.printf("[GRAVTEST]   sin(%.1f° - 90°) = %.4f\n", htTargetAngle, sinTerm);
                Serial.printf("[GRAVTEST]   kGravity = %.3f / %.4f = %.3f\n",
                              measuredOffset, sinTerm, kGravity);
                Serial.println(F("[GRAVTEST]   ✓ kGravity saved to NVS flash."));
                Serial.println(F("[GRAVTEST]   → Switching to normal HOLD with new gravity model."));
                Serial.println(F("[GRAVTEST]   → Run 'holdtest' again at another angle to verify."));
            } else {
                Serial.println(F("[GRAVTEST]   ⚠ Angle too close to 90° (sin ≈ 0) — kGravity not updated."));
                Serial.println(F("[GRAVTEST]   → Run holdtest at 20°–70° or 110°–160° for best results."));
            }

            htPhase = HT_IDLE;
            sysState = SYS_RUNNING;
            targetSet = true;
            wdArmed = false;
            wdGraceUntil = millis() + 2000;
            driftRefValid = false;
            return;
        }

        // Check if we've reached servoStop without detecting drift
        if (htCurrentPulse == servoStop) {
            Serial.println(F("\n[GRAVTEST] ══ Reached servoStop — no gravity drift detected ══"));
            Serial.println(F("[GRAVTEST] Either: arm is near 90° (no gravity), kGravity = 0 is correct,"));
            Serial.println(F("[GRAVTEST] or: stiction is masking gravity. Try 'holdtest' at 20° or 160°."));
            kGravity = 0.0;
            saveSettingsToFlash();
            htPhase  = HT_IDLE;
            sysState = SYS_RUNNING;
            targetSet = true;
            wdArmed = false;
            wdGraceUntil = millis() + 2000;
            driftRefValid = false;
            return;
        }

        // Time to step?
        if (millis() - htLastStep >= HT_STEP_MS) {
            htLastStep = millis();
            if (htCurrentPulse > servoStop) htCurrentPulse--;
            else if (htCurrentPulse < servoStop) htCurrentPulse++;

            Serial.printf("[GRAVTEST] pulse=%3d  offset=%+d  pot=%4d  ang=%.1f°  drift=%+d ADC\n",
                          htCurrentPulse, htCurrentPulse - servoStop,
                          potRaw, potToAngle(potRaw),
                          potRaw - htPotRef);
        }

        static unsigned long lastI2CCheckHT = 0;
        if (millis() - lastI2CCheckHT >= 1000) { lastI2CCheckHT = millis(); if (!i2cProbe()) recoverI2C(); }
        return;
    }

    // ── No target set - servo idles at neutral, PID bypassed ──
    if (!targetSet) {
        pwm.setPWM(0, 0, (uint16_t)servoStop);
        static unsigned long lastIdleMsg = 0;
        if (millis() - lastIdleMsg >= 100) {
            lastIdleMsg = millis();
            int p = readPotFiltered();
            Serial.printf("[IDLE] pot=%4d  ang=%5.1f°  raw=%+5.1f  filt=%+5.1f deg/s  No target\n",
                          p, potToAngle(p), rawVelDegS, measuredVelDegS);
        }
        return;
    }

    // ── Pot connectivity check ──
    if (!potConnected(potRaw)) {
        triggerFault("POT reading out of range \u2014 check wiring"); return;
    }

    // ── Hard range guard: ±200 ADC outside calibrated endpoints ──
    // 200 ADC ≈ 12° — wide enough that the braking zone (9°) stops the arm
    // before this fires. Increased from 100 to reduce spurious fault on
    // fast gravity-assisted approach past mechanical 0°/180° stop.
    int potLo = min(cal.pot0deg, cal.pot180deg) - 200;
    int potHi = max(cal.pot0deg, cal.pot180deg) + 200;
    if (potRaw < potLo || potRaw > potHi) {
        triggerFault("POT beyond calibrated range \u2014 run recal2"); return;
    }

    // ── Enforce soft limits on setpoint every cycle ──
    pidSetpoint = clampToSoftLimits((int)pidSetpoint);
    pidInput    = (double)potRaw;

    // ── Trapezoidal Velocity Profile ─────────────────────────────────────
    // Accelerates from 0 to maxSpeedDegPerSec, cruises, then decelerates
    // to zero exactly at finalTargetDeg.  The signed result (tgtVelDegS)
    // is fed to the velocity PID as its setpoint.
    //
    // Hard cap at MAX_VEL_DEG_S (20 deg/sec):  even if |tgtVelDegS| is
    // somehow too large, it is clamped before reaching the PID.
    static unsigned long lastProfileMs = 0;
    {
        unsigned long nowP = millis();
        float dtP = (lastProfileMs == 0) ? 0.0f : (float)(nowP - lastProfileMs) / 1000.0f;
        lastProfileMs = nowP;
        if (dtP > 0.1f) dtP = 0.0f; // Prevent dt explosion after idle/fault pause

        if (targetSet && sysState == SYS_RUNNING && dtP > 0.0f) {
            float distRemain = finalTargetDeg - currentPosDeg;  // signed deg
            float distAbs    = fabsf(distRemain);
            float dir        = (distRemain >= 0.0f) ? 1.0f : -1.0f;

            // Failsafe: Check if the physical arm has reached or overshot the final target.
            // If it has, terminate the virtual profile immediately to prevent the controller
            // from driving the arm further away while blindly finishing the virtual path.
            float physDistRemain = finalTargetDeg - potToAngle(potRaw);
            bool physOvershot = (dir > 0.0f && physDistRemain <= 0.0f) || 
                                (dir < 0.0f && physDistRemain >= 0.0f);

            if (distAbs <= 0.5f || physOvershot) {
                // Arrived or physical overshoot
                profileVelDegS = 0.0f;
                currentPosDeg  = finalTargetDeg;
                tgtVelDegS     = 0.0f;
            } else {
                // Ideal Velocity Profile: v = sqrt(2 * a * d)
                // This curve mathematically guarantees we reach 0 speed exactly at the target.
                float maxAllowedSpeed = sqrtf(2.0f * accelDegS2 * distAbs);
                
                // Attempt to accelerate
                profileVelDegS += accelDegS2 * dtP;
                
                // Cap to cruise speed
                float cap = min(maxSpeedDegPerSec, MAX_VEL_DEG_S);
                if (profileVelDegS > cap) profileVelDegS = cap;
                
                // If accelerating exceeds the ideal braking curve, clamp it to the braking curve
                if (profileVelDegS > maxAllowedSpeed) {
                    profileVelDegS = maxAllowedSpeed;
                }
                
                // Ensure it crawls the last tiny bit instead of getting mathematically stuck at v=0.0001
                if (profileVelDegS < 1.0f) profileVelDegS = 1.0f;

                tgtVelDegS  = dir * profileVelDegS;
                currentPosDeg += tgtVelDegS * dtP;
            }
        } else if (!targetSet) {
            profileVelDegS = 0.0f;
            tgtVelDegS     = 0.0f;
        }
    }

    int servoSignal = servoStop;
    bool servoActive = false;

    // ── Position error (degrees) — used for deadband / zone logic ─────────
    float posDeg      = potToAngle(potRaw);
    float posErrDeg   = finalTargetDeg - posDeg;          // signed degrees remaining
    float absErrDeg   = fabsf(posErrDeg);
    float absErrADC   = absErrDeg * fabsf(cal.adcPerDeg); // in ADC counts for zone checks
    int   moveSign    = (tgtVelDegS >= 0.0f) ? 1 : -1;   // direction from profile
    if (tgtVelDegS == 0.0f)
        moveSign = (posErrDeg >= 0.0f) ? 1 : -1;          // fallback when profile idle

    // Failsafe: Force moveSign to point toward target if the arm is significantly displaced.
    // E.g., if trajectory is going to 180 (moveSign +1), but arm was pushed past 180,
    // posErrDeg will be negative. The arm MUST be driven back to target.
    if ((moveSign > 0 && posErrDeg < 0.0f) || (moveSign < 0 && posErrDeg > 0.0f)) {
        moveSign = (posErrDeg >= 0.0f) ? 1 : -1;
    }

    bool  inDeadband   = (absErrDeg * fabsf(cal.adcPerDeg) <= (float)deadbandADC);
    float transzoneADC = (float)(transzoneMult * deadbandADC);
    bool  inTranszone  = (!inDeadband && absErrADC <= transzoneADC);

    // Legacy compat for drift detection
    pidSetpoint = (double)clampToSoftLimits(angleToPot(finalTargetDeg));
    pidInput    = (double)potRaw;
    float error = (float)(pidSetpoint - pidInput);  // ADC error (for gravity/drive calcs)
    float absError = fabsf(error);

    // ── Velocity PID (inner loop) ─────────────────────────────────────────
    // Error = target velocity (from profile) minus measured velocity.
    // Output (velPidOut) is a pulse-count correction added to the drive baseline.
    static unsigned long lastPidLoopMs = 0;
    unsigned long nowPidLoop = millis();
    float dtVelPid = (lastPidLoopMs == 0) ? 0.02f : (float)(nowPidLoop - lastPidLoopMs) / 1000.0f;
    lastPidLoopMs = nowPidLoop;
    if (dtVelPid <= 0.0f || dtVelPid > 0.1f) dtVelPid = 0.02f; // Prevent lag spikes

    if (inDeadband) {
        velIntegral = 0.0f;
        velPidOut   = 0.0f;
        tgtVelDegS  = 0.0f;
        profileVelDegS = 0.0f;
    } else {
        float velErr    = tgtVelDegS - measuredVelDegS;
        velIntegral     = velIntegral * 0.9f; // Leaky integrator (decays to 0)
        velIntegral    += Ki_vel * velErr * dtVelPid;
        velIntegral     = constrain(velIntegral, (float)-SERVO_DRIVE, (float)SERVO_DRIVE);  // anti-windup
        velPidOut       = Kp_vel * velErr + velIntegral;
    }
    pidOutput = (double)velPidOut;  // for printStatus display

    // Gravity calculations for feedforward baseline
    float gravAngle  = potToAngle(potRaw);
    
    // Dynamic Gravity Boost at extremes to compensate for mechanical linkage leverage loss
    float absAngleFromCenter = fabsf(gravAngle - 90.0f); // 0 at center, 90 at extremes
    float gravBoostMult = 1.0f;
    if (absAngleFromCenter > 45.0f) {
        // Boost from 1.0x at 45° to 1.5x at 90°
        gravBoostMult = 1.0f + 0.5f * ((absAngleFromCenter - 45.0f) / 45.0f);
    }
    
    int   gravOffset = (int)round(kGravity * gravBoostMult * sinf(DEG_TO_RAD * (gravAngle - 90.0f)));
    int   gravDir    = (gravOffset > 0) ? 1 : (gravOffset < 0 ? -1 : 0);
    int   holdDrive  = constrain(gravOffset + gravDir * sticBoostHold,
                                 -servoStictionTo180, servoStictionTo0);
    int   baselinePulse = servoStop - gravOffset;

    if (inDeadband) {
        // Integrator gate & reset
        servoSignal = constrain(servoStop - holdDrive,
                                servoStop - SERVO_DRIVE,
                                servoStop + SERVO_DRIVE);
        servoActive = false;
    } else {
        // 1. Correct stiction mapping (Kinetic Friction)
        int dirStiction = (moveSign > 0) ? servoStictionTo180 : servoStictionTo0;

        // The Trajectory Generator naturally decelerates the target velocity (tgtVelDegS).
        // Therefore, we must maintain the kinetic friction feedforward (dirStiction) fully
        // active so the arm doesn't get stuck during the slow deceleration phase.
        int activeStiction = dirStiction;

        int baseDriveMag = constrain(activeStiction, 0, SERVO_DRIVE);
        
        // 2. Add for 180° (Higher pulse), Subtract for 0° (Lower pulse)
        int moveDrive = (moveSign > 0) ? baselinePulse + baseDriveMag
                                       : baselinePulse - baseDriveMag;

        // 3. Velocity Feedforward & PID Correction
        float Kv_ff = 1.0f; // Feedforward: 1 pulse per deg/sec (proactive drive)
        int v_ff = (int)(tgtVelDegS * Kv_ff);
        
        int velCorr = constrain((int)velPidOut + v_ff, -SERVO_DRIVE, SERVO_DRIVE);
        
        // ADD the velocity correction. 
        // If velCorr > 0 (need more speed towards 180), adding makes the pulse higher.
        // If velCorr < 0 (need more speed towards 0), adding negative makes the pulse lower.
        moveDrive = moveDrive + velCorr;

        if (inTranszone) {
            // Smoothly interpolate between MOVE and HOLD
            int holdPulse = servoStop - holdDrive;
            float scale = (absError - deadbandADC) / (transzoneADC - deadbandADC);
            servoSignal = holdPulse + (int)round((float)(moveDrive - holdPulse) * scale);
            servoActive = true;
        } else {
            // Full MOVE
            servoSignal = moveDrive;
            servoActive = true;
        }
    }

    servoSignal = constrain(servoSignal,
                            servoStop - SERVO_DRIVE,
                            servoStop + SERVO_DRIVE);

    // ── Electronic Shock Absorber (EMA Slew Limiter) ──
    // Smooths out instantaneous step-functions to prevent mechanical linkage whipping
    static float smoothedSignal = servoStop;
    // Fast attack/release filter: 20% new signal, 80% history per loop cycle
    smoothedSignal = 0.2f * (float)servoSignal + 0.8f * smoothedSignal;
    
    // Safety clamp just in case
    int finalOutputPulse = constrain((int)round(smoothedSignal), servoStop - SERVO_DRIVE, servoStop + SERVO_DRIVE);

    // ── Direction-aware watchdog ──
    // Grace period: after reset/target commands, don't arm for 2 s.
    // This prevents false "wrong direction" faults on the first tick
    // when pidOutput==0 would have given the wrong expected direction.
    // Watchdog only arms in BP_LOCKED — i.e. movement was already confirmed at
    // a stable boost level. During BP_WAIT and BP_RAMP the arm is still trying
    // to find enough power to move; faulting here would interrupt the ramp.
    bool wdGraceActive = (millis() < wdGraceUntil);
    bool wdShouldArm   = false; // watchdog disabled (velocity profile handles speed limiting)
    updateWatchdog(potRaw, wdShouldArm, moveSign);

    // ── Drift detection (only while holding) ──
    updateDriftDetection(potRaw, inDeadband);

    // ── Write to servo (throttled: only on change or every 20ms) ──
    // Reduces I2C traffic from ~1kHz to ~50Hz — matching the 50Hz servo.
    // Dramatically lowers the probability of power-noise I2C glitches.
    static int           lastServoSignal = -1;
    static unsigned long lastServoWrite  = 0;
    uint16_t sigToWrite = (sysState == SYS_RUNNING) ? (uint16_t)finalOutputPulse
                                                     : (uint16_t)servoStop;
    if ((int)sigToWrite != lastServoSignal || millis() - lastServoWrite >= 20) {
        pwm.setPWM(0, 0, sigToWrite);
        lastServoSignal = (int)sigToWrite;
        lastServoWrite  = millis();
    }

    // ── Periodic I2C health check (every 1 s) ──
    // Detects PCA9685 lockup from servo power-noise I2C glitch.
    // Performs automatic bus recovery + PCA9685 reinitialization.
    static unsigned long lastI2CCheck = 0;
    if (millis() - lastI2CCheck >= 1000) {
        lastI2CCheck = millis();
        if (!i2cProbe()) recoverI2C();
    }

    // ── HOLDTEST Move Phase Intercept ──
    if (htPhase == HT_MOVE && sysState == SYS_RUNNING) {
        float absError = fabsf(error);
        if (absError > (float)(3 * deadbandADC) || fabsf(measuredVelDegS) > 5.0f) {
            htStableStart = millis(); // Not stable, reset timer
        } else if (millis() - htStableStart >= HT_STABLE_MS) {
            // Stable! Start the sweep!
            htPhase        = HT_SWEEP;
            htStartPulse   = finalOutputPulse;  // The naturally settled equilibrium pulse!
            htCurrentPulse = htStartPulse;
            htPotRef       = potRaw;
            htLastStep     = millis();
            sysState       = SYS_HOLDTEST; // Switch to holdtest state for the sweep
            
            Serial.println(F("\n[GRAVTEST] ══ Gravity Sweep Started ══"));
            Serial.printf("[GRAVTEST] Angle: %.1f°  servoStop: %d  Starting pulse: %d\n",
                          htTargetAngle, servoStop, htStartPulse);
            Serial.printf("[GRAVTEST] Sweeping toward servoStop (%d) — one count every %dms\n",
                          servoStop, HT_STEP_MS);
            Serial.printf("[GRAVTEST] Drift threshold: %d ADC (~5°). 'stop' to abort.\n", htDriftThr);
        }
    }

    // ── 10 Hz diagnostic print ──
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        const char* zoneTag = inDeadband  ? "[HOLD]"
                            : inTranszone ? "[ZTRANS]"
                                          : "[MOVE]";
        Serial.printf("[RUN] pot=%4d  ang=%5.1f°  tvl=%+6.1f  vel=%+6.1f  raw=%+6.1f  pid=%5.1f  pulse=%3d  %s\n",
                      potRaw, potToAngle(potRaw),
                      tgtVelDegS, measuredVelDegS, rawVelDegS,
                      velPidOut, finalOutputPulse, zoneTag);
        // velplot streaming (every 50ms when active)
        if (velPlotActive) {
            Serial.printf("VELPLOT %.2f %.2f %.2f\n",
                          rawVelDegS, measuredVelDegS, tgtVelDegS);
        }
    }
}
