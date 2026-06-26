#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>

#define NVS_KEY_SERVO_STOP "srv_stop"
#define NVS_KEY_DEADBAND "dband"
#define NVS_KEY_STICTION_TO0 "stic_to0"
#define NVS_KEY_STICTION_TO180 "stic_to180"
#define NVS_KEY_KGRAVITY "kgrav"
#define NVS_KEY_TRANSZONE "tzone"
#define NVS_KEY_SBHOLD "sbhold"
#define NVS_KEY_VELCUT "vcut"
#define NVS_KEY_0DEG "0deg"
#define NVS_KEY_180DEG "180deg"

#define SERVO_DRIVE 120
#define HT_STEP_MS 500
#define HT_STABLE_MS 1000
#define SLOWTEST_STEP_MS 600


extern bool i2cProbe();
extern void recoverI2C();

// System State Enums

#define WATCHDOG_WINDOW_MS      800
#define WATCHDOG_MIN_DELTA      20
#define WATCHDOG_DIR_TOLERANCE  5
#define TARGET_DECEL_ADC_DEFAULT 400
#define BRAKE_ZONE_ADC_DEFAULT 150
#define DRIFT_DETECT_THRESHOLD  30
#define DRIFT_DETECT_WINDOW_MS  3000
#define DRIFT_MAX_AUTO_CORRECT  200
#define RECAL_STABILITY_MS      2000
#define RECAL_VARIANCE_MAX      15
#define RECAL_MAX_SHIFT_ADC     500
#define RECAL_MIN_SPAN_ADC      500
#define POT_BROKEN_LOW          50
#define POT_BROKEN_HIGH         4050
#define BOOST_WAIT_MS   800
#define BOOST_RAMP_MS  1200
#define BOOST_MOVE_THR  50


#define SERVO_STOP_DEFAULT    369
#define DEADBAND_ADC_DEFAULT  80
#define SERVO_STICTION_DEFAULT 26
#define DEFAULT_POT_0DEG     3186
#define DEFAULT_POT_180DEG   319



    // Constants
#define NUM_CHANNELS 16

enum SystemState {
    SYS_RUNNING,        // normal operation
    SYS_DRIFT_WARN,     // drift detected
    SYS_RECAL_ACTIVE,   // recalibration in progress
    SYS_SLOWTEST,       // stiction sweep
    SYS_JOG,            // timed jog
    SYS_FAULT,          // latched fault
    SYS_HOLDTEST        // gravity sweep
};

enum HoldTestPhase { HT_IDLE, HT_MOVE, HT_SWEEP };
enum RecalPhase { RECAL_IDLE, RECAL_COLLECT_0DEG, RECAL_COLLECT_180DEG };
enum BoostPhase { BP_WAIT, BP_RAMP, BP_LOCKED, BP_TIMEOUT };

struct CalData {
    int   pot0deg;
    int   pot180deg;
    int   softLimitLow;
    int   softLimitHigh;
    float adcPerDeg;
    int   marginADC;
};

// Inline PID Class
class PID {
public:
    double   Kp, Ki, Kd;
    double   outMin, outMax;
    double   iMin, iMax;
    uint32_t sampleMs;

    void init(double kp, double ki, double kd, double oMin, double oMax, double iLo, double iHi, uint32_t ms);
    double compute(double input, double setpoint);
    void reset(double currentInput);
    void setGains(double kp, double ki, double kd);
    void resetIntegral();

private:
    double   _integral  = 0;
    double   _lastInput = 0;
    double   _lastOut   = 0;
    uint32_t _lastTime  = 0;
};

// The modular Axis class!
class Axis {
public:
    // Hardware Mapping
    uint8_t id;
    int potPin;
    uint8_t pwmChannel;
    String nvsNamespace;

    // References to shared hardware
    Adafruit_PWMServoDriver* pwm;
    Preferences* prefs;

    // --- State Variables (Moved from globals) ---
    CalData cal;
    SystemState sysState = SYS_RUNNING;
    RecalPhase recalPhase = RECAL_IDLE;

    // Trajectory
    float maxSpeedDegPerSec = 15.0f;
    float accelDegS2 = 30.0f;
    float finalTargetDeg = 90.0f;
    float currentPosDeg = 90.0f;
    float profileVelDegS = 0.0f;
    float tgtVelDegS = 0.0f;

    // Velocity Filter & PID
    float measuredVelDegS = 0.0f;
    float rawVelDegS = 0.0f;
    float Kv_alpha = 0.15f;
    int lastPotForVel = -1;
    unsigned long lastVelTimeMs = 0;
    float velIntegral = 0.0f;
    float velPidOut = 0.0f;
    float Kp_vel = 0.4f;
    float Ki_vel = 0.02f;
    // Time-constant for leaky integrator decay. Replaces the old per-tick 0.9f
    // factor so the memory length is independent of loop speed.
    float velIntegralTauSec = 0.5f;
    // Smoothed velocity setpoint fed to PI — keeps smoothing upstream of the
    // closed loop so the post-PI EMA filter is no longer needed.
    float smoothedTgtVelDegS = 0.0f;
    float velSmoothAlpha = 0.3f;   // how snappy the velocity reference is
    double pidSetpoint = 0;
    double pidInput = 0;
    double pidOutput = 0;
    bool velPlotActive = false;
    bool muteIdle = true;
    PID controller;

    // NVS Tunables
    int servoStop = 369;
    int servoStictionTo0 = 26;
    int servoStictionTo180 = 26;
    int deadbandADC = 80;
    int brakeZoneADC = 150;
    int targetDecelADC = 400;
    double kGravity = 0.0;
    int transzoneMult = 2;
    int sticBoostHold = 0;
    int velCutThr = 30;

    // Target State
    float targetAngleDeg = 90.0f;
    float targetTravelDeg = 0.0f;
    bool targetSet = false;

    // Filter
    static const int FILTER_SAMPLES = 10;
    int filterBuf[FILTER_SAMPLES] = {0};
    int filterIdx = 0;
    long filterSum = 0;
    bool filterFull = false;

    // Watchdog
    bool wdArmed = false;
    int wdStartPot = 0;
    unsigned long wdStartTime = 0;
    int wdExpectedDir = 0;
    unsigned long wdGraceUntil = 0;

    // Drift Detection
    int driftRefPot = 0;
    unsigned long driftRefTime = 0;
    bool driftRefValid = false;

    // Recal State
    int recalMode = 0;
    unsigned long recalStartMs = 0;
    long recalAccum = 0;
    int recalSamples = 0;
    int recalMin = 9999;
    int recalMax = 0;
    int pendingPot0 = -1;

    // Live Extremes
    int observedPotMin = 9999;
    int observedPotMax = 0;

    // Slowtest State
    int stDrive = 0;
    int stDir = 1;
    int stStartPot = 0;
    unsigned long stLastStep = 0;
    int stMotionThrADC = 80;

    // Jog State
    unsigned long jogEndMs = 0;
    uint16_t jogPulse = 0;

    // Boost State
    int sticBoost = 40;
    int sticRetry = 0;
    BoostPhase boostPhase = BP_WAIT;
    unsigned long boostPhaseStart = 0;
    int boostLocked = 0;
    int boostPotRef = 0;
    int boostPhaseDirRef = 0;

    // Holdtest State
    HoldTestPhase htPhase = HT_IDLE;
    float htTargetAngle = 0.0f;
    int htStartPulse = 0;
    int htCurrentPulse = 0;
    int htPotRef = 0;
    int htDriftThr = 80;
    unsigned long htStableStart = 0;
    unsigned long htLastStep = 0;

    // I2C Throttling
    int lastServoSignal = -1;
    unsigned long lastServoWrite = 0;

    
#define SERVO_STOP_DEFAULT    369
#define DEADBAND_ADC_DEFAULT  80
#define SERVO_STICTION_DEFAULT 26
#define DEFAULT_POT_0DEG     3186
#define DEFAULT_POT_180DEG   319


    // State Tracking for loops
    unsigned long lastFaultMsg = 0;
    unsigned long lastDriftMsg = 0;
    unsigned long lastJogMsg = 0;
    unsigned long lastI2CCheckHT = 0;
    unsigned long lastIdleMsg = 0;
    unsigned long lastProfileMs = 0;
    unsigned long lastPidLoopMs = 0;
    // NOTE: smoothedSignal (post-PI EMA) is intentionally removed — smoothing
    // is now applied upstream on the velocity reference (smoothedTgtVelDegS).
    // If a local variable is ever needed temporarily, declare it inline.

    // dt histogram buckets: 0-5ms, 5-10ms, ..., >35ms (8 buckets, 5ms wide)
    static const int DT_BUCKETS = 8;
    uint32_t dtHistogram[DT_BUCKETS] = {0};
    unsigned long lastI2CCheck = 0;
    unsigned long lastPrint = 0;
    unsigned long lastRecalPrint = 0;

    // Constants
    const float MAX_VEL_DEG_S = 20.0f;
    const int SAFETY_MARGIN_DEG = 3;

    // --- Methods ---
    Axis(uint8_t id, int pin, uint8_t channel, String nvs, Adafruit_PWMServoDriver* pwmDriver, Preferences* preferences);

    void init();
    void update();
    
    // NVS
    void saveSettingsToFlash();
    void loadSettingsFromFlash();
    void saveCalToFlash();
    void loadCalFromFlash();
    
    // Core logic
    int readPotFiltered();
    float potToAngle(int potValue);
    int angleToPot(float angle);
    int clampToSoftLimits(int rawADC);
    void recomputeCalibration(int pot0 = -1, int pot180 = -1);
    bool potConnected(int val);
    
    // Command Helpers
    struct ReachResult { bool ok; float clampedAngle; int clampedPot; const char* reason; };
    ReachResult checkReachability(float targetDeg);
    void triggerFault(const char* reason);
    void triggerDriftWarn(int driftADC);
    
    void updateVelocityMeasurement(int potRaw);
    void updateDriftDetection(int potRaw, bool inDeadband);
    void updateWatchdog(int potNow, bool shouldBeActive, int pidSign);
    
    // Commands
    void startRecal(int mode);
    void abortRecal(const char* reason = nullptr);
    void updateRecal(int potRaw);
    void reset();
    void stop();
    void setTarget(float angle);
    void setServoStop(int pulse);
    void startSlowtest(int dir);

    // Diagnostic: dump and clear the dt histogram over serial.
    void printDtHistogram();
};

#endif
