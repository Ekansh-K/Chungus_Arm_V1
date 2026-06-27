#include "Axis.h"
#include <math.h>

void PID::init(double kp, double ki, double kd, double oMin, double oMax, double iLo, double iHi, uint32_t ms) {
    Kp = kp; Ki = ki; Kd = kd;
    outMin = oMin; outMax = oMax;
    iMin = iLo;    iMax = iHi;
    sampleMs = ms;
}

double PID::compute(double input, double setpoint) {
    uint32_t now = millis();
    if (now - _lastTime < sampleMs) return _lastOut;
    double dt = (now - _lastTime) / 1000.0;
    _lastTime = now;
    double error  = setpoint - input;
    double dInput = input - _lastInput;
    _integral += Ki * error * dt;
    _integral  = constrain(_integral, iMin, iMax);
    double p = Kp * error;
    double d = (dt > 0.001) ? (-Kd * dInput / dt) : 0.0;
    double out = p + _integral + d;
    out = constrain(out, outMin, outMax);
    _lastInput = input;
    _lastOut   = out;
    return out;
}

void PID::reset(double currentInput) {
    _integral  = 0;
    _lastInput = currentInput;
    _lastOut   = 0;
    _lastTime  = millis();
}

void PID::setGains(double kp, double ki, double kd) {
    Kp = kp; Ki = ki; Kd = kd;
}

void PID::resetIntegral() {
    _integral = 0;
}

Axis::Axis(uint8_t id, int pin, uint8_t channel, String nvs, Adafruit_PWMServoDriver* pwmDriver, Preferences* preferences) {
    this->id = id;
    this->potPin = pin;
    this->pwmChannel = channel;
    this->nvsNamespace = nvs;
    this->pwm = pwmDriver;
    this->prefs = preferences;
}

void Axis::saveSettingsToFlash() {
    prefs->begin(nvsNamespace.c_str(), false);
    prefs->putInt(NVS_KEY_SERVO_STOP,     servoStop);
    prefs->putInt(NVS_KEY_DEADBAND,       deadbandADC);
    prefs->putInt(NVS_KEY_STICTION_TO0,   servoStictionTo0);
    prefs->putInt(NVS_KEY_STICTION_TO180, servoStictionTo180);
    prefs->putDouble(NVS_KEY_KGRAVITY,    kGravity);
    prefs->putInt(NVS_KEY_TRANSZONE,      transzoneMult);
    prefs->putInt(NVS_KEY_SBHOLD,         sticBoostHold);
    prefs->putInt(NVS_KEY_VELCUT,         velCutThr);
    prefs->end();
    Serial.printf("[S%d][CFG] Settings saved to NVS\n", this->id);
}

void Axis::loadSettingsFromFlash() {
    prefs->begin(nvsNamespace.c_str(), false);
    int    s   = prefs->getInt(NVS_KEY_SERVO_STOP,     SERVO_STOP_DEFAULT);
    int    d   = prefs->getInt(NVS_KEY_DEADBAND,       DEADBAND_ADC_DEFAULT);
    int    s0  = prefs->getInt(NVS_KEY_STICTION_TO0,   SERVO_STICTION_DEFAULT);
    int    s18 = prefs->getInt(NVS_KEY_STICTION_TO180, SERVO_STICTION_DEFAULT);
    double kg  = prefs->getDouble(NVS_KEY_KGRAVITY,    0.0);
    int    tz  = prefs->getInt(NVS_KEY_TRANSZONE,      2);
    int    sbh = prefs->getInt(NVS_KEY_SBHOLD,         0);
    int    vct = prefs->getInt(NVS_KEY_VELCUT,         30);
    prefs->end();

    // Validate ranges before accepting (sanitize corrupted values like 50)
    servoStop          = (s   >= 200 && s   <= 500) ? s   : SERVO_STOP_DEFAULT;
    deadbandADC        = (d   >= 2   && d   <= 200) ? d   : DEADBAND_ADC_DEFAULT;
    servoStictionTo0   = (s0  >= 1   && s0  <= 80)  ? s0  : SERVO_STICTION_DEFAULT;
    servoStictionTo180 = (s18 >= 1   && s18 <= 80)  ? s18 : SERVO_STICTION_DEFAULT;
    kGravity           = (kg  >= -200.0 && kg <= 200.0) ? kg : 0.0;
    transzoneMult      = (tz  >= 1  && tz  <= 6)   ? tz  : 3;  // Tuned default: 3 (was 2)
    sticBoostHold      = (sbh >= 0  && sbh <= 20)  ? sbh : 0;
    velCutThr          = (vct >= 10 && vct <= 100)  ? vct : 30;

    Serial.printf("[CFG] servoStop=%d  stiction(to0)=%d  stiction(to180)=%d  deadband=%d ADC\n",
                  servoStop, servoStictionTo0, servoStictionTo180, deadbandADC);
    Serial.printf("[CFG] kGravity=%.3f  transzone=%d  sticBoostHold=%d  velCutThr=%d\n",
                  kGravity, transzoneMult, sticBoostHold, velCutThr);
}

void Axis::recomputeCalibration(int new0deg, int new180deg) {
    cal.pot0deg   = new0deg;
    cal.pot180deg = new180deg;
    // adcPerDeg is negative (inverted pot: high ADC = 0°, low ADC = 180°)
    cal.adcPerDeg = (float)(new180deg - new0deg) / 180.0f;
    cal.marginADC = (int)(SAFETY_MARGIN_DEG * fabsf(cal.adcPerDeg));

        cal.softLimitHigh = angleToPot(SAFETY_MARGIN_DEG);
    cal.softLimitLow  = angleToPot(180.0f - SAFETY_MARGIN_DEG);

    observedPotMin = new180deg;
    observedPotMax = new0deg;
    driftRefValid  = false;

    Serial.println(F("\n[CAL] ══════ Calibration Updated ══════"));
    Serial.printf("[S%d][CAL]  0°   → %d ADC\n", this->id,          cal.pot0deg);
    Serial.printf("[S%d][CAL]  180° → %d ADC\n", this->id,          cal.pot180deg);
    Serial.printf("[S%d][CAL]  Scale: %.3f ADC/deg\n", this->id,    cal.adcPerDeg);
    Serial.printf("[CAL]  Soft limits: ADC [%d – %d]  (%.1f° – %.1f°)\n",
                  cal.softLimitLow, cal.softLimitHigh,
                  SAFETY_MARGIN_DEG, 180.0f - SAFETY_MARGIN_DEG);
    Serial.printf("[S%d][CAL] ══════════════════════════════════\n\n", this->id);
}

void Axis::update() {
    int potRaw = readPotFiltered();
    updateVelocityMeasurement(potRaw);

    if (sysState == SYS_RECAL_ACTIVE) {
        updateRecal(potRaw);
        pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
        return;
    }

    if (sysState == SYS_FAULT) {
        pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
        
        if (millis() - lastFaultMsg >= 100) {
            lastFaultMsg = millis();
            int p = readPotFiltered();
            Serial.printf("[S%d][FAULT] pot=%4d  ang=%5.1f°  Send 'reset'\n", this->id, p, potToAngle(p));
        }
        return;
    }

    if (sysState == SYS_DRIFT_WARN) {
        pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
        
        if (millis() - lastDriftMsg >= 100) {
            lastDriftMsg = millis();
            int p = readPotFiltered();
            Serial.printf("[S%d][DRIFT] pot=%4d  ang=%5.1f°  Paused. Send 'recal2' or 'resume'\n", this->id, p, potToAngle(p));
        }
        return;
    }

    // ── SLOWTEST — sweep drive 1 count at a time and detect stiction ──
    if (sysState == SYS_SLOWTEST) {
        int potNow = readPotFiltered();
        int delta  = abs(potNow - stStartPot);

        if (delta >= stMotionThrADC) {
            // Pot moved ≥5° — confirmed real motion, not noise
            pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
            sysState = SYS_RUNNING;
            int detectedSig = servoStop + stDir * stDrive;
            float degMoved  = fabsf(cal.adcPerDeg) > 0.01f
                              ? delta / fabsf(cal.adcPerDeg) : 0;
            const char* cmdSave   = (stDir < 0) ? "stiction0"   : "stiction180";
            const char* dirName   = (stDir < 0) ? "toward 0\xc2\xb0"  : "toward 180\xc2\xb0";
            const char* verifyDir = (stDir < 0) ? "-"           : "+";
            Serial.printf("[S%d][SLOWTEST] \u2550\u2550\u2550 MOTION CONFIRMED \u2550\u2550\u2550\n", this->id);
            Serial.printf("[S%d][SLOWTEST] Direction: %s\n", this->id, dirName);
            Serial.printf("[S%d][SLOWTEST] Servo moved %.1f\u00b0 (%d ADC) from start\n", this->id, degMoved, delta);
            Serial.printf("[SLOWTEST] Pulse at motion: %3d  (offset from neutral: %+d)\n",
                          detectedSig, stDir * stDrive);
            Serial.printf("[S%d][SLOWTEST] \u2192 Real stiction %s = %d pulse counts\n", this->id, dirName, stDrive);
            Serial.printf("[S%d][SLOWTEST] \u2192 Save with: %s %d\n", this->id, cmdSave, stDrive);
            Serial.printf("[S%d][SLOWTEST] \u2192 Then verify other direction: slowtest %s\n", this->id, verifyDir);
            return;
        }

        if (millis() - stLastStep >= SLOWTEST_STEP_MS) {
            stLastStep = millis();
            stDrive++;
            if (stDrive > 60) {
                pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
                sysState = SYS_RUNNING;
                Serial.printf("[S%d][SLOWTEST] Max drive (60) reached \u2014 no motion detected\n", this->id);
                Serial.printf("[S%d][SLOWTEST] Check servo wiring or run 'savestop' first\n", this->id);
                return;
            }
            uint16_t sig = (uint16_t)(servoStop + stDir * stDrive);
            pwm->setPWM(pwmChannel, 0, sig);
            Serial.printf("[SLOWTEST] offset=%+d  pulse=%3d  pot=%4d  ang=%.1f\u00b0  delta=%+d\n",
                          stDir * stDrive, sig, potNow,
                          potToAngle(potNow), delta);
        } else {
            pwm->setPWM(pwmChannel, 0, (uint16_t)(servoStop + stDir * stDrive));
        }
        return;
    }

    // ── JOG — hold fixed stiction pulse for timed duration, then idle ──
    if (sysState == SYS_JOG) {
        if (millis() >= jogEndMs) {
            pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
            sysState  = SYS_RUNNING;
            targetSet = false;
            int p = readPotFiltered();
            Serial.printf("[JOG] Done — servo back to neutral  pos=%.1f\u00b0 (pot=%d)\n",
                          potToAngle(p), p);
        } else {
            pwm->setPWM(pwmChannel, 0, jogPulse);
            
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
        pwm->setPWM(pwmChannel, 0, (uint16_t)htCurrentPulse);

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
                Serial.printf("[S%d][GRAVTEST]   sin(%.1f° - 90°) = %.4f\n", this->id, htTargetAngle, sinTerm);
                Serial.printf("[GRAVTEST]   kGravity = %.3f / %.4f = %.3f\n",
                              measuredOffset, sinTerm, kGravity);
                Serial.printf("[S%d][GRAVTEST]   ✓ kGravity saved to NVS flash.\n", this->id);
                Serial.printf("[S%d][GRAVTEST]   → Switching to normal HOLD with new gravity model.\n", this->id);
                Serial.printf("[S%d][GRAVTEST]   → Run 'holdtest' again at another angle to verify.\n", this->id);
            } else {
                Serial.printf("[S%d][GRAVTEST]   ⚠ Angle too close to 90° (sin ≈ 0) — kGravity not updated.\n", this->id);
                Serial.printf("[S%d][GRAVTEST]   → Run holdtest at 20°–70° or 110°–160° for best results.\n", this->id);
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
            Serial.printf("[S%d][GRAVTEST] Either: arm is near 90° (no gravity), kGravity = 0 is correct,\n", this->id);
            Serial.printf("[S%d][GRAVTEST] or: stiction is masking gravity. Try 'holdtest' at 20° or 160°.\n", this->id);
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

        
        if (millis() - lastI2CCheckHT >= 1000) { lastI2CCheckHT = millis(); if (!i2cProbe()) recoverI2C(); }
        return;
    }

    // ── No target set - servo idles at neutral, PID bypassed ──
    if (!targetSet) {
        pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
        
        if (!muteIdle && (millis() - lastIdleMsg >= 2000)) {
            lastIdleMsg = millis();
            int p = readPotFiltered();
            Serial.printf("[S%d][IDLE] pot=%4d  ang=%5.1f°  raw=%+5.1f  filt=%+5.1f deg/s  No target\n",
                          this->id, p, potToAngle(p), rawVelDegS, measuredVelDegS);
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
    
    unsigned long nowPidLoop = millis();
    float dtVelPid = (lastPidLoopMs == 0) ? 0.02f : (float)(nowPidLoop - lastPidLoopMs) / 1000.0f;
    lastPidLoopMs = nowPidLoop;
    if (dtVelPid <= 0.0f || dtVelPid > 0.1f) dtVelPid = 0.02f; // Prevent lag spikes

    // ── Fix 2: Record dt into histogram for loop-timing diagnostics ──────────
    // Buckets: 0-5ms, 5-10ms, 10-15ms, ..., >35ms (8 x 5ms-wide slots)
    {
        int bucket = (int)(dtVelPid * 1000.0f / 5.0f);
        if (bucket < 0) bucket = 0;
        if (bucket >= DT_BUCKETS) bucket = DT_BUCKETS - 1;
        dtHistogram[bucket]++;
    }

    if (inDeadband) {
        // ── Fix 3: Smooth decay instead of hard-zero ────────────────────────
        // Hard-zeroing velIntegral / velPidOut on deadband entry causes a
        // discontinuity: the next transzone entry starts from a cold integrator
        // even though the system may still have nonzero velocity error "memory".
        // Decaying with a short tau (0.15 s) keeps the state continuous while
        // still reaching zero well before re-entry drives become significant.
        float dbDecay       = expf(-dtVelPid / 0.15f);
        velIntegral        *= dbDecay;
        velPidOut          *= dbDecay;
        smoothedTgtVelDegS *= dbDecay;
        tgtVelDegS          = 0.0f;
        profileVelDegS      = 0.0f;
    } else {
        // ── Fix 4 (part A): Smooth the velocity reference UPSTREAM ───────────
        // Applying an EMA to the PI *output* stacks two dynamic elements in
        // series (PI + lag filter), risking beat-frequency oscillation.
        // Filtering the *setpoint* tgtVelDegS instead gives the PI a smooth
        // reference to track, which naturally shapes the output without the
        // double-lag problem.
        smoothedTgtVelDegS = velSmoothAlpha * tgtVelDegS
                           + (1.0f - velSmoothAlpha) * smoothedTgtVelDegS;

        float velErr    = smoothedTgtVelDegS - measuredVelDegS;

        // ── Fix 1: Time-based leaky integrator decay ─────────────────────────
        // Old: velIntegral * 0.9f per tick — effective tau = dtVelPid / 0.105.
        // If dtVelPid bounces 5–40 ms the integrator memory swings 8×.
        // New: expf(-dt/tau) gives exactly velIntegralTauSec time-constant
        // regardless of loop jitter.
        float decayFactor = expf(-dtVelPid / velIntegralTauSec);
        velIntegral       = velIntegral * decayFactor;
        velIntegral      += Ki_vel * velErr * dtVelPid;
        velIntegral       = constrain(velIntegral, -50.0f, 50.0f);  // Tuned anti-windup: was ±SERVO_DRIVE(120), tightened to ±50 to prevent catastrophic overshoot on long gravity-assisted slews
        velPidOut         = Kp_vel * velErr + velIntegral;
    }
    pidOutput = (double)velPidOut;  // for printStatus display

    // Gravity calculations for feedforward baseline
    float gravAngle  = potToAngle(potRaw);
    
    // ── kGravity gravity feedforward ──────────────────────────────────────────
    // TODO (Step 1): Replace kGravity * gravBoostMult * sin(angle) with a
    // lookup table gravOffset(angle[]) measured via multi-angle holdtest sweeps.
    // The gravBoostMult ramp was fit by eyeballing one test run; it is
    // physically plausible (linkage leverage loss at extremes) but not derived
    // from actual geometry.  When payload changes, the leverage-loss effect and
    // the payload-mass effect do NOT necessarily scale together, so the hand-fit
    // shape will drift.  Until multi-angle holdtest data is collected, the
    // current formula is left unchanged — do not remove gravBoostMult without
    // replacing it with measured data.  See GravityProfile in Axis.h TODO notes.
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

    // ── Fix 4 (part B): Post-PI EMA slew filter REMOVED ─────────────────────
    // The old EMA (0.2 * new + 0.8 * history) sat after a closed loop that
    // already has P+leaky-I dynamics, creating two stacked dynamic elements in
    // series. This is a classic recipe for either sluggish response or
    // beat-frequency oscillation depending on relative time constants.
    // Smoothing is now applied upstream on smoothedTgtVelDegS (see above).
    // If mechanical whipping still occurs after retuning, lower Kp_vel — that
    // is the root cause, not evidence that a second filter stage is needed.
    int finalOutputPulse = constrain(servoSignal, servoStop - SERVO_DRIVE, servoStop + SERVO_DRIVE);

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
    
    
    uint16_t sigToWrite = (sysState == SYS_RUNNING) ? (uint16_t)finalOutputPulse
                                                     : (uint16_t)servoStop;
    if ((int)sigToWrite != lastServoSignal || millis() - lastServoWrite >= 20) {
        pwm->setPWM(pwmChannel, 0, sigToWrite);
        lastServoSignal = (int)sigToWrite;
        lastServoWrite  = millis();
    }

    // ── Periodic I2C health check (every 1 s) ──
    // Detects PCA9685 lockup from servo power-noise I2C glitch.
    // Performs automatic bus recovery + PCA9685 reinitialization.
    
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
            Serial.printf("[S%d][GRAVTEST] Drift threshold: %d ADC (~5°). 'stop' to abort.\n", this->id, htDriftThr);
        }
    }

    // ── 10 Hz diagnostic print ──
    
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        const char* zoneTag = inDeadband  ? "[HOLD]"
                            : inTranszone ? "[ZTRANS]"
                                          : "[MOVE]";
        Serial.printf("[S%d][RUN] pot=%4d  ang=%5.1f°  tvl=%+6.1f  vel=%+6.1f  raw=%+6.1f  pid=%5.1f  pulse=%3d  %s\n",
                      this->id, potRaw, potToAngle(potRaw),
                      tgtVelDegS, measuredVelDegS, rawVelDegS,
                      velPidOut, finalOutputPulse, zoneTag);
        // velplot streaming (every 50ms when active)
        if (velPlotActive) {
            Serial.printf("VELPLOT %.2f %.2f %.2f\n",
                          rawVelDegS, measuredVelDegS, tgtVelDegS);
        }
    }
}

bool Axis::potConnected(int val) {
    return (val > POT_BROKEN_LOW) && (val < POT_BROKEN_HIGH);
}

void Axis::reset() {
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
}

void Axis::stop() {
    targetSet      = false;
    wdArmed        = false;
    htPhase        = HT_IDLE;
    profileVelDegS = 0.0f;
    tgtVelDegS     = 0.0f;
    velIntegral    = 0.0f;
    velPidOut      = 0.0f;
    if (sysState == SYS_SLOWTEST || sysState == SYS_RUNNING || sysState == SYS_HOLDTEST || sysState == SYS_JOG) {
        sysState = SYS_RUNNING;
    }
    pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
}

void Axis::setTarget(float ang) {
    ReachResult r = checkReachability(ang);
    targetAngleDeg  = r.clampedAngle;
    finalTargetDeg  = r.clampedAngle;
    currentPosDeg   = potToAngle(readPotFiltered());
    profileVelDegS  = 0.0f;
    tgtVelDegS      = 0.0f;
    velIntegral     = 0.0f;
    velPidOut       = 0.0f;
    wdArmed         = false;
    wdGraceUntil    = millis() + 2000;
    sticRetry       = 0;
    targetSet       = true;
    pidSetpoint     = (double)r.clampedPot;
    lastPotForVel   = readPotFiltered();
    lastVelTimeMs   = millis();
    measuredVelDegS = 0.0f;
}

void Axis::init() {
    pinMode(potPin, INPUT);
    // Initialize filter
    int initialVal = analogRead(potPin);
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        filterBuf[i] = initialVal;
    }
    filterSum = initialVal * FILTER_SAMPLES;
    filterFull = true;

    loadSettingsFromFlash();
    loadCalFromFlash();

    currentPosDeg = potToAngle(readPotFiltered());
    targetAngleDeg = currentPosDeg;
    finalTargetDeg = currentPosDeg;

    controller.init(Kp_vel, Ki_vel, 0.0, -SERVO_DRIVE, SERVO_DRIVE, -SERVO_DRIVE, SERVO_DRIVE, 50);
}

int Axis::readPotFiltered() {
    int raw = analogRead(potPin);
    filterSum -= filterBuf[filterIdx];
    filterBuf[filterIdx] = raw;
    filterSum += raw;
    filterIdx = (filterIdx + 1) % FILTER_SAMPLES;
    return filterSum / FILTER_SAMPLES;
}

float Axis::potToAngle(int potVal) {
    if (fabsf(cal.adcPerDeg) < 0.01f) return 0.0f;
    return constrain((float)(potVal - cal.pot0deg) / cal.adcPerDeg, 0.0f, 180.0f);
}

int Axis::angleToPot(float angle) {
    return cal.pot0deg + (int)(angle * cal.adcPerDeg);
}

int Axis::clampToSoftLimits(int rawADC) {
    int highADC = max(cal.softLimitHigh, cal.softLimitLow);
    int lowADC = min(cal.softLimitHigh, cal.softLimitLow);
    return constrain(rawADC, lowADC, highADC);
}

Axis::ReachResult Axis::checkReachability(float targetDeg) {
    ReachResult res;
    res.ok = true;
    res.clampedAngle = constrain(targetDeg, SAFETY_MARGIN_DEG, 180.0f - SAFETY_MARGIN_DEG);
    res.clampedPot = angleToPot(res.clampedAngle);
    res.reason = "";
    return res;
}

void Axis::startRecal(int mode) {
    recalMode = mode;
    sysState = SYS_RECAL_ACTIVE;
    recalPhase = (mode == 1) ? RECAL_COLLECT_180DEG : RECAL_COLLECT_0DEG;
    recalStartMs = millis();
    recalAccum = 0;
    recalSamples = 0;
    recalMin = 9999;
    recalMax = 0;
    pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
}


void Axis::loadCalFromFlash() {
    prefs->begin(nvsNamespace.c_str(), false);
    int s0   = prefs->getInt(NVS_KEY_0DEG,   DEFAULT_POT_0DEG);
    int s180 = prefs->getInt(NVS_KEY_180DEG, DEFAULT_POT_180DEG);
    prefs->end();

    bool valid = (s0   > POT_BROKEN_LOW  && s0   < POT_BROKEN_HIGH) &&
                 (s180 > POT_BROKEN_LOW  && s180 < POT_BROKEN_HIGH) &&
                 (abs(s0 - s180) >= RECAL_MIN_SPAN_ADC);

    if (!valid) {
        Serial.printf("[S%d][CAL] NVS invalid — using factory defaults\n", this->id);
        s0   = DEFAULT_POT_0DEG;
        s180 = DEFAULT_POT_180DEG;
    } else {
        Serial.printf("[S%d][CAL] Loaded from NVS flash\n", this->id);
    }
    recomputeCalibration(s0, s180);
}

void Axis::updateVelocityMeasurement(int potRaw) {
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

void Axis::updateRecal(int potNow) {
    if (recalPhase == RECAL_IDLE) return;

    recalAccum += potNow;
    recalSamples++;
    if (potNow < recalMin) recalMin = potNow;
    if (potNow > recalMax) recalMax = potNow;

    unsigned long elapsed = millis() - recalStartMs;
    if (elapsed < RECAL_STABILITY_MS) {
        
        if (millis() - lastRecalPrint >= 500) {
            lastRecalPrint = millis();
            Serial.printf("[RECAL] Collecting... %lums  variance=%d\n",
                          elapsed, recalMax - recalMin);
        }
        return;
    }

    int variance   = recalMax - recalMin;
    int newReading = (int)(recalAccum / recalSamples);
    Serial.printf("[S%d][RECAL] Done: reading=%d  variance=%d\n", this->id, newReading, variance);

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
            Serial.printf("[S%d][RECAL] ✓ 0° recalibrated and saved\n", this->id);

        } else if (recalMode == 1) {
            // Single 180° — GATE 4
            if (abs(newReading - cal.pot0deg) < RECAL_MIN_SPAN_ADC) {
                abortRecal("New 180° too close to existing 0° reading"); return;
            }
            recomputeCalibration(cal.pot0deg, newReading);
            saveCalToFlash();
            recalPhase = RECAL_IDLE;  sysState = SYS_RUNNING;
            Serial.printf("[S%d][RECAL] ✓ 180° recalibrated and saved\n", this->id);

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
        Serial.printf("[S%d][RECAL] ✓ Two-point recalibration complete and saved\n", this->id);
    }

    pidSetpoint = clampToSoftLimits(angleToPot(targetAngleDeg));
}

void Axis::updateWatchdog(int potNow, bool shouldBeActive, int pidSign) {
    // Watchdog is completely disabled to allow slow trajectory movements!
    return;
}

void Axis::updateDriftDetection(int potNow, bool inDeadband) {
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
            if (fabsf(cal.adcPerDeg) > 0.01f) {
                finalTargetDeg += (float)correction / cal.adcPerDeg;
                targetAngleDeg  = finalTargetDeg;
            }
            Serial.printf("[S%d][DRIFT] Minor drift %d ADC — setpoint auto-corrected\n", this->id, drift);
        } else {
            triggerDriftWarn(drift);
        }
    }
    driftRefPot  = potNow;
    driftRefTime = millis();
}

void Axis::triggerFault(const char* reason) {
    pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
    sysState = SYS_FAULT;
    Serial.print(F("[FAULT] "));
    Serial.println(reason);
    Serial.printf("[S%d][FAULT] Send 'reset' to clear and resume\n", this->id);
}

void Axis::saveCalToFlash() {
    prefs->begin(nvsNamespace.c_str(), false);
    prefs->putInt(NVS_KEY_0DEG,   cal.pot0deg);
    prefs->putInt(NVS_KEY_180DEG, cal.pot180deg);
    prefs->end();
    Serial.printf("[S%d][CAL] Saved to NVS flash\n", this->id);
}

void Axis::abortRecal(const char* reason) {
    recalPhase  = RECAL_IDLE;
    pendingPot0 = -1;
    sysState    = SYS_RUNNING;
    Serial.print(F("[RECAL] ABORTED — "));
    Serial.println(reason);
    Serial.printf("[S%d][RECAL] Old calibration kept — resuming\n", this->id);
}

void Axis::triggerDriftWarn(int driftADC) {
    pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
    sysState = SYS_DRIFT_WARN;
    Serial.printf("[S%d][DRIFT] Warning — pot drifted %d ADC while stationary\n", this->id, driftADC);
    Serial.printf("[S%d][DRIFT] Send 'recal2' to recalibrate, or 'resume' to ignore\n", this->id);
}

void Axis::setServoStop(int pulse) {
    if (pulse < 200 || pulse > 500) {
        Serial.printf("[S%d][ERROR] Invalid neutral pulse %d! Must be 200–500 (~1500us).\n", this->id, pulse);
        return;
    }
    servoStop = pulse;
    saveSettingsToFlash();
    pwm->setPWM(pwmChannel, 0, (uint16_t)servoStop);
    Serial.printf("[S%d][CFG] New neutral servoStop saved: %d\n", this->id, servoStop);
}

void Axis::startSlowtest(int dir) {
    stDir = (dir >= 0) ? 1 : -1;
    stDrive = 0;
    stStartPot = readPotFiltered();
    stLastStep = millis();
    targetSet = true; // Enable execution loop to reach SLOWTEST state
    sysState = SYS_SLOWTEST;
    Serial.printf("[S%d][SLOWTEST] Starting stiction sweep %s\n", this->id, (stDir > 0) ? "+" : "-");
}

// ── printDtHistogram ───────────────────────────────────────────────────────────
// Dumps the dt distribution histogram over serial, then resets it.
// Use the 'dtstats <id>' serial command to call this.
// Interpret output: tight cluster around one bucket = loop is regular.
// Bimodal or fat tail >35ms = loop is stalling (likely blocking serial
// or other long-running code in loop()).
void Axis::printDtHistogram() {
    Serial.printf("[S%d][DTSTATS] dt distribution (last %u samples):\n",
                  this->id,
                  dtHistogram[0] + dtHistogram[1] + dtHistogram[2] + dtHistogram[3] +
                  dtHistogram[4] + dtHistogram[5] + dtHistogram[6] + dtHistogram[7]);
    const char* labels[DT_BUCKETS] = {
        " 0-5ms", " 5-10ms", "10-15ms", "15-20ms",
        "20-25ms", "25-30ms", "30-35ms", "  >35ms"
    };
    for (int i = 0; i < DT_BUCKETS; i++) {
        Serial.printf("  [%s]: %u\n", labels[i], dtHistogram[i]);
        dtHistogram[i] = 0;  // reset after printing
    }
}