
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include "Axis.h"

// ───────────────────────────────────────────────
//  HARDWARE CONFIG
// ───────────────────────────────────────────────
#define I2C_SDA         26
#define I2C_SCL         27
#define PCA9685_ADDR    0x40

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
Preferences prefs;

bool printRawPots = false;

// Define our two modular axes
// Axis(ID, PotPin, PCAChannel, NVS_Namespace, pwmDriver, prefs)
Axis axes[2] = {
    Axis(0, 34, 0, "servo0", &pwm, &prefs),
    Axis(1, 35, 1, "servo1", &pwm, &prefs)
};

// ───────────────────────────────────────────────
//  I2C RECOVERY
// ───────────────────────────────────────────────
bool i2cProbe() {
    Wire.beginTransmission(PCA9685_ADDR);
    return (Wire.endTransmission() == 0);
}

void recoverI2C() {
    Serial.println(F("[FATAL] PCA9685 unresponsive! Attempting I2C bus recovery..."));
    Wire.end();
    
    pinMode(I2C_SDA, OUTPUT);
    pinMode(I2C_SCL, OUTPUT);
    digitalWrite(I2C_SDA, HIGH);
    digitalWrite(I2C_SCL, HIGH);
    delay(10);
    
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL, LOW);
        delayMicroseconds(10);
        digitalWrite(I2C_SCL, HIGH);
        delayMicroseconds(10);
    }
    
    digitalWrite(I2C_SDA, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SDA, HIGH);
    delay(10);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    
    if (i2cProbe()) {
        Serial.println(F("[OK] I2C bus recovered. Re-initializing PCA9685..."));
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(50);
        delay(10);
    } else {
        Serial.println(F("[ERROR] I2C recovery failed. Hardware check required."));
    }
}

// ───────────────────────────────────────────────
//  COMMAND PARSING
// ───────────────────────────────────────────────
void printHelp() {
    Serial.println(F("\n═══════════ MULTI-AXIS SERVO CMD ═══════════"));
    Serial.println(F("Format: <command> <axis_id> <value>"));
    Serial.println(F("Example: target 0 90"));
    Serial.println(F("  target <id> <angle>   move servo to 3.0°–177.0°"));
    Serial.println(F("  notarget <id>         stop PID, servo idle at neutral"));
    Serial.println(F("  stop <id>             alias for notarget"));
    Serial.println(F("  reset <id>            clear fault and resume"));
    Serial.println(F("  stiction0 <id> <n>    set stiction offset toward 0°"));
    Serial.println(F("  stiction180 <id> <n>  set stiction offset toward 180°"));
    Serial.println(F("  kgrav <id> <float>    set gravity compensation gain"));
    Serial.println(F("  cal0 <id> <adc>       manually set 0° endpoint ADC"));
    Serial.println(F("  cal180 <id> <adc>     manually set 180° endpoint ADC"));
    Serial.println(F("  recal0 <id>           auto-calibrate pot at 0° endpoint"));
    Serial.println(F("  recal180 <id>         auto-calibrate pot at 180° endpoint"));
    Serial.println(F("  status                print active settings for all axes"));
    Serial.println(F("══════════════════════════════════════════════\n"));
}

void printStatus() {
    Serial.println(F("--- SYSTEM STATUS ---"));
    
    if (printRawPots) {
        static unsigned long lastRaw = 0;
        if (millis() - lastRaw >= 500) {
            lastRaw = millis();
            int p0 = analogRead(34);
            int p1 = analogRead(35);
            Serial.printf("[RAW] Pin 34 (S0): %4d  |  Pin 35 (S1): %4d\n", p0, p1);
        }
    }

    for (int i = 0; i < 2; i++) {
        Axis &ax = axes[i];
        Serial.printf("Axis %d: Pin=%d, Ch=%d\n", ax.id, ax.potPin, ax.pwmChannel);
        Serial.printf("  Target: %.1f  Current: %.1f  Pulse: %d\n", ax.targetAngleDeg, ax.currentPosDeg, ax.lastServoSignal);
    }
}

void handleCommand(String cmd) {
    cmd.trim();
    if (!cmd.length()) return;

    if (cmd.equalsIgnoreCase("help"))   { printHelp();   return; }
    if (cmd.equalsIgnoreCase("status")) { printStatus(); return; }
    
    if (cmd.equalsIgnoreCase("raw") || cmd.equalsIgnoreCase("rawpots") || 
        cmd.equalsIgnoreCase("raw on") || cmd.equalsIgnoreCase("rawpots on")) {
        printRawPots = true;
        Serial.println("[OK] Raw POT values mode ENABLED. Type 'raw off' to exit.");
        return;
    }
    if (cmd.equalsIgnoreCase("raw off") || cmd.equalsIgnoreCase("rawpots off") ||
        cmd.equalsIgnoreCase("noraw")) {
        printRawPots = false;
        Serial.println("[OK] Raw POT values mode DISABLED.");
        return;
    }
    
    // Parse syntax: <command> <axis_id> [value]
    int firstSpace = cmd.indexOf(' ');
    if (firstSpace == -1) {
        Serial.println(F("[ERROR] Missing Axis ID! Example: target 0 90"));
        return;
    }
    
    String action = cmd.substring(0, firstSpace);
    action.trim();
    action.toLowerCase();
    
    String remaining = cmd.substring(firstSpace + 1);
    remaining.trim();
    
    int secondSpace = remaining.indexOf(' ');
    String axIdStr = "";
    String valStr = "";
    
    if (secondSpace == -1) {
        axIdStr = remaining;
    } else {
        axIdStr = remaining.substring(0, secondSpace);
        valStr = remaining.substring(secondSpace + 1);
        valStr.trim();
    }
    
    if (axIdStr != "0" && axIdStr != "1") {
        Serial.println(F("[ERROR] Invalid Axis ID! Must be 0 or 1."));
        return;
    }
    
    int ax_id = axIdStr.toInt();
    Axis &ax = axes[ax_id];
    
    if (action == "target") {
        if (valStr == "") { Serial.println(F("[ERROR] target requires angle")); return; }
        ax.setTarget(valStr.toFloat());
        return;
    }
    if (action == "stop" || action == "notarget") {
        ax.stop();
        Serial.printf("[INFO] Axis %d stopped.\n", ax_id);
        return;
    }
    if (action == "reset") {
        ax.reset();
        Serial.printf("[INFO] Axis %d reset.\n", ax_id);
        return;
    }
    if (action == "kgrav") {
        if (valStr == "") { Serial.println(F("[ERROR] kgrav requires value")); return; }
        ax.kGravity = valStr.toFloat();
        ax.saveSettingsToFlash();
        Serial.printf("[CFG] Axis %d kGravity = %.3f\n", ax_id, ax.kGravity);
        return;
    }
    if (action == "stiction0") {
        if (valStr == "") { Serial.println(F("[ERROR] stiction0 requires value")); return; }
        ax.servoStictionTo0 = valStr.toInt();
        ax.saveSettingsToFlash();
        Serial.printf("[CFG] Axis %d stictionTo0 = %d\n", ax_id, ax.servoStictionTo0);
        return;
    }
    if (action == "stiction180") {
        if (valStr == "") { Serial.println(F("[ERROR] stiction180 requires value")); return; }
        ax.servoStictionTo180 = valStr.toInt();
        ax.saveSettingsToFlash();
        Serial.printf("[CFG] Axis %d stictionTo180 = %d\n", ax_id, ax.servoStictionTo180);
        return;
    }
    if (action == "cal0") {
        if (valStr == "") { Serial.println(F("[ERROR] cal0 requires ADC value")); return; }
        ax.recomputeCalibration(valStr.toInt(), ax.cal.pot180deg);
        ax.saveCalToFlash();
        return;
    }
    if (action == "cal180") {
        if (valStr == "") { Serial.println(F("[ERROR] cal180 requires ADC value")); return; }
        ax.recomputeCalibration(ax.cal.pot0deg, valStr.toInt());
        ax.saveCalToFlash();
        return;
    }
    if (action == "recal0") { ax.startRecal(0); return; }
    if (action == "recal180") { ax.startRecal(1); return; }
    if (action == "recal2") { ax.startRecal(2); return; }
    
    Serial.println(F("[ERROR] Unknown command or bad syntax. Type 'help'."));
}

// ───────────────────────────────────────────────
//  SETUP & LOOP
// ───────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    
    if (!i2cProbe()) {
        Serial.println(F("[FATAL] PCA9685 not found on I2C bus!"));
        while (1) delay(100);
    }
    
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    delay(10);
    
    Serial.println(F("\n\n=========================================="));
    Serial.println(F(" MULTI-AXIS SERVO CONTROLLER BOOTING..."));
    Serial.println(F("=========================================="));
    
    
    if (printRawPots) {
        static unsigned long lastRaw = 0;
        if (millis() - lastRaw >= 500) {
            lastRaw = millis();
            int p0 = analogRead(34);
            int p1 = analogRead(35);
            Serial.printf("[RAW] Pin 34 (S0): %4d  |  Pin 35 (S1): %4d\n", p0, p1);
        }
    }

    for (int i = 0; i < 2; i++) {
        axes[i].init();
    }
    
    Serial.println(F("[OK] System running. Type 'help' for commands."));
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }
    
    
    if (printRawPots) {
        static unsigned long lastRaw = 0;
        if (millis() - lastRaw >= 500) {
            lastRaw = millis();
            int p0 = analogRead(34);
            int p1 = analogRead(35);
            Serial.printf("[RAW] Pin 34 (S0): %4d  |  Pin 35 (S1): %4d\n", p0, p1);
        }
    }

    for (int i = 0; i < 2; i++) {
        axes[i].update();
    }
}
