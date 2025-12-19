/**
   Arduino Electronic Safe  -> Adapted for Landing Gear Deploy/Retract Timing Simulation

   Original Copyright (C) 2020, Uri Shaked.
   Released under the MIT License.
*/

#include <LiquidCrystal.h>
#include <Keypad.h>
#include <Servo.h>
#include <math.h>
#include "SafeState.h"
#include "icons.h"

/* --------------------------------------------------------------------------
   Landing gear config injection
---------------------------------------------------------------------------*/
struct GearConfig {
  const char* name;
  uint16_t pump_latency_ms;
  float actuator_speed_mm_per_100ms;
  uint16_t extension_distance_mm;
  uint16_t lock_time_ms;
  uint16_t requirement_time_ms;
};

GearConfig configA = {"Config A - Shared Pump",    300,  8.0f, 700, 300, 8000};
GearConfig configB = {"Config B - Dedicated Pump", 100, 12.0f, 700, 300, 8000};

// Choose one:
GearConfig cfg = configA;   // change to configB when needed

/* --------------------------------------------------------------------------
   Locking mechanism definitions (re-used as uplock/downlock actuator)
---------------------------------------------------------------------------*/
#define SERVO_PIN 6
#define SERVO_LOCK_POS   20
#define SERVO_UNLOCK_POS 90
Servo lockServo;

/* --------------------------------------------------------------------------
   Display
---------------------------------------------------------------------------*/
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

/* --------------------------------------------------------------------------
   Keypad setup
---------------------------------------------------------------------------*/
const byte KEYPAD_ROWS = 4;
const byte KEYPAD_COLS = 4;
byte rowPins[KEYPAD_ROWS] = {5, 4, 3, 2};
byte colPins[KEYPAD_COLS] = {A3, A2, A1, A0};
char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

/* SafeState stores the secret code in EEPROM */
SafeState safeState;

/* --------------------------------------------------------------------------
   Timing instrumentation
---------------------------------------------------------------------------*/
unsigned long t_lock_start = 0, t_lock_end = 0;
unsigned long t_unlock_start = 0, t_unlock_end = 0;
unsigned long t_op_start = 0, t_op_end = 0;

/* --------------------------------------------------------------------------
   Lock/Unlock primitives (instrumented)
   Note: We treat UNLOCK as "release lock" and LOCK as "engage lock".
---------------------------------------------------------------------------*/
void lock() {
  t_lock_start = millis();

  lockServo.write(SERVO_LOCK_POS);
  safeState.lock();                 // keep state behavior (optional but harmless)

  delay(cfg.lock_time_ms);          // model physical lock engage time

  t_lock_end = millis();
  Serial.print("LOCK time (ms): ");
  Serial.println(t_lock_end - t_lock_start);
}

void unlock() {
  t_unlock_start = millis();

  lockServo.write(SERVO_UNLOCK_POS);

  delay(cfg.lock_time_ms);          // model physical lock release time

  t_unlock_end = millis();
  Serial.print("UNLOCK time (ms): ");
  Serial.println(t_unlock_end - t_unlock_start);
}

/* --------------------------------------------------------------------------
   Reporting helpers
---------------------------------------------------------------------------*/
void reportOpTime(const char* opName, unsigned long elapsedMs) {
  Serial.print(opName);
  Serial.print(" total time (ms): ");
  Serial.println(elapsedMs);

  Serial.print("Requirement (ms): ");
  Serial.println(cfg.requirement_time_ms);

  Serial.print("Result: ");
  Serial.println(elapsedMs <= cfg.requirement_time_ms ? "PASS" : "FAIL");
}

unsigned long extensionTimeMs(float speed_mm_per_100ms, uint16_t distance_mm) {
  // Time = 100 * (distance / speed), quantized to 100ms ticks (embedded-realistic)
  float steps = (float)distance_mm / speed_mm_per_100ms;
  unsigned long t = (unsigned long)(ceil(steps) * 100.0f);
  return t;
}

/* --------------------------------------------------------------------------
   Landing gear deploy/retract simulation using lock/unlock as endpoints
---------------------------------------------------------------------------*/
void simulateDeploy() {
  Serial.println();
  Serial.print("Running DEPLOY: ");
  Serial.println(cfg.name);

  t_op_start = millis();

  // Pump latency
  delay(cfg.pump_latency_ms);

  // Release uplock
  unlock();

  // Extension travel time
  unsigned long t_extend = extensionTimeMs(cfg.actuator_speed_mm_per_100ms,
                                          cfg.extension_distance_mm);
  delay(t_extend);

  // Engage downlock
  lock();

  t_op_end = millis();
  reportOpTime("DEPLOY", t_op_end - t_op_start);
}

void simulateRetract() {
  Serial.println();
  Serial.print("Running RETRACT: ");
  Serial.println(cfg.name);

  t_op_start = millis();

  // Pump latency
  delay(cfg.pump_latency_ms);

  // Release downlock
  unlock();

  // Retraction travel time (same distance/speed model)
  unsigned long t_retract = extensionTimeMs(cfg.actuator_speed_mm_per_100ms,
                                           cfg.extension_distance_mm);
  delay(t_retract);

  // Engage uplock
  lock();

  t_op_end = millis();
  reportOpTime("RETRACT", t_op_end - t_op_start);
}

/* --------------------------------------------------------------------------
   Original UI helpers (mostly unchanged)
---------------------------------------------------------------------------*/
void showStartupMessage() {
  lcd.setCursor(4, 0);
  lcd.print("Welcome!");
  delay(1000);

  lcd.setCursor(0, 1);
  String message = "ArduinoSafe v1.0";
  for (byte i = 0; i < message.length(); i++) {
    lcd.print(message[i]);
    delay(100);
  }
  delay(500);
}

String inputSecretCode() {
  lcd.setCursor(5, 1);
  lcd.print("[____]");
  lcd.setCursor(6, 1);
  String result = "";
  while (result.length() < 4) {
    char key = keypad.getKey();
    if (key >= '0' && key <= '9') {
      lcd.print('*');
      result += key;
    }
  }
  return result;
}

void showWaitScreen(int delayMillis) {
  lcd.setCursor(2, 1);
  lcd.print("[..........]");
  lcd.setCursor(3, 1);
  for (byte i = 0; i < 10; i++) {
    delay(delayMillis);
    lcd.print("=");
  }
}

bool setNewCode() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter new code:");
  String newCode = inputSecretCode();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Confirm new code");
  String confirmCode = inputSecretCode();

  if (newCode.equals(confirmCode)) {
    safeState.setCode(newCode);
    return true;
  } else {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Code mismatch");
    lcd.setCursor(0, 1);
    lcd.print("Safe not locked!");
    delay(2000);
    return false;
  }
}

void showUnlockMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(ICON_UNLOCKED_CHAR);
  lcd.setCursor(4, 0);
  lcd.print("Unlocked!");
  lcd.setCursor(15, 0);
  lcd.write(ICON_UNLOCKED_CHAR);
  delay(1000);
}

void safeUnlockedLogic() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.write(ICON_UNLOCKED_CHAR);
  lcd.setCursor(2, 0);
  lcd.print(" # to deploy");
  lcd.setCursor(15, 0);
  lcd.write(ICON_UNLOCKED_CHAR);

  bool newCodeNeeded = true;

  if (safeState.hasCode()) {
    lcd.setCursor(0, 1);
    lcd.print("  A = new code");
    newCodeNeeded = false;
  } else {
    lcd.setCursor(0, 1);
    lcd.print("  A = set code");
  }

  auto key = keypad.getKey();
  while (key != 'A' && key != '#') {
    key = keypad.getKey();
  }

  bool readyToDeploy = true;
  if (key == 'A' || newCodeNeeded) {
    readyToDeploy = setNewCode();
  }

  if (readyToDeploy) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Deploying...");

    // Treat "lock" transition as deploy action in this adapted demo
    safeState.lock();
    simulateDeploy();
    showWaitScreen(100);
  }
}

void safeLockedLogic() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(ICON_LOCKED_CHAR);
  lcd.print(" Safe Locked! ");
  lcd.write(ICON_LOCKED_CHAR);

  String userCode = inputSecretCode();
  bool unlockedSuccessfully = safeState.unlock(userCode);
  showWaitScreen(200);

  if (unlockedSuccessfully) {
    showUnlockMessage();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Retracting...");

    simulateRetract();
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Access Denied!");
    showWaitScreen(1000);
  }
}

/* --------------------------------------------------------------------------
   Arduino entry points
---------------------------------------------------------------------------*/
void setup() {
  lcd.begin(16, 2);
  init_icons(lcd);

  lockServo.attach(SERVO_PIN);

  Serial.begin(115200);
  Serial.println("LGCS timing sim starting...");
  Serial.print("Selected config: ");
  Serial.println(cfg.name);

  // Ensure initial servo position matches saved state
  if (safeState.locked()) {
    lock();
  } else {
    unlock();
  }

  showStartupMessage();
}

void loop() {
  if (safeState.locked()) {
    safeLockedLogic();     // correct code -> retract
  } else {
    safeUnlockedLogic();   // # -> deploy
  }
}
