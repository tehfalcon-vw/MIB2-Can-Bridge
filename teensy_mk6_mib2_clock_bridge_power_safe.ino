#include <FlexCAN_T4.h>
#include <Snooze.h>

static constexpr const char *FIRMWARE_VERSION = "MK6-CLOCK-POWER-SAFE-RC1-2026-07-25";
static constexpr uint32_t BUS_SPEED = 100000;
static constexpr uint32_t CLOCK_DIAG_1_CAN_ID = 0x65D;
static constexpr uint32_t CLOCK_KOMBI_K2_CAN_ID = 0x623;
static constexpr uint32_t RUN_ACTIVITY_CAN_ID_2C3 = 0x2C3;
static constexpr uint32_t RUN_ACTIVITY_CAN_ID_575 = 0x575;
static constexpr uint32_t POWER_COMPAT_CAN_ID_436 = 0x436;
static constexpr uint32_t POWER_COMPAT_CAN_ID_439 = 0x439;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_481 = 0x481;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_484 = 0x484;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_485 = 0x485;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_486 = 0x486;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_5A1 = 0x5A1;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_5A3 = 0x5A3;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_604 = 0x604;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_661 = 0x661;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_665 = 0x665;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_666 = 0x666;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_66C = 0x66C;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_66F = 0x66F;
static constexpr uint32_t RADIO_AWAKE_CAN_ID_67D = 0x67D;
static constexpr uint8_t SERIAL_BUFFER_MAX = 64;
static constexpr uint8_t OBSERVED_ID_SLOTS = 16;
static constexpr uint32_t OBSERVED_ID_WINDOW_MS = 3000;
static constexpr uint32_t CLOCK_REPEAT_INTERVAL_MS = 1000;
static constexpr uint32_t CLOCK_REPEAT_ACTIVE_HOLD_MS = 5000;
static constexpr uint32_t CLOCK_CACHE_FRESHNESS_MS = 3000;
static constexpr uint32_t RADIO_WAKE_BURST_GAP_MS = 250;
static constexpr uint8_t RADIO_WAKE_MIN_BURST_FRAMES = 8;
static constexpr uint8_t RADIO_WAKE_MIN_NON436_FRAMES = 2;
static constexpr uint32_t RUN_ACTIVITY_TIMEOUT_MS = 10000;
static constexpr uint32_t RADIO_SHUTDOWN_GRACE_MS = 300000;
static constexpr uint32_t INITIAL_WAKE_PROBE_TIMEOUT_MS = 10000;
static constexpr uint32_t WAKE_PROBE_TIMEOUT_MS = 500;
static constexpr uint8_t VEHICLE_CAN_RX_WAKE_PIN = 23;
static constexpr uint8_t VEHICLE_TRANSCEIVER_RS_PIN = 20;
static constexpr uint8_t RADIO_TRANSCEIVER_RS_PIN = 21;

enum class PowerState : uint8_t {
  WakeProbe,
  Active,
  ShutdownGrace,
};

struct CanIdActivityCounter {
  uint32_t id;
  uint32_t count;
  uint32_t lastSeenMs;
};

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canVehicle;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> canRadio;
SnoozeDigital vehicleCanWake;
SnoozeUSBSerial sleepUsb;
SnoozeBlock powerSleepConfig(sleepUsb, vehicleCanWake);

static char serialBuffer[SERIAL_BUFFER_MAX] = {};
static uint8_t serialBufferLen = 0;

static bool verboseLogging = false;
static uint32_t forwardedFramesVehicleToRadio = 0;
static uint32_t forwardedFramesRadioToVehicle = 0;
static uint32_t clock65dVehicleToRadio = 0;
static uint32_t clock65dRadioToVehicle = 0;
static uint32_t clock623VehicleToRadio = 0;
static uint32_t clock623RadioToVehicle = 0;
static uint32_t power436VehicleToRadio = 0;
static uint32_t power436RadioToVehicle = 0;
static uint32_t power439VehicleToRadio = 0;
static uint32_t power439RadioToVehicle = 0;
static uint32_t lastVehicleActivityMs = 0;
static uint32_t lastRadioActivityMs = 0;
static bool clockRepeatEnabled = true;
static bool cachedClock65dValid = false;
static uint8_t cachedClock65d[8] = {};
static uint8_t cachedClock65dLen = 0;
static uint32_t lastClock65dVehicleMs = 0;
static uint32_t lastClockRepeatMs = 0;
static uint32_t repeatedClock65dToRadio = 0;
static uint32_t rejectedClock65dFrames = 0;
static uint32_t failedClock65dRepeats = 0;
static uint32_t radioWakeBurstStartedMs = 0;
static uint32_t lastRadioWakeFrameMs = 0;
static uint8_t radioWakeBurstFrames = 0;
static uint8_t radioWakeBurstNon436Frames = 0;
static uint32_t radioClockRepeatAllowedUntilMs = 0;
static bool deepSleepEnabled = true;
static bool bridgeForwardingEnabled = false;
static PowerState powerState = PowerState::WakeProbe;
static uint32_t powerStateStartedMs = 0;
static uint32_t lastRunActivityMs = 0;
static uint32_t runActivity2c3Frames = 0;
static uint32_t runActivity575Frames = 0;
static uint32_t shutdownGraceEntries = 0;
static uint32_t deepSleepEntries = 0;
static uint32_t deepSleepWakeups = 0;
static int lastDeepSleepWakeSource = -1;
static CanIdActivityCounter vehicleObservedIds[OBSERVED_ID_SLOTS] = {};
static CanIdActivityCounter radioObservedIds[OBSERVED_ID_SLOTS] = {};

void enterActivePowerState();

void printFrame(const char *prefix, const CAN_message_t &msg) {
  Serial.print(prefix);
  Serial.print(" id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(" len=");
  Serial.print(msg.len);
  Serial.print(" data=");
  for (uint8_t i = 0; i < msg.len; ++i) {
    if (msg.buf[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(msg.buf[i], HEX);
    if (i + 1 < msg.len) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

void clearObservedCanIds(CanIdActivityCounter *counters, uint8_t slots) {
  for (uint8_t i = 0; i < slots; ++i) {
    counters[i].id = 0;
    counters[i].count = 0;
    counters[i].lastSeenMs = 0;
  }
}

void noteObservedCanId(const CAN_message_t &msg, CanIdActivityCounter *counters, uint8_t slots) {
  if (msg.flags.extended) {
    return;
  }

  const uint32_t now = millis();
  int8_t emptySlot = -1;
  int8_t oldestSlot = 0;

  for (uint8_t i = 0; i < slots; ++i) {
    if (counters[i].count == 0) {
      if (emptySlot < 0) {
        emptySlot = static_cast<int8_t>(i);
      }
      continue;
    }

    if (counters[i].id == msg.id) {
      counters[i].count++;
      counters[i].lastSeenMs = now;
      return;
    }

    if (counters[i].lastSeenMs < counters[oldestSlot].lastSeenMs) {
      oldestSlot = static_cast<int8_t>(i);
    }
  }

  const int8_t slot = emptySlot >= 0 ? emptySlot : oldestSlot;
  counters[slot].id = msg.id;
  counters[slot].count = 1;
  counters[slot].lastSeenMs = now;
}

void printObservedCanIds(const char *label, const CanIdActivityCounter *counters, uint8_t slots,
                         uint32_t now, uint32_t recentWindowMs) {
  Serial.print(label);
  Serial.println(":");

  bool printedAny = false;
  bool used[OBSERVED_ID_SLOTS] = {};
  for (uint8_t rank = 0; rank < slots; ++rank) {
    int8_t best = -1;
    for (uint8_t i = 0; i < slots; ++i) {
      if (used[i] || counters[i].count == 0 || counters[i].lastSeenMs == 0) {
        continue;
      }
      if (now - counters[i].lastSeenMs > recentWindowMs) {
        continue;
      }
      if (best < 0 || counters[i].count > counters[best].count) {
        best = static_cast<int8_t>(i);
      }
    }

    if (best < 0) {
      break;
    }

    used[best] = true;
    printedAny = true;
    Serial.print("  id=0x");
    Serial.print(counters[best].id, HEX);
    Serial.print(" count=");
    Serial.print(counters[best].count);
    Serial.print(" lastSeenMsAgo=");
    Serial.println(now - counters[best].lastSeenMs);
  }

  if (!printedAny) {
    Serial.println("  none in recent window");
  }
}

void printObservedIds() {
  const uint32_t now = millis();
  Serial.print("Observed CAN IDs within ");
  Serial.print(OBSERVED_ID_WINDOW_MS);
  Serial.println(" ms:");
  printObservedCanIds("vehicle->radio observed", vehicleObservedIds, OBSERVED_ID_SLOTS, now,
                      OBSERVED_ID_WINDOW_MS);
  printObservedCanIds("radio->vehicle observed", radioObservedIds, OBSERVED_ID_SLOTS, now,
                      OBSERVED_ID_WINDOW_MS);
}

bool isRadioAwakeSignalFrame(const CAN_message_t &msg) {
  if (msg.flags.extended) {
    return false;
  }

  switch (msg.id) {
    case POWER_COMPAT_CAN_ID_436:
    case RADIO_AWAKE_CAN_ID_481:
    case RADIO_AWAKE_CAN_ID_484:
    case RADIO_AWAKE_CAN_ID_485:
    case RADIO_AWAKE_CAN_ID_486:
    case RADIO_AWAKE_CAN_ID_5A1:
    case RADIO_AWAKE_CAN_ID_5A3:
    case RADIO_AWAKE_CAN_ID_604:
    case RADIO_AWAKE_CAN_ID_661:
    case RADIO_AWAKE_CAN_ID_665:
    case RADIO_AWAKE_CAN_ID_666:
    case RADIO_AWAKE_CAN_ID_66C:
    case RADIO_AWAKE_CAN_ID_66F:
    case RADIO_AWAKE_CAN_ID_67D:
      return true;
    default:
      return false;
  }
}

bool deadlineActive(uint32_t now, uint32_t deadline) {
  return deadline != 0 && static_cast<int32_t>(deadline - now) >= 0;
}

bool clockRepeatAllowedNow(uint32_t now) {
  return deadlineActive(now, radioClockRepeatAllowedUntilMs);
}

bool clockCacheFreshNow(uint32_t now) {
  return cachedClock65dValid && now - lastClock65dVehicleMs <= CLOCK_CACHE_FRESHNESS_MS;
}

void noteRadioWakeActivity(const CAN_message_t &msg) {
  if (!isRadioAwakeSignalFrame(msg)) {
    return;
  }

  const uint32_t now = millis();
  const bool wasRepeatAllowed = clockRepeatAllowedNow(now);
  if (lastRadioWakeFrameMs == 0 || now - lastRadioWakeFrameMs > RADIO_WAKE_BURST_GAP_MS) {
    radioWakeBurstStartedMs = now;
    radioWakeBurstFrames = 1;
    radioWakeBurstNon436Frames = msg.id == POWER_COMPAT_CAN_ID_436 ? 0 : 1;
  } else {
    if (radioWakeBurstFrames < 0xFF) {
      radioWakeBurstFrames++;
    }
    if (msg.id != POWER_COMPAT_CAN_ID_436 && radioWakeBurstNon436Frames < 0xFF) {
      radioWakeBurstNon436Frames++;
    }
  }

  lastRadioWakeFrameMs = now;

  if (radioWakeBurstFrames >= RADIO_WAKE_MIN_BURST_FRAMES &&
      radioWakeBurstNon436Frames >= RADIO_WAKE_MIN_NON436_FRAMES) {
    radioClockRepeatAllowedUntilMs = now + CLOCK_REPEAT_ACTIVE_HOLD_MS;
  } else if (msg.id != POWER_COMPAT_CAN_ID_436 && radioClockRepeatAllowedUntilMs != 0 &&
             clockRepeatAllowedNow(now)) {
    radioClockRepeatAllowedUntilMs = now + CLOCK_REPEAT_ACTIVE_HOLD_MS;
  }

  if (!wasRepeatAllowed && clockRepeatAllowedNow(now)) {
    // Start each qualified radio wake with an immediate clock update.
    lastClockRepeatMs = 0;
  }
}

void sendClockRepeatToRadio() {
  if (powerState != PowerState::Active || !bridgeForwardingEnabled ||
      !clockCacheFreshNow(millis()) || cachedClock65dLen != 8) {
    return;
  }

  CAN_message_t msg = {};
  msg.id = CLOCK_DIAG_1_CAN_ID;
  msg.len = cachedClock65dLen;
  msg.flags.extended = 0;
  for (uint8_t i = 0; i < cachedClock65dLen && i < 8; ++i) {
    msg.buf[i] = cachedClock65d[i];
  }

  if (canRadio.write(msg)) {
    repeatedClock65dToRadio++;
  } else {
    failedClock65dRepeats++;
  }
  lastClockRepeatMs = millis();

  if (verboseLogging) {
    printFrame("repeat ->radio", msg);
  }
}

void tickClockRepeat() {
  if (powerState != PowerState::Active || !bridgeForwardingEnabled ||
      !clockRepeatEnabled || !cachedClock65dValid) {
    return;
  }

  const uint32_t now = millis();
  if (!clockRepeatAllowedNow(now)) {
    radioClockRepeatAllowedUntilMs = 0;
    return;
  }

  if (!clockCacheFreshNow(now)) {
    return;
  }

  if (lastClockRepeatMs == 0 || now - lastClockRepeatMs >= CLOCK_REPEAT_INTERVAL_MS) {
    sendClockRepeatToRadio();
  }
}

void resetCounters() {
  forwardedFramesVehicleToRadio = 0;
  forwardedFramesRadioToVehicle = 0;
  clock65dVehicleToRadio = 0;
  clock65dRadioToVehicle = 0;
  clock623VehicleToRadio = 0;
  clock623RadioToVehicle = 0;
  power436VehicleToRadio = 0;
  power436RadioToVehicle = 0;
  power439VehicleToRadio = 0;
  power439RadioToVehicle = 0;
  repeatedClock65dToRadio = 0;
  rejectedClock65dFrames = 0;
  failedClock65dRepeats = 0;
  runActivity2c3Frames = 0;
  runActivity575Frames = 0;
  shutdownGraceEntries = 0;
  deepSleepEntries = 0;
  deepSleepWakeups = 0;
  clearObservedCanIds(vehicleObservedIds, OBSERVED_ID_SLOTS);
  clearObservedCanIds(radioObservedIds, OBSERVED_ID_SLOTS);
}

const char *powerStateName(PowerState state) {
  switch (state) {
    case PowerState::WakeProbe: return "WakeProbe";
    case PowerState::Active: return "Active";
    case PowerState::ShutdownGrace: return "ShutdownGrace";
  }
  return "Unknown";
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h            - help");
  Serial.println("  s            - status");
  Serial.println("  ids          - show recent CAN IDs seen on each side");
  Serial.println("  repeat on/off/status - control 0x65D radio-side repeater");
  Serial.println("  power status - show power and sleep state");
  Serial.println("  power sleep on/off - control deep sleep until reboot");
  Serial.println("  power active - force active state for bench testing");
  Serial.println("  reset        - clear counters");
  Serial.println("  log          - toggle raw frame logging");
}

void printStatus() {
  Serial.println("Status:");
  Serial.print("  firmwareVersion=");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("  verboseLogging=");
  Serial.println(verboseLogging ? "on" : "off");
  Serial.print("  forwarded vehicle->radio=");
  Serial.println(forwardedFramesVehicleToRadio);
  Serial.print("  forwarded radio->vehicle=");
  Serial.println(forwardedFramesRadioToVehicle);
  Serial.print("  0x65D vehicle->radio/radio->vehicle=");
  Serial.print(clock65dVehicleToRadio);
  Serial.print("/");
  Serial.println(clock65dRadioToVehicle);
  Serial.print("  0x65D repeated->radio=");
  Serial.println(repeatedClock65dToRadio);
  Serial.print("  0x65D rejected/failed-repeat=");
  Serial.print(rejectedClock65dFrames);
  Serial.print("/");
  Serial.println(failedClock65dRepeats);
  Serial.print("  0x623 vehicle->radio/radio->vehicle=");
  Serial.print(clock623VehicleToRadio);
  Serial.print("/");
  Serial.println(clock623RadioToVehicle);
  Serial.print("  0x436 vehicle->radio/radio->vehicle=");
  Serial.print(power436VehicleToRadio);
  Serial.print("/");
  Serial.println(power436RadioToVehicle);
  Serial.print("  0x439 vehicle->radio/radio->vehicle=");
  Serial.print(power439VehicleToRadio);
  Serial.print("/");
  Serial.println(power439RadioToVehicle);
  Serial.print("  lastVehicleActivityMsAgo=");
  if (lastVehicleActivityMs == 0) {
    Serial.println("n/a");
  } else {
    Serial.println(millis() - lastVehicleActivityMs);
  }
  Serial.print("  lastRadioActivityMsAgo=");
  if (lastRadioActivityMs == 0) {
    Serial.println("n/a");
  } else {
    Serial.println(millis() - lastRadioActivityMs);
  }
  Serial.print("  clockRepeatEnabled=");
  Serial.println(clockRepeatEnabled ? "on" : "off");
  Serial.print("  cachedClock65dValid=");
  Serial.println(cachedClock65dValid ? "yes" : "no");
  Serial.print("  cachedClock65dFresh=");
  Serial.println(clockCacheFreshNow(millis()) ? "yes" : "no");
  Serial.print("  clockRepeatAllowedNow=");
  Serial.println(clockRepeatAllowedNow(millis()) ? "yes" : "no");
  Serial.print("  powerState=");
  Serial.println(powerStateName(powerState));
  Serial.print("  bridgeForwardingEnabled=");
  Serial.println(bridgeForwardingEnabled ? "yes" : "no");
  Serial.print("  deepSleepEnabled=");
  Serial.println(deepSleepEnabled ? "yes" : "no");
  Serial.print("  run 0x2C3/0x575=");
  Serial.print(runActivity2c3Frames);
  Serial.print("/");
  Serial.println(runActivity575Frames);
  Serial.print("  shutdownGraceEntries=");
  Serial.println(shutdownGraceEntries);
  Serial.print("  deepSleep entries/wakeups=");
  Serial.print(deepSleepEntries);
  Serial.print("/");
  Serial.println(deepSleepWakeups);
  Serial.print("  lastDeepSleepWakeSource=");
  Serial.println(lastDeepSleepWakeSource);
}

void processCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (cmd == nullptr) {
    return;
  }

  if (strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "s") == 0 || strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(cmd, "ids") == 0 || strcmp(cmd, "wakeids") == 0) {
    printObservedIds();
    return;
  }

  if (strcmp(cmd, "repeat") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "on") == 0) {
      clockRepeatEnabled = true;
      Serial.println("clockRepeatEnabled=on");
      return;
    }

    if (strcmp(subcommand, "off") == 0) {
      clockRepeatEnabled = false;
      radioClockRepeatAllowedUntilMs = 0;
      lastClockRepeatMs = 0;
      Serial.println("clockRepeatEnabled=off");
      return;
    }

    Serial.println("Unknown repeat command. Use: repeat on/off/status");
    return;
  }

  if (strcmp(cmd, "power") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "sleep") == 0) {
      char *setting = strtok(nullptr, " \t");
      if (setting != nullptr && strcmp(setting, "on") == 0) {
        deepSleepEnabled = true;
        Serial.println("deepSleepEnabled=on");
        return;
      }
      if (setting != nullptr && strcmp(setting, "off") == 0) {
        deepSleepEnabled = false;
        Serial.println("deepSleepEnabled=off");
        return;
      }
      Serial.println("Use: power sleep on/off");
      return;
    }

    if (strcmp(subcommand, "active") == 0) {
      lastRunActivityMs = millis();
      enterActivePowerState();
      Serial.println("Power state forced active for bench testing");
      return;
    }

    Serial.println("Unknown power command. Use: power status/sleep/active");
    return;
  }

  if (strcmp(cmd, "reset") == 0) {
    resetCounters();
    Serial.println("Counters reset");
    return;
  }

  if (strcmp(cmd, "log") == 0) {
    verboseLogging = !verboseLogging;
    Serial.print("verboseLogging=");
    Serial.println(verboseLogging ? "on" : "off");
    return;
  }

  Serial.println("Unknown command. Send 'h' for help.");
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialBuffer[serialBufferLen] = '\0';
      processCommand(serialBuffer);
      serialBufferLen = 0;
      continue;
    }

    if (static_cast<size_t>(serialBufferLen) + 1 < sizeof(serialBuffer)) {
      serialBuffer[serialBufferLen++] = c;
    } else {
      serialBufferLen = 0;
      Serial.println("Command too long");
    }
  }
}

void setupCan(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &bus) {
  bus.begin();
  bus.setBaudRate(BUS_SPEED);
  bus.enableFIFO();
}

void setupCan(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &bus) {
  bus.begin();
  bus.setBaudRate(BUS_SPEED);
  bus.enableFIFO();
}

bool isRunActivityFrame(const CAN_message_t &msg) {
  return !msg.flags.extended &&
         (msg.id == RUN_ACTIVITY_CAN_ID_2C3 || msg.id == RUN_ACTIVITY_CAN_ID_575);
}

void setTransceiverStandby(bool vehicleStandby, bool radioStandby) {
  digitalWrite(VEHICLE_TRANSCEIVER_RS_PIN, vehicleStandby ? HIGH : LOW);
  digitalWrite(RADIO_TRANSCEIVER_RS_PIN, radioStandby ? HIGH : LOW);
}

void enterActivePowerState() {
  setTransceiverStandby(false, false);
  bridgeForwardingEnabled = true;
  powerState = PowerState::Active;
  powerStateStartedMs = millis();
}

void noteRunActivity(const CAN_message_t &msg) {
  if (!isRunActivityFrame(msg)) {
    return;
  }

  lastRunActivityMs = millis();
  if (msg.id == RUN_ACTIVITY_CAN_ID_2C3) {
    runActivity2c3Frames++;
  } else {
    runActivity575Frames++;
  }

  if (powerState != PowerState::Active || !bridgeForwardingEnabled) {
    enterActivePowerState();
  }
}

void enterShutdownGrace() {
  if (powerState == PowerState::ShutdownGrace) {
    return;
  }

  // Native traffic may finish the radio's shutdown handshake, but the module
  // must stop creating clock traffic as soon as run activity disappears.
  radioClockRepeatAllowedUntilMs = 0;
  lastClockRepeatMs = 0;
  radioWakeBurstStartedMs = 0;
  lastRadioWakeFrameMs = 0;
  radioWakeBurstFrames = 0;
  radioWakeBurstNon436Frames = 0;
  bridgeForwardingEnabled = true;
  setTransceiverStandby(false, false);
  powerState = PowerState::ShutdownGrace;
  powerStateStartedMs = millis();
  shutdownGraceEntries++;
}

void enterDeepPowerSleep() {
  if (!deepSleepEnabled) {
    return;
  }

  bridgeForwardingEnabled = false;
  powerState = PowerState::WakeProbe;
  powerStateStartedMs = 0;
  lastRunActivityMs = 0;
  radioClockRepeatAllowedUntilMs = 0;
  lastClockRepeatMs = 0;
  radioWakeBurstStartedMs = 0;
  lastRadioWakeFrameMs = 0;
  radioWakeBurstFrames = 0;
  radioWakeBurstNon436Frames = 0;
  cachedClock65dValid = false;
  cachedClock65dLen = 0;

  // SN65HVD230 standby disables both drivers. The vehicle receiver remains
  // available to wake the Teensy from a dominant edge on CRX1/pin 23.
  setTransceiverStandby(true, true);
  Serial.flush();
  delay(2);

  deepSleepEntries++;
  lastDeepSleepWakeSource = Snooze.deepSleep(powerSleepConfig);
  deepSleepWakeups++;

  // Do not expose the radio to traffic until a run-state frame is observed.
  setTransceiverStandby(false, true);
  setupCan(canVehicle);
  setupCan(canRadio);
  powerState = PowerState::WakeProbe;
  powerStateStartedMs = millis();
}

void powerTick() {
  const uint32_t now = millis();

  switch (powerState) {
    case PowerState::Active:
      if (lastRunActivityMs == 0 ||
          now - lastRunActivityMs > RUN_ACTIVITY_TIMEOUT_MS) {
        enterShutdownGrace();
      }
      break;
    case PowerState::ShutdownGrace:
      if (now - powerStateStartedMs >= RADIO_SHUTDOWN_GRACE_MS) {
        enterDeepPowerSleep();
      }
      break;
    case PowerState::WakeProbe:
      if (powerStateStartedMs != 0) {
        const uint32_t timeout = deepSleepWakeups == 0 ?
                                 INITIAL_WAKE_PROBE_TIMEOUT_MS :
                                 WAKE_PROBE_TIMEOUT_MS;
        if (now - powerStateStartedMs >= timeout) {
          enterDeepPowerSleep();
        }
      }
      break;
  }
}

void handleVehicleToRadio() {
  CAN_message_t msg;
  while (canVehicle.read(msg)) {
    lastVehicleActivityMs = millis();
    noteObservedCanId(msg, vehicleObservedIds, OBSERVED_ID_SLOTS);
    noteRunActivity(msg);

    if (msg.id == CLOCK_DIAG_1_CAN_ID) {
      clock65dVehicleToRadio++;
      if (!msg.flags.extended && msg.len == sizeof(cachedClock65d)) {
        cachedClock65dLen = msg.len;
        for (uint8_t i = 0; i < msg.len; ++i) {
          cachedClock65d[i] = msg.buf[i];
        }
        cachedClock65dValid = true;
        lastClock65dVehicleMs = lastVehicleActivityMs;
      } else {
        rejectedClock65dFrames++;
      }
    } else if (msg.id == CLOCK_KOMBI_K2_CAN_ID) {
      clock623VehicleToRadio++;
    } else if (msg.id == POWER_COMPAT_CAN_ID_436) {
      power436VehicleToRadio++;
    } else if (msg.id == POWER_COMPAT_CAN_ID_439) {
      power439VehicleToRadio++;
    }

    if (!bridgeForwardingEnabled) {
      continue;
    }

    forwardedFramesVehicleToRadio++;
    if (verboseLogging) {
      printFrame("vehicle->radio", msg);
    }

    canRadio.write(msg);
  }
}

void handleRadioToVehicle() {
  CAN_message_t msg;
  while (canRadio.read(msg)) {
    lastRadioActivityMs = millis();
    noteObservedCanId(msg, radioObservedIds, OBSERVED_ID_SLOTS);

    if (msg.id == CLOCK_DIAG_1_CAN_ID) {
      clock65dRadioToVehicle++;
    } else if (msg.id == CLOCK_KOMBI_K2_CAN_ID) {
      clock623RadioToVehicle++;
    } else if (msg.id == POWER_COMPAT_CAN_ID_436) {
      power436RadioToVehicle++;
    } else if (msg.id == POWER_COMPAT_CAN_ID_439) {
      power439RadioToVehicle++;
    }

    if (!bridgeForwardingEnabled) {
      continue;
    }

    forwardedFramesRadioToVehicle++;
    if (powerState == PowerState::Active) {
      noteRadioWakeActivity(msg);
    }

    if (verboseLogging) {
      printFrame("radio->vehicle", msg);
    }

    canVehicle.write(msg);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("Mk6 MIB2 clock bridge starting");
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("Assumed bus speed: 100000");
  Serial.println("Clock focus: 0x65D / 0x623");
  Serial.println("Clock repeater: enabled, run-state and radio-awake gated");
  Serial.println("Power: radio isolated until 0x2C3/0x575; deep sleep enabled");
  Serial.println("Send 'h' over USB serial for commands.");

  pinMode(VEHICLE_TRANSCEIVER_RS_PIN, OUTPUT);
  pinMode(RADIO_TRANSCEIVER_RS_PIN, OUTPUT);
  setTransceiverStandby(false, true);
  vehicleCanWake.pinMode(VEHICLE_CAN_RX_WAKE_PIN, INPUT_PULLUP, FALLING);

  setupCan(canVehicle);
  setupCan(canRadio);
  resetCounters();
  bridgeForwardingEnabled = false;
  powerState = PowerState::WakeProbe;
  powerStateStartedMs = millis();
  printStatus();
}

void loop() {
  handleVehicleToRadio();
  handleRadioToVehicle();
  tickClockRepeat();
  powerTick();
  processSerialCommands();
}
