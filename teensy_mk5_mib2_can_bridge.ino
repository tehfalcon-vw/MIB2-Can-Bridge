#include <FlexCAN_T4.h>
#include <EEPROM.h>

/*
  Mk5 VW -> MIB2/RCD330 dimming bridge

  What this starter firmware does:
  - Bridges frames between two CAN buses
  - Rewrites CAN ID 0x635 so the radio sees a usable brightness value
  - Mirrors Mk5 dash-dimmer byte 0 into MIB2 brightness byte 2
  - This is running on a Teensy 4.0
  - Vehicle infotainment CAN is 100 kbit/s
  - CAN1 is connected to the vehicle-side transceiver
  - CAN2 is connected to the radio-side transceiver

  Important:
  - Verify the bus speed and CAN pin mapping on your exact hardware before connecting in-car
  - Verify the 0x635 payload layout on your car with logging before trusting the rewrite logic
*/

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canVehicle;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> canRadio;

static constexpr uint32_t BUS_SPEED = 100000;
static constexpr uint32_t DIMMING_CAN_ID = 0x635;
static constexpr uint8_t DIMMING_FRAME_LEN = 3;
static constexpr uint8_t DEFAULT_DAY_BRIGHTNESS = 0xFD;
static constexpr uint8_t DEFAULT_LIGHTS_ON_MAX_BRIGHTNESS = 0xD0;
static constexpr uint8_t DEFAULT_NIGHT_BRIGHTNESS = 0x20;
static constexpr uint32_t EEPROM_MAGIC = 0x4D494236;  // "MIB6" profile with separate lights-on max brightness
static constexpr int EEPROM_ADDR_MAGIC = 0;
static constexpr int EEPROM_ADDR_DAY = EEPROM_ADDR_MAGIC + sizeof(uint32_t);
static constexpr int EEPROM_ADDR_LIGHTS_ON_MAX = EEPROM_ADDR_DAY + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_NIGHT = EEPROM_ADDR_LIGHTS_ON_MAX + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_DDP_ENABLED = EEPROM_ADDR_NIGHT + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_DDP_TARGET = EEPROM_ADDR_DDP_ENABLED + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_AUDIO_TRANSLATE = EEPROM_ADDR_DDP_TARGET + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_NAV_TRANSLATE = EEPROM_ADDR_AUDIO_TRANSLATE + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_AUDIO_CACHE_VALID = EEPROM_ADDR_NAV_TRANSLATE + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_AUDIO_CACHE_LINE1 = EEPROM_ADDR_AUDIO_CACHE_VALID + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_AUDIO_CACHE_LINE2 = EEPROM_ADDR_AUDIO_CACHE_LINE1 + 17;
static constexpr int EEPROM_ADDR_STEERING_AUDIO_BUTTONS = EEPROM_ADDR_AUDIO_CACHE_LINE2 + 17;
static constexpr int EEPROM_ADDR_DIMMER_MIRROR = EEPROM_ADDR_STEERING_AUDIO_BUTTONS + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_DIMMER_SCALE = EEPROM_ADDR_DIMMER_MIRROR + sizeof(uint8_t);
static constexpr int EEPROM_ADDR_NAV_ARROW_HINTS = EEPROM_ADDR_DIMMER_SCALE + sizeof(uint8_t);
static constexpr uint8_t EEPROM_AUDIO_CACHE_VALID = 0xA5;

static constexpr uint32_t DDP_AUDIO_TO_DISPLAY_ID = 0x680;
static constexpr uint32_t DDP_AUDIO_FROM_DISPLAY_ID = 0x681;
static constexpr uint32_t DDP_NAV_TO_DISPLAY_ID = 0x682;
static constexpr uint32_t DDP_NAV_FROM_DISPLAY_ID = 0x683;
static constexpr uint8_t DDP_AUDIO_DEVICE_ID = 0x52;
static constexpr uint8_t DDP_NAV_DEVICE_ID = 0x4D;
static constexpr uint8_t DDP_AUDIO_CENTRAL_AREA_ID = 0x00;
static constexpr uint8_t DDP_NAV_CENTRAL_AREA_ID = 0x10;
static constexpr uint32_t DDP_WAIT_TIMEOUT_MS = 1500;
static constexpr uint32_t DDP_AREA_AVAILABLE_TIMEOUT_MS = 8000;
static constexpr uint32_t DDP_KEEPALIVE_INTERVAL_MS = 1000;
static constexpr uint32_t DDP_BOOT_START_DELAY_MS = 20000;
static constexpr uint32_t DDP_PERIODIC_REBIND_INTERVAL_MS = 30000;
static constexpr uint32_t DDP_COLD_START_WINDOW_MS = 60000;
static constexpr uint32_t DDP_COLD_START_DRAW_DELAY_MS = 5000;
static constexpr uint32_t DDP_SLEEP_IDLE_TIMEOUT_MS = 120000;
static constexpr uint32_t DDP_WAKE_BURST_GAP_MS = 250;
static constexpr uint32_t DDP_WAKE_PENDING_ACTIVITY_TIMEOUT_MS = 3000;
static constexpr uint32_t DDP_SESSION_STUCK_TIMEOUT_MS = 60000;
static constexpr uint8_t DDP_WAKE_MIN_BURST_FRAMES = 12;
static constexpr uint32_t AUDIO_CACHE_SAVE_INTERVAL_MS = 30000;
// DBC: PQ35 infotainment CAN names 0x66C as BAP_AUDIO from Radio_2DIN.
static constexpr uint32_t BAP_AUDIO_CAN_ID = 0x66C;
// DBC: BAP_ASG_07 is the Gateway companion/request channel used by several BAP modules.
static constexpr uint32_t BAP_ASG_07_CAN_ID = 0x67C;
// DBC: BAP_NAVI is the navigation/compass-module BAP channel.
static constexpr uint32_t BAP_NAVI_CAN_ID = 0x67D;
// DBC: PSD_02 carries predictive route/navigation data when supported by the nav source.
static constexpr uint32_t PSD_NAV_CAN_ID = 0x3A2;
static constexpr uint32_t CLOCK_KOMBI_K2_CAN_ID = 0x623;
static constexpr uint32_t CLOCK_DIAG_1_CAN_ID = 0x65D;
static constexpr uint32_t CLOCK_CANDIDATE_1_CAN_ID = 0x621;
static constexpr uint32_t CLOCK_CANDIDATE_2_CAN_ID = 0x651;
static constexpr uint32_t CLOCK_CANDIDATE_3_CAN_ID = 0x653;
static constexpr uint32_t CLOCK_CANDIDATE_4_CAN_ID = 0x655;
static constexpr uint32_t CLOCK_CANDIDATE_5_CAN_ID = 0x658;
static constexpr uint32_t CLOCK_CANDIDATE_6_CAN_ID = 0x65F;
static constexpr uint32_t CLOCK_CANDIDATE_7_CAN_ID = 0x661;
static constexpr uint32_t CLOCK_CANDIDATE_8_CAN_ID = 0x665;
static constexpr uint32_t POWER_COMPAT_CAN_ID_436 = 0x436;
static constexpr uint32_t POWER_COMPAT_CAN_ID_439 = 0x439;
static constexpr uint32_t STEERING_BUTTON_CAN_ID = 0x5C1;
static constexpr uint8_t WAKE_ID_COUNTER_SLOTS = 24;
static constexpr uint32_t WAKE_ID_RECENT_WINDOW_MS = 3000;
static constexpr uint32_t BAP_LOG_MIN_CAN_ID = 0x660;
static constexpr uint32_t BAP_LOG_MAX_CAN_ID = 0x66F;
static constexpr uint8_t BAP_RX_CHANNELS = 4;
static constexpr uint8_t BAP_PAYLOAD_MAX = 72;
static constexpr uint8_t AUDIO_LINE_MAX = 17;
static constexpr uint8_t STEERING_BUTTON_CLEAR = 0x00;
static constexpr uint8_t STEERING_BUTTON_MFD_UP = 0x22;
static constexpr uint8_t STEERING_BUTTON_MFD_DOWN = 0x23;
static constexpr uint8_t STEERING_BUTTON_RADIO_NEXT = 0x02;
static constexpr uint8_t STEERING_BUTTON_RADIO_PREV = 0x03;
static constexpr bool DEFAULT_DDP_BOOT_ENABLED = true;
static constexpr bool DEFAULT_AUDIO_TRANSLATE_ENABLED = true;
static constexpr bool DEFAULT_NAV_TRANSLATE_ENABLED = true;
static constexpr bool DEFAULT_NAV_ARROW_HINTS_ENABLED = false;
static constexpr bool DEFAULT_STEERING_AUDIO_BUTTONS_ENABLED = true;
static constexpr bool DEFAULT_DIMMER_MIRROR_ENABLED = false;
static constexpr bool DEFAULT_DIMMER_SCALE_ENABLED = true;
static constexpr uint8_t DIMMER_WHEEL_MIN_OBSERVED = 0x1A;
static constexpr uint8_t DIMMER_WHEEL_MAX_OBSERVED = 0x64;

// Tunable defaults. These are runtime-adjustable over USB serial.
static uint8_t dayBrightness = DEFAULT_DAY_BRIGHTNESS;
static uint8_t lightsOnMaxBrightness = DEFAULT_LIGHTS_ON_MAX_BRIGHTNESS;
static uint8_t nightBrightness = DEFAULT_NIGHT_BRIGHTNESS;

// Observed Mk5/RCD330 behavior:
// - 0x635 [3] 00 00 00 appears when lights are off, but MIB2/RCD330 treats byte 2 as min brightness.
// - 0x635 [3] NN 00 00 appears when lights are on, where NN follows the dash dimmer.
// The reference RCD330 behavior leaves nonzero dimmer frames unchanged so the radio can follow byte 0.
// Mirror and scale modes are runtime fallbacks if a specific MIB2/RCD330 unit needs byte 2 populated.
static constexpr uint8_t BRIGHTNESS_BYTE_INDEX = 2;

static bool verboseLogging = false;
static bool bypassMode = false;
static uint32_t patchedFrames = 0;
static uint32_t forwardedFramesVehicleToRadio = 0;
static uint32_t forwardedFramesRadioToVehicle = 0;

char serialBuffer[64];
uint8_t serialBufferLen = 0;

enum class DdpState : uint8_t {
  Disabled,
  SendConnect,
  WaitConnect,
  SendIdentify,
  WaitIdentifyAck,
  WaitDisplayParams,
  SendAreaRequest,
  WaitAreaRequestAck,
  WaitAreaParams,
  SendBindAudio,
  WaitBindAck,
  WaitBindResponse,
  SendShowArea,
  WaitShowAck,
  WaitAreaAvailable,
  SendDrawTest,
  WaitDrawAck,
  Ready,
  Error
};

enum class DdpTarget : uint8_t {
  Audio,
  Navigation
};

static bool ddpEnabled = false;
static bool ddpBootEnabled = DEFAULT_DDP_BOOT_ENABLED;
static bool ddpVerbose = false;
static DdpTarget ddpTarget = DdpTarget::Audio;
static DdpTarget ddpRenderedTarget = DdpTarget::Audio;
static DdpState ddpState = DdpState::Disabled;
static uint32_t ddpStateStartedMs = 0;
static uint32_t ddpLastKeepaliveMs = 0;
static uint8_t ddpTxSeq = 0;
static uint8_t ddpExpectedAck = 0;
static uint8_t ddpRxPayload[80];
static uint8_t ddpRxPayloadLen = 0;
static uint32_t ddpTxFrames = 0;
static uint32_t ddpRxFrames = 0;
static uint32_t ddpAcksSent = 0;
static uint32_t ddpAcksReceived = 0;
static uint32_t ddpDrawsCompleted = 0;
static bool ddpRedrawPending = false;
static bool ddpForceRedrawPending = false;
static uint32_t ddpSessionStartedMs = 0;
static uint32_t ddpEarliestDrawMs = 0;
static uint32_t ddpNextPeriodicRebindMs = 0;
static uint32_t ddpBootStartMs = 0;
static bool ddpBootStartPending = false;
static bool ddpRenderedPageValid = false;
static bool ddpSleepSuspended = false;
static uint32_t lastNonDdpCanActivityMs = 0;
static uint32_t vehicleWakeBurstStartedMs = 0;
static uint32_t lastVehicleWakeFrameMs = 0;
static uint8_t vehicleWakeBurstFrames = 0;

struct BapRxChannel {
  bool active;
  uint32_t canId;
  uint16_t len;
  uint16_t done;
  uint8_t opcode;
  uint8_t node;
  uint8_t port;
  uint8_t data[BAP_PAYLOAD_MAX];
};

struct BapDecodedFrame {
  uint32_t canId;
  bool multiframe;
  uint8_t opcode;
  uint8_t node;
  uint8_t port;
  uint16_t len;
  uint8_t data[BAP_PAYLOAD_MAX];
};

struct NavManeuverDescriptorCandidate {
  bool valid;
  uint8_t offset;
  uint8_t mainElement;
  uint8_t direction;
  uint8_t zLevel;
  uint8_t sidestreetLen;
  int8_t score;
};

struct CanIdActivityCounter {
  uint32_t id;
  uint32_t count;
  uint32_t lastSeenMs;
};

static BapRxChannel bapChannels[BAP_RX_CHANNELS];
static bool bapLogging = false;
static bool bapLogAllCandidates = false;
static bool audioSniffMode = false;
static bool navSniffMode = false;
static bool clockSniffMode = false;
static bool audioTranslateEnabled = DEFAULT_AUDIO_TRANSLATE_ENABLED;
static bool navTranslateEnabled = DEFAULT_NAV_TRANSLATE_ENABLED;
static bool navArrowHintsEnabled = DEFAULT_NAV_ARROW_HINTS_ENABLED;
static bool steeringAudioButtonsEnabled = DEFAULT_STEERING_AUDIO_BUTTONS_ENABLED;
static bool dimmerMirrorEnabled = DEFAULT_DIMMER_MIRROR_ENABLED;
static bool dimmerScaleEnabled = DEFAULT_DIMMER_SCALE_ENABLED;
static bool steeringButtonLogging = false;
static bool steeringTranslatedActive = false;
static bool navShowManeuverCodes = false;
static bool navRouteActive = false;
static uint32_t bapFramesDecoded = 0;
static uint32_t audioTextUpdates = 0;
static uint32_t navTextUpdates = 0;
static uint32_t navSniffFrames = 0;
static uint32_t clockSniffFrames = 0;
static uint32_t steeringNextTranslations = 0;
static uint32_t steeringPrevTranslations = 0;
static uint32_t powerCompat436VehicleToRadio = 0;
static uint32_t powerCompat439VehicleToRadio = 0;
static uint32_t powerCompat436RadioToVehicle = 0;
static uint32_t powerCompat439RadioToVehicle = 0;
static CanIdActivityCounter vehicleObservedIds[WAKE_ID_COUNTER_SLOTS] = {};
static CanIdActivityCounter radioObservedIds[WAKE_ID_COUNTER_SLOTS] = {};
static char audioLine1[AUDIO_LINE_MAX] = "AUDIO";
static char audioLine2[AUDIO_LINE_MAX] = "Waiting radio";
static char lastFrequencyText[AUDIO_LINE_MAX] = "";
static bool audioLineCacheDirty = false;
static uint32_t lastAudioLineCacheSaveMs = 0;
static char navLine1[AUDIO_LINE_MAX] = "NAV";
static char navLine2[AUDIO_LINE_MAX] = "Waiting route";
static char navDistanceText[AUDIO_LINE_MAX] = "";
static char navManeuverText[AUDIO_LINE_MAX] = "";
static char navCurrentStreetText[AUDIO_LINE_MAX] = "";
static char navNextStreetText[AUDIO_LINE_MAX] = "";
static uint8_t navManeuverCode = 0xFF;
static uint8_t navManeuverKind = 0xFF;
static uint8_t navManeuverMainElement = 0xFF;
static uint8_t navManeuverDirection = 0xFF;
static uint8_t navManeuverZLevel = 0xFF;
static uint8_t navManeuverRecordOffset = 0xFF;
static uint32_t navManeuverUpdates = 0;

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

const char *ddpStateName(DdpState state) {
  switch (state) {
    case DdpState::Disabled: return "Disabled";
    case DdpState::SendConnect: return "SendConnect";
    case DdpState::WaitConnect: return "WaitConnect";
    case DdpState::SendIdentify: return "SendIdentify";
    case DdpState::WaitIdentifyAck: return "WaitIdentifyAck";
    case DdpState::WaitDisplayParams: return "WaitDisplayParams";
    case DdpState::SendAreaRequest: return "SendAreaRequest";
    case DdpState::WaitAreaRequestAck: return "WaitAreaRequestAck";
    case DdpState::WaitAreaParams: return "WaitAreaParams";
    case DdpState::SendBindAudio: return "SendBindAudio";
    case DdpState::WaitBindAck: return "WaitBindAck";
    case DdpState::WaitBindResponse: return "WaitBindResponse";
    case DdpState::SendShowArea: return "SendShowArea";
    case DdpState::WaitShowAck: return "WaitShowAck";
    case DdpState::WaitAreaAvailable: return "WaitAreaAvailable";
    case DdpState::SendDrawTest: return "SendDrawTest";
    case DdpState::WaitDrawAck: return "WaitDrawAck";
    case DdpState::Ready: return "Ready";
    case DdpState::Error: return "Error";
  }
  return "Unknown";
}

const char *ddpTargetName(DdpTarget target) {
  switch (target) {
    case DdpTarget::Audio: return "Audio";
    case DdpTarget::Navigation: return "Navigation";
  }
  return "Unknown";
}

uint32_t ddpTxCanId() {
  switch (ddpTarget) {
    case DdpTarget::Audio: return DDP_AUDIO_TO_DISPLAY_ID;
    case DdpTarget::Navigation: return DDP_NAV_TO_DISPLAY_ID;
  }
  return DDP_AUDIO_TO_DISPLAY_ID;
}

uint32_t ddpRxCanId() {
  switch (ddpTarget) {
    case DdpTarget::Audio: return DDP_AUDIO_FROM_DISPLAY_ID;
    case DdpTarget::Navigation: return DDP_NAV_FROM_DISPLAY_ID;
  }
  return DDP_AUDIO_FROM_DISPLAY_ID;
}

uint8_t ddpDeviceId() {
  switch (ddpTarget) {
    case DdpTarget::Audio: return DDP_AUDIO_DEVICE_ID;
    case DdpTarget::Navigation: return DDP_NAV_DEVICE_ID;
  }
  return DDP_AUDIO_DEVICE_ID;
}

uint8_t ddpCentralAreaId() {
  switch (ddpTarget) {
    case DdpTarget::Audio: return DDP_AUDIO_CENTRAL_AREA_ID;
    case DdpTarget::Navigation: return DDP_NAV_CENTRAL_AREA_ID;
  }
  return DDP_AUDIO_CENTRAL_AREA_ID;
}

const char *ddpPageTitle() {
  switch (ddpTarget) {
    case DdpTarget::Audio: return "Audio";
    case DdpTarget::Navigation: return "Nav";
  }
  return "Audio";
}

void copyAudioLine(char *dest, const char *source) {
  uint8_t i = 0;
  while (source[i] != '\0' && i + 1 < AUDIO_LINE_MAX) {
    dest[i] = source[i];
    i++;
  }
  dest[i] = '\0';
}

bool isDefaultAudioWaitingText() {
  return strcmp(audioLine1, "AUDIO") == 0 && strcmp(audioLine2, "Waiting radio") == 0;
}

bool isPrintableCachedLine(const char *line) {
  for (uint8_t i = 0; i < AUDIO_LINE_MAX && line[i] != '\0'; ++i) {
    const char c = line[i];
    if (c < 0x20 || c > 0x7E) {
      return false;
    }
  }
  return true;
}

void writeEepromLine(int address, const char *line) {
  for (uint8_t i = 0; i < AUDIO_LINE_MAX; ++i) {
    const char c = line[i];
    EEPROM.write(address + i, static_cast<uint8_t>(c));
    if (c == '\0') {
      for (uint8_t j = i + 1; j < AUDIO_LINE_MAX; ++j) {
        EEPROM.write(address + j, 0);
      }
      return;
    }
  }
  EEPROM.write(address + AUDIO_LINE_MAX - 1, 0);
}

void readEepromLine(int address, char *line) {
  for (uint8_t i = 0; i + 1 < AUDIO_LINE_MAX; ++i) {
    line[i] = static_cast<char>(EEPROM.read(address + i));
  }
  line[AUDIO_LINE_MAX - 1] = '\0';
}

void saveAudioLineCache() {
  if (isDefaultAudioWaitingText()) {
    return;
  }

  EEPROM.write(EEPROM_ADDR_AUDIO_CACHE_VALID, EEPROM_AUDIO_CACHE_VALID);
  writeEepromLine(EEPROM_ADDR_AUDIO_CACHE_LINE1, audioLine1);
  writeEepromLine(EEPROM_ADDR_AUDIO_CACHE_LINE2, audioLine2);
  audioLineCacheDirty = false;
  lastAudioLineCacheSaveMs = millis();
}

void scheduleAudioLineCacheSave() {
  if (isDefaultAudioWaitingText()) {
    return;
  }

  const bool sourceOnlyLine = audioLine1[0] != '\0' && audioLine2[0] == '\0';
  if (sourceOnlyLine || lastAudioLineCacheSaveMs == 0 || millis() - lastAudioLineCacheSaveMs >= AUDIO_CACHE_SAVE_INTERVAL_MS) {
    saveAudioLineCache();
  } else {
    audioLineCacheDirty = true;
  }
}

void tickAudioLineCacheSave() {
  if (!audioLineCacheDirty || millis() - lastAudioLineCacheSaveMs < AUDIO_CACHE_SAVE_INTERVAL_MS) {
    return;
  }
  saveAudioLineCache();
}

void loadAudioLineCache() {
  if (EEPROM.read(EEPROM_ADDR_AUDIO_CACHE_VALID) != EEPROM_AUDIO_CACHE_VALID) {
    return;
  }

  char cachedLine1[AUDIO_LINE_MAX] = {};
  char cachedLine2[AUDIO_LINE_MAX] = {};
  readEepromLine(EEPROM_ADDR_AUDIO_CACHE_LINE1, cachedLine1);
  readEepromLine(EEPROM_ADDR_AUDIO_CACHE_LINE2, cachedLine2);

  if (cachedLine1[0] == '\0' || !isPrintableCachedLine(cachedLine1) || !isPrintableCachedLine(cachedLine2)) {
    return;
  }

  copyAudioLine(audioLine1, cachedLine1);
  copyAudioLine(audioLine2, cachedLine2);
  if (isFrequencyLikeText(audioLine2)) {
    copyAudioLine(lastFrequencyText, audioLine2);
  } else {
    copyAudioLine(lastFrequencyText, "");
  }
}

bool audioPageActivelyShowing() {
  return ddpEnabled &&
         ddpState == DdpState::Ready &&
         ddpRenderedPageValid &&
         ddpTarget == DdpTarget::Audio &&
         ddpRenderedTarget == DdpTarget::Audio &&
         !navRouteActive;
}

bool audioPeriodicRefreshAllowed() {
  return ddpEnabled &&
         ddpState == DdpState::Ready &&
         ddpTarget == DdpTarget::Audio &&
         !navRouteActive;
}

bool navPageActivelyShowing() {
  return ddpEnabled &&
         ddpState == DdpState::Ready &&
         ddpRenderedPageValid &&
         ddpTarget == DdpTarget::Navigation &&
         ddpRenderedTarget == DdpTarget::Navigation &&
         navRouteActive;
}

bool navPeriodicRefreshAllowed() {
  return ddpEnabled &&
         ddpState == DdpState::Ready &&
         ddpTarget == DdpTarget::Navigation &&
         navRouteActive;
}

bool isSteeringButtonFrame(const CAN_message_t &msg) {
  return msg.id == STEERING_BUTTON_CAN_ID && msg.len >= 1 && !msg.flags.extended;
}

void forceSteeringButtonFrameLen(CAN_message_t &msg) {
  const uint8_t button = msg.buf[0];
  msg.len = 8;
  msg.buf[0] = button;
  for (uint8_t i = 1; i < 8; ++i) {
    msg.buf[i] = 0x00;
  }
}

bool rewriteSteeringButtonsForAudioPage(CAN_message_t &msg) {
  if (!steeringAudioButtonsEnabled || !isSteeringButtonFrame(msg)) {
    return false;
  }

  const uint8_t originalButton = msg.buf[0];
  if (originalButton == STEERING_BUTTON_CLEAR && steeringTranslatedActive) {
    forceSteeringButtonFrameLen(msg);
    steeringTranslatedActive = false;
    if (steeringButtonLogging) {
      Serial.println("steering audio clear");
    }
    return true;
  }

  if (!audioPageActivelyShowing()) {
    return false;
  }

  if (originalButton == STEERING_BUTTON_MFD_UP) {
    msg.buf[0] = STEERING_BUTTON_RADIO_NEXT;
    forceSteeringButtonFrameLen(msg);
    steeringTranslatedActive = true;
    steeringNextTranslations++;
    if (steeringButtonLogging) {
      Serial.println("steering audio: MFD up -> radio next");
    }
    return true;
  }

  if (originalButton == STEERING_BUTTON_MFD_DOWN) {
    msg.buf[0] = STEERING_BUTTON_RADIO_PREV;
    forceSteeringButtonFrameLen(msg);
    steeringTranslatedActive = true;
    steeringPrevTranslations++;
    if (steeringButtonLogging) {
      Serial.println("steering audio: MFD down -> radio previous");
    }
    return true;
  }

  return false;
}

void requestDdpRedraw() {
  ddpRedrawPending = true;
}

void requestDdpForceRedraw() {
  ddpRedrawPending = true;
  ddpForceRedrawPending = true;
}

void ddpSetState(DdpState state) {
  ddpState = state;
  ddpStateStartedMs = millis();
  if (ddpVerbose) {
    Serial.print("DDP state=");
    Serial.println(ddpStateName(ddpState));
  }
}

void completeDdpDraw(const char *reason) {
  if (ddpState != DdpState::WaitDrawAck) {
    return;
  }

  ddpDrawsCompleted++;
  ddpRenderedTarget = ddpTarget;
  ddpRenderedPageValid = true;
  ddpNextPeriodicRebindMs = millis() + DDP_PERIODIC_REBIND_INTERVAL_MS;
  if (ddpVerbose) {
    Serial.print("DDP draw accepted by ");
    Serial.println(reason);
  }
  ddpSetState(DdpState::Ready);
}

bool ddpColdStartActive() {
  return millis() - ddpSessionStartedMs < DDP_COLD_START_WINDOW_MS;
}

void scheduleDdpBootStart() {
  ddpEnabled = false;
  ddpBootStartPending = true;
  ddpSleepSuspended = false;
  ddpBootStartMs = millis() + DDP_BOOT_START_DELAY_MS;
  ddpSetState(DdpState::Disabled);
}

void suspendDdpForSleep() {
  ddpEnabled = false;
  ddpBootStartPending = false;
  ddpRedrawPending = false;
  ddpForceRedrawPending = false;
  ddpRxPayloadLen = 0;
  ddpBootStartMs = 0;
  ddpNextPeriodicRebindMs = 0;
  ddpRenderedPageValid = false;
  ddpSleepSuspended = true;
  vehicleWakeBurstStartedMs = 0;
  lastVehicleWakeFrameMs = 0;
  vehicleWakeBurstFrames = 0;
  ddpSetState(DdpState::Disabled);
}

void writeVehicleFrame(uint32_t id, uint8_t len, const uint8_t *data) {
  CAN_message_t msg = {};
  msg.id = id;
  msg.len = len;
  msg.flags.extended = 0;
  for (uint8_t i = 0; i < len && i < 8; ++i) {
    msg.buf[i] = data[i];
  }
  canVehicle.write(msg);
}

void sendDdpControl(const uint8_t *data, uint8_t len) {
  writeVehicleFrame(ddpTxCanId(), len, data);
  ddpTxFrames++;

  if (ddpVerbose) {
    CAN_message_t msg = {};
    msg.id = ddpTxCanId();
    msg.len = len;
    for (uint8_t i = 0; i < len && i < 8; ++i) {
      msg.buf[i] = data[i];
    }
    printFrame("ddp tx control", msg);
  }
}

void sendDdpAck(uint8_t nextSequence) {
  uint8_t data[] = { static_cast<uint8_t>(0xB0 | (nextSequence & 0x0F)) };
  sendDdpControl(data, sizeof(data));
  ddpAcksSent++;
}

void sendDdpConnectRequest() {
  const uint8_t data[] = { 0xA0, 0x04, 0x59, 0xFF, 0x32, 0xFF };
  sendDdpControl(data, sizeof(data));
}

void sendDdpConnectionResponse() {
  const uint8_t data[] = { 0xA1, 0x04, 0x59, 0xFF, 0x32, 0xFF };
  sendDdpControl(data, sizeof(data));
}

void sendDdpKeepalive() {
  const uint8_t data[] = { 0xA3 };
  sendDdpControl(data, sizeof(data));
  ddpLastKeepaliveMs = millis();
}

uint8_t sendTpPayload(const uint8_t *payload, uint8_t len) {
  uint8_t offset = 0;
  uint8_t lastExpectedAck = 0;

  while (offset < len) {
    const uint8_t remaining = len - offset;
    const uint8_t chunk = remaining > 7 ? 7 : remaining;
    const bool lastFrame = (offset + chunk) >= len;
    uint8_t frame[8] = {};

    // Use no-ack continuation frames and an ack-required final frame.
    frame[0] = static_cast<uint8_t>((lastFrame ? 0x10 : 0x20) | (ddpTxSeq & 0x0F));
    for (uint8_t i = 0; i < chunk; ++i) {
      frame[i + 1] = payload[offset + i];
    }

    writeVehicleFrame(ddpTxCanId(), chunk + 1, frame);
    ddpTxFrames++;

    if (ddpVerbose) {
      CAN_message_t msg = {};
      msg.id = ddpTxCanId();
      msg.len = chunk + 1;
      for (uint8_t i = 0; i < msg.len; ++i) {
        msg.buf[i] = frame[i];
      }
      printFrame("ddp tx data   ", msg);
    }

    ddpTxSeq = (ddpTxSeq + 1) & 0x0F;
    offset += chunk;
    lastExpectedAck = static_cast<uint8_t>(0xB0 | (ddpTxSeq & 0x0F));
    delay(5);
  }

  ddpExpectedAck = lastExpectedAck;
  return ddpExpectedAck;
}

void sendDdpIdentifyRadio() {
  const uint8_t payload[] = { 0x00, ddpDeviceId(), 0x00, 0xFF };
  sendTpPayload(payload, sizeof(payload));
}

void sendDdpCentralAreaRequest() {
  const uint8_t payload[] = { 0x01, 0x12 };
  sendTpPayload(payload, sizeof(payload));
}

void sendDdpBindAudioArea() {
  uint8_t payload[14] = {
    0x02, 0x70, ddpDeviceId(), 0x12,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  const char *title = ddpPageTitle();
  uint8_t titleLen = 0;
  while (title[titleLen] != '\0' && titleLen < 10) {
    payload[4 + titleLen] = static_cast<uint8_t>(title[titleLen]);
    titleLen++;
  }
  sendTpPayload(payload, static_cast<uint8_t>(4 + titleLen));
}

void sendDdpShowCentralArea() {
  const uint8_t payload[] = { 0x0C, ddpCentralAreaId() };
  sendTpPayload(payload, sizeof(payload));
}

uint8_t appendDdpClear(uint8_t *payload, uint8_t offset) {
  payload[offset++] = 0x60;
  payload[offset++] = 0x09;
  payload[offset++] = 0x00;
  payload[offset++] = 0x00;
  payload[offset++] = 0x00;
  payload[offset++] = 0x00;
  payload[offset++] = 0x00;
  payload[offset++] = 0x6E;
  payload[offset++] = 0x00;
  payload[offset++] = 0x5B;
  payload[offset++] = 0x00;
  return offset;
}

uint8_t appendDdpText(uint8_t *payload, uint8_t offset, uint8_t x, uint8_t y, const char *text) {
  uint8_t textLen = 0;
  while (text[textLen] != '\0' && textLen < 16) {
    textLen++;
  }

  payload[offset++] = 0x61;
  payload[offset++] = 0x00;
  payload[offset++] = 0x00;
  payload[offset++] = 0x00;
  payload[offset++] = x;
  payload[offset++] = 0x00;
  payload[offset++] = y;
  payload[offset++] = 0x00;

  payload[offset - 7] = static_cast<uint8_t>(6 + textLen);

  for (uint8_t i = 0; i < textLen && offset < BAP_PAYLOAD_MAX; ++i) {
    payload[offset++] = static_cast<uint8_t>(text[i]);
  }
  return offset;
}

void sendDdpDraw() {
  uint8_t payload[BAP_PAYLOAD_MAX] = {};
  uint8_t offset = 0;
  payload[offset++] = 0x09;
  payload[offset++] = ddpCentralAreaId();
  offset = appendDdpClear(payload, offset);

  switch (ddpTarget) {
    case DdpTarget::Navigation:
      offset = appendDdpText(payload, offset, 0x03, 0x18, navLine1);
      offset = appendDdpText(payload, offset, 0x03, 0x34, navLine2);
      break;
    case DdpTarget::Audio:
    default:
      offset = appendDdpText(payload, offset, 0x03, 0x18, audioLine1);
      offset = appendDdpText(payload, offset, 0x03, 0x34, audioLine2);
      break;
  }
  payload[offset++] = 0x08;

  sendTpPayload(payload, offset);
}

void ddpResetCounters() {
  ddpTxFrames = 0;
  ddpRxFrames = 0;
  ddpAcksSent = 0;
  ddpAcksReceived = 0;
  ddpDrawsCompleted = 0;
}

void ddpStart() {
  ddpEnabled = true;
  ddpBootEnabled = true;
  ddpBootStartPending = false;
  ddpSleepSuspended = false;
  ddpRedrawPending = false;
  ddpForceRedrawPending = false;
  ddpTxSeq = 0;
  ddpExpectedAck = 0;
  ddpRxPayloadLen = 0;
  ddpLastKeepaliveMs = 0;
  ddpSessionStartedMs = millis();
  ddpEarliestDrawMs = 0;
  ddpNextPeriodicRebindMs = 0;
  ddpBootStartMs = 0;
  ddpRenderedPageValid = false;
  ddpResetCounters();
  ddpSetState(DdpState::SendConnect);
}

void selectDdpTarget(DdpTarget target, bool persistSetting) {
  if (ddpTarget == target) {
    if (persistSetting) {
      saveSettings();
    }
    return;
  }

  ddpTarget = target;
  if (ddpEnabled) {
    ddpStart();
  }
  if (persistSetting) {
    saveSettings();
  }
}

void applyAutomaticDdpPriority() {
  if (!ddpEnabled) {
    return;
  }

  const DdpTarget desiredTarget = navRouteActive ? DdpTarget::Navigation : DdpTarget::Audio;
  selectDdpTarget(desiredTarget, false);
}

void ddpStop() {
  ddpEnabled = false;
  ddpBootEnabled = false;
  ddpBootStartPending = false;
  ddpSleepSuspended = false;
  ddpRedrawPending = false;
  ddpForceRedrawPending = false;
  ddpRxPayloadLen = 0;
  ddpBootStartMs = 0;
  ddpNextPeriodicRebindMs = 0;
  ddpRenderedPageValid = false;
  ddpSetState(DdpState::Disabled);
}

void processDdpPayload(const uint8_t *payload, uint8_t len) {
  if (len == 0) {
    return;
  }

  if (ddpVerbose) {
    Serial.print("DDP payload rx:");
    for (uint8_t i = 0; i < len; ++i) {
      Serial.print(' ');
      if (payload[i] < 0x10) {
        Serial.print('0');
      }
      Serial.print(payload[i], HEX);
    }
    Serial.println();
  }

  const uint8_t command = payload[0];

  if (ddpState == DdpState::WaitDisplayParams) {
    ddpSetState(DdpState::SendAreaRequest);
    return;
  }

  if (ddpState == DdpState::WaitAreaParams) {
    ddpSetState(DdpState::SendBindAudio);
    return;
  }

  if (command == 0x23 && len >= 4) {
    const uint8_t areaId = payload[1];
    const bool areaAvailable = payload[2] == 0x01;

    if (ddpVerbose) {
      Serial.print("DDP area 0x");
      Serial.print(areaId, HEX);
      Serial.print(areaAvailable ? " available" : " unavailable");
      Serial.println();
    }

    if (ddpState == DdpState::Ready && areaId == ddpCentralAreaId() && !areaAvailable) {
      ddpRenderedPageValid = false;
    }

    if (ddpState == DdpState::WaitBindAck || ddpState == DdpState::WaitBindResponse) {
      ddpSetState(DdpState::SendShowArea);
      return;
    }

    if (ddpState == DdpState::WaitAreaAvailable && areaId == ddpCentralAreaId()) {
      if (areaAvailable) {
        ddpEarliestDrawMs = ddpColdStartActive() ? millis() + DDP_COLD_START_DRAW_DELAY_MS : 0;
        ddpSetState(DdpState::SendDrawTest);
      }
      return;
    }

    if (ddpState == DdpState::WaitDrawAck && areaId == ddpCentralAreaId()) {
      completeDdpDraw(areaAvailable ? "area-available status" : "area-status response");
      return;
    }
  }

  if (command == 0x27 && len >= 3 && payload[1] == ddpCentralAreaId()) {
    completeDdpDraw("render-complete payload");
  }
}

bool handleDdpVehicleFrame(const CAN_message_t &msg) {
  if (!ddpEnabled || msg.id != ddpRxCanId() || msg.flags.extended) {
    return false;
  }

  ddpRxFrames++;
  if (ddpVerbose) {
    printFrame("ddp rx        ", msg);
  }

  if (msg.len == 0) {
    return true;
  }

  const uint8_t control = msg.buf[0];
  const uint8_t highNibble = control & 0xF0;
  const uint8_t sequence = control & 0x0F;

  if (control == 0xA1) {
    if (ddpState == DdpState::WaitConnect) {
      ddpSetState(DdpState::SendIdentify);
    }
    return true;
  }

  if (control == 0xA3) {
    completeDdpDraw("post-draw keepalive");
    sendDdpConnectionResponse();
    return true;
  }

  if (control == 0xA8) {
    ddpSetState(DdpState::Error);
    return true;
  }

  if (highNibble == 0xB0) {
    ddpAcksReceived++;
    if (control != ddpExpectedAck && ddpVerbose) {
      Serial.print("DDP unexpected ack: got 0x");
      Serial.print(control, HEX);
      Serial.print(" expected 0x");
      Serial.println(ddpExpectedAck, HEX);
    }

    switch (ddpState) {
      case DdpState::WaitIdentifyAck:
        ddpSetState(DdpState::WaitDisplayParams);
        break;
      case DdpState::WaitAreaRequestAck:
        ddpSetState(DdpState::WaitAreaParams);
        break;
      case DdpState::WaitBindAck:
        ddpSetState(DdpState::WaitBindResponse);
        break;
      case DdpState::WaitShowAck:
        // The cluster can ACK the show command before the central area is ready.
        // Wait for the follow-up area-available status so the first draw is not
        // accepted by transport but dropped by the MFD.
        ddpSetState(DdpState::WaitAreaAvailable);
        break;
      case DdpState::WaitDrawAck:
        // The render-complete payload often follows, but some clusters only ACK
        // the transfer. Treat the ACK as enough to avoid getting stuck during
        // rapid audio/RDS redraws.
        completeDdpDraw("transport ack");
        break;
      default:
        break;
    }
    return true;
  }

  if (highNibble == 0x90) {
    if (ddpState == DdpState::WaitDrawAck) {
      ddpAcksReceived++;
      completeDdpDraw("transport control 0x9x");
    }
    return true;
  }

  if (highNibble == 0x00 || highNibble == 0x10 || highNibble == 0x20 || highNibble == 0x30) {
    const bool ackRequired = highNibble == 0x00 || highNibble == 0x10;
    const bool lastFrame = highNibble == 0x10 || highNibble == 0x30;

    for (uint8_t i = 1; i < msg.len && ddpRxPayloadLen < sizeof(ddpRxPayload); ++i) {
      ddpRxPayload[ddpRxPayloadLen++] = msg.buf[i];
    }

    if (ackRequired) {
      sendDdpAck((sequence + 1) & 0x0F);
    }

    if (lastFrame) {
      processDdpPayload(ddpRxPayload, ddpRxPayloadLen);
      ddpRxPayloadLen = 0;
    }
    return true;
  }

  return true;
}

void ddpTick() {
  const uint32_t now = millis();

  if (ddpBootEnabled && ddpSleepSuspended && hasQualifiedVehicleWakeBurst(now)) {
    scheduleDdpBootStart();
    return;
  }

  if ((ddpEnabled || ddpBootStartPending) && lastNonDdpCanActivityMs != 0 &&
      now - lastNonDdpCanActivityMs > DDP_SLEEP_IDLE_TIMEOUT_MS) {
    if (ddpVerbose) {
      Serial.println("DDP sleep suspend: CAN idle");
    }
    suspendDdpForSleep();
    return;
  }

  if (!ddpEnabled && ddpBootStartPending) {
    if (lastNonDdpCanActivityMs == 0 ||
        now - lastNonDdpCanActivityMs > DDP_WAKE_PENDING_ACTIVITY_TIMEOUT_MS) {
      if (ddpVerbose) {
        Serial.println("DDP delayed boot canceled: wake activity stopped");
      }
      suspendDdpForSleep();
      return;
    }
    if (now >= ddpBootStartMs) {
      if (ddpVerbose) {
        Serial.println("DDP delayed boot start");
      }
      ddpStart();
    }
    return;
  }

  if (!ddpEnabled) {
    return;
  }

  if (!ddpRenderedPageValid && now - ddpSessionStartedMs > DDP_SESSION_STUCK_TIMEOUT_MS) {
    if (ddpVerbose) {
      Serial.println("DDP session stuck without rendered page; suspending");
    }
    suspendDdpForSleep();
    return;
  }

  applyAutomaticDdpPriority();

  if (ddpState != DdpState::Ready &&
      ddpState != DdpState::SendConnect &&
      ddpState != DdpState::SendIdentify &&
      ddpState != DdpState::SendAreaRequest &&
      ddpState != DdpState::SendBindAudio &&
      ddpState != DdpState::SendShowArea &&
      ddpState != DdpState::SendDrawTest &&
      ddpState != DdpState::Error &&
      now - ddpStateStartedMs > (ddpState == DdpState::WaitAreaAvailable ? DDP_AREA_AVAILABLE_TIMEOUT_MS : DDP_WAIT_TIMEOUT_MS)) {
    if (ddpVerbose) {
      Serial.println("DDP timeout; returning to connect");
    }
    ddpSetState(DdpState::SendConnect);
  }

  switch (ddpState) {
    case DdpState::SendConnect:
      ddpTxSeq = 0;
      sendDdpConnectRequest();
      ddpSetState(DdpState::WaitConnect);
      break;
    case DdpState::SendIdentify:
      sendDdpIdentifyRadio();
      ddpSetState(DdpState::WaitIdentifyAck);
      break;
    case DdpState::SendAreaRequest:
      sendDdpCentralAreaRequest();
      ddpSetState(DdpState::WaitAreaRequestAck);
      break;
    case DdpState::SendBindAudio:
      sendDdpBindAudioArea();
      ddpSetState(DdpState::WaitBindAck);
      break;
    case DdpState::SendShowArea:
      sendDdpShowCentralArea();
      ddpSetState(DdpState::WaitShowAck);
      break;
    case DdpState::SendDrawTest:
      if (ddpEarliestDrawMs != 0 && now < ddpEarliestDrawMs) {
        break;
      }
      ddpEarliestDrawMs = 0;
      ddpRedrawPending = false;
      ddpForceRedrawPending = false;
      sendDdpDraw();
      ddpSetState(DdpState::WaitDrawAck);
      break;
    case DdpState::Ready:
      if (ddpNextPeriodicRebindMs != 0 && now >= ddpNextPeriodicRebindMs) {
        ddpNextPeriodicRebindMs = 0;
        if (audioPeriodicRefreshAllowed() || navPeriodicRefreshAllowed()) {
          if (ddpVerbose) {
            Serial.println("DDP periodic refresh");
          }
          ddpSetState(DdpState::SendShowArea);
          break;
        }
      }
      if (ddpRedrawPending) {
        if (!ddpForceRedrawPending && ddpTarget == DdpTarget::Audio && !audioPageActivelyShowing()) {
          ddpRedrawPending = false;
          break;
        }
        if (!ddpForceRedrawPending && ddpTarget == DdpTarget::Navigation && !navPageActivelyShowing()) {
          ddpRedrawPending = false;
          break;
        }
        // A naked redraw from Ready can be ignored after ignition wake. Re-show
        // the area first so the cluster re-presents the page before drawing.
        ddpSetState(DdpState::SendShowArea);
        break;
      }
      if (now - ddpLastKeepaliveMs > DDP_KEEPALIVE_INTERVAL_MS) {
        sendDdpKeepalive();
      }
      break;
    case DdpState::Error:
      break;
    default:
      break;
  }
}

bool isBapCandidateFrame(const CAN_message_t &msg) {
  if (msg.flags.extended || msg.len < 2) {
    return false;
  }

  if (msg.id == BAP_AUDIO_CAN_ID || msg.id == BAP_NAVI_CAN_ID) {
    return true;
  }

  if (navSniffMode && (msg.id == BAP_ASG_07_CAN_ID || msg.id == BAP_NAVI_CAN_ID)) {
    return true;
  }

  return (bapLogAllCandidates || audioSniffMode) && msg.id >= BAP_LOG_MIN_CAN_ID && msg.id <= BAP_LOG_MAX_CAN_ID;
}

bool isVehicleWakeActivityFrame(const CAN_message_t &msg) {
  if (msg.flags.extended) {
    return false;
  }

  if (msg.id == DDP_AUDIO_FROM_DISPLAY_ID || msg.id == DDP_NAV_FROM_DISPLAY_ID) {
    return false;
  }

  return true;
}

void noteVehicleWakeActivity() {
  const uint32_t now = millis();
  lastNonDdpCanActivityMs = now;

  if (lastVehicleWakeFrameMs == 0 || now - lastVehicleWakeFrameMs > DDP_WAKE_BURST_GAP_MS) {
    vehicleWakeBurstStartedMs = now;
    vehicleWakeBurstFrames = 1;
  } else if (vehicleWakeBurstFrames < 0xFF) {
    vehicleWakeBurstFrames++;
  }

  lastVehicleWakeFrameMs = now;
}

bool hasQualifiedVehicleWakeBurst(uint32_t now) {
  if (vehicleWakeBurstFrames < DDP_WAKE_MIN_BURST_FRAMES) {
    return false;
  }

  if (lastVehicleWakeFrameMs == 0 || now - lastVehicleWakeFrameMs > DDP_WAKE_BURST_GAP_MS) {
    return false;
  }

  return vehicleWakeBurstStartedMs != 0 && now >= vehicleWakeBurstStartedMs;
}

bool isPowerCompatibilityFrame(const CAN_message_t &msg) {
  if (msg.flags.extended) {
    return false;
  }

  return msg.id == POWER_COMPAT_CAN_ID_436 || msg.id == POWER_COMPAT_CAN_ID_439;
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
  bool used[WAKE_ID_COUNTER_SLOTS] = {};
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

void printWakeIdActivity() {
  const uint32_t now = millis();
  Serial.print("Wake-related CAN IDs seen within ");
  Serial.print(WAKE_ID_RECENT_WINDOW_MS);
  Serial.println(" ms:");
  printObservedCanIds("vehicle->radio observed", vehicleObservedIds, WAKE_ID_COUNTER_SLOTS, now,
                      WAKE_ID_RECENT_WINDOW_MS);
  printObservedCanIds("radio->vehicle observed", radioObservedIds, WAKE_ID_COUNTER_SLOTS, now,
                      WAKE_ID_RECENT_WINDOW_MS);
}

const char *dbcFrameName(uint32_t id) {
  switch (id) {
    case BAP_AUDIO_CAN_ID: return "BAP_AUDIO";
    case BAP_ASG_07_CAN_ID: return "BAP_ASG_07";
    case BAP_NAVI_CAN_ID: return "BAP_NAVI";
    case PSD_NAV_CAN_ID: return "PSD_02";
    case CLOCK_KOMBI_K2_CAN_ID: return "mKombi_K2";
    case CLOCK_DIAG_1_CAN_ID: return "mDiagnose_1";
    case CLOCK_CANDIDATE_1_CAN_ID: return "clock_621";
    case CLOCK_CANDIDATE_2_CAN_ID: return "clock_651";
    case CLOCK_CANDIDATE_3_CAN_ID: return "clock_653";
    case CLOCK_CANDIDATE_4_CAN_ID: return "clock_655";
    case CLOCK_CANDIDATE_5_CAN_ID: return "clock_658";
    case CLOCK_CANDIDATE_6_CAN_ID: return "clock_65F";
    case CLOCK_CANDIDATE_7_CAN_ID: return "clock_661";
    case CLOCK_CANDIDATE_8_CAN_ID: return "clock_665";
  }
  return "unknown";
}

bool isClockCandidateFrame(const CAN_message_t &msg) {
  if (msg.flags.extended) {
    return false;
  }

  switch (msg.id) {
    case CLOCK_KOMBI_K2_CAN_ID:
    case CLOCK_DIAG_1_CAN_ID:
    case CLOCK_CANDIDATE_1_CAN_ID:
    case CLOCK_CANDIDATE_2_CAN_ID:
    case CLOCK_CANDIDATE_3_CAN_ID:
    case CLOCK_CANDIDATE_4_CAN_ID:
    case CLOCK_CANDIDATE_5_CAN_ID:
    case CLOCK_CANDIDATE_6_CAN_ID:
    case CLOCK_CANDIDATE_7_CAN_ID:
    case CLOCK_CANDIDATE_8_CAN_ID:
      return true;
    default:
      return false;
  }
}

uint32_t extractLittleEndianBits(const uint8_t *data, uint8_t startBit, uint8_t bitLength) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < bitLength; ++i) {
    const uint8_t bitIndex = startBit + i;
    if ((data[bitIndex / 8] & (1U << (bitIndex % 8))) != 0) {
      value |= (1UL << i);
    }
  }
  return value;
}

uint8_t decodeBcdByte(uint8_t value) {
  return static_cast<uint8_t>(((value >> 4) & 0x0F) * 10 + (value & 0x0F));
}

void printClockSniffFrame(const CAN_message_t &msg, const char *direction) {
  if (!clockSniffMode || !isClockCandidateFrame(msg)) {
    return;
  }

  clockSniffFrames++;
  Serial.print("clock sniff ms=");
  Serial.print(millis());
  Serial.print(" dir=");
  Serial.print(direction);
  Serial.print(" id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(" name=");
  Serial.print(dbcFrameName(msg.id));
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

  if (msg.id == CLOCK_KOMBI_K2_CAN_ID && msg.len >= 8) {
    Serial.print(" decoded=");
    Serial.print("reset=");
    Serial.print((msg.buf[0] & 0x02) != 0 ? "1" : "0");
    Serial.print(" 24h=");
    Serial.print((msg.buf[0] & 0x04) != 0 ? "1" : "0");
    Serial.print(" time=");
    Serial.print(decodeBcdByte(msg.buf[1]));
    Serial.print(':');
    Serial.print(decodeBcdByte(msg.buf[2]));
    Serial.print(':');
    Serial.print(decodeBcdByte(msg.buf[3]));
    Serial.print(" date=");
    Serial.print(decodeBcdByte(msg.buf[6]));
    Serial.print(decodeBcdByte(msg.buf[7]));
    Serial.print('-');
    Serial.print(decodeBcdByte(msg.buf[5]));
    Serial.print('-');
    Serial.print(decodeBcdByte(msg.buf[4]));
  } else if (msg.id == CLOCK_DIAG_1_CAN_ID && msg.len >= 8) {
    const uint32_t year = 2000UL + extractLittleEndianBits(msg.buf, 28, 7);
    const uint32_t month = extractLittleEndianBits(msg.buf, 35, 4);
    const uint32_t day = extractLittleEndianBits(msg.buf, 39, 5);
    const uint32_t hour = extractLittleEndianBits(msg.buf, 44, 5);
    const uint32_t minute = extractLittleEndianBits(msg.buf, 49, 6);
    const uint32_t second = extractLittleEndianBits(msg.buf, 55, 6);
    const uint32_t altKm = extractLittleEndianBits(msg.buf, 62, 1);
    const uint32_t altTime = extractLittleEndianBits(msg.buf, 63, 1);

    Serial.print(" decoded=");
    Serial.print("time=");
    Serial.print(hour);
    Serial.print(':');
    Serial.print(minute);
    Serial.print(':');
    Serial.print(second);
    Serial.print(" date=");
    Serial.print(year);
    Serial.print('-');
    Serial.print(month);
    Serial.print('-');
    Serial.print(day);
    Serial.print(" altTime=");
    Serial.print(altTime);
    Serial.print(" altKm=");
    Serial.print(altKm);
  }

  Serial.println();
}

void formatHexByte(uint8_t value, char *out) {
  static const char hex[] = "0123456789ABCDEF";
  out[0] = hex[(value >> 4) & 0x0F];
  out[1] = hex[value & 0x0F];
  out[2] = '\0';
}

void printBapFrame(const BapDecodedFrame &frame) {
  Serial.print("bap rx id=0x");
  Serial.print(frame.canId, HEX);
  Serial.print(" ");
  Serial.print(frame.multiframe ? "mf" : "sf");
  Serial.print(" op=");
  Serial.print(frame.opcode);
  Serial.print(" node=");
  Serial.print(frame.node);
  Serial.print(" port=");
  Serial.print(frame.port);
  Serial.print(" len=");
  Serial.print(frame.len);
  Serial.print(" data=");
  for (uint16_t i = 0; i < frame.len && i < BAP_PAYLOAD_MAX; ++i) {
    if (frame.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame.data[i], HEX);
    if (i + 1 < frame.len && i + 1 < BAP_PAYLOAD_MAX) {
      Serial.print(' ');
    }
  }
  Serial.print(" text='");
  for (uint16_t i = 0; i < frame.len && i < BAP_PAYLOAD_MAX; ++i) {
    const uint8_t c = frame.data[i];
    Serial.print(c >= 0x20 && c <= 0x7E ? static_cast<char>(c) : '.');
  }
  Serial.println("'");
}

void printAudioSniffFrame(const BapDecodedFrame &frame) {
  char text[AUDIO_LINE_MAX] = {};
  const bool hasText = extractPrintableBapText(frame, text, sizeof(text));
  if (hasText) {
    trimAudioText(text);
  }

  if (frame.opcode != 4 && !hasText) {
    return;
  }

  Serial.print("audio sniff ms=");
  Serial.print(millis());
  Serial.print(" id=0x");
  Serial.print(frame.canId, HEX);
  Serial.print(" ");
  Serial.print(frame.multiframe ? "mf" : "sf");
  Serial.print(" op=");
  Serial.print(frame.opcode);
  Serial.print(" node=");
  Serial.print(frame.node);
  Serial.print(" port=");
  Serial.print(frame.port);
  Serial.print(" len=");
  Serial.print(frame.len);
  Serial.print(" kind=");
  if (hasText && isFrequencyLikeText(text)) {
    Serial.print("freq");
  } else if (hasText && hasStationNameChars(text)) {
    Serial.print("text");
  } else {
    Serial.print("raw");
  }
  Serial.print(" text='");
  if (hasText) {
    Serial.print(text);
  }
  Serial.print("' data=");
  for (uint16_t i = 0; i < frame.len && i < BAP_PAYLOAD_MAX; ++i) {
    if (frame.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame.data[i], HEX);
    if (i + 1 < frame.len && i + 1 < BAP_PAYLOAD_MAX) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

void printRawNavSniffFrame(const CAN_message_t &msg, const char *direction) {
  if (!navSniffMode || msg.flags.extended) {
    return;
  }

  if (msg.id != BAP_ASG_07_CAN_ID && msg.id != BAP_NAVI_CAN_ID && msg.id != PSD_NAV_CAN_ID) {
    return;
  }

  navSniffFrames++;
  Serial.print("nav sniff ms=");
  Serial.print(millis());
  Serial.print(" dir=");
  Serial.print(direction);
  Serial.print(" id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(" name=");
  Serial.print(dbcFrameName(msg.id));
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
  if (msg.id == PSD_NAV_CAN_ID && msg.len > 0) {
    Serial.print(" psdMux=");
    Serial.print(msg.buf[0] & 0x03);
  }
  Serial.println();
}

void printNavSniffBapFrame(const BapDecodedFrame &frame, const char *direction) {
  if (!navSniffMode || (frame.canId != BAP_ASG_07_CAN_ID && frame.canId != BAP_NAVI_CAN_ID)) {
    return;
  }

  char text[AUDIO_LINE_MAX] = {};
  const bool hasText = extractPrintableBapText(frame, text, sizeof(text));
  if (hasText) {
    trimAudioText(text);
  }

  Serial.print("nav bap ms=");
  Serial.print(millis());
  Serial.print(" dir=");
  Serial.print(direction);
  Serial.print(" id=0x");
  Serial.print(frame.canId, HEX);
  Serial.print(" name=");
  Serial.print(dbcFrameName(frame.canId));
  Serial.print(" ");
  Serial.print(frame.multiframe ? "mf" : "sf");
  Serial.print(" op=");
  Serial.print(frame.opcode);
  Serial.print(" node=");
  Serial.print(frame.node);
  Serial.print(" port=");
  Serial.print(frame.port);
  Serial.print(" len=");
  Serial.print(frame.len);
  if (frame.canId == BAP_NAVI_CAN_ID && frame.port == 21 && frame.len >= 7) {
    const NavManeuverDescriptorCandidate descriptor = decodeNavManeuverDescriptorCandidate(frame);
    char maneuverCode[3] = {};
    formatHexByte(frame.data[6], maneuverCode);
    Serial.print(" maneuver=M");
    Serial.print(maneuverCode);
    Serial.print(" kind=");
    Serial.print(frame.data[0], HEX);
    if (descriptor.valid) {
      Serial.print(" descOff=");
      Serial.print(descriptor.offset);
      Serial.print(" main=0x");
      Serial.print(descriptor.mainElement, HEX);
      Serial.print(" dir=0x");
      Serial.print(descriptor.direction, HEX);
      Serial.print(" z=0x");
      Serial.print(descriptor.zLevel, HEX);
    }
  }
  Serial.print(" text='");
  if (hasText) {
    Serial.print(text);
  }
  Serial.print("' data=");
  for (uint16_t i = 0; i < frame.len && i < BAP_PAYLOAD_MAX; ++i) {
    if (frame.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame.data[i], HEX);
    if (i + 1 < frame.len && i + 1 < BAP_PAYLOAD_MAX) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

bool finishBapFrame(const BapRxChannel &channel, BapDecodedFrame &decoded) {
  if (channel.len > BAP_PAYLOAD_MAX) {
    return false;
  }

  decoded.canId = channel.canId;
  decoded.multiframe = true;
  decoded.opcode = channel.opcode;
  decoded.node = channel.node;
  decoded.port = channel.port;
  decoded.len = channel.len;
  for (uint16_t i = 0; i < channel.len; ++i) {
    decoded.data[i] = channel.data[i];
  }
  return true;
}

bool decodeBapFrame(const CAN_message_t &msg, BapDecodedFrame &decoded) {
  const uint8_t first = msg.buf[0];

  if ((first & 0x80) == 0) {
    const uint16_t header = (static_cast<uint16_t>(msg.buf[0]) << 8) | msg.buf[1];
    decoded.canId = msg.id;
    decoded.multiframe = false;
    decoded.opcode = (header >> 12) & 0x07;
    decoded.node = (header >> 6) & 0x3F;
    decoded.port = header & 0x3F;
    decoded.len = msg.len - 2;
    for (uint16_t i = 0; i < decoded.len && i < BAP_PAYLOAD_MAX; ++i) {
      decoded.data[i] = msg.buf[i + 2];
    }
    return decoded.len <= BAP_PAYLOAD_MAX;
  }

  const uint8_t channelIndex = (first >> 4) & 0x03;
  BapRxChannel &channel = bapChannels[channelIndex];

  if ((first & 0x40) == 0) {
    if (msg.len < 4) {
      channel.active = false;
      return false;
    }

    const uint16_t totalLen = (static_cast<uint16_t>(first & 0x0F) << 8) | msg.buf[1];
    const uint16_t header = (static_cast<uint16_t>(msg.buf[2]) << 8) | msg.buf[3];
    const uint8_t thisLen = msg.len - 4;

    channel.active = totalLen <= BAP_PAYLOAD_MAX;
    channel.canId = msg.id;
    channel.len = totalLen;
    channel.done = 0;
    channel.opcode = (header >> 12) & 0x07;
    channel.node = (header >> 6) & 0x3F;
    channel.port = header & 0x3F;

    if (!channel.active || thisLen > channel.len) {
      channel.active = false;
      return false;
    }

    for (uint8_t i = 0; i < thisLen; ++i) {
      channel.data[channel.done++] = msg.buf[i + 4];
    }

    if (channel.done == channel.len) {
      const bool ok = finishBapFrame(channel, decoded);
      channel.active = false;
      return ok;
    }
    return false;
  }

  if (!channel.active || channel.canId != msg.id) {
    return false;
  }

  const uint8_t thisLen = msg.len - 1;
  if (channel.done + thisLen > channel.len) {
    channel.active = false;
    return false;
  }

  for (uint8_t i = 0; i < thisLen; ++i) {
    channel.data[channel.done++] = msg.buf[i + 1];
  }

  if (channel.done == channel.len) {
    const bool ok = finishBapFrame(channel, decoded);
    channel.active = false;
    return ok;
  }

  return false;
}

bool extractPrintableBapText(const BapDecodedFrame &frame, char *out, uint8_t outSize) {
  uint16_t bestStart = 0;
  uint8_t bestLen = 0;
  uint16_t runStart = 0;
  uint8_t runLen = 0;

  for (uint16_t i = 0; i < frame.len && i < BAP_PAYLOAD_MAX; ++i) {
    const uint8_t c = frame.data[i];
    const bool printable = c >= 0x20 && c <= 0x7E;

    if (printable) {
      if (runLen == 0) {
        runStart = i;
      }
      runLen++;
      if (runLen > bestLen) {
        bestStart = runStart;
        bestLen = runLen;
      }
    } else {
      runLen = 0;
    }
  }

  if (bestLen < 2) {
    return false;
  }

  const uint8_t copyLen = bestLen < outSize - 1 ? bestLen : outSize - 1;
  for (uint8_t i = 0; i < copyLen; ++i) {
    out[i] = static_cast<char>(frame.data[bestStart + i]);
  }
  out[copyLen] = '\0';
  return true;
}

void trimAudioText(char *text) {
  uint8_t start = 0;
  while (text[start] == ' ') {
    start++;
  }

  if (start > 0) {
    uint8_t i = 0;
    do {
      text[i] = text[start + i];
    } while (text[i++] != '\0');
  }

  uint8_t len = 0;
  while (text[len] != '\0') {
    len++;
  }

  while (len > 0 && text[len - 1] == ' ') {
    text[--len] = '\0';
  }
}

bool isFrequencyLikeText(const char *text) {
  bool hasDigit = false;
  bool hasDot = false;
  bool onlyFrequencyChars = true;

  for (uint8_t i = 0; text[i] != '\0'; ++i) {
    const char c = text[i];
    if (c >= '0' && c <= '9') {
      hasDigit = true;
      continue;
    }
    if (c == '.') {
      hasDot = true;
      continue;
    }
    if (c != ' ' && c != 'M' && c != 'H' && c != 'z') {
      onlyFrequencyChars = false;
    }
  }

  return hasDigit && (hasDot || onlyFrequencyChars);
}

bool hasStationNameChars(const char *text) {
  for (uint8_t i = 0; text[i] != '\0'; ++i) {
    const char c = text[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
      return true;
    }
  }
  return false;
}

char asciiLower(char c) {
  if (c >= 'A' && c <= 'Z') {
    return c + ('a' - 'A');
  }
  return c;
}

bool textEqualsIgnoreCase(const char *text, const char *expected) {
  uint8_t i = 0;
  while (text[i] != '\0' && expected[i] != '\0') {
    if (asciiLower(text[i]) != asciiLower(expected[i])) {
      return false;
    }
    i++;
  }
  return text[i] == '\0' && expected[i] == '\0';
}

bool textContainsIgnoreCase(const char *text, const char *needle) {
  if (needle[0] == '\0') {
    return true;
  }

  for (uint8_t i = 0; text[i] != '\0'; ++i) {
    uint8_t j = 0;
    while (needle[j] != '\0' && text[i + j] != '\0' && asciiLower(text[i + j]) == asciiLower(needle[j])) {
      j++;
    }
    if (needle[j] == '\0') {
      return true;
    }
  }
  return false;
}

bool isSourceOnlyAudioText(const char *text) {
  return textContainsIgnoreCase(text, "carplay") ||
         textContainsIgnoreCase(text, "android auto") ||
         textContainsIgnoreCase(text, "bluetooth") ||
         textContainsIgnoreCase(text, "bt audio") ||
         textEqualsIgnoreCase(text, "aux") ||
         textEqualsIgnoreCase(text, "usb") ||
         textEqualsIgnoreCase(text, "sd") ||
         textEqualsIgnoreCase(text, "media") ||
         textEqualsIgnoreCase(text, "ipod");
}

bool copyBapAsciiField(const uint8_t *data, uint16_t len, uint16_t start, uint8_t fieldLen, char *out, uint8_t outSize) {
  if (outSize == 0 || start >= len) {
    return false;
  }

  const uint16_t available = len - start;
  const uint8_t copyLen = fieldLen < available ? fieldLen : available;
  uint8_t written = 0;

  for (uint8_t i = 0; i < copyLen && written + 1 < outSize; ++i) {
    const uint8_t c = data[start + i];
    if (c >= 0x20 && c <= 0x7E) {
      out[written++] = static_cast<char>(c);
    }
  }

  out[written] = '\0';
  trimAudioText(out);
  return out[0] != '\0';
}

void setAudioLines(const char *top, const char *bottom) {
  copyAudioLine(audioLine1, top);
  copyAudioLine(audioLine2, bottom);
}

bool setAudioSourceOnlyLine(const char *source) {
  if (!isSourceOnlyAudioText(source)) {
    return false;
  }

  const bool changed = strcmp(source, audioLine1) != 0 || audioLine2[0] != '\0' || lastFrequencyText[0] != '\0';
  setAudioLines(source, "");
  copyAudioLine(lastFrequencyText, "");
  return changed;
}

void requestDdpRedrawIfTarget(DdpTarget target) {
  if (ddpTarget != target) {
    return;
  }

  if (target == DdpTarget::Audio && !audioPageActivelyShowing()) {
    return;
  }

  if (target == DdpTarget::Navigation && !navPageActivelyShowing()) {
    return;
  }

  if (target == DdpTarget::Navigation || audioPageActivelyShowing()) {
    requestDdpRedraw();
  }
}

void setNavLines(const char *top, const char *bottom) {
  const bool changed = strcmp(navLine1, top) != 0 || strcmp(navLine2, bottom) != 0;
  copyAudioLine(navLine1, top);
  copyAudioLine(navLine2, bottom);
  if (changed) {
    navTextUpdates++;
    requestDdpRedrawIfTarget(DdpTarget::Navigation);
  }
}

void buildNavTopLine(const char *distance, const char *maneuver, char *out, uint8_t outSize) {
  if (outSize == 0) {
    return;
  }

  if (distance == nullptr || distance[0] == '\0') {
    copyAudioLine(out, maneuver != nullptr && maneuver[0] != '\0' ? maneuver : "NAV");
    return;
  }

  if (maneuver == nullptr || maneuver[0] == '\0') {
    copyAudioLine(out, distance);
    return;
  }

  snprintf(out, outSize, "%s %s", distance, maneuver);
}

const char *navMainElementName(uint8_t value) {
  switch (value) {
    case 0x0B: return "FOLLOW";
    case 0x0C: return "LANE";
    case 0x0D: return "TURN";
    case 0x0E: return "MAIN";
    case 0x0F: return "EXIT R";
    case 0x10: return "EXIT L";
    case 0x11: return "SR R";
    case 0x12: return "SR L";
    case 0x13: return "FORK2";
    case 0x14: return "FORK3";
    case 0x19: return "UTURN";
    case 0x1C: return "PREP";
    default: return nullptr;
  }
}

const char *navDirectionLabel(uint8_t value) {
  switch (value) {
    case 0x00: return "STRAIGHT";
    case 0x20: return "SLIGHT L";
    case 0x40: return "LEFT";
    case 0x60: return "SHARP L";
    case 0x80: return "UTURN";
    case 0xA0: return "SHARP R";
    case 0xC0: return "RIGHT";
    case 0xE0: return "SLIGHT R";
    default: return nullptr;
  }
}

bool isCardinalDirectionByte(uint8_t value) {
  return (value & 0x1F) == 0;
}

NavManeuverDescriptorCandidate decodeNavManeuverDescriptorCandidate(const BapDecodedFrame &frame) {
  NavManeuverDescriptorCandidate best = {false, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, -1};

  if (frame.port != 21 || frame.len < 4) {
    return best;
  }

  for (uint8_t offset = 0; offset + 3 < frame.len; ++offset) {
    const uint8_t mainElement = frame.data[offset];
    const uint8_t direction = frame.data[offset + 1];
    const uint8_t zLevel = frame.data[offset + 2];
    const uint8_t sidestreetLen = frame.data[offset + 3];

    int8_t score = 0;
    if (mainElement <= 0x33) {
      score += 1;
    }
    if (mainElement >= 0x0B && mainElement <= 0x33) {
      score += 4;
    } else if (mainElement <= 0x0A) {
      score -= 2;
    }
    if (isCardinalDirectionByte(direction)) {
      score += 2;
    }
    if (zLevel == 0x00 || zLevel == 0x01 || zLevel == 0x02 || zLevel == 0xFF) {
      score += 1;
    }
    if (sidestreetLen <= 0x11 && static_cast<uint16_t>(offset) + 4 + sidestreetLen <= frame.len) {
      score += 1;
    } else if (sidestreetLen > 0x11) {
      continue;
    }

    if (!best.valid || score > best.score) {
      best.valid = true;
      best.offset = offset;
      best.mainElement = mainElement;
      best.direction = direction;
      best.zLevel = zLevel;
      best.sidestreetLen = sidestreetLen;
      best.score = score;
    }
  }

  if (!best.valid || best.score < 7) {
    NavManeuverDescriptorCandidate invalid = {false, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, -1};
    return invalid;
  }

  return best;
}

void refreshNavLines() {
  char topLine[AUDIO_LINE_MAX] = "NAV";
  const char *top = topLine;
  const char *bottom = "Waiting route";

  if (navRouteActive) {
    buildNavTopLine(navDistanceText, navManeuverText, topLine, sizeof(topLine));
    if (navNextStreetText[0] != '\0') {
      bottom = navNextStreetText;
    } else if (navCurrentStreetText[0] != '\0') {
      bottom = navCurrentStreetText;
    } else {
      bottom = "Guidance active";
    }
  }

  setNavLines(top, bottom);
}

void formatNavManeuver(uint8_t maneuverKind, uint8_t maneuverCode, char *out, uint8_t outSize) {
  if (outSize == 0) {
    return;
  }

  if (maneuverCode == 0xFF || !navShowManeuverCodes) {
    out[0] = '\0';
    return;
  }

  char code[3] = {};
  char kind[3] = {};
  formatHexByte(maneuverCode, code);
  formatHexByte(maneuverKind, kind);
  if (maneuverKind != 0xFF) {
    snprintf(out, outSize, "K%sM%s", kind + 1, code);
  } else {
    snprintf(out, outSize, "M%s", code);
  }
}

void rebuildNavManeuverText() {
  formatNavManeuver(navManeuverKind, navManeuverCode, navManeuverText, sizeof(navManeuverText));
  refreshNavLines();
}

void formatNavDistance(uint16_t rawFeet, uint8_t unitHint, char *out, uint8_t outSize) {
  if (outSize == 0) {
    return;
  }

  if (rawFeet == 0) {
    copyAudioLine(out, "NOW");
    return;
  }

  // Live BAP_NAVI port 16 behaves like a big-endian distance in feet.
  // The third byte is kept as a future unit/status hint; for now we display a human-friendly distance.
  (void)unitHint;
  if (rawFeet >= 1000) {
    const uint16_t tenths = (static_cast<uint32_t>(rawFeet) * 10UL + 2640UL) / 5280UL;
    snprintf(out, outSize, "%u.%u mi", tenths / 10, tenths % 10);
  } else {
    snprintf(out, outSize, "%u ft", rawFeet);
  }
}

bool updateNavRouteActive(const BapDecodedFrame &frame) {
  if (frame.port != 17 || frame.len < 1) {
    return false;
  }

  const bool newRouteActive = frame.data[0] != 0x00;
  if (newRouteActive != navRouteActive) {
    navRouteActive = newRouteActive;
    if (!navRouteActive) {
      copyAudioLine(navDistanceText, "");
      copyAudioLine(navManeuverText, "");
      copyAudioLine(navCurrentStreetText, "");
      copyAudioLine(navNextStreetText, "");
      navManeuverCode = 0xFF;
      navManeuverKind = 0xFF;
      navManeuverMainElement = 0xFF;
      navManeuverDirection = 0xFF;
      navManeuverZLevel = 0xFF;
      navManeuverRecordOffset = 0xFF;
    }
    refreshNavLines();
    applyAutomaticDdpPriority();
    if (bapLogging || navSniffMode) {
      Serial.print("nav route active=");
      Serial.println(navRouteActive ? "yes" : "no");
    }
  }
  return true;
}

bool updateNavDistance(const BapDecodedFrame &frame) {
  if (frame.port != 16 || frame.len < 3) {
    return false;
  }

  if (!navRouteActive) {
    if (navDistanceText[0] != '\0') {
      copyAudioLine(navDistanceText, "");
      refreshNavLines();
    }
    return true;
  }

  const uint16_t rawDistance = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  char distance[AUDIO_LINE_MAX] = {};
  formatNavDistance(rawDistance, frame.data[2], distance, sizeof(distance));
  if (strcmp(distance, navDistanceText) != 0) {
    copyAudioLine(navDistanceText, distance);
    refreshNavLines();
    if (bapLogging || navSniffMode) {
      Serial.print("nav distance update: ");
      Serial.println(navDistanceText);
    }
  }
  return true;
}

bool updateNavStreetText(const BapDecodedFrame &frame) {
  if (frame.port != 19 && frame.port != 20) {
    return false;
  }

  if (frame.len < 2) {
    return true;
  }

  if (frame.data[0] == 0) {
      char *target = frame.port == 20 ? navNextStreetText : navCurrentStreetText;
      if (target[0] != '\0') {
        copyAudioLine(target, "");
        refreshNavLines();
      }
      return true;
  }

  if (!navRouteActive) {
    return true;
  }

  char street[AUDIO_LINE_MAX] = {};
  if (!copyBapAsciiField(frame.data, frame.len, 1, frame.data[0], street, sizeof(street))) {
    return true;
  }

  char *target = frame.port == 20 ? navNextStreetText : navCurrentStreetText;
  if (strcmp(street, target) != 0) {
    copyAudioLine(target, street);
    refreshNavLines();
    if (bapLogging || navSniffMode) {
      Serial.print(frame.port == 20 ? "nav next street update: " : "nav current street update: ");
      Serial.println(street);
    }
  }
  return true;
}

bool updateNavManeuver(const BapDecodedFrame &frame) {
  if (frame.port != 21 || frame.len < 7) {
    return false;
  }

  const NavManeuverDescriptorCandidate descriptor = decodeNavManeuverDescriptorCandidate(frame);
  const uint8_t newKind = frame.data[0];
  const uint8_t newCode = frame.data[6];
  const bool invalidCode = newCode == 0xFF || newKind == 0xFF;

  if (invalidCode) {
    if (navManeuverText[0] != '\0' || navManeuverCode != 0xFF || navManeuverKind != 0xFF ||
        navManeuverMainElement != 0xFF || navManeuverDirection != 0xFF || navManeuverRecordOffset != 0xFF) {
      navManeuverCode = 0xFF;
      navManeuverKind = 0xFF;
      navManeuverMainElement = 0xFF;
      navManeuverDirection = 0xFF;
      navManeuverZLevel = 0xFF;
      navManeuverRecordOffset = 0xFF;
      copyAudioLine(navManeuverText, "");
      refreshNavLines();
    }
    return true;
  }

  const uint8_t newMainElement = descriptor.valid ? descriptor.mainElement : 0xFF;
  const uint8_t newDirection = descriptor.valid ? descriptor.direction : 0xFF;
  const uint8_t newZLevel = descriptor.valid ? descriptor.zLevel : 0xFF;
  const uint8_t newOffset = descriptor.valid ? descriptor.offset : 0xFF;

  if (newCode != navManeuverCode || newKind != navManeuverKind ||
      newMainElement != navManeuverMainElement || newDirection != navManeuverDirection ||
      newZLevel != navManeuverZLevel || newOffset != navManeuverRecordOffset) {
    navManeuverCode = newCode;
    navManeuverKind = newKind;
    navManeuverMainElement = newMainElement;
    navManeuverDirection = newDirection;
    navManeuverZLevel = newZLevel;
    navManeuverRecordOffset = newOffset;
    formatNavManeuver(navManeuverKind, navManeuverCode, navManeuverText, sizeof(navManeuverText));
    navManeuverUpdates++;
    refreshNavLines();
    if (bapLogging || navSniffMode) {
      Serial.print("nav maneuver update: ");
      Serial.print(navManeuverText);
      Serial.print(" kind=0x");
      Serial.print(navManeuverKind, HEX);
      Serial.print(" code=0x");
      Serial.print(navManeuverCode, HEX);
      Serial.print(" main=0x");
      Serial.print(navManeuverMainElement, HEX);
      Serial.print(" dir=0x");
      Serial.print(navManeuverDirection, HEX);
      Serial.print(" off=");
      Serial.println(navManeuverRecordOffset);
    }
  }
  return true;
}

void clearNavText() {
  navRouteActive = false;
  copyAudioLine(navDistanceText, "");
  copyAudioLine(navManeuverText, "");
  copyAudioLine(navCurrentStreetText, "");
  copyAudioLine(navNextStreetText, "");
  navManeuverCode = 0xFF;
  navManeuverKind = 0xFF;
  navManeuverMainElement = 0xFF;
  navManeuverDirection = 0xFF;
  navManeuverZLevel = 0xFF;
  navManeuverRecordOffset = 0xFF;
  setNavLines("NAV", "Waiting route");
  applyAutomaticDdpPriority();
}

void updateNavTextFromBap(const BapDecodedFrame &frame) {
  if (!navTranslateEnabled || frame.canId != BAP_NAVI_CAN_ID || frame.node != 50) {
    return;
  }

  if (frame.opcode != 3 && frame.opcode != 4) {
    return;
  }

  if (updateNavRouteActive(frame)) {
    return;
  }
  if (updateNavDistance(frame)) {
    return;
  }
  if (updateNavManeuver(frame)) {
    return;
  }
  updateNavStreetText(frame);
}

bool updateAudioFromPort21(const BapDecodedFrame &frame) {
  if (frame.canId != BAP_AUDIO_CAN_ID || frame.opcode != 4 || frame.node != 49 || frame.port != 21 || frame.len < 2) {
    return false;
  }

  bool changed = false;
  char firstField[AUDIO_LINE_MAX] = {};
  char secondField[AUDIO_LINE_MAX] = {};
  char thirdField[AUDIO_LINE_MAX] = {};
  bool firstFieldIsFrequency = false;
  bool hasFirstField = false;
  bool hasSecondField = false;
  bool hasThirdField = false;

  // Port 21 carries a frequency field first. Observed layout:
  // [firstBlockLen] [freq/source ASCII] [optional status bytes] [titleLen] [title] [artistLen] [artist] ...
  // FM frequency frames include non-ASCII status bytes in the first block, while source labels like
  // "Apple CarPlay" use the whole first block as text.
  if (frame.data[0] > 0) {
    hasFirstField = copyBapAsciiField(frame.data, frame.len, 1, frame.data[0], firstField, sizeof(firstField));
    firstFieldIsFrequency = hasFirstField && isFrequencyLikeText(firstField);
  }

  const uint16_t titleLenIndex = frame.data[0] + 1;
  const bool hasStatusTail = frame.len >= 3 && frame.data[frame.len - 3] == 0x01 && frame.data[frame.len - 2] == 0x00 && frame.data[frame.len - 1] == 0x00;

  if (titleLenIndex < frame.len) {
    const uint8_t titleLen = frame.data[titleLenIndex];
    hasSecondField = titleLen > 0 && copyBapAsciiField(frame.data, frame.len, titleLenIndex + 1, titleLen, secondField, sizeof(secondField));

    const uint16_t artistLenIndex = titleLenIndex + 1 + titleLen;
    if (artistLenIndex < frame.len) {
      const uint8_t artistLen = frame.data[artistLenIndex];
      hasThirdField = artistLen > 0 && copyBapAsciiField(frame.data, frame.len, artistLenIndex + 1, artistLen, thirdField, sizeof(thirdField));
    }
  }

  if (firstFieldIsFrequency) {
    if (strcmp(firstField, audioLine2) != 0) {
      copyAudioLine(audioLine2, firstField);
      changed = true;
    }

    if (strcmp(firstField, lastFrequencyText) != 0) {
      copyAudioLine(audioLine1, "AUDIO");
      copyAudioLine(lastFrequencyText, firstField);
      changed = true;
    }

    if (hasSecondField && strcmp(secondField, audioLine1) != 0) {
      copyAudioLine(audioLine1, secondField);
      changed = true;
      if (bapLogging || audioSniffMode) {
        Serial.print("audio title update: ");
        Serial.println(audioLine1);
      }
    }
  } else if (hasFirstField && hasStatusTail && !hasThirdField) {
    if (strcmp(firstField, audioLine1) != 0 || audioLine2[0] != '\0' || lastFrequencyText[0] != '\0') {
      setAudioLines(firstField, "");
      copyAudioLine(lastFrequencyText, "");
      changed = true;
      if (bapLogging || audioSniffMode) {
        Serial.print("audio source-only update: ");
        Serial.println(audioLine1);
      }
    }
  } else if (hasSecondField) {
    if (strcmp(secondField, audioLine1) != 0) {
      copyAudioLine(audioLine1, secondField);
      changed = true;
      if (bapLogging || audioSniffMode) {
        Serial.print("audio title update: ");
        Serial.println(audioLine1);
      }
    }
    if (hasFirstField && strcmp(firstField, audioLine2) != 0) {
      copyAudioLine(audioLine2, firstField);
      copyAudioLine(lastFrequencyText, "");
      changed = true;
      if (bapLogging || audioSniffMode) {
        Serial.print("audio artist/source update: ");
        Serial.println(audioLine2);
      }
    }
  } else if (hasFirstField && isSourceOnlyAudioText(firstField)) {
    if (setAudioSourceOnlyLine(firstField)) {
      changed = true;
      if (bapLogging || audioSniffMode) {
        Serial.print("audio source-only update: ");
        Serial.println(audioLine1);
      }
    }
  } else if (hasFirstField && hasStationNameChars(firstField)) {
    if (strcmp(firstField, audioLine1) != 0) {
      copyAudioLine(audioLine1, firstField);
      changed = true;
      if (bapLogging || audioSniffMode) {
        Serial.print("audio station update: ");
        Serial.println(audioLine1);
      }
    }
  }

  if (changed) {
    audioTextUpdates++;
    scheduleAudioLineCacheSave();
    requestDdpRedrawIfTarget(DdpTarget::Audio);
    if (firstFieldIsFrequency && firstField[0] != '\0') {
      if (bapLogging || audioSniffMode) {
        Serial.print("audio frequency update: ");
        Serial.println(audioLine2);
      }
    }
  }

  return true;
}

void updateAudioTextFromBap(const BapDecodedFrame &frame) {
  if (!audioTranslateEnabled || frame.canId != BAP_AUDIO_CAN_ID) {
    return;
  }

  if (updateAudioFromPort21(frame)) {
    return;
  }

  char text[AUDIO_LINE_MAX] = {};
  if (!extractPrintableBapText(frame, text, sizeof(text))) {
    return;
  }
  trimAudioText(text);

  if (text[0] == '\0') {
    return;
  }

  if (isFrequencyLikeText(text)) {
    if (strcmp(text, audioLine2) == 0) {
      return;
    }
    if (strcmp(text, lastFrequencyText) != 0) {
      copyAudioLine(audioLine1, "AUDIO");
      copyAudioLine(lastFrequencyText, text);
    }
    copyAudioLine(audioLine2, text);
    Serial.print("audio frequency update: ");
    Serial.println(audioLine2);
  } else if (isSourceOnlyAudioText(text)) {
    if (!setAudioSourceOnlyLine(text)) {
      return;
    }
    Serial.print("audio source-only update: ");
    Serial.println(audioLine1);
  } else if (hasStationNameChars(text)) {
    if (strcmp(text, audioLine1) == 0) {
      return;
    }
    copyAudioLine(audioLine1, text);
    Serial.print("audio station update: ");
    Serial.println(audioLine1);
  } else {
    return;
  }

  audioTextUpdates++;
  scheduleAudioLineCacheSave();
  requestDdpRedrawIfTarget(DdpTarget::Audio);
}

void handleBapFrame(const CAN_message_t &msg, const char *direction) {
  if (!isBapCandidateFrame(msg)) {
    return;
  }

  BapDecodedFrame decoded = {};
  if (!decodeBapFrame(msg, decoded)) {
    return;
  }

  bapFramesDecoded++;
  printNavSniffBapFrame(decoded, direction);
  if (audioSniffMode) {
    printAudioSniffFrame(decoded);
  } else if (bapLogging) {
    printBapFrame(decoded);
  }
  if (msg.id == BAP_AUDIO_CAN_ID) {
    updateAudioTextFromBap(decoded);
  } else if (msg.id == BAP_NAVI_CAN_ID) {
    updateNavTextFromBap(decoded);
  }
}

bool isDimmingFrame(const CAN_message_t &msg) {
  return msg.id == DIMMING_CAN_ID && msg.len == DIMMING_FRAME_LEN && !msg.flags.extended;
}

bool isLightsOffDimmingFrame(const CAN_message_t &msg) {
  return isDimmingFrame(msg) && msg.buf[0] == 0x00 && msg.buf[1] == 0x00 && msg.buf[2] == 0x00;
}

bool isDashDimmerFrame(const CAN_message_t &msg) {
  return isDimmingFrame(msg) && msg.buf[0] != 0x00 && msg.buf[1] == 0x00 && msg.buf[2] == 0x00;
}

uint8_t scaledDimmerBrightness(uint8_t wheelValue) {
  const uint8_t clamped = wheelValue < DIMMER_WHEEL_MIN_OBSERVED ? DIMMER_WHEEL_MIN_OBSERVED :
                          wheelValue > DIMMER_WHEEL_MAX_OBSERVED ? DIMMER_WHEEL_MAX_OBSERVED :
                          wheelValue;
  const uint8_t low = nightBrightness < lightsOnMaxBrightness ? nightBrightness : lightsOnMaxBrightness;
  const uint8_t high = nightBrightness < lightsOnMaxBrightness ? lightsOnMaxBrightness : nightBrightness;
  const uint16_t inputSpan = DIMMER_WHEEL_MAX_OBSERVED - DIMMER_WHEEL_MIN_OBSERVED;
  const uint16_t outputSpan = high - low;
  return low + ((static_cast<uint16_t>(clamped - DIMMER_WHEEL_MIN_OBSERVED) * outputSpan + inputSpan / 2) / inputSpan);
}

bool needsDimmingRewrite(const CAN_message_t &msg) {
  return isLightsOffDimmingFrame(msg) || ((dimmerMirrorEnabled || dimmerScaleEnabled) && isDashDimmerFrame(msg));
}

void rewriteDimmingFrame(CAN_message_t &msg) {
  if (!needsDimmingRewrite(msg)) {
    return;
  }

  if (isLightsOffDimmingFrame(msg)) {
    msg.buf[BRIGHTNESS_BYTE_INDEX] = dayBrightness;
  } else if (dimmerScaleEnabled) {
    msg.buf[BRIGHTNESS_BYTE_INDEX] = scaledDimmerBrightness(msg.buf[0]);
  } else {
    msg.buf[BRIGHTNESS_BYTE_INDEX] = msg.buf[0];
  }
}

void printStatus() {
  Serial.println("Status:");
  Serial.print("  bypassMode=");
  Serial.println(bypassMode ? "on" : "off");
  Serial.print("  verboseLogging=");
  Serial.println(verboseLogging ? "on" : "off");
  Serial.print("  dayBrightness=0x");
  Serial.println(dayBrightness, HEX);
  Serial.print("  lightsOnMaxBrightness=0x");
  Serial.println(lightsOnMaxBrightness, HEX);
  Serial.print("  nightBrightness=0x");
  Serial.println(nightBrightness, HEX);
  Serial.print("  dimmerMirrorEnabled=");
  Serial.println(dimmerMirrorEnabled ? "on" : "off");
  Serial.print("  dimmerScaleEnabled=");
  Serial.println(dimmerScaleEnabled ? "on" : "off");
  Serial.println("  note: all-zero frames use dayBrightness; scaled headlights-on dimmer frames use nightBrightness..lightsOnMaxBrightness");
  Serial.print("  defaults day/onmax/night=0x");
  Serial.print(DEFAULT_DAY_BRIGHTNESS, HEX);
  Serial.print("/0x");
  Serial.print(DEFAULT_LIGHTS_ON_MAX_BRIGHTNESS, HEX);
  Serial.print("/0x");
  Serial.println(DEFAULT_NIGHT_BRIGHTNESS, HEX);
  Serial.print("  patchedFrames=");
  Serial.println(patchedFrames);
  Serial.print("  forwarded vehicle->radio=");
  Serial.println(forwardedFramesVehicleToRadio);
  Serial.print("  forwarded radio->vehicle=");
  Serial.println(forwardedFramesRadioToVehicle);
  Serial.print("  powerCompat 0x436 v->r/r->v=");
  Serial.print(powerCompat436VehicleToRadio);
  Serial.print("/");
  Serial.println(powerCompat436RadioToVehicle);
  Serial.print("  powerCompat 0x439 v->r/r->v=");
  Serial.print(powerCompat439VehicleToRadio);
  Serial.print("/");
  Serial.println(powerCompat439RadioToVehicle);
  Serial.print("  lastNonDdpCanActivityMsAgo=");
  if (lastNonDdpCanActivityMs == 0) {
    Serial.println("n/a");
  } else {
    Serial.println(millis() - lastNonDdpCanActivityMs);
  }
  Serial.print("  wakeBurstFrames=");
  Serial.println(vehicleWakeBurstFrames);
  Serial.print("  wakeBurstQualified=");
  Serial.println(hasQualifiedVehicleWakeBurst(millis()) ? "yes" : "no");
  Serial.print("  ddpEnabled=");
  Serial.println(ddpEnabled ? "on" : "off");
  Serial.print("  ddpBootEnabled=");
  Serial.println(ddpBootEnabled ? "on" : "off");
  Serial.print("  ddpSleepSuspended=");
  Serial.println(ddpSleepSuspended ? "yes" : "no");
  Serial.print("  ddpBootStartPending=");
  Serial.println(ddpBootStartPending ? "yes" : "no");
  if (ddpBootStartPending) {
    const uint32_t now = millis();
    const uint32_t remainingMs = ddpBootStartMs > now ? ddpBootStartMs - now : 0;
    Serial.print("  ddpBootStartInMs=");
    Serial.println(remainingMs);
  }
  Serial.print("  ddpTarget=");
  Serial.println(ddpTargetName(ddpTarget));
  Serial.print("  ddpPriority=");
  Serial.println("auto: Navigation during route guidance, Audio otherwise");
  Serial.print("  ddp tx/rx ids=0x");
  Serial.print(ddpTxCanId(), HEX);
  Serial.print("/0x");
  Serial.println(ddpRxCanId(), HEX);
  Serial.print("  ddpState=");
  Serial.println(ddpStateName(ddpState));
  Serial.print("  ddpRenderedPage=");
  Serial.print(ddpRenderedPageValid ? ddpTargetName(ddpRenderedTarget) : "none");
  Serial.print(" activeAudio=");
  Serial.println(audioPageActivelyShowing() ? "yes" : "no");
  Serial.print("  ddp tx/rx/acks/draws=");
  Serial.print(ddpTxFrames);
  Serial.print("/");
  Serial.print(ddpRxFrames);
  Serial.print("/");
  Serial.print(ddpAcksReceived);
  Serial.print("/");
  Serial.println(ddpDrawsCompleted);
  Serial.print("  audioTranslateEnabled=");
  Serial.println(audioTranslateEnabled ? "on" : "off");
  Serial.print("  navTranslateEnabled=");
  Serial.println(navTranslateEnabled ? "on" : "off");
  Serial.print("  navArrowHintsEnabled=");
  Serial.println(navArrowHintsEnabled ? "on" : "off");
  Serial.print("  steeringAudioButtonsEnabled=");
  Serial.println(steeringAudioButtonsEnabled ? "on" : "off");
  Serial.print("  steeringButtonLogging=");
  Serial.println(steeringButtonLogging ? "on" : "off");
  Serial.print("  steering next/prev translations=");
  Serial.print(steeringNextTranslations);
  Serial.print("/");
  Serial.println(steeringPrevTranslations);
  Serial.print("  navShowManeuverCodes=");
  Serial.println(navShowManeuverCodes ? "on" : "off");
  Serial.print("  navRouteActive=");
  Serial.println(navRouteActive ? "yes" : "no");
  Serial.print("  navManeuver=kind 0x");
  Serial.print(navManeuverKind, HEX);
  Serial.print(" code 0x");
  Serial.print(navManeuverCode, HEX);
  Serial.print(" main 0x");
  Serial.print(navManeuverMainElement, HEX);
  Serial.print(" dir 0x");
  Serial.print(navManeuverDirection, HEX);
  Serial.print(" off 0x");
  Serial.print(navManeuverRecordOffset, HEX);
  Serial.print(" text=");
  Serial.println(navManeuverText);
  Serial.print("  bapLogging=");
  Serial.println(bapLogging ? "on" : "off");
  Serial.print("  bapLogAllCandidates=");
  Serial.println(bapLogAllCandidates ? "on" : "off");
  Serial.print("  audioSniffMode=");
  Serial.println(audioSniffMode ? "on" : "off");
  Serial.print("  navSniffMode=");
  Serial.println(navSniffMode ? "on" : "off");
  Serial.print("  clockSniffMode=");
  Serial.println(clockSniffMode ? "on" : "off");
  Serial.print("  bapFramesDecoded=");
  Serial.println(bapFramesDecoded);
  Serial.print("  audioTextUpdates=");
  Serial.println(audioTextUpdates);
  Serial.print("  navTextUpdates=");
  Serial.println(navTextUpdates);
  Serial.print("  navManeuverUpdates=");
  Serial.println(navManeuverUpdates);
  Serial.print("  navSniffFrames=");
  Serial.println(navSniffFrames);
  Serial.print("  clockSniffFrames=");
  Serial.println(clockSniffFrames);
  Serial.print("  audioLine1=");
  Serial.println(audioLine1);
  Serial.print("  audioLine2=");
  Serial.println(audioLine2);
  Serial.print("  audioLineCacheDirty=");
  Serial.println(audioLineCacheDirty ? "yes" : "no");
  Serial.print("  navLine1=");
  Serial.println(navLine1);
  Serial.print("  navLine2=");
  Serial.println(navLine2);
}

bool parseHexByte(const char *text, uint8_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  unsigned long parsed = strtoul(text, &endPtr, 16);
  if (*endPtr != '\0' || parsed > 0xFF) {
    return false;
  }

  value = static_cast<uint8_t>(parsed);
  return true;
}

uint8_t storedDdpTargetValue(DdpTarget target) {
  switch (target) {
    case DdpTarget::Audio: return 0;
    case DdpTarget::Navigation: return 1;
  }
  return 0;
}

DdpTarget ddpTargetFromStoredValue(uint8_t value) {
  return value == 1 ? DdpTarget::Navigation : DdpTarget::Audio;
}

bool boolFromStoredValue(uint8_t value, bool defaultValue) {
  if (value == 0) {
    return false;
  }
  if (value == 1) {
    return true;
  }
  return defaultValue;
}

void saveSettings() {
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR_DAY, dayBrightness);
  EEPROM.put(EEPROM_ADDR_LIGHTS_ON_MAX, lightsOnMaxBrightness);
  EEPROM.put(EEPROM_ADDR_NIGHT, nightBrightness);
  EEPROM.put(EEPROM_ADDR_DDP_ENABLED, static_cast<uint8_t>(ddpBootEnabled ? 1 : 0));
  EEPROM.put(EEPROM_ADDR_DDP_TARGET, storedDdpTargetValue(ddpTarget));
  EEPROM.put(EEPROM_ADDR_AUDIO_TRANSLATE, static_cast<uint8_t>(audioTranslateEnabled ? 1 : 0));
  EEPROM.put(EEPROM_ADDR_NAV_TRANSLATE, static_cast<uint8_t>(navTranslateEnabled ? 1 : 0));
  EEPROM.put(EEPROM_ADDR_STEERING_AUDIO_BUTTONS, static_cast<uint8_t>(steeringAudioButtonsEnabled ? 1 : 0));
  EEPROM.put(EEPROM_ADDR_DIMMER_MIRROR, static_cast<uint8_t>(dimmerMirrorEnabled ? 1 : 0));
  EEPROM.put(EEPROM_ADDR_DIMMER_SCALE, static_cast<uint8_t>(dimmerScaleEnabled ? 1 : 0));
  EEPROM.put(EEPROM_ADDR_NAV_ARROW_HINTS, static_cast<uint8_t>(navArrowHintsEnabled ? 1 : 0));
}

void loadSettings() {
  uint32_t storedMagic = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, storedMagic);

  if (storedMagic != EEPROM_MAGIC) {
    dayBrightness = DEFAULT_DAY_BRIGHTNESS;
    lightsOnMaxBrightness = DEFAULT_LIGHTS_ON_MAX_BRIGHTNESS;
    nightBrightness = DEFAULT_NIGHT_BRIGHTNESS;
    ddpBootEnabled = DEFAULT_DDP_BOOT_ENABLED;
    ddpTarget = DdpTarget::Audio;
    audioTranslateEnabled = DEFAULT_AUDIO_TRANSLATE_ENABLED;
    navTranslateEnabled = DEFAULT_NAV_TRANSLATE_ENABLED;
    navArrowHintsEnabled = DEFAULT_NAV_ARROW_HINTS_ENABLED;
    steeringAudioButtonsEnabled = DEFAULT_STEERING_AUDIO_BUTTONS_ENABLED;
    dimmerMirrorEnabled = DEFAULT_DIMMER_MIRROR_ENABLED;
    dimmerScaleEnabled = DEFAULT_DIMMER_SCALE_ENABLED;
    saveSettings();
    return;
  }

  EEPROM.get(EEPROM_ADDR_DAY, dayBrightness);
  EEPROM.get(EEPROM_ADDR_LIGHTS_ON_MAX, lightsOnMaxBrightness);
  EEPROM.get(EEPROM_ADDR_NIGHT, nightBrightness);

  uint8_t storedDdpEnabled = 0;
  EEPROM.get(EEPROM_ADDR_DDP_ENABLED, storedDdpEnabled);
  ddpBootEnabled = boolFromStoredValue(storedDdpEnabled, DEFAULT_DDP_BOOT_ENABLED);

  uint8_t storedDdpTarget = 0;
  EEPROM.get(EEPROM_ADDR_DDP_TARGET, storedDdpTarget);
  ddpTarget = ddpTargetFromStoredValue(storedDdpTarget);
  if (!navRouteActive) {
    ddpTarget = DdpTarget::Audio;
  }

  uint8_t storedAudioTranslate = 0xFF;
  EEPROM.get(EEPROM_ADDR_AUDIO_TRANSLATE, storedAudioTranslate);
  audioTranslateEnabled = boolFromStoredValue(storedAudioTranslate, DEFAULT_AUDIO_TRANSLATE_ENABLED);

  uint8_t storedNavTranslate = 0xFF;
  EEPROM.get(EEPROM_ADDR_NAV_TRANSLATE, storedNavTranslate);
  navTranslateEnabled = boolFromStoredValue(storedNavTranslate, DEFAULT_NAV_TRANSLATE_ENABLED);

  uint8_t storedNavArrowHints = 0xFF;
  EEPROM.get(EEPROM_ADDR_NAV_ARROW_HINTS, storedNavArrowHints);
  navArrowHintsEnabled = boolFromStoredValue(storedNavArrowHints, DEFAULT_NAV_ARROW_HINTS_ENABLED);

  uint8_t storedSteeringAudioButtons = 0xFF;
  EEPROM.get(EEPROM_ADDR_STEERING_AUDIO_BUTTONS, storedSteeringAudioButtons);
  steeringAudioButtonsEnabled = boolFromStoredValue(storedSteeringAudioButtons, DEFAULT_STEERING_AUDIO_BUTTONS_ENABLED);

  uint8_t storedDimmerMirror = 0xFF;
  EEPROM.get(EEPROM_ADDR_DIMMER_MIRROR, storedDimmerMirror);
  dimmerMirrorEnabled = boolFromStoredValue(storedDimmerMirror, DEFAULT_DIMMER_MIRROR_ENABLED);

  uint8_t storedDimmerScale = 0xFF;
  EEPROM.get(EEPROM_ADDR_DIMMER_SCALE, storedDimmerScale);
  dimmerScaleEnabled = boolFromStoredValue(storedDimmerScale, DEFAULT_DIMMER_SCALE_ENABLED);
  if (dimmerScaleEnabled) {
    dimmerMirrorEnabled = false;
  }

  loadAudioLineCache();
}

void restoreDefaults() {
  dayBrightness = DEFAULT_DAY_BRIGHTNESS;
  lightsOnMaxBrightness = DEFAULT_LIGHTS_ON_MAX_BRIGHTNESS;
  nightBrightness = DEFAULT_NIGHT_BRIGHTNESS;
  ddpBootEnabled = DEFAULT_DDP_BOOT_ENABLED;
  ddpTarget = DdpTarget::Audio;
  audioTranslateEnabled = DEFAULT_AUDIO_TRANSLATE_ENABLED;
  navTranslateEnabled = DEFAULT_NAV_TRANSLATE_ENABLED;
  navArrowHintsEnabled = DEFAULT_NAV_ARROW_HINTS_ENABLED;
  steeringAudioButtonsEnabled = DEFAULT_STEERING_AUDIO_BUTTONS_ENABLED;
  dimmerMirrorEnabled = DEFAULT_DIMMER_MIRROR_ENABLED;
  dimmerScaleEnabled = DEFAULT_DIMMER_SCALE_ENABLED;
  steeringButtonLogging = false;
  steeringTranslatedActive = false;
  setAudioLines("AUDIO", "Waiting radio");
  copyAudioLine(lastFrequencyText, "");
  audioLineCacheDirty = false;
  EEPROM.write(EEPROM_ADDR_AUDIO_CACHE_VALID, 0);
  saveSettings();
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h            - help");
  Serial.println("  s            - status");
  Serial.println("  l            - toggle verbose logging");
  Serial.println("  b            - toggle bypass mode");
  Serial.println("  day XX       - set day brightness hex byte, example: day D0");
  Serial.println("  onmax XX     - set the maximum brightness while headlights are on, example: onmax C0");
  Serial.println("  night XX     - set scaled dimmer low brightness byte, example: night 20");
  Serial.println("  dim pass     - pass dash-dimmer frames unchanged");
  Serial.println("  dim mirror   - populate byte 2 with the raw dash-dimmer byte");
  Serial.println("  dim scale    - scale dash-dimmer byte into night/day brightness range");
  Serial.println("  save         - persist brightness, DDP, Audio, and Nav settings to EEPROM");
  Serial.println("  defaults     - restore production defaults and save them");
  Serial.println("  ddp on       - enable DDP static MFD test page");
  Serial.println("  ddp off      - disable DDP test mode");
  Serial.println("  ddp audio    - debug-select Audio DDP channel 0x680/0x681");
  Serial.println("  ddp nav      - debug-select Navigation DDP channel 0x682/0x683");
  Serial.println("  ddp redraw   - request the DDP test page again");
  Serial.println("  ddp log      - toggle DDP verbose logging");
  Serial.println("  audio on/off - enable or disable BAP->DDP audio text translation");
  Serial.println("  audio log    - toggle decoded BAP audio logging");
  Serial.println("  audio all    - toggle logging candidate BAP IDs 0x660-0x66F");
  Serial.println("  audio sniff  - toggle focused audio text/RDS sniffing on IDs 0x660-0x66F");
  Serial.println("  audio test   - draw a manual audio test line on the MFD");
  Serial.println("  audio clear  - reset Audio page text to defaults");
  Serial.println("  steering on/off/log - gate MFD up/down as radio next/prev only while Audio page is active");
  Serial.println("  nav on/off   - enable or disable BAP_NAVI->DDP nav text translation");
  Serial.println("  nav arrows   - toggle experimental left/right maneuver hints from observed MIB2 code families");
  Serial.println("  nav codes    - toggle raw maneuver code display, e.g. M35");
  Serial.println("  nav sniff    - toggle focused nav sniffing on 0x67C, 0x67D, and 0x3A2");
  Serial.println("  clock sniff  - toggle focused clock sniffing on 0x623, 0x65D, and nearby 0x65x candidates");
  Serial.println("  wakeids      - show recently active CAN IDs on both sides of the bridge");
  Serial.println("  nav test     - draw a manual nav test line on the MFD");
  Serial.println("  nav clear    - reset Navigation page text to defaults");
  Serial.println("  reset        - clear frame counters");
}

void resetCounters() {
  patchedFrames = 0;
  forwardedFramesVehicleToRadio = 0;
  forwardedFramesRadioToVehicle = 0;
  powerCompat436VehicleToRadio = 0;
  powerCompat439VehicleToRadio = 0;
  powerCompat436RadioToVehicle = 0;
  powerCompat439RadioToVehicle = 0;
  clearObservedCanIds(vehicleObservedIds, WAKE_ID_COUNTER_SLOTS);
  clearObservedCanIds(radioObservedIds, WAKE_ID_COUNTER_SLOTS);
  ddpResetCounters();
  bapFramesDecoded = 0;
  audioTextUpdates = 0;
  navTextUpdates = 0;
  navManeuverUpdates = 0;
  navSniffFrames = 0;
  clockSniffFrames = 0;
  steeringNextTranslations = 0;
  steeringPrevTranslations = 0;
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

  if (strcmp(cmd, "l") == 0 || strcmp(cmd, "log") == 0) {
    verboseLogging = !verboseLogging;
    Serial.print("verboseLogging=");
    Serial.println(verboseLogging ? "on" : "off");
    return;
  }

  if (strcmp(cmd, "b") == 0 || strcmp(cmd, "bypass") == 0) {
    bypassMode = !bypassMode;
    Serial.print("bypassMode=");
    Serial.println(bypassMode ? "on" : "off");
    return;
  }

  if (strcmp(cmd, "reset") == 0) {
    resetCounters();
    Serial.println("Counters reset");
    return;
  }

  if (strcmp(cmd, "wakeids") == 0) {
    printWakeIdActivity();
    return;
  }

  if (strcmp(cmd, "dim") == 0 || strcmp(cmd, "dimmer") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "pass") == 0 || strcmp(subcommand, "off") == 0) {
      dimmerMirrorEnabled = false;
      dimmerScaleEnabled = false;
      Serial.println("dimmerMode=pass");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "mirror") == 0) {
      dimmerMirrorEnabled = true;
      dimmerScaleEnabled = false;
      Serial.println("dimmerMode=mirror");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "scale") == 0) {
      dimmerMirrorEnabled = false;
      dimmerScaleEnabled = true;
      Serial.println("dimmerMode=scale");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    Serial.println("Unknown dim command. Use: dim pass/mirror/scale/status");
    return;
  }

  if (strcmp(cmd, "steering") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "on") == 0) {
      steeringAudioButtonsEnabled = true;
      saveSettings();
      Serial.println("Steering Audio-page next/previous enabled");
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "off") == 0) {
      steeringAudioButtonsEnabled = false;
      steeringTranslatedActive = false;
      saveSettings();
      Serial.println("Steering Audio-page next/previous disabled");
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "log") == 0) {
      steeringButtonLogging = !steeringButtonLogging;
      Serial.print("steeringButtonLogging=");
      Serial.println(steeringButtonLogging ? "on" : "off");
      return;
    }

    Serial.println("Unknown steering command. Use: steering on/off/log/status");
    return;
  }

  if (strcmp(cmd, "ddp") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "on") == 0) {
      ddpStart();
      Serial.println("DDP test mode enabled");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "off") == 0) {
      ddpStop();
      Serial.println("DDP test mode disabled");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "audio") == 0) {
      ddpTarget = DdpTarget::Audio;
      if (ddpEnabled) {
        ddpStart();
      }
      Serial.println("DDP target=Audio");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "nav") == 0 || strcmp(subcommand, "navigation") == 0) {
      ddpTarget = DdpTarget::Navigation;
      if (ddpEnabled) {
        ddpStart();
      }
      Serial.println("DDP target=Navigation");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "redraw") == 0) {
      if (!ddpEnabled) {
        Serial.println("DDP is off. Use 'ddp on' first.");
      } else {
        requestDdpForceRedraw();
        Serial.println("DDP forced redraw requested");
      }
      return;
    }

    if (strcmp(subcommand, "log") == 0) {
      ddpVerbose = !ddpVerbose;
      Serial.print("ddpVerbose=");
      Serial.println(ddpVerbose ? "on" : "off");
      return;
    }

    Serial.println("Unknown DDP command. Use: ddp on/off/audio/nav/redraw/log/status");
    return;
  }

  if (strcmp(cmd, "audio") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "on") == 0) {
      audioTranslateEnabled = true;
      Serial.println("audioTranslateEnabled=on");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "off") == 0) {
      audioTranslateEnabled = false;
      Serial.println("audioTranslateEnabled=off");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "log") == 0) {
      bapLogging = !bapLogging;
      Serial.print("bapLogging=");
      Serial.println(bapLogging ? "on" : "off");
      return;
    }

    if (strcmp(subcommand, "all") == 0) {
      bapLogAllCandidates = !bapLogAllCandidates;
      Serial.print("bapLogAllCandidates=");
      Serial.println(bapLogAllCandidates ? "on" : "off");
      return;
    }

    if (strcmp(subcommand, "sniff") == 0) {
      audioSniffMode = !audioSniffMode;
      if (audioSniffMode) {
        bapLogging = false;
      }
      Serial.print("audioSniffMode=");
      Serial.println(audioSniffMode ? "on" : "off");
      return;
    }

    if (strcmp(subcommand, "test") == 0) {
      copyAudioLine(audioLine1, "TEST STATION");
      copyAudioLine(audioLine2, "101.5");
      copyAudioLine(lastFrequencyText, "101.5");
      audioTextUpdates++;
      requestDdpForceRedraw();
      Serial.println("Audio test redraw requested");
      return;
    }

    if (strcmp(subcommand, "clear") == 0) {
      copyAudioLine(audioLine1, "AUDIO");
      copyAudioLine(audioLine2, "Waiting radio");
      copyAudioLine(lastFrequencyText, "");
      requestDdpForceRedraw();
      Serial.println("Audio text cleared");
      return;
    }

    Serial.println("Unknown audio command. Use: audio on/off/log/all/sniff/test/clear/status");
    return;
  }

  if (strcmp(cmd, "nav") == 0 || strcmp(cmd, "navigation") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "on") == 0) {
      navTranslateEnabled = true;
      Serial.println("navTranslateEnabled=on");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "off") == 0) {
      navTranslateEnabled = false;
      Serial.println("navTranslateEnabled=off");
      saveSettings();
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "codes") == 0) {
      navShowManeuverCodes = !navShowManeuverCodes;
      rebuildNavManeuverText();
      Serial.print("navShowManeuverCodes=");
      Serial.println(navShowManeuverCodes ? "on" : "off");
      return;
    }

    if (strcmp(subcommand, "arrows") == 0 || strcmp(subcommand, "hint") == 0 || strcmp(subcommand, "hints") == 0) {
      navArrowHintsEnabled = !navArrowHintsEnabled;
      rebuildNavManeuverText();
      saveSettings();
      Serial.print("navArrowHintsEnabled=");
      Serial.println(navArrowHintsEnabled ? "on" : "off");
      Serial.println("Settings saved");
      return;
    }

    if (strcmp(subcommand, "sniff") == 0) {
      navSniffMode = !navSniffMode;
      if (navSniffMode) {
        bapLogging = false;
      }
      Serial.print("navSniffMode=");
      Serial.println(navSniffMode ? "on" : "off");
      Serial.println("Watching DBC nav candidates: 0x67C BAP_ASG_07, 0x67D BAP_NAVI, 0x3A2 PSD_02");
      return;
    }

    if (strcmp(subcommand, "test") == 0) {
      navRouteActive = true;
      copyAudioLine(navDistanceText, "450 ft");
      navManeuverKind = 0x04;
      navManeuverCode = 0x35;
      rebuildNavManeuverText();
      copyAudioLine(navNextStreetText, "B St");
      copyAudioLine(navCurrentStreetText, "Spring St");
      refreshNavLines();
      applyAutomaticDdpPriority();
      Serial.println("Navigation test redraw requested");
      return;
    }

    if (strcmp(subcommand, "clear") == 0) {
      clearNavText();
      Serial.println("Navigation text cleared");
      return;
    }

    Serial.println("Unknown nav command. Use: nav on/off/arrows/codes/sniff/test/clear/status");
    return;
  }

  if (strcmp(cmd, "clock") == 0) {
    char *subcommand = strtok(nullptr, " \t");
    if (subcommand == nullptr || strcmp(subcommand, "status") == 0) {
      printStatus();
      return;
    }

    if (strcmp(subcommand, "sniff") == 0) {
      clockSniffMode = !clockSniffMode;
      Serial.print("clockSniffMode=");
      Serial.println(clockSniffMode ? "on" : "off");
      Serial.println("Watching clock candidates: 0x623, 0x65D, 0x621, 0x651, 0x653, 0x655, 0x658, 0x65F, 0x661, 0x665");
      return;
    }

    Serial.println("Unknown clock command. Use: clock sniff/status");
    return;
  }

  if (strcmp(cmd, "save") == 0) {
    saveSettings();
    Serial.println("Settings saved");
    return;
  }

  if (strcmp(cmd, "defaults") == 0) {
    restoreDefaults();
    Serial.println("Defaults restored and saved");
    printStatus();
    return;
  }

  if (strcmp(cmd, "day") == 0 || strcmp(cmd, "onmax") == 0 ||
      strcmp(cmd, "night") == 0) {
    char *valueText = strtok(nullptr, " \t");
    uint8_t parsedValue = 0;
    if (!parseHexByte(valueText, parsedValue)) {
      Serial.println("Invalid value. Use a hex byte like D0 or 50.");
      return;
    }

    if (strcmp(cmd, "day") == 0) {
      dayBrightness = parsedValue;
      Serial.print("dayBrightness=0x");
      Serial.println(dayBrightness, HEX);
    } else if (strcmp(cmd, "onmax") == 0) {
      lightsOnMaxBrightness = parsedValue;
      Serial.print("lightsOnMaxBrightness=0x");
      Serial.println(lightsOnMaxBrightness, HEX);
    } else {
      nightBrightness = parsedValue;
      Serial.print("nightBrightness=0x");
      Serial.println(nightBrightness, HEX);
    }
    saveSettings();
    Serial.println("Settings saved");
    if (strcmp(cmd, "night") == 0) {
      Serial.println("Note: nightBrightness is the low endpoint for 'dim scale' mode.");
    } else if (strcmp(cmd, "onmax") == 0) {
      Serial.println("Note: lightsOnMaxBrightness is the high endpoint for 'dim scale' mode while headlights are on.");
    }
    return;
  }

  Serial.println("Unknown command. Send 'h' for help.");
}

void handleVehicleToRadio() {
  CAN_message_t msg;
  while (canVehicle.read(msg)) {
    noteObservedCanId(msg, vehicleObservedIds, WAKE_ID_COUNTER_SLOTS);
    if (msg.id == POWER_COMPAT_CAN_ID_436) {
      powerCompat436VehicleToRadio++;
    } else if (msg.id == POWER_COMPAT_CAN_ID_439) {
      powerCompat439VehicleToRadio++;
    }
    if (isVehicleWakeActivityFrame(msg)) {
      noteVehicleWakeActivity();
    }
    forwardedFramesVehicleToRadio++;
    printClockSniffFrame(msg, "vehicle->radio");
    printRawNavSniffFrame(msg, "vehicle->radio");
    handleBapFrame(msg, "vehicle->radio");

    if (handleDdpVehicleFrame(msg)) {
      continue;
    }

    rewriteSteeringButtonsForAudioPage(msg);

    if (isDimmingFrame(msg)) {
      CAN_message_t original = msg;
      const bool shouldPatch = !bypassMode && needsDimmingRewrite(msg);
      if (shouldPatch) {
        rewriteDimmingFrame(msg);
        patchedFrames++;
      }

      if (verboseLogging) {
        printFrame("dimming original", original);
        printFrame(shouldPatch ? "dimming patched " : "dimming passed  ", msg);
      }
    }

    canRadio.write(msg);
  }
}

void handleRadioToVehicle() {
  CAN_message_t msg;
  while (canRadio.read(msg)) {
    noteObservedCanId(msg, radioObservedIds, WAKE_ID_COUNTER_SLOTS);
    if (msg.id == POWER_COMPAT_CAN_ID_436) {
      powerCompat436RadioToVehicle++;
    } else if (msg.id == POWER_COMPAT_CAN_ID_439) {
      powerCompat439RadioToVehicle++;
    }
    forwardedFramesRadioToVehicle++;
    printClockSniffFrame(msg, "radio->vehicle");
    printRawNavSniffFrame(msg, "radio->vehicle");
    handleBapFrame(msg, "radio->vehicle");
    canVehicle.write(msg);
  }
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

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("Mk5 MIB2 dimming bridge starting");
  Serial.println("Assumed bus speed: 100000");
  Serial.println("Dimming frame: 0x635");
  Serial.println("Send 'h' over USB serial for commands.");

  EEPROM.begin();
  loadSettings();

  setupCan(canVehicle);
  setupCan(canRadio);
  lastNonDdpCanActivityMs = millis();

  if (ddpBootEnabled) {
    scheduleDdpBootStart();
  }

  printStatus();
}

void loop() {
  handleVehicleToRadio();
  handleRadioToVehicle();
  ddpTick();
  tickAudioLineCacheSave();
  processSerialCommands();
}
