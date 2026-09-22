#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <vector>

#define DEVICE_NAME         "test"
#define SERVO_PIN           19

#define GATE_CLOSED_ANGLE   0
#define GATE_OPEN_ANGLE     90
#define STEP_DELAY_MS       2

#define BEACON_INTERVAL_MS  1000
#define ACK_ENABLED         true

static const uint8_t ESPNOW_PMK[16] = "MyPMKKey1234567";
static const uint8_t ESPNOW_LMK[16] = "MyLMKKey1234567";

#define ANTIREPLAY_SECRET   0x5A17C0DEUL
#define SEQ_PERSIST_BUFFER  50UL

Preferences prefs;

enum PacketType : uint8_t {
  PACKET_ANNOUNCE     = 0,
  PACKET_CMD          = 1,
  PACKET_ACK          = 2
};

enum GateState : int { GATE_CLOSED = 0, GATE_OPEN = 1 };
enum GateCmd   : int { CMD_CLOSE = 0, CMD_OPEN = 1, CMD_TOGGLE = 2 };

typedef struct {
  uint8_t  type;
  char     deviceName[16];
  int      payload;
  uint32_t seq;
  uint32_t ackSeq;
  uint32_t auth;
} DevicePacket;

struct MasterInfo {
  uint8_t  mac[6];
  uint32_t lastCmdSeq;
  uint32_t seqThreshold;
  int      cachedAckState;
};

std::vector<MasterInfo> masters;

uint32_t computeAuth(uint8_t type, const char *deviceName, int payload, uint32_t seq, uint32_t ackSeq)
{
  uint32_t h = 2166136261UL ^ ANTIREPLAY_SECRET;
  h ^= type;              h *= 16777619UL;
  for (int i = 0; i < 16 && deviceName[i] != '\0'; i++) { h ^= (uint8_t)deviceName[i]; h *= 16777619UL; }
  h ^= (uint32_t)payload; h *= 16777619UL;
  h ^= seq;               h *= 16777619UL;
  h ^= ackSeq;            h *= 16777619UL;
  h ^= ANTIREPLAY_SECRET; h *= 16777619UL;
  return h;
}

bool macEqual(const uint8_t *a, const uint8_t *b) { return memcmp(a, b, 6) == 0; }

String macKey(const uint8_t *mac)
{
  char buf[13];
  snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

MasterInfo* findMaster(const uint8_t *mac)
{
  for (auto &m : masters)
    if (macEqual(m.mac, mac)) return &m;
  return nullptr;
}

MasterInfo& getOrCreateMaster(const uint8_t *mac)
{
  MasterInfo *m = findMaster(mac);
  if (m) return *m;

  MasterInfo nm = {};
  memcpy(nm.mac, mac, 6);
  prefs.begin("anrb", true);
  nm.seqThreshold = prefs.getUInt(macKey(mac).c_str(), 0);
  prefs.end();
  nm.lastCmdSeq = nm.seqThreshold;
  masters.push_back(nm);
  return masters.back();
}

void bumpMasterThreshold(MasterInfo *m, uint32_t seq)
{
  if (seq >= m->seqThreshold) {
    m->seqThreshold = seq + SEQ_PERSIST_BUFFER;
    prefs.begin("anrb", false);
    prefs.putUInt(macKey(m->mac).c_str(), m->seqThreshold);
    prefs.end();
  }
}

Servo *myServo = nullptr;
int currentAngle = GATE_CLOSED_ANGLE;
int targetAngle  = GATE_CLOSED_ANGLE;
GateState currentState = GATE_CLOSED;

unsigned long lastBeacon = 0;
uint8_t broadcastAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint32_t outSeq       = 0;
uint32_t outSeqAnchor = 0;

void ensurePeer(const uint8_t *mac, bool encrypted) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t pi = {};
    memcpy(pi.peer_addr, mac, 6);
    pi.channel = 0;
    pi.encrypt = encrypted;
    if (encrypted) memcpy(pi.lmk, ESPNOW_LMK, 16);
    esp_now_add_peer(&pi);
  }
}

void initOutgoingSeq() {
  prefs.begin("espnow", false);
  outSeqAnchor = prefs.getUInt("seqAnchor", 0);
  outSeq = outSeqAnchor;
  outSeqAnchor = outSeq + SEQ_PERSIST_BUFFER;
  prefs.putUInt("seqAnchor", outSeqAnchor);
  prefs.end();
}

uint32_t nextOutgoingSeq() {
  outSeq++;
  if (outSeq >= outSeqAnchor) {
    outSeqAnchor = outSeq + SEQ_PERSIST_BUFFER;
    prefs.begin("espnow", false);
    prefs.putUInt("seqAnchor", outSeqAnchor);
    prefs.end();
  }
  return outSeq;
}

void sendAck(const uint8_t *destMac, int state, uint32_t ackOfSeq) {
  ensurePeer(destMac, true);
  DevicePacket pkt = {};
  pkt.type = PACKET_ACK;
  strncpy(pkt.deviceName, DEVICE_NAME, sizeof(pkt.deviceName)-1);
  pkt.payload = state;
  pkt.seq     = nextOutgoingSeq();
  pkt.ackSeq  = ackOfSeq;
  pkt.auth    = computeAuth(pkt.type, pkt.deviceName, pkt.payload, pkt.seq, pkt.ackSeq);
  esp_now_send(destMac, (uint8_t*)&pkt, sizeof(pkt));
}

void sendBeacon() {
  DevicePacket pkt = {};
  pkt.type    = PACKET_ANNOUNCE;
  strncpy(pkt.deviceName, DEVICE_NAME, sizeof(pkt.deviceName)-1);
  pkt.payload = (int)currentState;
  pkt.seq     = nextOutgoingSeq();
  pkt.ackSeq  = 0;
  pkt.auth    = computeAuth(pkt.type, pkt.deviceName, pkt.payload, pkt.seq, pkt.ackSeq);
  esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}

void setGate(GateState newState) {
  if (currentState == newState) return;
  currentState = newState;
  if (currentState == GATE_OPEN) {
    targetAngle = GATE_OPEN_ANGLE;
    Serial.println("[S] OPEN");
  } else {
    targetAngle = GATE_CLOSED_ANGLE;
    Serial.println("[S] CLOSE");
  }
}

void toggleGate() {
  setGate(currentState == GATE_CLOSED ? GATE_OPEN : GATE_CLOSED);
}

void updateServo() {
  if (!myServo) return;
  static unsigned long lastStep = 0;
  if (currentAngle != targetAngle && millis() - lastStep >= STEP_DELAY_MS) {
    lastStep = millis();
    currentAngle += (currentAngle < targetAngle) ? 1 : -1;
    myServo->write(currentAngle);
  }
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len != sizeof(DevicePacket)) return;

  DevicePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.auth != computeAuth(pkt.type, pkt.deviceName, pkt.payload, pkt.seq, pkt.ackSeq)) {
    return;
  }

  if (pkt.type == PACKET_CMD) {
    if (strncmp(pkt.deviceName, DEVICE_NAME, sizeof(pkt.deviceName)) != 0) return;

    MasterInfo &m = getOrCreateMaster(mac_addr);

    if (pkt.seq < m.lastCmdSeq) return;
    if (pkt.seq == m.lastCmdSeq) {
      if (ACK_ENABLED) sendAck(mac_addr, m.cachedAckState, pkt.seq);
      return;
    }

    m.lastCmdSeq = pkt.seq;
    bumpMasterThreshold(&m, pkt.seq);

    if      (pkt.payload == CMD_OPEN)   setGate(GATE_OPEN);
    else if (pkt.payload == CMD_CLOSE)  setGate(GATE_CLOSED);
    else if (pkt.payload == CMD_TOGGLE) toggleGate();

    m.cachedAckState = (int)currentState;
    if (ACK_ENABLED) sendAck(mac_addr, m.cachedAckState, pkt.seq);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[S] ESP-NOW init FAIL");
    return;
  }

  esp_now_set_pmk(ESPNOW_PMK);
  ensurePeer(broadcastAddr, false);

  initOutgoingSeq();
  esp_now_register_recv_cb(onDataRecv);

  myServo = new Servo();
  myServo->setPeriodHertz(50);
  myServo->attach(SERVO_PIN, 500, 2400);
  myServo->write(currentAngle);

  Serial.println("[S] Ready");
}

void loop() {
  if (millis() - lastBeacon >= BEACON_INTERVAL_MS) {
    lastBeacon = millis();
    sendBeacon();
  }
  updateServo();
}
