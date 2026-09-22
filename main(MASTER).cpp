#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LovyanGFX.hpp>
#include <vector>
#include <math.h>

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7735S _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;

public:
  LGFX()
  {
    auto cfg = _bus.config();
    cfg.spi_host = SPI2_HOST;
    cfg.spi_mode = 0;
    cfg.freq_write = 27000000;
    cfg.freq_read = 16000000;
    cfg.spi_3wire = false;
    cfg.use_lock = true;
    cfg.dma_channel = SPI_DMA_CH_AUTO;
    cfg.pin_sclk = 5;
    cfg.pin_mosi = 3;
    cfg.pin_miso = -1;
    cfg.pin_dc = 2;
    _bus.config(cfg);
    _panel.setBus(&_bus);

    auto pcfg = _panel.config();
    pcfg.pin_cs = 4;
    pcfg.pin_rst = 1;
    pcfg.pin_busy = -1;
    pcfg.panel_width = 80;
    pcfg.panel_height = 160;
    pcfg.memory_width = 132;
    pcfg.memory_height = 162;
    pcfg.offset_x = 26;
    pcfg.offset_y = 1;
    pcfg.offset_rotation = 0;
    pcfg.dummy_read_pixel = 8;
    pcfg.dummy_read_bits = 1;
    pcfg.readable = false;
    pcfg.invert = true;
    pcfg.rgb_order = true;
    pcfg.dlen_16bit = false;
    pcfg.bus_shared = false;
    _panel.config(pcfg);

    auto lcfg = _light.config();
    lcfg.pin_bl = 38;
    lcfg.invert = true;
    lcfg.freq = 12000;
    lcfg.pwm_channel = 7;
    _light.config(lcfg);
    _panel.setLight(&_light);

    setPanel(&_panel);
  }
};

LGFX tft;

#define BTN_PIN           0
#define LONG_PRESS_MS     500
#define SLAVE_TIMEOUT_MS  5000

enum PacketType : uint8_t {
  PACKET_ANNOUNCE = 0,
  PACKET_CMD      = 1,
  PACKET_ACK      = 2
};

enum GateState : int {
  GATE_CLOSED = 0,
  GATE_OPEN   = 1
};

enum GateCmd : int {
  CMD_CLOSE  = 0,
  CMD_OPEN   = 1,
  CMD_TOGGLE = 2
};

typedef struct {
  uint8_t type;
  char    deviceName[16];
  int     payload;
} DevicePacket;

struct SlaveInfo {
  char deviceName[16];
  uint8_t mac[6];
  unsigned long lastSeen;
  int  gateState;
  bool acked;
};

std::vector<SlaveInfo> discoveredSlaves;
int selectedSlaveIndex = -1;

bool btnWasPressed = false;
unsigned long btnPressStart = 0;
unsigned long lastDraw = 0;

float animAngle = 0.0f;
uint8_t pulseTick = 0;

#define COLOR_BG        TFT_BLACK
#define COLOR_TEXT      TFT_WHITE
#define COLOR_SELECTED  TFT_CYAN
#define COLOR_DIM       0x4208
#define COLOR_RED       0xF800
#define COLOR_YELLOW    0xFFE0
#define COLOR_GREEN     0x07E0
#define COLOR_ROAD      0x2104

bool macEqual(const uint8_t *a, const uint8_t *b)
{
  return memcmp(a, b, 6) == 0;
}

void ensurePeer(const uint8_t *mac)
{
  if (!esp_now_is_peer_exist(mac))
  {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

int findSlaveByMac(const uint8_t *mac)
{
  for (size_t i = 0; i < discoveredSlaves.size(); i++)
  {
    if (macEqual(discoveredSlaves[i].mac, mac))
      return (int)i;
  }
  return -1;
}

int signalLevelFromAge(unsigned long ageMs)
{
  if (ageMs > SLAVE_TIMEOUT_MS) return 0;
  float frac = 1.0f - ((float)ageMs / (float)SLAVE_TIMEOUT_MS);
  if (frac > 0.75f) return 4;
  if (frac > 0.50f) return 3;
  if (frac > 0.25f) return 2;
  return 1;
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  if (len != sizeof(DevicePacket)) return;

  DevicePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  int idx = findSlaveByMac(mac_addr);

  if (pkt.type == PACKET_ANNOUNCE)
  {
    if (idx == -1)
    {
      SlaveInfo s;
      memset(&s, 0, sizeof(s));
      strncpy(s.deviceName, pkt.deviceName, sizeof(s.deviceName) - 1);
      memcpy(s.mac, mac_addr, 6);
      s.lastSeen = millis();
      s.gateState = pkt.payload;
      s.acked = false;

      discoveredSlaves.push_back(s);
      ensurePeer(mac_addr);

      if (selectedSlaveIndex == -1)
        selectedSlaveIndex = 0;

      Serial.printf("[MASTER] Найден шлагбаум: %s\n", s.deviceName);
    }
    else
    {
      discoveredSlaves[idx].lastSeen = millis();
      discoveredSlaves[idx].gateState = pkt.payload;
    }
  }
  else if (pkt.type == PACKET_ACK)
  {
    if (idx != -1)
    {
      discoveredSlaves[idx].acked = true;
      discoveredSlaves[idx].gateState = pkt.payload;
    }
  }
}

void sendCmdToSelected(GateCmd cmd)
{
  if (selectedSlaveIndex < 0 || selectedSlaveIndex >= (int)discoveredSlaves.size())
    return;

  SlaveInfo &s = discoveredSlaves[selectedSlaveIndex];

  DevicePacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.type = PACKET_CMD;
  strncpy(pkt.deviceName, s.deviceName, sizeof(pkt.deviceName) - 1);
  pkt.payload = cmd;

  ensurePeer(s.mac);
  esp_now_send(s.mac, (uint8_t*)&pkt, sizeof(pkt));

  s.acked = false;
  Serial.printf("[MASTER] -> %s: Команда=%d\n", s.deviceName, cmd);
}

void purgeStaleSlaves()
{
  unsigned long now = millis();

  for (int i = (int)discoveredSlaves.size() - 1; i >= 0; i--)
  {
    if (now - discoveredSlaves[i].lastSeen > SLAVE_TIMEOUT_MS)
    {
      discoveredSlaves.erase(discoveredSlaves.begin() + i);

      if (selectedSlaveIndex == i)
      {
        selectedSlaveIndex = discoveredSlaves.empty() ? -1 : 0;
      }
      else if (selectedSlaveIndex > i)
      {
        selectedSlaveIndex--;
      }
    }
  }
}

void drawScreen();

void handleButton()
{
  bool pressed = (digitalRead(BTN_PIN) == LOW);
  unsigned long now = millis();

  if (pressed && !btnWasPressed)
  {
    btnWasPressed = true;
    btnPressStart = now;
  }
  else if (!pressed && btnWasPressed)
  {
    btnWasPressed = false;
    unsigned long heldFor = now - btnPressStart;

    if (heldFor >= LONG_PRESS_MS)
    {
      if (!discoveredSlaves.empty())
      {
        selectedSlaveIndex = (selectedSlaveIndex + 1) % (int)discoveredSlaves.size();
        Serial.printf("[MASTER] Выбран: %s\n", discoveredSlaves[selectedSlaveIndex].deviceName);
      }
    }
    else
    {
      sendCmdToSelected(CMD_TOGGLE);
    }
    drawScreen();
  }
}

void drawWifiIcon(int x, int y, int level)
{
  const int barW = 2;
  const int gap = 1;
  const int heights[4] = { 3, 5, 8, 11 };

  uint16_t activeColor = COLOR_GREEN;
  if (level <= 1) activeColor = COLOR_RED;
  else if (level == 2) activeColor = COLOR_YELLOW;

  for (int i = 0; i < 4; i++)
  {
    int bx = x + i * (barW + gap);
    int bh = heights[i];
    int by = y + (heights[3] - bh);

    uint16_t col = (i < level) ? activeColor : COLOR_DIM;
    tft.fillRect(bx, by, barW, bh, col);
  }
}

void drawAnimatedBarrier(int baseX, int baseY, float angle)
{
  tft.fillRect(baseX - 4, baseY + 18, 36, 4, COLOR_ROAD);
  tft.drawFastHLine(baseX + 10, baseY + 20, 10, COLOR_YELLOW);

  tft.fillRect(baseX, baseY + 2, 7, 16, 0x8410);
  tft.drawRect(baseX, baseY + 2, 7, 16, TFT_WHITE);
  tft.fillCircle(baseX + 3, baseY + 4, 3, COLOR_YELLOW);

  float rad = angle * 0.0174532925f;
  float cosA = cos(rad);
  float sinA = sin(rad);

  int pivotX = baseX + 3;
  int pivotY = baseY + 4;
  int armLen = 26;

  int endX = pivotX + (int)(cosA * armLen);
  int endY = pivotY - (int)(sinA * armLen);

  tft.drawLine(pivotX, pivotY, endX, endY, TFT_WHITE);
  tft.drawLine(pivotX, pivotY - 1, endX, endY - 1, COLOR_RED);
  tft.drawLine(pivotX, pivotY + 1, endX, endY + 1, COLOR_RED);

  for (int d = 6; d < armLen; d += 6)
  {
    int segX = pivotX + (int)(cosA * d);
    int segY = pivotY - (int)(sinA * d);
    tft.fillCircle(segX, segY, 1, TFT_WHITE);
  }
}

void drawScreen()
{
  pulseTick++;
  tft.startWrite();
  tft.fillScreen(COLOR_BG);
  tft.setTextSize(1);

  if (discoveredSlaves.empty())
  {
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setCursor(22, 22);
    tft.print("SEARCHING GATES");

    int dotCount = (pulseTick / 3) % 4;
    tft.setCursor(118, 22);
    for (int d = 0; d < dotCount; d++) tft.print(".");

    tft.drawFastHLine(20, 36, 120, COLOR_DIM);
    tft.setCursor(16, 46);
    tft.setTextColor(COLOR_DIM, COLOR_BG);
    tft.print("Waiting for Beacon...");

    int scanX = 20 + ((pulseTick * 4) % 115);
    tft.fillCircle(scanX, 36, 2, COLOR_SELECTED);

    tft.endWrite();
    return;
  }

  SlaveInfo &sel = discoveredSlaves[selectedSlaveIndex];
  bool isOpen = (sel.gateState == GATE_OPEN);
  float targetAngle = isOpen ? 90.0f : 0.0f;

  if (animAngle < targetAngle) {
    animAngle += 12.0f;
    if (animAngle > targetAngle) animAngle = targetAngle;
  } else if (animAngle > targetAngle) {
    animAngle -= 12.0f;
    if (animAngle < targetAngle) animAngle = targetAngle;
  }

  drawAnimatedBarrier(4, 38, animAngle);

  tft.drawFastVLine(42, 16, 48, COLOR_DIM);

  tft.setCursor(4, 3);
  tft.setTextColor(COLOR_SELECTED, COLOR_BG);
  tft.printf("> %s", sel.deviceName);

  uint16_t badgeCol = isOpen ? COLOR_GREEN : COLOR_RED;
  tft.drawRoundRect(92, 1, 66, 13, 3, badgeCol);
  if ((pulseTick / 4) % 2 == 0) {
    tft.fillCircle(97, 7, 2, badgeCol);
  }
  tft.setCursor(104, 4);
  tft.setTextColor(badgeCol, COLOR_BG);
  tft.print(isOpen ? "OPEN" : "CLOSED");

  tft.drawFastHLine(0, 15, 160, COLOR_DIM);

  int rowY = 19;
  for (size_t i = 0; i < discoveredSlaves.size(); i++)
  {
    SlaveInfo &s = discoveredSlaves[i];
    unsigned long ago = millis() - s.lastSeen;
    bool isSelected = ((int)i == selectedSlaveIndex);
    int level = signalLevelFromAge(ago);
    bool sOpen = (s.gateState == GATE_OPEN);

    tft.setCursor(46, rowY);
    tft.setTextColor(isSelected ? COLOR_SELECTED : COLOR_TEXT, COLOR_BG);
    tft.printf("%s%-6.6s", isSelected ? ">" : " ", s.deviceName);

    tft.setCursor(96, rowY);
    tft.setTextColor(sOpen ? COLOR_GREEN : COLOR_RED, COLOR_BG);
    tft.print(sOpen ? "[OPN]" : "[CLS]");

    if (s.acked) {
      tft.setCursor(128, rowY);
      tft.setTextColor(COLOR_YELLOW, COLOR_BG);
      tft.print("OK");
    }

    drawWifiIcon(146, rowY, level);

    rowY += 12;
    if (rowY >= 63) break;
  }

  tft.drawFastHLine(0, 66, 160, COLOR_DIM);
  tft.setCursor(4, 70);
  tft.setTextColor(COLOR_DIM, COLOR_BG);
  tft.print("Tap:TOGGLE | Hold:NEXT");

  tft.endWrite();
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(BTN_PIN, INPUT_PULLUP);

  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);

  tft.init();
  tft.setBrightness(255);
  tft.setRotation(1);
  tft.fillScreen(COLOR_BG);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextSize(1);
  tft.setCursor(15, 30);
  tft.println("Starting Gate Remote...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[MASTER] Ошибка инициализации ESP-NOW");
    tft.fillScreen(COLOR_BG);
    tft.setCursor(10, 30);
    tft.setTextColor(COLOR_RED, COLOR_BG);
    tft.println("ESP-NOW init FAIL");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("[MASTER] Пульт запущен, слушаю эфир...");
  delay(300);
  drawScreen();
}

void loop()
{
  purgeStaleSlaves();
  handleButton();

  if (millis() - lastDraw > 40)
  {
    lastDraw = millis();
    drawScreen();
  }
}
