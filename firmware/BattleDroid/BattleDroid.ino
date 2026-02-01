#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>


// --- PIN DEFINITIONS ---
#define OLED_SDA 4
#define OLED_SCL 5
#define PERIPH_PWR 15
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- DATA STRUCTURE ---
typedef struct {
  int msgType; // 0: Heartbeat, 1: ID Request, 2: ID Assign
  int droidID;
} packet;

packet outPkg, inPkg;
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- SYSTEM STATE ---
bool IS_MASTER = false;
int MY_ID = 0;
unsigned long lastMasterHeard = 0;
unsigned long droidLastSeen[11]; // Track IDs 1-8

// --- MASTER UI: TACTICAL GRID ---
void updateMasterUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Header
  display.println("MAS - HADB BattleDroid");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  unsigned long now = millis();
  for (int i = 1; i <= 8; i++) {
    int col = (i - 1) % 4;
    int row = (i - 1) / 4;
    int x = col * 32;
    int y = 15 + (row * 24);

    display.drawRect(x + 2, y, 28, 20, SSD1306_WHITE);

    bool isOnline = (droidLastSeen[i] > 0 && (now - droidLastSeen[i] < 8000));
    if (isOnline) {
      display.fillRect(x + 2, y, 28, 20, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(x + 6, y + 6);
      display.printf("D%d", i);
    } else {
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(x + 12, y + 6);
      display.print("-");
    }
  }
  display.display();
}

// --- SLAVE UI ---
void updateSlaveUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Header
  display.println("SLV - HADB BattleDroid");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  display.setCursor(0, 25);
  display.setTextSize(2);
  if (MY_ID == 0)
    display.println("SEARCHING");
  else
    display.printf("UNIT: D%d", MY_ID);
  display.display();
}

// --- ESP-NOW CALLBACK ---
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&inPkg, data, sizeof(inPkg));

  if (IS_MASTER && inPkg.msgType == 1) {
    int assignedSlot = -1;
    unsigned long now = millis();
    for (int i = 1; i <= 8; i++) {
      if (droidLastSeen[i] == 0 || (now - droidLastSeen[i] > 10000)) {
        assignedSlot = i;
        break;
      }
    }
    if (assignedSlot != -1) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, info->src_addr, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      // FIXED THE TYPO HERE:
      if (!esp_now_is_peer_exist(info->src_addr))
        esp_now_add_peer(&peerInfo);

      packet assign = {2, assignedSlot};
      esp_now_send(info->src_addr, (uint8_t *)&assign, sizeof(assign));
    }
  }

  if (!IS_MASTER && inPkg.msgType == 2 && MY_ID == 0) {
    MY_ID = inPkg.droidID;
  }

  if (inPkg.msgType == 0) {
    lastMasterHeard = millis();
    if (IS_MASTER && inPkg.droidID > 0 && inPkg.droidID <= 8) {
      droidLastSeen[inPkg.droidID] = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PERIPH_PWR, OUTPUT);
  digitalWrite(PERIPH_PWR, HIGH);
  delay(1000);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }
  display.clearDisplay();
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK)
    Serial.println("ESP-NOW Fail");
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t bcast = {};
  memcpy(bcast.peer_addr, broadcastAddr, 6);
  esp_now_add_peer(&bcast);

  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (lastMasterHeard > 0)
      break;
    delay(100);
  }

  if (lastMasterHeard == 0) {
    IS_MASTER = true;
    MY_ID = 99;
  } else {
    packet req = {1, 0};
    esp_now_send(broadcastAddr, (uint8_t *)&req, sizeof(req));
  }
}

void loop() {
  unsigned long now = millis();
  static unsigned long lastUpdate = 0;

  if (now - lastUpdate > 1500) {
    outPkg.msgType = 0;
    outPkg.droidID = MY_ID;
    esp_now_send(broadcastAddr, (uint8_t *)&outPkg, sizeof(outPkg));

    if (IS_MASTER)
      updateMasterUI();
    else
      updateSlaveUI();

    lastUpdate = now;
  }
}
