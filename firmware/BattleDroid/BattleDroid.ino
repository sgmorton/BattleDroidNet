#include "FS.h"
#include "SD.h"
#include "SPI.h"
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
#define SD_CS 22

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
String lastCommand = "IDLE";
String sdFile = "NONE";
unsigned long lastMasterHeard = 0;
unsigned long droidLastSeen[11]; // Track IDs 1-8

// --- MASTER UI: TACTICAL GRID ---
void updateMasterUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1
  display.setCursor(0, 0);
  display.println("HADB BattleDroid");

  // Line 2
  display.setCursor(0, 10);
  display.printf("ROLE: MASTER - UNIT D%d", MY_ID);

  // Line 3
  display.setCursor(0, 20);
  display.println(lastCommand);

  // Status Row (Line 4)
  unsigned long now = millis();
  int startX = 4;
  int y = 40; // Shifted up since FILE line is gone
  int boxSize = 12;
  int spacing = 15;

  for (int i = 1; i <= 8; i++) {
    int x = startX + (i - 1) * spacing;

    bool isOnline = (droidLastSeen[i] > 0 && (now - droidLastSeen[i] < 8000));

    if (isOnline) {
      display.fillRect(x, y, boxSize, boxSize, SSD1306_WHITE);
    } else {
      display.drawRect(x, y, boxSize, boxSize, SSD1306_WHITE);
    }
  }

  display.display();
}

// --- SLAVE UI ---
void updateSlaveUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1
  display.setCursor(0, 0);
  display.println("HADB BattleDroid");

  // Line 2
  display.setCursor(0, 10);
  if (MY_ID == 0)
    display.println("ROLE: SLAVE - SEARCHING");
  else
    display.printf("ROLE: SLAVE - UNIT D%d", MY_ID);

  display.setCursor(0, 20);
  display.println(lastCommand);

  display.setCursor(0, 32);
  display.setTextSize(2);
  if (MY_ID == 0)
    display.println("SEARCH");
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

  // SD INIT
  SPI.begin(18, 19, 23, SD_CS);         // Explicitly set VSPI pins
  if (!SD.begin(SD_CS, SPI, 4000000)) { // Use 4MHz for stability
    Serial.println("SD card Mount Failed");
    sdFile = "ERR: NO SD";
  } else {
    File root = SD.open("/");
    if (!root) {
      sdFile = "ERR: NO ROOT";
    } else if (!root.isDirectory()) {
      sdFile = "ERR: NOT DIR";
    } else {
      File file = root.openNextFile();
      if (file) {
        sdFile = String(file.name());
        if (sdFile.startsWith("/"))
          sdFile.remove(0, 1); // Clean up leading slash
        file.close();
      } else {
        sdFile = "EMPTY";
      }
    }
    root.close();
  }

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
    MY_ID = 1;
    droidLastSeen[1] = millis(); // Master sees itself
    Serial.println("MAESTRO: Operating as Droid 1");
  } else {
    Serial.println("SLAVE: Master found, requesting ID...");
    packet req = {1, 0};
    esp_now_send(broadcastAddr, (uint8_t *)&req, sizeof(req));

    // Wait for ID assignment (up to 3 seconds)
    unsigned long waitID = millis();
    while (MY_ID == 0 && millis() - waitID < 3000) {
      delay(100);
    }

    if (MY_ID == 0) {
      Serial.println("SLAVE: ID request timed out!");
    } else {
      Serial.printf("SLAVE: Assigned ID D%d\n", MY_ID);
    }
  }

  // File management on boot
  if (MY_ID > 0) {
    if (IS_MASTER)
      updateMasterUI();
    else
      updateSlaveUI();
    manageSDFiles();
  }

  // Final UI update to show "Configured..."
  if (IS_MASTER)
    updateMasterUI();
  else
    updateSlaveUI();
}

void manageSDFiles() {
  // Use existing SD mount from setup()

  // 1. Delete files in root
  File root = SD.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        String fileName = "/" + String(file.name());
        if (fileName.startsWith("//"))
          fileName.remove(0, 1);
        file.close(); // Close before removing
        SD.remove(fileName);
      } else {
        file.close();
      }
      file = root.openNextFile();
    }
    root.close();
  }

  // 2. Identify source folder
  String srcFolderPath = "/" + String(MY_ID);
  File srcFolder = SD.open(srcFolderPath);
  if (srcFolder && srcFolder.isDirectory()) {
    File file = srcFolder.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        String srcPath = srcFolderPath + "/" + String(file.name());
        String dstPath = "/" + String(file.name());
        copyFile(srcPath.c_str(), dstPath.c_str());
      }
      file.close();
      file = srcFolder.openNextFile();
    }
    srcFolder.close();
  }

  // Update display with confirmation
  lastCommand = "Configured for Droid " + String(MY_ID);
}

void copyFile(const char *src, const char *dst) {
  File srcFile = SD.open(src);
  File dstFile = SD.open(dst, FILE_WRITE);
  if (srcFile && dstFile) {
    uint8_t buf[512];
    while (srcFile.available()) {
      size_t n = srcFile.read(buf, sizeof(buf));
      dstFile.write(buf, n);
    }
  }
  if (srcFile)
    srcFile.close();
  if (dstFile)
    dstFile.close();
}

void loop() {
  unsigned long now = millis();
  static unsigned long lastUpdate = 0;

  if (now - lastUpdate > 1500) {
    outPkg.msgType = 0;
    outPkg.droidID = MY_ID;
    esp_now_send(broadcastAddr, (uint8_t *)&outPkg, sizeof(outPkg));

    droidLastSeen[MY_ID] = now; // Mark self as online

    if (IS_MASTER)
      updateMasterUI();
    else
      updateSlaveUI();

    lastUpdate = now;
  }
}
