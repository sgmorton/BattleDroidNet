#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// --- COMMAND DEFINITIONS ---
#define CMD_HEARTBEAT 0
#define CMD_ID_REQUEST 1
#define CMD_ID_ASSIGN 2
#define CMD_SERVO_MOVE 3
#define CMD_PLAY_AUDIO 4
#define CMD_TALK 5
#define CMD_RESET 6

// --- MODE DEFINITIONS ---
#define MODE_AUTO 0
#define MODE_MANUAL 1
#define MODE_TEST 2

// --- SERVO DEFINITIONS ---
#define HEAD_TURN_CHUX 0
#define HEAD_TILT_CHUX 1
// Add more if needed

#define TARGET_ALL 0xFF

#define OLED_SDA 4
#define OLED_SCL 5
#define SD_CS 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// --- DATA STRUCTURE ---
typedef struct {
  uint8_t msgType;     // CMD_...
  uint8_t targetDroid; // Droid ID or TARGET_ALL
  int param1;          // Position, duration, or file index
  int param2;          // duration or second position
  int param3;          // time_to_move_in_ms
  char cmdStr[16];     // For audio file name or servo name
} packet;

// --- FUNCTION PROTOTYPES ---
void updateMasterUI();
void updateSlaveUI();
void executeCommand(packet pkg);

packet outPkg, inPkg;
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- SYSTEM STATE ---
bool IS_MASTER = false;
int MY_ID = 0;
String lastCommand = "IDLE";
unsigned long lastMasterHeard = 0;
unsigned long droidLastSeen[11]; // Track IDs 1-8

// --- SERVO CONFIG ---
typedef struct {
  int pin;
  int minPulse;
  int maxPulse;
  int currentPos;    // 0-100%
  int targetPos;     // 0-100%
  int startPos;      // 0-100%
  unsigned long moveStartTime;
  int moveDuration;  // ms
  Servo servo;
} ServoConfig;

ServoConfig servos[2] = {
  {13, 500, 2500, 50, 50, 50, 0, 0}, // Head Turn
  {14, 500, 2500, 50, 50, 50, 0, 0}  // Head Tilt/Talk
};

// --- TALK ANIMATION ---
bool isTalking = false;
unsigned long talkEndTime = 0;
unsigned long nextTalkMove = 0;

bool audioStarted = false; // Kept as placeholder if needed or just remove

// --- MASTER SEQUENCER ---
int currentMode = MODE_AUTO; // Changed from bool masterAutoMode
unsigned long nextSequenceTime = 0;
unsigned long sequenceStartTime = 0;
File seqFile;
unsigned long nextEventTime = 0;
bool sequenceActive = false;
String pendingCommands = "";

// --- TEST MODE STATE ---
int testDroidIdx = 0; // Start at 0 so first move finds D1
int testServoIdx = 0;
int testPointIdx = 0;
unsigned long lastTestMove = 0;
int testPoints[] = {25, 50, 75};
const char* testServos[] = {"headturn", "headtilt"};

unsigned long parseTime(String tStr) {
  tStr.trim();
  int firstDot = tStr.indexOf('.');
  int secondDot = tStr.indexOf('.', firstDot + 1);
  if (firstDot == -1 || secondDot == -1) return 0;
  int m = tStr.substring(0, firstDot).toInt();
  int s = tStr.substring(firstDot + 1, secondDot).toInt();
  int c = tStr.substring(secondDot + 1).toInt();
  return (m * 60000) + (s * 1000) + (c * 10);
}

void broadcastCommand(packet pkg) {
  esp_now_send(broadcastAddr, (uint8_t *)&pkg, sizeof(pkg));
  // If we are also a droid (MY_ID > 0), execute it ourselves too
  if (MY_ID > 0) executeCommand(pkg);
}

void startSequence(const char* filename) {
  if (seqFile) seqFile.close();
  seqFile = SD.open(filename);
  if (!seqFile) {
    Serial.printf("SEQ: Failed to open %s\n", filename);
    return;
  }
  sequenceStartTime = millis();
  sequenceActive = true;
  nextEventTime = 0;
  pendingCommands = "";
  Serial.printf("SEQ: Playing %s\n", filename);
}

void updateSequencer() {
  if (!IS_MASTER) return;

  unsigned long now = millis();

  // Handle TEST Mode
  if (currentMode == MODE_TEST) {
    if (now - lastTestMove > 800) {
      // Find the next online droid for the current servo/point
      bool foundNext = false;
      int startSearch = testDroidIdx;
      for (int i = 0; i < 8; i++) {
        testDroidIdx++;
        if (testDroidIdx > 8) testDroidIdx = 1;

        bool isOnline = (droidLastSeen[testDroidIdx] > 0 && (now - droidLastSeen[testDroidIdx] < 8000)) || testDroidIdx == 1;
        if (isOnline) {
          foundNext = true;
          break;
        }
        
        // If we wrapped back to the start droid, we've finished this droid loop
        if (testDroidIdx == startSearch) break;
      }

      // If we finished a full droid cycle (or it's the first one), check if we wrapped
      static int lastSentDroid = 0;
      if (testDroidIdx <= lastSentDroid) {
        // We probably wrapped, but let's be more explicit:
        // Or just check if we've reached a point where we should move to next servo/position
      }
      
      // A better way: check if the droid we just found is "behind" the previous one or if we just started
      // But let's keep it simple: just keep finding the next online droid.
      // The user wants: D1 S0 25, D2 S0 25... then loop next move.
      // So we need to detect when we've done all droids for the current move.

      if (foundNext) {
        packet p = {};
        p.targetDroid = testDroidIdx;
        
        if (testServoIdx < 2) {
          p.msgType = CMD_SERVO_MOVE;
          p.param2 = testPoints[testPointIdx];
          p.param3 = 400;
          strncpy(p.cmdStr, testServos[testServoIdx], 15);
        } else {
          p.msgType = CMD_PLAY_AUDIO;
          strncpy(p.cmdStr, "test", 15);
        }
        broadcastCommand(p);

        int prevDroid = lastSentDroid;
        lastSentDroid = testDroidIdx;

        if (testDroidIdx <= prevDroid) {
          // Wrapped around droids, move to next step
          testServoIdx++;
          if (testServoIdx >= 3) { // 0:turn, 1:tilt, 2:audio
            testServoIdx = 0;
            testPointIdx++;
            if (testPointIdx >= 3) testPointIdx = 0;
          }
        }
      }
      lastTestMove = now;
    }
  }

  // Handle Master Status Reporting to Controller
  static unsigned long lastStatusReport = 0;
  if (now - lastStatusReport > 2000) {
    char sBuf[32];
    int pos = 0;
    pos += snprintf(sBuf + pos, sizeof(sBuf) - pos, "[[STATUS:");
    for (int i = 1; i <= 8; i++) {
      bool isOnline = (droidLastSeen[i] > 0 && (now - droidLastSeen[i] < 8000)) || i == 1;
      sBuf[pos++] = isOnline ? '1' : '0';
    }
    snprintf(sBuf + pos, sizeof(sBuf) - pos, "]]");
    Serial.println(sBuf);
    lastStatusReport = now;
  }

  // Handle Auto Mode Trigger
  if (currentMode == MODE_AUTO && !sequenceActive && now > nextSequenceTime) {
    // Pick a random sequence
    // For now, let's assume we have seq1.txt, seq2.txt ...
    // More robust: list /sequences folder and pick random
    startSequence("/seq1.txt"); 
    nextSequenceTime = now + random(120000, 300000); // 2-5 mins
  }

  if (sequenceActive) {
    unsigned long seqElapsed = now - sequenceStartTime;

    // Read file until we find an event that's due
    while (seqFile.available() || pendingCommands.length() > 0) {
      if (pendingCommands.length() == 0) {
        String line = seqFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        int dashIdx = line.indexOf('-');
        if (dashIdx != -1) {
          String tStr = line.substring(0, dashIdx);
          nextEventTime = parseTime(tStr);
          pendingCommands = line.substring(dashIdx + 1);
          pendingCommands.trim();
        } else {
          // Just a command on the same timestamp?
          pendingCommands = line;
        }
      }

      if (seqElapsed >= nextEventTime) {
        // Execute/Broadcast pendingCommands
        // Parse: PA(*, seq1) or T(1, 3000)
        packet p = {};
        p.targetDroid = TARGET_ALL;

        if (pendingCommands.startsWith("PA")) {
          p.msgType = CMD_PLAY_AUDIO;
          int start = pendingCommands.indexOf('(');
          int comma = pendingCommands.indexOf(',');
          int end = pendingCommands.indexOf(')');
          String idStr = pendingCommands.substring(start + 1, comma);
          idStr.trim();
          if (idStr != "*") p.targetDroid = idStr.toInt();
          String fileStr = pendingCommands.substring(comma + 1, end);
          fileStr.trim();
          strncpy(p.cmdStr, fileStr.c_str(), 15);
          broadcastCommand(p);
        } 
        else if (pendingCommands.startsWith("SM")) {
          p.msgType = CMD_SERVO_MOVE;
          // SM(2, headturn, 75, 750)
          int start = pendingCommands.indexOf('(');
          int end = pendingCommands.indexOf(')');
          String args = pendingCommands.substring(start + 1, end);
          int c1 = args.indexOf(',');
          int c2 = args.indexOf(',', c1 + 1);
          int c3 = args.indexOf(',', c2 + 1);
          
          String dId = args.substring(0, c1); dId.trim();
          if (dId != "*") p.targetDroid = dId.toInt();
          
          String sName = args.substring(c1 + 1, c2); sName.trim();
          strncpy(p.cmdStr, sName.c_str(), 15);
          
          p.param2 = args.substring(c2 + 1, c3).toInt(); // position
          p.param3 = args.substring(c3 + 1).toInt();     // duration
          broadcastCommand(p);
        }
        else if (pendingCommands.startsWith("T(")) {
          p.msgType = CMD_TALK;
          int start = pendingCommands.indexOf('(');
          int comma = pendingCommands.indexOf(',');
          int end = pendingCommands.indexOf(')');
          p.targetDroid = pendingCommands.substring(start + 1, comma).toInt();
          p.param1 = pendingCommands.substring(comma + 1, end).toInt();
          broadcastCommand(p);
        }
        else if (pendingCommands.startsWith("R(")) {
          p.msgType = CMD_RESET;
          int start = pendingCommands.indexOf('(');
          int end = pendingCommands.indexOf(')');
          String idStr = pendingCommands.substring(start + 1, end);
          idStr.trim();
          if (idStr != "*") p.targetDroid = idStr.toInt();
          broadcastCommand(p);
        }

        pendingCommands = ""; // Done with this line
      } else {
        // Not time yet
        break;
      }
    }

    if (!seqFile.available() && pendingCommands.length() == 0) {
      sequenceActive = false;
      seqFile.close();
      Serial.println("SEQ: Finished.");
    }
  }
}

void updateSerial() {
  if (!IS_MASTER) return;
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "AUTO") {
      currentMode = MODE_AUTO;
      Serial.println("MODE: AUTO");
    } else if (line == "MANUAL") {
      currentMode = MODE_MANUAL;
      Serial.println("MODE: MANUAL");
    } else if (line == "TEST") {
      currentMode = MODE_TEST;
      Serial.println("MODE: TEST");
    } else if (line.startsWith("PLAY:")) {
      String file = "/" + line.substring(5);
      startSequence(file.c_str());
    } else if (line.startsWith("CMD:")) {
      // Direct command injection e.g. CMD:PA(*,seq1)
      pendingCommands = line.substring(4);
      nextEventTime = 0; // Trigger immediately in updateSequencer
      sequenceActive = true; 
      // Note: this hackily reuses the sequencer logic
    }
  }
}

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
  const char* modeStrs[] = {"AUTO", "MANUAL", "TEST"};
  display.printf("D%d - %s", MY_ID, modeStrs[currentMode]);

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
    display.println("SLAVE - SEARCHING");
  else {
    // Slaves don't have modes themselves, but they follow the master's heartbeat?
    // Actually, let's just show IDLE or current action
    display.printf("SLAVE - UNIT D%d", MY_ID);
  }

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

void executeCommand(packet pkg) {
  bool isTarget = (pkg.targetDroid == TARGET_ALL || pkg.targetDroid == MY_ID);
  
  // UI/LOGGING: Show if we are the target OR if we are the Master (seeing what we sent/received)
  if (isTarget || IS_MASTER) {
    char buf[32];
    int sIndex = -1;

    switch (pkg.msgType) {
      case CMD_SERVO_MOVE:
        if (strcmp(pkg.cmdStr, "headturn") == 0) sIndex = 0;
        else if (strcmp(pkg.cmdStr, "headtilt") == 0) sIndex = 1;
        else sIndex = pkg.param1;

        if (sIndex >= 0 && sIndex < 2) {
          if (isTarget) {
            servos[sIndex].targetPos = pkg.param2;
            servos[sIndex].startPos = servos[sIndex].currentPos;
            servos[sIndex].moveDuration = pkg.param3;
            servos[sIndex].moveStartTime = millis();
          }
          snprintf(buf, sizeof(buf), "D%d: S%d -> %d%%", pkg.targetDroid, sIndex, pkg.param2);
          lastCommand = String(buf);
          Serial.printf("EXEC: %s (%dms)\n", buf, pkg.param3);
        }
        break;

      case CMD_PLAY_AUDIO:
        snprintf(buf, sizeof(buf), "D%d: Play %s", pkg.targetDroid == TARGET_ALL ? 0 : pkg.targetDroid, pkg.cmdStr);
        lastCommand = String(buf);
        Serial.printf("EXEC: %s\n", buf);
        break;

      case CMD_TALK:
        if (isTarget) {
          isTalking = true;
          talkEndTime = millis() + pkg.param1;
        }
        snprintf(buf, sizeof(buf), "D%d: Talk %dms", pkg.targetDroid == TARGET_ALL ? 0 : pkg.targetDroid, pkg.param1);
        lastCommand = String(buf);
        Serial.printf("EXEC: %s\n", buf);
        break;

      case CMD_RESET:
        if (isTarget) {
          for (int i = 0; i < 2; i++) {
            servos[i].targetPos = 50;
            servos[i].startPos = servos[i].currentPos;
            servos[i].moveDuration = 1000;
            servos[i].moveStartTime = millis();
          }
        }
        lastCommand = "D0: RESET ALL";
        Serial.println("EXEC: D0: RESET ALL");
        break;
    }
  }
}

// --- ESP-NOW CALLBACK ---
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&inPkg, data, sizeof(inPkg));

  if (inPkg.msgType == CMD_ID_REQUEST) {
    if (IS_MASTER) {
      int assignedSlot = -1;
      unsigned long now = millis();
      for (int i = 2; i <= 8; i++) {
        if (droidLastSeen[i] == 0 || (now - droidLastSeen[i] > 10000)) {
          assignedSlot = i;
          break;
        }
      }
      if (assignedSlot != -1) {
        Serial.printf("MASTER: Assigning ID %d to new droid\n", assignedSlot);
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, info->src_addr, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (!esp_now_is_peer_exist(info->src_addr))
          esp_now_add_peer(&peerInfo);

        packet assign = {CMD_ID_ASSIGN, (uint8_t)assignedSlot};
        esp_now_send(info->src_addr, (uint8_t *)&assign, sizeof(assign));
      } else {
        Serial.println("MASTER: No free ID slots!");
      }
    }
  }

  if (!IS_MASTER && inPkg.msgType == CMD_ID_ASSIGN && MY_ID == 0) {
    MY_ID = inPkg.targetDroid;
    Serial.printf("SLAVE: ID assigned: %d\n", MY_ID);
  }

  if (inPkg.msgType == CMD_HEARTBEAT) {
    if (inPkg.targetDroid == 1) { // Only the Master's heartbeat counts
      lastMasterHeard = millis();
    }
    
    if (IS_MASTER && inPkg.targetDroid > 0 && inPkg.targetDroid <= 8) {
      droidLastSeen[inPkg.targetDroid] = millis();
    }
  } else if (!IS_MASTER) {
    executeCommand(inPkg);
  }
}

// --- AUDIO CALLBACK REMOVED ---

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n--- BATTLEDROID STARTUP ---");
  Serial.printf("Free Heap: %d\n", ESP.getFreeHeap());
  Serial.printf("PSRAM Size: %d\n", ESP.getPsramSize());
  Serial.printf("PSRAM Free: %d\n", ESP.getFreePsram());
  Serial.printf("PSRAM Found: %s\n", psramFound() ? "YES" : "NO");

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

  /* SD INIT BYPASSED BY USER REQUEST 
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(16000000); 

  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD card Mount Failed");
  } else {
    Serial.println("SD Card Mounted");
  }
  */

  /* Removed redundant audio init from setup */

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
    packet req = {CMD_ID_REQUEST, 0};
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

  // Servo Setup
  for (int i = 0; i < 2; i++) {
    servos[i].servo.attach(servos[i].pin, servos[i].minPulse, servos[i].maxPulse);
    servos[i].servo.write(map(servos[i].currentPos, 0, 100, 0, 180));
  }

  // File management on boot
  if (MY_ID > 0) {
    if (IS_MASTER)
      updateMasterUI();
    else
      updateSlaveUI();

    if (IS_MASTER)
      updateMasterUI();
    else
      updateSlaveUI();

    // Audio Init Removed
  }

  // Final UI update
  if (IS_MASTER)
    updateMasterUI();
  else
    updateSlaveUI();
}


void loop() {
  unsigned long now = millis();
  static unsigned long lastUpdate = 0;

  if (now - lastUpdate > 1500) {
    outPkg.msgType = CMD_HEARTBEAT;
    outPkg.targetDroid = MY_ID;
    esp_now_send(broadcastAddr, (uint8_t *)&outPkg, sizeof(outPkg));

    droidLastSeen[MY_ID] = now; // Mark self as online

    if (IS_MASTER)
      updateMasterUI();
    else
      updateSlaveUI();

    lastUpdate = now;
  }

  updateSequencer();
  updateSerial();

  // --- SERVO SMOOTHING ---
  for (int i = 0; i < 2; i++) {
    if (servos[i].currentPos != servos[i].targetPos) {
      if (servos[i].moveDuration <= 0) {
        servos[i].currentPos = servos[i].targetPos;
      } else {
        unsigned long elapsed = now - servos[i].moveStartTime;
        if (elapsed >= (unsigned long)servos[i].moveDuration) {
          servos[i].currentPos = servos[i].targetPos;
        } else {
          float t = (float)elapsed / servos[i].moveDuration;
          servos[i].currentPos = servos[i].startPos + (servos[i].targetPos - servos[i].startPos) * t;
        }
      }
      servos[i].servo.write(map(servos[i].currentPos, 0, 100, 0, 180));
    }
  }

  // --- TALK ANIMATION ---
  if (isTalking) {
    if (now > talkEndTime) {
      isTalking = false;
      servos[1].targetPos = 50; // Return head tilt to center
      servos[1].moveDuration = 200;
      servos[1].moveStartTime = now;
      servos[1].startPos = servos[1].currentPos;
    } else if (now > nextTalkMove) {
      // Random quick head tilt movements
      servos[1].targetPos = random(40, 70);
      servos[1].moveDuration = random(50, 150);
      servos[1].moveStartTime = now;
      servos[1].startPos = servos[1].currentPos;
      nextTalkMove = now + servos[1].moveDuration + random(20, 50);
    }
  }
}
