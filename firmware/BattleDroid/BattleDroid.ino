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
#include "driver/i2s.h"

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
#define TORSO_TURN_CHUX 2

#define VOL_UP_PIN 39
#define VOL_DN_PIN 36

#define TARGET_ALL 0xFF

#define OLED_SDA 4
#define OLED_SCL 5
#define SD_CS 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define BTN_PROV 37 

// Makerfabs I2S Pins
#define I2S_BCLK 26
#define I2S_LRC  25
#define I2S_DOUT 27

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- WAV HEADER STRUCTURE ---
struct wav_header_t {
  char chunkID[4];
  uint32_t chunkSize;
  char format[4];
  char subchunk1ID[4];
  uint32_t subchunk1Size;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  char subchunk2ID[4];
  uint32_t subchunk2Size;
};

// --- DATA STRUCTURE ---
typedef struct {
  uint8_t msgType;
  uint8_t targetDroid;
  int param1;
  int param2;
  int param3;
  char cmdStr[16];
} packet;

// --- FUNCTION PROTOTYPES ---
void updateMasterUI();
void updateSlaveUI();
void executeCommand(packet pkg);
void statusDisplay(String msg, int size = 1);
void loadSettings();
void saveSettings();

void clearSDRoot();
void copyDirToRoot(int id);
void playDroidAudio(int id, int times);
void startProvisioning();
void playWavFile(String path);
void updateSequencer();
void updateSerial();


packet outPkg, inPkg;
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t myMac[6];

// --- SYSTEM STATE ---
bool IS_MASTER = false;
bool isMasterController = false;
int MY_ID = 0;
String lastCommand = "IDLE";
unsigned long lastMasterHeard = 0;
unsigned long droidLastSeen[11];
bool sdAvailable = false;
int globalVolume = 5; // Default 1-10

// --- SERVO CONFIG ---
typedef struct {
  int pin;
  int minPulse;
  int maxPulse;
  int currentPos;
  int targetPos;
  int startPos;
  unsigned long moveStartTime;
  int moveDuration;
  Servo servo;
} ServoConfig;

ServoConfig servos[3] = {
  {13, 500, 2500, 50, 50, 50, 0, 0}, // Head Turn
  {14, 500, 2500, 50, 50, 50, 0, 0}, // Head Tilt
  {12, 500, 2500, 50, 50, 50, 0, 0}  // Torso Turn
};

bool isTalking = false;
unsigned long talkEndTime = 0;
unsigned long nextTalkMove = 0;

int currentMode = MODE_MANUAL; // Start in MANUAL mode for readiness
unsigned long nextSequenceTime = 0;
unsigned long sequenceStartTime = 0;
File seqFile;
unsigned long nextEventTime = 0;
bool sequenceActive = false;
String pendingCommands = "";

// --- TEST MODE STATE ---
int testDroidIdx = 0;
int testServoIdx = 0;
int testPointIdx = 0;
unsigned long lastTestMove = 0;
int testPoints[] = {25, 50, 75};
const char* testServos[] = {"headturn", "headtilt", "torsoturn"};
File audioFile;
bool audioStreaming = false;
uint32_t audioSampleRate = 44100;
int audioChannels = 2;

// --- PSRAM AUDIO BUFFER ---
uint8_t* psramAudioBuffer = nullptr;
int audioBufferSize = 0;
volatile int audioHead = 0;
volatile int audioTail = 0;
TaskHandle_t audioTaskHandle;

int getAudioBytesInQueue() {
  int h = audioHead;
  int t = audioTail;
  if (h >= t) return h - t;
  return audioBufferSize - (t - h);
}

int getAudioSpaceAvailable() {
  return audioBufferSize - getAudioBytesInQueue() - 1;
}
TaskHandle_t servoTaskHandle;

int lastServoWrite[2] = {-1, -1};

unsigned long parseTime(String tStr) {
  tStr.trim();
  int firstDot = tStr.indexOf('.');
  int secondDot = tStr.indexOf('.', firstDot + 1);
  if (firstDot == -1 || secondDot == -1) return 0;
  return (tStr.substring(0, firstDot).toInt() * 60000) + (tStr.substring(firstDot + 1, secondDot).toInt() * 1000) + (tStr.substring(secondDot + 1).toInt() * 10);
}

void broadcastCommand(packet pkg) {
  esp_err_t result = esp_now_send(broadcastAddr, (uint8_t *)&pkg, sizeof(pkg));
  if (result != ESP_OK) Serial.println("SEND FAIL: " + String(result));
  else Serial.println("SENT: type=" + String(pkg.msgType) + ", target=" + String(pkg.targetDroid));
  // Execute locally if we are a droid (ID > 0) or the promoted controller
  if (MY_ID > 0 || isMasterController) executeCommand(pkg);
}

volatile uint32_t audioDataRemaining = 0;

void playWavFile(String path) {
  if (!sdAvailable) return;
  if (audioStreaming) { 
    audioStreaming = false; 
    vTaskDelay(50 / portTICK_PERIOD_MS); // Let audioTask pause safely
    if (audioFile) audioFile.close(); 
  }
  
  if (!SD.exists(path)) { Serial.println("WAV: File not found: " + path); return; }
  audioFile = SD.open(path);
  
  char chunkId[5] = {0};
  uint32_t chunkSize = 0;
  
  if (audioFile.read((uint8_t*)chunkId, 4) != 4 || strncmp(chunkId, "RIFF", 4) != 0) { audioFile.close(); return; }
  audioFile.read((uint8_t*)&chunkSize, 4);
  if (audioFile.read((uint8_t*)chunkId, 4) != 4 || strncmp(chunkId, "WAVE", 4) != 0) { audioFile.close(); return; }

  bool foundData = false;
  while (audioFile.available()) {
    if (audioFile.read((uint8_t*)chunkId, 4) != 4) break;
    if (audioFile.read((uint8_t*)&chunkSize, 4) != 4) break;
    
    if (strncmp(chunkId, "fmt ", 4) == 0) {
      uint16_t audioFormat, numChannels;
      uint32_t sampleRate, byteRate;
      audioFile.read((uint8_t*)&audioFormat, 2);
      audioFile.read((uint8_t*)&numChannels, 2);
      audioFile.read((uint8_t*)&sampleRate, 4);
      audioFile.read((uint8_t*)&byteRate, 4);
      audioFile.seek(audioFile.position() + 4); 
      
      audioSampleRate = sampleRate;
      audioChannels = numChannels;
      Serial.printf("WAV: %dHz, %dch\n", audioSampleRate, audioChannels);
      if (chunkSize > 16) audioFile.seek(audioFile.position() + (chunkSize - 16));
    } else if (strncmp(chunkId, "data", 4) == 0) {
      foundData = true;
      audioDataRemaining = chunkSize;
      // If chunkSize is zero or invalid, fallback to reading until EOF
      if (audioDataRemaining < 100) audioDataRemaining = 0xFFFFFFFF; 
      Serial.printf("WAV: Data %u bytes\n", chunkSize);
      break;
    } else {
      audioFile.seek(audioFile.position() + chunkSize);
    }
  }

  if (!foundData) { Serial.println("WAV: No data"); audioFile.close(); return; }
  i2s_set_sample_rates(I2S_NUM_0, audioSampleRate);
  
  audioHead = 0;
  audioTail = 0;

  // --- PRE-BUFFERING ---
  // Load initial data to prevent stuttering
  int prebufferSize = 256 * 1024; // Increased to 256KB for extra stability
  if (audioBufferSize < prebufferSize) prebufferSize = audioBufferSize / 2;

  while (audioDataRemaining > 0 && getAudioBytesInQueue() < prebufferSize) {
    uint8_t tempBuf[2048];
    int toRead = min((uint32_t)sizeof(tempBuf), (uint32_t)audioDataRemaining);
    int bytesRead = audioFile.read(tempBuf, toRead);
    if (bytesRead <= 0) break;

    audioDataRemaining -= bytesRead;
    if (globalVolume < 10) {
      int16_t* samples = (int16_t*)tempBuf;
      int numSamples = bytesRead / 2;
      for (int i = 0; i < numSamples; i++) {
        samples[i] = (int16_t)((int32_t)samples[i] * globalVolume / 10);
      }
    }

    if (audioChannels == 1) enqueueMonoAsStereo(tempBuf, bytesRead);
    else enqueueAudio(tempBuf, bytesRead);
  }

  audioStreaming = true;
}

void enqueueAudio(uint8_t* data, int len) {
  if (getAudioSpaceAvailable() < len) return;
  int firstChunk = min(len, audioBufferSize - audioHead);
  memcpy(psramAudioBuffer + audioHead, data, firstChunk);
  if (firstChunk < len) memcpy(psramAudioBuffer, data + firstChunk, len - firstChunk);
  audioHead = (audioHead + len) % audioBufferSize;
}

void enqueueMonoAsStereo(uint8_t* monoData, int len) {
  if (getAudioSpaceAvailable() < len * 2) return;
  for (int i = 0; i < len; i += 2) {
    psramAudioBuffer[audioHead] = monoData[i]; audioHead = (audioHead + 1) % audioBufferSize;
    psramAudioBuffer[audioHead] = monoData[i+1]; audioHead = (audioHead + 1) % audioBufferSize;
    psramAudioBuffer[audioHead] = monoData[i]; audioHead = (audioHead + 1) % audioBufferSize;
    psramAudioBuffer[audioHead] = monoData[i+1]; audioHead = (audioHead + 1) % audioBufferSize;
  }
}

void audioTask(void *parameter) {
  while (true) {
    if (audioStreaming) {
      bool didWork = false;
      
      // Aggressive Buffer Refill
      while (audioDataRemaining > 0 && getAudioSpaceAvailable() >= 8192) {
        uint8_t tempBuf[4096];
        int toRead = min((uint32_t)sizeof(tempBuf), (uint32_t)audioDataRemaining);
        int bytesRead = audioFile.read(tempBuf, toRead);
        if (bytesRead > 0) {
          audioDataRemaining -= bytesRead;
          
          // Apply Volume Scaling
          if (globalVolume < 10) {
            int16_t* samples = (int16_t*)tempBuf;
            int numSamples = bytesRead / 2;
            for (int i = 0; i < numSamples; i++) {
              samples[i] = (int16_t)((int32_t)samples[i] * globalVolume / 10);
            }
          }
          if (audioChannels == 1) enqueueMonoAsStereo(tempBuf, bytesRead);
          else enqueueAudio(tempBuf, bytesRead);
          didWork = true;
        } else {
          audioDataRemaining = 0; // Force end on read error
          break;
        }
        // Yield occasionally if we're doing a lot of reads
        vTaskDelay(1 / portTICK_PERIOD_MS); 
      }

      int bytesInQueue = getAudioBytesInQueue();
      if (bytesInQueue > 0) {
        size_t bytesWritten = 0;
        int chunk = min(bytesInQueue, 4096);
        int contigChunk = min(chunk, audioBufferSize - audioTail);
        i2s_write(I2S_NUM_0, psramAudioBuffer + audioTail, contigChunk, &bytesWritten, 0);
        if (bytesWritten > 0) {
          audioTail = (audioTail + bytesWritten) % audioBufferSize;
          didWork = true;
        }
      } else if (audioDataRemaining == 0) {
        vTaskDelay(250 / portTICK_PERIOD_MS); 
        audioStreaming = false;
        audioFile.close();
        i2s_zero_dma_buffer(I2S_NUM_0);
        Serial.println("WAV: Finished");
      }
      
      if (!didWork) vTaskDelay(1 / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

void loop() {
  static bool loopStarted = false;
  if (!loopStarted) { Serial.println("LOOP: STARTED"); loopStarted = true; }
  unsigned long now = millis(); static unsigned long lastUpdate = 0;
  if (now - lastUpdate > 200) { 
    if (isMasterController) MY_ID = 0; 

    // Handle Volume Rocker
    static int lastVolUp = HIGH, lastVolDn = HIGH;
    int volUp = digitalRead(VOL_UP_PIN);
    int volDn = digitalRead(VOL_DN_PIN);
    if (volUp == LOW && lastVolUp == HIGH) {
      if (globalVolume < 10) { globalVolume++; Serial.printf("VOL UP: %d\n", globalVolume); saveSettings(); }
    }
    if (volDn == LOW && lastVolDn == HIGH) {
      if (globalVolume > 0) { globalVolume--; Serial.printf("VOL DN: %d\n", globalVolume); saveSettings(); }
    }
    lastVolUp = volUp; lastVolDn = volDn;

    if (IS_MASTER) {
      outPkg.msgType = CMD_HEARTBEAT; outPkg.targetDroid = MY_ID;
      esp_now_send(broadcastAddr, (uint8_t *)&outPkg, sizeof(outPkg));
      droidLastSeen[MY_ID] = now;
      updateMasterUI();
    } else {
      static unsigned long lastHb = 0;
      if (now - lastHb > 1000) { 
        outPkg.msgType = CMD_HEARTBEAT; outPkg.targetDroid = MY_ID;
        esp_now_send(broadcastAddr, (uint8_t *)&outPkg, sizeof(outPkg));
        Serial.println("SENT HB ME=" + String(MY_ID));
        lastHb = now;
      }
      static unsigned long lastUi = 0;
      if (now - lastUi > 500) { updateSlaveUI(); lastUi = now; }
    }
    lastUpdate = now;
  }
  updateSequencer(); 
  updateSerial();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void servoTask(void *parameter) {
  while (true) {
    unsigned long now = millis();
    for (int i = 0; i < 2; i++) {
      if (servos[i].currentPos != servos[i].targetPos) {
        if (servos[i].moveDuration <= 0) servos[i].currentPos = servos[i].targetPos;
        else {
          unsigned long elapsed = now - servos[i].moveStartTime;
          if (elapsed >= (unsigned long)servos[i].moveDuration) servos[i].currentPos = servos[i].targetPos;
          else servos[i].currentPos = servos[i].startPos + (servos[i].targetPos - servos[i].startPos) * ((float)elapsed / servos[i].moveDuration);
        }
        int angle = map(servos[i].currentPos, 0, 100, 0, 180);
        if (angle != lastServoWrite[i]) {
          servos[i].servo.write(angle);
          lastServoWrite[i] = angle;
        }
      }
    }
    
    if (isTalking) {
      if (now > talkEndTime) { isTalking = false; servos[1].targetPos = 50; }
      else if (now > nextTalkMove) {
        servos[1].targetPos = random(40, 70); servos[1].moveDuration = random(50, 150);
        servos[1].moveStartTime = now; servos[1].startPos = servos[1].currentPos;
        nextTalkMove = now + servos[1].moveDuration + random(20, 50);
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS); // 50Hz update rate
  }
}

void startSequence(const char* filename) {
  if (seqFile) seqFile.close();
  seqFile = SD.open(filename);
  if (!seqFile) return;
  sequenceStartTime = millis(); sequenceActive = true; nextEventTime = 0; pendingCommands = "";
}

void updateSequencer() {
  if (!IS_MASTER) return;
  unsigned long now = millis();

  if (currentMode == MODE_TEST) {
    int delayTime = (testServoIdx == 0) ? 5000 : 1500; // Wait 5s after the last step (Audio was index 2)
    if (now - lastTestMove > (unsigned long)delayTime) {
      bool foundNext = false; 
      if (testServoIdx == 0) {
        int startSearch = testDroidIdx;
        for (int i = 0; i < 8; i++) {
          testDroidIdx++; if (testDroidIdx > 8) testDroidIdx = 1;
          // Test if droid is online OR if it's Droid 1 (always test D1)
          if ((droidLastSeen[testDroidIdx] > 0 && (now - droidLastSeen[testDroidIdx] < 8000)) || testDroidIdx == 1) { foundNext = true; break; }
          if (testDroidIdx == startSearch) break;
        }
      } else { foundNext = true; }

      if (foundNext) {
        packet p = {}; p.targetDroid = testDroidIdx;
        char statusMsg[32];
        if (testServoIdx == 0) {
          p.msgType = CMD_SERVO_MOVE; p.param2 = testPoints[testPointIdx]; p.param3 = 400;
          strncpy(p.cmdStr, "headturn", 15);
          snprintf(statusMsg, sizeof(statusMsg), "TEST D%d TURN", testDroidIdx);
        } else if (testServoIdx == 1) {
          p.msgType = CMD_SERVO_MOVE; p.param2 = testPoints[testPointIdx]; p.param3 = 400;
          strncpy(p.cmdStr, "headtilt", 15);
          snprintf(statusMsg, sizeof(statusMsg), "TEST D%d TILT", testDroidIdx);
        } else {
          // Audio step: Show status on OLED but don't send broadcast command (keep quiet)
          snprintf(statusMsg, sizeof(statusMsg), "TEST D%d AUDIO (SILENT)", testDroidIdx);
          p.msgType = 0xFF; // Invalid type to prevent execution
        }
        lastCommand = statusMsg;
        if (p.msgType != 0xFF) broadcastCommand(p);
        
        testServoIdx++; 
        if (testServoIdx >= 3) { 
          testServoIdx = 0; 
          testPointIdx++; if (testPointIdx >= 3) testPointIdx = 0;
        }
      }
      lastTestMove = millis(); 
    }
  }

  static unsigned long lastStatusReport = 0;
  if (now - lastStatusReport > 2000) {
    char sBuf[32]; int pos = 0; pos += snprintf(sBuf + pos, sizeof(sBuf) - pos, "[[STATUS:");
    for (int i = 1; i <= 8; i++) {
      bool isOnline = (droidLastSeen[i] > 0 && (now - droidLastSeen[i] < 8000)) || i == 1;
      sBuf[pos++] = isOnline ? '1' : '0';
    }
    snprintf(sBuf + pos, sizeof(sBuf) - pos, "]]"); Serial.println(sBuf);
    
    // Mode report for Web UI
    const char* modeStrs[] = {"AUTO", "MANUAL", "TEST"};
    Serial.println("[[MODE:" + String(modeStrs[currentMode]) + "]]");

    lastStatusReport = now;
  }

  if (currentMode == MODE_AUTO && !sequenceActive && now > nextSequenceTime) {
    startSequence("/seq1.txt"); nextSequenceTime = now + random(120000, 300000);
  }

  if (sequenceActive) {
    unsigned long seqElapsed = now - sequenceStartTime;
    while (seqFile.available() || pendingCommands.length() > 0) {
      if (pendingCommands.length() == 0) {
        String line = seqFile.readStringUntil('\n'); line.trim();
        if (line.length() == 0) continue;
        int dashIdx = line.indexOf('-');
        if (dashIdx != -1) {
          nextEventTime = parseTime(line.substring(0, dashIdx));
          pendingCommands = line.substring(dashIdx + 1); pendingCommands.trim();
        } else { pendingCommands = line; }
      }
      if (seqElapsed >= nextEventTime) {
        packet p = {}; p.targetDroid = TARGET_ALL;
        if (pendingCommands.startsWith("PA")) {
          p.msgType = CMD_PLAY_AUDIO;
          int s = pendingCommands.indexOf('('); int c = pendingCommands.indexOf(','); int e = pendingCommands.indexOf(')');
          String idStr = pendingCommands.substring(s + 1, c); idStr.trim();
          if (idStr != "*") p.targetDroid = idStr.toInt();
          String cmd = pendingCommands.substring(c + 1, e); cmd.trim();
          memset(p.cmdStr, 0, 16); strncpy(p.cmdStr, cmd.c_str(), 15);
          broadcastCommand(p);
        } else if (pendingCommands.startsWith("SM")) {
          p.msgType = CMD_SERVO_MOVE;
          int s = pendingCommands.indexOf('('); int e = pendingCommands.indexOf(')');
          String args = pendingCommands.substring(s + 1, e);
          int c1 = args.indexOf(','); int c2 = args.indexOf(',', c1 + 1); int c3 = args.indexOf(',', c2 + 1);
          String idStr = args.substring(0, c1); idStr.trim();
          if (idStr != "*") p.targetDroid = idStr.toInt();
          String cmd = args.substring(c1 + 1, c2); cmd.trim();
          memset(p.cmdStr, 0, 16); strncpy(p.cmdStr, cmd.c_str(), 15);
          p.param2 = args.substring(c2 + 1, c3).toInt(); p.param3 = args.substring(c3 + 1).toInt();
          broadcastCommand(p);
        } else if (pendingCommands.startsWith("T(")) {
          p.msgType = CMD_TALK;
          int s = pendingCommands.indexOf('('); int c = pendingCommands.indexOf(','); int e = pendingCommands.indexOf(')');
          p.targetDroid = pendingCommands.substring(s + 1, c).toInt(); p.param1 = pendingCommands.substring(c + 1, e).toInt();
          broadcastCommand(p);
        }
        pendingCommands = "";
      } else { break; }
    }
    if (!seqFile.available() && pendingCommands.length() == 0) { sequenceActive = false; seqFile.close(); }
  }
}

void updateSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n'); line.trim();
    Serial.println("RECV SERIAL: " + line);
    if (line == "CONTROLLER") {
      isMasterController = true;
      IS_MASTER = true;
      MY_ID = 0; 
      for(int i=0; i<11; i++) droidLastSeen[i] = 0;
      statusDisplay("CTRL MODE", 1);
      Serial.println("PROMOTED TO D0 CONTROLLER");
      return;
    }
    
    if (!IS_MASTER) return; // Only Master (or promoted controller) handles fleet commands

    if (line == "AUTO") currentMode = MODE_AUTO;
    else if (line == "MANUAL") currentMode = MODE_MANUAL;
    else if (line == "TEST") currentMode = MODE_TEST;
    else if (line.startsWith("SETID ")) {
      MY_ID = line.substring(6).toInt();
      saveSettings();
      Serial.printf("ID SET TO D%d\n", MY_ID);
    }
    else if (line == "RESET") {
      packet p = {}; p.msgType = CMD_RESET; p.targetDroid = TARGET_ALL;
      broadcastCommand(p);
      Serial.println("SENT FLEET RESET");
    }
    else if (line.startsWith("PLAY:")) startSequence(("/" + line.substring(5)).c_str());
    else if (line.startsWith("CMD:")) {
      String cmd = line.substring(4); cmd.trim();
      packet p = {}; p.targetDroid = TARGET_ALL;
      
      int openP = cmd.indexOf('('); int comma = cmd.indexOf(','); int closeP = cmd.indexOf(')');
      if (openP != -1 && comma != -1) {
        String targetStr = cmd.substring(openP + 1, comma); targetStr.trim();
        if (targetStr == "*") p.targetDroid = TARGET_ALL;
        else p.targetDroid = (uint8_t)targetStr.toInt();

        String typeStr = cmd.substring(0, openP);
        if (typeStr == "PA") {
          p.msgType = CMD_PLAY_AUDIO;
          String file = cmd.substring(comma + 1, closeP); file.trim();
          memset(p.cmdStr, 0, 16); strncpy(p.cmdStr, file.c_str(), 15);
          Serial.printf("SERIAL PA: Target=%d File=%s\n", p.targetDroid, p.cmdStr);
          broadcastCommand(p);
        } else if (typeStr == "SM") {
          p.msgType = CMD_SERVO_MOVE;
          int c2 = cmd.indexOf(',', comma + 1); int c3 = cmd.indexOf(',', c2 + 1);
          String servo = cmd.substring(comma + 1, c2); servo.trim();
          memset(p.cmdStr, 0, 16); strncpy(p.cmdStr, servo.c_str(), 15);
          p.param2 = cmd.substring(c2 + 1, c3).toInt(); p.param3 = cmd.substring(c3 + 1, closeP).toInt();
          broadcastCommand(p);
        } else if (typeStr == "T" || typeStr == "TK") {
          p.msgType = CMD_TALK;
          p.param1 = cmd.substring(comma + 1, closeP).toInt();
          broadcastCommand(p);
        }
      }
    }
  }
}

void updateMasterUI() {
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); display.println("HADB BattleDroid");
  display.setCursor(0, 10); 
  const char* modeStrs[] = {"AUTO", "MANUAL", "TEST"}; 
  if (isMasterController) display.printf("CTRL - %s", modeStrs[currentMode]);
  else display.printf("D%d - %s", MY_ID, modeStrs[currentMode]);
  
  display.setCursor(0, 20); display.println(lastCommand);
  unsigned long now = millis(); int startX = 4; int y = 40;
  for (int i = 1; i <= 8; i++) {
    int x = startX + (i - 1) * 15;
    bool isOnline = (droidLastSeen[i] > 0 && (now - droidLastSeen[i] < 8000));
    if (isOnline) display.fillRect(x, y, 12, 12, SSD1306_WHITE);
    else display.drawRect(x, y, 12, 12, SSD1306_WHITE);
  }
  
  // Volume Bar (Right Side)
  int volH = map(globalVolume, 0, 10, 0, 62);
  display.drawFastVLine(127, 63 - volH, volH, SSD1306_WHITE);
  
  display.display();
}

void updateSlaveUI() {
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); display.println("HADB BattleDroid");
  display.setCursor(0, 10); if (MY_ID == 0) display.println("SLAVE - SEARCHING"); else display.printf("SLAVE - UNIT D%d", MY_ID);
  display.setCursor(0, 20); display.println(lastCommand);
  display.setCursor(0, 32); display.setTextSize(2); 
  if (MY_ID == 0) display.println("SEARCH"); 
  else display.printf("UNIT: D%d", MY_ID);

  // Volume Bar (Right Side)
  int volH = map(globalVolume, 0, 10, 0, 62);
  display.drawFastVLine(127, 63 - volH, volH, SSD1306_WHITE);

  display.display();
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  // Prevent executing our own broadcasts (loopback)
  if (memcmp(info->src_addr, myMac, 6) == 0) return;

  memcpy(&inPkg, data, sizeof(inPkg));
  if (IS_MASTER && inPkg.msgType != CMD_HEARTBEAT) Serial.println("RECV PKT: T=" + String(inPkg.targetDroid) + " ME=" + String(MY_ID) + " TYPE=" + String(inPkg.msgType));

  if (inPkg.msgType == CMD_HEARTBEAT) {
    if (inPkg.targetDroid == 0 || inPkg.targetDroid == 1) lastMasterHeard = millis();
    if (IS_MASTER && inPkg.targetDroid > 0 && inPkg.targetDroid <= 8) {
      if (droidLastSeen[inPkg.targetDroid] == 0) Serial.println("FLEET: D" + String(inPkg.targetDroid) + " ONLINE");
      droidLastSeen[inPkg.targetDroid] = millis();
    }
  } else if (inPkg.msgType == CMD_RESET) {
    Serial.println("FLEET RESET RECEIVED");
    if (!isMasterController) {
      MY_ID = 0; lastMasterHeard = 0; IS_MASTER = false; lastCommand = "RESETTING...";
    }
  } else if (inPkg.msgType >= CMD_SERVO_MOVE) {
    executeCommand(inPkg);
  } else if (inPkg.msgType == CMD_ID_REQUEST && IS_MASTER) {
    int slot = -1; unsigned long now = millis();
    for (int i = 2; i <= 8; i++) { if (droidLastSeen[i] == 0 || (now - droidLastSeen[i] > 10000)) { slot = i; break; } }
    if (slot != -1) {
      esp_now_peer_info_t p = {}; memcpy(p.peer_addr, info->src_addr, 6);
      if (!esp_now_is_peer_exist(info->src_addr)) esp_now_add_peer(&p);
      packet assign = {CMD_ID_ASSIGN, (uint8_t)slot}; esp_now_send(info->src_addr, (uint8_t *)&assign, sizeof(assign));
    }
  } else if (inPkg.msgType == CMD_ID_ASSIGN && MY_ID == 0 && !isMasterController) {
    MY_ID = inPkg.targetDroid;
  }
}

void setup() {
  Serial.begin(115200);
  for(int i=0; i<3; i++) { 
    Serial.println("###################################");
    Serial.println("###   BATTLE DROID V2.1 ACTIVE  ###");
    Serial.println("###################################");
    delay(100); 
  }
  Serial.println("--- BattleDroid System Online ---");

  Wire.begin(OLED_SDA, OLED_SCL); Wire.setClock(400000); display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.display();

  // --- I2S LOW-LEVEL INIT ---
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = 64, .dma_buf_len = 512
  };
  i2s_pin_config_t pin_config = { .bck_io_num = I2S_BCLK, .ws_io_num = I2S_LRC, .data_out_num = I2S_DOUT, .data_in_num = -1 };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL); i2s_set_pin(I2S_NUM_0, &pin_config);

  // --- ALLOCATE PSRAM AUDIO BUFFER ---
  if (psramFound()) {
    audioBufferSize = 1024 * 1024 * 2; // 2 MB Buffer for WROVER
    psramAudioBuffer = (uint8_t*)ps_malloc(audioBufferSize);
    if (!psramAudioBuffer) Serial.println("PSRAM MALLOC FAIL!");
    else Serial.println("PSRAM 2MB Audio Buffer OK");
  } 
  if (!psramAudioBuffer) {
    audioBufferSize = 32 * 1024; // 32 KB Fallback
    psramAudioBuffer = (uint8_t*)malloc(audioBufferSize);
    Serial.println("Using 32KB DRAM Audio Buffer");
  }

  WiFi.mode(WIFI_STA); WiFi.disconnect();
  WiFi.macAddress(myMac);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.setSleep(false);
  
  esp_now_init(); esp_now_register_recv_cb(onDataRecv);

  pinMode(VOL_UP_PIN, INPUT);
  pinMode(VOL_DN_PIN, INPUT);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!SD.begin(SD_CS, SPI, 8000000)) { Serial.println("SD FAIL"); delay(1000); }
  else {
    sdAvailable = true;
    loadSettings();
  }

  esp_now_peer_info_t bcast = {}; 
  memcpy(bcast.peer_addr, broadcastAddr, 6); 
  bcast.channel = 1; 
  bcast.encrypt = false;
  bcast.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&bcast);

  unsigned long start = millis();
  while (millis() - start < 5000) { if (lastMasterHeard > 0) break; delay(100); }
  if (lastMasterHeard == 0) { IS_MASTER = true; MY_ID = 1; droidLastSeen[1] = millis(); }
  else {
    packet req = {CMD_ID_REQUEST, 0}; esp_now_send(broadcastAddr, (uint8_t *)&req, sizeof(req));
    unsigned long wait = millis(); while (MY_ID == 0 && millis() - wait < 3000) delay(100);
  }

  runProvisioningEngine();

  // Attach servos after SD/SPI init to avoid pin conflicts
  for (int i = 0; i < 3; i++) {
    servos[i].servo.attach(servos[i].pin, servos[i].minPulse, servos[i].maxPulse);
    servos[i].servo.write(map(servos[i].currentPos, 0, 100, 0, 180));
  }

  // Start background tasks last
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, NULL, 2, &audioTaskHandle, 1);
  xTaskCreatePinnedToCore(servoTask, "ServoTask", 4096, NULL, 0, &servoTaskHandle, 0);
  Serial.println("SETUP COMPLETE");
}

void loadSettings() {
  if (!sdAvailable) return;
  if (!SD.exists("/settings")) SD.mkdir("/settings");
  if (!SD.exists("/settings/settings.ini")) {
    saveSettings(); // Create default
    return;
  }
  
  File f = SD.open("/settings/settings.ini");
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("HT_MIN=")) servos[0].minPulse = line.substring(7).toInt();
    else if (line.startsWith("HT_MAX=")) servos[0].maxPulse = line.substring(7).toInt();
    else if (line.startsWith("HLT_MIN=")) servos[1].minPulse = line.substring(8).toInt();
    else if (line.startsWith("HLT_MAX=")) servos[1].maxPulse = line.substring(8).toInt();
    else if (line.startsWith("TR_MIN=")) servos[2].minPulse = line.substring(7).toInt();
    else if (line.startsWith("TR_MAX=")) servos[2].maxPulse = line.substring(7).toInt();
    else if (line.startsWith("VOL=")) globalVolume = line.substring(4).toInt();
  }
  f.close();
  Serial.printf("Settings Loaded: Vol=%d\n", globalVolume);
}

void saveSettings() {
  if (!sdAvailable) return;
  File f = SD.open("/settings/settings.ini", FILE_WRITE);
  if (!f) return;
  f.printf("HT_MIN=%d\n", servos[0].minPulse);
  f.printf("HT_MAX=%d\n", servos[0].maxPulse);
  f.printf("HLT_MIN=%d\n", servos[1].minPulse);
  f.printf("HLT_MAX=%d\n", servos[1].maxPulse);
  f.printf("TR_MIN=%d\n", servos[2].minPulse);
  f.printf("TR_MAX=%d\n", servos[2].maxPulse);
  f.printf("VOL=%d\n", globalVolume);
  f.close();
  Serial.println("Settings Saved");
}

void clearSDRoot() { 
  if (!sdAvailable) return;
  File r = SD.open("/"); if (!r) return;
  while(1) { File f = r.openNextFile(); if (!f) break; if (!f.isDirectory()) SD.remove("/" + String(f.name())); f.close(); } 
  r.close();
}
void copyDirToRoot(int id) {
  if (!sdAvailable) return;
  File d = SD.open("/" + String(id)); 
  if (!d) return;
  while(1) {
    File f = d.openNextFile(); if (!f) break;
    if (f.isDirectory()) { f.close(); continue; }
    File df = SD.open("/" + String(f.name()), FILE_WRITE);
    if (df) { uint8_t b[512]; while(f.available()) df.write(b, f.read(b, 512)); df.close(); }
    f.close();
  }
  d.close();
}

void playDroidAudio(int id, int times) {
  String path = "/BD" + String(id) + ".wav";
  if (!SD.exists(path)) return;
  for (int i = 0; i < times; i++) playWavFile(path);
}

void runProvisioningEngine() { 
  Serial.println("PROVISION CHECK: ID=" + String(MY_ID) + " SD=" + String(sdAvailable));
  if (MY_ID == 0 || !sdAvailable) return; 
  statusDisplay("PROVISIONING", 1); 
  statusDisplay("CLEANING SD", 1);
  clearSDRoot(); 
  statusDisplay("COPYING D" + String(MY_ID), 1);
  copyDirToRoot(MY_ID); 
  statusDisplay("SYSTEM READY - SYNC OK", 1); 
}
void statusDisplay(String msg, int size) {
  Serial.println("STATUS: " + msg);
  lastCommand = msg;
  // Removed forced updateMasterUI/updateSlaveUI to prevent I2S starvation during sequences
}

void executeCommand(packet pkg) {
  // STRICT FILTERING: Only execute if target is ALL or OUR ID.
  if (pkg.targetDroid != TARGET_ALL && pkg.targetDroid != MY_ID) return;
  
  char logBuf[64];
  // User-Friendly Status for OLED
  if (pkg.msgType == CMD_SERVO_MOVE) {
    if (strcmp(pkg.cmdStr, "headturn") == 0) snprintf(logBuf, sizeof(logBuf), "Turning Head: %d%%", pkg.param2);
    else if (strcmp(pkg.cmdStr, "headtilt") == 0) snprintf(logBuf, sizeof(logBuf), "Tilting Head: %d%%", pkg.param2);
    else if (strcmp(pkg.cmdStr, "torsoturn") == 0) snprintf(logBuf, sizeof(logBuf), "Turning Torso: %d%%", pkg.param2);
    else snprintf(logBuf, sizeof(logBuf), "Moving %s: %d%%", pkg.cmdStr, pkg.param2);
  } 
  else if (pkg.msgType == CMD_PLAY_AUDIO) {
    if (pkg.cmdStr[0] == '\0') snprintf(logBuf, sizeof(logBuf), "Playing Default BD%d", MY_ID);
    else snprintf(logBuf, sizeof(logBuf), "Playing: %s", pkg.cmdStr);
  }
  else if (pkg.msgType == CMD_TALK) snprintf(logBuf, sizeof(logBuf), "Talking (%dms)", pkg.param1);
  else if (pkg.msgType == CMD_HEARTBEAT) return; 
  else snprintf(logBuf, sizeof(logBuf), "Command: %d", pkg.msgType);
  
  Serial.printf("EXEC (ME=D%d Target=D%d Type=%d): %s\n", MY_ID, pkg.targetDroid, pkg.msgType, logBuf);
  lastCommand = logBuf;
  
  if (pkg.msgType == CMD_SERVO_MOVE) {
    int s = -1;
    if (strcmp(pkg.cmdStr, "headturn") == 0) s = 0;
    else if (strcmp(pkg.cmdStr, "headtilt") == 0) s = 1;
    else if (strcmp(pkg.cmdStr, "torsoturn") == 0) s = 2;
    
    if (s != -1) {
      servos[s].targetPos = pkg.param2; servos[s].moveDuration = pkg.param3; 
      servos[s].moveStartTime = millis(); servos[s].startPos = servos[s].currentPos;
    }
  } else if (pkg.msgType == CMD_PLAY_AUDIO) {
    String p = (pkg.cmdStr[0] != '\0' && strcmp(pkg.cmdStr, "test") != 0) ? "/" + String(pkg.cmdStr) : "/BD" + String(MY_ID) + ".wav";
    playWavFile(p);
  } else if (pkg.msgType == CMD_TALK) { isTalking = true; talkEndTime = millis() + pkg.param1; }
}
