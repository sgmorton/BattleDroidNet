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
#define CMD_MODE_SET 7
#define CMD_START_SEQUENCE 10

// --- MODE DEFINITIONS ---
#define MODE_NET_AUTO 0
#define MODE_NET_MANUAL 1
#define MODE_STANDALONE 2

// --- SERVO DEFINITIONS ---
#define HEAD_TURN_CHUX 0
#define HEAD_TILT_CHUX 1
#define TORSO_TURN_CHUX 2

#define VOL_UP_PIN 2
#define VOL_DN_PIN 15
#define VOL_PUSH_PIN 33

#define NAV_RIGHT_PIN 39
#define NAV_LEFT_PIN 36
#define NAV_PUSH_PIN 35

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
void copyDirToRoot(String folder);
void playDroidAudio(int id, int times);
void startProvisioning();
void playWavFile(String path);
void runSystemCheck();
void updateSequencer();
void updateSerial();


packet outPkg, inPkg;
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t myMac[6];

// --- SYSTEM STATE ---
bool IS_MASTER = false;
bool isMasterController = false;
int MY_ID = 0;
String lastCommand = "WAITING";
unsigned long lastMasterHeard = 0;
unsigned long droidLastSeen[11];
bool sdAvailable = false;
int globalVolume = 10; 

// --- SERVO CONFIG ---
typedef struct {
  int pin;
  int minPulse;
  int maxPulse;
  bool inverted;
  int currentPos;
  int targetPos;
  int startPos;
  unsigned long moveStartTime;
  int moveDuration;
  Servo servo;
} ServoConfig;

ServoConfig servos[3] = {
  {13, 500, 2500, false, 50, 50, 50, 0, 0}, // Head Turn
  {14, 500, 2500, false, 50, 50, 50, 0, 0}, // Head Tilt
  {12, 500, 2500, false, 50, 50, 50, 0, 0}  // Torso Turn
};

bool isTalking = false;
unsigned long talkEndTime = 0;
bool pendingStandaloneInit = false; // Deferred flag: run provision+syscheck on next loop tick
unsigned long nextTalkMove = 0;

int currentMode = MODE_NET_MANUAL; // Start in NET MANUAL mode for readiness
unsigned long nextSequenceTime = 0;
unsigned long sequenceStartTime = 0;
#define MAX_SEQ_LINES 150
String seqLines[MAX_SEQ_LINES];
int seqLineCount = 0;
int seqCurrentLine = 0;
unsigned long nextEventTime = 0;
bool sequenceActive = false;
String pendingCommands = "";
unsigned long nextStandaloneAction = 0;
bool standaloneSound = true;
bool standaloneMovement = true;
int standaloneMinDelay = 5000;
String standaloneSeqName = "";

// --- MENU STATE ---
bool menuActive = false;
bool menuSubActive = false;
int menuIdx = 0;
int menuSubIdx = 1;
unsigned long lastMenuAction = 0;

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
  if (currentMode != MODE_STANDALONE) {
    esp_err_t result = esp_now_send(broadcastAddr, (uint8_t *)&pkg, sizeof(pkg));
    if (result != ESP_OK) Serial.println("SEND FAIL: " + String(result));
    else Serial.println("SENT: type=" + String(pkg.msgType) + ", target=" + String(pkg.targetDroid));
  } else {
    Serial.println("STANDALONE: Local Execute type=" + String(pkg.msgType) + ", target=" + String(pkg.targetDroid));
  }
  // Execute locally if we are a droid (ID > 0) or the promoted controller
  if (MY_ID > 0 || isMasterController) executeCommand(pkg);
}

volatile uint32_t audioDataRemaining = 0;

void playWavFile(String path) {
  if (currentMode == MODE_STANDALONE && !standaloneSound) return;
  if (!sdAvailable) { lastCommand = "SD ERROR"; return; }
  if (!SD.exists(path)) { 
    String disp = path;
    if (disp.startsWith("/")) disp = disp.substring(1);
    disp.replace(".wav", "");
    disp.replace(".WAV", "");
    lastCommand = "404: " + disp;
    Serial.println("WAV: " + path + " NOT FOUND"); 
    return; 
  }
  if (audioStreaming) { 
    audioStreaming = false; 
    vTaskDelay(50 / portTICK_PERIOD_MS); // Let audioTask pause safely
    if (audioFile) audioFile.close(); 
  }
  
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
      if (audioDataRemaining < 100) audioDataRemaining = 0xFFFFFFFF; 
      Serial.printf("WAV: Data chunk %u bytes\n", chunkSize);
      break;
    } else {
      if (chunkSize % 2 != 0) chunkSize++; // WAV chunks are strictly 2-byte aligned
      audioFile.seek(audioFile.position() + chunkSize);
    }
  }

  if (!foundData) { Serial.println("WAV: No data"); audioFile.close(); return; }

  // Only reinitialise I2S hardware if the sample rate actually changed.
  // Calling i2s_set_sample_rates() resets the DMA and causes a brief glitch.
  static uint32_t currentI2SSampleRate = 0;
  if (audioSampleRate != currentI2SSampleRate) {
    i2s_set_sample_rates(I2S_NUM_0, audioSampleRate);
    currentI2SSampleRate = audioSampleRate;
    Serial.printf("WAV: I2S rate changed -> %u Hz\n", audioSampleRate);
  }

  // Wipe the DMA clean before pre-roll so no stale audio from the last track plays
  i2s_zero_dma_buffer(I2S_NUM_0);
  audioHead = 0;
  audioTail = 0;

  // --- PRE-BUFFERING ---
  // Fill a large initial buffer before starting playback to prevent startup stutter.
  // We wait until 75% of the PSRAM buffer is filled OR the file is fully loaded.
  uint32_t prerollTarget = min((uint32_t)(audioBufferSize * 3 / 4), (uint32_t)(512 * 1024));
  Serial.printf("WAV: Pre-rolling %u KB...\n", prerollTarget / 1024);

  while (audioDataRemaining > 0 && getAudioBytesInQueue() < prerollTarget) {
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
          audioDataRemaining = 0; // Natural EOF reached
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
        // Use portMAX_DELAY so we never drop a chunk when the DMA is temporarily full
        i2s_write(I2S_NUM_0, psramAudioBuffer + audioTail, contigChunk, &bytesWritten, portMAX_DELAY);
        if (bytesWritten > 0) {
          audioTail = (audioTail + bytesWritten) % audioBufferSize;
          didWork = true;
        }
      } else if (audioDataRemaining == 0) {
        // Silence Flush: Push zeroes to force all real audio out of the DMA pipeline
        uint8_t silence[4096] = {0};
        for (int i = 0; i < 34; i++) { // 34 * 4096 = 139KB > 64*512*4 = 131KB DMA capacity
          size_t bw = 0;
          i2s_write(I2S_NUM_0, silence, sizeof(silence), &bw, portMAX_DELAY);
        }
        i2s_zero_dma_buffer(I2S_NUM_0); // Wipe any residual audio from hardware registers
        audioStreaming = false;
        audioFile.close();
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

    // Handle buttons (ignore first 2s to prevent boot noise)
    if (now > 2000) {
      static int lastVolUp = -1, lastVolDn = -1;
      static int lastNavL = -1, lastNavR = -1, lastNavP = -1;
      if (lastNavP == -1) {
        lastVolUp = digitalRead(VOL_UP_PIN); lastVolDn = digitalRead(VOL_DN_PIN);
        lastNavL = digitalRead(NAV_LEFT_PIN); lastNavR = digitalRead(NAV_RIGHT_PIN); lastNavP = digitalRead(NAV_PUSH_PIN);
      }

      // Handle Volume Rocker
      int volUp = digitalRead(VOL_UP_PIN);
      int volDn = digitalRead(VOL_DN_PIN);
      if (volUp == LOW && lastVolUp == HIGH) {
        if (globalVolume < 10) { globalVolume++; Serial.printf("VOL: %d\n", globalVolume); saveSettings(); }
      }
      if (volDn == LOW && lastVolDn == HIGH) {
        if (globalVolume > 0) { globalVolume--; Serial.printf("VOL: %d\n", globalVolume); saveSettings(); }
      }
      lastVolUp = volUp; lastVolDn = volDn;

      // Handle Menu Rocker (NAV)
      int navL = digitalRead(NAV_LEFT_PIN);
      int navR = digitalRead(NAV_RIGHT_PIN);
      int navP = digitalRead(NAV_PUSH_PIN);

      if ((navL == LOW && lastNavL == HIGH) || (navR == LOW && lastNavR == HIGH) || (navP == LOW && lastNavP == HIGH)) {
        lastMenuAction = now;
        if (!menuActive) {
        if (navP == LOW && lastNavP == HIGH) {
          menuActive = true;
          menuSubActive = false; // Always start at main menu
          menuIdx = 0;
          Serial.println("MENU: ACTIVE");
        }
      } else {
        if (menuSubActive) {
          if (navR == LOW && lastNavR == HIGH) { // Up button (39) decreases ID
            menuSubIdx--; if (menuSubIdx < 1) menuSubIdx = 8;
          }
          else if (navL == LOW && lastNavL == HIGH) { // Down button (36) increases ID
            menuSubIdx++; if (menuSubIdx > 8) menuSubIdx = 1;
          }
          else if (navP == LOW && lastNavP == HIGH) {
            Serial.printf("MENU: ID CHANGE -> D%d\n", menuSubIdx);
            MY_ID = menuSubIdx; 
            currentMode = MODE_NET_MANUAL; // Reset to active fleet mode
            if (MY_ID == 1) { IS_MASTER = true; isMasterController = false; }
            else { IS_MASTER = false; }
            saveSettings(); 
            menuSubActive = false; menuActive = false;
            if (IS_MASTER) updateMasterUI(); else updateSlaveUI();
            provisionAssets();
          }
        } else {
          int maxIdx = IS_MASTER ? 3 : 1; 
          if (navL == LOW && lastNavL == HIGH) { menuIdx++; if (menuIdx > maxIdx) menuIdx = 0; } // Down (L) moves Down
          else if (navR == LOW && lastNavR == HIGH) { menuIdx--; if (menuIdx < 0) menuIdx = maxIdx; } // Up (R) moves Up
          else if (navP == LOW && lastNavP == HIGH) {
            if (IS_MASTER) {
              if (menuIdx == 0 || menuIdx == 1) {
                if (currentMode == MODE_STANDALONE) {
                  currentMode = (menuIdx == 0) ? MODE_NET_AUTO : MODE_NET_MANUAL;
                  startNetworkJoin();
                  menuActive = false;
                } else {
                  currentMode = (menuIdx == 0) ? MODE_NET_AUTO : MODE_NET_MANUAL;
                  packet p = {CMD_MODE_SET, (uint8_t)currentMode}; broadcastCommand(p);
                }
              }
              else if (menuIdx == 2) { menuSubActive = true; menuSubIdx = (MY_ID == 0 ? 1 : MY_ID); Serial.println("MENU: SUB -> CHOOSE DROID"); }
              else if (menuIdx == 3) { 
                currentMode = MODE_STANDALONE; 
                Serial.println("[[MODE:STANDALONE]]"); Serial.flush(); 
                packet p = {CMD_HEARTBEAT, (uint8_t)MY_ID}; p.param1 = 0xFFFF; broadcastCommand(p);
                saveSettings();
                pendingStandaloneInit = true; // Defer heavy SD work to main loop
              }
            } else {
              if (menuIdx == 0) { 
                if (currentMode == MODE_STANDALONE) {
                  currentMode = MODE_NET_AUTO;
                  startNetworkJoin();
                  menuActive = false;
                } else {
                  menuSubActive = true; menuSubIdx = (MY_ID == 0 ? 1 : MY_ID); Serial.println("MENU: SUB -> CHOOSE DROID"); 
                }
              }
              else if (menuIdx == 1) { 
                currentMode = MODE_STANDALONE; 
                Serial.println("MODE: STANDALONE ACTIVE"); 
                packet p = {CMD_HEARTBEAT, (uint8_t)MY_ID}; p.param1 = 0xFFFF; broadcastCommand(p);
                saveSettings();
                pendingStandaloneInit = true; // Defer heavy SD work to main loop
              }
            }
            bool isIDSelection = (IS_MASTER && menuIdx == 2) || (!IS_MASTER && menuIdx == 0);
            if (!isIDSelection) menuActive = false; // Close menu on mode change
            if (!menuActive) { if (IS_MASTER) updateMasterUI(); else updateSlaveUI(); } 
            saveSettings();
          }
        }
      }
      }
      if (menuActive && now - lastMenuAction > 15000) { menuActive = false; menuSubActive = false; Serial.println("MENU: TIMEOUT"); }
      lastNavL = navL; lastNavR = navR; lastNavP = navP;
    }
    updateSequencer();
    handleStandaloneMode();
  
    static unsigned long lastTele = 0;
    static unsigned long lastUi = 0;
    if (now - lastUi > 500) {
      if (IS_MASTER && currentMode != MODE_STANDALONE) {
        outPkg.msgType = CMD_HEARTBEAT; outPkg.targetDroid = MY_ID;
        esp_now_send(broadcastAddr, (uint8_t *)&outPkg, sizeof(outPkg));
        droidLastSeen[MY_ID] = now;
        updateMasterUI();
      } else {
        static unsigned long lastHb = 0;
        if (currentMode != MODE_STANDALONE && now - lastHb > 1000) { 
          outPkg.msgType = CMD_HEARTBEAT; outPkg.targetDroid = MY_ID;
          esp_now_send(broadcastAddr, (uint8_t *)&outPkg, sizeof(outPkg));
          lastHb = now;
        }
        updateSlaveUI();
      }
      lastUi = now;
    }
    
    lastUpdate = now;
  }
  updateSequencer(); 
  updateSerial();

  // Deferred standalone init — runs outside menu handler to avoid FreeRTOS blocking
  if (pendingStandaloneInit) {
    pendingStandaloneInit = false;
    Serial.println("INIT: Running deferred standalone provision + system check");
    provisionAssets();
  }

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
  File f = SD.open(filename);
  if (!f) return;
  seqLineCount = 0;
  seqCurrentLine = 0;
  while(f.available() && seqLineCount < MAX_SEQ_LINES) {
    String line = f.readStringUntil('\n'); line.trim();
    if (line.length() > 0) {
      seqLines[seqLineCount++] = line;
    }
  }
  f.close();
  sequenceStartTime = millis(); sequenceActive = true; nextEventTime = 0; pendingCommands = "";
}

void updateSequencer() {
  if (MY_ID != 1 && !isMasterController && currentMode != MODE_STANDALONE) return;
  unsigned long now = millis();

  if ((IS_MASTER || isMasterController) && currentMode != MODE_STANDALONE) {
    static unsigned long lastStatusReport = 0;
    if (now - lastStatusReport > 2000) {
      char sBuf[32]; int pos = 0; pos += snprintf(sBuf + pos, sizeof(sBuf) - pos, "[[STATUS:");
      for (int i = 1; i <= 8; i++) {
        bool isOnline = (droidLastSeen[i] > 0 && (now - droidLastSeen[i] < 8000));
        // Droid 1 is implicitly online only if we are the standalone Master (not web controlled)
        if (i == 1 && !isMasterController) isOnline = true;
        sBuf[pos++] = isOnline ? '1' : '0';
      }
      snprintf(sBuf + pos, sizeof(sBuf) - pos, "]]"); Serial.println(sBuf);
      
      // Mode report for Web UI
      const char* modeStrs[] = {"NET AUTO", "NET MANUAL", "STAND ALONE"};
      Serial.println("[[MODE:" + String(modeStrs[currentMode]) + "]]");

      lastStatusReport = now;
    }
  }

  if (!sdAvailable) return;

  if (IS_MASTER && currentMode == MODE_NET_AUTO && !sequenceActive && now > nextSequenceTime) {
    startSequence("/seq1.txt"); nextSequenceTime = now + random(120000, 300000);
  }

  if (sequenceActive) {
    unsigned long seqElapsed = now - sequenceStartTime;
    while (seqCurrentLine < seqLineCount || pendingCommands.length() > 0) {
      if (pendingCommands.length() == 0) {
        String line = seqLines[seqCurrentLine++];
        if (line.length() == 0) continue;
        float seqT = (millis() - sequenceStartTime) / 1000.0;
        Serial.printf("[SEQ: %5.2fs] PARSE: %s\n", seqT, line.c_str());
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
          float seqFire = (millis() - sequenceStartTime) / 1000.0;
          Serial.printf("[SEQ: %5.2fs] FIRE: %s\n", seqFire, pendingCommands.c_str());
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
          float seqFire = (millis() - sequenceStartTime) / 1000.0;
          Serial.printf("[SEQ: %5.2fs] FIRE: %s\n", seqFire, pendingCommands.c_str());
          broadcastCommand(p);
        } else if (pendingCommands.startsWith("T(")) {
          p.msgType = CMD_TALK;
          int s = pendingCommands.indexOf('('); int c = pendingCommands.indexOf(','); int e = pendingCommands.indexOf(')');
          String idStr = pendingCommands.substring(s + 1, c); idStr.trim();
          if (idStr != "*") p.targetDroid = idStr.toInt();
          p.param1 = pendingCommands.substring(c + 1, e).toInt();
          float seqFire = (millis() - sequenceStartTime) / 1000.0;
          Serial.printf("[SEQ: %5.2fs] FIRE: %s\n", seqFire, pendingCommands.c_str());
          broadcastCommand(p);
        } else if (pendingCommands.startsWith("RS")) {
          p.msgType = CMD_RESET;
          int s = pendingCommands.indexOf('('); int e = pendingCommands.indexOf(')');
          String idStr = pendingCommands.substring(s + 1, e); idStr.trim();
          if (idStr != "*") p.targetDroid = idStr.toInt();
          float seqFire = (millis() - sequenceStartTime) / 1000.0;
          Serial.printf("[SEQ: %5.2fs] FIRE: %s\n", seqFire, pendingCommands.c_str());
          broadcastCommand(p);
        }
        pendingCommands = "";
      } else { break; }
    }
    if (seqCurrentLine >= seqLineCount && pendingCommands.length() == 0) { sequenceActive = false; }
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
      // Scan SD for .seq files and report back to the web controller
      if (sdAvailable) {
        File root = SD.open("/");
        String seqList = "[[SEQLIST:";
        bool first = true;
        while (true) {
          File entry = root.openNextFile();
          if (!entry) break;
          String name = String(entry.name());
          if (!entry.isDirectory() && (name.endsWith(".seq") || name.endsWith(".SEQ"))) {
            if (!first) seqList += ",";
            seqList += name;
            first = false;
          }
          entry.close();
        }
        root.close();
        seqList += "]]";
        Serial.println(seqList);
      }
      return;
    }
    
    if (!IS_MASTER) return; // Only Master (or promoted controller) handles fleet commands

    if (line == "AUTO") { currentMode = MODE_NET_AUTO; saveSettings(); packet p = {CMD_MODE_SET, (uint8_t)currentMode}; broadcastCommand(p); }
    else if (line == "MANUAL") { currentMode = MODE_NET_MANUAL; saveSettings(); packet p = {CMD_MODE_SET, (uint8_t)currentMode}; broadcastCommand(p); }
    else if (line == "STANDALONE") { currentMode = MODE_STANDALONE; saveSettings(); provisionAssets(); }
    else if (line == "RESET") {
      packet p = {}; p.msgType = CMD_RESET; p.targetDroid = TARGET_ALL;
      broadcastCommand(p);
      Serial.println("SENT FLEET RESET");
    }
    else if (line.startsWith("SETID ")) {
      MY_ID = line.substring(6).toInt();
      saveSettings();
      Serial.printf("ID SET TO D%d\n", MY_ID);
    }
    else if (line.startsWith("PLAY:")) {
      String fn = line.substring(5);
      if (isMasterController) {
        packet p = {CMD_START_SEQUENCE, 1}; // Send to Droid 1
        strncpy(p.cmdStr, fn.c_str(), sizeof(p.cmdStr));
        broadcastCommand(p);
      } else {
        startSequence(("/" + fn).c_str());
      }
    }
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
    else {
      int eqIdx = line.indexOf('=');
      if (eqIdx != -1) {
        String key = line.substring(0, eqIdx);
        String val = line.substring(eqIdx + 1);
        if (key == "vol") { globalVolume = val.toInt(); Serial.printf("SERIAL VOL: %d\n", globalVolume); }
        saveSettings();
      }
    }
  }
}

void updateMasterUI() {
  display.clearDisplay();
  
  // Header (Large)
  display.setTextSize(2); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if (currentMode == MODE_STANDALONE) {
    if (sequenceActive || audioStreaming) {
      display.print(standaloneSeqName.substring(0, 10));
    } else {
      unsigned long rem = (nextStandaloneAction > millis()) ? (nextStandaloneAction - millis()) / 1000 : 0;
      display.printf("NEXT: %lus", rem);
    }
  }
  else if (isMasterController) display.print("CONTROLLER");
  else if (MY_ID == 1) display.print("MASTER");
  else { display.setTextSize(1); display.print("ONLINE HADB"); }
  
  // Mode and Command (Normal)
  display.setTextSize(1);
  const char* modeStrs[] = {"NET AUTO", "NET MANUAL", "STAND ALONE"};
  display.setCursor(0, 20); display.printf("MODE: %s", modeStrs[currentMode]);
  
  String uiMsg = lastCommand;
  bool isAudioBusy = audioStreaming || (getAudioBytesInQueue() > 100); 
  if (!isAudioBusy && !isTalking && !sequenceActive) {
    if (uiMsg.startsWith("Playing") || uiMsg.startsWith("Talking") || uiMsg.startsWith("Command")) uiMsg = "WAITING";
  }
  display.setCursor(0, 32); display.print(uiMsg.substring(0, 20));

  // Fleet Status (Boxes at bottom)
  unsigned long now = millis();
  int startX = 2; int y = 48; int boxW = 12; int boxH = 10; int gap = 4;
  for (int i = 1; i <= 8; i++) {
    int x = startX + (i - 1) * (boxW + gap);
    bool online = (droidLastSeen[i] > 0 && (now - droidLastSeen[i] < 10000));
    if (online) {
      display.fillRect(x, y, boxW, boxH, SSD1306_WHITE);
    } else {
      display.drawRect(x, y, boxW, boxH, SSD1306_WHITE);
    }
  }

  // Volume Bar (Horizontal at bottom)
  int volW = map(globalVolume, 0, 10, 0, 124);
  display.drawFastHLine(2, 60, volW, SSD1306_WHITE);
  display.drawFastHLine(2, 61, volW, SSD1306_WHITE); // 2px height

  if (menuActive) {
    display.fillRect(10, 10, 108, 44, SSD1306_BLACK);
    display.drawRect(10, 10, 108, 44, SSD1306_WHITE);
    display.setTextSize(1); 
    if (menuSubActive) {
      display.setCursor(15, 15);
      display.println("SELECT DROID ID:");
      display.setTextSize(2); display.setCursor(50, 30); display.print("D" + String(menuSubIdx));
      display.setTextSize(1);
      bool online = (droidLastSeen[menuSubIdx] > 0 && (millis() - droidLastSeen[menuSubIdx] < 10000));
      if (online && menuSubIdx != MY_ID) display.print(" - ONLINE");
    } else {
      display.setCursor(15, 15);
      display.println("MENU:");
      const char* opt0 = (currentMode == MODE_STANDALONE) ? "JOIN NET (AUTO)" : "NET AUTO";
      const char* opt1 = (currentMode == MODE_STANDALONE) ? "JOIN NET (MAN)" : "NET MANUAL";
      const char* opts[] = {opt0, opt1, "CHOOSE DROID", "STAND ALONE"};
      for(int i=0; i<4; i++) {
        display.setCursor(15, 23 + (i * 8));
        if (menuIdx == i) {
          display.fillRect(15, 23 + (i * 8), 98, 8, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.setTextColor(SSD1306_WHITE);
        }
        display.println(opts[i]);
      }
      display.setTextColor(SSD1306_WHITE);
    }
  }

  display.display();
}

void updateSlaveUI() {
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); 
  if (currentMode == MODE_STANDALONE) {
    if (sequenceActive || audioStreaming) {
      display.println(standaloneSeqName.substring(0, 21));
    } else {
      unsigned long rem = (nextStandaloneAction > millis()) ? (nextStandaloneAction - millis()) / 1000 : 0;
      display.printf("NEXT SEQ: %lus\n", rem);
    }
  }
  else if (isMasterController) { display.setTextSize(2); display.println("CONTROLLER"); display.setTextSize(1); }
  else if (MY_ID == 1) { display.setTextSize(2); display.println("MASTER"); display.setTextSize(1); }
  else display.println("ONLINE HADB");
  if (currentMode == MODE_STANDALONE) { display.setCursor(0, 10); display.println("MODE: STAND ALONE"); }
  else { display.setCursor(0, 10); if (MY_ID == 0) display.println("SLAVE - SEARCHING"); else display.printf("SLAVE - UNIT D%d", MY_ID); }
  
  String uiMsg = lastCommand;
  bool isAudioBusy = audioStreaming || (getAudioBytesInQueue() > 100);
  if (!isAudioBusy && !isTalking && !sequenceActive) {
    if (uiMsg.startsWith("Playing") || uiMsg.startsWith("Talking") || uiMsg.startsWith("Command")) uiMsg = "WAITING";
  }
  display.setCursor(0, 20); display.println(uiMsg.substring(0, 20));
  display.setCursor(0, 32); display.setTextSize(2); 
  if (currentMode == MODE_STANDALONE) display.println("ISOLATION");
  else if (MY_ID == 0) display.println("SEARCH"); 
  else display.printf("UNIT: D%d", MY_ID);

  // Volume Bar (Horizontal at bottom)
  int volW = map(globalVolume, 0, 10, 0, 124);
  display.drawFastHLine(2, 60, volW, SSD1306_WHITE);
  display.drawFastHLine(2, 61, volW, SSD1306_WHITE); // 2px height

  if (menuActive) {
    display.fillRect(10, 10, 108, 44, SSD1306_BLACK);
    display.drawRect(10, 10, 108, 44, SSD1306_WHITE);
    display.setTextSize(1); 
    if (menuSubActive) {
      display.setCursor(15, 15);
      display.println("SELECT DROID ID:");
      display.setTextSize(2); display.setCursor(50, 30); display.print("D" + String(menuSubIdx));
      display.setTextSize(1);
      bool online = (droidLastSeen[menuSubIdx] > 0 && (millis() - droidLastSeen[menuSubIdx] < 10000));
      if (online && menuSubIdx != MY_ID) display.print(" - ONLINE");
    } else {
      display.setCursor(15, 15);
      display.println("MENU:");
      const char* opt0 = (currentMode == MODE_STANDALONE) ? "JOIN NETWORK" : "CHOOSE DROID";
      const char* opts[] = {opt0, "STAND ALONE"};
      for(int i=0; i<2; i++) {
        display.setCursor(15, 23 + (i * 8));
        if (menuIdx == i) {
          display.fillRect(15, 23 + (i * 8), 98, 8, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else {
          display.setTextColor(SSD1306_WHITE);
        }
        display.println(opts[i]);
      }
      display.setTextColor(SSD1306_WHITE);
    }
  }

  display.display();
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  // Prevent executing our own broadcasts (loopback)
  if (memcmp(info->src_addr, myMac, 6) == 0) return;

  memcpy(&inPkg, data, sizeof(inPkg));
  if (currentMode == MODE_STANDALONE) return; // Ignore everything in Standalone mode
  
  if (IS_MASTER && inPkg.msgType != CMD_HEARTBEAT) Serial.println("RECV PKT: T=" + String(inPkg.targetDroid) + " ME=" + String(MY_ID) + " TYPE=" + String(inPkg.msgType));

  if (inPkg.msgType == CMD_HEARTBEAT) {
    if (inPkg.targetDroid == 0 || inPkg.targetDroid == 1) lastMasterHeard = millis();
    if (IS_MASTER && inPkg.targetDroid > 0 && inPkg.targetDroid <= 8) {
      if (inPkg.param1 == 0xFFFF) {
        droidLastSeen[inPkg.targetDroid] = 0;
        Serial.println("FLEET: D" + String(inPkg.targetDroid) + " OFFLINE (MANUAL)");
      } else {
        if (droidLastSeen[inPkg.targetDroid] == 0) Serial.println("FLEET: D" + String(inPkg.targetDroid) + " ONLINE");
        droidLastSeen[inPkg.targetDroid] = millis();
      }
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
  } else if (inPkg.msgType == CMD_MODE_SET) {
    currentMode = inPkg.targetDroid;
    saveSettings(); 
    const char* modeStrs[] = {"NET AUTO", "NET MANUAL", "STAND ALONE"};
    Serial.println("[[MODE:" + String(modeStrs[currentMode]) + "]]"); Serial.flush();
  } else if (inPkg.msgType == CMD_ID_ASSIGN && MY_ID == 0 && !isMasterController) {
    MY_ID = inPkg.targetDroid;
  }
}

void setup() {
  Serial.begin(115200);
  for(int i=0; i<3; i++) { 
    Serial.println("###################################");
    Serial.println("###   BATTLE DROID V2.2 ACTIVE  ###");
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
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = 64, .dma_buf_len = 512,
    .tx_desc_auto_clear = true  // Auto-zero DMA buffers on underrun — prevents audio looping
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

  pinMode(VOL_UP_PIN, INPUT_PULLUP);
  pinMode(VOL_DN_PIN, INPUT_PULLUP);
  pinMode(VOL_PUSH_PIN, INPUT_PULLUP);
  pinMode(NAV_LEFT_PIN, INPUT);
  pinMode(NAV_RIGHT_PIN, INPUT);
  pinMode(NAV_PUSH_PIN, INPUT);

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

  startNetworkJoin();

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
  
  // Cleanup legacy settings folder if it exists
  if (SD.exists("/settings")) {
    Serial.println("SETTINGS: CLEANING LEGACY FOLDER");
    if (SD.exists("/settings/settings.ini")) SD.remove("/settings/settings.ini");
    SD.rmdir("/settings");
  }

  if (!SD.exists("/settings.ini")) {
    Serial.println("SETTINGS: NOT FOUND - CREATING DEFAULT");
    saveSettings(); 
    return;
  }
  
  File f = SD.open("/settings.ini");
  if (!f) { Serial.println("SETTINGS: OPEN FAIL (READ)"); return; }
  Serial.println("SETTINGS: LOADING...");
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("#") || line.length() == 0) continue; // Skip comments and blank lines
    int eq = line.indexOf('=');
    if (eq != -1) {
      String key = line.substring(0, eq);
      String val = line.substring(eq + 1);
      if (key == "min_head_turn") { servos[0].minPulse = val.toInt(); Serial.printf("  HEAD_TURN_MIN: %d\n", servos[0].minPulse); }
      else if (key == "max_head_turn") { servos[0].maxPulse = val.toInt(); Serial.printf("  HEAD_TURN_MAX: %d\n", servos[0].maxPulse); }
      else if (key == "inv_head_turn") servos[0].inverted = (val.toInt() == 1);
      else if (key == "min_head_tilt") { servos[1].minPulse = val.toInt(); Serial.printf("  HEAD_TILT_MIN: %d\n", servos[1].minPulse); }
      else if (key == "max_head_tilt") { servos[1].maxPulse = val.toInt(); Serial.printf("  HEAD_TILT_MAX: %d\n", servos[1].maxPulse); }
      else if (key == "inv_head_tilt") servos[1].inverted = (val.toInt() == 1);
      else if (key == "min_torso_turn") { servos[2].minPulse = val.toInt(); Serial.printf("  TORSO_TURN_MIN: %d\n", servos[2].minPulse); }
      else if (key == "max_torso_turn") { servos[2].maxPulse = val.toInt(); Serial.printf("  TORSO_TURN_MAX: %d\n", servos[2].maxPulse); }
      else if (key == "inv_torso_turn") servos[2].inverted = (val.toInt() == 1);
      else if (key == "vol") { globalVolume = val.toInt(); Serial.printf("  VOL: %d\n", globalVolume); }
      else if (key == "mode") { 
        if (val == "NET_AUTO" || val == "AUTO") currentMode = MODE_NET_AUTO;
        else if (val == "NET_MANUAL" || val == "MANUAL") currentMode = MODE_NET_MANUAL;
        else if (val == "STAND_ALONE" || val == "STANDALONE") currentMode = MODE_STANDALONE;
        Serial.printf("  MODE: %s\n", val.c_str()); 
      }
      else if (key == "id") { MY_ID = val.toInt(); Serial.printf("  ID: %d\n", MY_ID); }
    }
  }
  f.close();
  Serial.println("SETTINGS: LOADED");
  loadStandaloneSettings();
}

void loadStandaloneSettings() {
  if (!sdAvailable) return;
  if (!SD.exists("/standalone.ini")) {
    saveStandaloneSettings();
    return;
  }
  File f = SD.open("/standalone.ini");
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("#") || line.length() == 0) continue; // Skip comments and blank lines
    int eq = line.indexOf('=');
    if (eq != -1) {
      String key = line.substring(0, eq); key.toLowerCase();
      String val = line.substring(eq + 1); val.toLowerCase();
      if (key == "sound") standaloneSound = (val == "true");
      else if (key == "movement") standaloneMovement = (val == "true");
      else if (key == "mindelay") standaloneMinDelay = val.toInt() * 1000;
    }
  }
  f.close();
  Serial.printf("STANDALONE SETTINGS: Sound=%d, Move=%d, Delay=%d\n", standaloneSound, standaloneMovement, standaloneMinDelay);
}

void saveStandaloneSettings() {
  if (!sdAvailable) return;
  if (SD.exists("/standalone.ini")) SD.remove("/standalone.ini");
  File f = SD.open("/standalone.ini", FILE_WRITE);
  if (!f) return;
  f.print("Sound="); f.println(standaloneSound ? "True" : "False");
  f.print("Movement="); f.println(standaloneMovement ? "True" : "False");
  f.print("MinDelay="); f.println(standaloneMinDelay);
  f.close();
}

void saveSettings() {
  if (!sdAvailable) return;
  
  // Explicitly remove to ensure a fresh, truncated file
  if (SD.exists("/settings.ini")) SD.remove("/settings.ini");
  if (SD.exists("/SETTINGS.INI")) SD.remove("/SETTINGS.INI");
  
  File f = SD.open("/settings.ini", FILE_WRITE); // FILE_WRITE will create new since we removed it
  if (!f) { Serial.println("SETTINGS: OPEN FAIL (WRITE)"); return; }
  
  f.print("min_head_turn="); f.println(servos[0].minPulse);
  f.print("max_head_turn="); f.println(servos[0].maxPulse);
  f.print("inv_head_turn="); f.println((int)servos[0].inverted);
  
  f.print("min_head_tilt="); f.println(servos[1].minPulse);
  f.print("max_head_tilt="); f.println(servos[1].maxPulse);
  f.print("inv_head_tilt="); f.println((int)servos[1].inverted);
  
  f.print("min_torso_turn="); f.println(servos[2].minPulse);
  f.print("max_torso_turn="); f.println(servos[2].maxPulse);
  f.print("inv_torso_turn="); f.println((int)servos[2].inverted);
  
  f.print("vol="); f.println(globalVolume);
  
  f.print("mode=");
  if (currentMode == MODE_NET_AUTO) f.println("NET_AUTO");
  else if (currentMode == MODE_NET_MANUAL) f.println("NET_MANUAL");
  else if (currentMode == MODE_STANDALONE) f.println("STAND_ALONE");
  else f.println("NET_MANUAL"); // Default
  
  f.print("id="); f.println(MY_ID);
  
  f.flush();
  f.close();
  Serial.println("SETTINGS: SAVED SUCCESSFULLY");

  // Read-back verification for debugging
  File v = SD.open("/settings.ini", FILE_READ);
  if (v) {
    Serial.println("SETTINGS: VERIFYING FILE CONTENT:");
    while(v.available()) {
      Serial.write(v.read());
    }
    v.close();
    Serial.println("SETTINGS: VERIFICATION COMPLETE");
  }
}

void clearSDRoot() { 
  if (!sdAvailable) return;
  File r = SD.open("/"); if (!r) return;
  
  // ESP32 SD library doesn't like deleting files while iterating with openNextFile.
  // We need to collect names first, then delete them.
  String filesToDelete[30];
  int deleteCount = 0;
  
  while(1) { 
    File f = r.openNextFile(); if (!f) break; 
    String name = String(f.name());
    
    // Protect settings and folders
    if (!f.isDirectory() && name != "settings.ini" && name != "/settings.ini" && name != "standalone.ini" && name != "/standalone.ini") {
      if (deleteCount < 30) {
        filesToDelete[deleteCount++] = name.startsWith("/") ? name : ("/" + name);
      }
    }
    f.close(); 
  } 
  r.close();
  
  // Now perform the actual deletions
  for (int i = 0; i < deleteCount; i++) {
    Serial.println("PROV: Deleting " + filesToDelete[i]);
    SD.remove(filesToDelete[i]);
  }
}
void copyDirToRoot(String folder) {
  if (!sdAvailable) return;
  File d = SD.open("/" + folder); 
  if (!d) { Serial.printf("PROV: Folder /%s not found\n", folder.c_str()); return; }
  while(1) {
    File f = d.openNextFile(); if (!f) break;
    if (f.isDirectory()) { f.close(); continue; }
    String destPath = "/" + String(f.name());
    if (destPath.endsWith("settings.ini") || destPath.endsWith("standalone.ini")) { f.close(); continue; }
    File df = SD.open(destPath, FILE_WRITE);
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

void provisionAssets() { 
  if (!sdAvailable) return; 
  if (currentMode == MODE_STANDALONE) {
    statusDisplay("PROV STANDALONE", 1);
    clearSDRoot();
    copyDirToRoot("standalone");
    statusDisplay("PROVISIONED SA", 1);
    runSystemCheck();  // Servo sweep + audio confirmation
  } else {
    if (MY_ID == 0) return;
    statusDisplay("PROVISIONING D" + String(MY_ID), 1); 
    clearSDRoot(); 
    copyDirToRoot(String(MY_ID)); 
    statusDisplay("PROVISIONED", 1); 
  }
}

// Sweeps all three servos through 25/50/75/center at 750ms per step,
// then plays standalone.wav as an audio-ok confirmation.
void runSystemCheck() {
  Serial.println("SYSCHECK: Starting servo & audio check...");
  statusDisplay("SYSTEM CHECK", 1);

  // Helper: move servo[idx] to position 0-100, wait for move to settle
  auto moveServo = [](int idx, int pos) {
    int p1 = servos[idx].inverted ? servos[idx].maxPulse : servos[idx].minPulse;
    int p2 = servos[idx].inverted ? servos[idx].minPulse : servos[idx].maxPulse;
    int pulse = map(pos, 0, 100, p1, p2);
    servos[idx].servo.writeMicroseconds(pulse);
    servos[idx].currentPos = pos;
    servos[idx].targetPos  = pos;
    servos[idx].startPos   = pos;
    delay(750);
  };

  const int positions[] = {25, 50, 75, 50}; // 50 = center
  const char* names[]   = {"headturn", "headtilt", "torsoturn"};

  for (int s = 0; s < 3; s++) {
    Serial.printf("SYSCHECK: %s\n", names[s]);
    for (int p : positions) {
      moveServo(s, p);
    }
  }

  Serial.println("SYSCHECK: Playing standalone.wav");
  statusDisplay("CHECK OK", 1);
  playWavFile("/standalone.wav");
}

void startNetworkJoin() {
  if (currentMode == MODE_STANDALONE) {
    IS_MASTER = false;
    Serial.println("BOOT: STANDALONE MODE - SKIPPING FLEET SYNC");
    provisionAssets();
    return;
  }
  
  statusDisplay("SEARCHING NET", 0);
  Serial.println("PROV: SEARCHING FOR MASTER...");
  unsigned long start = millis();
  lastMasterHeard = 0; 
  MY_ID = 0; 
  
  while (millis() - start < 5000 && lastMasterHeard == 0) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
  if (lastMasterHeard == 0) {
    Serial.println("PROV: NO MASTER FOUND. BECOMING MASTER D1");
    IS_MASTER = true; MY_ID = 1;
    statusDisplay("BECOMING MASTER", 1);
  } else {
    Serial.println("PROV: MASTER FOUND. REQUESTING ID...");
    packet req = {CMD_ID_REQUEST, 0}; esp_now_send(broadcastAddr, (uint8_t *)&req, sizeof(req));
    unsigned long wait = millis(); 
    while (MY_ID == 0 && millis() - wait < 3000) vTaskDelay(100 / portTICK_PERIOD_MS);
    
    if (MY_ID == 0) {
       IS_MASTER = true; MY_ID = 1;
       statusDisplay("BECOMING MASTER", 1);
    } else {
       IS_MASTER = false;
       statusDisplay("JOINED FLEET", 1);
    }
  }
  saveSettings();
  provisionAssets();
}
void statusDisplay(String msg, int size) {
  Serial.println("STATUS: " + msg);
  lastCommand = msg;
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
  
  float seqT = sequenceActive ? (millis() - sequenceStartTime) / 1000.0 : 0.0;
  if (sequenceActive) {
    Serial.printf("[SEQ: %5.2fs] EXEC (ME=D%d Target=D%d Type=%d): %s\n", seqT, MY_ID, pkg.targetDroid, pkg.msgType, logBuf);
  } else {
    Serial.printf("[NET] EXEC (ME=D%d Target=D%d Type=%d): %s\n", MY_ID, pkg.targetDroid, pkg.msgType, logBuf);
  }
  
  lastCommand = logBuf;
  
  if (pkg.msgType == CMD_SERVO_MOVE) {
    if (currentMode == MODE_STANDALONE && !standaloneMovement) return;
    int sIdx = -1;
    if (strcmp(pkg.cmdStr, "headturn") == 0) sIdx = 0;
    else if (strcmp(pkg.cmdStr, "headtilt") == 0) sIdx = 1;
    else if (strcmp(pkg.cmdStr, "torsoturn") == 0) sIdx = 2;
    
    if (sIdx != -1) {
      int p1 = servos[sIdx].inverted ? servos[sIdx].maxPulse : servos[sIdx].minPulse;
      int p2 = servos[sIdx].inverted ? servos[sIdx].minPulse : servos[sIdx].maxPulse;
      int pulse = map(pkg.param2, 0, 100, p1, p2);
      servos[sIdx].servo.writeMicroseconds(pulse);
      lastServoWrite[sIdx] = pkg.param2;
      servos[sIdx].targetPos = pkg.param2; servos[sIdx].moveDuration = pkg.param3; 
      servos[sIdx].moveStartTime = millis(); servos[sIdx].startPos = servos[sIdx].currentPos;
    }
  } else if (pkg.msgType == CMD_PLAY_AUDIO) {
    String p = "";
    if (pkg.cmdStr[0] != '\0' && strcmp(pkg.cmdStr, "test") != 0) {
      p = "/" + String(pkg.cmdStr);
      if (!p.endsWith(".wav") && !p.endsWith(".WAV")) p += ".wav";
    } else {
      p = "/BD" + String(MY_ID) + ".wav";
    }
    playWavFile(p);
  } else if (pkg.msgType == CMD_TALK) { isTalking = true; talkEndTime = millis() + pkg.param1; }
  else if (pkg.msgType == CMD_START_SEQUENCE) {
    if (MY_ID == 1) startSequence(("/" + String(pkg.cmdStr)).c_str());
  }
}
void handleStandaloneMode() {
  if (currentMode != MODE_STANDALONE || !sdAvailable) return;

  if (sequenceActive || audioStreaming) {
    nextStandaloneAction = 0; // Reset timer while active
    return;
  }

  if (nextStandaloneAction == 0) {
    // Sequence just finished (or we just entered standalone mode). Start the timer.
    nextStandaloneAction = millis() + (unsigned long)standaloneMinDelay + random(0, standaloneMinDelay);
    Serial.printf("STANDALONE: Waiting %lu ms before next sequence\n", nextStandaloneAction - millis());
    return;
  }

  if (millis() < nextStandaloneAction) return;

  // Scan for .seq files
  File root = SD.open("/");
  if (!root) return;

  String seqFiles[16];
  int count = 0;
  while (1) {
    File f = root.openNextFile();
    if (!f) break;
    String name = String(f.name());
    if (name.endsWith(".seq") || name.endsWith(".SEQ")) {
      seqFiles[count++] = "/" + name;
    }
    f.close();
    if (count >= 16) break;
  }
  root.close();

  if (count > 0) {
    int idx = random(0, count);
    standaloneSeqName = seqFiles[idx];
    standaloneSeqName.replace("/", "");
    standaloneSeqName.replace(".seq", "");
    standaloneSeqName.replace(".SEQ", "");
    standaloneSeqName.toUpperCase();
    
    Serial.printf("STANDALONE: Starting random sequence %s\n", seqFiles[idx].c_str());
    startSequence(seqFiles[idx].c_str());
  } else {
    // No sequences found, check again in 5s
    nextStandaloneAction = millis() + 5000;
  }
}

