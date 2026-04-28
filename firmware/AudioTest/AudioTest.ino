#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "driver/i2s.h"

// Makerfabs V2.0 Pins
#define BCLK 26
#define LRC  25
#define DOUT 27

#define SD_CS 22
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

// Wav Header Structure
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

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- MANUAL WAV-I2S TEST START ---");

  // Mount SD
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if(!SD.begin(SD_CS, SPI, 40000000)) {
    Serial.println("SD FAIL");
    while(1);
  }

  // Find a WAV file
  String fileName = "";
  File root = SD.open("/");
  File file = root.openNextFile();
  while(file) {
    String n = String(file.name());
    if (n.endsWith(".wav")) { fileName = "/" + n; break; }
    file = root.openNextFile();
  }
  
  if (fileName == "") {
    Serial.println("No WAV file found on SD!");
    while(1);
  }
  Serial.printf("Playing: %s\n", fileName.c_str());

  // I2S Config
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100, // Will update from header
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128
  };
  i2s_pin_config_t pin_config = { .bck_io_num = BCLK, .ws_io_num = LRC, .data_out_num = DOUT, .data_in_num = -1 };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  File wavFile = SD.open(fileName);
  wav_header_t header;
  wavFile.read((uint8_t*)&header, sizeof(wav_header_t));

  Serial.printf("Sample Rate: %d, Channels: %d, Bits: %d\n", header.sampleRate, header.numChannels, header.bitsPerSample);
  i2s_set_sample_rates(I2S_NUM_0, header.sampleRate);

  uint8_t buf[512];
  size_t bytesWritten;
  while(wavFile.available()) {
    int bytesRead = wavFile.read(buf, sizeof(buf));
    i2s_write(I2S_NUM_0, buf, bytesRead, &bytesWritten, portMAX_DELAY);
  }
  wavFile.close();
  Serial.println("Done.");
}

void loop() {}
