#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SD.h>
#include <SPI.h>
#include "driver/i2s.h"
#include <ArduinoJson.h>

#define SD_CS_PIN 5                // Pin CS untuk modul SD Card
#define I2S_WS 15                  // Word Select (L/R clock)
#define I2S_SD 13                  // Serial Data
#define I2S_SCK 2                  // Bit Clock
#define BUFFER_SIZE 512

const uint32_t SAMPLE_RATE = 16000;  // Sample rate 16 kHz

const char* ssid = "ASTHA";      // Ganti dengan SSID Wi-Fi Anda
const char* password = "testing123";  // Ganti dengan password Wi-Fi Anda
const int LED_PIN = 2;              // LED in-build pada ESP32

const char* serverUrl = "http://192.168.43.88:3000/api/";

File audioFile;
volatile bool isRecording = false;
uint8_t fileIndex = 1;              // Indeks file untuk penamaan
String lastStatus = "";

void startRecording();
void stopRecording();
void setupI2S();
String generateFileName();
void connectWiFi();
String fetchServerStatus();
void uploadRecording(const String& fileName);
void fetchStatusTask(void *param);
void recordAudioTask(void *param);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Hubungkan ke Wi-Fi
  connectWiFi();

  // Inisialisasi SD Card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }
  Serial.println("SD Card initialized.");

  // Konfigurasi I2S
  setupI2S();

  Serial.println("System initialized. Monitoring server status...");

  // Replace loop usage with tasks, pinned to different cores
  xTaskCreatePinnedToCore(fetchStatusTask, "fetchStatusTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(recordAudioTask, "recordAudioTask", 8192, NULL, 1, NULL, 1);
}

void loop() {
  // Leave empty or minimal; tasks run in parallel
}

void setupI2S() {
  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pinConfig = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2sConfig, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pinConfig);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void startRecording() {
  String fileName = generateFileName();
  audioFile = SD.open(fileName, FILE_WRITE);

  if (!audioFile) {
    Serial.println("Failed to open file for recording.");
    return;
  }

  // Placeholder for RIFF/WAVE header
  const uint8_t riff[] = {'R','I','F','F'};
  audioFile.write(riff, sizeof(riff));
  uint32_t fileSizePlaceholder = 0;
  audioFile.write((uint8_t*)&fileSizePlaceholder, sizeof(fileSizePlaceholder));
  const uint8_t wave[] = {'W','A','V','E'};
  audioFile.write(wave, sizeof(wave));
  const uint8_t fmt[] = {'f','m','t',' '};
  audioFile.write(fmt, sizeof(fmt));
  uint32_t subchunk1Size = 16;
  audioFile.write((uint8_t*)&subchunk1Size, sizeof(subchunk1Size));
  uint16_t audioFormat = 1;
  audioFile.write((uint8_t*)&audioFormat, sizeof(audioFormat));
  uint16_t numChannels = 1;
  audioFile.write((uint8_t*)&numChannels, sizeof(numChannels));
  audioFile.write((uint8_t*)&SAMPLE_RATE, sizeof(SAMPLE_RATE));
  uint32_t byteRate = SAMPLE_RATE * numChannels * (16 / 8);
  audioFile.write((uint8_t*)&byteRate, sizeof(byteRate));
  uint16_t blockAlign = numChannels * (16 / 8);
  audioFile.write((uint8_t*)&blockAlign, sizeof(blockAlign));
  uint16_t bitsPerSample = 16;
  audioFile.write((uint8_t*)&bitsPerSample, sizeof(bitsPerSample));
  const uint8_t data[] = {'d','a','t','a'};
  audioFile.write(data, sizeof(data));
  audioFile.write((uint8_t*)&fileSizePlaceholder, sizeof(fileSizePlaceholder));

  Serial.print("Recording to file: ");
  Serial.println(fileName);

  isRecording = true;
  digitalWrite(LED_PIN, HIGH);
}

void stopRecording() {
  if (!isRecording) return;

  isRecording = false;
  digitalWrite(LED_PIN, LOW);

  size_t fileSize = audioFile.size();
  uint32_t fileSizeMinus8 = fileSize - 8;
  uint32_t dataChunkSize = fileSize - 44;

  audioFile.seek(4);
  audioFile.write((uint8_t*)&fileSizeMinus8, sizeof(fileSizeMinus8));
  audioFile.seek(40);
  audioFile.write((uint8_t*)&dataChunkSize, sizeof(dataChunkSize));

  String fileName = audioFile.name();
  audioFile.close();

  Serial.println("Recording stopped and saved to SD card.");

  uploadRecording(fileName);
}

String generateFileName() {
  String fileName = "/recording" + String(fileIndex) + ".wav";
  while (SD.exists(fileName)) {
    fileIndex++;
    fileName = "/recording" + String(fileIndex) + ".wav";
  }
  return fileName;
}

void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

String fetchServerStatus() {
  HTTPClient http;
  http.begin((String(serverUrl) + "stt_status").c_str());
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String response = http.getString();
    http.end();

    // Parse JSON response
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);

    if (!error) {
      const char* status = doc["status"];
      Serial.println("response :");
      Serial.println(String(status));
      return String(status);
    } else {
      Serial.println("Failed to parse JSON response.");
    }
  } else {
    Serial.printf("Failed to fetch status, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
  return "";
}

void uploadRecording(const String& fileName) {

  File file = SD.open("/"+fileName, FILE_READ);
  Serial.println(fileName);

  if (!file) {
    Serial.println("Failed to open file for upload.");

    return;
  }

  HTTPClient http;
  http.begin((String(serverUrl) + "stt_push").c_str());
  http.setTimeout(60000); // Set timeout to 60 seconds
  http.addHeader("Content-Type", "audio/wav");

  int httpResponseCode = http.sendRequest("POST", &file, file.size());

  if (httpResponseCode > 0) {
    Serial.printf("File uploaded, server responded with code: %d\n", httpResponseCode);
    String response = http.getString();
    Serial.println("Server Response:");
    Serial.println(response);
  } else {
    Serial.printf("File upload failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
  file.close();
}

void fetchStatusTask(void *param) {
  for (;;) {
    String status = fetchServerStatus();
    lastStatus = status; // Store the latest status
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void recordAudioTask(void *param) {
  for (;;) {
    // Check if we should start or stop recording
    if (lastStatus == "recording" && !isRecording) {
      Serial.println("start recording");
      startRecording();
    } else if (lastStatus != "recording" && isRecording) {
      Serial.println("stop recording");
      stopRecording();
    }

    // If recording, read from I2S and write to file
    if (isRecording) {
      int16_t buffer[BUFFER_SIZE];
      size_t bytesRead;
      i2s_read(I2S_NUM_0, (char*)buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);
      if (bytesRead > 0) {
        audioFile.write((uint8_t*)buffer, bytesRead);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}
