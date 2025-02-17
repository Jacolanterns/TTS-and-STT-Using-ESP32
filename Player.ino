#include <WiFi.h>                // Include WiFi library for ESP32
#include <HTTPClient.h>          // Include HTTP client library
#include "FS.h"                  // Include filesystem support
#include "SD.h"                  // Include SD card support
#include "SPI.h"                 // Include SPI library for SD card communication
#include <ArduinoJson.h>         // Include ArduinoJson library
#include <driver/i2s.h>          // Include ESP32 I2S driver

#define LED_BUILTIN 2 // Onboard LED pin

// Wi-Fi credentials
const char *ssid = "ASTHA";
const char *password = "testing123";

// Web server details
const char *serverUrl = "http://192.168.43.88:3000/api"; // Base URL of your web server

// SD Card SPI Pins
#define SD_CS 5    // Chip Select pin for SD card
#define SD_SCK 18  // SPI Clock pin
#define SD_MOSI 23 // SPI MOSI pin
#define SD_MISO 19 // SPI MISO pin

// I2S Pins for MAX98357
#define I2S_DOUT 25 // Data pin (DIN)
#define I2S_BCLK 26 // Bit clock pin (BCLK)
#define I2S_LRC 27  // Left/Right clock pin (LRC)

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED while connecting
    delay(500);
    Serial.print(".");
  }
  digitalWrite(LED_BUILTIN, HIGH); // Turn LED ON when connected
  Serial.println("\nConnected to WiFi!");
  Serial.println(WiFi.localIP());
}

bool downloadAndSaveFile(const char *filename, const char *url) {
  HTTPClient http;
  http.setTimeout(10000); // Set timeout to 10 seconds
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
      http.end();
      return false;
    }

    WiFiClient *stream = http.getStreamPtr();
    if (!stream) {
      Serial.println("Failed to get HTTP stream");
      file.close();
      http.end();
      return false;
    }

    uint8_t buffer[128];
    size_t size = 0;

    while (http.connected() && (size = stream->readBytes(buffer, sizeof(buffer))) > 0) {
      file.write(buffer, size);
    }

    file.close();
    http.end();
    Serial.println("File downloaded successfully");
    return true;
  } else {
    Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    Serial.println("Server response: " + http.getString()); // Print server response
    http.end();
    return false;
  }
}

void playAudioFile(const char *filename) {
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Skip the WAV header (44 bytes for standard WAV files)
  file.seek(44);

  Serial.println("Playing audio file...");

  // Read and play the audio data
  uint8_t buffer[256]; // Increased buffer size for higher sample rate
  size_t bytesRead;
  while (file.available()) {
    bytesRead = file.read(buffer, sizeof(buffer));

    // Apply digital gain
    for (size_t i = 0; i < bytesRead; i += 2) {
      int16_t sample = buffer[i] | (buffer[i + 1] << 8);
      sample = (int16_t)(sample * 1.5); // Amplify by 1.5x

      // Prevent clipping
      if (sample > 32767) sample = 32767;
      if (sample < -32768) sample = -32768;

      buffer[i] = sample & 0xFF;
      buffer[i + 1] = (sample >> 8) & 0xFF;
    }

    size_t bytesWritten = 0;
    i2s_write(I2S_NUM_0, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
  }

  file.close();
  Serial.println("Audio playback finished.");
  // Send TTS control request after playback
  sendTTSControl();
}

String getTTSStatus() {
  HTTPClient http;
  String url = String(serverUrl) + "/tts_status";
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String jsonResponse = http.getString();
    http.end();

    // Parse JSON response
    DynamicJsonDocument jsonDoc(1024); // Adjust size based on your JSON payload
    DeserializationError error = deserializeJson(jsonDoc, jsonResponse);

    if (error) {
      Serial.println("Failed to parse JSON response");
      return "";
    }

    // Extract the "status" field
    String status = jsonDoc["status"];
    return status;
  } else {
    Serial.printf("Failed to get TTS status, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return "";
  }
}

void requestTTSFile() {
  HTTPClient http;
  String url = String(serverUrl) + "/tts_req";
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    Serial.println("TTS request successful");
    http.end();
  } else {
    Serial.printf("Failed to request TTS file, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
  }
}

bool sendTTSControl() {
  HTTPClient http;
  String url = String(serverUrl) + "/tts_control";
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    Serial.println("TTS control request successful");
    http.end();
    return true;
  } else {
    Serial.printf("Failed to send TTS control request, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Turn off LED initially

  Serial.begin(115200);

  // Initialize SPI for SD card
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    while (true);
  }
  Serial.println("SD Card initialized.");

  // Configure I2S with updated sample rate and optimized buffer sizes
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 12000, // Changed from 12000 to 44100
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 16,   // Increased from 8 to 16
    .dma_buf_len = 128,    // Increased from 64 to 128
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  Serial.println("I2S initialized.");

  // Connect to Wi-Fi
  connectToWiFi();
}

void loop() {
  // Check TTS status
  String ttsStatus = getTTSStatus();
  if (ttsStatus == "ready") {
    Serial.println("TTS status: ready. Requesting TTS file...");

    // Request TTS file
    // requestTTSFile();

    // Download and save the new audio file
    if (downloadAndSaveFile("/audio.wav", (String(serverUrl) + "/tts_req").c_str())) {
      Serial.println("Audio file saved as /audio.wav");

      // Play the audio file
      playAudioFile("/audio.wav");

    }
  } else {
    Serial.println("TTS status: not ready. Waiting...");
    delay(500);
  }
}