/*
 * ESP32-S3 BLE MIDI Receiver (接收端)
 *
 * Arduino IDE 设置:
 *   Tools → USB Mode → USB-OTG (TinyUSB)
 *   Tools → USB CDC On Boot → Disabled
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Adafruit_NeoPixel.h>
#include "USB.h"
#include "USBMIDI.h"

// ==================== 配置 ====================
#define TARGET_DEVICE_NAME "BLE-MIDI-TX"
#define LED_PIN 48
#define LED_COUNT 1

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define COLOR_OFF pixels.Color(0, 0, 0)
#define COLOR_RED pixels.Color(50, 0, 0)
#define COLOR_GREEN pixels.Color(0, 50, 0)
#define COLOR_BLUE pixels.Color(0, 0, 50)
#define COLOR_PURPLE pixels.Color(50, 0, 50)
#define COLOR_CYAN pixels.Color(0, 50, 50)

void setLED(uint32_t color) {
  pixels.setPixelColor(0, color);
  pixels.show();
}

// ==================== BLE MIDI ====================
#define MIDI_SERVICE_UUID "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

USBMIDI MIDI;

static bool doConnect = false;
static bool connected = false;
static bool doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice* myDevice = nullptr;
static BLEClient* pClient = nullptr;
static uint8_t lastStatus = 0;
static volatile unsigned long lastMidiActivity = 0;  // MIDI活动时间戳
#define MIDI_LED_DURATION 150  // 青色持续时间(ms)

// ==================== USB MIDI 发送函数 ====================
void sendNoteOn(uint8_t note, uint8_t velocity, uint8_t channel) {
  midiEventPacket_t event = {0x09, (uint8_t)(0x90 | (channel - 1)), note, velocity};
  MIDI.writePacket(&event);
}

void sendNoteOff(uint8_t note, uint8_t velocity, uint8_t channel) {
  midiEventPacket_t event = {0x08, (uint8_t)(0x80 | (channel - 1)), note, velocity};
  MIDI.writePacket(&event);
}

void sendControlChange(uint8_t controller, uint8_t value, uint8_t channel) {
  midiEventPacket_t event = {0x0B, (uint8_t)(0xB0 | (channel - 1)), controller, value};
  MIDI.writePacket(&event);
}

void sendProgramChange(uint8_t program, uint8_t channel) {
  midiEventPacket_t event = {0x0C, (uint8_t)(0xC0 | (channel - 1)), program, 0};
  MIDI.writePacket(&event);
}

void sendPitchBend(int16_t value, uint8_t channel) {
  uint16_t bendValue = (uint16_t)(value + 8192);
  uint8_t lsb = bendValue & 0x7F;
  uint8_t msb = (bendValue >> 7) & 0x7F;
  midiEventPacket_t event = {0x0E, (uint8_t)(0xE0 | (channel - 1)), lsb, msb};
  MIDI.writePacket(&event);
}

void sendChannelPressure(uint8_t pressure, uint8_t channel) {
  midiEventPacket_t event = {0x0D, (uint8_t)(0xD0 | (channel - 1)), pressure, 0};
  MIDI.writePacket(&event);
}

void sendPolyPressure(uint8_t note, uint8_t pressure, uint8_t channel) {
  midiEventPacket_t event = {0x0A, (uint8_t)(0xA0 | (channel - 1)), note, pressure};
  MIDI.writePacket(&event);
}

// ==================== BLE MIDI 数据解析 ====================
// 修复：使用新版 BLE 库的回调函数签名
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                           uint8_t* pData, size_t length, bool isNotify) {
  if (length < 3) return;

  Serial.printf("[MIDI] Received %d bytes: ", length);
  for (size_t i = 0; i < length && i < 10; i++) {
    Serial.printf("%02X ", pData[i]);
  }
  Serial.println();

  // 记录MIDI活动时间，用于LED显示
  lastMidiActivity = millis();
  setLED(COLOR_CYAN);

  size_t idx = 0;
  // 跳过 BLE MIDI 头部字节 (Header byte with timestamp high bits)
  if (pData[idx] & 0x80) idx++;

  while (idx < length) {
    // 跳过时间戳字节 (Timestamp byte)
    if (idx < length && (pData[idx] & 0x80)) {
      idx++;
      if (idx >= length) break;
    }

    uint8_t status = pData[idx];
    if (!(status & 0x80)) {
      // Running status: 使用上一个状态字节
      status = lastStatus;
    } else {
      lastStatus = status;
      idx++;
    }

    if (idx >= length) break;

    uint8_t msgType = status & 0xF0;
    uint8_t channel = (status & 0x0F) + 1;

    switch (msgType) {
      case 0x80:
        if (idx + 1 < length) {
          sendNoteOff(pData[idx] & 0x7F, pData[idx + 1] & 0x7F, channel);
          idx += 2;
        }
        break;

      case 0x90:
        if (idx + 1 < length) {
          uint8_t note = pData[idx] & 0x7F;
          uint8_t velocity = pData[idx + 1] & 0x7F;
          idx += 2;
          if (velocity == 0) sendNoteOff(note, 0, channel);
          else sendNoteOn(note, velocity, channel);
        }
        break;

      case 0xA0:
        if (idx + 1 < length) {
          sendPolyPressure(pData[idx] & 0x7F, pData[idx + 1] & 0x7F, channel);
          idx += 2;
        }
        break;

      case 0xB0:
        if (idx + 1 < length) {
          sendControlChange(pData[idx] & 0x7F, pData[idx + 1] & 0x7F, channel);
          idx += 2;
        }
        break;

      case 0xE0:
        if (idx + 1 < length) {
          uint8_t lsb = pData[idx] & 0x7F;
          uint8_t msb = pData[idx + 1] & 0x7F;
          idx += 2;
          sendPitchBend((int16_t)(((msb << 7) | lsb) - 8192), channel);
        }
        break;

      case 0xC0:
        if (idx < length) { sendProgramChange(pData[idx] & 0x7F, channel); idx++; }
        break;

      case 0xD0:
        if (idx < length) { sendChannelPressure(pData[idx] & 0x7F, channel); idx++; }
        break;

      default:
        idx++;
        break;
    }
  }
  // 不在这里切换回绿色，由 loop() 根据时间控制
}

// ==================== BLE 回调 ====================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("[BLE] onConnect callback");
  }
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    lastStatus = 0;
    setLED(COLOR_RED);
    Serial.println("[BLE] Disconnected!");
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String name = advertisedDevice.getName().c_str();
    if (name.length() > 0) Serial.printf("[SCAN] %s\n", name.c_str());

    if (name == TARGET_DEVICE_NAME ||
        (advertisedDevice.haveServiceUUID() &&
         advertisedDevice.isAdvertisingService(BLEUUID(MIDI_SERVICE_UUID)))) {
      Serial.println("[SCAN] Found target device!");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

// ==================== 连接服务器 ====================
bool connectToServer() {
  Serial.printf("[BLE] Connecting to %s...\n", myDevice->getAddress().toString().c_str());
  setLED(COLOR_PURPLE);

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial.println("[BLE] Failed to connect!");
    setLED(COLOR_RED);
    return false;
  }
  Serial.println("[BLE] Connected to server");

  // 设置较大MTU
  pClient->setMTU(256);

  BLERemoteService* pRemoteService = pClient->getService(BLEUUID(MIDI_SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.println("[BLE] Failed to find MIDI service!");
    pClient->disconnect();
    setLED(COLOR_RED);
    return false;
  }
  Serial.println("[BLE] Found MIDI service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(MIDI_CHARACTERISTIC_UUID));
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("[BLE] Failed to find MIDI characteristic!");
    pClient->disconnect();
    setLED(COLOR_RED);
    return false;
  }
  Serial.println("[BLE] Found MIDI characteristic");

  // ============ 修复：正确启用 Notify ============
  if (pRemoteCharacteristic->canNotify()) {
    Serial.println("[BLE] Characteristic can notify, subscribing...");

    // 方法1：使用 registerForNotify (兼容旧版库)
    // 第二个参数 true 表示启用通知 (写入 CCCD)
    pRemoteCharacteristic->registerForNotify(notifyCallback, true);

    // 方法2 (备选)：如果上面不工作，手动写入 CCCD 描述符
    // 获取 CCCD 描述符 (UUID: 0x2902) 并写入 0x0001 启用通知
    BLERemoteDescriptor* pCCCD = pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (pCCCD != nullptr) {
      uint8_t notifyOn[] = {0x01, 0x00};  // 启用 Notify
      pCCCD->writeValue(notifyOn, 2, true);
      Serial.println("[BLE] CCCD written to enable notifications");
    } else {
      Serial.println("[BLE] Warning: CCCD descriptor not found!");
    }

    Serial.println("[BLE] Notifications enabled");
  } else {
    Serial.println("[BLE] Warning: Characteristic cannot notify!");
  }

  connected = true;
  lastStatus = 0;
  setLED(COLOR_GREEN);
  Serial.println("[BLE] Connected! Ready for MIDI.");
  return true;
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(500);

  pixels.begin();
  pixels.setBrightness(50);
  setLED(COLOR_RED);

  Serial.println("\n=== ESP32-S3 BLE MIDI Receiver ===");
  Serial.printf("[MEM] Free heap: %d\n", ESP.getFreeHeap());

  MIDI.begin();
  USB.begin();

  // 初始化BLE，设置较低功耗以减少干扰
  BLEDevice::init("MIDI-RX");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setActiveScan(true);

  Serial.println("[BLE] Scanning...");
  setLED(COLOR_BLUE);
  pBLEScan->start(0, false);
}

// ==================== Loop ====================
void loop() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  static unsigned long lastReconnect = 0;

  if (doConnect) {
    delay(100);
    if (!connectToServer()) {
      delay(2000);
      doScan = true;
    }
    doConnect = false;
  }

  if (connected && (pClient == nullptr || !pClient->isConnected())) {
    connected = false;
    setLED(COLOR_RED);
    delay(1000);
    doScan = true;
  }

  if (doScan && millis() - lastReconnect > 3000) {
    lastReconnect = millis();
    setLED(COLOR_BLUE);
    if (myDevice) { delete myDevice; myDevice = nullptr; }
    BLEDevice::getScan()->start(0, false);
    doScan = false;
  }

  // 已连接时的LED控制：有MIDI活动显示青色，否则绿色
  if (connected) {
    if (millis() - lastMidiActivity < MIDI_LED_DURATION) {
      setLED(COLOR_CYAN);  // 有数据活动时显示青色
    } else {
      setLED(COLOR_GREEN);  // 空闲时显示绿色
    }
  }
  // 未连接时闪烁蓝色
  else if (millis() - lastBlink > 300) {
    ledState = !ledState;
    setLED(ledState ? COLOR_BLUE : COLOR_OFF);
    lastBlink = millis();
  }

  delay(10);
}
