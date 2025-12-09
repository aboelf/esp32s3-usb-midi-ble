/*
 * ESP32-S3 BLE MIDI Receiver (接收端)
 * 功能：接收 BLE-MIDI-TX 发送的MIDI信号，通过USB Device发送给电脑
 *
 * 硬件：ESP32-S3 双USB口开发板 + WS2812 RGB LED
 *
 * 对接：main.c (BLE-MIDI-TX 发送端)
 *
 * 注意: 需要在 platformio.ini 中配置:
 *   build_flags = -DARDUINO_USB_MODE=0 -DARDUINO_USB_CDC_ON_BOOT=0
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
// 发送端设备名 (与 main.c 中的 DEVICE_NAME 一致)
#define TARGET_DEVICE_NAME "BLE-MIDI-TX"

// WS2812 LED配置
#define LED_PIN 48
#define LED_COUNT 1
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// LED颜色
#define COLOR_OFF pixels.Color(0, 0, 0)
#define COLOR_RED pixels.Color(50, 0, 0)
#define COLOR_GREEN pixels.Color(0, 50, 0)
#define COLOR_BLUE pixels.Color(0, 0, 50)
#define COLOR_YELLOW pixels.Color(50, 50, 0)
#define COLOR_PURPLE pixels.Color(50, 0, 50)
#define COLOR_CYAN pixels.Color(0, 50, 50)

void setLED(uint32_t color) {
  pixels.setPixelColor(0, color);
  pixels.show();
}

// ==================== BLE MIDI ====================
// BLE MIDI UUIDs (与 main.c 一致)
#define MIDI_SERVICE_UUID "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

// USB MIDI设备 (使用 ESP32-S3 TinyUSB)
USBMIDI MIDI;

// BLE相关
static bool doConnect = false;
static bool connected = false;
static bool doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice* myDevice = nullptr;
static BLEClient* pClient = nullptr;

// MIDI解析状态 (Running Status)
static uint8_t lastStatus = 0;

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
  // value: -8192 to 8191, 转换为 0-16383
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
/*
 * BLE MIDI 数据包格式 (来自 main.c 的 send_ble_midi):
 * [Header Byte] [Timestamp Byte] [MIDI Data...]
 *
 * Header Byte: (timestamp_high | 0x80)
 * Timestamp Byte: (timestamp_low | 0x80)
 * MIDI Data: 1-3字节的标准MIDI消息
 */
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                           uint8_t* pData, size_t length, bool isNotify) {
  if (length < 3) return;  // 最小: Header + Timestamp + 1字节MIDI

  // 闪烁LED表示接收数据
  setLED(COLOR_CYAN);

  // 调试输出
  Serial.printf("[RX] %d bytes: ", length);
  for (size_t i = 0; i < length; i++) {
    Serial.printf("%02X ", pData[i]);
  }
  Serial.println();

  // 解析BLE MIDI数据包
  size_t idx = 0;

  // 跳过 Header byte (最高位为1)
  if (pData[idx] & 0x80) {
    idx++;
  }

  while (idx < length) {
    // 跳过 Timestamp byte (最高位为1)
    if (idx < length && (pData[idx] & 0x80)) {
      idx++;
      if (idx >= length) break;
    }

    // 读取MIDI状态字节
    uint8_t status = pData[idx];

    // Running Status: 如果最高位为0，使用上一个状态字节
    if (!(status & 0x80)) {
      status = lastStatus;
    } else {
      lastStatus = status;
      idx++;
    }

    if (idx >= length) break;

    uint8_t msgType = status & 0xF0;
    uint8_t channel = (status & 0x0F) + 1;  // MIDI通道 1-16

    switch (msgType) {
      // 3字节消息
      case 0x80:  // Note Off
        if (idx + 1 < length) {
          uint8_t note = pData[idx] & 0x7F;
          uint8_t velocity = pData[idx + 1] & 0x7F;
          idx += 2;
          sendNoteOff(note, velocity, channel);
          Serial.printf("[MIDI] Ch%d Note Off: %d vel=%d\n", channel, note, velocity);
        }
        break;

      case 0x90:  // Note On
        if (idx + 1 < length) {
          uint8_t note = pData[idx] & 0x7F;
          uint8_t velocity = pData[idx + 1] & 0x7F;
          idx += 2;
          if (velocity == 0) {
            sendNoteOff(note, 0, channel);
            Serial.printf("[MIDI] Ch%d Note Off: %d\n", channel, note);
          } else {
            sendNoteOn(note, velocity, channel);
            Serial.printf("[MIDI] Ch%d Note On: %d vel=%d\n", channel, note, velocity);
          }
        }
        break;

      case 0xA0:  // Polyphonic Aftertouch
        if (idx + 1 < length) {
          uint8_t note = pData[idx] & 0x7F;
          uint8_t pressure = pData[idx + 1] & 0x7F;
          idx += 2;
          sendPolyPressure(note, pressure, channel);
          Serial.printf("[MIDI] Ch%d PolyAT: note=%d pressure=%d\n", channel, note, pressure);
        }
        break;

      case 0xB0:  // Control Change
        if (idx + 1 < length) {
          uint8_t controller = pData[idx] & 0x7F;
          uint8_t value = pData[idx + 1] & 0x7F;
          idx += 2;
          sendControlChange(controller, value, channel);
          Serial.printf("[MIDI] Ch%d CC: %d = %d\n", channel, controller, value);
        }
        break;

      case 0xE0:  // Pitch Bend
        if (idx + 1 < length) {
          uint8_t lsb = pData[idx] & 0x7F;
          uint8_t msb = pData[idx + 1] & 0x7F;
          idx += 2;
          int16_t pitchBendValue = (int16_t)(((msb << 7) | lsb) - 8192);
          sendPitchBend(pitchBendValue, channel);
          Serial.printf("[MIDI] Ch%d PitchBend: %d\n", channel, pitchBendValue);
        }
        break;

      // 2字节消息
      case 0xC0:  // Program Change
        if (idx < length) {
          uint8_t program = pData[idx] & 0x7F;
          idx++;
          sendProgramChange(program, channel);
          Serial.printf("[MIDI] Ch%d Program: %d\n", channel, program);
        }
        break;

      case 0xD0:  // Channel Pressure (Aftertouch)
        if (idx < length) {
          uint8_t pressure = pData[idx] & 0x7F;
          idx++;
          sendChannelPressure(pressure, channel);
          Serial.printf("[MIDI] Ch%d Pressure: %d\n", channel, pressure);
        }
        break;

      case 0xF0:  // System Messages
        // 跳过系统消息
        idx++;
        break;

      default:
        idx++;
        break;
    }
  }

  // 恢复绿色表示连接正常
  setLED(COLOR_GREEN);
}

// ==================== BLE 客户端回调 ====================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("[BLE] Connected callback");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    lastStatus = 0;  // 重置MIDI状态
    Serial.println("[BLE] Disconnected from BLE-MIDI-TX");
    setLED(COLOR_RED);
  }
};

// ==================== BLE 扫描回调 ====================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String name = advertisedDevice.getName().c_str();

    if (name.length() > 0) {
      Serial.printf("[SCAN] Found: %s", name.c_str());

      // 打印RSSI
      if (advertisedDevice.haveRSSI()) {
        Serial.printf(" (RSSI: %d)", advertisedDevice.getRSSI());
      }
      Serial.println();
    }

    // 方式1: 通过设备名匹配
    if (name == TARGET_DEVICE_NAME) {
      Serial.println("[SCAN] >>> Found BLE-MIDI-TX by name!");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
      return;
    }

    // 方式2: 通过 MIDI Service UUID 匹配
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(BLEUUID(MIDI_SERVICE_UUID))) {
      Serial.println("[SCAN] >>> Found BLE MIDI device by Service UUID!");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

// ==================== 连接到 BLE MIDI 服务器 ====================
bool connectToServer() {
  Serial.printf("[BLE] Connecting to %s (%s)...\n",
                myDevice->getName().c_str(),
                myDevice->getAddress().toString().c_str());
  setLED(COLOR_PURPLE);  // 紫色 = 连接中

  // 创建BLE客户端
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  // 连接到服务器
  if (!pClient->connect(myDevice)) {
    Serial.println("[BLE] Failed to connect");
    setLED(COLOR_RED);
    return false;
  }
  Serial.println("[BLE] Connected to server");

  // 设置MTU以获得更好的性能
  pClient->setMTU(128);  // 与 main.c 中的 esp_ble_gatt_set_local_mtu(128) 匹配

  // 获取MIDI服务
  BLERemoteService* pRemoteService = pClient->getService(BLEUUID(MIDI_SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.println("[BLE] Failed to find MIDI service");
    pClient->disconnect();
    setLED(COLOR_RED);
    return false;
  }
  Serial.println("[BLE] Found MIDI service");

  // 获取MIDI特征值
  pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(MIDI_CHARACTERISTIC_UUID));
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("[BLE] Failed to find MIDI characteristic");
    pClient->disconnect();
    setLED(COLOR_RED);
    return false;
  }
  Serial.println("[BLE] Found MIDI characteristic");

  // 注册通知回调 (接收MIDI数据)
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println("[BLE] Subscribed to MIDI notifications");
  } else {
    Serial.println("[BLE] WARNING: Characteristic doesn't support notify!");
  }

  connected = true;
  lastStatus = 0;  // 重置MIDI状态
  setLED(COLOR_GREEN);  // 绿色 = 已连接

  Serial.println("========================================");
  Serial.println("  Ready to receive MIDI from BLE-MIDI-TX");
  Serial.println("========================================");

  return true;
}

// ==================== 初始化 ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 初始化WS2812 LED
  pixels.begin();
  pixels.setBrightness(50);
  setLED(COLOR_RED);  // 红色 = 初始化中

  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESP32-S3 BLE MIDI Receiver");
  Serial.println("  Target: " TARGET_DEVICE_NAME);
  Serial.println("========================================");

  // 初始化USB MIDI设备 (ESP32-S3 TinyUSB)
  MIDI.begin();
  USB.begin();
  Serial.println("[USB] MIDI Device initialized");

  // 初始化BLE
  BLEDevice::init("MIDI-Receiver");

  // 配置并开始扫描
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);   // 扫描间隔 (ms)
  pBLEScan->setWindow(99);      // 扫描窗口 (ms)
  pBLEScan->setActiveScan(true);

  Serial.println("[BLE] Scanning for " TARGET_DEVICE_NAME "...");
  setLED(COLOR_BLUE);  // 蓝色 = 扫描中
  pBLEScan->start(0, false);  // 持续扫描直到找到设备
}

// ==================== 主循环 ====================
void loop() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  static unsigned long lastReconnect = 0;

  // 连接到发现的设备
  if (doConnect) {
    Serial.println("[BLE] Attempting connection...");
    delay(100);

    if (connectToServer()) {
      Serial.println("[BLE] Connection successful!");
    } else {
      Serial.println("[BLE] Connection failed, will retry...");
      delay(2000);
      doScan = true;
    }
    doConnect = false;
  }

  // 检查连接状态
  if (connected) {
    if (pClient == nullptr || !pClient->isConnected()) {
      connected = false;
      Serial.println("[BLE] Connection lost!");
      setLED(COLOR_RED);
      delay(1000);
      doScan = true;
    }
  }

  // 重新扫描
  if (doScan && millis() - lastReconnect > 3000) {
    lastReconnect = millis();
    Serial.println("[BLE] Restarting scan...");
    setLED(COLOR_BLUE);

    // 清理之前的设备
    if (myDevice != nullptr) {
      delete myDevice;
      myDevice = nullptr;
    }

    BLEDevice::getScan()->start(0, false);
    doScan = false;
  }

  // LED状态指示 (未连接时闪烁蓝色)
  if (!connected && millis() - lastBlink > 300) {
    ledState = !ledState;
    setLED(ledState ? COLOR_BLUE : COLOR_OFF);
    lastBlink = millis();
  }

  delay(10);
}
