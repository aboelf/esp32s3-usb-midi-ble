# ESP32-S3 Bluetooth MIDI Transceiver

一对蓝牙MIDI收发器，用于无线传输电吹管MIDI信号。

## 硬件要求
- 2x ESP32-S3开发板（带双USB Type-C接口）
- WS2812 RGB LED

## 项目结构
```
esp32s3-ble-midi/
├── tx/                 # 发送端 (USB Host MIDI → BLE MIDI)
│   ├── CMakeLists.txt
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   └── main.c
│   └── sdkconfig.defaults
├── rx/                 # 接收端 (BLE MIDI → USB Device MIDI)
│   ├── CMakeLists.txt
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   └── main.c
│   └── sdkconfig.defaults
└── README.md
```

## 编译和烧录

### 发送端
```bash
cd tx
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### 接收端
```bash
cd rx
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## LED状态指示
- 红色：未连接
- 蓝色闪烁：正在搜索/等待连接
- 绿色：已连接
- 青色闪烁：MIDI数据传输中

## 功能说明
- 发送端：作为USB Host连接电吹管，接收MIDI数据，通过BLE MIDI发送
- 接收端：接收BLE MIDI数据，作为USB MIDI设备输出到电脑
