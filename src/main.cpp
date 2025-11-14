/*
  ESP32 BLE-MIDI Bridge
  Mode B (BLE MIDI) for STM32 synth controller
  - UART (31250) <--> BLE-MIDI (bidirectional)
  - For ESP32-WROOM-32 (no USB MIDI on ESP32)
  - RX_PIN, TX_PIN used for UART to STM32 hardware MIDI
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

const char* DEVICE_NAME = "VelarBridge-BLE";

// BLE MIDI UUIDs
static BLEUUID midiServiceUUID("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
static BLEUUID midiCharUUID("7772E5DB-3868-4112-A1A9-F2669D106BF3");

// UART to STM32
HardwareSerial MidiSerial(2); // use UART2 (Serial2) on pins RX2/TX2
const int RX_PIN = 16; // connect here STM32 TX -> ESP32 RX (GPIO16)
const int TX_PIN = 17; // connect here ESP32 TX -> STM32 RX (GPIO17)
const uint32_t MIDI_BAUD = 31250; // hardware MIDI baud

BLECharacteristic *pMidiCharacteristic;
BLEServer *pServer;
volatile bool bleConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    Serial.println("BLE client disconnected");
    // resume advertising
    delay(50);
    pServer->getAdvertising()->start();
  }
};

class MidiCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    std::string val = pChar->getValue();
    if (val.length() == 0) return;
    // BLE-MIDI packet format: first byte is timestamp (0x80 used here),
    // subsequent bytes are MIDI messages. We'll forward any >= 0x80 or <0x80 bytes as raw MIDI.
    for (size_t i = 0; i < val.length(); ++i) {
      uint8_t b = (uint8_t)val[i];
      // Skip timestamp bytes (they are >= 0x80 and designated by BLE-MIDI spec).
      // Simple rule: if byte equals 0x80 then skip, else forward.
      if (b == 0x80) continue;
      MidiSerial.write(b); // send to STM32 hardware MIDI
    }
  }
};

void setupBleMidi() {
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(midiServiceUUID);
  pMidiCharacteristic = pService->createCharacteristic(
    midiCharUUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  pMidiCharacteristic->addDescriptor(new BLE2902());
  pMidiCharacteristic->setCallbacks(new MidiCharCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(midiServiceUUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  Serial.println("BLE MIDI advertising started");
}

// Send raw MIDI bytes as BLE-MIDI notifications (prefix packets with timestamp byte 0x80)
void bleSendMidiBytes(const uint8_t *data, size_t len) {
  if (!bleConnected || pMidiCharacteristic == nullptr) return;

  // BLE ATT payload small — send in chunks
  const size_t CHUNK = 16; // payload per notification minus 1 for timestamp
  size_t pos = 0;
  while (pos < len) {
    size_t chunk = min(CHUNK, len - pos);
    // build payload: first byte timestamp 0x80 then MIDI bytes
    std::vector<uint8_t> payload;
    payload.reserve(chunk + 1);
    payload.push_back(0x80); // timestamp (simple)
    for (size_t i = 0; i < chunk; ++i) payload.push_back(data[pos + i]);
    pMidiCharacteristic->setValue(payload.data(), payload.size());
    pMidiCharacteristic->notify();
    pos += chunk;
    delay(2); // small gap for BLE stack
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("ESP32 BLE-MIDI Bridge starting...");

  // Init UART (Serial2)
  MidiSerial.begin(MIDI_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(20);

  setupBleMidi();
}

void loop() {
  // Read from UART (STM32) and forward to BLE MIDI
  while (MidiSerial.available()) {
    uint8_t b = MidiSerial.read();
    // Send single-byte MIDI messages via BLE (best-effort)
    bleSendMidiBytes(&b, 1);
  }
  // nothing else to do, BLE callbacks handle incoming writes
  delay(1);
}
