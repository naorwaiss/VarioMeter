#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_bt_device.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
uint8_t mac[6];

// Make it global so it's accessible in both setup() and loop()
BLECharacteristic *pCharacteristic;

void getMAC(uint8_t *pMacAddress) {
    const uint8_t *mac = esp_bt_dev_get_address();
    memcpy(pMacAddress, mac, 6);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("XIAO_ESP32C6");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY  // Add NOTIFY
                    );

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Characteristic defined! Now you can read it in your phone!");
  getMAC(mac);
}

void loop() {
  Serial.println("MAC: " + macToString(mac));


  static int counter = 0;
  String dataToSend = "Counter: " + String(counter++);
  
  Serial.println("Sending: " + dataToSend);
  pCharacteristic->setValue(dataToSend.c_str());  // Set new value
  pCharacteristic->notify();                      // Notify connected client

  delay(100);
}

String macToString(const uint8_t *mac) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}


