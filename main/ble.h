#ifndef MAIN_BLE_H

#define MAIN_BLE_H

#include <memory>
#include "HIDKeyboardTypes.h"
#include "NimBLECharacteristic.h"
#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "NimBLEServer.h"

class BLEKeyboard {
 public:
  void init();
  // task to handle sending BLE HID keyboard events on separate core
  static void send_char_task(void* pvParameter);

  ~BLEKeyboard();

  void send_char(char c);

 private:
  NimBLEServer* server;
  std::unique_ptr<NimBLEHIDDevice> hid;
  NimBLECharacteristic* input;
  QueueHandle_t bleQueue;
  TaskHandle_t send_char_task_handle;
};

#endif
