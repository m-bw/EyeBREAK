#include "ble.h"

#define KEYBOARD_ID 1

static const uint8_t _hidReportDescriptor[] = {
    USAGE_PAGE(1), 0x01,  // USAGE_PAGE (Generic Desktop Ctrls)
    USAGE(1), 0x06,       // USAGE (Keyboard)
    COLLECTION(1), 0x01,  // COLLECTION (Application)
    // ------------------------------------------------- Keyboard
    REPORT_ID(1), KEYBOARD_ID,  //   REPORT_ID (1)
    USAGE_PAGE(1), 0x07,        //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1), 0xE0,     //   USAGE_MINIMUM (0xE0)
    USAGE_MAXIMUM(1), 0xE7,     //   USAGE_MAXIMUM (0xE7)
    LOGICAL_MINIMUM(1), 0x00,   //   LOGICAL_MINIMUM (0)
    LOGICAL_MAXIMUM(1), 0x01,   //   Logical Maximum (1)
    REPORT_SIZE(1), 0x01,       //   REPORT_SIZE (1)
    REPORT_COUNT(1), 0x08,      //   REPORT_COUNT (8)
    HIDINPUT(1), 0x02,          //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    REPORT_COUNT(1), 0x01,      //   REPORT_COUNT (1) ; 1 byte (Reserved)
    REPORT_SIZE(1), 0x08,       //   REPORT_SIZE (8)
    HIDINPUT(1), 0x01,          //   INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    REPORT_COUNT(1), 0x05,      //   REPORT_COUNT (5) ; 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1), 0x01,       //   REPORT_SIZE (1)
    USAGE_PAGE(1), 0x08,        //   USAGE_PAGE (LEDs)
    USAGE_MINIMUM(1), 0x01,     //   USAGE_MINIMUM (0x01) ; Num Lock
    USAGE_MAXIMUM(1), 0x05,     //   USAGE_MAXIMUM (0x05) ; Kana
    HIDOUTPUT(1), 0x02,         //   OUTPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    REPORT_COUNT(1), 0x01,      //   REPORT_COUNT (1) ; 3 bits (Padding)
    REPORT_SIZE(1), 0x03,       //   REPORT_SIZE (3)
    HIDOUTPUT(1), 0x01,     //   OUTPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    REPORT_COUNT(1), 0x06,  //   REPORT_COUNT (6) ; 6 bytes (Keys)
    REPORT_SIZE(1), 0x08,   //   REPORT_SIZE(8)
    LOGICAL_MINIMUM(1), 0x00,  //   LOGICAL_MINIMUM(0)
    LOGICAL_MAXIMUM(1), 0x65,  //   LOGICAL_MAXIMUM(0x65) ; 101 keys
    USAGE_PAGE(1), 0x07,       //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1), 0x00,    //   USAGE_MINIMUM (0)
    USAGE_MAXIMUM(1), 0x65,    //   USAGE_MAXIMUM (0x65)
    HIDINPUT(1), 0x00,         //   INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    END_COLLECTION(0),         // END_COLLECTION
};

void BLEKeyboard::init() {
  NimBLEDevice::init("EyeBREAK");
  NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_SC);

  server = NimBLEDevice::createServer();

  hid = std::make_unique<NimBLEHIDDevice>(server);
  input = hid->inputReport(KEYBOARD_ID);  // <-- input REPORTID from report map
  // technically don't need any of the other stuff

  hid->hidInfo(0x00, 0x01);

  hid->reportMap(const_cast<uint8_t*>(_hidReportDescriptor), sizeof(_hidReportDescriptor));
  hid->startServices();

  bleQueue = xQueueCreate(10, sizeof(char));
  // pin ble to core 1
  xTaskCreatePinnedToCore(send_char_task, "send_char_task", 2048, this, tskIDLE_PRIORITY, &send_char_task_handle, 1);

  NimBLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAppearance(HID_KEYBOARD);
  pAdvertising->addServiceUUID(hid->hidService()->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
}

void BLEKeyboard::send_char_task(void* pvParameter) {
  auto self = static_cast<BLEKeyboard*>(pvParameter);
  char c{};
  while (true) {
    if (xQueueReceive(self->bleQueue, &c, 0)) {
      // if (server->getConnectedCount() > 0) {
      KEYMAP key = keymap[static_cast<int>(c)];
      uint8_t key_report[8] = {key.modifier, 0, key.usage, 0, 0, 0, 0, 0};
      uint8_t stop_report[8] = {0};

      self->input->setValue(key_report, sizeof(key_report));
      self->input->notify();
      vTaskDelay(5 / portTICK_PERIOD_MS);

      // immediately send another report that stops the char input
      self->input->setValue(stop_report, sizeof(stop_report));
      self->input->notify();
      //}
    }
  }
}

BLEKeyboard::~BLEKeyboard() {
  vTaskDelete(send_char_task_handle);
  NimBLEDevice::deinit(true);
}

void BLEKeyboard::send_char(char c) { xQueueSend(bleQueue, static_cast<void*>(&c), 10); }
