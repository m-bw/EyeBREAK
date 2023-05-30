#include <string>
#undef EPS
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"
#define EPS 192

#include <string.h>
#include "ble.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "jpeg_decoder.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "model_data.cpp"
#include "morse.h"
#include "nvs_flash.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#define EXAMPLE_ESP_WIFI_SSID    "EyeBREAK"
#define EXAMPLE_ESP_WIFI_PASS    ""
#define EXAMPLE_ESP_WIFI_CHANNEL 6
#define EXAMPLE_MAX_STA_CONN     1

static const char* TAG = "eyebreak";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* model_input = nullptr;

constexpr int kTensorArenaSize = 8 * 1024;
static uint8_t* tensor_arena;

extern long long softmax_total_time;
extern long long conv_total_time;
extern long long fc_total_time;
extern long long pooling_total_time;
extern long long add_total_time;
extern long long mul_total_time;

static BLEKeyboard bleKeyboard;

// see https://github.com/espressif/esp-who/blob/master/components/modules/camera/who_camera.c

#define CAM_MODULE_NAME "ESP-EYE"
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    4
#define CAM_PIN_SIOD    18
#define CAM_PIN_SIOC    23

#define CAM_PIN_D7    36
#define CAM_PIN_D6    37
#define CAM_PIN_D5    38
#define CAM_PIN_D4    39
#define CAM_PIN_D3    35
#define CAM_PIN_D2    14
#define CAM_PIN_D1    13
#define CAM_PIN_D0    34
#define CAM_PIN_VSYNC 5
#define CAM_PIN_HREF  27
#define CAM_PIN_PCLK  25

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,  // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_96X96,        // QVGA = 320x240, CIF = 400x296, HVGA = 480x320

    .jpeg_quality = 12,  // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2,       // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera() {
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
  }
  return err;
}

// see https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/softAP/main/softap_example_main.c
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
    ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
    ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
  }
}

void wifi_init_softap() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(
      esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));

  wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .pmf_cfg = {
                .required = true,
            }
        },
    };
  if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", EXAMPLE_ESP_WIFI_SSID,
           EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t* req) {
  camera_fb_t* fb = nullptr;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t* _jpg_buf;
  char* part_buf[64];
  static int64_t last_frame = 0;
  float p_open = 1.0;
  bool is_open = true;
  std::string output{};
  std::string morse_char{};

  MorseCode morse{};

  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    last_frame = esp_timer_get_time();
    fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGE(TAG, "Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    int64_t fr_done = esp_timer_get_time();
    int64_t frame_done = (fr_done - last_frame) / 1000;

    int64_t start_time = fr_done;

    cv::Mat cv_buf = cv::Mat(96, 96, CV_8U, fb->buf);
    // NOTE: don't look at the buffer contents, might be overwritten during inference
    cv::Mat final_buf = cv::Mat(36, 36, CV_8U, model_input->data.data);
    // TODO can resize in Tflite instead
    cv::resize(cv_buf, final_buf, cv::Size(36, 36), cv::INTER_NEAREST);
    // convert to int8
    for (int i = 0; i < final_buf.total(); i++) {
      final_buf.data[i] ^= 0x80;
    }

    if (kTfLiteOk != interpreter->Invoke()) {
      ESP_LOGE(TAG, "Invoke failed.");
    }

    TfLiteTensor* model_output = interpreter->output(0);
    int8_t eye_score = model_output->data.int8[0];

    p_open = (eye_score - model_output->params.zero_point) * model_output->params.scale;
    // ESP_LOGI(TAG, "score: %d, zero point: %"PRIi32", scale: %f", eye_score, model_output->params.zero_point,
    // model_output->params.scale);

    is_open = p_open > 0.5;

    auto [next_morse, next_char] = morse.Update(!is_open, start_time);
    char next_morse_char = next_morse == Morse::Dot ? '.' : '-';
    int last_morse_length = morse_char.size();
    bool delete_last_char = false;
    if (next_morse != Morse::Invalid) {
      morse_char += next_morse_char;
    } else if (next_char == '\b') {
      if (!output.empty()) {
        output.pop_back();
        delete_last_char = true;
      }
      morse_char.clear();
    } else if (next_char != '\0') {
      output += next_char;
      morse_char.clear();
    }

    int64_t fr_inf = esp_timer_get_time();
    int64_t frame_inf = (fr_inf - fr_done) / 1000;

    // need to pass an lvalue to xQueueSend
    char del_char = '\b';
    // if in progress, write out the dots and dashes of the current character
    if (next_morse != Morse::Invalid) {
      bleKeyboard.send_char(next_morse_char);
    } else if (next_char == '\b') {
      for (int i = 0; i < last_morse_length; i++) {
        bleKeyboard.send_char(del_char);
      }
      if (delete_last_char) {
        bleKeyboard.send_char(del_char);
      }
    } else if (next_char != '\0') {
      for (int i = 0; i < last_morse_length; i++) {
        bleKeyboard.send_char(del_char);
      }
      bleKeyboard.send_char(next_char);
    }

    if (!output.empty()) {
      cv::putText(cv_buf, output, cv::Point(0, 96), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));
    }

    if (output.size() > 10) {
      output.clear();
    }

    if (!morse_char.empty()) {
      cv::putText(cv_buf, morse_char, cv::Point(48, 10), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));
    }

    cv::putText(cv_buf, is_open ? "open" : "close", cv::Point(0, 10), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));

    bool jpeg_converted = frame2jpg(fb, 60, &_jpg_buf, &_jpg_buf_len);
    if (!jpeg_converted) {
      ESP_LOGE(TAG, "JPEG compression failed");
      esp_camera_fb_return(fb);
      res = ESP_FAIL;
    }

    int64_t fr_enc = esp_timer_get_time();
    int64_t frame_enc = (fr_enc - fr_inf) / 1000;

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char*) part_buf, 64, _STREAM_PART, _jpg_buf_len);

      res = httpd_resp_send_chunk(req, (const char*) part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*) _jpg_buf, _jpg_buf_len);
    }
    if (fb->format != PIXFORMAT_JPEG) {
      free(_jpg_buf);
    }
    esp_camera_fb_return(fb);
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    // ESP_LOGI(TAG, "NN: FC %lld conv %lld pool %lld add %lld mul %lld\n", fc_total_time / 1000, conv_total_time /
    // 1000,
    //         pooling_total_time / 1000, add_total_time / 1000, mul_total_time / 1000);
    ESP_LOGI(TAG,
             "c%d \"%s\" %s (%.2f) capture: %" PRIu32 "ms encoding: %" PRIu32 "ms infer: %" PRIu32 "ms total: %" PRIu32
             "ms (%.1ffps)",
             esp_cpu_get_core_id(), output.c_str(), is_open ? "OPEN" : "CLOSED", p_open, (uint32_t) frame_done,
             (uint32_t) frame_enc, (uint32_t) frame_inf, (uint32_t) frame_time, 1000.0 / (uint32_t) frame_time);
  }

  last_frame = 0;
  return res;
}

httpd_uri_t uri_get = {.uri = "/", .method = HTTP_GET, .handler = jpg_stream_httpd_handler, .user_ctx = nullptr};

httpd_handle_t start_webserver() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 20000;

  httpd_handle_t server = nullptr;

  esp_err_t err = httpd_start(&server, &config);
  if (err == ESP_OK) {
    httpd_register_uri_handler(server, &uri_get);
  } else {
    ESP_LOGE(TAG, "failed to start server: %s", esp_err_to_name(err));
  }
  return server;
}

static inline void print_meminfo() {
  ESP_LOGI(TAG, "Heap free: %d PSRAM, %d internal", heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM),
           heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
}

extern "C" void app_main() {
  // allow usage of pins 13 and 14 as inputs (used for ESP-EYE camera)
  gpio_config_t conf;
  conf.mode = GPIO_MODE_INPUT;
  conf.pull_up_en = GPIO_PULLUP_ENABLE;
  conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  conf.intr_type = GPIO_INTR_DISABLE;
  conf.pin_bit_mask = 1LL << 13;
  gpio_config(&conf);
  conf.pin_bit_mask = 1LL << 14;
  gpio_config(&conf);
  ESP_LOGI(TAG, "Initializing camera");
  if (ESP_OK != init_camera()) {
    return;
  }

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  bleKeyboard.init();
  ESP_LOGD(TAG, "Advertising started!");

  ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
  wifi_init_softap();

  httpd_handle_t handle = start_webserver();
  if (!handle) {
    return;
  }

  model = tflite::GetModel(__10k_cnn_i8_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "Model provided is schema version %" PRIu32 " not equal to supported version %d.", model->version(),
             TFLITE_SCHEMA_VERSION);
    return;
  }

  if (tensor_arena == nullptr) {
    tensor_arena = (uint8_t*) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (tensor_arena == nullptr) {
    ESP_LOGE(TAG, "Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
    return;
  }

  static tflite::MicroMutableOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddTranspose();
  micro_op_resolver.AddMaxPool2D();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddLogistic();

  // Build an interpreter to run the model with.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroInterpreter static_interpreter(model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    ESP_LOGE(TAG, "AllocateTensors() failed");
    return;
  }

  // Get information about the memory area to use for the model's input.
  model_input = interpreter->input(0);
  std::string dims{};
  for (int i = 0; i < model_input->dims->size; i++) {
    dims += std::to_string(model_input->dims->data[i]);
    if (i < model_input->dims->size - 1) {
      dims += 'x';
    }
  }
  ESP_LOGI(TAG, "model input: %s %s (%d)", TfLiteTypeGetName(model_input->type), dims.c_str(), model_input->bytes);

  print_meminfo();
}
