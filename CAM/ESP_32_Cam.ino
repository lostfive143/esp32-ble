#include <WiFi.h>
#include <esp_now.h>

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
// #include "soc/soc.h"           // Disable brownour problems
// #include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// VARS
unsigned long t = 0;
char message = 'd';
// Define variables to store incoming readings
float incomingWeight;
String incomingMessage;

/*_________________________________________ESP-NOW____________________*/
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x97, 0xE4, 0xB0}; // esp32-wroom

//Must match the receiver structure
typedef struct struct_message {
    float weight;
    char s;
} struct_message;

typedef struct struct_message2 {
    char Msg;
} struct_message2;

// Get readings
struct_message WeightReadings;

// Create Send messages
struct_message2 sentMessages;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&WeightReadings, incomingData, sizeof(WeightReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingWeight = WeightReadings.weight;
  Serial.print("Weight received: ");
  Serial.println(WeightReadings.weight);
  Serial.print("Char received: ");
  Serial.println(WeightReadings.s);

}
void printMessage(){
  // Display Readings in Serial Monitor
  Serial.println("ESP-32 recieved weight: ");
  Serial.print(incomingWeight);
  Serial.print(incomingMessage);
}
/*________________________________________________ESP-NOW_END____________________*/ 


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

/*____________________________________________________CAMERA____________________*/
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
/*____________________________________________________CAMERA_END___________________*/

/*____________________________________________________ESP-NOW____________________*/
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 // WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
/*____________________________________________________ESP-NOW_END____________________*/ 
/*____________________________________________________SSD____________________*/
  //Serial.println("Starting SD Card");
  if (!SD_MMC.begin()) { 
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
/*____________________________________________________SSD_END___________________*/
}
 
void loop() {
  // MESSAGES
    const int serialPrintInterval = 2000; //increase value to slow down serial print activity
    // Show weight
    //Serial.println(incomingWeight, 1);
    // send message
    if (millis() > t + serialPrintInterval) {
        sentMessages.Msg = message;
        esp_now_send(broadcastAddress, (uint8_t *) &sentMessages, sizeof(sentMessages));
        Serial.print("SENT MESSAGE: ");
        Serial.println(message);
      t = millis();
    }
  
}

/*_______________TakePhoto_____________*/
void takePhoto(String dateTime) {
  // Take Picture with Camera
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  // Save picture
  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(dateTime) + ".jpg";

  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());

  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
  }
  file.close();
  esp_camera_fb_return(fb);
}