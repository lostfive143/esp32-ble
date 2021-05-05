#include <WiFi.h>
#include <esp_now.h>
#include <HX711_ADC.h>
// #include <EEPROM.h>

//pins:
const int HX711_dout = 2; //white
const int HX711_sck = 14; //green

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// const int calVal_eepromAdress = 0;
unsigned long t = 0;
String test = "testnSTRIng";
float weight;

/*____________________________________ESP_NOW_________________*/
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xFC, 0xF5, 0xC4, 0x0D, 0xD6, 0x48};
// Define variables to store incoming measage
String incomingMsg;

typedef struct struct_message {
    float weight;
    String s;
} struct_message;

struct_message WeightReadings;

typedef struct struct_message2 {
    String Msg;
} struct_message2;

struct_message2 incomingMessages;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&WeightReadings, incomingData, sizeof(WeightReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingMsg = incomingMessages.Msg;
  Serial.println(incomingMsg);
}
void printMessage(){
  // Display Readings in Serial Monitor
  Serial.println();
  Serial.print("ESP32-WROOM GOT MESSAGE:   ");
  Serial.print(incomingMsg);
}
/*____________________________________ESP_NOW_end_________________*/
void setup() {
  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");
/*______________________Load cell_____________________________*/
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 242.0; // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266)|| defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); 
  unsigned long stabilizingtime = 4000; 
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
/*____________________________________________________________*/
/*___________________________ESP_NOW__________________________*/
// Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  //WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
 // esp_now_set_self_role(ESP_NOW_ROLE_COMBO); 
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
 //TODO
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  /*_______________________________ESP_NOW_end___________________________*/
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 300; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}
