#include <WiFi.h>
#include <esp_now.h>
#include <HX711_ADC.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// #include <EEPROM.h>
// const int calVal_eepromAdress = 0;

//  BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pCharacteristic2 = NULL;
#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "b4174651-4eac-4b0a-9fe4-776cb68036c2"
#define CHARACTERISTIC2_UUID "b4174652-4eac-4b0a-9fe4-776cb68036c2"
#define PASSKEY 999999
bool deviceConnected = false;
//pins:
const int HX711_dout = 15; //white
const int HX711_sck = 2; //green

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
unsigned long t = 0;
char weight[10]; 


/*____________________________________ESP_NOW_________________*/
char test = 't'; // must be '' and 1 char
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xFC, 0xF5, 0xC4, 0x0D, 0xD6, 0x48};
// Define variables to store incoming measage
String incomingMsg;

typedef struct struct_message {
    float weight;
    char s;
} struct_message;

struct_message WeightReadings;

typedef struct struct_message2 {
    char Msg;
} struct_message2;

struct_message2 incomingMessages;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessages, incomingData, sizeof(incomingMessages));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Message received: ");
  incomingMsg = incomingMessages.Msg;
  Serial.println(incomingMsg);
  Serial.println("__________________________");
}
/*____________________________________ESP_NOW_end_________________*/
void setup() {
  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");
  loadcellSetup();
  bleInit();
  // espNowSetup();

}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 1000; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;
  if (deviceConnected){
    // get smoothed value from the dataset:
    if (newDataReady) {
      if (millis() > t + serialPrintInterval) {
        float i = LoadCell.getData();
        Serial.print("Load_cell output val: ");
        Serial.println(i);
        newDataReady = 0;
        t = millis();
        sprintf(weight,"%4.2f",i);
        pCharacteristic->setValue(weight);
        pCharacteristic->notify();
        // WeightReadings.weight = i;
        // WeightReadings.s = test;
        // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &WeightReadings, sizeof(WeightReadings));
      }
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

void loadcellSetup(){
    LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 242.0; // uncomment this if you want to set the calibration value in the sketch
//#if defined(ESP8266)|| defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 
//#endif
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
}

void espNowSetup(){
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 // WiFi.begin();
 // WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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
}

class ServerCallback: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println(" - ServerCallback - onConnect");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println(" - ServerCallback - onDisconnect");
      deviceConnected = false;
    }
};

class Characteristic2Callbacks: public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      if (value == "t"){
        Serial.println("Tare initiated");
        LoadCell.tareNoDelay();
      }
      Serial.println("*********");
      Serial.print("New value: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(value[i]);
      }

      Serial.println();
      Serial.println("*********");
    }
  }
};

class SecurityCallback : public BLESecurityCallbacks {

  uint32_t onPassKeyRequest(){
    return 000000;
  }

  void onPassKeyNotify(uint32_t pass_key){}

  bool onConfirmPIN(uint32_t pass_key){
    vTaskDelay(5000);
    return true;
  }

  bool onSecurityRequest(){
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
    if(cmpl.success){
      Serial.println("   - SecurityCallback - Authentication Success");       
    }else{
      Serial.println("   - SecurityCallback - Authentication Failure*");
      pServer->removePeerDevice(pServer->getConnId(), true);
    }
    // BLEDevice::startAdvertising(); // more than 1 device
  }
};


void bleSecurity(){
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;          
  uint8_t key_size = 16;     
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = PASSKEY;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void bleInit(){
  BLEDevice::init("BLE-Secure-Server");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallback());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCharacteristic2 = pService->createCharacteristic(
                      CHARACTERISTIC2_UUID,
                      BLECharacteristic::PROPERTY_WRITE 
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic2->addDescriptor(new BLE2902());
  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCharacteristic2->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCharacteristic2->setCallbacks(new Characteristic2Callbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  bleSecurity();
}
