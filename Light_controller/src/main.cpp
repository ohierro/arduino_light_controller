#include <esp_now.h>
#include <WiFi.h>
// #include <Arduino.h>

// // the number of the LED pin
const int ledPin = 13;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
// Structure example to receive data
// Must match the sender structure
typedef struct test_struct {
  int op;
  int value;
} test_struct;

//Create a struct_message called myData
test_struct myData;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  if (myData.op == 0) {
    Serial.print("Toggle on/off: ");
    Serial.println(myData.value);
  }
  if (myData.op == 1) {
    Serial.println("Change value to: ");
    Serial.println(myData.value);
    ledcWrite(ledChannel, myData.value * 255 / 100);
  }
  // Serial.println(len);
  // Serial.print("x: ");
  // Serial.println(myData.x);
  // Serial.print("y: ");
  // Serial.println(myData.y);
  // Serial.println();
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // light
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
}

void loop() {

}

// ------- CONTROL LIGHTS
// #include <Arduino.h>

// // the number of the LED pin
// const int ledPin = 16;  // 16 corresponds to GPIO16

// // setting PWM properties
// const int freq = 5000;
// const int ledChannel = 0;
// const int resolution = 8;

// void setup() {
//   // configure LED PWM functionalitites
//   ledcSetup(ledChannel, freq, resolution);

//   // attach the channel to the GPIO to be controlled
//   ledcAttachPin(ledPin, ledChannel);
// }

// void loop() {
//   // increase the LED brightness
//   for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){
//     // changing the LED brightness with PWM
//     ledcWrite(ledChannel, dutyCycle);
//     delay(15);
//   }

//   // decrease the LED brightness
//   for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
//     // changing the LED brightness with PWM
//     ledcWrite(ledChannel, dutyCycle);
//     delay(15);
//   }
// }



// ------------ GET MAC ADDRESS
// #ifdef ESP32
//   #include <WiFi.h>
// #else
//   #include <ESP8266WiFi.h>
// #endif

// void setup(){
//   Serial.begin(115200);
//   Serial.println();
//   Serial.print("ESP Board MAC Address:  ");
//   Serial.println(WiFi.macAddress());
// }

// void loop(){

// }