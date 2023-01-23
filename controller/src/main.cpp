/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>

#define ROT_CLK	4
#define ROT_DT	2
#define ROT_SW	5

#define ON	1
#define OFF	0

ESP32Encoder encoder;

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
// uint8_t broadcastAddress1[] = {0x08, 0x3A, 0xF2, 0x7D, 0x0F, 0xFC};
uint8_t broadcastAddress1[] = {0x24, 0x62, 0xAB, 0xFC, 0x6C, 0xCC};
// uint8_t broadcastAddress2[] = {0xFF, , , , , };
// uint8_t broadcastAddress3[] = {0xFF, , , , , };

typedef struct test_struct {
  int op;
  int value;
} test_struct;

test_struct test;
int currentValue;
int currentState = ON;

esp_now_peer_info_t peerInfo;

void setupEncoder();
void toggleLight();
void sendValue();

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer
  // memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }
  // /// register third peer
  // memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }

	setupEncoder();
}

void loop() {
  // test.x = random(0,20);

	static int isClicking = false;
	int readValue = digitalRead(ROT_SW);
	if (readValue == LOW && !isClicking) {
		// Serial.print("Click");
		toggleLight();
		isClicking = true;
	} else {
		if (readValue == HIGH) {
			isClicking = false;
		}
	}
	test.op = 1;
	test.value = (int32_t)encoder.getCount();

	int encoderValue = encoder.getCount();
	if (encoderValue > 100) {
		encoderValue = 100;
	}
	if (encoderValue < 0) {
		encoderValue  = 0;
	}
	if (encoderValue != currentValue) {
		currentValue = encoderValue;
		sendValue();

		// test.op = 1;
		// test.value = encoderValue;
		// esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

		// if (result == ESP_OK) {
		// 	Serial.println("Sent with success");
		// }
		// else {
		// 	Serial.println("Error sending the data");
		// }
	}
  // test.y = random(0,20);

  // esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  delay(100);
}

void setupEncoder() {
	//ESP32Encoder::useInternalWeakPullResistors=DOWN;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;

	// use pin 19 and 18 for the first encoder
	encoder.attachHalfQuad(ROT_CLK, ROT_DT);
	// use pin 17 and 16 for the second encoder
	// encoder2.attachHalfQuad(17, 16);
  pinMode(ROT_SW, INPUT);

	// set starting count value after attaching
	encoder.setCount(70);

	// clear the encoder's raw count and set the tracked count to zero
	// encoder2.clearCount();
	Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
}

void toggleLight() {
	if (currentState == ON) {
		currentState = OFF;
	} else {
		currentState = ON;
	}

	test.op = 0;
	test.value = currentState;
	esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

	if (result == ESP_OK) {
		Serial.println("Sent with success");
	}
	else {
		Serial.println("Error sending the data");
	}
}

void sendValue() {
	test.op = 1;
	test.value = currentValue;
	esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

	if (result == ESP_OK) {
		Serial.println("Sent with success");
	}
	else {
		Serial.println("Error sending the data");
	}
}

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



// #include <ESP32Encoder.h>

// ESP32Encoder encoder;
// ESP32Encoder encoder2;

// timer and flag for example, not needed for encoders
// unsigned long encoder2lastToggled;
// bool encoder2Paused = false;

// void setup(){

// 	Serial.begin(9600);
// 	// Enable the weak pull down resistors

// 	//ESP32Encoder::useInternalWeakPullResistors=DOWN;
// 	// Enable the weak pull up resistors
// 	ESP32Encoder::useInternalWeakPullResistors=UP;

// 	// use pin 19 and 18 for the first encoder
// 	encoder.attachHalfQuad(4, 2);
// 	// use pin 17 and 16 for the second encoder
// 	// encoder2.attachHalfQuad(17, 16);
//   pinMode(5, INPUT);

// 	// set starting count value after attaching
// 	encoder.setCount(37);

// 	// clear the encoder's raw count and set the tracked count to zero
// 	// encoder2.clearCount();
// 	Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
// 	// set the lastToggle
// 	// encoder2lastToggled = millis();
// }

// void loop(){
// 	// Loop and read the count
// 	// Serial.println("Encoder count = " + String((int32_t)encoder.getCount()) + " " + String((int32_t)encoder2.getCount()));
//   Serial.println("Encoder count = " + String((int32_t)encoder.getCount()));
//   // let digitalRead(5)
// 	delay(100);

// 	// every 5 seconds toggle encoder 2
// 	// if (millis() - encoder2lastToggled >= 5000) {
// 	// 	if(encoder2Paused) {
// 	// 		Serial.println("Resuming Encoder 2");
// 	// 		encoder2.resumeCount();
// 	// 	} else {
// 	// 		Serial.println("Paused Encoder 2");
// 	// 		encoder2.pauseCount();
// 	// 	}

// 	// 	encoder2Paused = !encoder2Paused;
// 	// 	encoder2lastToggled = millis();
// 	// }
// }

