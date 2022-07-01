/* ==========================================================
   ------------------------ MASTER --------------------------
   ==========================================================

   Master MAC: 24:0A:C4:EC:07:CC
   Slave  MAC: 24:0A:C4:EC:A6:F0

   ** MASTER RECEIVES LIDAR DATA FROM SLAVE AND PRINTS TO SERIAL **
*/

#include <Arduino.h>
#include <esp_now.h>
#include <Wifi.h>

uint8_t masterBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC };
uint8_t slaveBroadcastAddresss[] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0xF0 };

uint16_t incomingLidarReading;

typedef struct struct_message {
    uint16_t lidar_reading;
} struct_message;

struct_message incomingSensorReading;

esp_now_peer_info_t peerInfo;

// when this microcontroller sends a message, this function is triggered
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success":"Delivery Failure");
}

// when this microcontroller receives a message, this function is triggered
void OnDataReceive(const uint8_t* mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&incomingSensorReading, incomingData, sizeof(incomingSensorReading));
  Serial.print(" --> Number of Bytes Received: ");
  Serial.println(len);
  incomingLidarReading = incomingSensorReading.lidar_reading;
  Serial.print("\nIncoming LIDAR READING = ");
  Serial.println(incomingLidarReading);
}

void setup() {
  // 
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // station
  Serial.print("Hello, i'm Master, my MAC Address is: ");
  Serial.println(WiFi.macAddress());

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW!");
    for(;;); // just loop forever because there's no point going further
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, slaveBroadcastAddresss, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    for(;;); // just lay down and die
  }

  esp_now_register_recv_cb(OnDataReceive);
}

// MASTER
void loop() {
  // master won't have anything in its loop. It just sits and waits to receive data
}