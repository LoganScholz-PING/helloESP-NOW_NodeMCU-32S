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
#include <SPI.h>
#include <Adafruit_MMA8451.h>

uint8_t masterBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC };
uint8_t slaveBroadcastAddresss[] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0xF0 };

// these variables are for holding any received data (if code is uploaded to master unit)
uint16_t incoming_accel_x_raw;
uint16_t incoming_accel_y_raw;
uint16_t incoming_accel_z_raw;
float    incoming_accel_x_calculated;     // m/s^2
float    incoming_accel_y_calculated;     // m/s^2
float    incoming_accel_z_calculated;     // m/s^2
uint8_t  incoming_accel_orientation_enum;

typedef struct struct_accel_message {
  uint16_t accel_x_raw;
  uint16_t accel_y_raw;
  uint16_t accel_z_raw;
  float    accel_x_calculated;     // m/s^2
  float    accel_y_calculated;     // m/s^2
  float    accel_z_calculated;     // m/s^2
  uint8_t  accel_orientation_enum; // this is an enumerated value (every different value means something specific)
} struct_accel_message;

struct_accel_message incomingSensorReading;
struct_accel_message outgoingSensorReading;

esp_now_peer_info_t peerInfo;

void interpretOrientationEnum(uint8_t orientation) {
  Serial.print(" ----> ORIENTATION: ");
  switch (orientation) {
    case MMA8451_PL_PUF: 
      Serial.println("Portrait Up Front");
      break;
    case MMA8451_PL_PUB: 
      Serial.println("Portrait Up Back");
      break;    
    case MMA8451_PL_PDF: 
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB: 
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF: 
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB: 
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF: 
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB: 
      Serial.println("Landscape Left Back");
      break;
    }
}

void printAccelerometerDataNice() {
  Serial.println("\n=================== [RECEIVING DATA (MASTER SIDE)] ===================");
  interpretOrientationEnum(incoming_accel_orientation_enum); // print the meaning of the incoming orientation enumeration
  Serial.print(" ----> X_RAW: "); Serial.print(incoming_accel_x_raw); Serial.print(" (X calculated = "); Serial.print(incoming_accel_x_calculated); Serial.println(")");
  Serial.print(" ----> Y_RAW: "); Serial.print(incoming_accel_y_raw); Serial.print(" (Y calculated = "); Serial.print(incoming_accel_y_calculated); Serial.println(")");
  Serial.print(" ----> Z_RAW: "); Serial.print(incoming_accel_z_raw); Serial.print(" (Z calculated = "); Serial.print(incoming_accel_z_calculated); Serial.println(")");
}

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

  // now that the incomingSensorReading memory structure's values are filled  
  // in, move the data into local variables so we can do stuff with it
  incoming_accel_x_raw = incomingSensorReading.accel_x_raw;
  incoming_accel_y_raw = incomingSensorReading.accel_y_raw;
  incoming_accel_z_raw = incomingSensorReading.accel_z_raw;
  
  incoming_accel_x_calculated = incomingSensorReading.accel_x_calculated;
  incoming_accel_y_calculated = incomingSensorReading.accel_y_calculated;
  incoming_accel_z_calculated = incomingSensorReading.accel_z_calculated;
  
  incoming_accel_orientation_enum = incomingSensorReading.accel_orientation_enum;

  // we've harvested all the data from the incoming data packet and stored 
  // it into local variables. 
  // Now print it nicely to the console so we can see it
  printAccelerometerDataNice();
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
