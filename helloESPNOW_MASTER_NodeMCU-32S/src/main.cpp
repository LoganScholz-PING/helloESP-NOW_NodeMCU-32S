/* ==========================================================
   ------------------------ CONTROLLER --------------------------
   ==========================================================

   Controller MAC: 24:0A:C4:EC:07:CC
   Peripheral  MAC: 24:0A:C4:EC:A6:F0

   ** CONTROLLER RECEIVES ACCELEROMETER DATA FROM PERIPHERAL AND PRINTS TO SERIAL **
*/

#include <Arduino.h>
#include <esp_now.h>
#include <Wifi.h>
#include <SPI.h>
#include <Adafruit_MMA8451.h>

uint8_t controllerBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC };
uint8_t peripheralBroadcastAddresss[] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0xF0 };

// these variables are for holding any received data
// for use with ACCEL.getMotion6(..) function
int16_t incoming_accel_x_raw;
int16_t incoming_accel_y_raw;
int16_t incoming_accel_z_raw;
int16_t incoming_gyro_x_raw;
int16_t incoming_gyro_y_raw;
int16_t incoming_gyro_z_raw;
// for use with ACCEL.getAcceleration(..) function
int16_t incoming_accel_x_processed;
int16_t incoming_accel_y_processed;
int16_t incoming_accel_z_processed;
// for use with ACCEL.getRotation(..) function
int16_t incoming_gyro_x_processed;
int16_t incoming_gyro_y_processed;
int16_t incoming_gyro_z_processed;

typedef struct struct_accel_message {
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;

  int16_t accel_x_proc;
  int16_t accel_y_proc;
  int16_t accel_z_proc;
  int16_t gyro_x_proc;
  int16_t gyro_y_proc;
  int16_t gyro_z_proc;
  unsigned long total_measurement_time; // total time it took for 1 measurement (milliseconds)
} struct_accel_message;

struct_accel_message incomingSensorReading; // for controller code
struct_accel_message outgoingSensorReading; // for peripheral code

esp_now_peer_info_t peerInfo;

unsigned long last_message_received_time = 0;

// These variables are for helping us track the time between each measurement
unsigned long count = 0;
unsigned long total_time_delta = 0;

void printAccelerometerDataNice() {
  if(millis() - last_message_received_time >= 1000) count = 0;
  Serial.print(count++);
  Serial.print("-");
  Serial.print(incoming_accel_x_raw);
  Serial.print(",");
  Serial.print(incoming_accel_y_raw);
  Serial.print(",");
  Serial.print(incoming_accel_z_raw);
  Serial.print(",");
  Serial.print(incoming_gyro_x_raw);
  Serial.print(",");
  Serial.print(incoming_gyro_y_raw);
  Serial.print(",");
  Serial.print(incoming_gyro_z_raw);
  Serial.print(",");
  Serial.print(incoming_accel_x_processed);
  Serial.print(",");
  Serial.print(incoming_accel_y_processed);
  Serial.print(",");
  Serial.print(incoming_accel_z_processed);
  Serial.print(",");
  Serial.print(incoming_gyro_x_processed);
  Serial.print(",");
  Serial.print(incoming_gyro_y_processed);
  Serial.print(",");
  Serial.print(incoming_gyro_z_processed);
  Serial.print(",");
  Serial.println(total_time_delta);

  last_message_received_time = millis();
}

// when this microcontroller sends a message, this function is triggered
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success":"Delivery Failure");
}

// when this microcontroller receives a message, this function is triggered
void OnDataReceive(const uint8_t* mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&incomingSensorReading, incomingData, sizeof(incomingSensorReading));

  // now that the incomingSensorReading memory structure's values are filled  
  // in, move the data into local variables so we can do stuff with it
  incoming_accel_x_raw = incomingSensorReading.accel_x_raw;
  incoming_accel_y_raw = incomingSensorReading.accel_y_raw;
  incoming_accel_z_raw = incomingSensorReading.accel_z_raw;
  incoming_gyro_x_raw  = incomingSensorReading.gyro_x_raw;
  incoming_gyro_y_raw  = incomingSensorReading.gyro_y_raw;
  incoming_gyro_z_raw  = incomingSensorReading.gyro_z_raw;

  incoming_accel_x_processed = incomingSensorReading.accel_x_proc;
  incoming_accel_y_processed = incomingSensorReading.accel_y_proc;
  incoming_accel_z_processed = incomingSensorReading.accel_z_proc;
  incoming_gyro_x_processed = incomingSensorReading.gyro_x_proc;
  incoming_gyro_y_processed = incomingSensorReading.gyro_y_proc;
  incoming_gyro_z_processed = incomingSensorReading.gyro_z_proc;

  printAccelerometerDataNice();

  total_time_delta            = incomingSensorReading.total_measurement_time;

  // we've harvested all the data from the incoming data packet and stored 
  // it into local variables. 
  // Now print it nicely to the console so we can see it
  printAccelerometerDataNice();
}

void setup() {
  // 
  Serial.begin(38400);
  WiFi.mode(WIFI_STA); // station
  Serial.print("Hello, i'm Controller, my MAC Address is: ");
  Serial.println(WiFi.macAddress());

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW!");
    for(;;); // just loop forever because there's no point going further
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, peripheralBroadcastAddresss, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    for(;;); // just lay down and die
  }

  esp_now_register_recv_cb(OnDataReceive);

  Serial.println(F("\nFORMAT OF OUTPUT: [COUNT]-[AXR],[AYR],[AZR],[GXR],[GYR],[GZR],[AXP],[AYP],[AZP],[GXP],[GYP],[GZP],[MEASUREMENT TIME]\n"));
  Serial.println("...WAITING FOR DATA\n");
}

// controller
void loop() {
  // controller won't have anything in its loop. It just sits and waits to receive data
}