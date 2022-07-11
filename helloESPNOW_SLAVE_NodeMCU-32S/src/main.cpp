/* ===============================================================
   ------------------------ PERIPHERAL ---------------------------
   ===============================================================

   Controller MAC: 24:0A:C4:EC:07:CC
   Peripheral MAC: 24:0A:C4:EC:A6:F0

   ** PERIPHERAL READS ACCELEROMETER DATA AND TRANSMITS TO CONTROLLER **
*/

#include <Arduino.h>
#include <esp_now.h>
#include <Wifi.h>
#include <Wire.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#define START_TRANSMITTING_BUTTON 15

MPU6050 ACCEL;

uint8_t controllerBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC }; // good one
//uint8_t controllerBroadcastAddress[] = { 0x30, 0xC6, 0xF7, 0x29, 0xBE, 0x68 }; // test unit
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

// These variables are for helping us track the time between each measurement
unsigned long count = 0;
unsigned long start_measurement_time = 0;
unsigned long end_measurement_time   = 0;

void readCurrentACCELEROMETERValue() {
  // ask the accelerometer for current readings
  //
  // (putting the "&" on the variable in the function call
  // is like giving the function a variable we created and
  // can read and saying "please put the result data here". This 
  // way we can keep the data in memory once the function 
  // exits)
  ACCEL.getMotion6(&incoming_accel_x_raw, 
                   &incoming_accel_y_raw, 
                   &incoming_accel_z_raw, 
                   &incoming_gyro_x_raw, 
                   &incoming_gyro_y_raw, 
                   &incoming_gyro_z_raw);

 ACCEL.getAcceleration(&incoming_accel_x_processed,
                       &incoming_accel_y_processed,
                       &incoming_accel_z_processed);

 ACCEL.getRotation(&incoming_gyro_x_processed,
                   &incoming_gyro_x_processed,
                   &incoming_gyro_x_processed);

  outgoingSensorReading.accel_x_raw  = incoming_accel_x_raw;
  outgoingSensorReading.accel_y_raw  = incoming_accel_y_raw;
  outgoingSensorReading.accel_z_raw  = incoming_accel_z_raw;
  outgoingSensorReading.gyro_x_raw   = incoming_gyro_x_raw;
  outgoingSensorReading.gyro_y_raw   = incoming_gyro_y_raw;
  outgoingSensorReading.gyro_z_raw   = incoming_gyro_z_raw;

  outgoingSensorReading.accel_x_proc = incoming_accel_x_processed;
  outgoingSensorReading.accel_y_proc = incoming_accel_y_processed;
  outgoingSensorReading.accel_z_proc = incoming_accel_z_processed;
  outgoingSensorReading.gyro_x_proc  = incoming_gyro_x_processed;
  outgoingSensorReading.gyro_y_proc  = incoming_gyro_y_processed;
  outgoingSensorReading.gyro_z_proc  = incoming_gyro_z_processed;
}

void sendCurrentACCELEROMETERValue() {
  // Send message via ESP-NOW
  end_measurement_time = millis();
  outgoingSensorReading.total_measurement_time = end_measurement_time - start_measurement_time;
  esp_err_t result = esp_now_send(controllerBroadcastAddress, (uint8_t *) &outgoingSensorReading, sizeof(outgoingSensorReading));
   
  if (result == ESP_OK) Serial.println("Sent with success");
  else Serial.println("Error sending the data");
}

void printAccelerometerDataNice() {
  // no need for peripheral to print
}

// when this microcontroller sends a message, this function is triggered
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success":"Delivery Failure");
}

// when this microcontroller receives a message, this function is triggered
void OnDataReceive(const uint8_t* mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&incomingSensorReading, incomingData, sizeof(incomingSensorReading));
  //Serial.print(" --> Number of Bytes Received: ");
  //Serial.println(len);
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
}

void setup() {
  // 
  pinMode(START_TRANSMITTING_BUTTON, INPUT_PULLUP);

  Serial.begin(38400);
  // start I2C communication
  Wire.begin();

  WiFi.mode(WIFI_STA); // station
  Serial.print("Hello, i'm peripheral, my MAC Address is: ");
  Serial.println(WiFi.macAddress());

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW!");
    for(;;); // just loop forever because there's no point going further
  }

  // tell the system to call the OnDataSent() function whenever data is transmitted
  esp_now_register_send_cb(OnDataSent);

  // tell this ESP who it will be talking to (the controller ESP32 which will print the Accelerometer reading out in this case)
  memcpy(peerInfo.peer_addr, controllerBroadcastAddress, 6); // ADDRESS OF THE EXPECTED RECEIVER!!
  peerInfo.channel = 0;     // default channel
  peerInfo.encrypt = false; // default encryption (none)

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    for(;;); // just lay down and die
  }

  esp_now_register_recv_cb(OnDataReceive);

  Serial.println("Initializing Accelerometer...");
  ACCEL.initialize();

  Serial.println("Testing Accelerometer Connection...");
  if(!ACCEL.testConnection()) {
    Serial.println("MPU6050 connection failed");
    for(;;);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  // TODO: figure out how to set settings on accelerometer

  Serial.println("Entering main loop and sleeping...");
  WiFi.setSleep(true);
}

// PERIPHERAL
unsigned long last_button_press_time = 0;
unsigned long button_debounce_time = 500; // 500 milliseconds
void loop() {
  bool button_pressed = false;
  while(!digitalRead(START_TRANSMITTING_BUTTON) && millis() - last_button_press_time >= button_debounce_time) {
    // read the accelerometer every X seconds and send packaged data to controller
    start_measurement_time = millis();
    readCurrentACCELEROMETERValue();
    //WiFi.setSleep(false); // try to not make the ESP32 overheat by only turning on WiFi when we need it
    sendCurrentACCELEROMETERValue();
    //WiFi.setSleep(true);
    
    button_pressed = true;

    delay(6); // this is in milliseconds. delaying the loop for 10ms *TOTAL* every iteration will 
              // yield ~100Hz measurement frequency. Sample rate is maxed at 4ms at the moment
              // NOTE: it has been tested that it takes about 4ms to take a reading from the
              // accelerometer, that's why it's delay(6) and not delay(10)
  }
  if(button_pressed) {
    last_button_press_time = millis();
  }
}
