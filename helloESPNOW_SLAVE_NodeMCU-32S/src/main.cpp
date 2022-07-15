/* ===============================================================
   ------------------------ PERIPHERAL ---------------------------
   ===============================================================

   Controller MAC: 24:0A:C4:EC:07:CC
   Peripheral MAC: 24:0A:C4:EC:A6:F0

   ** PERIPHERAL READS ACCELEROMETER DATA AND TRANSMITS TO CONTROLLER **
*/

//#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include <SPI.h>
//#include "I2Cdev.h"
#include <Wire.h>
#include "MPU6050.h"


#define I2CDEV_SERIAL_DEBUG 1

#define START_TRANSMITTING_BUTTON 15

MPU6050 ACCEL; // hardcode I2C address = 0x68 or 0x69

//uint8_t controllerBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC }; // good one
uint8_t controllerBroadcastAddress[] = { 0x30, 0xC6, 0xF7, 0x29, 0xBE, 0x68 }; // test unit
uint8_t peripheralBroadcastAddresss[] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0xF0 };

// these variables are for holding any received data
// for use with ACCEL.getMotion6(..) function
float incoming_accel_x_raw;
float incoming_accel_y_raw;
float incoming_accel_z_raw;
float incoming_gyro_x_raw;
float incoming_gyro_y_raw;
float incoming_gyro_z_raw;
// for use with ACCEL.getAcceleration(..) function
float incoming_accel_x_processed;
float incoming_accel_y_processed;
float incoming_accel_z_processed;
// for use with ACCEL.getRotation(..) function
float incoming_gyro_x_processed;
float incoming_gyro_y_processed;
float incoming_gyro_z_processed;

typedef struct struct_accel_message {
  float accel_x_raw;
  float accel_y_raw;
  float accel_z_raw;
  float gyro_x_raw;
  float gyro_y_raw;
  float gyro_z_raw;

  float accel_x_proc;
  float accel_y_proc;
  float accel_z_proc;
  float gyro_x_proc;
  float gyro_y_proc;
  float gyro_z_proc;
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
  Vector rawAccel  = ACCEL.readRawAccel();
  Vector normAccel = ACCEL.readNormalizeAccel();
  Vector rawGyro   = ACCEL.readRawGyro();
  Vector normGyro  = ACCEL.readNormalizeGyro();

  // TODO: This is kinda gross, put the vectors in the struct and shuttle them
  // around that way..... eventually

  outgoingSensorReading.accel_x_raw  = rawAccel.XAxis;
  outgoingSensorReading.accel_y_raw  = rawAccel.YAxis;
  outgoingSensorReading.accel_z_raw  = rawAccel.ZAxis;

  outgoingSensorReading.accel_x_proc = normAccel.XAxis;
  outgoingSensorReading.accel_y_proc = normAccel.YAxis;
  outgoingSensorReading.accel_z_proc = normAccel.ZAxis;

  outgoingSensorReading.gyro_x_raw  = rawGyro.XAxis;
  outgoingSensorReading.gyro_y_raw  = rawGyro.YAxis;
  outgoingSensorReading.gyro_z_raw  = rawGyro.ZAxis;

  outgoingSensorReading.gyro_x_proc = normGyro.XAxis;
  outgoingSensorReading.gyro_y_proc = normGyro.YAxis;
  outgoingSensorReading.gyro_z_proc = normGyro.ZAxis;
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

void printAccelSettings() {
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(ACCEL.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(ACCEL.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(ACCEL.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(ACCEL.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(ACCEL.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(ACCEL.getAccelOffsetZ());
  
  Serial.println();
}

void setup() {
  // 
  pinMode(START_TRANSMITTING_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  // start I2C communication
  Wire.setPins(32, 33); // SDA SCL?
  Wire.begin();
  //Wire.begin((uint8_t)0x68, 32, 33); // address, SDA, SCL, frequency(not needed)
  Serial.print("I2C Timeout (ms): ");
  Serial.println(Wire.getTimeOut(), DEC);

  Serial.println("Initializing Accelerometer...");
  while(!ACCEL.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G)) {
    Wire.clearWriteError();
    Serial.println("Could not find accelerometer. Check wiring?");
    delay(500);
  }

  printAccelSettings();

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
