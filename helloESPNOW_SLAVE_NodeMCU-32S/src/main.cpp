/* ==========================================================
   ------------------------ SLAVE ---------------------------
   ==========================================================

   Master MAC: 24:0A:C4:EC:07:CC
   Slave  MAC: 24:0A:C4:EC:A6:F0

   ** SLAVE READS LIDAR DATA AND TRANSMITS TO MASTER **
*/

#include <Arduino.h>
#include <esp_now.h>
#include <Wifi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 ACCEL = Adafruit_MMA8451();

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
  unsigned long total_measurement_time; // total time it took for 1 measurement
} struct_accel_message;

struct_accel_message incomingSensorReading;
struct_accel_message outgoingSensorReading;

esp_now_peer_info_t peerInfo;

// These variables are for helping us track the time between each measurement
unsigned long count = 0;
unsigned long start_measurement_time = 0;
unsigned long end_measurement_time   = 0;

void interpretOrientationEnum(uint8_t ori) {
  Serial.print(" --> ORIENTATION: ");
  switch (ori) {
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

void readCurrentACCELEROMETERValue() {
  ACCEL.read();
  outgoingSensorReading.accel_x_raw = ACCEL.x;
  outgoingSensorReading.accel_y_raw = ACCEL.y;
  outgoingSensorReading.accel_z_raw = ACCEL.z;

  sensors_event_t event;
  ACCEL.getEvent(&event);

  outgoingSensorReading.accel_x_calculated = event.acceleration.x;
  outgoingSensorReading.accel_y_calculated = event.acceleration.y;
  outgoingSensorReading.accel_z_calculated = event.acceleration.z;

  outgoingSensorReading.accel_orientation_enum = ACCEL.getOrientation();
}

void sendCurrentACCELEROMETERValue() {
  // Send message via ESP-NOW
  end_measurement_time = micros();
  outgoingSensorReading.total_measurement_time = end_measurement_time - start_measurement_time;
  esp_err_t result = esp_now_send(masterBroadcastAddress, (uint8_t *) &outgoingSensorReading, sizeof(outgoingSensorReading));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void printAccelerometerDataNice() {

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
  incoming_accel_x_raw = incomingSensorReading.accel_x_raw;
  incoming_accel_y_raw = incomingSensorReading.accel_y_raw;
  incoming_accel_z_raw = incomingSensorReading.accel_z_raw;
  
  incoming_accel_x_calculated = incomingSensorReading.accel_x_calculated;
  incoming_accel_y_calculated = incomingSensorReading.accel_y_calculated;
  incoming_accel_z_calculated = incomingSensorReading.accel_z_calculated;
  
  incoming_accel_orientation_enum = incomingSensorReading.accel_orientation_enum;
  
  interpretOrientationEnum(incoming_accel_orientation_enum); // print the meaning of the incoming orientation enumeration
  
  printAccelerometerDataNice();
}

void setup() {
  // 
  Serial.begin(115200);
  // start I2C communication
  Wire.begin();

  WiFi.mode(WIFI_STA); // station
  Serial.print("Hello, i'm slave, my MAC Address is: ");
  Serial.println(WiFi.macAddress());

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW!");
    for(;;); // just loop forever because there's no point going further
  }

  // tell the system to call the OnDataSent() function whenever data is transmitted
  esp_now_register_send_cb(OnDataSent);

  // tell this ESP who it will be talking to (the master ESP32 which will print the LIDAR reading out in this case)
  memcpy(peerInfo.peer_addr, masterBroadcastAddress, 6); // ADDRESS OF THE EXPECTED RECEIVER!!
  peerInfo.channel = 0;     // default channel
  peerInfo.encrypt = false; // default encryption (none)

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    for(;;); // just lay down and die
  }

  esp_now_register_recv_cb(OnDataReceive);

  if(!ACCEL.begin()) {
    Serial.println("Could not initialize accelerometer (is it plugged in correctly?)");
    for(;;); 
  }

  // idk why we do this but it's in the example, who am I to question it?
  ACCEL.setRange(MMA8451_RANGE_2_G);
  Serial.print("\nAccelerometer Range: ");
  Serial.print(2 << ACCEL.getRange());
  Serial.println(" G.");
}

// SLAVE
void loop() {
  // read the accelerometer every X seconds and send packaged data to master
  start_measurement_time = micros();
  readCurrentACCELEROMETERValue();
  sendCurrentACCELEROMETERValue();
  delay(10); // this is in milliseconds. delaying the loop for 10ms every iteration will (hopefully) yield
             // 100 measurements per second (aka measurement frequency of 100Hz)
}