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
#include <VL53L0X.h>

VL53L0X LIDAR;

//#define LONG_RANGE // makes LIDAR more sensitive... don't care for this test
#define HIGH_SPEED // high speed low accuracy mode for the LIDAR sensor
//#define HIGH_ACCURACY // you get high speed or high accuracy, choose one

uint8_t masterBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC };
uint8_t slaveBroadcastAddresss[] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0xF0 };

uint16_t incomingLidarReading;

typedef struct struct_message {
    uint16_t lidar_reading;
} struct_message;

struct_message incomingSensorReading;
struct_message outgoingSensorReading;

esp_now_peer_info_t peerInfo;

void readCurrentLIDARValue() {
  outgoingSensorReading.lidar_reading = LIDAR.readRangeSingleMillimeters();
}

void sendCurrentLIDARValue() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(masterBroadcastAddress, (uint8_t *) &outgoingSensorReading, sizeof(outgoingSensorReading));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
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
  incomingLidarReading = incomingSensorReading.lidar_reading;
}

void setup() {
  // 
  Serial.begin(115200);
  // start I2C communication
  Wire.begin();
  // initialize the LIDAR sensor (which communicates on the I2C protocol)
  LIDAR.setTimeout(250);
  if (!LIDAR.init()) {
    Serial.println("COULD NOT INIT THE LIDAR. IS IT CORRECTLY ATTACHED?");
    for(;;); // we can't go further if LIDAR doesn't work
  }

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  LIDAR.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  LIDAR.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  LIDAR.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  LIDAR.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  LIDAR.setMeasurementTimingBudget(200000);
#endif


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
  memcpy(peerInfo.peer_addr, masterBroadcastAddress, 6); // ADDRESS OF RECEIVER! (NOT OF THIS PARTICULAR uC!)
  peerInfo.channel = 0;     // default channel
  peerInfo.encrypt = false; // default encryption (none)

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    for(;;); // just lay down and die
  }

  esp_now_register_recv_cb(OnDataReceive);
}

// SLAVE
void loop() {
  readCurrentLIDARValue();
  Serial.print(" ----> Current LIDAR value (to be transmitted to Master): ");
  Serial.println(outgoingSensorReading.lidar_reading);
  sendCurrentLIDARValue();
  delay(500); // 0.5 sec delay between readings to not blow up our serial monitor
}