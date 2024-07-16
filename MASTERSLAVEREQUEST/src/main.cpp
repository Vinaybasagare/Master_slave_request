
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#define LED_PIN 2 // Example LED pin for status indication

// Define a data structure
struct DataPacket {
  float variable1;
  float variable2;
  float variable3;
  char variable4;

};

struct DataRecord {
  float variable1;       // Battery voltage
  float variable2;       // Temperature 1
  float variable3;       // Temperature 2
  uint8_t macAddr[6];    // MAC address of the sender
  String receivedTime;
  // Time when data was received
};

DataPacket receivedData;
DataPacket sendData;
uint8_t broadcastAddress[] = {0xE0, 0x5A, 0x1B, 0x31, 0x50, 0xD0};

const int maxRecords = 1500; // Adjust the size as needed
DataRecord dataRecords[maxRecords];
int recordIndex = 0;
esp_now_peer_info_t peerInfo;

// NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200, 60000); // Time offset 0, update interval 60 seconds

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

String getFormattedDateTime() {
  time_t rawTime = timeClient.getEpochTime();
  struct tm *timeInfo = localtime(&rawTime);

  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
  return String(buffer);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print("Received data length: ");
  Serial.println(len);

  // if (len == sizeof(DataPacket)) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Store received data in records
    if (recordIndex < maxRecords) {
      dataRecords[recordIndex].variable1 = receivedData.variable1;
      dataRecords[recordIndex].variable2 = receivedData.variable2;
      dataRecords[recordIndex].variable3 = receivedData.variable3;
      memcpy(dataRecords[recordIndex].macAddr, mac, 6);
      dataRecords[recordIndex].receivedTime = getFormattedDateTime(); // Store current date and time as string
      recordIndex++;

      // Print received data
      Serial.print("Data received: ");
      Serial.print("Received Battery Voltage: ");
      Serial.println(receivedData.variable1, 2); // Print with 2 decimal places
      Serial.print("Received Temperature 1: ");
      Serial.println(receivedData.variable2, 2); // Print with 2 decimal places
      Serial.print("Received Temperature 2: ");
      Serial.println(receivedData.variable3, 2); // Print with 2 decimal places

      // Print current date and time when data is received
      Serial.print("Current Date and Time: ");
      Serial.println(dataRecords[recordIndex - 1].receivedTime);
    } else {
      Serial.println("Data storage full. Increase maxRecords to store more data.");
    }
  // } else {
  //   Serial.println("Received data packet size does not match expected size");
  // }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP-NOW-AP", "password", 1, 0);  // Set to channel 1
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void printStoredData() {
  for (int i = 0; i < recordIndex; i++) {
    Serial.print("Record ");
    Serial.print(i);
    Serial.print(": Battery Voltage: ");
    Serial.print(dataRecords[i].variable1, 2);
    Serial.print(", Temperature 1: ");
    Serial.print(dataRecords[i].variable2, 2);
    Serial.print(", Temperature 2: ");
    Serial.print(dataRecords[i].variable3, 2);
    Serial.print(", MAC Address: ");
    for (int j = 0; j < 6; j++) {
      Serial.printf("%02X", dataRecords[i].macAddr[j]);
      if (j < 5) Serial.print(":");
    }
    Serial.print(", Received Time: ");
    Serial.println(dataRecords[i].receivedTime);
  }
}

void sendDataToSlave() {
  sendData.variable4 = 'p'; // Example value, replace with your actual data

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sendData, sizeof(sendData));

  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
  }
}

void loop() {
  timeClient.update();
  sendDataToSlave();
  delay(200);
  printStoredData();
}
