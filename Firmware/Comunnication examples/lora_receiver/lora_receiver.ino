#include <SPI.h>
#include <LoRa.h>


void decodePacket(unsigned char* buffer, size_t bufferSize);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    
    // read packet into buffer
    byte buffer[packetSize];
    for (int i = 0; i < packetSize; i++) {
      buffer[i] = LoRa.read();
    }

    decodePacket(buffer,packetSize);
    
    // print RSSI of packet
//    Serial.print("RSSI: ");
//    Serial.print(LoRa.packetRssi());
//    Serial.print(" dBm");
//
//    // print SNR of packet
//    Serial.print(", SNR: ");
//    Serial.print(LoRa.packetSnr());
//    Serial.print(" dB");
//    Serial.println();
  }
}


void decodePacket(unsigned char* buffer, size_t bufferSize) {
  // Check start byte
  if (buffer[0] != 0xFF) {
    Serial.println("Invalid start byte");
    return;
  }
  
  // Extract data from buffer
  int16_t roll = (buffer[1] << 8) | buffer[2];
  int16_t pitch = (buffer[3] << 8) | buffer[4];
  int16_t heading = (buffer[5] << 8) | buffer[6];
  int16_t temperature = (buffer[7] << 8) | buffer[8];
  int32_t pressure = (buffer[9] << 24) | (buffer[10] << 16) | (buffer[11] << 8) | buffer[12];
  int32_t seaLevelPressure = (buffer[13] << 24) | (buffer[14] << 16) | (buffer[15] << 8) | buffer[16];
  int32_t altitude = (buffer[17] << 24) | (buffer[18] << 16) | (buffer[19] << 8) | buffer[20];
  int32_t correctedAltitude = (buffer[21] << 24) | (buffer[22] << 16) | (buffer[23] << 8) | buffer[24];
  
  // Check stop byte
  if (buffer[25] != 0xFE) {
    Serial.println("Invalid stop byte");
    return;
  }
  
  // Print extracted data to serial monitor
//  Serial.print("Roll: ");
//  Serial.print((float)roll / 10.0);
//  Serial.print(", Pitch: ");
//  Serial.print((float)pitch / 10.0);
//  Serial.print(", Heading: ");
//  Serial.print((float)heading);
//  Serial.print(", Temperature: ");
//  Serial.print((float)temperature / 10.0);
//  Serial.print(", Pressure: ");
//  Serial.print((float)pressure / 100.0);
//  Serial.print(", Sea Level Pressure: ");
//  Serial.print((float)seaLevelPressure / 100.0);
//  Serial.print(", Altitude: ");
//  Serial.print((float)altitude / 100.0);
//  Serial.print(", Corrected Altitude: ");
//  Serial.println((float)correctedAltitude / 100.0);

  Serial.print((float)roll / 10.0);
  Serial.print(",");
  Serial.print((float)pitch / 10.0);
  Serial.print(",");
  Serial.print((float)heading);
  Serial.print(",");
  Serial.print((float)temperature / 10.0);
  Serial.print(",");
  Serial.print((float)pressure / 100.0);
  Serial.print(",");
  Serial.print((float)seaLevelPressure / 100.0);
  Serial.print(",");
  Serial.print((float)altitude / 100.0);
  Serial.print(",");
  Serial.print((float)correctedAltitude / 100.0);
  Serial.print(",");
  Serial.print(LoRa.packetRssi());
  Serial.print(",");
  Serial.print(LoRa.packetSnr());
  Serial.println();

}
