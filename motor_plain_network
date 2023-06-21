#include <SPI.h>
#include <Ethernet.h>
//#ifndef ntohf

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
// MAC address of your W5500 module
EthernetServer sendServer(8080);
EthernetServer receiveServer(9090);
byte Enet = 1;
byte ip[] = {192,168,1,22};

//float m = 0;

void setup() {
  // CS pin of w5500 module
  // arduino = pin10
  // esp32 = pin5
  Ethernet.init(5);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  Ethernet.begin(mac,ip);
  // check = Ethernet.begin(mac,ip);
  // int check = Ethernet.begin(mac,ip);
  
  // set serial begin
  Serial.begin(115200);
 

  while (!Serial){
    // wait for serial port connect
  }
  Serial.println("Ethernet");

  // if (Ethernet.begin(mac,ip) == 0) {

  //   Serial.println("Failed to configure Ethernet using DHCP");
  //   Enet = 0;
  // }
  delay(1000);

  if (Enet == 1) {
    Serial.print("Ethernet IP address: ");
    Serial.println(Ethernet.localIP());
  }
  else {
    Serial.println("Unable to connect to Ethernet");
  }

  sendServer.begin();
  receiveServer.begin();

  // finish setting client
}

void loop() {
  // using for check client status
  
  EthernetClient receiveClient = receiveServer.available();
  EthernetClient sendClient = sendServer.available();
  float m = -2.0;
  // send and receive
  if (sendClient || receiveClient){
    Serial.println("New client connected");
  
    while (receiveClient.connected() || sendClient.connected()){
      // read data
      float Data1 = analogRead(34);
      //float Data2 = analogRead(35);
      float Data2 = m;
      float read_Data = Data2;  // Modify to send a single float
      // send data from esp32
      //
      // serial print for check
      Serial.print("Send data: ");
      Serial.println(read_Data);
      //
      // sending from client to server
      sendClient.write((byte*)&read_Data, sizeof(read_Data));
      //
      // end send data to server
      
      // receive data back from server
      // Read data from client
      float receivedData;
      int dataSize = receiveClient.read((uint8_t*)&receivedData, sizeof(receivedData));

      // Convert endianness of received data
      receivedData = ntohf_custom(receivedData);

      // Serial print received data
      Serial.print("Received data: ");
      Serial.println(receivedData, 2);

      // LED
      if (receivedData <= 150){
        digitalWrite(32, HIGH);
        digitalWrite(33, LOW);
      }
      else if (receivedData >= 150 && receivedData <= 250){
        digitalWrite(32, HIGH);
        digitalWrite(33, HIGH);
      }
      else if (receivedData >= 250){
        digitalWrite(32, LOW);
        digitalWrite(33, HIGH);
      }

      //Send a float to the client
      float responseValue = 10.00;  // Modify to send a single float
      responseValue = htonf_custom(responseValue);
      receiveClient.write((uint8_t*)&responseValue, sizeof(responseValue));
      //End of receiving data

      m += 1;
      delay(20);
    }
    sendClient.stop();
    receiveClient.stop();
    Serial.println("Disconnected");
    delay(1000);  
  }  
  //End
}

// Custom implementation of ntohf
float ntohf_custom(float value) {
  uint32_t temp = ntohl_custom(*((uint32_t*)&value));
  return *((float*)&temp);
}

// Custom implementation of htonf
float htonf_custom(float value) {
  return ntohf_custom(value);
}

// Custom implementation of ntohl
uint32_t ntohl_custom(uint32_t value) {
  return ((value & 0xFF) << 24) | ((value & 0xFF00) << 8) | ((value >> 8) & 0xFF00) | ((value >> 24) & 0xFF);
}

// Custom implementation of htonl
uint32_t htonl_custom(uint32_t value) {
  return ntohl_custom(value);
}
