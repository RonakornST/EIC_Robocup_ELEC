#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <ezButton.h>
#include <Ethernet.h>

//define HSPI Channel and Motor Driver Pin
#define HSPI_CLK 14
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define DirPin 17
#define StepPin 16
#define HSPI_SS 15
#define ENC_IN_A 2 // pulse
#define ENC_IN_B 0 // direction

//define VSPI Channel
#define VSPI_CLK 18
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SS 5

SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

//define limitswitch pin
ezButton Limit_top_Pin(32);
ezButton Limit_bottom_Pin(33);

//36V 4A
boolean Direction = true;
volatile long wheel_pulse_count = 0;
float pitch = 0.4; //in cm
//float distance = -28; //in cm
int rev = 400; //step per round
//int s = (distance/pitch)*rev;
int currentstep = 0;
float currentdistance = 0;
float previousdistance = 0;
const float pulse_to_cm = pitch/rev;
float walk = 0;
unsigned long t = 0;
unsigned long t2 = 0;
unsigned long t3 = 0;
unsigned long t4 = 0;
int TopState = 1;
int BottomState = 1;
int i = 0;
const uint16_t StepPeriodUs = 800;
static const int spiClk = 1000000; // 1 MHz

HighPowerStepperDriver sd;
//#ifndef ntohf
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// MAC address of your W5500 module
EthernetServer sendServer(8080);
EthernetServer receiveServer(9090);
byte Enet = 1;
byte ip[] = {192,168,1,44};

void wheel_pulse()
{
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_B);
  if (val == LOW) {
    Direction = false; // Reverse
  }
  else {
    Direction = true; // Forward
  }

  if (Direction) {
    wheel_pulse_count++;
  }
  else {
    wheel_pulse_count--;
  }
}


void resetPolulu()
{
  // Enable the motor outputs.

}
// void step()
// {
//   // The STEP minimum high pulse width is 1.9 microseconds.
//   digitalWrite(StepPin, HIGH);
//   delayMicroseconds(3);
//   digitalWrite(StepPin, LOW);
//   delayMicroseconds(3);
// }

void stop()
{
  digitalWrite(StepPin, LOW);
}

void setDirection(bool dir)
{
  // The STEP pin must not change for at least 200 nanoseconds before and after
  // changing the DIR pin.
  delayMicroseconds(10);
  digitalWrite(DirPin, dir);
  delayMicroseconds(10);
}

void moveup()
{
  setDirection(1);
  //Serial.println(s);
  //Serial.println(TopState);
  sd.step();
  delayMicroseconds(StepPeriodUs);
  sd.enableDriver();
  currentstep--;
  Serial.println("Move");
}

void movedown()
{
  setDirection(0);
  //Serial.println(s);
  //Serial.println(BottomState);
  sd.step();

  delayMicroseconds(StepPeriodUs);
  sd.enableDriver();
  currentstep++;
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

void setup()
{
  // CS pin of w5500 module
  // arduino = pin10
  // esp32 = pin5
  Serial.begin(115200);
  //setup  2 SPI Bus Interface
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  vspi->begin(VSPI_CLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  hspi->begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
  //==============ETHERNET===========//
  Ethernet.init(5);
  //pinMode(32, OUTPUT);
  //pinMode(33, OUTPUT);
  Ethernet.begin(mac,ip);
  //delay(1000); DON'T FUCKING TURN THIS ON (CAUSE ETHERNET ERROR)
  Serial.print("Ethernet IP address: ");
  Serial.println(Ethernet.localIP());
  sendServer.begin();
  receiveServer.begin();
  //==============PIN SETUP===========//
  pinMode(vspi->pinSS(), OUTPUT); //VSPI SS
  pinMode(hspi->pinSS(), OUTPUT); //HSPI SS
  pinMode(ENC_IN_A, INPUT_PULLUP);
  pinMode(ENC_IN_B, INPUT);
  pinMode(HSPI_SS, OUTPUT);
  pinMode(StepPin, OUTPUT);
  pinMode(DirPin, OUTPUT);
  digitalWrite(StepPin, LOW);
  digitalWrite(DirPin, LOW);
  digitalWrite(HSPI_SS, LOW);
  //==============LIMIT SWITCH===========//
  Limit_top_Pin.setDebounceTime(50);
  Limit_bottom_Pin.setDebounceTime(50);
  //==============DRIVER===========//
  // Give the driver some time to power up.
  delay(200);

  //SPI.begin(SCK, MISO, MOSI, CSPin);
  sd.setChipSelectPin(HSPI_SS);
  // Reset the driver to its default settings and clear latched status
  // conditions. Try to change this
  sd.resetSettings();
  sd.clearStatus();
  sd.setDecayMode(HPSDDecayMode::AutoMixed);
  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd.setCurrentMilliamps36v4(2800);

  // 8 is smooth but slow, 2 is fast but not smooth
  sd.setStepMode(HPSDStepMode::MicroStep2);
  sd.enableDriver();
  attachInterrupt(digitalPinToInterrupt(ENC_IN_A), wheel_pulse, RISING);
  Serial.print(receiveServer.available());
  Serial.print(sendServer.available());
}

void loop()
{
  t = millis();
  EthernetClient receiveClient = receiveServer.available();
  EthernetClient sendClient = sendServer.available();
  //Serial.print(receiveServer.available());
  //Serial.print(sendServer.available());
  // send data every 1 sec
  if (t-t3 >= 100)
    {
       if (receiveClient.connected() || sendClient.connected())
       {
        sendClient.write((byte*)&walk, sizeof(walk));
        //Send a float to the client
        float responseValue = 10.00;  // Modify to send a single float
        responseValue = htonf_custom(responseValue);
        receiveClient.write((uint8_t*)&responseValue, sizeof(responseValue));

        // receive data back from server
        // Read data from client
        float receivedData;
        int dataSize = receiveClient.read((uint8_t*)&receivedData, sizeof(receivedData));
        // Convert endianness of received data
        receivedData = ntohf_custom(receivedData);
        float currentdistance = receivedData;
        currentstep = ((currentdistance - previousdistance)/pitch)*rev;
        previousdistance = currentdistance;
        // Serial print received data
        Serial.print("Received data: ");
        Serial.print(receivedData, 2);
        Serial.println(i);
        //Serial.println(s);
        }
      //Serial.println(s);
      t3 = t;
    }
    //since data have been sent continously , we will rotate motor untill current distance matchs recieved distance
  while(currentstep != 0) //rotate motor untill move into given distance
  {
    //Serial.println(currentstep);
    //Limit_top_Pin.loop();
    //Limit_bottom_Pin.loop();
    // push switch = 1 , not push = 0
    //int TopState = Limit_top_Pin.getState();
    //int BottomState = Limit_bottom_Pin.getState();
    // let driver work
    sd.enableDriver();
    //Dir 0 is down (A Green-Black, B Red-Blue)
    // get highest get down too
    digitalWrite(HSPI_SS, HIGH);
    if (t < 2000) //in case lift is pressing limit switch
    {
      if (currentstep > 0)
      {
        moveup();

      }
      else if (currentstep < 0)
      {
        movedown();
      }
    }
    else
    {
      if (currentstep > 0 && TopState == 1)
      {
        //Serial.println("MOVE");
        moveup();
      }
      else if (currentstep < 0 && BottomState == 1)
      {
        movedown();
      }
    }
    //Serial Communication with delay (w/o delay will cause motor to rotate slower)
    // walk = current distance
    walk = pulse_to_cm * wheel_pulse_count*0.4 ;
    // t, t2 will get bigger with time
    if (t-t2 >= 2000)
    {
      Serial.print(" distance: ");

      Serial.print(walk);
      t2 = t;
    }
  }
  if (t-t4 >= 100)
    {
      if (receiveClient.connected() || sendClient.connected())
      {
        Serial.print("Send data: ");

        Serial.println(walk);
      // sending from client to server
        sendClient.write((byte*)&walk, sizeof(walk));
        //Serial.println(i);
       i++;
      }
     t4 = t;
  // send and receive
  }
}
