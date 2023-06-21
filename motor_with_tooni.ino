#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <ezButton.h>

#define SCK 14
#define MISO 12
#define MOSI 13
#define DirPin 17
#define StepPin 16
#define CSPin 15
#define ENC_IN_A 2 // pulse
#define ENC_IN_B 0 // direction
ezButton Limit_top_Pin(32);
ezButton Limit_bottom_Pin(33);
//36V 4A
boolean Direction = true;
volatile long wheel_pulse_count = 0;
float pitch = 0.4; //in cm
float distance = -28; //in cm
int rev = 400; //step per round
int s = (distance/pitch)*rev;
const float pulse_to_cm = pitch/rev;
float walk = 0;
unsigned long t = 0;
unsigned long t2 = 0;


const uint16_t StepPeriodUs = 800;

HighPowerStepperDriver sd;

#include <SPI.h>
#include <Ethernet.h>
//#ifndef ntohf

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
// MAC address of your W5500 module
EthernetServer sendServer(8080);
EthernetServer receiveServer(9090);
byte Enet = 1;
byte ip[] = {192,168,1,44};

void setup()
{
  // CS pin of w5500 module
  // arduino = pin10
  // esp32 = pin5
  Serial.begin(115200);
  Ethernet.init(5);
  //pinMode(32, OUTPUT);
  //pinMode(33, OUTPUT);
  Ethernet.begin(mac,ip)
  delay(1000);
  Serial.print("Ethernet IP address: ");
  Serial.println(Ethernet.localIP());
  sendServer.begin();
  receiveServer.begin();
  // set serial begin
  
 

  pinMode(ENC_IN_A, INPUT_PULLUP);
  pinMode(ENC_IN_B, INPUT);

  pinMode(CSPin, OUTPUT);
  pinMode(StepPin, OUTPUT);
  digitalWrite(StepPin, LOW);
  pinMode(DirPin, OUTPUT);
  digitalWrite(DirPin, LOW);
  digitalWrite(CSPin, LOW);

  Limit_top_Pin.setDebounceTime(50);
  Limit_bottom_Pin.setDebounceTime(50);

  // Give the driver some time to power up.
  delay(200);

  SPI.begin(SCK, MISO, MOSI, CSPin);
  sd.setChipSelectPin(CSPin);
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
}

void loop()
{ 
  t = millis();
  EthernetClient receiveClient = receiveServer.available();
  EthernetClient sendClient = sendServer.available();
  // send and receive
  if (receiveClient.connected() || sendClient.connected()){

    // receive data back from server
    // Read data from client
    float receivedData;
    int dataSize = receiveClient.read((uint8_t*)&receivedData, sizeof(receivedData));
    // Convert endianness of received data
    receivedData = ntohf_custom(receivedData);
    float distance = receivedData;
    int s = (distance/pitch)*rev;
    // Serial print received data
    Serial.print("Received data: ");
    Serial.println(receivedData, 2);
  }
  while (s != 0) //rotate motor untill move into given distance 
  {
    Limit_top_Pin.loop();
    Limit_bottom_Pin.loop();
    // push switch = 1 , not push = 0
    int TopState = Limit_top_Pin.getState();
    int BottomState = Limit_bottom_Pin.getState();
    // let driver work
    sd.enableDriver();
    //Dir 0 is down (A Green-Black, B Red-Blue)
    // get highest get down too
    digitalWrite(CSPin, HIGH);
    if (t < 2000) //in case lift is pressing limit switch
    {
      if (s > 0)
      {
        moveup();
      }
      else if (s < 0)
      {
        movedown();
      }
    }
    else
    {
      if (s > 0 && TopState == 1)
      {
        moveup();
      }
      else if (s < 0 && BottomState == 1)
      {
        movedown();
      }
    }
        //Serial Communication with delay (w/o delay will cause motor to rotate slower)
    // walk = current distance
    walk = pulse_to_cm * wheel_pulse_count*0.4 ;
    // t, t2 will get bigger with time
    if (t-t2 >= 2000 && distance != 0)
    {
      Serial.print(" distance: ");
      Serial.println(walk);
      t2 = t;
    }
  }
  // send and receive
  if (receiveClient.connected() || sendClient.connected()){
    Serial.print("Send data: ");
    Serial.println(walk);
  // sending from client to server
    sendClient.write((byte*)&walk, sizeof(walk));    
  }
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
  s--;
}

void movedown()
{
  setDirection(0);
  //Serial.println(s);
  //Serial.println(BottomState);
  sd.step();
  delayMicroseconds(StepPeriodUs);
  sd.enableDriver();
  s++;  
}
