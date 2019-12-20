#include <SPI.h>
#include <Wire.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <ModbusMaster.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)

#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

String serverIP = "34.87.xx.xx"; // Your Server IP
String serverPort = "2318"; // Your Server Port

#define MAX485_DE      PA1

ModbusMaster node;
bool state = true;
void preTransmission()
{
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_DE, 0);
}


float x,V,A,W,Wh,F,PF = 0; 
uint16_t V_int=0,A_int=0,W_int=0,Wh_int=0;

#include "AIS_NB_NE866.h"
String apnName = "devkit.nb";
String udpData = "HelloWorld";

AIS_NB_NE866 AISnb;

const long interval = 30000;  //millisecond
unsigned long previousMillis = 0;

long cnt = 0;
#define SEALEVELPRESSURE_HPA (1013.25)

float temperature = 0;
float humidity = 0;
uint16_t tempC_int = 0;
uint8_t hum_int = 0;
uint16_t lux_int = 0;
int vbat_int;

void setup_vdd_sensor() {
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
    regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel
}

void read_pzem16(){
uint8_t result;

  //oled.setCursor(0,6);oled.print("Reading modbus...");

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  result = node.readInputRegisters(0x0000, 9); //read the 9 registers of the PZEM-014 / 016
  if (result == node.ku8MBSuccess)
  {
    float voltage = node.getResponseBuffer(0x0000) / 10.0;
    V = voltage;
    
    uint32_t tempdouble = 0x00000000;
    float current;
    tempdouble = node.getResponseBuffer(0x0001);       //LowByte
    tempdouble |= node.getResponseBuffer(0x0002) << 8;  //highByte
    A = tempdouble / 1000.0;
    
    float power;    
    tempdouble |= node.getResponseBuffer(0x0003);       //LowByte
    tempdouble |= node.getResponseBuffer(0x0004) << 8;  //highByte
    W = tempdouble / 10.0;
    oled.setCursor(0,6);oled.println("                     ");      
    
  } else {
    Serial.println("Failed to read modbus"); 
    oled.setCursor(0,6);oled.println("Failed to read modbus"); 
  }     
}

void readData()
{
    read_pzem16();
    
    adc_enable(ADC1);
    vbat_int = 120 * 4096 / adc_read(ADC1, 17);
    adc_disable(ADC1);    

    oled.setCursor(0,3);oled.print("VBat:");oled.print(vbat_int*0.01);    
    oled.setCursor(0,5);oled.print(V,1);oled.print("V ");oled.print(A,2);oled.print("A ");oled.print(W,0);oled.print("W    ");  
  
    Serial.print(V);Serial.print(" ");
    Serial.print(A);Serial.print(" ");
    Serial.print(W);Serial.print(" ");
    Serial.println(Wh);
}


void setup()
{ 
  delay(10000);
  Serial.begin(9600);
  delay(1000);
  
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0); 

  Serial1.begin(9600);
  delay(1000);
  node.begin(1,Serial1);
  
  Serial2.begin(9600);

  //tsl.begin();
  bme.begin(); 
  
  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0
  
  oled.setFont(Adafruit5x7);

  oled.clear();
  oled.println("NBIoT AIS PC06");

  AISnb.debug = true;
  AISnb.setupDevice(serverPort,serverIP);
  String ip1 = AISnb.getDeviceIP();  
  oled.setCursor(0,1);oled.print("IP:");oled.print(ip1); 
  delay(1000);
  //pingRESP pingR = AISnb.pingIP(serverIP);
  previousMillis = millis();

}

void loop()
{ 
  unsigned long currentMillis = millis();
      readData();     
      // Send data in String 
      String DataSend ="{\"id\":\"NB-IoT-08\",\"volt\":"+String(V)+",\"Current\":"+String(A)+",\"Watt\":"+String(W)+",\"Kwh\":"+String(Wh)+"}";
      UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, DataSend);
      previousMillis = currentMillis;
      UDPReceive resp = AISnb.waitResponse();     
      delay(60000);
}



