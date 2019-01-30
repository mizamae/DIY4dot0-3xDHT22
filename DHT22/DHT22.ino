/* 
 *  DEVICE:    LOLIN WEMOS D1R2 & mini
 *  ORDERS:
 *      /orders/reset
 *          Order to soft-reset the device
 *      /orders/SetConf
 *          Order to set the device code.
 *          Params: DEVC <int> to set the device code
*       /orders/resetStatics
*           Order to reset the static variables after a request
*       /orders/setrelay
*           Order to set the status of a relay
*           Params: REL <int> the relay number to be set (1,2,3)
*                   VAL <int> the value to be set (1=ON, 0=OFF)
*   REQUESTS:
*       /Conf.xml
*           Request to retrieve the current configuration of the device
*       /measure.xml
*           Request to get instantaneous values of the sensors
*       /data.xml
*           Request to get the aggregated values of the sensors
*           
      
*/
#include <ArduinoJson.h>

#include <DHT.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <SPI.h>                         // Include the SPI library
#include <FS.h>
#include <ESP8266HTTPUpdateServer.h>


//#define __TEST__
#define __DVT__ "3xDHT22"
#define __FW_VERSION__ "1.0"
#define __WEB_VERSION__ "1.0"
#define __SAMPLE_TIME__   20000    // sample time in ms 

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#include "wifi_details.h"
unsigned long timeNoPoll;
        
// DHT Sensor
const int DHTPin1 = D5; //D4
const int DHTPin2 = D6; //D2
const int DHTPin3 = D7; //D1
// Initialize DHT sensor.
DHT dht1(DHTPin1, DHTTYPE);
DHT dht2(DHTPin2, DHTTYPE);
DHT dht3(DHTPin3, DHTTYPE);
static float t1_mean=0,h1_mean=0,t2_mean=0,h2_mean=0,t3_mean=0,h3_mean=0;
static int t1_number=0,h1_number=0,t2_number=0,h2_number=0,t3_number=0,h3_number=0;
static byte STATUS_bits=0,Measure1OK=0,Measure2OK=0,Measure3OK=0;
// STATUS BITS
const byte _ErrorSensor1_=0;
const byte _ErrorSensor2_=1;
const byte _ErrorSensor3_=2;
const byte _Relay1_=3;
const byte _Relay2_=4;
const byte _Relay3_=5;

// Relays
const int Relay1Pin = D1; 
const int Relay2Pin = D2;
const int Relay3Pin = D3;
#define __RELAY_ON__ 0
#define __RELAY_OFF__ 1

uint8_t IPaddr[4];
uint8_t DeviceCode;
const String DeviceType=__DVT__; 
float T1,T2,T3;
float H1,H2,H3;
float DewPoint_centdeg;

MDNSResponder mdns;
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;

static uint8_t baseREQ=0,orderREQ=0,webREQ=0; 

struct XML_TAGS_BYTE // size 16 bytes
{
  const char _TAG_[5]; 
  const char _VALUE_TAG_;
  const char _cTAG_[6];   
}var_tag =  {
                {'<','V','A','R','>'},
                '$',
                {'<','/','V','A','R','>'} 
              };
              
struct XML_TAGS_FLOAT // size 20 bytes
{
  const char _TAG_[4]; 
  const char _VALUE_TAG_[7];
  const char _cTAG_[5];   
}var_tag_float =  {
                {'<','A','V','>'},
                {'$',',','$',',','$',',','$'},
                {'<','/','A','V','>'} 
              };

struct RESPONSE_OK
{
  const char  _HTML_RESPONSE_OK_[16];
  const char _BLANKLINE_ ;
}response_ok= {
                 {'H','T','T','P','/','1','.','1',' ','2','0','0',' ','O','K','\n'},
                  '\n',
                };

const char *pRESP_OK    =    &response_ok._HTML_RESPONSE_OK_[0];
const int __SIZE_RESPONSE_OK__    =    sizeof(response_ok);

struct XML_RESPONSES_Conf
{ 
  const char _XML_HEADER_[25];
  const char _XML_TAG_[3];
  const char _XML_DEVT_TAG_[6];
  const char _XML_DEVT_[7];
  const char _cXML_DEVT_TAG_[7];
  const char _XML_DEVC_TAG_[6];
  char _XML_DEVC_[1];
  const char _cXML_DEVC_TAG_[7];
  const char _cXML_TAG_[5];
}xml_response_Conf= {
                 {'<','?','x','m','l',' ','v','e','r','s','i','o','n',' ','=',' ','"','1','.','0','"',' ','?','>','\n'},
                 {'<','X','>'},
                 {'<','D','E','V','T','>'},{'3','x','D','H','T','2','2'},{'<','/','D','E','V','T','>'},
                 {'<','D','E','V','C','>'},{'$'},{'<','/','D','E','V','C','>'},
                 {'<','/','X','>','\n'}  
                };

const char *pXML_RESP_Conf    =    &xml_response_Conf._XML_HEADER_[0];
const int __SIZE_XML_RESPONSE_Conf__    =    sizeof(xml_response_Conf);

struct XML_RESPONSES_data
{
  const char _XML_HEADER_[25];
  const char _XML_TAG_[3];
  const char _XML_DEV_TAG_[5];
  const char _XML_DEV_[1];
  const char _cXML_DEV_TAG_[6];
  const char _XML_DId_TAG_[5];
  const char _XML_DId_[1];
  const char _cXML_DId_TAG_[6];
  struct XML_TAGS_BYTE BYTE_DATA_TAGS[1];
  struct XML_TAGS_FLOAT FLOAT_DATA_TAGS[6];
  const char _cXML_TAG_[5];
}xml_response_data = {
                 {'<','?','x','m','l',' ','v','e','r','s','i','o','n',' ','=',' ','"','1','.','0','"',' ','?','>','\n'},
                 {'<','X','>'},
                 {'<','D','E','V','>'},{'#'},{'<','/','D','E','V','>'},
                 {'<','D','I','d','>'},{'0'},{'<','/','D','I','d','>'},
                 {var_tag},{var_tag_float,var_tag_float,var_tag_float,var_tag_float,var_tag_float,var_tag_float},
                 {'<','/','X','>','\n'}  
                };

const char *pXML_RESP_data = &xml_response_data._XML_HEADER_[0];
const int __SIZE_XML_RESPONSE_data__ = sizeof(xml_response_data);

struct XML_RESPONSES_status
{
  const char _XML_HEADER_[25];
  const char _XML_TAG_[3];
  const char _XML_DEV_TAG_[5];
  const char _XML_DEV_[1];
  const char _cXML_DEV_TAG_[6];
  struct XML_TAGS_BYTE BYTE_DATA_TAGS[1];
  const char _cXML_TAG_[5];
}xml_response_status = {
                 {'<','?','x','m','l',' ','v','e','r','s','i','o','n',' ','=',' ','"','1','.','0','"',' ','?','>','\n'},
                 {'<','X','>'},
                 {'<','D','E','V','>'},{'#'},{'<','/','D','E','V','>'},
                 {var_tag},
                 {'<','/','X','>','\n'}  
                };

const char *pXML_RESP_status = &xml_response_status._XML_HEADER_[0];
const int __SIZE_XML_RESPONSE_status__ = sizeof(xml_response_status);

// EEPROM structure
const uint8_t _EEPROMaddrDEVICECODE_      = 0;     // address for the deviceCode in EEPROM
const uint8_t _EEPROMaddrIP_B0_           = 1;     // address for the LSB of IP address in EEPROM
const uint8_t _EEPROMaddrIP_B1_           = 2;     // address for the 2nd Byte of IP address in EEPROM
const uint8_t _EEPROMaddrIP_B2_           = 3;     // address for the 3rd Byte of IP address in EEPROM
const uint8_t _EEPROMaddrIP_B3_           = 4;     // address for the MSB of IP address in EEPROM


// CONFIG BUTON
const byte _CONF_BTN_                    = D6;        // pin to initiate config procedure

static bool configured_device=false;
static unsigned long timeSample;


volatile bool stopPolling=false,data_request=false;

void setup()
{
    Serial.begin(9600);
    // INITIALIZE RELAYS
    pinMode(Relay1Pin, OUTPUT);           // set pin to output
    digitalWrite(Relay1Pin, __RELAY_OFF__);       // turn off relay
    pinMode(Relay2Pin, OUTPUT);           // set pin to output
    digitalWrite(Relay2Pin, __RELAY_OFF__);       // turn off relay
    pinMode(Relay3Pin, OUTPUT);           // set pin to output
    digitalWrite(Relay3Pin, __RELAY_OFF__);       // turn off relay
    // TO FORCE CERTAIN IP ADDRESS
    #ifdef __TEST__
      EEPROM.begin(512);
      DeviceCode=254;
      EEPROM.write(_EEPROMaddrIP_B0_,DeviceCode);
      EEPROM.write(_EEPROMaddrIP_B1_,IP[1]);
      EEPROM.write(_EEPROMaddrIP_B2_,IP[2]);
      EEPROM.write(_EEPROMaddrIP_B3_,IP[3]);
      EEPROM.write(_EEPROMaddrDEVICECODE_,DeviceCode);
      EEPROM.end();
    #endif
    // END
    EEPROM.begin(512);
    DeviceCode = EEPROM.read(_EEPROMaddrDEVICECODE_);
    Serial.print("DeviceCode in EEPROM: ");
    Serial.println(DeviceCode);
    
    if ((DeviceCode==255)||(DeviceCode==254)||!digitalRead(_CONF_BTN_)) // the device has not been configured. This is so in Arduino devices where EEPROM defaults to 255. In ESP8266 flash is randomly defaulted!!!
    {
        DeviceCode=IP[0];      
        IPaddr[0]=IP[0];
        IPaddr[1]=IP[1];
        IPaddr[2]=IP[2];
        IPaddr[3]=IP[3];

        Serial.println("Unconfigured device");
        configured_device=false;
    }else
    {
        IPaddr[0]=EEPROM.read(_EEPROMaddrIP_B0_);
        IPaddr[1]=EEPROM.read(_EEPROMaddrIP_B1_);
        IPaddr[2]=EEPROM.read(_EEPROMaddrIP_B2_);
        IPaddr[3]=EEPROM.read(_EEPROMaddrIP_B3_);
        Serial.print("Configured device with IP: ");
        Serial.print(IPaddr[3]); 
        Serial.print(".");
        Serial.print(IPaddr[2]); 
        Serial.print(".");
        Serial.print(IPaddr[1]); 
        Serial.print(".");
        Serial.println(IPaddr[0]);    
        configured_device=true;
    }
    
    IPAddress ip(IPaddr[3], IPaddr[2], IPaddr[1], IPaddr[0]);   // IP address, may need to change depending on network
    IPAddress gateway(IPaddr[3], IPaddr[2], IPaddr[1], 1);      // set gateway to match your network
    IPAddress subnet(255, 255, 255, 0);                         // set subnet mask to match your network
    
    WiFi.config(ip, gateway, subnet);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
   
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
   
    // Print the IP address
    Serial.print("Use this URL to connect: ");
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    
    if (mdns.begin("esp8266", WiFi.localIP())) {
        Serial.println("MDNS responder started");
    }

    dht1.begin();
    dht2.begin();
    dht3.begin();
    
    server.on("/orders/SetConf", [](){
        if (server.arg("DEVC")!= "")  
        {
          DeviceCode = server.arg("DEVC").toInt();              //Get the value of the parameter
          EEPROM.write(_EEPROMaddrIP_B0_,DeviceCode);
          EEPROM.write(_EEPROMaddrIP_B1_,IP[1]);
          EEPROM.write(_EEPROMaddrIP_B2_,IP[2]);
          EEPROM.write(_EEPROMaddrIP_B3_,IP[3]);
          EEPROM.write(_EEPROMaddrDEVICECODE_,DeviceCode);
          EEPROM.end();
          Serial.print("DeviceCode to be set at ");
          Serial.println(DeviceCode);
              
          server.send(200, "text/html", "");
        }else
        {
          server.send(200, "text/html", "");
        }        
         
        delay(200);
    });
    
    server.on("/orders/resetStatics", [](){
        server.send(200, "text/html", "");
        Serial.print("Statics resetted");
        initialize_statics();
    });

    server.on("/orders/setrelay", [](){
        int relay,value,valueOUT;
        if (server.arg("REL")!= "" && server.arg("VAL")!= "")  
        {
          relay = server.arg("REL").toInt();              //Get the value of the parameter
          value = server.arg("VAL").toInt();              //Get the value of the parameter
          Serial.print("Relay to be set ");
          Serial.print(relay);
          Serial.print(". To the value ");
          Serial.println(value);
          if (value){valueOUT=__RELAY_ON__;}
          else{valueOUT=__RELAY_OFF__;}
          switch (relay)
          { case 1:
                  digitalWrite(Relay1Pin, valueOUT);       // turn on relay
                  break;
            case 2: 
                  digitalWrite(Relay2Pin, valueOUT);       // turn on relay
                  break;
            case 3:
                  digitalWrite(Relay3Pin, valueOUT);       // turn on relay
                  break;
            default:
                  Serial.print("Relay to be set ");
                  Serial.println(relay);
                  server.send(404, "text/html", "Relay number incorrect");
                  delay(200);
                  return;
          }
        }else
        {
          server.send(404, "text/html", "The parameter REL for the relay number and/or the VAL for the value were not found in the url");
          delay(200);
          return;
        }
        checkRelays();
        String response = XML_response(&STATUS_bits,__SIZE_XML_RESPONSE_status__,pXML_RESP_status); // sends the status of all switches
        server.send(200, "text/html", response);
        delay(200);
    });
    server.on("/Conf.xml", [](){
        String response = XML_response(&DeviceCode,__SIZE_XML_RESPONSE_Conf__,pXML_RESP_Conf); // sends the status of all switches
        //Serial.print(response);
        server.send(200, "text/xml", response);
        delay(200);
    });

    server.on("/measure.xml", [](){
        data_request=true;
    });

    server.on("/orders/reset", [](){
        server.send(200, "text/html", "");
        delay(200);
        //remove_setupJSON();
        ESP.reset();
    });
    
    server.on("/data.xml", [](){
        byte var[25];  
        float t1,h1,t2,h2,t3,h3;
        yield();
        if (Measure1OK==1)
        {
          h1=h1_mean;
          t1=t1_mean;
        }else
        {// no good measurement, send a NaN
          for (byte i=0;i<4;i++)
          {
            *((byte*)&t1+3-i)=0xFF;
            *((byte*)&h1+3-i)=0xFF;
          }
          STATUS_bits|=1<<_ErrorSensor1_;
        }
        if (Measure2OK==1)
        {
          h2=h2_mean;
          t2=t2_mean;
        }else
        {// no good measurement, send a NaN
          for (byte i=0;i<4;i++)
          {
            *((byte*)&t2+3-i)=0xFF;
            *((byte*)&h2+3-i)=0xFF;
          }
          STATUS_bits|=1<<_ErrorSensor2_;
        }
        if (Measure3OK==1)
        {
          h3=h3_mean;
          t3=t3_mean;
        }else
        {// no good measurement, send a NaN
          for (byte i=0;i<4;i++)
          {
            *((byte*)&t3+3-i)=0xFF;
            *((byte*)&h3+3-i)=0xFF;
          }
          STATUS_bits|=1<<_ErrorSensor3_;
        }

        checkRelays();
                  
        var[0]=STATUS_bits;
        for (byte i=0;i<4;i++)
        {
            yield();
            var[i+1]=*((byte*)&t1+3-i);

            var[i+5]=*((byte*)&h1+3-i);

            var[i+9]=*((byte*)&t2+3-i);

            var[i+13]=*((byte*)&h2+3-i);

            var[i+17]=*((byte*)&t3+3-i);

            var[i+21]=*((byte*)&h3+3-i);
        }
        String response = XML_response(&var[0],__SIZE_XML_RESPONSE_data__,pXML_RESP_data);
        
        server.send(200, "text/xml", response);
        Serial.println(response);
    });
    

    if (!SPIFFS.begin())
    {
      // Serious problem
      Serial.println("SPIFFS Mount failed");
      //ESP.reset();
    } else {
      Serial.println("SPIFFS Mount succesfull");
    }
    if (!SPIFFS.exists("/setup.json"))
    {
      create_setupJSON();
    }
    
    server.serveStatic("/img", SPIFFS, "/img");
    server.serveStatic("/", SPIFFS, "/index.html");
    server.serveStatic("/setup.json", SPIFFS, "/setup.json");
  

    // Start the server
    const char * updatepath = "/firmwareupdate";
    const char * username = "diy4dot0";
    const char * password = "diy4dot0";
    httpUpdater.setup(&server,updatepath,username,password);
    server.begin();
    Serial.println("Server started");
    
    
    timeSample=millis();
}

void checkRelays()
{
  if (digitalRead(Relay1Pin)==__RELAY_ON__){STATUS_bits|=1<<_Relay1_;
  }else{STATUS_bits&=~(1<<_Relay1_);}
  if (digitalRead(Relay2Pin)==__RELAY_ON__){STATUS_bits|=1<<_Relay2_;
  }else{STATUS_bits&=~(1<<_Relay2_);}
  if (digitalRead(Relay3Pin)==__RELAY_ON__){STATUS_bits|=1<<_Relay3_;
  }else{STATUS_bits&=~(1<<_Relay3_);}
}


void remove_setupJSON()
{
  if (SPIFFS.exists("/setup.json"))
    {
      SPIFFS.remove("/setup.json");
      Serial.println("setup.json removed OK");
    }
}

void create_setupJSON()
{
      File setupFile = SPIFFS.open("/setup.json", "w");
      const size_t bufferSize = JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(5);
      DynamicJsonBuffer jsonBuffer(bufferSize);
      
      JsonObject& root = jsonBuffer.createObject();
      root["DVT"] = __DVT__;
      root["fw_version"] = __FW_VERSION__;
      root["web_version"] = __WEB_VERSION__;
      root["sample"] = __SAMPLE_TIME__;
      
      JsonArray& sensor_pins = root.createNestedArray("sensor_pins");
      sensor_pins.add("D5");
      sensor_pins.add("D6");
      sensor_pins.add("D7");
      root.printTo(setupFile); // Export and save JSON object to SPIFFS area
      setupFile.close();  
      Serial.println("setup.json created OK");  
}

void loop()
{    
    if (data_request){
        byte var[25];
        data_request=false;
        Serial.println("Automatic polling stopped");
        stopPolling=true;  
        timeNoPoll=millis();
        yield();
        float h1 = dht1.readHumidity();
        float t1 = dht1.readTemperature();
        delay(100);
        if (isnan(h1) || isnan(t1) ){
          for (byte i=0;i<4;i++)
          {
            *((byte*)&t1+3-i)=0xFF;
            *((byte*)&h1+3-i)=0xFF;
          }
          STATUS_bits|=1<<_ErrorSensor1_;
        }else{STATUS_bits&=~(1<<_ErrorSensor1_);}
        float h2 = dht2.readHumidity();
        float t2 = dht2.readTemperature();
        delay(100);
        if (isnan(h2) || isnan(t2) ){
          for (byte i=0;i<4;i++)
          {
            *((byte*)&t2+3-i)=0xFF;
            *((byte*)&h2+3-i)=0xFF;
          }
          STATUS_bits|=1<<_ErrorSensor2_;
        }else{STATUS_bits&=~(1<<_ErrorSensor2_);}
        float h3 = dht3.readHumidity();
        float t3 = dht3.readTemperature();
        delay(100);
        if (isnan(h3) || isnan(t3) ){
          for (byte i=0;i<4;i++)
          {
            *((byte*)&t3+3-i)=0xFF;
            *((byte*)&h3+3-i)=0xFF;
          }
          STATUS_bits|=1<<_ErrorSensor3_;
        }else{STATUS_bits&=~(1<<_ErrorSensor3_);}

        checkRelays();
        
        var[0]=STATUS_bits;
        for (byte i=0;i<4;i++)
        {
            yield();
            var[i+1]=*((byte*)&t1+3-i);

            var[i+5]=*((byte*)&h1+3-i);

            var[i+9]=*((byte*)&t2+3-i);

            var[i+13]=*((byte*)&h2+3-i);

            var[i+17]=*((byte*)&t3+3-i);

            var[i+21]=*((byte*)&h3+3-i);
        }
        String response = XML_response(&var[0],__SIZE_XML_RESPONSE_data__,pXML_RESP_data);
        
        server.send(200, "text/xml", response);  
    }
    
    if (configured_device && (abs(millis()-timeSample)>=__SAMPLE_TIME__) && !stopPolling)
    {
        unsigned long time1,time2;
        time1=millis();
        float h1 = dht1.readHumidity();
        yield();
        float t1 = dht1.readTemperature();// Read temperature as Celsius (the default)
        delay(100);
        float h2 = dht2.readHumidity();
        yield();
        float t2 = dht2.readTemperature();// Read temperature as Celsius (the default)
        delay(100);
        float h3 = dht3.readHumidity();
        yield();
        float t3 = dht3.readTemperature();// Read temperature as Celsius (the default)
        time2=millis();

        if (isnan(h1) || isnan(t1) ) 
        { 
                
        }
        else
        {
          h1_number++;
          t1_number++;
          h1_mean=h1_mean+(h1-h1_mean)/h1_number; // moving average
          t1_mean=t1_mean+(t1-t1_mean)/t1_number; // moving average
          STATUS_bits&=~(1<<0);
          Measure1OK=1;
        }
        yield();
        if (isnan(h2) || isnan(t2) ) 
        {
       
        }
        else
        {
          h2_number++;
          t2_number++;
          h2_mean=h2_mean+(h2-h2_mean)/h2_number; // moving average
          t2_mean=t2_mean+(t2-t2_mean)/t2_number; // moving average
          STATUS_bits&=~(1<<1);
          Measure2OK=1;
        }
        yield();
        if (isnan(h3) || isnan(t3) ) 
        {
      
        }
        else
        {
          h3_number++;
          t3_number++;
          h3_mean=h3_mean+(h3-h3_mean)/h3_number; // moving average
          t3_mean=t3_mean+(t3-t3_mean)/t3_number; // moving average
          STATUS_bits&=~(1<<2);
          Measure3OK=1;
        }
        timeSample=millis();
    }

    if (!configured_device && (abs(millis()-timeSample)>=__SAMPLE_TIME__) && !stopPolling)
    {
        unsigned long time1,time2;
        time1=millis();
        h1_number=1;
        t1_number=1;
        h2_number=1;
        t2_number=1;
        h3_number=1;
        t3_number=1;
        Measure1OK=1;
        Measure2OK=1;
        Measure3OK=1;
        float h1 = dht1.readHumidity();
        h1_mean=h1;
        float t1 = dht1.readTemperature();// Read temperature as Celsius (the default)
        t1_mean=t1;
        if (isnan(h1) || isnan(t1) ){Measure1OK=0;}
        delay(100);
        float h2 = dht2.readHumidity();
        h2_mean=h2;
        float t2 = dht2.readTemperature();// Read temperature as Celsius (the default)
        t2_mean=t2;
        if (isnan(h2) || isnan(t2) ){Measure2OK=0;}
        delay(100);
        float h3 = dht3.readHumidity();
        h3_mean=h3;
        float t3 = dht3.readTemperature();// Read temperature as Celsius (the default)
        t3_mean=t3;
        if (isnan(h3) || isnan(t3) ){Measure3OK=0;}
        time2=millis();
        Serial.println("NEW MEASUREMENT CYCLE:");
        if (isnan(h1) || isnan(t1) ) 
        { 
          Serial.println("Failed to read from DHT sensor 1!");     
        }else
        {
          Serial.print("SENSOR 1 Humidity: ");
          Serial.print(h1);
          Serial.print(" %\t");
          Serial.print("Temperature: ");
          Serial.println(t1);
        }
        if (isnan(h2) || isnan(t2) ) 
        { 
          Serial.println("Failed to read from DHT sensor 2!");     
        }else
        {
          Serial.print("SENSOR 2 Humidity: ");
          Serial.print(h2);
          Serial.print(" %\t");
          Serial.print("Temperature: ");
          Serial.println(t2);
        }
        if (isnan(h3) || isnan(t3) ) 
        { 
          Serial.println("Failed to read from DHT sensor 3!");     
        }else
        {
          Serial.print("SENSOR 3 Humidity: ");
          Serial.print(h3);
          Serial.print(" %\t");
          Serial.print("Temperature: ");
          Serial.println(t3);
        }
        timeSample=millis();
    }

    if (stopPolling&&(millis()-timeNoPoll>=__SAMPLE_TIME__)){stopPolling=false;Serial.println("Automatic polling restored");}
    server.handleClient();
}

void initialize_statics()
{
  t1_mean=0.;
  t2_mean=0.;
  t3_mean=0.;
  h1_mean=0.;
  h2_mean=0.;
  h3_mean=0.;
  t1_number=0;
  t2_number=0;
  t3_number=0;
  h1_number=0;
  h2_number=0;
  h3_number=0;
  Measure1OK=0;
  Measure2OK=0;
  Measure3OK=0;
}

String XML_response(byte *data, const int num_bytes, const char *pRESP)
{
    uint8_t d = 0;
    String frame="";
    for (int i = 0; i<num_bytes; i++)
    {
        if (*(pRESP + i) == '$') // data to be inserted
        {
            frame += String((*(data + d)));
            d++;
        }
        else if (*(pRESP + i) == '#') // DeviceCode to be inserted
        {
            frame+=String(DeviceCode);
        }
        else  // literal characters from XML structure
        {
            frame+=(*(pRESP + i));
        }
    }
    return frame;
}
