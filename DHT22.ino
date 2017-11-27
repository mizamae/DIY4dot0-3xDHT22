#include <DHT.h>

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// DHT Sensor
const int DHTPin1 = 0;
const int DHTPin2 = 2;
const int DHTPin3 = 4;
// Initialize DHT sensor.
DHT dht1(DHTPin1, DHTTYPE);
DHT dht2(DHTPin2, DHTTYPE);
DHT dht3(DHTPin3, DHTTYPE);
static float t1_mean=0,h1_mean=0,t2_mean=0,h2_mean=0,t3_mean=0,h3_mean=0;
static int t1_number=0,h1_number=0,t2_number=0,h2_number=0,t3_number=0,h3_number=0;
static byte STATUS_bits=0,MeasureOK=0;
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <SPI.h>                         // Include the SPI library

// MAC address from Ethernet shield sticker under board
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0x41, 0x42, 0x21 }; // make sure you change these values so that no MAC collision exist in your network
const char* ssid = "diy4dot0";
const char* password = "Valtierra1981";

uint8_t IPaddr[4];
uint8_t DeviceCode;
const String DeviceType="3xDHT22"; 
float T1,T2,T3;
float H1,H2,H3;
float DewPoint_centdeg;


WiFiServer server(80);  // create a server at port 80
WiFiClient client;
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
  const char  _HTML_RESPONSE_OK_[16];
  const char _BLANKLINE_ ;
  
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
                 {'H','T','T','P','/','1','.','1',' ','2','0','0',' ','O','K','\n'},
                  '\n',
                 {'<','?','x','m','l',' ','v','e','r','s','i','o','n',' ','=',' ','"','1','.','0','"',' ','?','>','\n'},
                 {'<','X','>'},
                 {'<','D','E','V','T','>'},{'3','x','D','H','T','2','2'},{'<','/','D','E','V','T','>'},
                 {'<','D','E','V','C','>'},{'$'},{'<','/','D','E','V','C','>'},
                 {'<','/','X','>','\n'}  
                };

const char *pXML_RESP_Conf    =    &xml_response_Conf._HTML_RESPONSE_OK_[0];
const int __SIZE_XML_RESPONSE_Conf__    =    sizeof(xml_response_Conf);

struct XML_RESPONSES_data
{
  const char  _HTML_RESPONSE_OK_[16];
//  const char _KEEP_ALIVE_[30];
//  const char _HTML_CONN_ALIVE_[23];
//  const char _XML_CONTENT_TYPE_[30];
  //  const char _USERAGENT_[24];
  const char _BLANKLINE_ ;

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
                 {'H','T','T','P','/','1','.','1',' ','2','0','0',' ','O','K','\n'},
//                 {'K','e','e','p','-','A','l','i','v','e',':',' ','t','i','m','e','o','u','t','=','5',',',' ','m','a','x','=','9','9','\n'},
//                 {'C','o','n','n','e','c','t','i','o','n',':',' ','K','e','e','p','-','A','l','i','v','e','\n'},
//                 {'C','o','n','t','e','n','t','-','T','y','p','e',':',' ','a','p','p','l','i','c','a','t','i','o','n','/','x','m','l','\n'},
//                  {'U','s','e','r','-','A','g','e','n','t',':',' ','M','o','z','i','l','l','a','/','5','.','0','\n'},
                 '\n',
                 {'<','?','x','m','l',' ','v','e','r','s','i','o','n',' ','=',' ','"','1','.','0','"',' ','?','>','\n'},
                 {'<','X','>'},
                 {'<','D','E','V','>'},{'#'},{'<','/','D','E','V','>'},
                 {'<','D','I','d','>'},{'0'},{'<','/','D','I','d','>'},
                 {var_tag},{var_tag_float,var_tag_float,var_tag_float,var_tag_float,var_tag_float,var_tag_float},
                 {'<','/','X','>','\n'}  
                };

const char *pXML_RESP_data = &xml_response_data._HTML_RESPONSE_OK_[0];
const int __SIZE_XML_RESPONSE_data__ = sizeof(xml_response_data);


// EEPROM structure
const uint8_t _EEPROMaddrDEVICECODE_      = 0;     // address for the deviceCode in EEPROM
const uint8_t _EEPROMaddrIP_B0_           = 1;     // address for the LSB of IP address in EEPROM
const uint8_t _EEPROMaddrIP_B1_           = 2;     // address for the 2nd Byte of IP address in EEPROM
const uint8_t _EEPROMaddrIP_B2_           = 3;     // address for the 3rd Byte of IP address in EEPROM
const uint8_t _EEPROMaddrIP_B3_           = 4;     // address for the MSB of IP address in EEPROM


// CONFIG BUTON
const byte _CONF_BTN_                    = 0;        // pin to initiate config procedure

static bool configured_device=false;
static unsigned long timeSample;
const unsigned long _SAMPLETIME_        = 10000;    // sample time in ms 
void setup()
{
    Serial.begin(115200);
    
    EEPROM.begin(512);
    DeviceCode = EEPROM.read(_EEPROMaddrDEVICECODE_);
    Serial.print("DeviceCode in EEPROM: ");
    Serial.println(DeviceCode);
    //DeviceCode=255;
    
    if ((DeviceCode==255)||(DeviceCode==254)||!digitalRead(_CONF_BTN_)) // the device has not been configured. This is so in Arduino devices where EEPROM defaults to 255. In ESP8266 flash is randomly defaulted!!!
    {
        DeviceCode=254;
        IPaddr[0]=254;
        IPaddr[1]=10;
        IPaddr[2]=10;
        IPaddr[3]=10;
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
     
    // Start the server
    server.begin();
    Serial.println("Server started");
   
    // Print the IP address
    Serial.print("Use this URL to connect: ");
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    dht1.begin();
    dht2.begin();
    dht3.begin();
    timeSample=millis();
}

void loop()
{
    client = server.available();  // try to get client
    
    if (client && (MeasureOK > 0))  // got client?
    {
        unsigned long time1,time2;
        Serial.println("new client");
        time1=millis();
        uint8_t timeout=0;
        
        while((!client.available()) && (timeout<10)){
            delay(1);
            timeout++;
        }
        processREQWIFI(&baseREQ,&orderREQ,&webREQ);
        yield();
        if (baseREQ||orderREQ||webREQ)
        {
            responseREQ(&baseREQ,&orderREQ,&webREQ);
        }
        
        client.stop(); // close the connection
        time2=millis();
        Serial.print("HTTP request took  ");
        Serial.print(time2-time1);
    }else if (configured_device && (abs(millis()-timeSample)>=_SAMPLETIME_))
    {
       unsigned long time1,time2;
       time1=millis();
       float h1 = dht1.readHumidity();
       float t1 = dht1.readTemperature();// Read temperature as Celsius (the default)
       float h2 = dht2.readHumidity();
       float t2 = dht2.readTemperature();// Read temperature as Celsius (the default)
       float h3 = dht3.readHumidity();
       float t3 = dht3.readTemperature();// Read temperature as Celsius (the default)
       time2=millis();
       
       if (isnan(h1) || isnan(t1) ) { 
              STATUS_bits|=1;      
            }
       else{
              h1_number++;
              t1_number++;
              h1_mean=h1_mean+(h1-h1_mean)/h1_number; // moving average
              t1_mean=t1_mean+(t1-t1_mean)/t1_number; // moving average
              STATUS_bits&=~(1<<0);
              MeasureOK=1;
            }
       if (isnan(h2) || isnan(t2) ) {
              STATUS_bits|=1<<1;         
            }
       else{
              h2_number++;
              t2_number++;
              h2_mean=h2_mean+(h2-h2_mean)/h2_number; // moving average
              t2_mean=t2_mean+(t2-t2_mean)/t2_number; // moving average
              STATUS_bits&=~(1<<1);
              MeasureOK=1;
            }
       if (isnan(h3) || isnan(t3) ) {
              STATUS_bits|=1<<2;        
            }
       else{
              h3_number++;
              t3_number++;
              h3_mean=h3_mean+(h3-h3_mean)/h3_number; // moving average
              t3_mean=t3_mean+(t3-t3_mean)/t3_number; // moving average
              STATUS_bits&=~(1<<2);
              MeasureOK=1;
            }
        timeSample=millis();
    }else // the device is not configured
    {
      
    }
}


void processREQWIFI(byte *baseREQ,byte *orderREQ,byte *webREQ)
{
    // HTML BUFFER CONFIGURATION
    #define REQ_BUF_SZ   60
    char HTTP_req[REQ_BUF_SZ] = { 0 }; // buffered HTTP request stored as null terminated string
    unsigned int req_index = 0;              // index into HTTP_req buffer

    unsigned long time_ini = millis();
    boolean buffer_overflow = false;
    boolean currentLineIsBlank = true;
    if (client.available()) {   // client data available to read
        // last line of client request is blank and ends with 
        String request = client.readStringUntil('\r');
        client.flush();
        Serial.println(request);

        if (StrContains(&request[0], "orders"))
          {
                if (StrContains(&request[0], "SetConf"))// repeat this conditional for the different orders to be received             
                {// request="POST /orders/SetConf.htm?DEVC=2 HTTP/1.1"
                    *orderREQ = 10;
                    uint8_t index=0;
                    while (request[index]!='='){index++;}
                    Serial.print("Found = at position ");
                    Serial.println(index);
                    index++;
                    uint8_t code[3];
                    uint8_t i=0;
                    while (request[index]!=' ')
                    {
                      code[i]=request[index]-48; // to obtain the number from the ascii char
                      index++;
                      i++;
                    }
                    if (i==1){DeviceCode=code[0];}
                    if (i==2){DeviceCode=code[0]*10+code[1];}
                    if (i==3){DeviceCode=code[0]*100+code[1]*10+code[2];}
                }
                
                if (StrContains(&request[0], "D1_ON"))
                {
                    *orderREQ=11;
                }
                
                if (StrContains(&request[0], "D1_OFF"))
                {
                    *orderREQ=12;
                }
                
                if (StrContains(&request[0], "LOG=1"))
                {
                    *orderREQ=13;
                }
                if (StrContains(&request[0], "resetStatics"))
                {
                    *orderREQ=14;
                    Serial.println("Requested reset statics");
                }
                    
          } 
          
          if (StrContains(&request[0], "Conf.xml"))
          {
            *baseREQ = 1;
          }
          
          if (StrContains(&request[0], "data.xml"))
          {
            *baseREQ = 2;
            //Serial.println("Requested powers");
          }

     }        
}     

void responseREQ(byte *baseREQ,byte *orderREQ,byte *webREQ)
{
    byte var[25];   
    byte kk;
    switch (*orderREQ)
    {
        case 10: // orders from the controller to setup the new configuration

            *orderREQ=0;
            EEPROM.write(_EEPROMaddrIP_B0_,DeviceCode);
            EEPROM.write(_EEPROMaddrIP_B1_,10);
            EEPROM.write(_EEPROMaddrIP_B2_,10);
            EEPROM.write(_EEPROMaddrIP_B3_,10);
            EEPROM.write(_EEPROMaddrDEVICECODE_,DeviceCode);
            EEPROM.end();
            Serial.print("DeviceCode to be set at ");
            Serial.println(DeviceCode);
            
            XML_response2(&kk,__SIZE_RESPONSE_OK__,pRESP_OK); // sends the status of all switches
            break;  
        
        case 11: // Execution for Activate digital pin 1 
        
            *orderREQ=0;
            break;  
        
        case 12: // Execution for Deactivate digital pin 1 
        
            *orderREQ=0;
            break;  
        
        case 13: // Execution for Start logging 
        
            *orderREQ=0;
            break;
        case 14: // Execution for resetStatics
            XML_response2(&kk,__SIZE_RESPONSE_OK__,pRESP_OK); // sends the status of all switches
            initialize_statics();
            *orderREQ=0;
            break;    
    
    }
    switch (*baseREQ)
    {
        case 1: // request of the configuration "Conf.xml"
          XML_response2(&DeviceCode,__SIZE_XML_RESPONSE_Conf__,pXML_RESP_Conf); // sends the status of all switches
          delay(10);
          *baseREQ=0;
          break; 
            
        case 2:   // request for "powers.xml"
          
          var[0]=STATUS_bits;
          for (byte i=0;i<4;i++)
          {
             
             var[i+1]=*((byte*)&t1_mean+3-i);

             var[i+5]=*((byte*)&h1_mean+3-i);

             var[i+9]=*((byte*)&t2_mean+3-i);

             var[i+13]=*((byte*)&h2_mean+3-i);

             var[i+17]=*((byte*)&t3_mean+3-i);

             var[i+21]=*((byte*)&h3_mean+3-i);
          }
          XML_response2(&var[0],__SIZE_XML_RESPONSE_data__,pXML_RESP_data);
          *baseREQ=0;
          break;               

    }
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
  STATUS_bits=0;
  MeasureOK=0;
}

byte XML_response2(byte *data, const int num_bytes, const char *pRESP)
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
    Serial.println(frame);
    client.print(frame);
}


// sets every element of str to 0 (clears array)
void StrClear(char *str, char length)
{
    for (int i = 0; i < length; i++) {
        str[i] = 0;
    }
}

// searches for the string sfind in the string str
// returns 1 if sfind is found in str
// returns 0 if sfind is not found in str
byte StrContains(char *str, char *sfind)
{
    char found = 0;
    char index = 0;
    char lenstr, lensfind;

    lenstr = strlen(str);
    lensfind = strlen(sfind);
    
    while (index < lenstr) 
    {
        if (str[index] == sfind[found]) 
        {
            found++;
            if (lensfind == found) 
            {
                return 1;
            }
        }
        else 
        {
            found = 0;
        }
        index++;
    }

    return 0;
}        
