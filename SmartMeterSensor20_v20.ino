/////////////////////////////////////////////////////////////////////////////////////
// Name: MySensors node for Kamstrup smartmeter
// Version: 2.0
// Date: Sep 28, 2016
// Author: Jaap van Alphen
// Description:
// Exposes the following sensors:
// Id   Name                              Additional info
//  1   +T1                               Updated every hour
//  2   +T2                               Updated every hour
//  3   -T1                               Updated every hour
//  4   -T2                               Updated every hour
//  5   Tariff                            Updated on change of tariff
//  6   Current usage electricity         Updated every 10 seconds
//  7   Current production electricity    Updated every 10 seconds
//  8   Switch state                      Updated on each change of switch state
//  9   Gas reading                       Updated when time stamp changes (every hour)
//  10  Gas time stamp                    Time stamp of last gas reading
//
// This software comes as-is, you may alter this source code to your wish, but author is in no circumstance 
// responsible for any damage you make to your smart meter, or any other hardware.
//
// Connections:
// D6         DTR line to start output of readings from  meter
// D8         Serial input from smartmeter (default port for AltSoftSerial). 
//            Note that meter output sometimes needs to be inverted.
/////////////////////////////////////////////////////////////////////////////////////// 
#define MY_RADIO_NRF24

#define MY_DEBUG 
#define MY_NODE_ID 20

#include <AltSoftSerial.h>
#include <MySensors.h>
#include <SPI.h>


//#define NO_RADIO                      (1)       // Define this symbol when no radio is connected
#define EMETER_TIMEOUT                (600000) // Time between sending the electric meter standings
#define DTR_PIN                       (6)      // DTR pin to start Smartmeter

//  SensorDefinitions
#define NODE_ID                       (20)  // Node ID for the Smartmeter node
#define SENSOR_ID_PLUS_T1             (1)   // Sensor ID for Electric meter, Plus T1
#define SENSOR_ID_PLUS_T2             (2)   // Sensor ID for Electric meter, Plus T2
#define SENSOR_ID_MIN_T1              (3)   // Sensor ID for Electric meter, Min T1
#define SENSOR_ID_MIN_T2              (4)   // Sensor ID for Electric meter, Min T2
#define SENSOR_ID_CURRENT_TARIFF      (5)   // Sensor ID for Electric Current tariff
#define SENSOR_ID_CURRENT_USAGE       (6)   // Sensor ID for Electric current usage
#define SENSOR_ID_CURRENT_PRODUCTION  (7)   // Sensor ID for Electric Electric current production
//#define SENSOR_ID_SWITCH_STATE        (8)   // Sensor ID for Electric meter, switch state
#define SENSOR_ID_GAS_METER           (9)   // Sensor ID for Gas meter reading
//#define SENSOR_ID_GAS_TIMESTAMP       (10)  // Sensor ID for Gas meter, timestamp of last measurement

#define BUFSIZE 450 // Receive buffer size
#define DELAY_BETWEEN_PRESENT (150)

// Search filters in received data.
// NOTE: This data remains in Flash!
const char pT1_Filter[]     PROGMEM ={"1-0:1.8.1("};
const char pT2_Filter[]     PROGMEM ={"8.2("};
const char mT1_Filter[]     PROGMEM ={"1-0:2.8.1("};
const char mT2_Filter[]     PROGMEM ={"1-0:2.8.2("};
const char cT_Filter[]      PROGMEM ={"0-0:96.14.0("};
const char cUsageFilter[]   PROGMEM ={"1-0:1.7.0("};
const char curProdFilter[]  PROGMEM ={"1-0:2.7.0("};
const char cSwitchSFilter[] PROGMEM ={"0-0:96.3.10("};
const char cGasMFilter[]    PROGMEM ={"(0-1:24.2.1)(m3)"};
const char cGasTSFilter[]   PROGMEM ={"0-1:24.3.0("};

AltSoftSerial mySerial;


char buffer[BUFSIZE]; //Buffer for serial data to find.
int led = 13; //LED op pin 13. LED zal branden als er een telegram wordt uitgestuurd

// Global values
char          LastGasTimestamp[13];
int           LastCurTariff=0;
int           LastSwitchState=0;
unsigned long lastEMeters=0;
bool          firstRun=true;

// Gateway send functions
void SendFloat(int SensorId, int Precision, float Value, int units);
void SendBool(int SensorId, bool Value, int units);
void SendString(int SensorId, char* Value, int units);

// Data parse functions
void ParseCurrentTariff();
void ParseCurrentUsage();
void ParseEMeters();
bool ParseGasMeter();
void ParseSwitchState();

void SendFloat(int SensorId, int Precision, float Value, int units)
{
  bool bSucceeded = false;
  int ResendCount = 0;
  MyMessage msg(SENSOR_ID_PLUS_T1,units);
#ifndef NO_RADIO
  
  while (bSucceeded ==false)
  {
    bSucceeded =  send(msg.setSensor(SensorId).set(Value,Precision));
    if (bSucceeded == false)
    {
      if (ResendCount < 10)
      {
        delay(50);
        ResendCount++;
      }
      else
      {
        bSucceeded = true;
      }
    }
  }
#endif
  
}

void SendBool(int SensorId, bool Value, int units)
{
  bool bSucceeded = false;
  int ResendCount = 0;
  int IntValue=0;
  MyMessage msg(SENSOR_ID_PLUS_T1,units);

#ifndef NO_RADIO

  if (Value == false)
  {
    IntValue = 0;
  }
  else
  {
    IntValue = 1;
  }

  while (bSucceeded ==false)
  {
    bSucceeded =  send(msg.setSensor(SensorId).set(IntValue));
    if (bSucceeded == false)
    {
      if (ResendCount < 10)
      {
        delay(50);
        ResendCount++;
      }
      else
      {
        bSucceeded = true;
      }
    }
  }
#endif
  
}

void SendString(int SensorId, char* Value, int units)
{
  bool bSucceeded = false;
  int ResendCount = 0;
  MyMessage msg(SENSOR_ID_PLUS_T1,units);

#ifndef NO_RADIO
  
  while (bSucceeded ==false)
  {
    bSucceeded =  send(msg.setSensor(SensorId).set(Value));
    if (bSucceeded == false)
    {
      if (ResendCount < 10)
      {
        delay(50);
        ResendCount++;
      }
      else
      {
        bSucceeded = true;
      }
    }
  }
#endif

}

void ParseCurrentTariff()
{
  int  curTariff = 0;
  char* index=0;

  index = strstr_P(buffer, cT_Filter);
  if (index != 0)
  {
    index+=strlen_P(cT_Filter);

    if (sscanf(index,"%d%%*s" , &curTariff) >0 ) 
    {
      Serial.print(F("Huidig tarief:  "));
      Serial.println(curTariff);
      if ((curTariff != LastCurTariff) || (firstRun==true))
      {
        LastCurTariff = curTariff;
        firstRun = false;
        if (curTariff == 0)
        {
          SendBool(SENSOR_ID_CURRENT_TARIFF, false, V_VAR2);
        }
        else
        {
          SendBool(SENSOR_ID_CURRENT_TARIFF, true, V_VAR2);
        }
        
      }
    }
 
  }

}
void ParseCurrentUsage()
{
  long curUsage = 0;
  char* index=0;
  long tl = 0;
  long tld =0;

  index = strstr_P(buffer, cUsageFilter);

  if (index != 0)
  {
    index+=strlen_P(cUsageFilter);

    // 1-0:1.7.0 = Electricity usage (W)
   if (sscanf(index,"%ld.%ld%*s" , &tl , &tld) >0 ) 
   {
     curUsage = tl * 1000 + tld * 10;
     Serial.print(F("Elektra - actueel verbruik (W): "));
     Serial.println(curUsage);
     SendFloat(SENSOR_ID_CURRENT_USAGE, 0, curUsage, V_WATT);
//     gw.send(msg.setSensor(SENSOR_ID_CURRENT_USAGE).set(curUsage,0));  // Send sensor info
   }
  }

  index = strstr_P(buffer, curProdFilter);
  if (index != 0)
  {
    index+=strlen_P(curProdFilter);

    // 1-0:2.7.0 = Electricity production (W)
   if (sscanf(index,"%ld.%ld%*s" , &tl , &tld) >0 ) 
   {
     curUsage = tl * 1000 + tld * 10;
     Serial.print(F("Elektra - actueel productie (W): "));
     Serial.println(curUsage);
     SendFloat(SENSOR_ID_CURRENT_PRODUCTION, 0, curUsage, V_WATT);
//     gw.send(msg.setSensor(SENSOR_ID_CURRENT_PRODUCTION).set(curUsage,0));  // Send sensor info
   }
  }
  
}

void ParseEMeters()
{
  float fT = 0.0; //Meter reading Electrics - consumption low tariff
  char* index=0;
  long tl = 0;
  long tld =0;

  index = strstr_P(buffer, pT1_Filter);
  if (index != 0)
  {
    index+=strlen_P(pT1_Filter);

    // 1-0:1.8.1 = Electricity consumption low tariff (DSMR v4.0)
    
    if (sscanf(index,"%ld%.%ld%*s" , &tl, &tld) >0 ) 
    {
      fT = (tl*1000 + tld) / 1000;
         Serial.print(F("Elektra - meterstand +T1 (Wh): "));
         Serial.println(fT);
      SendFloat(SENSOR_ID_PLUS_T1, 1, fT, V_KWH);   
      //gw.send(msg.setSensor(SENSOR_ID_PLUS_T1).set(fT,1));  // Send sensor info
    }
 
  }
  index = strstr_P(buffer, pT2_Filter);
  if (index != 0)
  {
    index+=strlen_P(pT2_Filter);

    // 8.2 = Electricity consumption high tariff (DSMR v4.0)
    if (sscanf(index,"%ld%.%ld%*s" , &tl, &tld) >0 ) 
    {
      fT = (tl * 1000 + tld) / 1000;
      Serial.print(F("Elektra - meterstand +T2 (Wh): "));
      Serial.println(fT);
      SendFloat(SENSOR_ID_PLUS_T2, 1, fT, V_KWH);   
      //gw.send(msg.setSensor(SENSOR_ID_PLUS_T2).set(fT,1));  // Send sensor info
    }
  }
  index = strstr_P(buffer, mT1_Filter);
  if (index != 0)
  {
    index+=strlen_P(mT1_Filter);

    // 8.2 = Electricity production high tariff (DSMR v4.0)
    if (sscanf(index,"%ld%.%ld%*s" , &tl, &tld) >0 ) 
    {
      fT = (tl * 1000 + tld)/1000;
      Serial.print(F("Elektra - meterstand -T1 (Wh): "));
      Serial.println(fT);
      SendFloat(SENSOR_ID_MIN_T1, 1, fT, V_KWH);   
      //gw.send(msg.setSensor(SENSOR_ID_MIN_T1).set(fT,1));  // Send sensor info
    }
 
  }

  index = strstr_P(buffer, mT2_Filter);
  if (index != 0)
  {
    index+=strlen_P(mT2_Filter);

    // 8.2 = Electricity production high tariff (DSMR v4.0)
    if (sscanf(index,"%ld%.%ld%*s" , &tl, &tld) >0 ) 
    {
      fT = (tl * 1000 + tld)/1000;
      Serial.print(F("Elektra - meterstand -T2 (Wh): "));
      Serial.println(fT);
      SendFloat(SENSOR_ID_MIN_T2, 1, fT, V_KWH);
      //gw.send(msg.setSensor(SENSOR_ID_MIN_T2).set(fT,1));  // Send sensor info
    }
  }
}

bool ParseGasMeter()
{
  char* index=0;
  float fT = 0.0; //Meter reading Electrics - consumption low tariff
  long tl = 0;
  long tld =0;
  bool bSucceeded=false;
  int ResendCount=0;
  char gasTimestamp[13];
  bool bUpdateReadings = false;

  memset(gasTimestamp, '\0', 13);
  index = strstr_P(buffer, cGasMFilter);
  if (index != 0)
  {
    index+=strlen_P(cGasMFilter);
    index = strstr(index, "(");
    index+=strlen("(");
  // (0-1:24.2.1)(m3)\n( = Gasmeter stand
   if (sscanf(index,"%ld.%ld)" , &tl , &tld) >0 ) 
   {
     fT = (tl * 1000 + tld)/1000;
     Serial.print(F("Gas - stand (m3): "));
     Serial.println(fT);
   }
  }  
  index = strstr_P(buffer, cGasTSFilter);
  if (index != 0)
  {
    index+=strlen_P(cGasTSFilter);
    strncpy(gasTimestamp, index, 12); 
    Serial.print(F("Gas - TIMESTAMP: "));
    Serial.println(gasTimestamp);

     if (strcmp(LastGasTimestamp, gasTimestamp))
     {
      strcpy(LastGasTimestamp, gasTimestamp);
      SendFloat(SENSOR_ID_GAS_METER, 3, fT,V_VAR1);
//      SendString(SENSOR_ID_GAS_TIMESTAMP, gasTimestamp);
      bUpdateReadings = true;
     }
  }
  return bUpdateReadings;
}
/*
void ParseSwitchState()
{
  char* index=0;
  long tl = 0;
  long tld =0;
  int switchState=0;

  index = strstr_P(buffer, cSwitchSFilter);
  if (index != 0)
  {
    index+=strlen_P(cSwitchSFilter);

    if (sscanf(index,"%d%%*s" , &switchState) >0 ) 
    {
      Serial.print(F("Huidig Switch state:  "));
      Serial.println(switchState);
      if ((switchState != LastSwitchState) || (firstRun==true))
      {
        LastSwitchState = switchState;
        SendBool(SENSOR_ID_SWITCH_STATE, switchState);
      }
    }
   }
}
*/
void presentation()
{
#ifndef NO_RADIO
//  gw.begin(NULL, 20);

  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Smartmeter", "0.2");

  present(SENSOR_ID_PLUS_T1, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
  present(SENSOR_ID_PLUS_T2, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
  present(SENSOR_ID_MIN_T1, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
  present(SENSOR_ID_MIN_T2, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
  present(SENSOR_ID_CURRENT_TARIFF, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
  present(SENSOR_ID_CURRENT_USAGE, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
  present(SENSOR_ID_CURRENT_PRODUCTION, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
//  present(SENSOR_ID_SWITCH_STATE, S_POWER);
//  delay(DELAY_BETWEEN_PRESENT);
  present(SENSOR_ID_GAS_METER, S_POWER);
  delay(DELAY_BETWEEN_PRESENT);
//  present(SENSOR_ID_GAS_TIMESTAMP, S_POWER);
//  delay(DELAY_BETWEEN_PRESENT);
  present(254, S_BINARY);
#endif
  
  
}

void receive(const MyMessage &message)
{
  if((message.type ==V_STATUS) && (message.sensor == 254))
  {
    Serial.println("Reset request received!");

    wdt_disable();  
    wdt_enable(WDTO_15MS);
    while (1) {}
  }
  
}
void setup()
{
  mySerial.begin(9600); //P1 baudrate
  Serial.begin(115200); //Serial console baudrate

  Serial.println(F("Smart meter sensor for MySensors"));
  Serial.println(F("- entering setup()"));
  
  pinMode(DTR_PIN, OUTPUT);
  // Startup and initialize MySensors library. Set callback for incoming messages. 
  memset(LastGasTimestamp, '\0', 13); 

  digitalWrite(DTR_PIN, 1);

  Serial.println(F("- exit setup()"));
}
       
void loop() 
{ 
  unsigned long currentMillis;
  char input; // incoming serial data (byte)
  char* StartofLine=buffer;
  static int bufpos = 0;
  int readval=0;

  if (mySerial.available()) 
  {
    input = mySerial.read();    
    // --- 7 bits setting ---
    input &= ~(1 << 7);
    char inChar = (char)input;
    // --- 7 bits setting ---u

    buffer[bufpos] = inChar;
    bufpos++;
         
//    Serial.print(inChar); //Debug
    if (inChar == '!')
    {
      Serial.println("end of frame!");
      Serial.print("Received: ");
      Serial.print(buffer);
      buffer[bufpos] = '\0';

      Serial.print("Length of packet: ");
      Serial.println(strlen(buffer));
      if (strlen(buffer) != 0)
      {
        ParseCurrentUsage();
        delay(DELAY_BETWEEN_PRESENT);
        ParseCurrentTariff();
        delay(DELAY_BETWEEN_PRESENT);
        
        if (ParseGasMeter())
        {
          delay(DELAY_BETWEEN_PRESENT);
          ParseEMeters();    

//        delay(20);
//        ParseSwitchState();
        }
      }
        bufpos=0;
  
    }
    
  }
  
}
