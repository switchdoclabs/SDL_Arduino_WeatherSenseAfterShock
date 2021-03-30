// SDL_Arduino_WeatherSenseAfterShock
// SwitchDoc Labs EarthQuake Detector
//



#define TXDEBUG
//#undef TXDEBUG
#include <JeeLib.h>

#include "MemoryFree.h"


// WeatherSenseProtocol of 8 is SolarMAX LiPo   BatV < 7V
// WeatherSenseProtocol of 10 is SolarMAX LeadAcid   BatV > 7V LoRa version
// WeatherSenseProtocol of 11 is SolarMAX4 LeadAcid BatV > 7V
// WeatherSenseProtocol of 15 is WeatherSense AQI 433MHz
// WeatherSenseProtocol of 16 is WeatherSense ThunderBoard 433MHz
// WeatherSenseProtocol of 17 is Generic data
// WeatherSenseProtocol of 18 is WeatherSense AfterShock

#define WEATHERSENSEPROTOCOL 18

#define LED 13
// Software version
#define SOFTWAREVERSION 1

// unique ID of this WeatherSenseAfterShock system - change if you have multiple WeatherSenseThunderBoard systems
#define WEATHERSENESTHBID 1
// Which WeatherSense TAfterShock Protocol Version
#define WEATHERSENSEPROTOCOLVERSION 1

// ThunderBoard


// Device ID is changed if you have more than one WeatherSense ThunderBoard in the area

// Number of milliseconds between wake up  30 seconds.  Every 30 wakeups send packet  - if you move this over 60000 ms, you will need to add the watchdog in the sleep loop - see SDL_Arduino_WeatherSenseAQI ResetWatchDog
#define SLEEPCYCLE 30000
#define WAKEUPS 30
//#define SLEEPCYCLE 14000

#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

#include <RH_ASK.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include "SDL_Arduino_INA3221.h"


SDL_Arduino_INA3221 INA3221;



// the three channels of the INA3221 named for INA3221 Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3


// Other Pins
#define WATCHDOG_1 5

#define TXPIN 8
#define RXPIN 9



// Number of milliseconds between data ou






RH_ASK driver(2000, RXPIN, TXPIN);

unsigned long MessageCount = 0;



#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>


#include <Wire.h>

typedef enum  {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;


// Device Present State Variables

bool INA3221_Present;

bool AfterShock_Present;

byte byteBuffer[100]; // contains string to be sent to RX unit

// State Variables

long TimeStamp;



// State Status

float BatteryVoltage;
float BatteryCurrent;
float LoadVoltage;
float LoadCurrent;
float SolarPanelVoltage;
float SolarPanelCurrent;
byte AuxA;
byte SoftwareVersion;

// AuxA has state information
// coded in the byte
// 0000DCBA


// A = 1, AftereShock Present, 0 not present
// B = 1, IN3221 (Solar) Present, 0 not present
// C = 1, Low battery, Lighting Chip shut off (too many false alarms in low voltage mode)
// D = 1, I'm alive message





wakestate wakeState;  // who woke us up?


long nextSleepLength;





int convert4ByteLongVariables(int bufferCount, long myVariable)
{

  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;

}

int convert4ByteFloatVariables(int bufferCount, float myVariable)
{
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];


    bufferCount++;
  }

  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable)
{


  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;


  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;

  return bufferCount;

}

int convert1ByteVariables(int bufferCount, int myVariable)
{


  byteBuffer[bufferCount] = (byte) myVariable;
  bufferCount++;
  return bufferCount;

}

int checkSum(int bufferCount)
{
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}




// variables

long EQCount = 0;

float last5_0_SI = 0.0;
float last5_0_PGA = 0.0;
float highInstantaneousSI = 0.0;
float highInstantaneousPGA = 0.0;






int buildProtocolMessage()
{

  int bufferCount;


  bufferCount = 0;

  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);


  byteBuffer[bufferCount] = WEATHERSENESTHBID; // WeatherSenseAfterShock unique ID
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOL; // Type of WeatherSense System
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOLVERSION; // WeatherSense AfterShock protocol version
  bufferCount++;

  // EQ Count
  // Saved SI
  // Saved PGA
  // Instan SI
  // Instan PGA

  bufferCount = convert4ByteLongVariables(bufferCount, EQCount);

  bufferCount = convert4ByteFloatVariables(bufferCount, last5_0_SI);
  bufferCount = convert4ByteFloatVariables(bufferCount, last5_0_PGA);
  bufferCount = convert4ByteFloatVariables(bufferCount, highInstantaneousSI);
  bufferCount = convert4ByteFloatVariables(bufferCount, highInstantaneousPGA);



  bufferCount = convert4ByteFloatVariables(bufferCount, LoadVoltage);  // Solar Data
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);



  byteBuffer[bufferCount] = AuxA;  // Aux
  bufferCount++;
  byteBuffer[bufferCount] =  SOFTWAREVERSION;
  bufferCount++;



  return bufferCount;


}





void return2Digits(char returnString[], char *buffer2, int digits)
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);
}



void ResetWatchdog()
{


  digitalWrite(WATCHDOG_1, LOW);
  delay(200);
  digitalWrite(WATCHDOG_1, HIGH);

#if defined(TXDEBUG)
  Serial.println(F("Watchdog1 Reset - Patted the Dog"));
#endif

}



#define ENABLE 4
#define CONTROL 3


//
//
//
#include "D7S.h"

#define INT1_PIN 2 //Arduino pin connected to the INT1 pin of the D7S sensor
#define INT2_PIN 3 //Arduino pin connected to the INT2 pin of the D7S sensor



//flag variables to handle collapse/shutoff only one time during an earthquake


volatile bool EQInProgress = false;
volatile bool PreviousEQInProgress = false;

//function to handle the start of an earthquake
void EarthquakeHandler() {



  if (digitalRead(INT2_PIN) == 0)
  {

    PreviousEQInProgress = EQInProgress;
    EQInProgress = true;

  }
  else
  {

    PreviousEQInProgress = EQInProgress;
    EQInProgress = false;


  }
  Serial.print(F("IN2_PIN="));
  Serial.println(digitalRead(INT2_PIN));
  Serial.print(F("After Prev/Current="));
  Serial.print(PreviousEQInProgress);
  Serial.print(F(" / "));

  Serial.println(EQInProgress);
}



void printLast5()
{
  int i;
  float currentValue;
  for (i = 0; i < 5; i++)
  {
    currentValue = D7S.getSIDataLast(i);
    Serial.print("Last 5 - Element=");
    Serial.print(i);
    Serial.print("  SI Value: ");
    Serial.println(currentValue);

    currentValue = D7S.getPGADataLast(i);

    Serial.print("Last 5 - Element=");
    Serial.print(i);
    Serial.print("  PGA Value: ");
    Serial.println(currentValue);

    currentValue = D7S.getTempDataLast(i);

    Serial.print("Last 5 - Element=");
    Serial.print(i);
    Serial.print("  Temp Value: ");
    Serial.println(currentValue);


  }
}

void printGreatest5()
{
  int i;
  float currentValue;
  for (i = 0; i < 5; i++)
  {
    currentValue = D7S.getSIDataGreatest(i);
    Serial.print("Greatest 5 - Element=");
    Serial.print(i);
    Serial.print("  SI Value: ");
    Serial.println(currentValue);

    currentValue = D7S.getPGADataGreatest(i);

    Serial.print("Greatest 5 - Element=");
    Serial.print(i);
    Serial.print("  PGA Value: ");
    Serial.println(currentValue);

    currentValue = D7S.getTempDataGreatest(i);

    Serial.print("Greatest 5 - Element=");
    Serial.print(i);
    Serial.print("  Temp Value: ");
    Serial.println(currentValue);

  }

}

String getInt1Int2()
{

  String Value;
  Value = "";

  if (digitalRead(INT1_PIN)  == 0)
  {
    Value = "0";
  }
  else
  {
    Value = "1";
  }

  if (digitalRead(INT2_PIN)  == 0)
  {
    Value = Value + "0";
  }
  else
  {
    Value = Value + "1";
  }
  return Value;

}
int state = 0;
unsigned long wakeCount;

void sendMessage(float si, float pga)
{



#if defined(TXDEBUG)
  Serial.println(F("###############"));
  Serial.print(F(" MessageCount="));
  Serial.println(MessageCount);
  Serial.print(F(" STATUS - WeatherSenseProtocol:"));
  Serial.println(WEATHERSENSEPROTOCOL);
  Serial.print(F(" WakeState="));
  Serial.println(wakeState);
  Serial.print(F(" wakeCount="));
  Serial.println(wakeCount);


  Serial.print(F("EQ Count: "));
  Serial.println(EQCount);
  Serial.print(F("Saved SI: "));
  Serial.println(si);
  Serial.print(F("Saved PGA:"));
  Serial.println(pga);
  Serial.print(F("High Instantaneous SI:"));
  Serial.println(highInstantaneousSI);
  Serial.print(F("High Instantaneous PGA:"));
  Serial.println(highInstantaneousPGA);





  Serial.print(F(" Battery Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
  Serial.print(F(" Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
  Serial.print(F(" Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
  Serial.print(F(" Solar Current:  ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
  Serial.print(F(" Load Voltage:  ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
  Serial.print(F(" Load Current:       ")); Serial.print(LoadCurrent); Serial.println(" mA");
  Serial.print(F(" Currentmillis() = "));
  Serial.println(millis());

  Serial.print(F("  AuxA State:"));
  Serial.print(AuxA);
  Serial.print(F(" "));
  Serial.println(AuxA, HEX);

  Serial.println(F("###############"));
#endif
  // write out the current protocol to message and send.
  int bufferLength;


  Serial.println(F("----------Sending packets----------"));
  bufferLength = buildProtocolMessage();

  // Send a message



  Serial.print(F("bufferlength="));
  Serial.println(bufferLength);

  detachInterrupt(digitalPinToInterrupt(INT2_PIN));

  driver.send(byteBuffer, bufferLength);

  if (!driver.waitPacketSent(6000))
  {
    //Serial.println(F("Timeout on transmission"));
    // re-initialize board
    if (!driver.init())
    {
      //Serial.println(F("init failed"));
      while (1);
    }
    //Serial.println(F("----------Board Reinitialized----------"));
  }

  EIFR |= bit(INTF1); // clear INT1 interrupt flag
  attachInterrupt(digitalPinToInterrupt(INT2_PIN), &EarthquakeHandler, CHANGE);


  Serial.println(F("----------After Sending packet----------"));

  for (int i = 0; i < bufferLength; i++) {
    Serial.print(" ");
    if (byteBuffer[i] < 16)
    {
      Serial.print(F("0"));
    }
    Serial.print(byteBuffer[i], HEX);           //  write buffer to hardware serial port
  }
  Serial.println();


  Serial.println(F("----------After Wait Sending packet----------"));
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);

  MessageCount++;


}



void setup()
{



  Serial.begin(115200);    // TXDEBUGging only
  // Pat the WatchDog
  ResetWatchdog();
  wakeCount = 0;

  AuxA = 0x00;
  // turn on USB Power for power check.

  Serial.println();
  Serial.println();
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println(F("WeatherSense AfterShock"));
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);
  Serial.print(F("Unit ID:"));
  Serial.println(WEATHERSENESTHBID);

  if (!driver.init())
  {
    Serial.println(F("init failed"));
    while (1);
  }

  Serial.print("max message length=");
  Serial.println(driver.maxMessageLength());







  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);

  // setup initial values of variables

  wakeState = REBOOT;

  nextSleepLength = SLEEPCYCLE;



  TimeStamp = 0;


  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;







  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);


  Wire.begin();




  // test for INA3221_Present
  INA3221_Present = false;



  int MIDNumber;
  INA3221.wireReadRegister(0xFE, &MIDNumber);
  Serial.print(F("Manuf ID:   0x"));
  Serial.print(MIDNumber, HEX);
  Serial.println();

  if (MIDNumber != 0x5449)
  {
    INA3221_Present = false;
    Serial.println(F("INA3221 Not Present"));
  }
  else
  {
    INA3221_Present = true;

    // State Variable
    AuxA = AuxA | 0X02;
  }

  // look for AfterShock at 0x55

  int error;
  Wire.beginTransmission(0x55);
  error = Wire.endTransmission();
  AfterShock_Present = false;

  if (error == 0)
  {
    Serial.println("AfterShock device found");
    AfterShock_Present = true;
    // State Variable
    AuxA = AuxA | 0X01;
  }
  else if (error == 4)
  {
    Serial.println("AfterShock device Not Found");
    AfterShock_Present = false;
  }

  Serial.print(F("Starting AfterShock communications (it may take some time)..."));
  //start D7S connection
  D7S.begin();
  D7S.clearEarthquakeData();
  D7S.zeroClear();

  //D7S.setThreshold(THRESHOLD_LOW);
  //wait until the D7S is ready
  while (!D7S.isReady())
  {
    Serial.print(F("."));
    delay(500);
  }
  Serial.println(F("AfterShock STARTED"));

  //setting the D7S to switch the axis at inizialization time
  Serial.println(F("Setting AfterShock sensor to switch axis at inizialization time."));
  D7S.setAxis(SWITCH_AT_INSTALLATION);

  Serial.println(F("Initializing the AfterShock sensor in 2 seconds. Please keep it steady during the initializing process."));
  delay(2000);
  Serial.print(F("Initializing..."));
  //start the initial installation procedure
  D7S.initialize();
  //wait until the D7S is ready (the initializing process is ended)
  while (!D7S.isReady())
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("INITIALIZED!");




  //--- INTERRUPT SETTINGS ---


  pinMode(INT2_PIN, INPUT);
  digitalWrite(INT2_PIN, HIGH);

  EIFR |= bit(INTF1); // clear INT1 interrupt flag




  attachInterrupt(digitalPinToInterrupt(INT2_PIN), &EarthquakeHandler, CHANGE);


  Serial.println(F("\nListening for earthquakes!"));
  //printLast5();
  //printGreatest5();

}



void loop()
{




  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState="));
  Serial.println(wakeState);
#endif


  if ((wakeState == SLEEP_INTERRUPT) || (wakeState == REBOOT))
  {

    wakeState = NO_INTERRUPT;






    TimeStamp = millis();

    // if INA3221 present, read charge data

    if (INA3221_Present)
    {


      BatteryVoltage = INA3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
      BatteryCurrent = INA3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);

      SolarPanelVoltage = INA3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
      SolarPanelCurrent = -INA3221.getCurrent_mA(SOLAR_CELL_CHANNEL);


      LoadVoltage = INA3221.getBusVoltage_V(OUTPUT_CHANNEL);
      LoadCurrent = INA3221.getCurrent_mA(OUTPUT_CHANNEL) * 0.75;


    }


    if (BatteryVoltage < 2.80)
      AuxA = AuxA | 0x04;
    else
      AuxA = AuxA & 0xFB;









    // check if it is time to send message (every 10 minutes or on interrupt) - 30 seconds pre check

    if (((wakeCount % WAKEUPS) == 0) || (EQInProgress == true))
    {
      Serial.print(F("EQInProgress="));
      Serial.println(EQInProgress);
      Serial.print(F("PreviousEQInProgress="));
      Serial.println(PreviousEQInProgress);
      // Pat the WatchDog
      ResetWatchdog();


      if ((EQInProgress == true) && ( PreviousEQInProgress == false))
      {
        Serial.println(F("-------------------- EARTHQUAKE STARTED! --------------------\n"));

        state = D7S.getState();
        Serial.print("state=");
        Serial.println(state);
        PreviousEQInProgress = true;

        EQCount++;



        while (D7S.isEarthquakeOccuring())
        {

          //print information about the current earthquake
          float currentSI = D7S.getInstantaneousSI();
          float currentPGA = D7S.getInstantaneousPGA();

          if (currentSI > highInstantaneousSI)
          {
            highInstantaneousSI = currentSI;
          }
          //getting Instantaneous SI
          Serial.print(F("\tInstantaneous SI: "));
          Serial.print(currentSI);
          Serial.println(F(" [m/s]"));

          if (currentPGA > highInstantaneousPGA)
          {
            highInstantaneousPGA = currentPGA;
          }
          //getting Instantaneous PGA
          Serial.print(F("\tInstantaneous PGA (Peak Ground Acceleration): "));
          Serial.print(currentPGA);
          Serial.println(" [m/s^2]\n");
          // Pat the WatchDog
          ResetWatchdog();


        }



        PreviousEQInProgress = false;
        Serial.println("-------------------- EARTHQUAKE ENDED! --------------------");
        state = D7S.getState();
        Serial.print("state=");
        Serial.println(state);

        if (state == 2)
        {
          PreviousEQInProgress = EQInProgress;
          EQInProgress = false;
        }

        if ((state == 1 )   || (state == 0)) // 1 is normal reading earthquake, 0 is normal standby
        {
          printLast5();
          printGreatest5();

          // end of EarthQuake

          float si;
          float pga;
          float temperature;

          si = D7S.getSIDataLast(0);
          last5_0_SI = si;
          pga = D7S.getPGADataLast(0);
          last5_0_PGA = pga;

          temperature = D7S.getTempDataLast(0);

          //printing the SI
          Serial.print(F("\tSI: "));
          Serial.print(si);
          Serial.println(F(" [m/s]"));

          //printing the PGA
          Serial.print(F("\tPGA (Peak Ground Acceleration): "));
          Serial.print(pga);
          Serial.println(F(" [m/s^2]"));

          //printing the temperature at which the earthquake has occured
          Serial.print(F("\tTemperature: "));
          Serial.print(temperature);
          Serial.println(F(" [°C]\n"));

          // print instantaneous highs
          Serial.print(F("\tInstantaneous High SI: "));
          Serial.print(highInstantaneousSI);
          Serial.println(F(" [m/s]"));

          Serial.print(F("\tInstantanenous High PGA: "));
          Serial.print(highInstantaneousPGA);
          Serial.println(F(" [m/s^2]"));
          //reset earthquake events



          // transmit information

          // If the earthquake is zero for both storage and highInstantaneous then  ignore  instead of transmitting.

          // If highInstantaneous is zero and Last5 (0) is the same, ignore instead of transmitting.

          if ((highInstantaneousSI < 0.000001) && (si < 0.000001))  // floats don't always like equality  : )
          {
            Serial.println("Transmit request ignored highInt and si == 0.00");
          }
          else if ((highInstantaneousSI < 0.000001) && (si == last5_0_SI))
          {
            Serial.println("Transmit request ignored highInt and si == prev si ");
          }
          else
          {
            // transmit


            if ((AuxA & 0x04)  == false)   // If the battery vboltage is less than 2.80V, then AfterShock is flaky
            {
              // Now send the message


              Serial.println(F(">>>>>>>>>>>>>>>Transmitting earthquake message<<<<<<<<<<<<"));
              sendMessage(si, pga);

              Serial.println(F("----------Packet Sent.  Sleeping Now----------"));


            }
          }



          D7S.resetEvents();
        }
      }
      else
      {
        // send the I'm Alive Message

        AuxA = AuxA | 0x08;  // bit on

        Serial.println(F(">>>>>>>>>>>>>>>Transmitting Alive message<<<<<<<<<<<<"));
        float si, pga;
        si = D7S.getSIDataLast(0);
        pga = D7S.getPGADataLast(0);
        printLast5();

        sendMessage(si, pga); // send I'm Alive Message

        AuxA = AuxA & 0xF7;  // bit off


      }
    }

  }
  // Pat the WatchDog
  ResetWatchdog();

  if (wakeState != REBOOT)
    wakeState = SLEEP_INTERRUPT;
  long timeBefore;
  long timeAfter;
  timeBefore = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeBeforeSleep="));
  Serial.println(timeBefore);
#endif
  delay(100);


  //Sleepy::loseSomeTime(nextSleepLength);
  for (long i = 0; i < nextSleepLength / 16; ++i)
  {
    Sleepy::loseSomeTime(16);

    // wake up for the EQ Interrupt

    if (EQInProgress == true)
      i = nextSleepLength / 16;
  }

  wakeState = SLEEP_INTERRUPT;

  wakeCount++;

#if defined(TXDEBUG)
  Serial.print(F("Awake now: "));
#endif
  timeAfter = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeAfterSleep="));
  Serial.println(timeAfter);

  Serial.print(F("SleepTime = "));
  Serial.println(timeAfter - timeBefore);

  Serial.print(F("Millis Time: "));
#endif
  long time;
  time = millis();
#if defined(TXDEBUG)
  //prints time since program started
  Serial.println(time / 1000.0);
  Serial.print(F("2wakeState="));
  Serial.println(wakeState);
#endif





  // Pat the WatchDog
  ResetWatchdog();



}
