////////////////////////////////////////////////////////////////////////////
// RTC
////////////////////////////////////////////////////////////////////////////
#include <SPI.h>

#include <RTClib.h>       // library from https://github.com/MrAlvin/RTClib  there are many other DS3231 libs availiable
RTC_DS3231 rtc;
// creates an RTC object in the code
// variables for reading the RTC time & handling the INT(0) interrupt it generates
#define DS3231_I2C_ADDRESS 0x68

const byte PROGMEM RTC_INTERRUPT_PIN = 2;

// Time
uint32_t unixTime = 0;
DateTime now(unixTime);

float T_rtc = 0;
byte tMSB = 0;
byte tLSB = 0;

////////////////////////////////////////////////////////////////////////////
// SLEEP
////////////////////////////////////////////////////////////////////////////

#include "LowPower.h"     // from https://github.com/rocketscream/Low-Power
byte alarmhour;
byte alarmminute;
byte alarmsecond;
volatile boolean clockInterrupt = false;

//const PROGMEM byte sampleIntervalMinutes = 0;
//const PROGMEM byte sampleIntervalSeconds = 5;
//const PROGMEM bool evenSeconds = false;
//const PROGMEM int bufferLen = 2; //10-20


const PROGMEM byte sampleIntervalMinutes = 1;
const PROGMEM byte sampleIntervalSeconds = 0;
const PROGMEM bool evenSeconds = true;
const PROGMEM int bufferLen = 20; //5 ok 30 not, 20 ok?


////////////////////////////////////////////////////////////////////////////
// SD
////////////////////////////////////////////////////////////////////////////

#include <SD.h>  // I much prefer SdFat.h by Greiman over the old SD.h library used here
const PROGMEM int SD_CS_PIN = 10;    //CS orange
//const PROGMEM int transistorPin = 9; // green
const PROGMEM int SD_LED_PIN = 6;    // blue
//const int PROGMEM transistorPin = 9;
char logPath[ ] = "00000000.log";

uint8_t bufferLoc = 0;

// rtc
int buffer_T_rtc[bufferLen];

// si7021
int  buffer_T_si7021[bufferLen];
int  buffer_RH_si7021[bufferLen];
bool buffer_heaterOn_si7021[bufferLen];

// sht31
int   buffer_T_sht31[bufferLen];
int   buffer_RH_sht31[bufferLen];
bool  buffer_heaterOn_sht31[bufferLen];

// battery
int   buffer_battery_voltage[bufferLen];

// time
uint32_t buffer_time[bufferLen];

////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////

const PROGMEM int SENSOR_LED_PIN = 7;    // green
#include "SparkFun_Si7021_Breakout_Library.h"
//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
Weather si7021;
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// si7021
float T_si7021 = 0;
float RH_si7021 = 0;
bool heaterOn_si7021 = false;

// sht31
float T_sht31 = 0;
float RH_sht31 = 0;
bool heaterOn_sht31 = false;

////////////////////////////////////////////////////////////////////////////
// Voltage divider
////////////////////////////////////////////////////////////////////////////

const float PROGMEM divider_factor = 1.856; // config with AMP-330-UER, one decimal, ideal 2 (2 x 10MOhm)    //1.94;//*1.04761904765;
float battery_voltage = 0.0;            // calculated voltage
#define voltage_divider_pin A2


////////////////////////////////////////////////////////////////////////////
// RAMDOM
////////////////////////////////////////////////////////////////////////////

//#include <avr/wdt.h>

const PROGMEM char header1[] = "Data logger for tempereture and humidity.";
const PROGMEM char header2[] = "Code, design, and soldering by Antti Mikkonen, a.mikkonen@iki.fi, 2019.";
const PROGMEM char header3[] = "Based on open source projects and libraries. See source for details.";
const PROGMEM char codebuild[]  = __FILE__;  // loads the compiled source code directory & filename into a varaible
const PROGMEM char header[]  = "UTC time (year/month/date hour:minute:second), T RTC (C), T sht31 (C), T si7021 (C), RH sht31 (-), RH si7021 (-), heaterOn sht31 (bool),  heaterOn si7021 (bool), battery voltage (V)";

void setup()
{

  Serial.begin(9600);
//  Serial.println("TEST");
//  Serial.flush();
  ////////////////////////////////////////////////////////////////////////////
  // RTC
  ////////////////////////////////////////////////////////////////////////////
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);

  // Init
  Wire.begin();          // start the i2c interface for the RTC
  rtc.begin();           // start the RTC
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

//  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//    
//  Serial.println("TEST2");
//  Serial.flush();

  
  readTime();
//  Serial.println("TEST3");
//  Serial.flush();
  printTime();
  Serial.println();
  ////////////////////////////////////////////////////////////////////////////
  // SD
  ////////////////////////////////////////////////////////////////////////////
  //pinMode(transistorPin, OUTPUT);
  pinMode(SD_LED_PIN, OUTPUT);
  // Setting the SPI pins high helps some sd cards go into sleep mode
  // the following pullup resistors only needs to be enabled for the ProMini builds - not the UNO loggers
  pinMode(SD_CS_PIN , OUTPUT); digitalWrite(SD_CS_PIN, HIGH);
  //digitalWrite(chipSelect, HIGH); //ALWAYS pullup the ChipSelect pin with the SD library
  //and you may need to pullup MOSI/MISO, usually MOSIpin=11, and MISOpin=12
  //pinMode(MOSIpin, OUTPUT); digitalWrite(MOSIpin, HIGH); //pullup the MOSI pin
  //pinMode(MISOpin, INPUT); digitalWrite(MISOpin, HIGH);  //pullup the MISO pin
  delay(1);

//  Serial.println("TEST4");
//  Serial.flush();

  ////////////////////////////////////////////////////////////////////////////
  // Sensors
  ////////////////////////////////////////////////////////////////////////////

  pinMode(SENSOR_LED_PIN, OUTPUT);

//  Serial.println("TEST5");
//  Serial.flush();
  sht31.begin(0x44);
//  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
//    Serial.println(F("Couldn't find SHT31."));
//    while (1) delay(1);
//  } else {
//    Serial.println(F("SHT31 Found"));
//  }
//
//  Serial.println("TEST6");
//  Serial.flush();

  //Initialize the I2C sensors and ping them
  si7021.begin();
  Serial.flush();
  setAlarmOff();

  // Check SD 

  // Power up
  //digitalWrite(transistorPin, HIGH);
  if (SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD Found"));
  } else {
    Serial.println(F("Couldn't find SD."));  
  }
  Serial.flush();
  //digitalWrite(transistorPin, LOW);
  //wdt_enable(WDTO_8S);
}

void(* resetFunc) (void) = 0;


void loop()
{
  
  setNewAlarm();
  goToSleepAndWakeUp();
  setAlarmOff();

  readTime();
  readData();
  writeData();
  
  //wdt_reset();


  //  setAlarmOff();
  //  readTime();
  //  readData();
  //  writeData();
  //  setNewAlarm();
  //  goToSleepAndWakeUp();


  //  goToSleepAndWakeUp();
  //  rtc.update();
  //  readData();
  //  writeData();
  //  setNewAlarm();
}

void readTime() {
  now = rtc.now();
  unixTime = now.unixtime();
}

void setAlarmOff() {
  // We set the clockInterrupt in the ISR, deal with that now:
  if (clockInterrupt) {
    if (rtc.checkIfAlarm(1)) {       //Is the RTC alarm still on?
      rtc.turnOffAlarm(1);              //then turn it off.
    }
    //print (optional) debugging message to the serial window if you wish
    //Serial.print("RTC Alarm on INT-0 triggered at ");
    //Serial.println(CycleTimeStamp);
    clockInterrupt = false;                //reset the interrupt flag to false
  }//—————————————————————–
}

void setNewAlarm() {

  alarmhour = now.hour();
  alarmminute = now.minute() + sampleIntervalMinutes;
  alarmsecond = now.second() + sampleIntervalSeconds;

  // check for roll-overs
  if (alarmsecond > 59) { //error catching the 60 rollover!
    alarmsecond = alarmsecond - 60;
    alarmminute = alarmminute + 1;
  }
  // check for roll-overs
  if (alarmminute > 59) { //error catching the 60 rollover!
    alarmminute = alarmminute - 60;
    alarmhour = alarmhour + 1;
  }
  if (alarmhour > 23) {
    alarmhour = 0;
    // put ONCE-PER-DAY code here -it will execute on the 24 hour rollover
  }

  if (evenSeconds) {
    alarmsecond = 0;
  }

  // then set the alarm
  //rtc.setAlarm1Simple(alarmhour, alarmminute);
  rtc.setA1Time(0, alarmhour, alarmminute, alarmsecond, 0b00001000, false, false, false);

  rtc.turnOnAlarm(1);
  //if (RTC.checkAlarmEnabled(1)) {
  //  //you would comment out most of this message printing
  //  //if your logger was actually being deployed in the field
  //  Serial.print(F("RTC Alarm Enabled!"));
  //  Serial.print(F(" Going to sleep for : "));
  //  Serial.print(sampleIntervalMinutes);
  //  Serial.println(F(" minutes"));
  //  Serial.println();Serial.flush();//adds a carriage return & waits for buffer to empty
  //}

}



void readRTCTemperature() {
  // read the RTC temp register and print that out
  // Note: the DS3231 temp registers (11h-12h) are only updated every 64seconds
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);                     //the register where the temp data is stored
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);   //ask for two bytes of data
  if (Wire.available()) {
    tMSB = Wire.read();            //2’s complement int portion
    tLSB = Wire.read();             //fraction portion
    T_rtc = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0;  // Allows for readings below freezing: thanks to Coding Badly
    //T_RTC_ok = true;
  }
  else {
    T_rtc = 1000;
    //T_RTC_ok = false;
  }

}

void goToSleepAndWakeUp() {


  //delay(25); //this optional delay is only here so we can see the LED’s otherwise the entire loop executes so fast you might not see it.
  //digitalWrite(GREEN_PIN, LOW);
  // Note: Normally you would NOT leave a red indicator LED on during sleep! This is just so you can see when your logger is sleeping, & when it's awake
  //digitalWrite(RED_PIN, HIGH);  // Turn on red led as our indicator that the Arduino is sleeping. Remove this before deployment.

  //delay(5000); //this optional delay is only here so we can see the LED’s otherwise the entire loop executes so fast you might not see it.
  //digitalWrite(LED_BUILTIN,LOW);//turning LED on

  //——– sleep and wait for next RTC alarm ————–
  // Enable interrupt on pin2 & attach it to rtcISR function:
  attachInterrupt(0, rtcISR, LOW);
  // Enter power down state with ADC module disabled to save power:
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  //processor starts HERE AFTER THE RTC ALARM WAKES IT UP
  detachInterrupt(0); // immediately disable the interrupt on waking
  //digitalWrite(LED_BUILTIN,HIGH);//turning LED on

  //digitalWrite(RED_PIN, LOW);
  //digitalWrite(GREEN_PIN, HIGH); //Interupt woke processor, turn on green led

}

// This is the Interrupt subroutine that only executes when the RTC alarm goes off
void rtcISR() {
  clockInterrupt = true;
}


void readData() {

  // LED ON
  digitalWrite(SENSOR_LED_PIN, HIGH);

  ///////////////////////////////////////////
  // rtc
  ///////////////////////////////////////////
  readRTCTemperature();

  ///////////////////////////////////////////
  // sht31
  ///////////////////////////////////////////

  //sht31.reset()

  // Read sensor values
  T_sht31 = sht31.readTemperature();
  RH_sht31 = sht31.readHumidity();

  if (RH_sht31 > 98) {
    sht31.heater(true);
    heaterOn_sht31 = true;
  } else {
    sht31.heater(false);
    heaterOn_sht31 = false;
  }

  ///////////////////////////////////////////
  // si7021
  ///////////////////////////////////////////
  // Read sensor values
  RH_si7021 = si7021.getRH();
  T_si7021 = si7021.getTemp();


  if (RH_si7021 > 98) {
    si7021.heaterOn();
    heaterOn_si7021 = true;
  } else {
    si7021.heaterOff();
    heaterOn_si7021 = false;
  }


  ///////////////////////////////////////////
  // Battery voltage
  ///////////////////////////////////////////
  read_battery_voltage();

  // LED OFF
  digitalWrite(SENSOR_LED_PIN, LOW);
}

void read_battery_voltage() {
  battery_voltage = analogRead(voltage_divider_pin) / 1023.0 * 3.3 * divider_factor;
}


void writeData() {

  bool bufferFull = addToBuffer();

  if (bufferFull) {
    if (!writeToSD()) {
      Serial.println(F("Wait one second and try again."));
      delay(1000);
      if (!writeToSD()) {
        Serial.println(F("Giving up. RESET!"));
        Serial.flush();
        resetFunc();
      }
    }
  }
  writeSerial();
}


//void writeData() {
//
//  bool timeToWrite = false;
//  if (latestDate != uint8_t(rtc.date())) {
//    timeToWrite = true;
//  } else {
//    latestDate = rtc.date();
//    timeToWrite = addToBuffer();
//    writeSerial(timeToWrite);
//  }
//
//  if (timeToWrite) {
//    if (!writeToSD()) {
//      Serial.println(F("Wait one second and try again."));
//      delay(1000);
//      if (!writeToSD()) {
//        Serial.println(F("Giving up. RESET!"));
//        Serial.flush();
//        resetFunc();
//      }
//    }
//  }
//
//  if (latestDate != uint8_t(rtc.date())) {
//    Serial.println(F("New date. RESET!")); Serial.flush();
//    resetFunc();
//  }
//}

void printTime() {
  //////////////////////////////////////////////
  // TIME
  //////////////////////////////////////////////
  if (now.year() < 10) {
    Serial.print(F("0"));
  }
  Serial.print(now.year()); Serial.print("/");
  if (now.month() < 10) {
    Serial.print(F("0"));
  }
  Serial.print(now.month()); Serial.print("/");
  if (now.day() < 10) {
    Serial.print(F("0"));
  }
  Serial.print(now.day()); Serial.print(" ");
  if (now.hour() < 10) {
    Serial.print(F("0"));
  }
  Serial.print(now.hour()); Serial.print(":");
  if (now.minute() < 10) {
    Serial.print(F("0"));
  }
  Serial.print(now.minute()); Serial.print(":");
  if (now.second() < 10) {
    Serial.print(F("0"));
  }
  Serial.print(now.second()); Serial.print(", ");
  Serial.flush();
  //Serial.println();
}

void writeSerial() {

  //  if (bufferFull) {
  //    Serial.println((__FlashStringHelper*)header);
  //  }

  //////////////////////////////////////////////
  // TIME
  //////////////////////////////////////////////
  printTime();

  //////////////////////////////////////////////
  // Data
  //////////////////////////////////////////////
  // T
  Serial.print(T_rtc); Serial.print(", ");
  Serial.print(T_sht31); Serial.print(", ");
  Serial.print(T_si7021); Serial.print(", ");
  // RH
  Serial.print(RH_sht31); Serial.print(", ");
  Serial.print(RH_si7021); Serial.print(", ");
  // heater
  Serial.print(heaterOn_sht31); Serial.print(", ");
  Serial.print(heaterOn_si7021); Serial.print(", ");
  // battery voltage
  Serial.print(battery_voltage);
  // End
  Serial.println();
  Serial.flush();
}



bool writeToSD() {
  // Power up
  //digitalWrite(transistorPin, HIGH);
  digitalWrite(SD_LED_PIN, HIGH);
  //delay(10);
  delay(100);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("Card failed, or not present"));
    Serial.flush();
    // Leave power on, i.e. also LED on. Return false.
    return false;
  }
  Serial.println(F("writeToSD"));
  Serial.flush();
  //delay(10000);

  // Open data file and check if success
  setLogPath();

  bool newFile;
  if (!SD.exists(logPath)) {
    newFile = true;
  } else {
    newFile = false;
  }
  File dataFile = SD.open(logPath, FILE_WRITE);
  if (!dataFile) {
    Serial.println(F("error opening datafile"));
    Serial.flush();
    return false;
  }
  if (newFile) {
    printHeader(dataFile);
  }

  // Write buffer
  byte last = bufferLen;
  if (bufferLoc != 0) {
    last = bufferLoc;
  }

  for (int k = 0; k < bufferLen; k++) {
    //for (int k=0; k < last; k++){
    writeOneBufferTimeToSD(dataFile, k);
  }

  // Close data file
  dataFile.close();

  // Shut down power
  //digitalWrite(transistorPin, LOW);
  digitalWrite(SD_LED_PIN, LOW);

  return true;
}



void writeOneBufferTimeToSD(File dataFile, int k) {
  //////////////////////////////////////////////
  // TIME
  //////////////////////////////////////////////
  DateTime thisTime(buffer_time[k]);

  if (thisTime.year() < 10) {
    dataFile.print(F("0"));
  }
  dataFile.print(thisTime.year()); dataFile.print("/");
  if (thisTime.month() < 10) {
    dataFile.print(F("0"));
  }
  dataFile.print(thisTime.month()); dataFile.print("/");
  if (thisTime.day() < 10) {
    dataFile.print(F("0"));
  }
  dataFile.print(thisTime.day()); dataFile.print(" ");
  if (thisTime.hour() < 10) {
    dataFile.print(F("0"));
  }
  dataFile.print(thisTime.hour()); dataFile.print(":");
  if (thisTime.minute() < 10) {
    dataFile.print(F("0"));
  }
  dataFile.print(thisTime.minute()); dataFile.print(":");
  if (thisTime.second() < 10) {
    dataFile.print(F("0"));
  }
  dataFile.print(thisTime.second()); dataFile.print(", "); // Print second
  //Serial.println();
  //////////////////////////////////////////////
  // Data
  //////////////////////////////////////////////
  // T
  dataFile.print(int2float(buffer_T_rtc[k])); dataFile.print(", "); 
  dataFile.print(int2float(buffer_T_sht31[k])); dataFile.print(", ");
  dataFile.print(int2float(buffer_T_si7021[k])); dataFile.print(", ");
  // RH
  dataFile.print(int2float(buffer_RH_sht31[k])); dataFile.print(", ");
  dataFile.print(int2float(buffer_RH_si7021[k])); dataFile.print(", ");
  // heater
  dataFile.print(buffer_heaterOn_sht31[k]); dataFile.print(", ");
  dataFile.print(buffer_heaterOn_si7021[k]); dataFile.print(", ");
  // battery voltage
  dataFile.print(int2float(buffer_battery_voltage[k]));

  // End
  dataFile.println();
  dataFile.flush();
}


void printHeader(File dataFile) {
  if (dataFile) { // if the file is available, write to it:
    dataFile.println((__FlashStringHelper*)header1);
    dataFile.println((__FlashStringHelper*)header2);
    dataFile.println((__FlashStringHelper*)header3);
    dataFile.println((__FlashStringHelper*)codebuild); // writes the entire path + filename to the start of the data file
    dataFile.println((__FlashStringHelper*)header);
    dataFile.close();
  }
  else {
    Serial.println(F("error opening data file"));
  }
}

void setLogPath() {
  sprintf(logPath, "%04d%02d%02d.log", now.year(), now.month(), now.day());
}

bool addToBuffer() {

  // Add to buffer
  buffer_time[bufferLoc] = unixTime;

  // rtc
  buffer_T_rtc[bufferLoc] = float2int(T_rtc);

  // si7021
  buffer_T_si7021[bufferLoc] = float2int(T_si7021);
  buffer_RH_si7021[bufferLoc] = float2int(RH_si7021);
  buffer_heaterOn_si7021[bufferLoc] = heaterOn_si7021;

  // sht31
  buffer_T_sht31[bufferLoc] = float2int(T_sht31);
  buffer_RH_sht31[bufferLoc] = float2int(RH_sht31);
  buffer_heaterOn_sht31[bufferLoc] = heaterOn_sht31;

  // battery volate
  buffer_battery_voltage[bufferLoc] = float2int(battery_voltage);

  ++bufferLoc;

  // Buffer full?
  if (bufferLoc == bufferLen) {
    bufferLoc = 0;
    return true;
  } else {
    return false;
  }
}

int float2int(float number) {
  //Serial.print("float2store("); Serial.print(number); Serial.print(") ");Serial.println(int(number*100.0));
  return int(number * 100.0);
}

float int2float(int number) {
  //Serial.print("store2float("); Serial.print(number); Serial.print(") "); Serial.println(float(number)/100.0);
  return float(number) / 100.0;
}
