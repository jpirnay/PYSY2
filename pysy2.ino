///////////////////////////// defines and global variables here /////////////////////////

// PIN MAP - Each I/O pin (use or unused) is defined . . .
// I2C Pins for RTC - D19 (SCL), D18 (SDA)
#define BUZZER_PIN       16             // sw jumper for buzzer
#define LED_PIN          14             // L1 LED on Logger Shield
// D13 (CLK), D12 (MISO), D11 (MOSI)    SPI Pins for SD card
#define CS_PIN           10             // Logger Shield Chip Select pin for the SD card
//                        D9 & D8          available
//                        D7 D6            serial comm to GPS
//                        D4 D3            available
//                        D2               Interrupt 1 for Geiger 
//                        D1 & D0          interface to pc

// These are DEFAULTS! - only used if menu has not been run
#define DOSE_PARAM        1             // defaults to Dose Mode ON
// #define PRI_RATIO       175             // defaults to SBM-20 ratio
#define PRI_RATIO       122             // defaults to SI-29BG ratio
#define DISP_PERIOD    5000.0           // defaults to 5 sec sample & display
#define LOGGING_PEROID    1             // defaults a 1 min logging period

// other defines . . .
#define LOW_VCC          4200 //mV      // if Vcc < LOW_VCC give low voltage warning

// BEGIN USER PARAMETER AREA . . .
#define DEBUG       true              // if true shows available memory
// END USER PARAMETER AREA

#include <SdFat.h>                      // the complete SD library is too fat (pun intended)  
#include <Time.h>                       // time functions
#include <Wire.h>                       // Two Wire interface
#include <Adafruit_BMP085.h>            // BMP085 barometric module
#include <TinyGPS.h>                    // MODIFIED LIB - parses GPS sentences
// MODIFIED LIB - _GPS_NO_STATS uncommented (and resulting compile error fixed) saves 80 bytes
#include <avr/pgmspace.h>               // for using PROGMEM or F("")
#include <SoftwareSerial.h>             // Had too much troubles with HW TX/RX so had to bypass that issue...

////////////////////////////////// globals /////////////////////////////////////

// These hold the local values that have been read from EEPROM
unsigned long LoggingPeriod;            // mS between writes to the card
float uSvRate;                          // holds the rate selected by jumper

boolean SD_OK = true;                   // assume SD is OK until init
boolean lowVcc = false;                 // true when Vcc < LOW_VCC

// variables for counting periods and counts . . .
unsigned long dispPeriodStart, dispPeriod; // for display period

unsigned long logPeriodStart;           // for logging period
unsigned long logCnt, logCPM;           // to count and log CPM

float uSv = 0.0;                        // display CPM converted to VERY APPROXIMATE uSv
float uSvLogged = 0.0;                  // logging CPM converted to VERY APPROXIMATE uSv
float avgCnt;                           // holds the previous moving average count
byte sampleCnt;                         // the number of samples making up the moving average
float temp_uSv = 0.0;                   // for converting CPM to uSv/h for running average
byte altDispCnt = 0;                    // counter to disp running average once / 4 displays
int Vcc_mV;                             // mV of Vcc from last check 

char timeString[10];                    // time format determined by #defines above
char dateString[10];                    // time format determined by #defines above
byte day_of_week,DST;                   // if daylight savings time, DST == true
int GMT_Offset;                         // defaults to 13 = GMT+1
unsigned long counter;                  // counter for logcount

// Communication with GPS
SoftwareSerial GPSModul(6, 7);          // 6 is RX, 7 is Tx
TinyGPS gps;                            // GPS conversion library

// Communication with SD card reader
SdFat sd;
SdFile logfile;
SdFile gpsfile;

// BMP085 barometric sensor
Adafruit_BMP085 bmp;                    // barometric sensor

// ---------------- INIT ---------------
void setup(){
  counter=0;
  Serial.begin(9600);                // comspec for GPS is 48,N,8,1
  GPSModul.begin(9600);

  attachInterrupt(0,GetEvent,FALLING);  // Geiger event on D2 triggers interrupt
//  digitalWrite(2, HIGH);              // set pull up on INT0
  pinMode(LED_PIN, OUTPUT);             // setup LED pin
  pinMode(10, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);          // sw control buzzer
  digitalWrite(BUZZER_PIN, HIGH);       // buzzer = on

  Get_Settings();                       // get the settings stored in EEPROM
  LoggingPeriod = 5 * 1000;              // every 5 seconds write entry to Logfile...

  initCard();                           // init the SD card
  initTime();							// read time from SD card 
  initLog();                            // prepare log files
  initGPS();                            // init the GPS to send output only when requested
  setSyncProvider(gpsTimeSync);         // define the function that syncs the time with GPS

//  Serial.println("Init BMP");  

  if (!bmp.begin()) {
    Serial.println(F("No BMP085"));
  }

  
//  Serial.print("RAM:");
//  Serial.println(AvailRam());

  logPeriodStart = 0;     // start logging timer
  logCnt= 0;
}

void Get_Settings(){ // read setting out of EEPROM and set local variables
  uSvRate = PRI_RATIO;
  GMT_Offset = 13; // readParam(ZONE_ADDR);
  // to avoid entering negative offsets 1-12 = neg zones, 13-24 = pos zones 1-12
  if (GMT_Offset >12) GMT_Offset = (GMT_Offset - 12);
  else GMT_Offset = GMT_Offset * -1;
  logPeriodStart = 0;     // start logging timer
}

// ---------------- MAIN LOOP ---------------

void loop(){
  //uncomment 4 lines below for self check (1/166ms X 60 = 360 CPM
  //dispCnt++;
  //logCnt++;
  //runCnt++;
  //delay(167);                         // 167 mS ~= 6 Hz    
 
    readGPS();  // make sure all serial input is parsed...
  
    if (millis() >= logPeriodStart + LoggingPeriod){ // LOGGING PERIOD
      if (SD_OK) LogCount(logCnt);      // pass in the counts to be logged
      logCnt = 0;                       // reset log event counter
      logPeriodStart = millis();        // reset log time
    }
}

void GetEvent(){   // ISR triggered for each new event (count)
  logCnt++;
}

// ---- some util functions  ---

void Blink(byte led, byte times){ // just to flash the LED
  for (byte i=0; i< times; i++){
    digitalWrite(led,HIGH);
    delay (120);
    digitalWrite(led,LOW);
    delay (90);
  }
}


long readVcc() { // SecretVoltmeter from TinkerIt
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

int AvailRam(){ 
  int memSize = 2048;                   // if ATMega328
  byte *buf;
  while ((buf = (byte *) malloc(--memSize)) == NULL);
  free(buf);
  return memSize;
} 


///////////////////////////////// Time Functions Here ///////////////////////////////
byte getLength(unsigned long number){
  byte length = 0;
  for (byte i = 1; i < 10; i++){
    if (number > pow(10,i)) length = i;
    else return length +1;
  }
}

void FormatDateTime(){  // get the time and date from the GPS and format it
  int dispYear;
  int i;
  memset(timeString,0,sizeof(timeString));  // initialize the strings
  memset(dateString,0,sizeof(dateString));

  // convert Sun=1 format to Sun=7 format (DST calc is based on this)
  day_of_week = (weekday()==1) ? 7: (weekday() - 1);

  // (The time lib will deal with AM/PM and 12 hour clock)
  // make time string
  AppendToString (hour(),timeString);         // add 24 hour time to string
  strcat(timeString,":");
  if (minute() < 10) strcat(timeString,"0");
  AppendToString (minute(),timeString);       // add MINUTES to string
  strcat(timeString,":");
  if (second() < 10) strcat(timeString,"0");
  AppendToString (second(),timeString);     // add SECONDS to string

  // make date string
  i=day();                              // add DAY to string
  if (i < 10) strcat(dateString,"0");
  AppendToString (i,dateString);  
  strcat(dateString, ".");

  i = month();                          // add MONTH to string  
  if (i < 10) strcat(dateString,"0");
  AppendToString (i,dateString);
  strcat(dateString, ".");

  i = year();                          // add YEAR to string
  AppendToString (i,dateString);                            
}


void AppendToString (int iValue, char *pString){ // appends a byte to string passed
  char tempStr[6];
  memset(tempStr,'\0',sizeof(tempStr));
  itoa(iValue,tempStr,10);
  strcat(pString,tempStr);
}

/////////////////////////////////// GPS Functions ////////////////////////////////
bool readGPS(){  //  request and get a sentence from the GPS
char c;
boolean res;
  res = false;
  while (GPSModul.available()){           // Get sentence from GPS
    c = GPSModul.read();
    if (SD_OK) {
       gpsfile.print(c);
    }   
    if (gps.encode(c)) res = true;
  }
  return res;
}


time_t gpsTimeSync(){  //  returns time if avail from gps, else returns 0
  unsigned long fix_age = 0 ;
  gps.get_datetime(NULL,NULL, &fix_age);

  if(fix_age < 1000) return gpsTimeToArduinoTime();   // only return time if recent fix  
  return 0;
}


time_t gpsTimeToArduinoTime(){  // returns time_t from GPS with GMT_Offset for hours
  tmElements_t tm;
  int year;
  readGPS();                    // make sure we process te most recent data
  gps.crack_datetime(&year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, NULL);
  tm.Year = year - 1970; 
  time_t time = makeTime(tm);
  if (InDST()) time = time + SECS_PER_HOUR;
  return time + (GMT_Offset * SECS_PER_HOUR);
}


bool InDST(){  // Returns true if in DST - Caution: works for central europe only
  // DST starts the last Sunday in March and ends the last Sunday in Octobre
  bool res;
  byte DOWinDST, nextSun;
  int Dy, Mn;  

  Dy = tmDay;
  Mn = tmMonth;
  //Dy = 27;  // FOR TESTING
  //Mn = 7;
  res = false;

  // Pre-qualify for in DST in the widest range (any date between  and 23.10) 
  // Earliest date in March is 25th
  // Earliest date in October is 25th
  if ((Mn == 3 && Dy >= 25) || (Mn > 3 && Mn < 10) || (Mn == 10 && Dy <= 23) ){
    DOWinDST = true;                    // assume it's in DST until proven otherwise
    nextSun = Dy + (7 - day_of_week);   // find the date of the last Sunday
  while (nextSun<24) nextSun += 7;
    if (nextSun > 31) nextSun -= 7;     // come back to month
    if (Mn == 3 && (Dy < nextSun)) DOWinDST = false;     // it's before the last Sun in March
    if (Mn == 10 && (Dy >= nextSun)) DOWinDST = false; // it's after the 1ast Sun in Oct.
    if (DOWinDST) res = true;           // DOW is still OK so it's in DST
  }
  return res;                         // Nope, not in DST
}

void initGPS(){   // send commands to the GPS for what to output - uses PROGMEM for this
  // could program GPS by sending some commands to ommit cerrtain stats, but why ? :-)

}
////////////////////////////// Logging Functions Here ///////////////////////////////

void LogCount(unsigned long lcnt){
  // SD File format: "Date", "Time", latitude, longitude, speed, CPM, uSv/hr, Vcc, temperature, pressure1, pressure2
  bool newdata = false; //TODO
  float flat, flon;
  float bmp_temp;
  int32_t bmp_pascal;
  uint32_t bmp_raw_pascal;
  unsigned long age;
/*  
  Serial.print("Logcount: ");
  Serial.println(lcnt);
*/
  counter++;
  bmp_temp = bmp.readTemperature();
  bmp_pascal = bmp.readPressure();
  bmp_raw_pascal = bmp.readRawPressure();
/*
  Serial.print("T="); Serial.println(bmp_temp);  
  Serial.print("P="); Serial.println(bmp_pascal);
*/
  FormatDateTime();                     // format the latest time into the strings
  Vcc_mV = readVcc();                   // check voltage
  if (Vcc_mV <= LOW_VCC) lowVcc = true; // check if Vcc is low 
  else lowVcc = false;

  logfile.print(counter);              // log the number
  logfile.print(",");                  // and comma delimited
  // Date & Time . . .
  logfile.print("\"");                  // log the time and date (quoted)
  logfile.print(dateString);            // formatted date string
  logfile.print("\",\"");               // and comma delimited
  logfile.print(timeString);            // formatted time string
  logfile.print("\"");                 

  // GPS . . .
  readGPS();                              // get a reading if any present
  newdata = true; // FORCING THIS FOR NOW

  gps.f_get_position(&flat, &flon, &age); // get the current GPS position
  logfile.print(",");                     // comma delimited
  logfile.print(flat,8);                  // now log the lat from the GPS

  logfile.print(",");                     // comma delimited
  logfile.print(flon,8);                  // now log the lon from the GPS

  logfile.print(",");                     // comma delimited
  logfile.print(gps.f_speed_kmph(),1);    // now log the speed from the GPS

  flat = gps.altitude() / 100.0;           // calc in m
  logfile.print(",");                     // comma delimited
  logfile.print(flat,1);                  // now log the altitude from the GPS

  logfile.print(",");                     // comma delimited
  logfile.print(gps.fixtype());           // now log the fixtype 0=none, 1=2D, 2=3D

  logfile.print(",");                     // comma delimited
  logfile.print(gps.satellites());           // now log the # of sats used from the GPS

  // Log counts . . .
  logCPM = float(lcnt) / (float(LoggingPeriod) / 60000);
  uSvLogged = logCPM / uSvRate;         // make uSV conversion
  logfile.print(",");                     // comma delimited
  logfile.print(logCPM,DEC); // log the CPM

  logfile.print(",");                   // comma delimited
  logfile.print(uSvLogged,4);

  logfile.print(",");                   // comma delimited
  logfile.print(Vcc_mV/1000. ,2);       //convert to Float, divide, and print 2 dec places
  // pressure data
  logfile.print(","); // comma delimited
  logfile.print(bmp_temp, 2);           // 2 decimal places should be enough :-)

  logfile.print(","); // comma delimited
  logfile.print(bmp_pascal);           // Pressure in Pascal

  logfile.print(","); // comma delimited
  logfile.print(bmp_raw_pascal);       // Pressure in Pascal
  
  logfile.println();

  // really need to tell if the GPS has a fix
  if (newdata = true) Blink(LED_PIN,1); // show it's receiving
  logfile.sync();                       // force update of the files
  gpsfile.sync();
}

void initCard(){   // initialize the SD card
  SD_OK = false;                        // don't try to write to the SD card
  pinMode(10, OUTPUT);                  // must set DEFAULT CS pin output, even if not used

  if (!sd.begin(10, SPI_HALF_SPEED))
 {  
    sd.initErrorHalt();
    error("Card");
  }
  SD_OK = true;
//  SdFile::dateTimeCallback(SDdateTime); 
  SdFile::dateTimeCallback(SDDateTime);
}

void initLog(){
  char filename[]="py000.csv";
  if ( gpsfile.open("pysy.log", O_WRONLY | O_CREAT) ) {
    gpsfile.println("PYSY-Startup");
    FormatDateTime();
    gpsfile.println(dateString);
    gpsfile.println(timeString);
    gpsfile.print("RAM:");
    gpsfile.println(AvailRam());
    gpsfile.sync();
  }
  else  
  {
    sd.errorHalt("pysy.log not writeable");
  }  
  
  for (uint8_t i = 0; i < 250; i++) {
    filename[2] = i/100 + '0';
    filename[3] = (i%100)/10 + '0';
    filename[4] = i%10 + '0';
/*    
    Serial.print("Trying: ");
    Serial.println(filename);
*/
    if (! sd.exists(filename)) {
        break;  // leave the loop!
    }  
  }
//  Serial.print("Final test:"); Serial.println(filename);
  if (logfile.open(filename, O_WRONLY | O_CREAT) ) {
//    Serial.println(F("SD OK, now write..."));
    logfile.println(F("NR,DATE,TIME,LAT,LON,KMPH,ALT,FIX,SATS,CPM,uSv,Vcc,TEMP,PR1,PR2"));
  }
  else
  {
     sd.errorHalt("opening logfile for write failed");
  }  

}

void error(char *str){
  Serial.println("CARD!");
  Serial.println(str);                       // display this error or status
  digitalWrite(LED_PIN, HIGH);          // red LED indicates error
}

/* Reads default time before any fix from GPS forces the right one, 
   tries to read time.txt from SD-card 
   Attention: uses logfile variable ! */
void initTime()
{
  char ch;
  int tpos[6];
  int valu, ct, rd;
  tpos[6]=2012; // YEAR
  tpos[5]=10;   // MONTH
  tpos[4]=20;   // DAY
  tpos[3]=0;    // SEC
  tpos[2]=0;    // MIN
  tpos[1]=14;   // HOUR
  if (SD_OK)
  {
    ct = 1;
    valu = 0;
    if (logfile.open("time.txt", O_READ))
    {
       while ((rd = logfile.read()) >= 0){
         ch = (char)rd;
         switch (ch) {
         case '\n':
           if (ct<=6) tpos[ct] = valu;
           ct = ct + 1;
           valu = 0;
           break;
         case '0': valu = valu * 10 + 0; break;
         case '1': valu = valu * 10 + 1; break;
         case '2': valu = valu * 10 + 2; break;
         case '3': valu = valu * 10 + 3; break;
         case '4': valu = valu * 10 + 4; break;
         case '5': valu = valu * 10 + 5; break;
         case '6': valu = valu * 10 + 6; break;
         case '7': valu = valu * 10 + 7; break;
         case '8': valu = valu * 10 + 8; break;
         case '9': valu = valu * 10 + 9; break;
         default:
           break;  
         }  
       } 
      logfile.close(); 
    }
  }
/*
  Serial.print("Set Time:");
  for (ct=1;ct<4;ct++){
     Serial.print(tpos[ct]); 
     Serial.print(":");
  }   
  Serial.print(" ");
  for (ct=4;ct<7;ct++){
     Serial.print(tpos[ct]); 
     Serial.print(".");
  }  
  Serial.println();
*/
  setTime(tpos[1], tpos[2], tpos[3], tpos[4], tpos[5], tpos[6]);
}

// call back for file timestamps
void SDDateTime(uint16_t* date, uint16_t* time) {
  time_t nw;
  nw = gpsTimeSync();
  if (nw==0) nw = now();
/*  
  Serial.print("SDDate:" );
  Serial.print(day(nw));  
  Serial.print(".");
  Serial.print(month(nw));  
  Serial.print(".");
  Serial.println(year(nw));
*/
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(nw), month(nw), day(nw));

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(nw), minute(nw), second(nw));
}
