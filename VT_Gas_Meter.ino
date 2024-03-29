/* ==============================
This version uses COSM.h library to upload data.  
Version 3

To do:
* Add LCD Display http://www.adafruit.com/products/931  to indicate activity: Pulse received, Successful upload to COSM, etc

=== When you create PCB shield ===
one button to increase, another to decrease
Status LEDs: heartbeat (grn), pulse in (amber)
Pin 8 for ethernet shield reset - use jumper so you can disable if using newer board
Terminal block for I2C, Power, Pulse in
two 3-position screw terminals incase you want to measure gas temp and pressure in the future.  
  Just have some wire pads you can solder with jumpers if you decide to use
Shield reset button
Arduino reset button
Power switch
Header to plug LCD into


 Hardware: 
 Arduino UNO v1 (Atmege 328)
 LadyAda Etheret shield v1.2 with Wiznet WIZ811MJ Rev 1.0 - Wire reset to pin 8, ref: http://forums.adafruit.com/viewtopic.php?f=19&p=104959
 ProtoScrewShield  http://www.sparkfun.com/products/9729
 1 10K ohm resistors (for inputs)
 2 560 ohm resistors (for LEDs)
 3 PCB Screw terminal block, 2-position Radio Shack Catalog # 276-1388 
 2-conductor 22 gauge shielded wire McMaster 70045K84
 
  
 D2 - (I) Pulse meter input
 D3 - (O) Pulse On red LED - turns on when pulse relay is closed
 D4 - (O) Heartbeat LED, green
 D8 - Reset Ethernet Shield
 D10 - Used by ethernet shield
 D11 - Used by ethernet shield
 D12 - Used by ethernet shield
 D13 - Used by ethernet shield
 ADC5 - connected to ProtoScrewShield button and room temp thermistor.  Thermister is wired backwards from other thermistors because the ProtoScrewShield uses ADC5 for a pushbutton

 Converting cu-ft propane to gallons and BTU.  Assumes propane is 36 degrees at 10 PSI
 1 cu-ft = 0.02549 gallons = 2328 BTU
 100,070 Therms = 1 BTU

 
COSM Datastreams:
   0 Therms/hour
   1 Pulse meter count
   2 Yesterday cu-ft gas used
   3 Yesterday Gas Cost
   4 Crawlspace Temp - depricated
   5 Basebaord temp  - depricated
   6 Outside Temp - depricated
   7 Heter supply temp - depricated
   8 Temp of resistor for heater sensor  - depricated
   9 Room temp - depricated
  10 Connection Successes
  11 Connection Failures 
  
 
 REVISION HISTORY
 =========================
 02/25/10 - changed daily cost from cents to dollars.  Put TempCount & TempSum in correct routine, they were only in the old one
 03/02/10 - Set remoteSensor[0] = 21.6 to see if outside averaging works properly 
 07/10/10 - change Pachube IP to new server: 173.203.98.29
 07/16/10 - save gas meter pulse to EEPROM.  Changed pulse variables from int to long
 01/16/11 - changed MeterReading from int to unsigned int
 02/06/11 - added InitializeMeterReading so if a new meter reading had to be set, sketch only has to be sent to pachube once
 02/20/11 - Renamed localclient to PachubeClient()
 11/27/11 - added watchdog timer (but it's commented out until I can verify bootload supports it).  Ref: http://tushev.org/articles/electronics/48-arduino-and-watchdog-timer
            There may be additional work to do to get watchdog timer to work, see this post: http://community.pachube.com/arduino/ethernet/watchdog
 11/27/11 - Swapped Duamelinavo for Uno because Macbook was getting errors trying to upload sketch
 12/04/11 - Updated code to work with Nanode.  Added #IF to set is code should be compiled for Nanode or Arduino
 12/12/11 - changed update interval from 10 seconds to 15 seconds.  Added code to calculate BTUH and feed to datastream 0,  Added BlinkLED function
 12/13/11 - Major rewrite
 12/14/11 - Changed Meter reading to long int
 12/17/11 - Uploaded new program to Arduino in Vermont
 12/19/11 - Took out all Pachube GET code.  I'll get outside temp from thermistor
 12/21/11 - Added 3 new temp sensors: Basebaord, Resistor, air supply, removed WDC code (even though it was commented out).  Removed avarage temps
 12/31/11 - Added code for A5 butotn on ProtoWireShield, when pressed it will decrase counter
 01/09/12 - added Pin 8 reset for Ethernet shield Ref: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1258355934.  Alternatively, you can install R/C circuit ref: http://marco.guardigli.it/2010/11/arduino-wiznet-ethernet-shield-proper.html?m=1
 01/10/12 - Added code for to check if connection is okay, if not, change blinking of green LED.  Added code so ADC 05, which is connected to WireShield button, can also be used with a thermisor
 01/15/12 - Added restor ProtoShieldBoard 
 01/21/12 - Added watchdog timer code, but it wasn't working properly, so I put it around #if
 01/22/12 - Added code to clear buffer after connection failure.  I saw this suggestion on the internet.
 01/31/12 - Added debounce code on pulse input.  Validated data before sent to pachube to make sure it's doesn't send unrealistic values
 03/04/12 - Added conditional #if for Arduino 100.  Reference http://blog.makezine.com/2011/12/01/arduino-1-0-is-out-heres-what-you-need-to-know/
 06/02/12 - Feed is freezing after about 2 hours.  Use ERxPachube to upload
            Moved a lot of code into specific functions.  Took out GMT Time code, not supported by ERxPachube library
 09/20/12 - Made success counter a byte so it rolls over at 255.  Failures reset when there is a successful upload. Put Serial.print() strings in flash wtih F()
            used dtostrf() to round first two streams to 1 decimal.  Added FreeRam function and software reset function. Instead of resetting Ethernet shield
            after 50 upload failures, code reboots after 10 failures.
 4/12/13    Added token.h, replaced erxpachube.h with cosm.h, removed temperature sensors, Removed WDT
 4/13/13    Sketch reads gas price from cosm stream.  Use interrupt for pulse input, use time libraries to get NTP time and set alarms for midnight stat calculation and 15 minute therm calculation
 
*/

#include <SPI.h>        // Required for Ethernet as of Arduino 0021, ref: http://arduino.cc/en/Reference/SPI
#include <Ethernet.h>   // Functions to control Ethernet shield Ref: http://www.arduino.cc/en/Reference/Ethernet
#include <EthernetUdp.h>// core library
#include <HttpClient.h> // used by Cosm.h  http://github.com/amcewen/HttpClient
#include <avr/eeprom.h> // Functions to read/write EEPROM
#include <Cosm.h>       // http://github.com/cosm/cosm-arduino
#include <tokens.h>     // COSM API key
#include <Time.h>       // http://www.pjrc.com/teensy/td_libs_Time.html#ntp
#include <TimeAlarms.h> // http://code.google.com/p/arduino-time/source/browse/trunk/TimeAlarms/


// #define PRER3ETHERNETSHLD // comment out if using R3 or later shield.  Older shields need a hard reset after power up
#define CRESTVIEW         // If at Crestview, change IP, Feed ID and use internal pull-up resistors

// #define USEDISPLAY       // Comment out if not using the OLED Display

#define UPDATE_INTERVAL         20000   // If the connection is good wait 15 seconds before updating again - should not be less than 5 seconds per cosm requirements
#define EEPROM_ADDR_GASPULSE        0   // Location in EEPROM where gas pulse will be saved (0-512 bytes)

// Network
byte mac[] = { 0x90, 0xA2, 0xDA, 0xEF, 0xFE, 0x82 }; // Assign MAC Address to ethernet shield
#ifdef CRESTVIEW
  #define COSM_FEED_ID       4663     // Test feed http://cosm.com/feeds/4663
  byte ip[] = { 192, 168, 216, 53 };  // Crestview IP
#else
  #define COSM_FEED_ID       4038     // cosm feed ID, http://cosm.com/feeds/4038
  byte ip[] = { 192, 168, 46, 94 };  // Vermont IP
#endif

// Setup variables for time and alarm functions
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov
const int timeZone = -5;  // Eastern Standard Time (USA)
EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

uint8_t successes = 0;     // Cosm upload success counter, using a byte will make counter rollover at 255, which is what we want
uint8_t failures  = 0;     // Cosm upload failure counter

// Propane and pulses
uint32_t MeterReading =             0;    // Reading on gas meter
uint32_t Meter15MinAgo =            0;    // Meter reading at 15 minute intervals  
uint32_t MeterStartDay =            0;    // Meter reading at begenning of day
   float therms =                 0.0;    // Propane usage converter to therms
   float yesterdayGasCost =       0.0;    // Previous day's propane cost
     int GasCuFtYesterday =         0;    // At midnight figure out the previous day's cubic feet
   float PropanePrice =          1.63;    // Propane cost per gallon
    byte pulseState_now =         LOW;    // current state of the gas meter pulse: HIGH or LOW
    byte pulseState_last =        LOW;    // previous state of the gas meter pulse
    byte ProtoShldBtnState_now =  LOW;    // current state of button on ProtoWireSheild connected to A5
    byte ProtoShldBtnState_last = LOW;    // current state of button on ProtoWireSheild connected to A5

// I/O Pins
#define GasMeterPulsePin      2   // digital pin 2 input from pulse on gas meter, interrupt int.0
#define GasMeterLEDPin        3   // digital pin 3 to light LED when pulse is active
#define ledHeartbeatPin       4   // LED flashes quickly twice to indicate programming is not frozen 
#define ResetEthernetPin      8   // Reset Ethernet shield
#define ProtoWireShieldBtn    5   // Analog Input connected to button on protoshield - decreases pulse meter count
// Use D6 for OLED reset


// Define Cosm datastreem IDs.  cosm.h requires this to ba a string
#define THERM_H               "0"     // Therms/hour based on 15 minute intervals.  1 Therm = 100,000 BTU
#define PULSE_METER_COUNT     "1"     // Pulse meter counter 
#define YESTERDAY_GAS_CUFT    "2"     // Yesterdays propane usage (in cu-ft)
#define YESTERDAY_GAS_COST    "3"     // Yesterdays propane cost (dollars)
#define CONNECT_SUCCESSES    "10"     // http connection successes
#define CONNECT_FAILURES     "11"     // http connection successes
#define GAS_PRICE_FEED       "12"     // Gas price, this is put in manually into COSM when you get a bill.  Sketch will read value for cost calculation
#define NUM_STREAMS            6      // Number of cosm streams 

// Setup COSM data stream array
CosmDatastream gasMeterStreams[] = 
{
  CosmDatastream(THERM_H,            strlen(THERM_H),            DATASTREAM_FLOAT),
  CosmDatastream(PULSE_METER_COUNT,  strlen(PULSE_METER_COUNT),  DATASTREAM_FLOAT),
  CosmDatastream(YESTERDAY_GAS_CUFT, strlen(YESTERDAY_GAS_CUFT), DATASTREAM_FLOAT),
  CosmDatastream(YESTERDAY_GAS_COST, strlen(YESTERDAY_GAS_COST), DATASTREAM_FLOAT),
  CosmDatastream(CONNECT_SUCCESSES,  strlen(CONNECT_SUCCESSES),  DATASTREAM_INT),
  CosmDatastream(CONNECT_FAILURES,  strlen(CONNECT_FAILURES),  DATASTREAM_INT)
};

CosmDatastream gasCostStream[] = 
{
  CosmDatastream(GAS_PRICE_FEED, strlen(GAS_PRICE_FEED), DATASTREAM_FLOAT),
};

// Wrap the datastreams into a feed
CosmFeed GasMeterFeed(COSM_FEED_ID, gasMeterStreams, NUM_STREAMS);
CosmFeed GasCostFeed(COSM_FEED_ID, gasCostStream, 1);

byte cosmConnectionOK = true; // If sketch is connecting to cosm ok, then set this to true, if not, set to false and change blinking on green LED


EthernetClient client;
CosmClient cosmclient(client);

uint32_t next_connect;   // Timer for uploading to COSM
char lcdBuff[22];  // character buffer to send text to LCD display

//Declare function prototypes (Reference: http://arduino.cc/forum/index.php/topic,87155.0.html)
void BlinkLED(int LEDtoBlink);
void GetMeterReadingFromEEPROM(uint32_t InitializeMeterReading);
void reset_ethernet_shield(int ResettPin);
void ReadPulse();
void ReadDecrementPulseBtn();
void calcThermUsage();
void calcYesterdayStats();

// ==============================================================================================================================================
//   Setup
// ==============================================================================================================================================
void setup()
{
  
  Serial.begin(9600);  // Need if using serial monitor
  Serial.println(F("Gas Meter v3 Setup"));
  delay(500);

  pinMode(GasMeterPulsePin, INPUT);       // Gas meter pulse, define digital as input
  pinMode(GasMeterLEDPin,   OUTPUT);        // LED that lights when gas meter pulse is on
  pinMode(ledHeartbeatPin,  OUTPUT);       // LED flashed every couple seconds to show sketch is running
  pinMode(ResetEthernetPin, OUTPUT);      // Resets ethernet shield

  #ifdef CRESTVIEW
    digitalWrite(GasMeterPulsePin, HIGH); // turn on pullup resistors  
  #endif

  #ifdef PRER3ETHERNETSHLD
    // Use Digital I/O to hard reset Ethernet shield
    reset_ethernet_shield(ResetEthernetPin); 
  #endif
  
  Ethernet.begin(mac, ip);

  Serial.println(Ethernet.localIP());
  Udp.begin(localPort);

  // Set the time for the alarm functions
  time_t t = getNtpTime();
  setTime(hour(t),minute(t),second(t),month(t),day(t),year(t));

  char timebuf[16];
  sprintf(timebuf, "Time: %02d:%02d:%02d", hour(),minute(),second());
  Serial.println(timebuf);

  // create the time events
  Alarm.alarmRepeat(0,0,0, calcYesterdayStats);   // Midnight every day
  Alarm.timerRepeat(15*60, calcThermUsage);  // timer for every 15 minutes    

  // Get meter reading from EEPROM, if you need sketch to use a new (higher) number, pass that to the function.
  // Function will compare EEPROM to number passed to it and use the higher one
  GetMeterReadingFromEEPROM(92458);
  
  setupOled();  // Initialize OLED display
  
  attachInterrupt(0, ReadPulse, FALLING);  // http://arduino.cc/en/Reference/AttachInterrupt
  dispText("End setup ", 2, true);
  sprintf(lcdBuff, "Free RAM = &d", freeRam(false));
  dispText(lcdBuff, 3, false);  // Printout free ram available

} // End Setup()


// ==============================================================================================================================================
//   Main Loop
// ==============================================================================================================================================
void loop()
{
  Alarm.delay(0); //  Alarm.delay must be used instead of the usual arduino delay function because the alarms are 
                  // serviced in the Alarm.delay method.
  
  BlinkLED(ledHeartbeatPin);  // Heartbeat

  // Turn on LED when pulse is on
  digitalWrite(GasMeterLEDPin, digitalRead(GasMeterPulsePin));
    
  ReadDecrementPulseBtn(); // read pulse from decriment pushbutton on shield

  //--------------------------------
  // Send data to Cosm
  //--------------------------------
  if ((long)(millis() - next_connect) >= 0 )
  {
    next_connect = millis() + UPDATE_INTERVAL;
    Serial.println(F("Uploading to Cosm..."));
    
    gasMeterStreams[0].setFloat(therms);
    gasMeterStreams[1].setFloat(MeterReading);
    gasMeterStreams[2].setFloat(GasCuFtYesterday);
    gasMeterStreams[3].setFloat(yesterdayGasCost);
    gasMeterStreams[4].setInt(successes);
    gasMeterStreams[5].setInt(failures);

    int status = cosmclient.put(GasMeterFeed, COSM_API_KEY);
    switch (status)
    {
      case 200:
        Serial.print(F("Upload succeeded, Success="));
        cosmConnectionOK = true;
        Serial.println(successes++);
        failures = 0; // Reset failures
        break;
      default:
        Serial.print(F("Upload failed, Error: = "));
        Serial.println(status);
        cosmConnectionOK = false;
        failures++;
    }
    
  } // end upload to cosm 
  
  // if connection failures reach 10, reboot 
  if(failures >= 10)
  {
    Serial.println(F("Over 10 Errors, software reset"));
    software_Reset();
  }

} // End loop()

// ==============================================================================================================================================
// Called by alarm library at midnight
// ==============================================================================================================================================
void calcYesterdayStats()
{
  Serial.println(F("New Day started")); 

  PropanePrice = GetGasPrice();  // Get new gas price
    
  // Calculate gas usage and cost for previous day
  GasCuFtYesterday = MeterReading - MeterStartDay;
  MeterStartDay = MeterReading;
  if(GasCuFtYesterday > 1000 || GasCuFtYesterday < 0) {GasCuFtYesterday = 0;}
  yesterdayGasCost = GasCuFtYesterday * 0.02549 * PropanePrice;
 
}  // calcYesterdayStats()

// ==============================================================================================================================================
// Read cosm gas price stream.  This stream is updated manually when bills come in
// This function takes about a minute to process
// ==============================================================================================================================================
float GetGasPrice()
{
  float oldGasPrice = PropanePrice;
  int status = cosmclient.get(GasCostFeed, COSM_API_KEY);
  if (status == 200 )
  { return GasCostFeed[0].getFloat(); }
  else
  { return oldGasPrice; }

}  // end GetGasPrice()

// ==============================================================================================================================================
// Read gas meter pulses stored in EEPROM
// ==============================================================================================================================================
void GetMeterReadingFromEEPROM(uint32_t InitializeMeterReading)
{
  // Meter pulses stored in EEPROM memory
  uint32_t StoredMeterReading;
  
  // Read last gas meter value from EEPROM.  If unit loses power, this value is used as starting point
  StoredMeterReading = eeprom_read_dword((const uint32_t *)EEPROM_ADDR_GASPULSE);
  
  // If InitializeMeterReading is greater then what's stored in EEPROM, it means a 
  // new value was loaded into EEPROM and the new value should be used
  if (InitializeMeterReading > StoredMeterReading) 
  {
    eeprom_write_dword((uint32_t *)EEPROM_ADDR_GASPULSE, InitializeMeterReading); 
    StoredMeterReading = InitializeMeterReading;
  }
 
  MeterStartDay = StoredMeterReading;
  MeterReading =  StoredMeterReading; 
  Meter15MinAgo = StoredMeterReading;
  
}  // End GetMeterReadingFromEEPROM()

// ==============================================================================================================================================
//    Read the Gas Meter Pulse input pin 
// ==============================================================================================================================================
void ReadPulse()
{
    pulseState_now = digitalRead(GasMeterPulsePin);
          
    // Compare the pulseState to its previous state
    if ( pulseState_now != pulseState_last ) 
    {
      // if the state has changed, check input one more time, then increment the counter
      if ( pulseState_now == HIGH ) 
      {
        delay(100);
        pulseState_now = digitalRead(GasMeterPulsePin); // Check input again after 100 mS debounce delay.  If it's still high then increment conter
        if (pulseState_now == HIGH)  
        {
          // if the current state is still HIGH then the pulse went from off to on:
          MeterReading++; 
          eeprom_write_dword((uint32_t *)EEPROM_ADDR_GASPULSE, MeterReading); // Save reading to EEPRROM
        }
      }
      pulseState_last = pulseState_now;  // save the current state as the last state, for next time through the loop
    } // End pulse state changed

    if( MeterReading  < 0 ) 
    { MeterReading  = 0; }
    
} // End ReadPulse()

// ==============================================================================================================================================
//    Read ProtoWireShield button on A5, if it's zero, decrement pulse counter
// ==============================================================================================================================================
void ReadDecrementPulseBtn()
{
    // Check for button on ProtoWireShield - when pressed decrease pulse counter by one
    if (analogRead(ProtoWireShieldBtn) == 0)
    { ProtoShldBtnState_now = HIGH; }
    else
    { ProtoShldBtnState_now = LOW; }
      
    // Compare the current button state to its previous state
    if (ProtoShldBtnState_now != ProtoShldBtnState_last) 
    {
      // if the state has changed, decrement the counter
      if (ProtoShldBtnState_now == HIGH) 
      { // if the current state is HIGH then the button went from off to on:
        MeterReading--;
        eeprom_write_dword((uint32_t *)EEPROM_ADDR_GASPULSE, MeterReading); // Save reading to EEPRROM
      }
      ProtoShldBtnState_last = ProtoShldBtnState_now;  // save the current state as the last state, for next time through the loop
    } // End protowireshield button pulse state changed

    if(MeterReading  < 0) 
    { MeterReading  = 0; }
  
}  // end ReadDecrementPulseBtn()



// ==============================================================================================================================================
//  Calculate therms used in the last 15 minutes
// ==============================================================================================================================================
void calcThermUsage()  
{ 
  //------------------------------------
  // Check for new 15 minute interval
  // Calculate BHU/hour and avg temperatures
  //------------------------------------
//  static uint32_t FifteenMinTimer;

//  if (long(millis() - FifteenMinTimer) >=0)
//  {
//    FifteenMinTimer = millis() + 900000;  // add 15 minutes to timer
    therms = (MeterReading - Meter15MinAgo) * 2328.0 * 4.0 / 100000.0;   // convert cu-ft used to BTU (2328) then multiply by 4 to convert from 15 minutes to hours
    
    if(therms >  100 || therms < 0) {therms = 0;}

    Serial.println(F("--- New 15 Minute Interval ---"));
    Serial.print(F("Cu-Ft Used in last 15 minutes = "));
    Serial.println(MeterReading - Meter15MinAgo);
    Serial.print(F("Therms/Hour = "));
    Serial.println(therms);

    Meter15MinAgo = MeterReading;
//  } 

} // End calcThermUsage()


// ==============================================================================================================================================
//   Blink LED twice quickly, wait a couple seconds and repeat.  Used to verify arduino is not frozen
//   Here is link to making LED fade up and down: http://www.hobbytronics.co.uk/tutorials-code/arduino-tutorials/arduino-tutorial5-fade
// ==============================================================================================================================================
void BlinkLED(int LEDtoBlink) 
{
  unsigned long LEDBlinkTime[] = {30, 30+150, 30+150+30, 30+150+30+2000}; 
  static unsigned long LEDBlinkStart;
  
  if (cosmConnectionOK == true)
  {
    // Set LED on/off sequence in mS, On for 30ms, off for 150, on for 30mS, off for 2000mS, repeat
    LEDBlinkTime[0] = 30;
    LEDBlinkTime[1] = 30+150;
    LEDBlinkTime[2] = 30+150+30;
    LEDBlinkTime[3] = 30-150+30+2000;
  }
  else
  {
    // If there are connection problems, flash LED 2x per second
    LEDBlinkTime[0] = 250;
    LEDBlinkTime[1] = 500;
    LEDBlinkTime[2] = 750;
    LEDBlinkTime[3] = 1000;
  }
  
  // See if millis() counter rolled over.  If so, reset LEDBlinkStart
  if (millis() <= LEDBlinkStart) 
  { LEDBlinkStart = millis(); }
  
  if ( millis() <= LEDBlinkStart + LEDBlinkTime[0] ) 
  {
    digitalWrite(LEDtoBlink, HIGH);   // LED on for short blink
  }
  else if ( millis() <= LEDBlinkStart + LEDBlinkTime[1] ) 
  {
    digitalWrite(LEDtoBlink, LOW);   // LED off for short time    
  }  
  else if ( millis() <= LEDBlinkStart + LEDBlinkTime[2] ) 
  {
    digitalWrite(LEDtoBlink, HIGH);   // LED on for short blink
  }  
  else if ( millis() <= LEDBlinkStart + LEDBlinkTime[3] ) 
  {
    digitalWrite(LEDtoBlink, LOW);   //  LED off for long time, couple seconds    
  }  
  else 
  {
    LEDBlinkStart = millis(); // Start over 
  }  

}  // End BlinkLED()


/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  return 0; // return 0 if unable to get the time
}  // end getNtpTime()

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
} // end sendNTPpacket()


// ==============================================================================================================================================
// The ethernet shield can be reset with one of the digital I/O pins
// ==============================================================================================================================================
void reset_ethernet_shield(int ResettPin)
{
  // Connect Ethernet shield reset to Pin 8 on arduino, code below will reset 
  Serial.println(F("Reset ethernet shield with output pin"));
  delay(500);
  digitalWrite(ResettPin, HIGH);
  delay(50);
  digitalWrite(ResettPin, LOW);  // This is the reset
  delay(50);
  digitalWrite(ResettPin, HIGH);
  delay(100);
  
}  // End reset_ethernet_shield()


//==========================================================================================================================
// Restarts program from beginning but does not reset the peripherals and registers
// Reference: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1241733710
/*
                   .d888 888                                                   8888888b.                            888    
                  d88P"  888                                                   888   Y88b                           888    
                  888    888                                                   888    888                           888    
.d8888b   .d88b.  888888 888888 888  888  888  8888b.  888d888 .d88b.          888   d88P .d88b.  .d8888b   .d88b.  888888 
88K      d88""88b 888    888    888  888  888     "88b 888P"  d8P  Y8b         8888888P" d8P  Y8b 88K      d8P  Y8b 888    
"Y8888b. 888  888 888    888    888  888  888 .d888888 888    88888888         888 T88b  88888888 "Y8888b. 88888888 888    
     X88 Y88..88P 888    Y88b.  Y88b 888 d88P 888  888 888    Y8b.             888  T88b Y8b.          X88 Y8b.     Y88b.  
 88888P'  "Y88P"  888     "Y888  "Y8888888P"  "Y888888 888     "Y8888 88888888 888   T88b "Y8888   88888P'  "Y8888   "Y888 
                                                                                                                           
*/
// Reboot the arduino
//==========================================================================================================================
void software_Reset(void) 
{
  asm volatile ("  jmp 0");  
} // End software_Reset()


/*
 .d888                          8888888b.                         
d88P"                           888   Y88b                        
888                             888    888                        
888888 888d888 .d88b.   .d88b.  888   d88P  8888b.  88888b.d88b.  
888    888P"  d8P  Y8b d8P  Y8b 8888888P"      "88b 888 "888 "88b 
888    888    88888888 88888888 888 T88b   .d888888 888  888  888 
888    888    Y8b.     Y8b.     888  T88b  888  888 888  888  888 
888    888     "Y8888   "Y8888  888   T88b "Y888888 888  888  888 
*/
//==========================================================================================================================
// Return the amount of free SROM
// Parameters - true: Print out RAM, false: Don't print
// http://www.controllerprojects.com/2011/05/23/determining-sram-usage-on-arduino/
//==========================================================================================================================
int freeRam(bool PrintRam) 
{
  int freeSRAM;
  extern int __heap_start, *__brkval; 
  int v; 
  
  freeSRAM =  (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 

  if(PrintRam)
  {
  Serial.print(F("RAM "));
  Serial.println(freeSRAM);
  }
  return freeSRAM;
  
} // freeRam()
