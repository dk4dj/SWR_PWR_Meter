/**************************************************
   SWR_PWR_Meter with touch screen and webserver
   Copyright (C) 2021  Helmut Gross, DL5HG
***************************************************
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <https://www.gnu.org/licenses/>.

   contact: dl5hg@darc.de

   This program requires the UTFT, URTouch and EEPROM libraries.
   It is assumed that the display module is connected to an
   appropriate shield or that you know how to change the pin
   numbers in the setup.
 **********************************************************************/

//#include <UTFT.h>
//#include <URTouch.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h>

/*################################################################
  ##                    initial system setup                    ##
  ##############################################################*/
// --------------------------------------------------------------
//  runtime control
// --------------------------------------------------------------
String ver = "1.36"; // version of program
#define caltype 0    // calibration type
//                      0 -> load 2 point cal data
//                      1 -> load cal table data
//                      2 -> overwrite eeprom with 2 point cal data set by this program
//                      3 -> overwrite eeprom with cal table set by this program
int webserver = 1;   // webserver on/off 1/0
int webcon    = 0;   // webclient connection status (initially 0)
#define debug 1      // displays infos via serial output (for debugging only)
#define avg   3      // number of averages for measured coupler samples
#define wgt   1      // weight of previous averaged value

// --------------------------------------------------------------
//  webserver setup
// --------------------------------------------------------------
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };   // physical mac address
byte ip[] = { 192, 168, 2, 90 };                       // ip in lan (address for webservice)
byte gateway[] = { 192, 168, 2, 1 };                 // internet access via router
byte subnet[] = { 255, 255, 255, 0 };                  // subnet mask

// -----------------------------------------------------------
// Initialize display
// Set the pins to the correct ones for your development board
// -----------------------------------------------------------
// Standard Arduino Uno/2009 Shield            : <display model>,19,18,17,16
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
// CTE TFT LCD/SD Shield for Arduino Due       : <display model>,25,26,27,28
// Teensy 3.x TFT Test Board                   : <display model>,23,22, 3, 4
// ElecHouse TFT LCD/SD Shield for Arduino Due : <display model>,22,23,31,33
//
// Remember to change the model parameter to suit your display module!
//UTFT    myGLCD(ITDB32S_V2, 38, 39, 40,  41); // hg board
//                         RS, WR, CS, RST

// -----------------------------------------------------------
// Initialize touchscreen
// Set the pins to the correct ones for your development board
// -----------------------------------------------------------
// Standard Arduino Uno/2009 Shield            : 15,10,14, 9, 8
// Standard Arduino Mega/Due shield            :  6, 5, 4, 3, 2
// CTE TFT LCD/SD Shield for Arduino Due       :  6, 5, 4, 3, 2
// Teensy 3.x TFT Test Board                   : 26,31,27,28,29
// ElecHouse TFT LCD/SD Shield for Arduino Due : 25,26,27,29,30
// ------------------------------------------------------------
//URTouch  myTouch( 6, 5, 4, 3, 2); // hg board original without ethernet shield
//URTouch  myTouch(  46,  45,   44,   43,   42);  // hg board with ethernet shield
//               TCLK, TCS, TDIN, TOUT, TIRQ

// ------------------------------------------------------------
//  Declare which fonts we will be using
// ------------------------------------------------------------
//extern uint8_t BigFont[];
//extern uint8_t SmallFont[];

// ------------------------------------------------------------
//  setup I/O pins
// ------------------------------------------------------------
#define fwd1Pin 6  // input fwd voltage channel 1 (HF)
#define rev1Pin 7  // input rev voltage channel 1 (HF)
#define fwd2Pin 8  // input fwd voltage channel 2 (VHF)
#define rev2Pin 9  // input rev voltage channel 2 (VHF)
//#define lcdPin 14 // input lcd on/off
#define expPin 15 // input external TRX power on/off state
#define rstPin 16 // output for ethernet shield reset
#define trxPin 48 // output TRX on/off switch
#define powPin 49 // output power switch

// ------------------------------------------------------------
// setup screen positions
// ------------------------------------------------------------
//int x, y;
//#define butx 63   // button delta x position in pixels
//#define buty 190  // button position in pixels from top
//#define butl 0    // button position in pixels from left
//#define butw 63   // button width/height in pixels

// ------------------------------------------------------------
//  setup runtime system variables
// ------------------------------------------------------------
#define eeAddress 0           // eeprom address for caltype 0 (2 points)
#define eeAddressTable 100    // eeprom address for caltype 1 (table)
int dBmlowlim = 0;            // lower fwd power limit in dBm for swr bar diagram
int changed   = 0;            // variable set to 1 if button status has changed
int cal              = 0;     // calibration mode active if set to 1
int screen           = 0;     // screen state 0 = off, 1 = on (default)
int trxState         = 0;     // TRX status read via digital input from TRX
int trxOnDelay       = 1000;  // switch on delay for TRX control in ms
int trxOffDelay      = 3000;  // switch off delay for TRX control in ms
int calcursor        = 0;     // cursor for cal table entity selection
int asize            = 0;     // for cursor control
int sum              = 0;     // sum of averaged integer values
int avgnum           = 0;     // counter for averaging
int bitmask          = 1020;  // mask two least bits of analog input
char header1[16]     = "SWR_PWR_Meter V\0";
char header2[10]     = "by DL5HG \0";
char calDatHeader[8] = "cal_Dat\0";
char calTabHeader[8] = "cal_Tab\0";
String mode          = "HF";  // mode HF/VHF
String power         = "OFF"; // power switch setting on display
String trx           = "OFF"; // TRX switch setting on display
String intswr        = "1";   // integer swr value (1..100) made from float swr

// ------------------------------------------------------------
//  setup measurement and calculation variables
// ------------------------------------------------------------
float fwd_v1[avg];      // measured fwd voltage channel 1 in mV
float rev_v1[avg];      // measured rev voltage channel 1 in mV
float fwd_v2[avg];      // measured fwd voltage channel 2 in mV
float rev_v2[avg];      // measured rev voltage channel 2 in mV
float fwd_v1a;          // averaged fwd voltage channel 1 in mV
float rev_v1a;          // averaged rev voltage channel 1 in mV
float fwd_v2a;          // averaged fwd voltage channel 2 in mV
float rev_v2a;          // averaged rev voltage channel 2 in mV
float fwd_db1 = 0;      // calculated fwd power channel 1 in dBm
float rev_db1 = 0;      // calculated rev power channel 1 in dBm
float fwd_db2 = 0;      // calculated fwd power channel 2 in dBm
float rev_db2 = 0;      // calculated rev power channel 2 in dBm
float swr1 = 0;         // calculated swr of channel 1
float swr2 = 0;         // calculated swr of channel 2
float swr = 0;          // swr either from swr1 or swr2 depending on mode
float oldval = 0;       // initial value for bar display on lcd
float pval = 0;         // power value in table mode
float dpow = 0;         // delta power in table mode
float dvolt = 0;        // delta voltage in table mode
const float mvperstep = 1000 * 5.0 / 1024; // voltage step of analog input

// ---------------------------------------------------------------------------
// 2 point calibration values of DL5HG HF/VHF prototype bidirectional couplers
// these values can be modified in caltype = 0 mode via LCD setup
// or loaded into EEPROM via caltype = 2 mode
// (if EEPROM is empty, this is done automatically by the program)
// ---------------------------------------------------------------------------
struct caldatObj {
  char calDatHeader[8];
  float hfdbm1;
  float hfdbm2;
  float vhfdbm1;
  float vhfdbm2;
  int hfref1f;
  int hfref2f;
  int hfref1r;
  int hfref2r;
  int vhfref1f;
  int vhfref2f;
  int vhfref1r;
  int vhfref2r;
};
// Calibration data for 2-point calibration
caldatObj caldat = {
  "cal_Dat\0", // 2-point calibration header
  40,          // hfdbm1 calibration power level high in dBm
  0,           // hfdbm2 calibration power level low in dBm
  40,          // vhfdbm1 calibration power level high in dBm
  0,           // vhfdbm2 calibration power level low in dBm
  1884,        // calibrated fwd voltage at hfdbm1
  996,         // calibrated fwd voltage at hfdbm2
  1830,        // calibrated rev voltage at hfdbm1
  944,         // calibrated rev voltage at hfdbm2
  1884,        // calibrated fwd voltage at vhfdbm1
  996,        // calibrated fwd voltage at vhfdbm2
  1830,        // calibrated rev voltage at vhfdbm1
  944          // calibrated rev voltage at vhfdbm2
};
caldatObj caldat0 = {"\0", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // dummy for input test
float hfmvdbf  = (caldat.hfref1f - caldat.hfref2f) / (caldat.hfdbm1 - caldat.hfdbm2); // mV/dB forward
float hfmvdbr  = (caldat.hfref1r - caldat.hfref2r) / (caldat.hfdbm1 - caldat.hfdbm2); // mV/dB reverse
float hfdb0f   = caldat.hfref2f - hfmvdbf * caldat.hfdbm2; // mV @ 0 dBm
float hfdb0r   = caldat.hfref2r - hfmvdbr * caldat.hfdbm2; // mV @ 0 dBm
float vhfmvdbf = (caldat.vhfref1f - caldat.vhfref2f) / (caldat.vhfdbm1 - caldat.vhfdbm2); // mV/dB forward
float vhfmvdbr = (caldat.vhfref1r - caldat.vhfref2r) / (caldat.vhfdbm1 - caldat.vhfdbm2); // mV/dB reverse
float vhfdb0f  = caldat.vhfref2f - vhfmvdbf * caldat.vhfdbm2; // mV @ 0 dBm
float vhfdb0r  = caldat.vhfref2r - vhfmvdbr * caldat.vhfdbm2; // mV @ 0 dBm
float fwd_pow1, fwd_eff1, rev_pow1, rev_eff1;
float fwd_pow2, fwd_eff2, rev_pow2, rev_eff2;

// -------------------------------------------------------------------------------------
// calibration table values of DL5HG HF/VHF prototype bidirectional couplers
// these values can only be modified here and loaded into EEPROM via caltype = 3 mode
// (if EEPROM is new, this is done automatically by the program)
// -------------------------------------------------------------------------------------
const int arraysize = 12;  // max size of each calibration table (less than 12 possible)
struct caltabObj {
  char calTabHeader[8];
  float powtab1[arraysize];
  int fwdtab1[arraysize];
  int revtab1[arraysize];
  float powtab2[arraysize];
  int fwdtab2[arraysize];
  int revtab2[arraysize];
};
// Calibration data for calibration table
caltabObj caltab = {
  "cal_Tab\0", // calibration table header
  {   50,   47,   43,   40,   37,   33,   30,   20,   10,   0, -10, -20},  // HF cal power level in dBm
  { 2086, 2031, 1943, 1884, 1826, 1744, 1666, 1435, 1217, 996, 771, 556},  // HF cal fwd voltage in mV
  { 2042, 1982, 1891, 1830, 1767, 1695, 1618, 1386, 1170, 944, 725, 506},  // HF cal rev voltage in mV
  {   50,   47,   43,   40,   37,   33,   30,   20,   10,   0, -10, -20},  // VHF cal power level in dBm
  { 2086, 2031, 1943, 1884, 1826, 1744, 1666, 1435, 1217, 996, 771, 556},  // VHF cal fwd voltage in mV
  { 2042, 1982, 1891, 1830, 1767, 1695, 1618, 1386, 1170, 944, 725, 506}   // VHF cal rev voltage in mV
};
caltabObj caltab0 = {"\0", 0, 0, 0, 0, 0, 0}; // dummy for input test

// ----------------------------------------
// initialize Ethernet access for webserver
// ----------------------------------------
EthernetServer server(80);        // server port
String HTTP_req;                  // Variable to store the HTTP request
int history;                      // history counter checks if window.back is possible
unsigned long starttime = 0;      // start time for webcon timeout
unsigned long timeout = 5000;     // timeout 5 seconds
unsigned long contime = timeout;  // client connection time since last refresh

//##############################################################
void setup()  //             system setup
//##############################################################
{
  pinMode(fwd1Pin, INPUT);
  pinMode(rev1Pin, INPUT);
  pinMode(fwd2Pin, INPUT);
  pinMode(rev2Pin, INPUT);
  pinMode(expPin, INPUT);
  //pinMode(lcdPin, INPUT);
  //digitalWrite(lcdPin, HIGH); // activate 20 kOhm pull-up resistor
  pinMode(powPin, OUTPUT);
  pinMode(trxPin, OUTPUT);
  pinMode(rstPin, OUTPUT);
  resetShields();
  Serial.begin(115200);
//  if (screen) {
//    // Display setup
//    myGLCD.InitLCD();
//    myGLCD.clrScr();
//    myTouch.InitTouch();
//    myTouch.setPrecision(PREC_MEDIUM);
//    presetScreen();
//  }
  if (debug) {
    Serial.print("program version = ");
    Serial.print(ver);
    Serial.print("\tcaltype = ");
    Serial.println(caltype);
  }
  if ((caltype == 0) || (caltype == 2)) {  // read/write 2 point cal data from eeprom
    // read cal data from EEPROM to check header info
    EEPROM.get(eeAddress, caldat0);
    if ((caltype == 2) || (String(caldat0.calDatHeader) != calDatHeader)) {
      // write cal data to EEPROM if no data has been stored before
      // or an overwrite is wanted with caltype = 2
      if (debug) {
        if (caltype == 2) {
          Serial.print("Writing cal_data to EEPROM\n");
        }
        else {
          Serial.println("No calDatHeader found!\nWriting cal_data to EEPROM");
        }
      }
      EEPROM.put(eeAddress, caldat);
    }
    // read cal data from EEPROM
    EEPROM.get(eeAddress, caldat);
    if (debug) {
      Serial.print("Reading cal_data from EEPROM\n");
    }
    if (debug) { // print EEPROM 2 point cal data
      Serial.print("EEPROM data HF CAL1:\n");
      Serial.print(" dBm\tfwd_mV\trev_mV\n");
      Serial.print(caldat.hfdbm1); Serial.print("\t");
      Serial.print(caldat.hfref1f); Serial.print("\t");
      Serial.print(caldat.hfref1r); Serial.print("\n");
      Serial.print(caldat.hfdbm2); Serial.print("\t");
      Serial.print(caldat.hfref2f); Serial.print("\t");
      Serial.print(caldat.hfref2r); Serial.print("\n");
      Serial.print("EEPROM data VHF CAL2:\n");
      Serial.print(" dBm\tfwd_mV\trev_mV\n");
      Serial.print(caldat.vhfdbm1); Serial.print("\t");
      Serial.print(caldat.vhfref1f); Serial.print("\t");
      Serial.print(caldat.vhfref1r); Serial.print("\n");
      Serial.print(caldat.vhfdbm2); Serial.print("\t");
      Serial.print(caldat.vhfref2f); Serial.print("\t");
      Serial.print(caldat.vhfref2r); Serial.print("\n");
    }
  } // end caltype 0 or 2
  else if ((caltype == 1) || (caltype == 3)) {
    // read cal table from EEPROM to check header info
    EEPROM.get(eeAddressTable, caltab0);
    if ((caltype == 3) || (String(caltab0.calTabHeader) != calTabHeader))
    {
      // write cal table to EEPROM if no data has been stored before
      if (debug) {
        if (caltype == 3) {
          Serial.print("Writing cal_tab to EEPROM\n");
        }
        else {
          Serial.println("No calTabHeader found!\nWriting cal_tab to EEPROM");
        }
      }
      EEPROM.put(eeAddressTable, caltab);
    }
    // read cal data from EEPROM
    if (debug) {
      Serial.print("Reading cal_tab from EEPROM\n");
    }
    EEPROM.get(eeAddressTable, caltab);
    if (debug) {
      Serial.print("EEPROM table HF CAL1:\n");
      Serial.print(" dBm\tfwd_mV\trev_mV\n");
      for (int x = 0; x <= arraysize - 1; x++) {
        // read HF table
        Serial.print(caltab.powtab1[x]); Serial.print("\t");
        Serial.print(caltab.fwdtab1[x]); Serial.print("\t");
        Serial.println(caltab.revtab1[x]);
      }
      Serial.print("EEPROM table VHF CAL2:\n");
      Serial.print(" dBm\tfwd_mV\trev_mV\n");
      for (int x = 0; x <= arraysize - 1; x++) {
        // read VHF table
        Serial.print(caltab.powtab2[x]); Serial.print("\t");
        Serial.print(caltab.fwdtab2[x]); Serial.print("\t");
        Serial.println(caltab.revtab2[x]);
      }
    }
  }
  if (webserver) {
    // Webserver setup
    // Ethernet.init(pin) to configure the CS pin
    Ethernet.init(10);  // Most Arduino shields
    // start the Ethernet connection and the server:
    Ethernet.begin(mac, ip, gateway, subnet);
    server.begin();
    if (debug) {
      Serial.print("webserver is at ");
      Serial.println(Ethernet.localIP());
    }
    // following only if SD card is missing
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
  } // end webserver
} // end setup()

//##############################################################
void loop()  //          system loop
//##############################################################
{
  /*************************************************************
  **  measurements, screen and webserver display starts here  **
  *************************************************************/
  updateValues();
//  checkScreenState();
//  if (screen) {
//    drawTRXbutton();
//    myTouch.read();
//    x = myTouch.getX();
//    y = myTouch.getY();
//    if ((y >= 140) && (y <= buty + 50)) // buttons selected
//    {
//      if ((x >= butl) && (x <= butl + butw) && (y >= 190))
//      {
//        waitForIt(butl, buty, butl + butw, buty + butw);
//        if (cal) {
//          cursorcontrol("up");                            // Button up
//          drawCalButtons();
//        }
//        else {
//          mode = "HF";                                    // Button: HF
//          cal = 0;
//          presetScreen();
//        }
//      }
//      if ((x >= butl + butx) && (x <= butl + butx + butw) && (y >= 190))
//      {
//        waitForIt(butl + butx, buty, butl + butx + butw, buty + butw);
//        if (cal) {
//          cursorcontrol("down");                          // Button up
//          drawCalButtons();
//        }
//        else {
//          mode = "VHF";                                    // Button: VHF
//          cal = 0;
//          presetScreen();
//        }
//      }
//      if ((x >= butl + (2 * butx)) && (x <= butl + (2 * butx) + butw) && (y >= 190)) // Button: CAL
//      {
//        waitForIt(butl + (2 * butx), buty, butl + (2 * butx) + butw, buty + butw);
//        if (cal == 0) {
//          cal = 1;  // go into cal mode
//        }
//        else {
//          cal = 0;  // go back to normal mode
//        }
//        presetScreen();
//      }
//      if ((x >= butl + (3 * butx)) && (x <= butl + (3 * butx) + butw) && (y >= 190))
//      {
//        waitForIt(butl + (3 * butx), buty, butl + (3 * butx) + butw, buty + butw);
//        if (cal) {                       // Button: F>T
//          storeData("fwd");
//          drawCalButtons();
//        }
//        else {                           // Button: POW
//          if (power == "ON") {
//            digitalWrite(powPin, LOW);
//            power = "OFF";
//          }
//          else if (power == "OFF") {
//            digitalWrite(powPin, HIGH);
//            power = "ON";
//          }
//          drawButtons();
//        }
//      }
//      if ((x >= butl + (4 * butx)) && (x <= butl + (4 * butx) + butw) && (y >= 190)) // Button: TRX
//      {
//        waitForIt(butl + (4 * butx), buty, butl + (4 * butx) + butw, buty + butw);
//        if (cal) {                       // Button: R>T
//          storeData("rev");
//          drawCalButtons();
//        }
//        else {                           // Button: TRX
//          if ((trxState == 0) && (trx == "ON"))       { // if no response via expPin
//            trx = "OFF";
//          }
//          else if ((trxState == 1) && (trx == "OFF")) { // if trx was already switched on
//            switchTRX(0);
//          }
//          else if ((trxState == 1) && (trx == "ON"))  { // trx switch off sequence
//            switchTRX(0);
//          }
//          else if ((trxState == 0) && (trx == "OFF")) { // trx switch on sequence
//            switchTRX(1);
//          }
//          drawButtons();
//        }
//      }
//    }
//  } // if (screen)

  /****************************************************
  **              web server starts here             **
  ****************************************************/
  if (webserver) {
    // Create a client connection
    EthernetClient client = server.available();
    /*
      When a request is received from a client, we will save the incoming data.
      The while loop that follows will be running as long as the client stays connected.
      We do not recommend changing the following part of the code unless you know exactly
      what you are doing.
    */

    if (client) {  // got client?
      boolean currentLineIsBlank = true;
      while (client.connected()) {
        if (client.available()) {   // client data available to read
          char c = client.read();   // read 1 byte (character) from client
          HTTP_req += c;
          // save the HTTP request 1 char at a time
          // last line of client request is blank and ends with \n
          // respond to client only after last line received
          if (c == '\n' && currentLineIsBlank) {
            // send a standard http response header
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: keep-alive");
            client.println();
            // AJAX request for switch state
            if (HTTP_req.indexOf("ajax_switch") > -1) {
              webcon = 1; // client connected
              starttime = millis();
              // read switch state and analog input
              GetAjaxData(client);
            }
            else {
              // HTTP request for web page
              // send web page - contains JavaScript with AJAX calls
              client.println("<!DOCTYPE html>");
              client.println("<html>");
              client.println("<head>");
              client.println("<title>SWR_PWR_Meter</title>");
              // CSS to style colors and the on/off buttons
              client.println("<style>");
              client.println("html { font-family: Helvetica; color: white; background-color: black; text-align: center;}");
              client.println(".button { background-color: #AAAAAA; color: white; padding: 2px 7px;}"); // grey
              client.println(".button2 {background-color: #00BB32;}"); // dark green
              client.println(".button3 {background-color: #FF0000;}"); // red
              client.println("</style>");
              client.println("<script>");
              client.println("function GetSwitchAnalogData() {");
              client.println("nocache = \"&nocache=\" + Math.random() * 1000000;");
              client.println("var request = new XMLHttpRequest();");
              client.println("request.onreadystatechange = function() {");
              client.println("if (this.readyState == 4) {");
              client.println("if (this.status == 200) {");
              client.println("if (this.responseText != null) {");
              client.println("document.getElementById(\"sw_an_data\").innerHTML = this.responseText;");
              client.println("window.history.back();"); // remove switch trailer from header
              client.println("}}}}");
              client.println("request.open(\"GET\", \"ajax_switch\" + nocache, true);");
              client.println("request.send(null);");
              client.println("setTimeout('GetSwitchAnalogData()', 1000);");
              client.println("}");
              client.println("</script>");
              client.println("</head>");
              client.println("<body onload=\"GetSwitchAnalogData()\">");
              client.println("<div id=\"sw_an_data\">");
              client.println("</div>");
              client.println("</body>");
              client.println("</html>");
            }
            break;
          }
          // every line of text received from the client ends with \r\n
          if (c == '\n') {
            // last character on line of received text
            // starting new line with next character read
            currentLineIsBlank = true;
          }
          else if (c != '\r') {
            // a text character was received from client
            currentLineIsBlank = false;
          }
        } // end if (client.available())
      } // end while (client.connected())
      // display received HTTP request on serial port
      //Serial.print(HTTP_req);
      HTTP_req = "";            // finished with request, empty string
      delay(1);      // give the web browser time to receive the data
      client.stop(); // close the connection
    } // end if (client)
  } // end if (webserver)
} // end loop

/*##############################################################################
  ##                            Required functions                            ##
  ############################################################################*/

//*******************************************************
void resetShields()  //  reset ethernet and LCD shields
//*******************************************************
{
  digitalWrite(rstPin, LOW);
  delay(100);
  digitalWrite(rstPin, HIGH);
} // end resetShields

//**************************************************************************************
void switchTRX(int trxState) // switch TRX on or off with according on/off pulse delay
//**************************************************************************************
{
  int swdly = trxOnDelay;
  if (trxState) {
    trx = "ON";
  }
  else {
    swdly = trxOffDelay;
    trx = "OFF";
  }
  digitalWrite(trxPin, HIGH);
  delay(swdly);
  digitalWrite(trxPin, LOW);
} // end switchTRX

//***********************************************************
//void drawTRXbutton()  // draw TRX button with status colour
//***********************************************************
//{
//  myGLCD.setFont(BigFont);
//  myGLCD.setBackColor(0, 0, 255);
//  trxState = digitalRead(expPin);
//  if ((trx == "ON") && (trxState == 0)) {
//    myGLCD.setColor(255, 0, 0);        // status is changing or incorrect!
//  }
//  else if (trxState == 1) {
//    myGLCD.setColor(0, 255, 0);       // trx is switched on (status "on")
//  }
//  else {
//    myGLCD.setColor(255, 255, 255);   // trx is switched off (status "off")
//  }
//  if (cal == 0) {
//    myGLCD.print("TRX", 8 + 4 * butx, buty + 18);
//  }
//} // end drawTRXbutton

//*********************************************************
//void drawButtons()  // draw normal menu buttons on screen
//*********************************************************
/*
{
  // Draw the row of buttons
  for (x = 0; x < 5; x++)
  {
    myGLCD.setColor(0, 0, 255);
    myGLCD.fillRect (butl + (x * butx), buty, butx + (x * butx), buty + butw);
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRect (butl + (x * butx), buty, butx + (x * butx), buty + butw);
  }
  myGLCD.setFont(BigFont);
  myGLCD.setBackColor(0, 0, 255);
  if (mode == "HF") {
    myGLCD.setColor(0, 255, 0);
  }
  else {
    myGLCD.setColor(255, 255, 255);
  }
  myGLCD.print("HF", 17, buty + 18);

  if (mode == "VHF") {
    myGLCD.setColor(0, 255, 0);
  }
  else {
    myGLCD.setColor(255, 255, 255);
  }
  myGLCD.print("VHF", 8 + butx, buty + 18);

  if (cal) {
    myGLCD.setColor(255, 0, 0);
  }
  else {
    myGLCD.setColor(255, 255, 255);
  }
  myGLCD.print("CAL", 8 + 2 * butx, buty + 18);

  if (power == "ON") {
    myGLCD.setColor(0, 255, 0);
  }
  else {
    myGLCD.setColor(255, 255, 255);
  }
  myGLCD.print("POW", 8 + 3 * butx, buty + 18);
} // end drawButtons
*/
//************************************************************
//void drawCalButtons()  // draw calibration buttons on screen
//************************************************************
/*
{
  // Draw the row of buttons
  for (x = 0; x < 5; x++)
  {
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect (butl + (x * butx), buty, butx + (x * butx), buty + butw);
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRect (butl + (x * butx), buty, butx + (x * butx), buty + butw);
  }
  myGLCD.setFont(BigFont);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("up", 17, buty + 18);
  myGLCD.print("dwn", 8 + butx, buty + 18);
  myGLCD.setColor(255, 0, 0);
  myGLCD.print("CAL", 8 + 2 * butx, buty + 18);
  myGLCD.setColor(255, 255, 255);
  myGLCD.print("F>T", 8 + 3 * butx, buty + 18);
  myGLCD.print("R>T", 8 + 4 * butx, buty + 18);

  // print caltable in small font on LCD
  myGLCD.setFont(SmallFont);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.setColor(255, 255, 255);
  if ((caltype == 0) || (caltype == 2)) {
    if (mode == "HF") {
      myGLCD.print("EEPROM 2 point calibration data (HF)", 0, 45);
    }
    else if (mode == "VHF") {
      myGLCD.print("EEPROM 2 point calibration data (VHF)", 0, 45);
    }
    myGLCD.print("  dBm   fwd_mV  rev_mV", 0, 60);
    if (mode == "HF") {
      // read HF table
      myGLCD.printNumF(caldat.hfdbm1, 1, 0, 75, '.', 6, ' ');
      myGLCD.printNumI(caldat.hfref1f, 70, 75, 4);
      myGLCD.printNumI(caldat.hfref1r, 130, 75, 4);
      myGLCD.printNumF(caldat.hfdbm2, 1, 0, 85, '.', 6, ' ');
      myGLCD.printNumI(caldat.hfref2f, 70, 85, 4);
      myGLCD.printNumI(caldat.hfref2r, 130, 85, 4);
    }
    else if (mode == "VHF") {
      // read VHF table
      myGLCD.printNumF(caldat.vhfdbm1, 1, 0, 75, '.', 6, ' ');
      myGLCD.printNumI(caldat.vhfref1f, 70, 75, 4);
      myGLCD.printNumI(caldat.vhfref1r, 130, 75, 4);
      myGLCD.printNumF(caldat.vhfdbm2, 1, 0, 85, '.', 6, ' ');
      myGLCD.printNumI(caldat.vhfref2f, 70, 85, 4);
      myGLCD.printNumI(caldat.vhfref2r, 130, 85, 4);
    }
    if (calcursor) {
      myGLCD.print("                   ", 165, 75);
      myGLCD.print("< select F>T or R>T", 165, 85);
    }
    else {
      myGLCD.print("                   ", 165, 85);
      myGLCD.print("< select F>T or R>T", 165, 75);
    }
  } // end if caltype 0 or 2
  else if ((caltype == 1) || (caltype == 3)) {
    EEPROM.get(eeAddressTable, caltab);
    if (mode == "HF") {
      myGLCD.print("EEPROM calibration table 1 (HF)", 0, 45);
    }
    else if (mode == "VHF") {
      myGLCD.print("EEPROM calibration table 2 (VHF)", 0, 45);
    }
    myGLCD.print("  dBm   fwd_mV  rev_mV", 0, 55);
    for (int x = 0; x <= arraysize - 1 ; x++) {
      if (mode == "HF") {
        // read HF table
        myGLCD.printNumF(caltab.powtab1[x], 1, 0, 65 + (10 * x), '.', 6, ' ');
        myGLCD.printNumI(caltab.fwdtab1[x], 70, 65 + (10 * x), 4);
        myGLCD.printNumI(caltab.revtab1[x], 130, 65 + (10 * x), 4);
      }
      else if (mode == "VHF") {
        // read VHF table
        myGLCD.printNumF(caltab.powtab2[x], 1, 0, 65 + (10 * x), '.', 6, ' ');
        myGLCD.printNumI(caltab.fwdtab2[x], 70, 65 + (10 * x), 4);
        myGLCD.printNumI(caltab.revtab2[x], 130, 65 + (10 * x), 4);
      }
      if (calcursor == x) {
        myGLCD.print("< select F>T or R>T", 165, 65 + (10 * x));
      }
      else {
        myGLCD.print("                   ", 165, 65 + (10 * x));
      }
    }
  } // end if caltype 1 or 3
} // end drawCalButtons
*/
//**********************************************************************
void cursorcontrol(String ctrl)  // control cursor up/down in cal mode
//**********************************************************************
{
  if ((caltype == 0) || (caltype == 2)) {
    asize = 2;
  } else if ((caltype == 1) || (caltype == 3)) {
    asize = arraysize;
  }
  if (ctrl == "down") {
    if (calcursor == asize - 1) {
      calcursor = 0;
    }
    else {
      calcursor++;
    }
  }
  if (ctrl == "up") {
    if (calcursor == 0) {
      calcursor = asize - 1;
    }
    else {
      calcursor--;
    }
  }
} // end cursorcontrol

//***********************************************************************************
void storeData(String stype)  // store data to EEPROM depending on calibration type
//***********************************************************************************
{
  sprint("cursor", calcursor);
  if ((caltype == 0) || (caltype == 2)) {
    if (calcursor == 0) {         // first point (high power)
      if (mode == "HF") {
        if (stype == "fwd") {
          if (debug) {
            sprint("pow1", caldat.hfdbm1); sprint("fwd_v1: old", caldat.hfref1f); sprintln("new", fwd_v1a);
          }
          caldat.hfref1f = int(round(fwd_v1a));
        } else if (stype == "rev") {
          if (debug) {
            sprint("pow1", caldat.hfdbm1); sprint("rev_v1: old", caldat.hfref1r); sprintln("new", rev_v1a);
          }
          caldat.hfref1r = int(round(rev_v1a));
        } // end rev

      } // end mode HF
      else if (mode == "VHF") {
        if (stype == "fwd") {
          if (debug) {
            sprint("pow1", caldat.vhfdbm1); sprint("fwd_v2: old", caldat.vhfref1f); sprintln("new", fwd_v2a);
          }
          caldat.vhfref1f = int(round(fwd_v2a));
        } else if (stype == "rev") {
          if (debug) {
            sprint("pow1", caldat.vhfdbm1); sprint("rev_v2: old", caldat.vhfref1r); sprintln("new", rev_v2a);
          }
          caldat.vhfref1r = int(round(rev_v2a));
        } // end rev
      } // end mode VHF
    } // end calcursor 0
    else if (calcursor == 1) {  // second point (low power)
      if (mode == "HF") {
        if (stype == "fwd") {
          if (debug) {
            sprint("pow2", caldat.hfdbm2); sprint("fwd_v2: old", caldat.hfref2f); sprintln("new", fwd_v1a);
          }
          caldat.hfref2f = int(round(fwd_v1a));
        } else if (stype == "rev") {
          if (debug) {
            sprint("pow2", caldat.hfdbm2); sprint("rev_v2: old", caldat.hfref2r); sprintln("new", rev_v1a);
          }
          caldat.hfref2r = int(round(rev_v1a));
        } // end rev
      } // end mode HF
      else if (mode == "VHF") {
        if (stype == "fwd") {
          if (debug) {
            sprint("pow2", caldat.vhfdbm2); sprint("fwd_v2: old", caldat.vhfref2f); sprintln("new", fwd_v2a);
          }
          caldat.vhfref2f = int(round(fwd_v2a));
        } else if (stype == "rev") {
          if (debug) {
            sprint("pow2", caldat.vhfdbm2); sprint("rev_v2: old", caldat.vhfref2r); sprintln("new", rev_v2a);
          }
          caldat.vhfref2r = int(round(rev_v2a));
        } // end rev
      } // end mode VHF
    } // end calcursor 1
    EEPROM.put(eeAddress, caldat); // finally write 2-point data to eeprom
  } // end caltype 0 or 2
  else if ((caltype == 1) || (caltype == 3)) {
    if (mode == "HF") {
      if (stype == "fwd") {
        if (debug) {
          sprint("powtab1", caltab.powtab1[calcursor]); sprint("fwd_v1: old", caltab.fwdtab1[calcursor]); sprintln("new", fwd_v1a);
        }
        caltab.fwdtab1[calcursor] = int(round(fwd_v1a));
      } else if (stype == "rev") {
        if (debug) {
          sprint("powtab1", caltab.powtab1[calcursor]); sprint("rev_v1: old", caltab.revtab1[calcursor]); sprintln("new", rev_v1a);
        }
        caltab.revtab1[calcursor] = int(round(rev_v1a));
      }
    }
    else if (mode == "VHF") {
      if (stype == "fwd") {
        if (debug) {
          sprint("powtab2", caltab.powtab2[calcursor]); sprint("fwd_v2: old", caltab.fwdtab2[calcursor]); sprintln("new", fwd_v2a);
        }
        caltab.fwdtab2[calcursor] = int(round(fwd_v2a));
      } else if (stype == "rev") {
        if (debug) {
          sprint("powtab2", caltab.powtab2[calcursor]); sprint("rev_v2: old", caltab.revtab2[calcursor]); sprintln("new", rev_v2a);
        }
        caltab.revtab2[calcursor] = int(round(rev_v2a));
      }
    }
    EEPROM.put(eeAddressTable, caltab); // finally write table to eeprom
  } // end caltype 1 or 3
} // end store data

//**************************************************************************
void serverState()  // check status of webserver and if a client is online
//**************************************************************************
{
  if (screen) {
    contime = millis() - starttime;
    //myGLCD.setFont(SmallFont);
    //myGLCD.setBackColor(0, 0, 0);
    //myGLCD.setColor(255, 255, 255);
    if (webserver) {
      if  ((webcon) && (contime < timeout)) {
      //  myGLCD.print("Web_online ", 230, 0);
      }
      else {
      //  myGLCD.print("Web_offline", 230, 0);
      }
    }
  } // if screen
} // if serverState

//**********************************************************
//void presetScreen()  // print header and buttons on screen
//**********************************************************
/*
{
  if (screen) {
    myGLCD.clrScr();
    myGLCD.setFont(SmallFont);
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.setColor(255, 255, 255);
    myGLCD.print(header1, LEFT, 0);
    myGLCD.print(String(ver), 122, 0);
    myGLCD.print(header2, 160, 0);
    serverState();
    drawButtons();
    if (cal) {
      drawCalButtons();
    }
  }
} // presetScreen
*/
//*****************************************************************************
//void checkScreenState()  // check if display is switched off via panel switch
//*****************************************************************************
/*
{
  serverState();
  if (screen != digitalRead(lcdPin)) // only if display switch was used
  {
    if (screen) {
      screen = 0;
      myGLCD.clrScr();
    }
    else             {
      screen = 1;
      myGLCD.clrScr();
      myTouch.InitTouch();
      myTouch.setPrecision(PREC_MEDIUM);
      presetScreen();
    }
  }
} // checkScreenState
*/
//********************************************************************************************
//void waitForIt(int x1, int y1, int x2, int y2)  // draw a red frame while a button is touched
//********************************************************************************************
/*
{
  myGLCD.setColor(255, 0, 0);
  myGLCD.drawRect (x1, y1, x2, y2);
  while (myTouch.dataAvailable())
    myTouch.read();
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRect (x1, y1, x2, y2);
} // end waitForIt
*/
//*******************************************************************
void updateValues()  // get new PWR and SWR values and display them
//*******************************************************************
{
  if (avgnum < avg) {
    if (cal == 0) {
      if (mode == "HF") // channel 1 (HF) selected
      {
        fwd_v1[avgnum] = mvperstep * (bitmask & analogRead(fwd1Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += fwd_v1[k];
        }
        fwd_v1a = int(round((sum + (wgt * fwd_v1a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          fwd_db1 = interpolate(fwd_v1a, arraysize, caltab.powtab1, caltab.fwdtab1);
        }
        if ((caltype == 0) || (caltype == 2)) {
          fwd_db1 = (fwd_v1a - hfdb0f) / hfmvdbf;
        }
        fwd_pow1 = pow(10, fwd_db1 / 10) / 1000;
        fwd_eff1 = sqrt(fwd_pow1 * 50);
        rev_v1[avgnum] = mvperstep * (bitmask & analogRead(rev1Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += rev_v1[k];
        }
        rev_v1a = int(round((sum + (wgt * rev_v1a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          rev_db1 = interpolate(rev_v1a, arraysize, caltab.powtab1, caltab.revtab1);
        }
        else if ((caltype == 0) || (caltype == 2)) {
          rev_db1 = (rev_v1a - hfdb0r) / hfmvdbr;
        }
        rev_pow1 = pow(10, rev_db1 / 10) / 1000;
        rev_eff1 = sqrt(rev_pow1 * 50);
        swr1 = (fwd_eff1 + rev_eff1) / (fwd_eff1 - rev_eff1);
        if ((fwd_db1 < dBmlowlim) || (swr1 < 1)) {
          swr1 = 0; // only show bar above dBmlowlim
        }
      } // end if mode HF
      else if (mode == "VHF") // channel 2 (VHF) selected
      {
        fwd_v2[avgnum] = mvperstep * (bitmask & analogRead(fwd2Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += fwd_v2[k];
        }
        fwd_v2a = int(round((sum + (wgt * fwd_v2a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          fwd_db2 = interpolate(fwd_v2a, arraysize, caltab.powtab2, caltab.fwdtab2);
        }
        else if ((caltype == 0) || (caltype == 2)) {
          fwd_db2 = (fwd_v2a - vhfdb0f) / vhfmvdbf;
        }
        fwd_pow2 = pow(10, fwd_db2 / 10) / 1000;
        fwd_eff2 = sqrt(fwd_pow2 * 50);
        rev_v2[avgnum] = mvperstep * (bitmask & analogRead(rev2Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += rev_v2[k];
        }
        rev_v2a = int(round((sum + (wgt * rev_v2a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          rev_db2 = interpolate(rev_v2a, arraysize, caltab.powtab2, caltab.revtab2);
        }
        else if ((caltype == 0) || (caltype == 2)) {
          rev_db2 = (rev_v2a - vhfdb0r) / vhfmvdbr;
        }
        rev_pow2 = pow(10, rev_db2 / 10) / 1000;
        rev_eff2 = sqrt(rev_pow2 * 50);
        swr2 = (fwd_eff2 + rev_eff2) / (fwd_eff2 - rev_eff2);
        if ((fwd_db2 < dBmlowlim) || (swr2 < 1)) {
          swr2 = 0; // only show bar above dBmlowlim
        }
      } // end if mode VHF
    } // end if (cal == 0)
    else if (cal == 1) {
      if (mode == "HF") {
        fwd_v1[avgnum] = mvperstep * (bitmask & analogRead(fwd1Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += fwd_v1[k];
        }
        fwd_v1a = int(round((sum + (wgt * fwd_v1a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          fwd_db1 = interpolate(fwd_v1a, arraysize, caltab.powtab1, caltab.fwdtab1);
        }
        if ((caltype == 0) || (caltype == 2)) {
          fwd_db1 = (fwd_v1a - hfdb0f) / hfmvdbf;
        }
        rev_v1[avgnum] = mvperstep * (bitmask & analogRead(rev1Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += rev_v1[k];
        }
        rev_v1a = int(round((sum + (wgt * rev_v1a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          rev_db1 = interpolate(rev_v1a, arraysize, caltab.powtab1, caltab.revtab1);
        }
        else if ((caltype == 0) || (caltype == 2)) {
          rev_db1 = (rev_v1a - hfdb0r) / hfmvdbr;
        }
      }
      else if (mode == "VHF") {
        fwd_v2[avgnum] = mvperstep * (bitmask & analogRead(fwd2Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += fwd_v2[k];
        }
        fwd_v2a = int(round((sum + (wgt * fwd_v2a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          fwd_db2 = interpolate(fwd_v2a, arraysize, caltab.powtab2, caltab.fwdtab2);
        }
        else if ((caltype == 0) || (caltype == 2)) {
          fwd_db2 = (fwd_v2a - vhfdb0f) / vhfmvdbf;
        }
        rev_v2[avgnum] = mvperstep * (bitmask & analogRead(rev2Pin));
        sum = 0;
        for (int k = 0 ; k < avg ; k++) {
          sum += rev_v2[k];
        }
        rev_v2a = int(round((sum + (wgt * rev_v2a)) / (avg + wgt)));
        if ((caltype == 1) || (caltype == 3)) {
          rev_db2 = interpolate(rev_v2a, arraysize, caltab.powtab2, caltab.revtab2);
        }
        else if ((caltype == 0) || (caltype == 2)) {
          rev_db2 = (rev_v2a - vhfdb0r) / vhfmvdbr;
        }
      } // end if mode VHF
    } // end if cal = 1
/*
    if (screen) {              // print screen only if display is switched on
      myGLCD.setBackColor(0, 0, 0);
      if (cal == 0) {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(0, 255, 0);
        myGLCD.print("FWD\0", LEFT, 40);
        myGLCD.print("REV\0", LEFT, 90);
        myGLCD.print("SWR\0", LEFT, 140);
        if (mode == "HF") // channel 1 (HF) selected
        {
          if (fwd_db1 < 0) {
            myGLCD.printNumF(fwd_db1, 1, 57, 40);
          }
          else {
            myGLCD.print("+", 57, 40);
            myGLCD.printNumF(fwd_db1, 1, 73, 40);
          }
          myGLCD.print("dBm\0", 140, 40);
          if (fwd_pow1 < 10) {
            myGLCD.printNumF(fwd_pow1, 2, 220, 40);
          } else {
            myGLCD.printNumF(fwd_pow1, 1, 220, 40);
          }
          myGLCD.print("W\0", 300, 40);
          if (rev_db1 < 0) {
            myGLCD.printNumF(rev_db1, 1, 57, 90);
          }
          else {
            myGLCD.print("+", 57, 90);
            myGLCD.printNumF(rev_db1, 1, 73, 90);
          }
          myGLCD.print("dBm\0", 140, 90);
          if (rev_pow1 < 10) {
            myGLCD.printNumF(rev_pow1, 2, 220, 90);
          } else {
            myGLCD.printNumF(rev_pow1, 1, 220, 90);
          }
          myGLCD.print("W\0", 300, 90);
          if ((swr1 >= 1) && (swr1 <= 3)) {
            myGLCD.printNumF(swr1, 1, 73, 140);
          }
          else {
            myGLCD.print(" --- \0", 54, 140);
          }
          drawBar(140, 155, 315, 140, swr1);
        } // end if mode HF
        else if (mode == "VHF") // channel 2 (VHF) selected
        {
          if (fwd_db2 < 0) {
            myGLCD.printNumF(fwd_db2, 1, 57, 40);
          }
          else {
            myGLCD.print("+", 57, 40);
            myGLCD.printNumF(fwd_db2, 1, 73, 40);
          }
          myGLCD.print("dBm\0", 140, 40);
          if (fwd_pow2 < 10) {
            myGLCD.printNumF(fwd_pow2, 2, 220, 40);
          } else {
            myGLCD.printNumF(fwd_pow2, 1, 220, 40);
          }
          myGLCD.print("W\0", 300, 40);
          if (rev_db2 < 0) {
            myGLCD.printNumF(rev_db2, 1, 57, 90);
          }
          else {
            myGLCD.print("+", 57, 90);
            myGLCD.printNumF(rev_db2, 1, 73, 90);
          }
          myGLCD.print("dBm\0", 140, 90);
          if (rev_pow2 < 10) {
            myGLCD.printNumF(rev_pow2, 2, 220, 90);
          } else {
            myGLCD.printNumF(rev_pow2, 1, 220, 90);
          }
          myGLCD.print("W\0", 300, 90);
          if ((swr2 >= 1) && (swr2 <= 3)) {
            myGLCD.printNumF(swr2, 1, 73, 140);
          }
          else {
            myGLCD.print(" --- \0", 54, 140);
          }
          drawBar(140, 155, 315, 140, swr2);
        } // end if mode VHF
      } // end if (cal == 0)
      else if (cal == 1) {
        myGLCD.setFont(SmallFont);
        myGLCD.setColor(0, 255, 0);
        myGLCD.print("FWD\0", LEFT, 20);
        myGLCD.print("REV\0", LEFT, 30);
        if (mode == "HF") {
          if (fwd_db1 < 0) {
            myGLCD.printNumF(fwd_db1, 1, 40, 20);
          }
          else {
            myGLCD.print("+", 40, 20);
            myGLCD.printNumF(fwd_db1, 1, 48, 20);
          }
          myGLCD.print("dBm\0", 80, 20);
          myGLCD.printNumI(int(fwd_v1a), 120, 20, 4);
          myGLCD.print("mV\0", 160, 20);
          if (rev_db1 < 0) {
          myGLCD.printNumF(rev_db1, 1, 40, 30);
          }
          else {
            myGLCD.print("+", 40, 30);
            myGLCD.printNumF(rev_db1, 1, 48, 30);
          }
          myGLCD.print("dBm\0", 80, 30);
          myGLCD.printNumI(int(rev_v1a), 120, 30, 4);
          myGLCD.print("mV\0", 160, 30);
        }
        else if (mode == "VHF") {
          if (fwd_db2 < 0) {
          myGLCD.printNumF(fwd_db2, 1, 40, 20);
          }
          else {
            myGLCD.print("+", 40, 20);
            myGLCD.printNumF(fwd_db2, 1, 48, 20);
          }
          myGLCD.print("dBm\0", 80, 20);
          myGLCD.printNumI(int(fwd_v2a), 120, 20, 4);
          myGLCD.print("mV\0", 160, 20);
          if (rev_db2 < 0) {
          myGLCD.printNumF(rev_db2, 1, 40, 30);
          }
          else {
            myGLCD.print("+", 40, 30);
            myGLCD.printNumF(rev_db2, 1, 48, 30);
          }
          myGLCD.print("dBm\0", 80, 30);
          myGLCD.printNumI(int(rev_v2a), 120, 30, 4);
          myGLCD.print("mV\0", 160, 30);
        } // end if mode VHF
      } // end if cal = 1
    } // end if screen
*/
    avgnum++;
    if (avgnum == avg) {
      avgnum = 0;
    }
  } // end if avgnum
} // end updateValues()

//******************************************************************
//void drawBar(int xmin, int ymin, int xmax, int ymax, float valueF)
//******************************************************************
/*draw swr bar on screen with green/yellow/red areas
  draw only if value changes to avoid flickering
  xmin,xmax,ymin,ymax = outer extention of bar
  valueF = measured value to be shown
*/
/*
{
  float lim1 = 2.0;  // first limit to change colour
  float lim2 = 2.5;  // second limit to change colour
  float lim3 = 3.0;  // maximum of bar
  int xlim1 = xmin + (xmax - xmin) * 0.5;  // swr = 2.0 -> 50 %
  int xlim2 = xmin + (xmax - xmin) * 0.75; // swr = 2.5 -> 75 %
  if (valueF == 0) {
    drawBarBox(xmin, ymin, xmax, ymax);
    return;
  }
  if (oldval != valueF)
  {
    oldval = valueF;
    drawBarBox(xmin, ymin, xmax, ymax);
    if (valueF <= lim1)
    {
      x = xmin + int(round((xlim1 - xmin) * (valueF - 1)));
      myGLCD.setColor(0, 255, 0);
      myGLCD.fillRect(xmin, ymin, x, ymax);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(x + 1, ymin, xmax, ymax);
    }
    if ((valueF > lim1) && (valueF <= lim2))
    {
      x = xlim1 + int(round((xlim2 - xlim1) * (valueF - lim1)));
      myGLCD.setColor(0, 255, 0);
      myGLCD.fillRect(xmin, ymin, xlim1, ymax);
      myGLCD.setColor(255, 255, 0);
      myGLCD.fillRect(xlim1, ymin, x, ymax);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(x + 1, ymin, xmax, ymax);
    }
    if ((valueF >= lim2) && (valueF <= lim3))
    {
      x = xlim2 + int(round((xmax - xlim2) * (valueF - lim2)));
      myGLCD.setColor(0, 255, 0);
      myGLCD.fillRect(xmin, ymin, xlim1, ymax);
      myGLCD.setColor(255, 255, 0);
      myGLCD.fillRect(xlim1, ymin, xlim2, ymax);
      myGLCD.setColor(255, 0, 0);
      myGLCD.fillRect(xlim2, ymin, x, ymax);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(x + 1, ymin, xmax, ymax);
    }
    if (valueF >= lim3)
    {
      myGLCD.setColor(0, 255, 0);
      myGLCD.fillRect(xmin, ymin, xlim1, ymax);
      myGLCD.setColor(255, 255, 0);
      myGLCD.fillRect(xlim1, ymin, xlim2, ymax);
      myGLCD.setColor(255, 0, 0);
      myGLCD.fillRect(xlim2, ymin, xmax, ymax);
    }
  } // end if oldval != valueF
} // end drawbar
*/
//******************************************************
//void drawBarBox(int xmin, int ymin, int xmax, int ymax)
//******************************************************
// draw box and legend for bar diagram
/*
{
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(xmin, ymin, xmax, ymax);
  myGLCD.setColor(200, 200, 200);
  myGLCD.drawRect(xmin - 1, ymin + 1, xmax + 1, ymax - 1);
  myGLCD.setFont(SmallFont);
  myGLCD.print("1\0", 140, 160);
  myGLCD.print("1.5\0", 173, 160);
  myGLCD.print("2\0", 223, 160);
  myGLCD.print("2.5\0", 261, 160);
  myGLCD.print("3\0", 312, 160);
} // end drawBox
*/
//************************************************************************
float interpolate(float voltage, int arraysize, float ptab[], int vtab[])
//************************************************************************
// interpolate power levels from cal table
{
  if (voltage >= float(vtab[0])) {
    pval = ptab[0];
  }
  else if (voltage <= float(vtab[arraysize - 1])) {
    pval = ptab[arraysize - 1];
  }
  else {
    for (int i = arraysize - 1; i > 0 ; i--) {
      if ((voltage >= float(vtab[i])) && (voltage <= float(vtab[i - 1]))) {
        dpow = ptab[i - 1] - ptab[i];
        dvolt = float(vtab[i - 1] - vtab[i]);
        pval = ptab[i] + (dpow / dvolt * (voltage - vtab[i]));
        break;
      }
    }
  }
  return pval;
} // end interpolate

//**********************************************************************
void GetAjaxData(EthernetClient cl)  // send status to the web browser
//**********************************************************************
{
  // check buttons
  serverState();
  changed = 0; // clear button state
  // pressing the HF/VHF buttons turns switches between modes
  if ((HTTP_req.indexOf("/sel/hf") >= 0) && (mode == "VHF")) {
    mode = "HF";
    changed = 1;
    if (debug) {
    Serial.println("MODE HF");
    }
  } else if ((HTTP_req.indexOf("/sel/vhf") >= 0) && (mode == "HF")) {
    mode = "VHF";
    changed = 1;
    if (debug) {
    Serial.println("MODE VHF");
    }
  }
  // pressing the POW buttons turns power on or off
  if ((HTTP_req.indexOf("/pow/on") >= 0) && (power == "OFF")) {
    power = "ON";
    digitalWrite(powPin, HIGH);
    changed = 1;
    if (debug) {
    Serial.println("POW on");
    }
  } else if ((HTTP_req.indexOf("/pow/off") >= 0) && (power == "ON")) {
    power = "OFF";
    digitalWrite(powPin, LOW);
    changed = 1;
    if (debug) {
    Serial.println("POW off");
    }
  }
  // pressing the TRX buttons turns the TRX on or off
  if ((HTTP_req.indexOf("/trx/on") >= 0) && (trx == "OFF") && (trxState == 0)) {
    switchTRX(1);
    trx = "ON";
    changed = 1;
    if (debug) {
    Serial.println("TRX on");
    }
  } else if ((HTTP_req.indexOf("/trx/off") >= 0) && (trx == "ON")) {
    switchTRX(0);
    trx = "OFF";
    changed = 1;
    if (debug) {
    Serial.println("TRX off");
    }
  }
  //   Update display, show current modes HF/VHF, POW and TRX
  //   measured fwd/rev values and swr with bar
  cl.println("<style>");
  cl.println("bar { font-family: Helvetica; color: white; background-color: black; text-align: center;}");
  cl.println("h2 { font-family: Helvetica; color: white; background-color: black; text-align: center;}");
  cl.println("h5 { font-family: Helvetica; color: white; background-color: black; text-align: center;}");
  cl.println("</style>");
  cl.println("<pre><h5>SWR_PWR_Meter V" + (String(ver)) + " by DL5HG</h5></pre>");
  // Display buttons
  cl.println("<p><div class=\"btn-group\">");
  if (mode == "HF") {
    cl.println("<a href=\"/sel/hf\"><button class=\"button button2\">HF</button></a>");
  }
  else {
    cl.println("<a href=\"/sel/hf\"><button class=\"button\">HF</button></a>");
  }
  if (mode == "VHF") {
    cl.println("<a href=\"/sel/vhf\"><button class=\"button button2\">VHF</button></a>");
  }
  else {
    cl.println("<a href=\"/sel/vhf\"><button class=\"button\">VHF</button></a>");
  }
  if (power == "OFF") {
    cl.println("<a href=\"/pow/off\"><button class=\"button button2\">POW off</button></a>");
  }
  else {
    cl.println("<a href=\"/pow/off\"><button class=\"button\">POW off</button></a>");
  }
  if (power == "ON") {
    cl.println("<a href=\"/pow/on\"><button class=\"button button2\">POW on</button></a>");
  }
  else {
    cl.println("<a href=\"/pow/on\"><button class=\"button\">POW on</button></a>");
  }
  if ((trx == "OFF") && (digitalRead(expPin) == 0)) {
    cl.println("<a href=\"/trx/off\"><button class=\"button button2\">TRX off</button></a>");
  }
  else if ((trx == "OFF") && (digitalRead(expPin) == 1)) { // trx not switched off yet!!!
    cl.println("<a href=\"/trx/off\"><button class=\"button button3\">TRX off</button></a>");
  }
  else {
    cl.println("<a href=\"/trx/off\"><button class=\"button\">TRX off</button></a>");
  }
  if ((trx == "ON") && (digitalRead(expPin) == 1)) {
    cl.println("<a href=\"/trx/on\"><button class=\"button button2\">TRX on</button></a>");
  }
  else if ((trx == "ON") && (digitalRead(expPin) == 0)) { // trx not switched on yet!!!
    cl.println("<a href=\"/trx/on\"><button class=\"button button3\">TRX on</button></a>");
  }
  else if ((trx == "OFF") && (digitalRead(expPin) == 1)) { // was already switched on!!!
    cl.println("<a href=\"/trx/on\"><button class=\"button button2\">TRX on</button></a>");
  }
  else {
    cl.println("<a href=\"/trx/on\"><button class=\"button\">TRX on</button></a>");
  }
  cl.println("</p>"); // end of <div class=\"btn-group\">

  // Display FWD and REV power depending on selected mode
  if (mode == "HF") {
    cl.println("<pre><h2><p>");
    if (fwd_db1 >= 0) {
      cl.println("FWD:   +" + (String(fwd_db1, 1)) + " dBm  /   " + (String(fwd_pow1)) + " W");
    }
    else {
      cl.println("FWD:   " + (String(fwd_db1, 1)) + " dBm  /   " + (String(fwd_pow1)) + " W");
    }
    cl.println("");
    if (rev_db1 >= 0) {
      cl.println("REV:   +" + (String(rev_db1, 1)) + " dBm  /   " + (String(rev_pow1)) + " W");
    }
    else {
      cl.println("REV:   " + (String(rev_db1, 1)) + " dBm  /   " + (String(rev_pow1)) + " W");
    }
    cl.println("");
    if ((swr1 >= 1) && (swr1 <= 3)) {
      cl.println("SWR:   " + (String(swr1, 1)) + "</p></h2></pre>");
    } else {
      cl.println("SWR:   ---</p></h2></pre>");
    }
  }
  else if (mode == "VHF") {
    cl.println("<pre><h2><p>");
    if (fwd_db2 >= 0) {
      cl.println("FWD:   +" + (String(fwd_db2, 1)) + " dBm  /   " + (String(fwd_pow2)) + " W");
    }
    else {
      cl.println("FWD:   " + (String(fwd_db2, 1)) + " dBm  /   " + (String(fwd_pow2)) + " W");
    }
    cl.println("");
    if (rev_db2 >= 0) {
      cl.println("REV:   +" + (String(rev_db2, 1)) + " dBm  /   " + (String(rev_pow2)) + " W");
    }
    else {
      cl.println("REV:   " + (String(rev_db2, 1)) + " dBm  /   " + (String(rev_pow2)) + " W");
    }
    cl.println("");
    if ((swr2 >= 1) && (swr2 <= 3)) {
      cl.println("SWR:   " + (String(swr2, 1)) + "</p></h2></pre>");
    } else {
      cl.println("SWR:   ---</p></h2></pre>");
    }
  }

  // draw bar diagram
  /* bar dimensions: x = 0...335 pixels, y = 0...25 pixels
     bar minimum =  0 (swr = 1)
     bar maximum = 100 (swr = 3 and higher)
     bar color for swr 1.0 to 1.9 : green
     bar color for swr 2.0 to 2.4 : yellow
     bar color for swr 2.5 to 3.0 : red
  */
  if (mode == "HF") {
    swr = swr1;
  }
  else if (mode == "VHF") {
    swr = swr2;
  }
  // change swr float number 1.0 to 3.0 into integer string from 1 to 100 for meter usage.
  // For swr greater than 3 and negative values show the red bar
  if (swr >= 3.0) {
    intswr = "100";
  }
  else if ((swr >= 0) && (swr < 1.0)) {
    intswr = "0";
  }
  else {
    intswr = String(int(round((swr - 1) * 50)));
  }
  cl.println("<style> meter { width: 345px; height: 25px; } </style>");
  cl.println("<meter value=\"" + intswr + "\" max=\"100\" min=\"0\" optimum= \"25\" low=\"50\" high=\"75\" ></meter>");
  // display swr scale
  cl.println("<pre><bar>1.0             1.5              2.0             2.5             3.0</bar></pre>");
  //cl.println(" ");
  //cl.println("<br>" + HTTP_req + "</br>");
  // update screen if active
  if (screen && changed) {
    //presetScreen();
    changed = 0;
  }
} // end GetAjaxData()

//***********************************************************************
void sprint(String text, float value)  // print info via serial monitor
//***********************************************************************
{
  Serial.print(text);
  Serial.print(" = ");
  Serial.print(value);
  Serial.print("\t");
}
//***********************************************************************************
void sprintln(String text, float value)  // print info via serial monitor with <cr>
//***********************************************************************************
{
  Serial.print(text);
  Serial.print(" = ");
  Serial.println(value);
}
