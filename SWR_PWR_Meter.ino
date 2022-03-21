/**************************************************
   SWR_PWR_Meter with touch screen and webserver
   Copyright (C) 2021  Helmut Gross, DL5HG
   Copyright (C) 2022  Dirk Juchmann, DK4DJ
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

   Original: contact: dl5hg@darc.de

   Custom version by DK4DJ (dk4dj@darc.de)
   2022-03-21  1.40   first custom version; source stripped down

   This program requires the EEPROM libraries.
 **********************************************************************/

#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h>

/*################################################################
  ##                    initial system setup                    ##
  ##############################################################*/
// --------------------------------------------------------------
//  runtime control
// --------------------------------------------------------------
String ver = "1.40"; // version of program
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

// ------------------------------------------------------------
//  setup I/O pins
// ------------------------------------------------------------
#define fwd1Pin 6  // input fwd voltage channel 1 (HF)
#define rev1Pin 7  // input rev voltage channel 1 (HF)
#define rstPin 16 // output for ethernet shield reset

// ------------------------------------------------------------
//  setup runtime system variables
// ------------------------------------------------------------
#define eeAddress 0           // eeprom address for caltype 0 (2 points)
#define eeAddressTable 100    // eeprom address for caltype 1 (table)
int dBmlowlim = 0;            // lower fwd power limit in dBm for swr bar diagram
int changed   = 0;            // variable set to 1 if button status has changed
int cal              = 0;     // calibration mode active if set to 1
int calcursor        = 0;     // cursor for cal table entity selection
int asize            = 0;     // for cursor control
int sum              = 0;     // sum of averaged integer values
int avgnum           = 0;     // counter for averaging
int bitmask          = 1020;  // mask two least bits of analog input
char header1[17]     = "SWR_PWR_Meter V\0";
char header2[31]     = "by DK4DJ (original by DL5HG) \0";
char calDatHeader[9] = "cal_Dat\0";
char calTabHeader[9] = "cal_Tab\0";
String mode          = "HF";  // mode HF/VHF
String intswr        = "1";   // integer swr value (1..100) made from float swr

// ------------------------------------------------------------
//  setup measurement and calculation variables
// ------------------------------------------------------------
float fwd_v1[avg];      // measured fwd voltage channel 1 in mV
float rev_v1[avg];      // measured rev voltage channel 1 in mV
float fwd_v1a;          // averaged fwd voltage channel 1 in mV
float rev_v1a;          // averaged rev voltage channel 1 in mV
float fwd_db1 = 0;      // calculated fwd power channel 1 in dBm
float rev_db1 = 0;      // calculated rev power channel 1 in dBm
float swr1 = 0;         // calculated swr of channel 1
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
  char calDatHeader[9];
  float hfdbm1;
  float hfdbm2;
  int hfref1f;
  int hfref2f;
  int hfref1r;
  int hfref2r;
};
// Calibration data for 2-point calibration
caldatObj caldat = {
  "cal_Dat\0", // 2-point calibration header
  40,          // hfdbm1 calibration power level high in dBm
  0,           // hfdbm2 calibration power level low in dBm
  1884,        // calibrated fwd voltage at hfdbm1
  996,         // calibrated fwd voltage at hfdbm2
  1830,        // calibrated rev voltage at hfdbm1
  944,         // calibrated rev voltage at hfdbm2
};
caldatObj caldat0 = {"\0", 0, 0, 0, 0, 0, 0}; //, 0, 0, 0, 0, 0, 0}; // dummy for input test
float hfmvdbf  = (caldat.hfref1f - caldat.hfref2f) / (caldat.hfdbm1 - caldat.hfdbm2); // mV/dB forward
float hfmvdbr  = (caldat.hfref1r - caldat.hfref2r) / (caldat.hfdbm1 - caldat.hfdbm2); // mV/dB reverse
float hfdb0f   = caldat.hfref2f - hfmvdbf * caldat.hfdbm2; // mV @ 0 dBm
float hfdb0r   = caldat.hfref2r - hfmvdbr * caldat.hfdbm2; // mV @ 0 dBm
float fwd_pow1, fwd_eff1, rev_pow1, rev_eff1;

// -------------------------------------------------------------------------------------
// calibration table values of DL5HG HF/VHF prototype bidirectional couplers
// these values can only be modified here and loaded into EEPROM via caltype = 3 mode
// (if EEPROM is new, this is done automatically by the program)
// -------------------------------------------------------------------------------------
const int arraysize = 12;  // max size of each calibration table (less than 12 possible)
struct caltabObj {
  char calTabHeader[9];
  float powtab1[arraysize];
  int fwdtab1[arraysize];
  int revtab1[arraysize];
};
// Calibration data for calibration table
caltabObj caltab = {
  "cal_Tab\0", // calibration table header
  {   50,   47,   43,   40,   37,   33,   30,   20,   10,   0, -10, -20},  // HF cal power level in dBm
  { 2086, 2031, 1943, 1884, 1826, 1744, 1666, 1435, 1217, 996, 771, 556},  // HF cal fwd voltage in mV
  { 2042, 1982, 1891, 1830, 1767, 1695, 1618, 1386, 1170, 944, 725, 506}   // HF cal rev voltage in mV
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
  pinMode(rstPin, OUTPUT);
  resetShields();
  Serial.begin(115200);
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
//              client.println(".button { background-color: #AAAAAA; color: white; padding: 2px 7px;}"); // grey
//              client.println(".button2 {background-color: #00BB32;}"); // dark green
//              client.println(".button3 {background-color: #FF0000;}"); // red
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
//              client.println("window.history.back();"); // remove switch trailer from header
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

//**********************************************************************
//void cursorcontrol(String ctrl)  // control cursor up/down in cal mode
//**********************************************************************
/*
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
*/
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
    EEPROM.put(eeAddressTable, caltab); // finally write table to eeprom
  } // end caltype 1 or 3
} // end store data

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
    } // end if cal = 1
    avgnum++;
    if (avgnum == avg) {
      avgnum = 0;
    }
  } // end if avgnum
} // end updateValues()

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
//  serverState();
  changed = 0; // clear button state
  // pressing the HF/VHF buttons turns switches between modes
  if ((HTTP_req.indexOf("/sel/hf") >= 0) && (mode == "VHF")) {
    mode = "HF";
    changed = 1;
    if (debug) {
    Serial.println("MODE HF");
    }
  } 
  //   Update display, show current modes HF/VHF, POW and TRX
  //   measured fwd/rev values and swr with bar
  cl.println("<style>");
  cl.println("bar { font-family: Helvetica; color: white; background-color: black; text-align: center;}");
  cl.println("h2 { font-family: Helvetica; color: white; background-color: black; text-align: center;}");
  cl.println("h5 { font-family: Helvetica; color: white; background-color: black; text-align: center;}");
  cl.println("</style>");
  cl.println("<pre><h5>SWR_PWR_Meter V" + (String(ver)) + " by DK4DJ (original by DL5HG)</h5></pre>");
  // Display buttons
//  cl.println("<p><div class=\"btn-group\">");
  if (mode == "HF") {
    cl.println("<a href=\"/sel/hf\"><button class=\"button button2\">HF</button></a>");
  }
  else {
    cl.println("<a href=\"/sel/hf\"><button class=\"button\">HF</button></a>");
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
