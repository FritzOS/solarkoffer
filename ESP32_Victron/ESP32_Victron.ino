/*
* Auslesen von Daten aus dem Solarkoffer und senden über LoRa TTN
*/
#include "Arduino.h"
#include <Wire.h>   
#include "HT_SSD1306Wire.h"
#include <LoRaWan_APP.h>
#include <CayenneLPP.h>

#define SERIAL_P            // Ausgabe von Daten auf die serielle Schnittstelle (Monitor)
//#define TRACE
#define DISPLAY_YES         // display TTN progress on OLED

#define MAXTIMETTNLOOP 180000     // 3 Min: Millisekunden nach denen TTN Loop abgebrochen wird

#define TTNPACE 43200    // 30 Min: Sekunden zwischen TTN Sendungen

uint32_t startloopTTN;      // Wann wurde loop_TTN() gestartet in Millisekunden

RTC_DATA_ATTR bool RTC_wokeUpFromSleep = false;
RTC_DATA_ATTR bool RTC_wokeupafterTTN  = false;

//-------------------------------------------------------------------------------------------------------------------
//--  Display -- Display -- Display -- Display -- Display -- Display -- Display -- Display -- Display -- Display   --
//-------------------------------------------------------------------------------------------------------------------
extern SSD1306Wire display;
//#ifdef Wireless_Stick_V3
//SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED); // addr , freq , i2c group , resolution , rst
//#else
//SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
//#endif

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//-- check battery level -- check battery level -- check battery level -- check battery level -- check battery level 
//-------------------------------------------------------------------------------------------------------------------
#define Read_VBAT_Voltage   1
#define ADC_CTRL           37       // Heltec GPIO to toggle VBatt read connection ... 
                                    // WARNING!!! This also connects VEXT to VCC=3.3v so be careful what is on header.  
                                    // Also, take care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH
#define ADC_READ_STABILIZE 10       // in ms (delay from GPIO control and ADC connections times)

float readBatLevel() {
    #ifdef TRACE
      Serial.println("readBatLevel");
    #endif
    
    pinMode(ADC_CTRL,OUTPUT);
    digitalWrite(ADC_CTRL, LOW);                
    delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
    int analogValue = analogRead(Read_VBAT_Voltage);

    #ifdef TRACE
      Serial.println("BatLevel = " + String(analogValue));
    #endif

    float voltage = 0.00403532794741887 * analogValue;
    #ifdef TRACE
      Serial.println("Voltage = " + String(voltage));
    #endif
    
    return voltage;
}
//-----------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//-- values received from MPPT
//-------------------------------------------------------------------------------------------------------------------
String label, val;

String PID = "";     //Produkt-ID
String FW = "";      //Firmware Version
String SER = "";     //Seriennummer
float V = 99999;     //[V]   Batteriespannung
float I = 99999;     //[mA]  Batterieladestrom
float VPV = 99999;   //[V] Solar-Panel Spannung
float PPV = 99999;   //[W] Solar-Panel Leistung
int CS = 99999;      //Laderegler Betriebsstatus 0=Aus, 2=Fehler, 3=Laden, 4=Entladen, 5=Batt V halten
String MPPT = "";    //MPPT Tracker Status (0=aus, 1= Spannungs- oder Strombegrenzt, 2= aktiv)
String OR = "";      //Off Reason
int ERR = 0;         //Fehler Code 0=Kein Fehler, 2=Batt V zu hoch, 17=Temp zu hoch, 18=Überstrom, 19=Amp umgekehrt, 20=Ladezeitlimit abgelaufen, 21=Amp Sensor Fehler, 26=Anschlüsse überhitzt, 33=Solar V zu hoch, 34=Solar A zu hoch, 38=Input abgeschaltet, 116=WerkseinstellungenDatenVerloren, 117=Falsche Firmware, 119=Einstellungen falsch
float LOAD = 99999;  //Lastausgang ON/OFF
float IL = 99999;    //[mA] Lastausgang-Strom
int H19 = 99999;     //[kWh] Ertrag über die gesamte Lebensdauer des Gerätes
int H20 = 99999;     //[kWh] Ertrag Heute
int H21 = 99999;     //[W] Max-Solarleistung Heute
int H22 = 99999;     //[kWh] Ertrag Gestern
int H23 = 99999;     //[W] Max-Solarleistung Gestern
int HSDS = 99999;    //[T] Tageszähler (0 bis 364)
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//-- deep sleep -- deep sleep -- deep sleep -- deep sleep -- deep sleep -- deep sleep -- deep sleep -- deep sleep --- 
//-------------------------------------------------------------------------------------------------------------------
void goToSleep(uint64_t sleepSeconds) {
  Radio.Sleep( );

  display.sleep();
  delay(100);
  VextOFF();

                          // Heltec V2       // Heltec V3		    // Heltec Wireless Stick V3
  pinMode(17, INPUT);     //  4 SDA          // 17 OLED SDA		  //  17 OLED SDA
  pinMode( 9,INPUT);      //  5 LORA SCK     //  9 LORA			    //   9 LORA SCK 
  pinMode(12,INPUT);      // 14 RST 1278     // 12 LORA RST	  	//  12 LORA RST
  pinMode(18, INPUT);     // 15 SCL          // 18 OLED SCL		  //  18 OLED SCL
  pinMode(21,INPUT);      // 16 OLED RST     // 21 OLED RST		  //  21 OLED RST
  //pinMode(17,INPUT);    // 17 HS1_DATA5    //					        // 
  pinMode( 8,INPUT);      // 18 LORA NSS     // 8 LORA NSS		  //   8 LORA NSS (auf VDD_3V3)
  pinMode(11,INPUT);      // 19 MISO         // 11 LORA MISO	  //  11 LORA MISO
  pinMode(14,INPUT);      // 26 DIO0       // 12 DIO1 (LORA)	  //  14 DIO1 (LORA
  pinMode(10,INPUT);      // 27 MOSI         // 10 LORA MOSI	  //  10 LORA MOSI

  #ifdef SERIAL_P
    Serial.printf("Deep sleeping for %d seconds\n", sleepSeconds);
  #endif
  esp_sleep_enable_timer_wakeup(sleepSeconds * (uint64_t)1000000);
  #ifdef SERIAL_P
    Serial.flush();
  #endif
  esp_deep_sleep_start();
  #ifdef SERIAL_P
    Serial.println("This will never be printed");
  #endif
}
//-----------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//-- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN -- TTN ----------
//-------------------------------------------------------------------------------------------------------------------

/* OTAA para*/
//                     70    B3    D5    7E    D0    06    34    9A
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x34, 0x9A };
//
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//                     44    DB    C3    4B    B3    92    A2    4A    CB    6A    64    BD    7C    E4    07    C6
uint8_t appKey[] = { 0x44, 0xDB, 0xC3, 0x4B, 0xB3, 0x92, 0xA2, 0x4A, 0xCB, 0x6A, 0x64, 0xBD, 0x7C, 0xE4, 0x07, 0xC6 };

/* ABP para, not used in this case*/
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t devAddr =  ( uint32_t )0x00000000;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000;  // 60000 = 1 minutes

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

//-- prepare payload for TTN uplink -------------------------------------------------------
static void prepareTxFrame( uint8_t port)
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
  CayenneLPP lpp(51);
  lpp.reset();

  int last_rssi = 0;
  int last_snr = 0;

  // void addGPS(uint8_t channel, float latitude, float longitude, int32_t altitude);
  // channel: The channel number to associate with this GPS data.
  // latitude: The latitude value in decimal degrees.
  // longitude: The longitude value in decimal degrees.
  // altitude: The altitude value in centimeters. This parameter represents the height above the sea level.

  lpp.addAnalogInput(1, V);               // [V] Batteriespannung
  lpp.addAnalogInput(2, PPV);             // [W] Solar-Panel Leistung
  lpp.addAnalogInput(3, H21);             // [W] Max-Solarleistung Heute
  lpp.addAnalogInput(4, H22);             // [kWh] Ertrag Gestern
  lpp.addAnalogInput(5, readBatLevel());  // [V] LiPo Level

  appDataSize = lpp.getSize();
  memcpy(appData, lpp.getBuffer(), appDataSize);

  #ifdef TRACE
    Serial.println("appDataSize = " + String(appDataSize));
    Serial.println();
  #endif  
}
// END static void prepareTxFrame( uint8_t port)

void loop_TTN()
{
  #ifdef TRACE_loop
    Serial.println("ttn_connection");    
  #endif
  
  switch( deviceState )
    {
      case DEVICE_STATE_INIT:
      {
        #ifdef TRACE
          Serial.println("DEVICE_STATE_INIT");    
        #endif

        extern bool IsLoRaMacNetworkJoined; // enforce new join to TTN
        IsLoRaMacNetworkJoined = false;

        LoRaWAN.init(loraWanClass,loraWanRegion);
        break;
      }
      case DEVICE_STATE_JOIN:
      {
        #ifdef TRACE
          Serial.println("DEVICE_STATE_JOIN");    
        #endif

        #ifdef DISPLAY_YES
          display.clear();
          display.drawString(0,10,"join-TTN");
          display.display();
        #endif
        
        LoRaWAN.join();

        #ifdef TRACE
          Serial.println("DEVICE_STATE_JOIN end");    
        #endif
        
        break;
      }
      case DEVICE_STATE_SEND:
      {
        #ifdef TRACE
          Serial.println("DEVICE_STATE_SEND");    
        #endif

        #ifdef DISPLAY_YES
          display.clear();
          display.drawString(0,10, "send-TTN");
          display.display();
        #endif

        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
      case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission    
        #ifdef TRACE
          Serial.println("DEVICE_STATE_CYCLE");    
        #endif

        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        
        #ifdef TRACE
          Serial.println("txDutyCycleTime = " + String(txDutyCycleTime));    
        #endif
        
        deviceState = DEVICE_STATE_SLEEP;
        break;          
      }
      case DEVICE_STATE_SLEEP:
      { 
        extern uint8_t ifDisplayAck;
        
        #ifdef TRACE_SLEEP
          Serial.println("DEVICE_STATE_SLEEP, ifDisplayAck = " + String(ifDisplayAck));    
        #endif

        if (ifDisplayAck == 1) // message has been sent out
        {
          #ifdef DISPLAY_YES
            display.clear();
            display.drawString(0,10, "ack-TTN");
            display.display(); 
          #endif

          LoRaWAN.displayAck();
          extern int revrssi, revsnr;

          #ifdef TRACE
            Serial.println("rssi = " + String(revrssi) + ", snr = " + String(revsnr));    
          #endif
        }
        else LoRaWAN.sleep(loraWanClass);

        break;
      }
      default:
      {
        #ifdef TRACE
          Serial.println("default");    
        #endif

        deviceState = DEVICE_STATE_INIT;
        break;
      }
    }
}
// END void loop_TTN()

//-------------------------------------------------------------------------------------------------------------------
//-- Setup -- Setup -- Setup -- Setup -- Setup -- Setup -- Setup -- Setup -- Setup -- Setup -- Setup -- Setup -------
//-------------------------------------------------------------------------------------------------------------------
void setup()  
{
  // initialize data
  PID = "";     //Produkt-ID
  FW = "";      //Firmware Version
  SER = "";     //Seriennummer
  V = 99999;    //[V]   Batteriespannung
  I = 99999;    //[mA]  Batterieladestrom
  VPV = 99999;  //[V] Solar-Panel Spannung
  PPV = 99999;  //[W] Solar-Panel Leistung
  CS = 99999;   //Laderegler Betriebsstatus 0=Aus, 2=Fehler, 3=Laden, 4=Entladen, 5=Batt V halten
  MPPT = "";    //MPPT Tracker Status (0=aus, 1= Spannungs- oder Strombegrenzt, 2= aktiv)
  OR = "";      //Off Reason
  ERR = 0;      //Fehler Code 0=Kein Fehler, 2=Batt V zu hoch, 17=Temp zu hoch, 18=Überstrom, 19=Amp umgekehrt, 20=Ladezeitlimit abgelaufen, 21=Amp Sensor Fehler, 26=Anschlüsse überhitzt, 33=Solar V zu hoch, 34=Solar A zu hoch, 38=Input abgeschaltet, 116=WerkseinstellungenDatenVerloren, 117=Falsche Firmware, 119=Einstellungen falsch
  LOAD = 99999; //Lastausgang ON/OFF
  IL = 99999;   //[mA] Lastausgang-Strom
  H19 = 99999;  //[kWh] Ertrag über die gesamte Lebensdauer des Gerätes
  H20 = 99999;  //[kWh] Ertrag Heute
  H21 = 99999;  //[W] Max-Solarleistung Heute
  H22 = 99999;  //[kWh] Ertrag Gestern
  H23 = 99999;  //[W] Max-Solarleistung Gestern
  HSDS = 99999; //[T] Tageszähler (0 bis 364)

  // initialize OLED
  VextON();
  delay(100);
  display.init();
  display.setFont(ArialMT_Plain_10);

  //serielle Schnittstellen starten
  Serial.begin(115200);

  //
  if (RTC_wokeupafterTTN) 
  {
    #ifdef SERIAL_P
      Serial.println();
      Serial.println("XXX regulaere Schlafzeit starten XXX");
      Serial.println();
    #endif

    RTC_wokeupafterTTN = false;
    goToSleep(TTNPACE);
  }

  // prepare LoRaWAN
  Mcu.begin();
  deviceState = DEVICE_STATE_INIT;

  RTC_wokeUpFromSleep = true;

  // read data from MPPT
  for (int i = 0; i < 2 ; i++) {
    read_MPPT();
	  print_to_Serial();
    #ifdef SERIAL_P
      print_to_Display();
    #endif   
  }

  #ifdef SERIAL_P
    Serial.println();
    Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    Serial.println("XXX     Sending to TTN        XXX");
    Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  #endif
  startloopTTN = millis();
  RTC_wokeupafterTTN = true;
} 
// END void setup()

//-------------------------------------------------------------------------------------------------------------------
//-- loop -- loop -- loop -- loop -- loop -- loop -- loop -- loop -- loop -- loop -- loop -- loop -- loop -- loop ---
//-------------------------------------------------------------------------------------------------------------------
void loop() {
  // Abbruch falls TTN Verbindung nicht erfolgreich
  if((millis() - startloopTTN) > MAXTIMETTNLOOP) 
  {
    #ifdef SERIAL_P
      Serial.println();
      Serial.println("XXX     Abbruch weil keine TTN Verbindung        XXX");
      Serial.println();
    #endif

    goToSleep(TTNPACE);
  }

  loop_TTN();
  
}

//-------------------------------------------------------------------------------------------------------------------
//------ read_MPPT -- read_MPPT -- read_MPPT -- read_MPPT -- read_MPPT -- read_MPPT -- read_MPPT -- read_MPPT -------
//-------------------------------------------------------------------------------------------------------------------
void read_MPPT() {
  Serial2.begin(19200,SERIAL_8N1,GPIO_NUM_47,-1); // Serial2 is not stable, therefore initialize new at each loop

  for( int i = 0; i < 30; i++) {
    if (Serial2.available() > 10) {
      label = Serial2.readStringUntil('\t');      
      val = Serial2.readStringUntil('\r\r\n');
      if(label == "PID") {
        PID = val;
      } else if (label == "FW") {
        FW = val;
      } else if (label == "SER#") {
        SER = val;
      } else if (label == "V") {
        float temp = val.toFloat();
        temp = temp / 1000;
        V = temp;
      } else if (label == "I") {
        I = val.toFloat();
      } else if (label == "VPV") {
        float temp = val.toFloat();
        temp = temp / 1000;
        VPV = temp;
      } else if (label == "PPV") {
        PPV = val.toFloat();
      } else if (label == "CS") {
        CS = val.toInt();
      } else if (label == "MPPT") {
        MPPT = val;
      } else if (label == "OR") {
        OR = val;
      } else if (label == "ERR") {
        ERR = val.toInt();
      } else if (label == "LOAD") {
        LOAD = val.toFloat();
      } else if (label == "IL") {
        IL = val.toFloat();
      } else if (label == "H19") {
        int temp = val.toInt();
        temp = temp * 10;
        H19 = temp;
      } else if (label == "H20") {
        int temp = val.toInt();
        temp = temp * 10;
        H20 = temp;
      } else if (label == "H21") {
        H21 = val.toInt();
      } else if (label == "H22") {
        int temp = val.toInt();
        temp = temp * 10;
        H22 = temp;
      } else if (label == "H23") {
        H23 = val.toInt();
      } else if (label == "HSDS") {
        HSDS = val.toInt();
      }
    }
    delay(300);
  }

}

//-------------------------------------------------------------------------------------------------------------------
//-- print_to_Serial -- print_to_Serial -- print_to_Serial -- print_to_Serial -- print_to_Serial -- rint_to_Serial  -
//-------------------------------------------------------------------------------------------------------------------
#ifdef SERIAL_P
  void print_to_Serial() {
    char tmp[100]; //Buffer für Ausgabe
    Serial.println("");
    Serial.println("");
    Serial.println("==============================");
    Serial.println("Product ID: " + PID);
    Serial.println("Firmware Version: " + FW);
    Serial.println("Seriennummer: " + SER);
    sprintf(tmp,"Batteriespannung: %6.2f V",V);
    Serial.println(tmp);
    sprintf(tmp,"Batterieladestrom: %6.2f mA",I);
    Serial.println(tmp);
      sprintf(tmp,"PV Leistung: %6.2f V",VPV);
    Serial.println(tmp);
    sprintf(tmp,"PV Leistung: %6.2f W",PPV);
    Serial.println(tmp);
    sprintf(tmp,"Charger-Status: %5d  (0=Aus, 2=Fehler, 3=Laden, 4=Entladen, 5=Batt V halten)",CS);
    Serial.println(tmp);
    Serial.println("MPPT Tracker Status: " + MPPT + " (0=aus, 1= Spannungs- oder Strombegrenzt, 2= aktiv)");
    Serial.println("Off Reason: " + OR);
    sprintf(tmp,"Fehler-Status: %4d", ERR);
    Serial.println(tmp);
    if (LOAD == 0) sprintf(tmp, "Lastausgang ist AN");
              else sprintf(tmp, "Lastausgang ist AUS");
    Serial.println(tmp);
    sprintf(tmp,"Strom Lastausgang: %6.2f",IL);
    Serial.println(tmp);
    sprintf(tmp,"Ertrag Heute: %5d kWh",H20);
    Serial.println(tmp);
    sprintf(tmp,"Max-Leistung Heute: %5d W",H21);
    Serial.println(tmp);
    sprintf(tmp,"Ertrag Gestern: %5d kWh",H22);
    Serial.println(tmp);
    sprintf(tmp,"Max-Leistung Gestern: %5d W",H23);
    Serial.println(tmp);
    sprintf(tmp,"Tageszähler: %4d",HSDS);
    Serial.println(tmp);
    sprintf(tmp,"Ertrag gesamt: %5d Wh",H19);
    Serial.println(tmp);
    Serial.println("==============================");
  }
#endif

//-------------------------------------------------------------------------------------------------------------------
//--  print_to_Display  --  print_to_Display  --  print_to_Display  --  print_to_Display  -- print_to_Display  ------
//-------------------------------------------------------------------------------------------------------------------
void print_to_Display() {
  char tmp[32]; //Buffer für Ausgabe
  
  display.clear();
  sprintf(tmp,"Batterie");
  display.drawString(0,0,tmp);
  sprintf(tmp,"    %4.1f V",V);
  display.drawString(0,10,tmp);
  sprintf(tmp,"     %4.0f mA",I);
  display.drawString(0,20,tmp);
  display.display();
  delay(6000);

  display.clear();
  sprintf(tmp,"Solarpanel");
  display.drawString(0,0,tmp);
  sprintf(tmp,"    %4.1f V", VPV);
  display.drawString(0,10,tmp);
  sprintf(tmp,"    %4.1f W ", PPV);
  display.drawString(0,20,tmp);
  display.display();
  delay(6000);
}