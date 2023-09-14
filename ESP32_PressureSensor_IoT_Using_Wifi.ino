//Create By Tommy Budimulyo
//Research and Development
//08-9-2023

#define DEVICE_ID "1" //device ID For database
#define WIFI_SSID "*change your SSID*" //SSID Wifi
#define WIFI_PASS "*change your wifi Password*" //Password Wifi
#define ESP_HOSTNAME "*ESP32 HOSTNAME*" DEVICE_ID

#define SEND_API_INTERVAL 1000UL
#define WATCHDONG_TIMEOUT_SECOND 60
#define HOST_PROTOCOL "http"
#define HOST_API "*enter your website host API*"
#define HOST_API_PORT 80
#define HOST_API_PATH "/api/device/?id=" DEVICE_ID
#define NTP_SERVER "pool.ntp.org"
#define NTP_GMTOFFSET 25200
#define NTP_DAYLIGHTOFFSET 3600
#define WIFI_CONNECT_TIMEOUT 10000UL

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_task_wdt.h>
#include "time.h"
#include <Preferences.h>

//permanently save key value
Preferences preferences;  

// NTP Setting
struct tm timeinfo;
ulong ts_send = 0;
uint8_t second_last_send;

unsigned long previousMillis = 0;
unsigned long interval = 30000;

WiFiClient client;
HTTPClient http;

// Measured value
float oxygen, medical, vacum, insair, carbon, nitrous;
bool oxygen_manifold_empty, nitrous_oxide_manifold_empty, carbon_dioxide_manifold_empty;

int CurrentPage = 0;

//Declare Analog Pin Sensor
//const uint8_t potensio1 = 32;
//const uint8_t potensio2 = 33;
const uint8_t potensio3 = 35;
//const uint8_t potensio4 = 34;
const uint8_t potensio5 = 39;
const uint8_t potensio6 = 36;

// pin Digital (manifold)
const uint8_t manoxy = 25;
const uint8_t mannox = 26;
const uint8_t mancox = 27;

// buzzer
const uint8_t buzz = 14;

// regresi linear
float a_oxy = -1.026522574;
float b_oxy = 0.004894374;

float a_nox = -0.821451148;
float b_nox = 0.005012397;

float a_air = -1.020204082;
float b_air = 0.004889796;

float a_vac = -35.37692308;
float b_vac = 0.073406593;

float a_ins = -0.8618125;
float b_ins = 0.00490625;

float a_cox = -0.841269716;
float b_cox = 0.004865931;

//Variable for ESP32
float oxylow, noxlow, airlow, vaclow, insvlow, coxvlow;
float oxyhigh, noxhigh, airhigh, inshigh, coxhigh;

//variable for API
float oxyvlowapi, noxvlowapi, airvlowapi, vaclowapi, insvlowapi, coxvlowapi;
float oxyhighapi, noxhighapi, airhighapi, inshighapi, coxhighapi;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //create a namespace for preferences.h
  preferences.begin("oxylow", false);
  preferences.begin("oxyhigh", false);
  
  preferences.begin("noxlow", false);
  preferences.begin("noxhigh", false);

  preferences.begin("vaclow", false);
  
  preferences.begin("airlow", false);
  preferences.begin("airhigh", false);

  preferences.begin("inslow", false);
  preferences.begin("inshigh", false);

  preferences.begin("coxlow", false);
  preferences.begin("coxhigh", false);

  //Wifi Hostname & running
  WiFi.setHostname(ESP_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  //Resolution Analog Read in 12bit
  analogReadResolution(12);

  //  pinMode(potensio1, INPUT);
  //  pinMode(potensio2, INPUT);
  pinMode(potensio5, INPUT);
  //  pinMode(potensio4, INPUT);
  pinMode(potensio3, INPUT);
  pinMode(potensio6, INPUT);
  pinMode(manoxy, INPUT);
  pinMode(mannox, INPUT);
  pinMode(mancox, INPUT);
  pinMode(buzz, OUTPUT);

  //========================Starting Program======================
  digitalWrite(buzz, HIGH);

  oxygenlow();
  nitrouslow();
  medicalairlow();
  vacuumlow();
  instrumentairlow();
  carbonlow();
  delay(750);

  oxygennormal();
  nitrousnormal();
  medicalairnormal();
  vacuumnormal();
  instrumentairnormal();
  carbonnormal();

  manoxynormal();
  mannoxnormal();
  mancoxnormal();
  delay(750);

  oxygenhigh();
  nitroushigh();
  medicalairhigh();
  instrumentairhigh();
  carbonhigh();

  manoxyempty();
  mannoxempty();
  mancoxempty();
  delay(750);

  // Init and get the time
  configTime(NTP_GMTOFFSET, NTP_DAYLIGHTOFFSET, NTP_SERVER);
  // printLocalTime();

  http.setReuse(true);

  // Init WatchDog timer
  esp_task_wdt_init(WATCHDONG_TIMEOUT_SECOND, true);
  esp_task_wdt_add(NULL); // Set it for current core, or core 0

  ts_send = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(700);

  float oxylow = preferences.getFloat("oxylow", 0);
  float oxyhigh = preferences.getFloat("oxyhigh", 0);

  float noxlow = preferences.getFloat("noxlow", 0);
  float noxhigh = preferences.getFloat("noxhigh", 0);

  float airlow = preferences.getFloat("airlow", 0);
  float airhigh = preferences.getFloat("airhigh", 0);

  float vaclow = preferences.getFloat("vaclow", 0);

  float inslow = preferences.getFloat("inslow", 0);
  float inshigh = preferences.getFloat("inshigh", 0);

  float coxlow = preferences.getFloat("coxlow", 0);
  float coxhigh = preferences.getFloat("coxhigh", 0);

//=====================Read Sensor & Convert====================
  // Oxygen
  int raw_oxy = analogRead(potensio5);
  oxygen = (a_oxy + (b_oxy * raw_oxy));
  if (oxygen < 0)
  {
    oxygen = 0;
  }

  //Nitrous
  int raw_nox = analogRead(potensio3);
  nitrous = (a_nox + (b_nox * raw_nox));
  if (nitrous < 0)
  {
    nitrous = 0;
  }

  // Medical Air
  int raw_air = analogRead(potensio6);
  medical = (a_air + b_air * raw_air);
  if (medical < 0)
  {
    medical = 0;
  }

  // Vacuum
//  int raw_vac = analogRead(potensio4);
//  vacum = (a_vac + b_vac * raw_vac);
//  if (vacum > 0)
//  {
//    vacum = 0;
//  }  

  //Instrument air
//  int raw_ins = analogRead(potensio5);
//  float insair = (a_ins + (b_ins * raw_ins));
//  if (insair < 0)
//  {
//  insair = 0;
//  }

  //Carbon Dioxide
//  int raw_cox = analogRead(potensio6);
//  float carbon = (a_cox + (b_cox * raw_cox));
//  if (carbon < 0)
//  {
//  carbon = 0;
//  }

//========================== Send to Nextion=====================
  if (CurrentPage == 0)
  { 
    //biar gak ngelag
    String oxx = " ";
    Serial.print("t0.txt=\"");
    Serial.print(oxx);
    Serial.print("\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    
    float oxy = float(oxygen);
    Serial.print("t12.txt=\"");
    Serial.print(oxy, 2);
    Serial.print("\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);

    //nitrous oxide
    float nox = float(nitrous);
    Serial.print("t1.txt=\"");
    Serial.print(nox,2);
    Serial.print("\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);

    // Medical Air
    float air = float(medical);
    Serial.print("t2.txt=\"");
    Serial.print(air, 2);
    Serial.print("\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);

    // Vacuum
    int vac = float(vacum);
    Serial.print("t3.txt=\"");
    Serial.print(vac);
    Serial.print("\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);

    //instrument air
    //  float ins = float(insair);
    //  Serial.print("t4.txt=\"");
    //  Serial.print(ins,2);
    //  Serial.print("\"");
    //  Serial.write(0xff);
    //  Serial.write(0xff);
    //  Serial.write(0xff);

    //carbon dioxide
    //  float cox = float(carbon);
    //  Serial.print("t5.txt=\"");
    //  Serial.print(cox,2);
    //  Serial.print("\"");
    //  Serial.write(0xff);
    //  Serial.write(0xff);
    //  Serial.write(0xff);

//=======================Set Pressure and Manifold=======================
//=======================Pressure High, Normal, Very Low=================
    // Oxygen
    if (oxygen < oxylow)
    {
      // VERY LOW, YELLOW, BUZZ ON
      oxygenlow();
    }

    else if (oxygen < oxyhigh && oxygen >= oxylow)
    {
      // NORM, GREEN
      oxygennormal();
    }
    else if (oxygen >= oxyhigh)
    {
      // HIGH, RED, BUZZ ON
      oxygenhigh();
    }

    // Nitrous
    if (nitrous < noxlow){
      // VERY LOW, YELLOW, BUZZ ON
      nitrouslow();
    }
    
    else if (nitrous < noxhigh && nitrous >= noxlow){
      // NORM, GREEN
      nitrousnormal();
    }
    else if (nitrous >= noxhigh){
    // HIGH, RED, BUZZ ON
    nitroushigh();
    }

    // Medical Air
    if (medical < airlow)
    {
      // VERY LOW, YELLOW, BUZZ ON
      medicalairlow();
    }

    else if (medical < airhigh && medical >= airlow)
    {
      // NORM, GREEN
      medicalairnormal();
    }
    else if (medical >= airhigh)
    {
      // HIGH, RED, BUZZ ON
      medicalairhigh();
    }

    // vacuum
    if (vacum > vaclow)
    {
      // VERY LOW, YELLOW, BUZZ ON
      vacuumlow();
    }
    else if (vacum < vaclow)
    {
      vacuumnormal();
    }

    // instrument air
    // if (insair < insvlow){
    //   // VERY LOW, YELLOW BLINK, BUZZ ON
    //   instrumentairlow();
    // }
    // else if (insair < inshigh && insair >= inslow){
    //   // NORM, GREEN
    //   instrumentairnormal();
    // }
    // else if (insair >= inshigh){
    //   // HIGH, RED
    //   instrumentairhigh();
    // }

    // Carbon Dioxide
    // if (carbon < coxvlow){
    //   // VERY LOW, YELLOW BLINK, BUZZ ON
    //   carbonlow();
    // }
    // else if (carbon < coxhigh && carbon >= coxlow){
    //   // NORM, GREEN
    //   carbonnormal();
    // }
    // else if (carbon >= coxhigh){
    //   // HIGH, RED
    //   carbonhigh();
    // }

//============================Manifold===============================
    //Oxygen
    int mano2 = digitalRead(manoxy);
    if (mano2 == HIGH) {
      manoxynormal();
    }
    else {
      manoxyempty();
    }

    //Nitrous Oxide
    int mann2o = digitalRead(mannox);
    if (mann2o == HIGH) {
      mannoxnormal();
    }
    else {
      mannoxempty();
    }

    //Carbon Dioxide
    // int manco2= digitalRead(mancox);
    // if (manco2 == HIGH) {
    //   mancoxnormal();
    // }
    // else {
    //   mancoxempty();
    // }

//========================NOT USE PRESSURE DISPLAY UI==================
    // oxygennotuse();
    // nitrousnotuse();
    // medicalairnotuse();
    // vacuumnotuse();
    instrumentairnotuse();
    carbonnotuse();

//========================SET BUZZ ON or OFF=========================
    if ((oxygen > oxyhigh) || (medical > airhigh) || (nitrous > noxhigh) 
    || (mann2o == LOW) || (mano2 == LOW))
    {
      digitalWrite(buzz, HIGH);
    }
    
    else if ((oxygen < oxylow) || (medical < airlow) 
    || (nitrous < noxlow) || (vacum > vaclow))
    {
      digitalWrite(buzz, HIGH);
      delay(250);
      digitalWrite(buzz, LOW);
      delay(250);
    }
    
    else 
    {
      digitalWrite(buzz, LOW);
    }
  }

//======================API COMMUNICATION=======================
  if (millis() >= ts_send + SEND_API_INTERVAL)
  {
    ts_send += SEND_API_INTERVAL;
    if (chk_measure_is_under_over_limit())
    {
      send_get_api();
    }
    else
    {
      if (!getLocalTime(&timeinfo))
      {
        Serial.println("[TIME] Failed to obtain time");
        return;
      }
      uint8_t second_now = timeinfo.tm_sec;
      if (second_last_send != second_now && second_now % 10 == 0)
      {
        if (send_get_api())
        {
          second_last_send = second_now;
        }
      }
    }
  } 
  esp_task_wdt_reset();
  unsigned long currentMillis = millis();

  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval))
  {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
    tes();
    bt_nointernet();
    mes_nointernet();
  }
}

//======================SEND & GET API===========================
bool send_get_api()
{
  bool ret = false;
  if (WiFi.status() == WL_CONNECTED)
  {
    String path_req = HOST_API_PATH;
    path_req += "&o=";
    path_req += oxygen;
    path_req += "&n=";
    path_req += nitrous;
    path_req += "&m=";
    path_req += medical;
    path_req += "&i=";
    path_req += insair;
    path_req += "&c=";
    path_req += carbon;
    path_req += "&v=";
    path_req += vacum;
    path_req += "&ome=";
    path_req += oxygen_manifold_empty ? "1" : "0";
    path_req += "&nome=";
    path_req += nitrous_oxide_manifold_empty ? "1" : "0";
    path_req += "&cdme=";
    path_req += carbon_dioxide_manifold_empty ? "1" : "0";

    http.begin(HOST_API, HOST_API_PORT, path_req); // HTTP
    Serial.println();
    Serial.print("[WiFi] ");
    Serial.print(WiFi.SSID());
    Serial.print('\t');
    Serial.println(WiFi.localIP());
    Serial.print("[HTTP] GET ");
    Serial.println(path_req);
    int httpCode = http.GET();

    if (httpCode > 0)
    {
      if (httpCode == HTTP_CODE_OK)
      {
        ret = true;
        String payload = http.getString();
        payload.trim();
        Serial.println(payload);

        if (payload.startsWith("~") && payload.endsWith("#"))
        {
          payload.replace("~", "");
          payload.replace("#", "");

          oxyvlowapi = atof(explode_string(payload, ':', 0).c_str());
          oxyhighapi = atof(explode_string(payload, ':', 1).c_str());

          noxvlowapi = atof(explode_string(payload, ':', 2).c_str());
          noxhighapi = atof(explode_string(payload, ':', 3).c_str());

          airvlowapi = atof(explode_string(payload, ':', 4).c_str());
          airhighapi = atof(explode_string(payload, ':', 5).c_str());

          insvlowapi = atof(explode_string(payload, ':', 6).c_str());
          inshighapi = atof(explode_string(payload, ':', 7).c_str());

          coxvlowapi = atof(explode_string(payload, ':', 8).c_str());
          coxhighapi = atof(explode_string(payload, ':', 9).c_str());

          vaclowapi = atof(explode_string(payload, ':', 10).c_str());
          
          preferences.putFloat("oxylow", oxyvlowapi);
          preferences.putFloat("oxyhigh", oxyhighapi);

          preferences.putFloat("noxlow", noxvlowapi);
          preferences.putFloat("noxhigh", noxhighapi);

          preferences.putFloat("airlow", airvlowapi);
          preferences.putFloat("airhigh", airhighapi);

          preferences.putFloat("vaclow", vaclowapi);

          preferences.putFloat("inxlow", insvlowapi);
          preferences.putFloat("inshigh", inshighapi);

          preferences.putFloat("coxlow", coxvlowapi);
          preferences.putFloat("coxhigh", coxhighapi);

          tes();
          bt_wifiok();
          mes_wifiok();
        }
      }
      else if (httpCode == HTTP_CODE_INTERNAL_SERVER_ERROR)
      {
        Serial.println("[HTTP] Server error");
        tes();
        bt_error500();
        mes_error500();
      }
      else if (httpCode == HTTP_CODE_BAD_REQUEST)
      {
        Serial.println("[HTTP] Send data error");
      }
    }
    else
    {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
    // WiFi.disconnect();
  }
  return ret;
}

//=======================TIMER FOR COUNTER & SERVER====================
void printLocalTime()
{
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

//======================DATA FROM API DEFINITION====================
String explode_string(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//===============LOGIC MEASURE WHEN CONNECT TO INTERNET=============
bool chk_measure_is_under_over_limit()
{
  if (oxygen <= oxyvlowapi)
  {
    return true;
  }
  if (oxygen >= oxyhighapi)
  {
    return true;
  }
  if (nitrous <= noxlow)
  {
    return true;
  }
  if (nitrous >= noxhigh)
  {
    return true;
  }
  if (medical <= airlow)
  {
    return true;
  }
  if (medical >= airhigh)
  {
    return true;
  }
  if (insair <= insvlow)
  {
    return true;
  }
  if (insair >= inshigh)
  {
    return true;
  }
  if (carbon <= coxvlow)
  {
    return true;
  }
  if (carbon >= coxhigh)
  {
    return true;
  }
  if (vacum <= vaclow)
  {
    return true;
  }
  return false;
}

//===================FUNCTION FOR UI==========================
//===============================Erorr Button===============================
void tes() {
      Serial.print("t13.pic=24");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }
    
void bt_wifiok() {
      Serial.print("icon.pic=23");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }
    
void bt_nointernet() {
      Serial.print("icon.pic=22");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }
    
void bt_error500() {
      Serial.print("icon.pic=21");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }

void bt_error400() {
      Serial.print("icon.pic=20");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }

//===============================Error Message==============================
void mes_wifiok() {
      Serial.print("err.pic=19");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }

void mes_nointernet() {
      Serial.print("err.pic=18");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }

void mes_error500() {
      Serial.print("err.pic=17");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }

void mes_error400() {
      Serial.print("err.pic=16");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }

//===============================Not Use Gases==============================
//oxygen NOT USE
void oxygennotuse() {
      Serial.print("t6.bco=46518");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t6.pco=35921");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t6.txt=\"NORMAL\"");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
}

//nitrous oxide NOT USE
void nitrousnotuse() {
      Serial.print("t7.bco=46518");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t7.pco=35921");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t7.txt=\"NORMAL\"");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
}

//Medical Air NOT USE
void medicalairnotuse() {
      Serial.print("t8.bco=46518");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t8.pco=35921");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t8.txt=\"NORMAL\"");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
}

//vacuum NOT USE
void vacuumnotuse() {
      Serial.print("t9.bco=46518");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t9.pco=35921");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t9.txt=\"NORMAL\"");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
}

//instrument air NOT USE
void instrumentairnotuse() {
      Serial.print("t10.bco=46518");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t10.pco=35921");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t10.txt=\"NORMAL\"");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
}

//carbon oxide NOT USE
void carbonnotuse() {
      Serial.print("t11.bco=46518");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t11.pco=35921");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      Serial.print("t11.txt=\"NORMAL\"");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
}

//===============================Low Pressure===================================

// oxygen very low
void oxygenlow()
{
  Serial.print("t6.bco=65504");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t6.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t6.txt=\"VERY LOW\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// nitrous very low
void nitrouslow()
{
  Serial.print("t7.bco=65504");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t7.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t7.txt=\"VERY LOW\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// medical air
void medicalairlow()
{
  Serial.print("t8.bco=65504");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t8.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t8.txt=\"VERY LOW\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// vacuum
void vacuumlow()
{
  Serial.print("t9.bco=65504");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t9.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t9.txt=\"VERY LOW\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// instrument air
void instrumentairlow()
{
  Serial.print("t10.bco=65504");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t10.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t10.txt=\"VERY LOW\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// Carbon dioxide
void carbonlow()
{
  Serial.print("t11.bco=65504");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t11.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t11.txt=\"VERY LOW\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

//=====================================Normal Pressure=======================================

// oxygen Normal
void oxygennormal()
{
  Serial.print("t6.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t6.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t6.txt=\"NORMAL\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// nitrous oxide normal
void nitrousnormal()
{
  Serial.print("t7.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t7.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t7.txt=\"NORMAL\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// medical Air normal
void medicalairnormal()
{
  Serial.print("t8.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t8.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t8.txt=\"NORMAL\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// vacuum normal
void vacuumnormal()
{
  Serial.print("t9.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t9.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t9.txt=\"NORMAL\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// instrument air normal
void instrumentairnormal()
{
  Serial.print("t10.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t10.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t10.txt=\"NORMAL\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// carbon oxide normal
void carbonnormal()
{
  Serial.print("t11.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t11.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t11.txt=\"NORMAL\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

//===============================High Pressure========================

void oxygenhigh()
{
  Serial.print("t6.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t6.pco=65535");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t6.txt=\"HIGH\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// nitrous oxide high
void nitroushigh()
{
  Serial.print("t7.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t7.pco=65535");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t7.txt=\"HIGH\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// medical air high
void medicalairhigh()
{
  Serial.print("t8.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t8.pco=65535");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t8.txt=\"HIGH\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// instrument air high
void instrumentairhigh()
{
  Serial.print("t10.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t10.pco=65535");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t10.txt=\"HIGH\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// carbon dioxide high
void carbonhigh()
{
  Serial.print("t11.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t11.pco=65535");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("t11.txt=\"HIGH\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

//=============================Manifold==============================

// Manifold Oxygen
void manoxynormal()
{
  oxygen_manifold_empty = false;
  Serial.print("man01.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man01.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man02.bco=50744");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man02.pco=27501");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void manoxyempty()
{
  oxygen_manifold_empty = true;
  Serial.print("man01.bco=50744");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man01.pco=27501");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man02.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man02.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// Manifold Nitrous
void mannoxnormal()
{
  nitrous_oxide_manifold_empty = false;
  Serial.print("man11.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man11.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man12.bco=50744");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man12.pco=27501");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  // digitalWrite(buzz, LOW);
}

void mannoxempty()
{
  nitrous_oxide_manifold_empty = true;
  Serial.print("man11.bco=50744");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man11.pco=27501");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man12.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man12.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

// Manifold Carbon
void mancoxnormal()
{
  carbon_dioxide_manifold_empty = false;
  Serial.print("man21.bco=2016");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man21.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man22.bco=50744");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man22.pco=27501");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}
void mancoxempty()
{
  carbon_dioxide_manifold_empty = true;
  Serial.print("man21.bco=50744");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man21.pco=27501");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man22.bco=63488");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("man22.pco=0");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}