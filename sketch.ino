/*****************************************************************
  Smart MediBox v2 – ESP32 + OLED + MQTT
  ---------------------------------------------------------------
  
*****************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHTesp.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>

/* --------- Hardware map ------------------------------------- */
#define OLED_W        128
#define OLED_H        64
#define OLED_RST      -1
#define OLED_ADDR     0x3C

#define BUZZER_PIN    5
#define STATUS_LED    15
#define BTN_CANCEL    34
#define BTN_OK        32
#define BTN_UP        33
#define BTN_DOWN      35
#define PIN_DHT       12
#define PIN_LDR       36
#define PIN_SERVO     23

/* --------- Wi-Fi & MQTT ------------------------------------- */
const char *wifiSSID = "Wokwi-GUEST";
const char *wifiPWD  = "";
WiFiClient         netClient;
PubSubClient       mqttCli(netClient);

/* --------- Devices ------------------------------------------ */
Adafruit_SSD1306  oled(OLED_W, OLED_H, &Wire, OLED_RST);
DHTesp            dht22;
Servo             servoDoor;

/* --------- Time & alarm ------------------------------------- */
#define NTP_SERVER   "pool.ntp.org"
#define TZ_OFFSET_S  19800        // default UTC+5 :30 (LK)
#define TZ_DST_S     0

int   dayNow, hrNow, minNow, secNow;
int   tzOffsetSeconds = TZ_OFFSET_S;
bool  alarmOn = true;

int   alarmH[2]        = {0, 0};
int   alarmM[2]        = {0, 0};
bool  alarmDone[2]     = {false, false};
int   alarmSlots       = 0;

/* --------- Lighting / servo control ------------------------- */
unsigned long sampleIntMs =  5000;   // LDR sample every 5 s
unsigned long publishIntMs = 120000; // MQTT publish every 2 min
unsigned long lastSampleMs = 0, lastPublishMs = 0;

float     lightSum = 0;
uint16_t  lightCnt = 0;

float minAngle    = 30.0;   // °   fully closed offset
float gainFactor  = 0.75;   // servo gain
float targetTemp  = 30.0;   // °C  reference

/* --------- Melody ------------------------------------------- */
const int  noteCount = 8;
const int  melody[noteCount] = {262, 294, 330, 349, 392, 440, 494, 523};

/* --------- Menu --------------------------------------------- */
int   menuPos = 0;
const int menuMax = 5;
const char *menuTxt[menuMax] = {
  "1- Time-Zone",
  "2- Alarm 1",
  "3- Alarm 2",
  "4- View Alms",
  "5- Del Alms"
};


void drawText(const String&, int, int, uint8_t);
void renderClock();
void refreshTime();
void refreshTimeAndAlarms();

int  waitButton();
void openMenu();
void execMenu(int);

void setTimeZone();
void configAlarm(int);
void showAlarms();
void removeAlarm();

void handleAlarm(int);
void monitorEnvironment();

void mqttCallback(char*, byte*, unsigned int);
void ensureMQTT();

/* ------------------------------------------------------------ */
/*                           SET-UP                             */
/* ------------------------------------------------------------ */
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BTN_CANCEL, INPUT);
  pinMode(BTN_OK, INPUT);
  pinMode(BTN_UP, INPUT);
  pinMode(BTN_DOWN, INPUT);

  Serial.begin(115200);

  /*   Sensors & Actuators   */
  dht22.setup(PIN_DHT, DHTesp::DHT22);
  servoDoor.attach(PIN_SERVO, 500, 2400);
  analogReadResolution(12);

  /*   OLED   */
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed"); while (true) {}
  }
  oled.display(); delay(400);

  /*   Wi-Fi   */
  drawText("Wi-Fi join...", 0, 0, 2);
  WiFi.begin(wifiSSID, wifiPWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
  drawText("Wi-Fi OK", 0, 20, 2);
  delay(700); oled.clearDisplay();

  /*   NTP   */
  configTime(tzOffsetSeconds, TZ_DST_S, NTP_SERVER);

  /*   MQTT   */
  mqttCli.setServer("broker.hivemq.com", 1883);
  mqttCli.setCallback(mqttCallback);

  /*   Welcome   */
  drawText("Smart MediBox", 5, 20, 2);
  delay(1200);
  oled.clearDisplay();
}

/* ------------------------------------------------------------ */
/*                            LOOP                              */
/* ------------------------------------------------------------ */
void loop() {
  unsigned long now = millis();

  if (!mqttCli.connected()) ensureMQTT();
  mqttCli.loop();

  /* --- LDR sampling ---------------------------------------- */
  if (now - lastSampleMs >= sampleIntMs) {
    lastSampleMs = now;
    float photo = (4095.0 - analogRead(PIN_LDR)) / 4095.0;
    lightSum += photo;
    lightCnt++;
  }

  /* --- Publish & servo move -------------------------------- */
  if (now - lastPublishMs >= publishIntMs && lightCnt > 0) {
    lastPublishMs = now;
    float avgLight = lightSum / lightCnt;

    mqttCli.publish("MEDIBOX/LIGHT", String(avgLight, 2).c_str(), true);

    TempAndHumidity t = dht22.getTempAndHumidity();
    float tempC = targetTemp;
    if (dht22.getStatus() == DHTesp::ERROR_NONE) {
      tempC = t.temperature;
      mqttCli.publish("MEDIBOX/TEMP", String(tempC, 1).c_str(), true);
    }

    /* Servo angle */
    float ts  = sampleIntMs  / 1000.0;
    float tu  = publishIntMs / 1000.0;
    float k   = (tu > ts) ? log(tu / ts) : 1.0;
    float ang = minAngle + (180.0 - minAngle) * avgLight * gainFactor * k * (tempC / targetTemp);
    ang = constrain(ang, 0, 180);
    servoDoor.write((int)ang);

    /* Tiny dashboard */
    oled.clearDisplay();
    drawText("L=" + String(avgLight, 2), 0, 0, 1);
    drawText("T=" + String(tempC, 1) + "C", 0, 16, 1);
    drawText("A=" + String(ang, 1) + "d", 0, 32, 1);

    lightSum = 0; lightCnt = 0;
  }

  refreshTimeAndAlarms();
  if (digitalRead(BTN_OK) == LOW) { delay(200); openMenu(); }
  monitorEnvironment();
}

/* ------------------------------------------------------------ */
/*                    MQTT & NETWORK HELPERS                    */
/* ------------------------------------------------------------ */
void ensureMQTT() {
  while (!mqttCli.connected()) {
    oled.clearDisplay();
    drawText("MQTT...", 0, 0, 2);
    if (mqttCli.connect("MEDIBOX-ESP32")) {
      drawText("MQTT OK", 0, 20, 2);
      mqttCli.subscribe("MEDIBOX/CFG/TS");
      mqttCli.subscribe("MEDIBOX/CFG/TU");
      mqttCli.subscribe("MEDIBOX/CFG/ANGLE");
      mqttCli.subscribe("MEDIBOX/CFG/GAIN");
      mqttCli.subscribe("MEDIBOX/CFG/TREF");
    } else {
      int8_t st = mqttCli.state();
      drawText("Err " + String(st), 0, 40, 1);
      delay(3000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int len) {
  char buf[16];
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, payload, len);
  buf[len] = '\0';

  if      (!strcmp(topic, "MEDIBOX/CFG/TS"))   sampleIntMs   = constrain(strtoul(buf,0,10)*1000UL, 1000UL, 60000UL);
  else if (!strcmp(topic, "MEDIBOX/CFG/TU"))   publishIntMs  = constrain(strtoul(buf,0,10)*1000UL, 30000UL, 600000UL);
  else if (!strcmp(topic, "MEDIBOX/CFG/ANGLE")) minAngle     = constrain(atof(buf), 0, 120);
  else if (!strcmp(topic, "MEDIBOX/CFG/GAIN"))  gainFactor   = constrain(atof(buf), 0, 1);
  else if (!strcmp(topic, "MEDIBOX/CFG/TREF"))  targetTemp   = constrain(atof(buf), 10, 40);
}

/* ------------------------------------------------------------ */
/*                 TIME, CLOCK & ALARM ROUTINES                 */
/* ------------------------------------------------------------ */
void refreshTime() {
  struct tm t;
  getLocalTime(&t);
  char b[3];
  strftime(b,3,"%d",&t); dayNow  = atoi(b);
  strftime(b,3,"%H",&t); hrNow   = atoi(b);
  strftime(b,3,"%M",&t); minNow  = atoi(b);
  strftime(b,3,"%S",&t); secNow  = atoi(b);
}

void renderClock() {
  oled.clearDisplay();
  drawText(String(dayNow), 0, 0, 2);
  drawText(":", 22, 0, 2);
  drawText(String(hrNow), 32, 0, 2);
  drawText(":", 54, 0, 2);
  drawText((minNow<10?"0": "")+String(minNow), 64, 0, 2);
  drawText(":", 96, 0, 2);
  drawText((secNow<10?"0": "")+String(secNow), 106, 0, 2);
}

void refreshTimeAndAlarms() {
  refreshTime();
  renderClock();

  if (hrNow==0 && minNow==0 && secNow==0)
    for(int i=0;i<alarmSlots;i++) alarmDone[i]=false;

  if (alarmOn) {
    for (int i=0;i<alarmSlots;i++) {
      if (!alarmDone[i] && alarmH[i]==hrNow && alarmM[i]==minNow) {
        alarmDone[i]=true;
        handleAlarm(i);
      }
    }
  }
}

/* ------------------------------------------------------------ */
/*                     DISPLAY & INPUT HELPERS                  */
/* ------------------------------------------------------------ */
void drawText(const String &txt, int x, int y, uint8_t sz) {
  oled.setTextSize(sz);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(x,y);
  oled.print(txt);
  oled.display();
}

int waitButton() {
  while (true) {
    if (digitalRead(BTN_UP)==LOW)    { delay(200); return BTN_UP; }
    if (digitalRead(BTN_DOWN)==LOW)  { delay(200); return BTN_DOWN; }
    if (digitalRead(BTN_OK)==LOW)    { delay(200); return BTN_OK; }
    if (digitalRead(BTN_CANCEL)==LOW){ delay(200); return BTN_CANCEL; }
    refreshTime();
  }
}

/* ------------------------------------------------------------ */
/*                          MENU SYSTEM                         */
/* ------------------------------------------------------------ */
void openMenu() {
  while (digitalRead(BTN_CANCEL)==HIGH) {
    oled.clearDisplay();
    drawText(menuTxt[menuPos], 0, 0, 2);

    int key = waitButton();
    if      (key==BTN_UP)    { menuPos = (menuPos+1)%menuMax; }
    else if (key==BTN_DOWN)  { menuPos = (menuPos==0?menuMax:menuPos)-1; }
    else if (key==BTN_OK)    { execMenu(menuPos); }
    else if (key==BTN_CANCEL){ break; }
  }
}

void execMenu(int m) {
  if      (m==0) setTimeZone();
  else if (m==1 || m==2) configAlarm(m-1);
  else if (m==3) showAlarms();
  else if (m==4) removeAlarm();
}

/* ----------- Menu actions ----------------------------------- */
void setTimeZone() {
  int tzH = TZ_OFFSET_S/3600, tzM = (TZ_OFFSET_S%3600)/60;

  while (true) {
    oled.clearDisplay();
    drawText("UTC Hr: "+String(tzH), 0, 0, 2);
    int k = waitButton();
    if      (k==BTN_UP)    tzH++;
    else if (k==BTN_DOWN)  tzH--;
    else if (k==BTN_OK)    break;
    else if (k==BTN_CANCEL) return;
  }
  while (true) {
    oled.clearDisplay();
    drawText("UTC Min: "+String(tzM), 0, 0, 2);
    int k = waitButton();
    if      (k==BTN_UP||k==BTN_DOWN) tzM = (tzM==30?0:30);
    else if (k==BTN_OK)    { tzOffsetSeconds = tzH*3600 + tzM*60; configTime(tzOffsetSeconds,TZ_DST_S,NTP_SERVER); break; }
    else if (k==BTN_CANCEL) return;
  }
  drawText("Time-zone set!", 0, 20, 2); delay(1000);
}

void configAlarm(int idx) {
  int h = alarmH[idx], m = alarmM[idx];

  /* hour */
  while (true) {
    oled.clearDisplay();
    drawText("Hour: "+String(h), 0, 0, 2);
    int k = waitButton();
    if      (k==BTN_UP)    h = (h+1)%24;
    else if (k==BTN_DOWN)  h = (h==0?23:h-1);
    else if (k==BTN_OK)    { alarmH[idx]=h; break; }
    else if (k==BTN_CANCEL)return;
  }
  /* minute */
  while (true) {
    oled.clearDisplay();
    drawText("Min: "+String(m), 0, 0, 2);
    int k = waitButton();
    if      (k==BTN_UP)    m = (m+1)%60;
    else if (k==BTN_DOWN)  m = (m==0?59:m-1);
    else if (k==BTN_OK)    { alarmM[idx]=m; break; }
    else if (k==BTN_CANCEL)return;
  }
  if (idx>=alarmSlots) alarmSlots=idx+1;
  drawText("Alarm saved!", 0, 20, 2); delay(1000);
}

void showAlarms() {
  if (!alarmSlots) {
    drawText("No alarms",0,0,2); while (digitalRead(BTN_CANCEL)==HIGH) delay(80); return;
  }
  while (digitalRead(BTN_CANCEL)==HIGH) {
    oled.clearDisplay();
    for (int i=0;i<alarmSlots;i++) {
      String txt=String(alarmH[i])+":"+(alarmM[i]<10?"0":"")+String(alarmM[i]);
      drawText("A"+String(i+1)+" "+txt, 0, i*16, 1);
    }
  }
}

void removeAlarm() {
  if (!alarmSlots) { drawText("No alarms",0,0,2); while(digitalRead(BTN_CANCEL)==HIGH) delay(80); return; }

  for (int i=0;i<alarmSlots;i++) {
    oled.clearDisplay();
    drawText("Del A"+String(i+1)+"?", 0, 0, 2);
    String txt=String(alarmH[i])+":"+(alarmM[i]<10?"0":"")+String(alarmM[i]);
    drawText(txt, 0, 20, 2);

    int k = waitButton();
    if (k==BTN_OK) {
      alarmH[i]=alarmM[i]=-1; alarmDone[i]=false;
      drawText("Deleted",0,40,2); delay(800);
      alarmSlots--;
    } else if (k==BTN_CANCEL) break;
  }
}

/* ------------------------------------------------------------ */
/*                       ALARM HANDLING                         */
/* ------------------------------------------------------------ */
void handleAlarm(int idx) {
  oled.clearDisplay();
  drawText("Medicine!", 10, 0, 2);
  drawText("Snooze?",  15, 40, 2);
  digitalWrite(STATUS_LED, HIGH);

  bool snooze = false;
  while (!snooze && digitalRead(BTN_CANCEL)==HIGH) {
    for (int n=0;n<noteCount;n++) {
      if (digitalRead(BTN_CANCEL)==LOW) { digitalWrite(STATUS_LED,LOW); return; }
      if (digitalRead(BTN_OK)==LOW)    { snooze=true; break; }
      tone(BUZZER_PIN, melody[n]); delay(500); noTone(BUZZER_PIN); delay(5);
    }
  }
  digitalWrite(STATUS_LED,LOW);

  if (snooze) {
    int m = (minNow+5)%60;
    int h = (minNow+5>=60) ? (hrNow+1)%24 : hrNow;
    alarmH[idx]=h; alarmM[idx]=m; alarmDone[idx]=false;
    oled.clearDisplay();
    drawText("Snoozed to",0,0,2);
    drawText(String(h)+":"+(m<10?"0":"")+String(m),0,24,2);
    delay(1200);
  }
}

/* ------------------------------------------------------------ */
/*                      ENVIRONMENT CHECK                       */
/* ------------------------------------------------------------ */
void monitorEnvironment() {
  TempAndHumidity d = dht22.getTempAndHumidity();

  oled.clearDisplay();
  drawText("T "+String(d.temperature,1)+"C", 0, 0, 1);
  drawText("H "+String(d.humidity,0)+"%",  0,16, 1);

  bool alert=false;
  if (d.temperature<24)      { drawText("Temp Low", 0, 32, 1); alert=true; }
  else if (d.temperature>32) { drawText("Temp High",0, 32, 1); alert=true; }

  if (d.humidity<65)         { drawText("Hum Low", 70,32, 1); alert=true; }
  else if (d.humidity>80)    { drawText("Hum High",70,32, 1); alert=true; }

  if (alert) {
    digitalWrite(STATUS_LED,HIGH); tone(BUZZER_PIN,1000); delay(100); noTone(BUZZER_PIN);
  } else {
    digitalWrite(STATUS_LED,LOW);
  }
  delay(800);
}
