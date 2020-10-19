#define WDTENABLE
#define DCPOWER
//#define LANG_EN
//#define MY_DEBUG
//#define MY_PASSIVE_NODE
//#define MY_NODE_ID 100
#define MY_RESET_REASON_TEXT
#define SN "EFEKTA WeatherStation 290"
#define SV "0.25"
#define MY_RADIO_NRF5_ESB
#define MY_NRF5_ESB_PA_LEVEL (RADIO_TXPOWER_TXPOWER_Pos3dBm)


#include "eink290.h"
#include "einkpaint.h"
#include "einkimgdata.h"
#include <TimeLib.h>

const uint16_t shortWait = 50;
uint16_t minuteT = 60000;
float tempThreshold = 0.33;
float humThreshold = 1.0;
float pressThreshold = 0.2;
#ifdef WDTENABLE
const uint32_t SLEEP_TIME_WDT = 10000;
uint32_t sleepTimeCount;
#endif

bool ckeck_hm;
bool colorPrint;
bool opposite_colorPrint;
bool change;
bool check;
bool chek_h = true;
bool tch;
bool hch;
bool bch;
bool pch;
bool fch;
bool metric;
bool timeReceived;
bool configMode;
bool button_flag;
bool nosleep;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
bool Ack_FP;
bool updateink1;
bool updateink2;
bool updateink3;
bool updateink4;
bool updateinkclear;
const int timeZone = 3;
uint8_t year_one;
uint8_t year_two;
uint8_t year_three;
uint8_t year_four;
uint8_t month_one;
uint8_t month_two;
uint8_t day_one;
uint8_t day_two;
uint8_t minutePrint;
uint8_t minute_one;
uint8_t minute_two;
uint8_t hour_one;
uint8_t hour_two;
uint8_t battery;
uint8_t old_battery;
uint8_t cpNom;
uint8_t cpCount;
uint8_t timeSend;
uint8_t battSend;
uint8_t err_delivery_beat;
uint8_t problem_mode_count;
uint16_t batteryVoltage;
int16_t nRFRSSI;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;

uint16_t BATT_TIME;
uint16_t BATT_COUNT;
uint32_t configMillis;
uint32_t previousMillis;
uint32_t SLEEP_TIME;
float batteryVoltageF;
float temperature;
float pressure;
float humidity;
float old_temperature;
float old_humidity;
float old_pressure;
unsigned char image[6000];
Paint paint(image, 0, 0);
Epd epd;
// ##############################################################################################################
// #                                                 INTERRUPT                                                  #
// ##############################################################################################################
uint32_t PIN_BUTTON_MASK;
volatile byte buttIntStatus = 0;
#define APP_GPIOTE_MAX_USERS 1
extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
static app_gpiote_user_id_t m_gpiote_user_id;
int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)
#include <MySensors.h>
#define TEMP_CHILD_ID 0
#define HUM_CHILD_ID 1
#define BARO_CHILD_ID 2
#define FORECAST_CHILD_ID 3
#define SIGNAL_Q_ID 100
#define BATTERY_VOLTAGE_ID 101
#define SET_TIME_SEND_ID 102
#define SET_BATT_SEND_ID 103
#define MY_SEND_RESET_REASON 105
#define SET_COLOR_ID 106
MyMessage msgTemp(TEMP_CHILD_ID, V_TEMP);
MyMessage msgHum(HUM_CHILD_ID, V_HUM);
MyMessage msgPres(BARO_CHILD_ID, V_PRESSURE);
MyMessage forecastMsg(FORECAST_CHILD_ID, V_VAR1);
MyMessage sqMsg(SIGNAL_Q_ID, V_VAR1);
MyMessage bvMsg(BATTERY_VOLTAGE_ID, V_VAR1);
MyMessage setTimeSendMsg(SET_TIME_SEND_ID, V_VAR1);
MyMessage setBattSendMsg(SET_BATT_SEND_ID, V_VAR1);
MyMessage sendMsg(MY_SEND_RESET_REASON, V_VAR1);
MyMessage setColor(SET_COLOR_ID, V_VAR1);


// ##############################################################################################################
// #                                                 FORECAST                                                   #
// ##############################################################################################################
#define CONVERSION_FACTOR (1.0/10.0)         // used by forecast algorithm to convert from Pa to kPa, by dividing hPa by 10.
const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,                     // "Stable Weather Pattern"
  SUNNY = 1,                      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,                     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,                   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4,               // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5                     // "Unknown (More Time needed)
};
int16_t forecast;
int16_t  old_forecast = -1;            // Stores the previous forecast, so it can be compared with a new forecast.
const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];
int minuteCount = 0;              // Helps the forecst algorithm keep time.
bool firstRound = true;           // Helps the forecast algorithm recognise if the sensor has just been powered up.
float pressureAvg;                // Average value is used in forecast algorithm.
float pressureAvg2;               // Average after 2 hours is used as reference value for the next iteration.
float dP_dt;                      // Pressure delta over time

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;


void colorChange(bool flag) {
  if (flag == true) {
    colorPrint = true;
    opposite_colorPrint = false;
  } else {
    colorPrint = false;
    opposite_colorPrint = true;
  }
  saveState(106, flag);
}


void receiveTime(unsigned long controllerTime) {
  controllerTime = controllerTime + timeZone * SECS_PER_HOUR;
  setTime(controllerTime);
  timeReceived = true;
}


// ##############################################################################################################
// #                                                 E-PAPER DISP                                               #
// ##############################################################################################################

void DrawImageWH(Paint * paint, int x, int y, const unsigned char* imgData, int Width, int Height, int colored)
{
  int i, j;
  const unsigned char* prt = imgData;
  for (j = 0; j < Height; j++) {
    for (i = 0; i < Width; i++) {
      if (pgm_read_byte(prt) & (0x80 >> (i % 8))) {
        paint->DrawPixel(x + i, y + j, colored);
      }
      if (i % 8 == 7) {
        prt++;
      }
    }
    if (Width % 8 != 0) {
      prt++;
    }
  }
}


void displayTemp(float temp, bool metr) {

#ifdef LANG_EN
  DrawImageWH(&paint, 6, 112, TEMPEN, 10, 72, colorPrint);
#else
  DrawImageWH(&paint, 6, 112, TEMP, 10, 72, colorPrint);
#endif

  int temperature_temp = round(temp * 10.0);

  if (metr) {
    if (temperature_temp >= 100) {

      DrawImageWH(&paint, 24, 63, NTC, 16, 20, colorPrint);

      byte one_t = temperature_temp / 100;
      byte two_t = temperature_temp % 100 / 10;
      byte three_t = temperature_temp % 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 22, 174, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 174, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 174, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 174, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 174, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 174, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 174, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 174, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 174, NT9, 66, 40, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 22, 134, NT0, 66, 40, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 22, 134, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 134, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 134, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 134, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 134, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 134, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 134, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 134, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 134, NT9, 66, 40, colorPrint);
          break;
      }

      DrawImageWH(&paint, 77, 123, NTP, 11, 11, colorPrint);

      switch (three_t) {
        case 0:
          DrawImageWH(&paint, 22, 83, NT0, 66, 40, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 22, 83, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 83, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 83, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 83, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 83, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 83, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 83, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 83, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 83, NT9, 66, 40, colorPrint);
          break;
      }
    } else {

      DrawImageWH(&paint, 24, 83, NTC, 16, 20, colorPrint);

      byte one_t = temperature_temp / 10;
      byte two_t = temperature_temp % 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 22, 154, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 154, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 154, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 154, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 154, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 154, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 154, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 154, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 154, NT9, 66, 40, colorPrint);
          break;
      }

      DrawImageWH(&paint, 77, 143, NTP, 11, 11, colorPrint);

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 22, 103, NT0, 66, 40, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 22, 103, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 103, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 103, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 103, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 103, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 103, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 103, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 103, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 103, NT9, 66, 40, colorPrint);
          break;
      }
    }
  } else {
    if (temperature_temp < 1000) {

      DrawImageWH(&paint, 24, 63, NTF, 16, 20, colorPrint);

      byte one_t = temperature_temp / 100;
      byte two_t = temperature_temp % 100 / 10;
      byte three_t = temperature_temp % 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 22, 174, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 174, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 174, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 174, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 174, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 174, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 174, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 174, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 174, NT9, 66, 40, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 22, 134, NT0, 66, 40, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 22, 134, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 134, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 134, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 134, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 134, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 134, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 134, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 134, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 134, NT9, 66, 40, colorPrint);
          break;
      }

      DrawImageWH(&paint, 77, 123, NTP, 11, 11, colorPrint);

      switch (three_t) {
        case 0:
          DrawImageWH(&paint, 22, 83, NT0, 66, 40, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 22, 83, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 83, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 83, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 83, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 83, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 83, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 83, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 83, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 83, NT9, 66, 40, colorPrint);
          break;
      }
    } else {

      DrawImageWH(&paint, 24, 68, NTF, 16, 20, colorPrint);

      byte one_t = temperature_temp / 1000;
      byte two_t = temperature_temp % 1000 / 100;
      byte three_t = temperature_temp % 100 / 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 22, 168, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 168, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 168, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 168, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 168, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 168, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 168, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 168, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 168, NT9, 66, 40, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 22, 128, NT0, 66, 40, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 22, 128, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 128, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 128, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 128, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 128, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 128, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 128, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 128, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 128, NT9, 66, 40, colorPrint);
          break;
      }

      switch (three_t) {
        case 0:
          DrawImageWH(&paint, 22, 88, NT0, 66, 40, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 22, 88, NT1, 66, 40, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 22, 88, NT2, 66, 40, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 22, 88, NT3, 66, 40, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 22, 88, NT4, 66, 40, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 22, 88, NT5, 66, 40, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 22, 88, NT6, 66, 40, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 22, 88, NT7, 66, 40, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 22, 88, NT8, 66, 40, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 22, 88, NT9, 66, 40, colorPrint);
          break;
      }
    }
  }
}



void displayPres(float pres, bool metr) {

#ifdef LANG_EN
  DrawImageWH(&paint, 43, 220, PRESEN, 10, 72, colorPrint);
#else
  DrawImageWH(&paint, 43, 220, PRES, 10, 72, colorPrint);
#endif

  int pressure_temp = round(pres * 10.0);

  byte one_p = pressure_temp / 1000;
  byte two_p = pressure_temp % 1000 / 100;
  byte three_p = pressure_temp % 100 / 10;
  byte four_p = pressure_temp % 10;

  if (metr) {

#ifdef LANG_EN
    DrawImageWH(&paint, 86, 226, MHPRESEN, 10, 61, colorPrint);
#else
    DrawImageWH(&paint, 86, 226, MHPRES, 10, 61, colorPrint);
#endif

    switch (one_p) {
      case 1:
        DrawImageWH(&paint, 59, 275, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59, 275, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59, 275, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59, 275, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59, 275, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59, 275, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59, 275, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59, 275, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59, 275, NPH9, 23, 16, colorPrint);
        break;
    }

    switch (two_p) {
      case 0:
        DrawImageWH(&paint, 59, 259, NPH0, 23, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 59, 259, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59, 259, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59, 259, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59, 259, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59, 259, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59, 259, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59, 259, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59, 259, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59, 259, NPH9, 23, 16, colorPrint);
        break;
    }

    switch (three_p) {
      case 0:
        DrawImageWH(&paint, 59, 243, NPH0, 23, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 59, 243, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59, 243, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59, 243, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59, 243, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59, 243, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59, 243, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59, 243, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59, 243, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59, 243, NPH9, 23, 16, colorPrint);
        break;
    }

    DrawImageWH(&paint, 78, 238, NPHP, 5, 5, colorPrint);

    switch (four_p) {
      case 0:
        DrawImageWH(&paint, 59, 222, NPH0, 23, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 59, 222, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59, 222, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59, 222, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59, 222, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59, 222, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59, 222, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59, 222, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59, 222, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59, 222, NPH9, 23, 16, colorPrint);
        break;
    }
  } else {

#ifdef LANG_EN
    DrawImageWH(&paint, 86, 226, PAPRESEN, 10, 61, colorPrint);
#else
    DrawImageWH(&paint, 86, 226, PAPRES, 10, 61, colorPrint);
#endif

    if (one_p == 1) {

      switch (one_p) {
        case 1:
          DrawImageWH(&paint, 59, 272, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 272, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 272, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 272, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 272, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 272, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 272, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 272, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 272, NPH9, 23, 16, colorPrint);
          break;
      }

      switch (two_p) {
        case 0:
          DrawImageWH(&paint, 59, 256, NPH0, 23, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59, 256, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 256, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 256, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 256, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 256, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 256, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 256, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 256, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 256, NPH9, 23, 16, colorPrint);
          break;
      }

      switch (three_p) {
        case 0:
          DrawImageWH(&paint, 59, 240, NPH0, 23, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59, 240, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 240, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 240, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 240, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 240, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 240, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 240, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 240, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 240, NPH9, 23, 16, colorPrint);
          break;
      }

      switch (four_p) {
        case 0:
          DrawImageWH(&paint, 59, 224, NPH0, 23, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59, 224, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 224, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 224, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 224, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 224, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 224, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 224, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 224, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 224, NPH9, 23, 16, colorPrint);
          break;
      }
    } else {

      switch (one_p) {
        case 1:
          DrawImageWH(&paint, 59, 275, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 275, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 275, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 275, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 275, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 275, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 275, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 275, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 275, NPH9, 23, 16, colorPrint);
          break;
      }

      switch (two_p) {
        case 0:
          DrawImageWH(&paint, 59, 259, NPH0, 23, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59, 259, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 259, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 259, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 259, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 259, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 259, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 259, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 259, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 259, NPH9, 23, 16, colorPrint);
          break;
      }

      switch (three_p) {
        case 0:
          DrawImageWH(&paint, 59, 243, NPH0, 23, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59, 243, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 243, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 243, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 243, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 243, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 243, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 243, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 243, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 243, NPH9, 23, 16, colorPrint);
          break;
      }

      DrawImageWH(&paint, 78, 238, NPHP, 5, 5, colorPrint);

      switch (four_p) {
        case 0:
          DrawImageWH(&paint, 59, 222, NPH0, 23, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59, 222, NPH1, 23, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59, 222, NPH2, 23, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59, 222, NPH3, 23, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59, 222, NPH4, 23, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59, 222, NPH5, 23, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59, 222, NPH6, 23, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59, 222, NPH7, 23, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59, 222, NPH8, 23, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59, 222, NPH9, 23, 16, colorPrint);
          break;
      }
    }
  }
}


void displayHum(float hum) {

#ifdef LANG_EN
  DrawImageWH(&paint, 43, 4, HUMEN, 10, 72, colorPrint);
#else
  DrawImageWH(&paint, 43, 4, HUM, 10, 72, colorPrint);
#endif

  int hum_temp = round(hum);

  byte one_h = hum_temp / 10;
  byte two_h = hum_temp % 10;

  DrawImageWH(&paint, 86, 34, PRHUM, 9, 10, colorPrint);

  switch (one_h) {
    case 0:
      DrawImageWH(&paint, 59, 39, NPH0, 23, 16, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 59, 39, NPH1, 23, 16, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 59, 39, NPH2, 23, 16, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 59, 39, NPH3, 23, 16, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 59, 39, NPH4, 23, 16, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 59, 39, NPH5, 23, 16, colorPrint);
      break;
    case 6:
      DrawImageWH(&paint, 59, 39, NPH6, 23, 16, colorPrint);
      break;
    case 7:
      DrawImageWH(&paint, 59, 39, NPH7, 23, 16, colorPrint);
      break;
    case 8:
      DrawImageWH(&paint, 59, 39, NPH8, 23, 16, colorPrint);
      break;
    case 9:
      DrawImageWH(&paint, 59, 39, NPH6, 23, 16, colorPrint);
      break;
  }

  switch (two_h) {
    case 0:
      DrawImageWH(&paint, 59, 23, NPH0, 23, 16, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 59, 23, NPH1, 23, 16, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 59, 23, NPH2, 23, 16, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 59, 23, NPH3, 23, 16, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 59, 23, NPH4, 23, 16, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 59, 23, NPH5, 23, 16, colorPrint);
      break;
    case 6:
      DrawImageWH(&paint, 59, 23, NPH6, 23, 16, colorPrint);
      break;
    case 7:
      DrawImageWH(&paint, 59, 23, NPH7, 23, 16, colorPrint);
      break;
    case 8:
      DrawImageWH(&paint, 59, 23, NPH8, 23, 16, colorPrint);
      break;
    case 9:
      DrawImageWH(&paint, 59, 23, NPH9, 23, 16, colorPrint);
      break;
  }
}


void displayLink(int8_t s) {

  if (flag_nogateway_mode == true) {
    s = 0;
  }

  if (s < 100) {
    if (s >= 10) {
      int8_t one_s = s / 10;
      int8_t two_s = s % 10;

      switch (one_s) {
        case 1:
          DrawImageWH(&paint, 111, 55, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 111, 55, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 111, 55, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 111, 55, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 111, 55, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 111, 55, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 111, 55, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 111, 55, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 111, 55, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_s) {
        case 0:
          DrawImageWH(&paint, 111, 46, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 111, 46, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 111, 46, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 111, 46, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 111, 46, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 111, 46, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 111, 46, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 111, 46, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 111, 46, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 111, 46, CD9, 16, 9, colorPrint);
          break;
      }
    } else {
      int8_t one_s = s;

      switch (one_s) {
        case 0:
          DrawImageWH(&paint, 111, 46, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 111, 46, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 111, 46, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 111, 46, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 111, 46, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 111, 46, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 111, 46, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 111, 46, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 111, 46, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 111, 46, CD9, 16, 9, colorPrint);
          break;
      }
    }
  } else {
    DrawImageWH(&paint, 111, 66, CD1, 16, 9, colorPrint);
    DrawImageWH(&paint, 111, 55, CD0, 16, 9, colorPrint);
    DrawImageWH(&paint, 111, 46, CD0, 16, 9, colorPrint);
  }

  DrawImageWH(&paint, 111, 37, CDPR, 16, 9, colorPrint);


  if (s == 0) {
    DrawImageWH(&paint, 111, 5, LINK0, 16, 29, colorPrint);
  }
  if (s > 0 && s < 20) {
    DrawImageWH(&paint, 111, 5, LINK20, 16, 29, colorPrint);
  }
  if (s >= 20 && s < 40) {
    DrawImageWH(&paint, 111, 5, LINK40, 16, 29, colorPrint);
  }
  if (s >= 40 && s < 60) {
    DrawImageWH(&paint, 111, 5, LINK60, 16, 29, colorPrint);
  }
  if (s >= 60 && s <= 80) {
    DrawImageWH(&paint, 111, 5, LINK80, 16, 29, colorPrint);
  }
  if (s >= 80 && s <= 100) {
    DrawImageWH(&paint, 111, 5, LINK100, 16, 29, colorPrint);
  }
}


void displayBatt(uint8_t b) {

  if (b < 100) {
    if (b >= 10) {
      uint8_t one_b = b / 10;
      uint8_t two_b = b % 10;

      switch (one_b) {
        case 1:
          DrawImageWH(&paint, 111, 244, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 111, 244, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 111, 244, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 111, 244, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 111, 244, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 111, 244, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 111, 244, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 111, 244, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 111, 244, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_b) {
        case 0:
          DrawImageWH(&paint, 111, 235, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 111, 235, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 111, 235, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 111, 235, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 111, 235, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 111, 235, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 111, 235, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 111, 235, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 111, 235, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 111, 235, CD9, 16, 9, colorPrint);
          break;
      }
    } else {
      uint8_t one_b = b;

      switch (one_b) {
        case 0:
          DrawImageWH(&paint, 111, 235, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 111, 235, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 111, 235, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 111, 235, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 111, 235, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 111, 235, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 111, 235, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 111, 235, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 111, 235, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 111, 235, CD9, 16, 9, colorPrint);
          break;
      }
    }
  } else {
    DrawImageWH(&paint, 111, 235, CD0, 16, 9, colorPrint);
    DrawImageWH(&paint, 111, 244, CD0, 16, 9, colorPrint);
    DrawImageWH(&paint, 111, 253, CD1, 16, 9, colorPrint);
  }

  DrawImageWH(&paint, 111, 226, CDPR, 16, 9, colorPrint);

  if (b < 2) {
    DrawImageWH(&paint, 111, 262, BAT0, 16, 29, colorPrint);
  }
  if (b >= 2 && b < 10) {
    DrawImageWH(&paint, 111, 262, BAT10, 16, 29, colorPrint);
  }
  if (b >= 10 && b < 24) {
    DrawImageWH(&paint, 111, 262, BAT24, 16, 29, colorPrint);
  }
  if (b >= 24 && b < 37) {
    DrawImageWH(&paint, 111, 262, BAT37, 16, 29, colorPrint);
  }
  if (b >= 37 && b <= 50) {
    DrawImageWH(&paint, 111, 262, BAT50, 16, 29, colorPrint);
  }
  if (b >= 50 && b <= 64) {
    DrawImageWH(&paint, 111, 262, BAT64, 16, 29, colorPrint);
  }
  if (b >= 64 && b <= 76) {
    DrawImageWH(&paint, 111, 262, BAT76, 16, 29, colorPrint);
  }
  if (b >= 76 && b <= 88) {
    DrawImageWH(&paint, 111, 262, BAT88, 16, 29, colorPrint);
  }
  if (b >= 88 && b <= 100) {
    DrawImageWH(&paint, 111, 262, BAT100, 16, 29, colorPrint);
  }
}


void displayForecast(uint8_t f) {
#ifdef LANG_EN
  switch (f) {
    case 0:
      DrawImageWH(&paint, 102, 78, F0EN, 24, 140, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 102, 78, F1EN, 24, 140, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 102, 78, F2EN, 24, 140, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 102, 78, F3EN, 24, 140, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 102, 78, F4EN, 24, 140, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 102, 78, F5EN, 24, 140, colorPrint);
      break;
  }
#else
  switch (f) {
    case 0:
      DrawImageWH(&paint, 102, 78, F0, 24, 140, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 102, 78, F1, 24, 140, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 102, 78, F2, 24, 140, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 102, 78, F3, 24, 140, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 102, 78, F4, 24, 140, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 102, 78, F5, 24, 140, colorPrint);
      break;
  }
#endif
}


void displayDMY() {
  uint16_t yearPrint = year();
  if (timeReceived) {

    year_one = yearPrint / 1000;
    year_two = yearPrint % 1000 / 100;
    year_three = yearPrint % 100 / 10;
    year_four = yearPrint % 10;
    switch (year_four) {
      case 0:
        DrawImageWH(&paint, 2, 6, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 6, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 6, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 6, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 6, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 6, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 6, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 6, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 6, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 6, CD9, 16, 9, colorPrint);
        break;
    }

    switch (year_three) {
      case 0:
        DrawImageWH(&paint, 2, 15, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 15, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 15, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 15, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 15, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 15, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 15, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 15, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 15, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 15, CD9, 16, 9, colorPrint);
        break;
    }

    switch (year_two) {
      case 0:
        DrawImageWH(&paint, 2, 24, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 24, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 24, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 24, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 24, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 24, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 24, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 24, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 24, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 24, CD9, 16, 9, colorPrint);
        break;
    }

    switch (year_one) {
      case 0:
        DrawImageWH(&paint, 2, 33, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 33, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 33, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 33, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 33, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 33, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 33, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 33, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 33, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 33, CD9, 16, 9, colorPrint);
        break;
    }
    DrawImageWH(&paint, 2, 42, CDP, 16, 5, colorPrint);


    uint16_t monthPrint = month();
    if (monthPrint > 9) {
      month_one = monthPrint / 10;
      month_two = monthPrint % 10;
    } else {
      month_one = 0;
      month_two = monthPrint;
    }
    switch (month_two) {
      case 0:
        DrawImageWH(&paint, 2, 47, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 47, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 47, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 47, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 47, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 47, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 47, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 47, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 47, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 47, CD9, 16, 9, colorPrint);
        break;
    }

    switch (month_one) {
      case 0:
        DrawImageWH(&paint, 2, 56, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 56, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 56, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 56, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 56, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 56, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 56, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 56, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 56, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 56, CD9, 16, 9, colorPrint);
        break;
    }

    DrawImageWH(&paint, 2, 65, CDP, 16, 5, colorPrint);

    uint16_t dayPrint = day();
    if (dayPrint > 9) {
      day_one = dayPrint / 10;
      day_two = dayPrint % 10;
    } else {
      day_one = 0;
      day_two = dayPrint;
    }

    switch (day_two) {
      case 0:
        DrawImageWH(&paint, 2, 70, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 70, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 70, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 70, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 70, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 70, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 70, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 70, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 70, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 70, CD9, 16, 9, colorPrint);
        break;
    }

    switch (day_one) {
      case 0:
        DrawImageWH(&paint, 2, 79, CD0, 16, 9, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 2, 79, CD1, 16, 9, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 79, CD2, 16, 9, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 79, CD3, 16, 9, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 79, CD4, 16, 9, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 79, CD5, 16, 9, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 79, CD6, 16, 9, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 79, CD7, 16, 9, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 2, 79, CD8, 16, 9, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 2, 79, CD9, 16, 9, colorPrint);
        break;
    }


    DrawImageWH(&paint, 2, 244, CDP2, 16, 5, colorPrint);
    byte weekdayPrint = weekday(); // Sunday is day 1
#ifdef LANG_EN
    switch (weekdayPrint) {
      case 1:
        DrawImageWH(&paint, 2, 214, SUNDAYEN, 16, 30, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 214, MONDAYEN, 16, 30, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 214, TUESDAYEN, 16, 30, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 214, WEDNESDAYEN, 16, 30, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 214, THURSDAYEN, 16, 30, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 214, FRIDAYEN, 16, 30, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 214, SATURDAYEN, 16, 30, colorPrint);
        break;
    }
#else
    switch (weekdayPrint) {
      case 1:
        DrawImageWH(&paint, 2, 214, SUNDAY, 16, 30, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 2, 214, MONDAY, 16, 30, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 2, 214, TUESDAY, 16, 30, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 2, 214, WEDNESDAY, 16, 30, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 2, 214, THURSDAY, 16, 30, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 2, 214, FRIDAY, 16, 30, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 2, 214, SATURDAY, 16, 30, colorPrint);
        break;
    }
#endif
  }
}


void displayHM(bool hm) {
  if (timeReceived) {
    if (hm == false) {
      uint16_t hourPrint = hour();
      if (hourPrint > 9) {
        hour_one = hourPrint / 10;
        hour_two = hourPrint % 10;
      } else {
        hour_one = 0;
        hour_two = hourPrint;
      }
      switch (hour_two) {
        case 0:
          DrawImageWH(&paint, 2, 272, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 272, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 272, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 272, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 272, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 272, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 272, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 272, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 272, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 272, CD9, 16, 9, colorPrint);
          break;
      }

      switch (hour_one) {
        case 0:
          DrawImageWH(&paint, 2, 281, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 281, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 281, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 281, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 281, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 281, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 281, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 281, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 281, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 281, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 2, 267, CD2P, 16, 5, colorPrint);

      uint8_t minutePrint = minute();
      if (minutePrint > 9) {
        minute_one = minutePrint / 10;
        minute_two = minutePrint % 10;
      } else {
        minute_one = 0;
        minute_two = minutePrint;
      }
      switch (minute_two) {
        case 0:
          DrawImageWH(&paint, 2, 249, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 249, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 249, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 249, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 249, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 249, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 249, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 249, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 249, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 249, CD9, 16, 9, colorPrint);
          break;
      }

      switch (minute_one) {
        case 0:
          DrawImageWH(&paint, 2, 258, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 258, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 258, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 258, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 258, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 258, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 258, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 258, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 258, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 258, CD9, 16, 9, colorPrint);
          break;
      }
    } else {
      uint16_t hourPrint = hour();
      if (hourPrint > 9) {
        hour_one = hourPrint / 10;
        hour_two = hourPrint % 10;
      } else {
        hour_one = 0;
        hour_two = hourPrint;
      }
      switch (hour_two) {
        case 0:
          DrawImageWH(&paint, 2, 23, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 23, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 23, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 23, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 23, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 23, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 23, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 23, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 23, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 23, CD9, 16, 9, colorPrint);
          break;
      }

      switch (hour_one) {
        case 0:
          DrawImageWH(&paint, 2, 32, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 32, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 32, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 32, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 32, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 32, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 32, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 32, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 32, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 32, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 2, 18, CD2P, 16, 5, colorPrint);

      uint8_t minutePrint = minute();
      if (minutePrint > 9) {
        minute_one = minutePrint / 10;
        minute_two = minutePrint % 10;
      } else {
        minute_one = 0;
        minute_two = minutePrint;
      }
      switch (minute_two) {
        case 0:
          DrawImageWH(&paint, 2, 0, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 0, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 0, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 0, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 0, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 0, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 0, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 0, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 0, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 0, CD9, 16, 9, colorPrint);
          break;
      }

      switch (minute_one) {
        case 0:
          DrawImageWH(&paint, 2, 9, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 2, 9, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 2, 9, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 2, 9, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 2, 9, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 2, 9, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 2, 9, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 2, 9, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 2, 9, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 2, 9, CD9, 16, 9, colorPrint);
          break;
      }
      ckeck_hm = false;
    }
  }
}


void display_Table()
{
  paint.DrawVerticalLine(110, 0, 74, colorPrint);
  paint.DrawVerticalLine(110, 222, 74, colorPrint);
}


void displayStart() {
  epd.Init(lut_full_update);
  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
  epd.DisplayFrame();
  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
  epd.DisplayFrame();

  epd.Init(lut_partial_update);
  paint.SetRotate(ROTATE_0);
  paint.SetWidth(128);
  paint.SetHeight(296);
  paint.Clear(opposite_colorPrint);
  DrawImageWH(&paint, 8, 80, LOGO, 110, 134, colorPrint);
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  delay(2000);

  paint.Clear(opposite_colorPrint);
#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, CONECTEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, CONECT, 64, 192, colorPrint);
#endif
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  epd.Sleep();
}


void displayUpdate(float t, float h, float p, int16_t f, bool m, bool hm) {
  epd.Init(lut_partial_update);
  if (hm == false) {
    paint.SetWidth(128);
    paint.SetHeight(296);
    if (colorPrint == true) {
      epd.ClearFrameMemory(0x00);
      epd.DisplayFrame();
    } else {
      epd.ClearFrameMemory(0xFF);
      epd.DisplayFrame();
    }
    paint.Clear(opposite_colorPrint);
    displayDMY();
    displayHM(hm);
    displayTemp(t, m);
    displayPres(p, m);
    displayHum(h);
    displayBatt(battery);
    displayLink(nRFRSSI);
    displayForecast(forecast);
    display_Table();

    epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
    epd.DisplayFrame();
    epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
    epd.DisplayFrame();
    epd.Sleep();
  } else {
    paint.SetWidth(24);
    paint.SetHeight(41);
    paint.Clear(opposite_colorPrint);
    displayHM(hm);
    epd.SetFrameMemory(paint.GetImage(), 0, 249, paint.GetWidth(), paint.GetHeight());
    epd.DisplayFrame();
    epd.SetFrameMemory(paint.GetImage(), 0, 249, paint.GetWidth(), paint.GetHeight());
    epd.DisplayFrame();
    epd.Sleep();
  }
}


void reseteinkset() {
  updateink1 = false;
  updateink2 = false;
  updateink3 = false;
  updateink4 = false;
  updateinkclear = false;
}


void clearOne() {
  if (colorPrint == true) {
    epd.ClearFrameMemory(0xFF);
    epd.DisplayFrame();
  } else {
    epd.ClearFrameMemory(0x00);
    epd.DisplayFrame();
  }
}


void einkZeropush() {
  epd.Init(lut_partial_update);

  if (colorPrint == true) {
    epd.ClearFrameMemory(0xFF);
    epd.DisplayFrame();
  } else {
    epd.ClearFrameMemory(0x00);
    epd.DisplayFrame();
  }
  paint.SetWidth(128);
  paint.SetHeight(296);
  paint.Clear(opposite_colorPrint);
  if (flag_nogateway_mode == false) {
#ifdef LANG_EN
    DrawImageWH(&paint, 32, 51, CONFEN, 64, 192, colorPrint);
#else
    DrawImageWH(&paint, 32, 51, CONF, 64, 192, colorPrint);
#endif
  } else {
#ifdef LANG_EN
    DrawImageWH(&paint, 32, 51, FINDEN, 64, 192, colorPrint);
#else
    DrawImageWH(&paint, 32, 51, FIND, 64, 192, colorPrint);
#endif
  }
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkZeroend() {
  paint.Clear(opposite_colorPrint);
  if (flag_nogateway_mode == false) {
#ifdef LANG_EN
    DrawImageWH(&paint, 32, 51, CONFAEN, 64, 192, colorPrint);
#else
    DrawImageWH(&paint, 32, 51, CONFA, 64, 192, colorPrint);
#endif
  } else {
#ifdef LANG_EN
    DrawImageWH(&paint, 32, 51, FINDAEN, 64, 192, colorPrint);
#else
    DrawImageWH(&paint, 32, 51, FINDA, 64, 192, colorPrint);
#endif
  }
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkOnepush() {
  paint.Clear(opposite_colorPrint);

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, PRESENTEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, PRESENT, 64, 192, colorPrint);
#endif

  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkOneend() {
  paint.Clear(opposite_colorPrint);

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, PRESENTAEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, PRESENTA, 64, 192, colorPrint);
#endif

  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkOnePluspush() {
  paint.Clear(opposite_colorPrint);

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, RESETEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, RESET, 64, 192, colorPrint);
#endif

  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkOnePlusend() {
  paint.Clear(opposite_colorPrint);

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, RESETAEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, RESETA, 64, 192, colorPrint);
#endif

  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkTwopush() {

}


void einkTwoend() {

}


void reportTimeInk() {
  paint.Clear(opposite_colorPrint);

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, NEWSENSINTEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, NEWSENSINT, 64, 192, colorPrint);
#endif

  if (timeSend >= 10) {
    byte one_t = timeSend / 10;
    byte two_t = timeSend % 10;
    switch (one_t) {
      case 1:
        DrawImageWH(&paint, 96, 148, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 96, 148, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 96, 148, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 96, 148, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 96, 148, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 96, 148, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 96, 148, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 96, 148, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 96, 148, NPH9, 23, 16, colorPrint);
        break;
    }

    switch (two_t) {
      case 0:
        DrawImageWH(&paint, 96, 132, NPH0, 23, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 96, 132, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 96, 132, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 96, 132, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 96, 132, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 96, 132, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 96, 132, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 96, 132, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 96, 132, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 96, 132, NPH9, 23, 16, colorPrint);
        break;
    }
  } else {
    switch (timeSend) {
      case 0:
        DrawImageWH(&paint, 96, 140, NPH0, 23, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 96, 140, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 96, 140, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 96, 140, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 96, 140, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 96, 140, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 96, 140, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 96, 140, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 96, 140, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 96, 140, NPH9, 23, 16, colorPrint);
        break;
    }
  }
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  wait(2000);
}


void reportBattInk() {
  paint.Clear(opposite_colorPrint);

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, NEWBATTINTEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, NEWBATTINT, 64, 192, colorPrint);
#endif

  if (battSend >= 10) {
    byte one_t = battSend / 10;
    byte two_t = battSend % 10;
    switch (one_t) {
      case 1:
        DrawImageWH(&paint, 96, 148, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 96, 148, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 96, 148, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 96, 148, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 96, 148, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 96, 148, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 96, 148, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 96, 148, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 96, 148, NPH9, 23, 16, colorPrint);
        break;
    }

    switch (two_t) {
      case 0:
        DrawImageWH(&paint, 96, 132, NPH0, 23, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 96, 132, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 96, 132, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 96, 132, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 96, 132, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 96, 132, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 96, 132, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 96, 132, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 96, 132, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 96, 132, NPH9, 23, 16, colorPrint);
        break;
    }
  } else {
    switch (battSend) {
      case 0:
        DrawImageWH(&paint, 96, 140, NPH0, 23, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 96, 140, NPH1, 23, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 96, 140, NPH2, 23, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 96, 140, NPH3, 23, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 96, 140, NPH4, 23, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 96, 140, NPH5, 23, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 96, 140, NPH6, 23, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 96, 140, NPH7, 23, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 96, 140, NPH8, 23, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 96, 140, NPH9, 23, 16, colorPrint);
        break;
    }
  }
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();

  wait(2000);
}



void blinkLed () {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}


void preHwInit() {
  pinMode(PIN_BUTTON, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  blinkLed ();
}


void before()
{
#ifndef DCPOWER
  NRF_POWER->DCDCEN = 1;
#endif
  NRF_SAADC ->ENABLE = 0;
  NRF_PWM0  ->ENABLE = 0;
  NRF_TWIM0 ->ENABLE = 0;
  NRF_TWIS0 ->ENABLE = 0;

  NRF_NFCT->TASKS_DISABLE = 1;
  NRF_NVMC->CONFIG = 1;
  NRF_UICR->NFCPINS = 0;
  NRF_NVMC->CONFIG = 0;
  NRF_PWM1  ->ENABLE = 0;
  NRF_PWM2  ->ENABLE = 0;
  NRF_TWIM1 ->ENABLE = 0;
  NRF_TWIS1 ->ENABLE = 0;

#ifndef MY_DEBUG
  NRF_UART0->ENABLE = 0;
#endif

  happy_init();

  timeSend = loadState(102);
  if (timeSend > 30) {
    timeSend = 1;
    saveState(102, timeSend);
  }
  //timeSend = 1; // для теста, 1 минута

  battSend = loadState(103);
  if (battSend > 24) {
    battSend = 3;
    saveState(103, battSend);
  }
  //battSend = 1; // для теста, 1 час

  if (loadState(106) > 1) {
    saveState(106, 0);
  }
  colorChange(loadState(106));
  //colorChange(true); // для теста, true или false

  timeConf();

  displayStart();
  
#ifdef WDTENABLE
  wdt_init();
#endif
}


void presentation()
{
  check = sendSketchInfo(SN, SV);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = sendSketchInfo(SN, SV);
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(TEMP_CHILD_ID, S_TEMP, "Temperature");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(TEMP_CHILD_ID, S_TEMP, "Temperature");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(HUM_CHILD_ID, S_HUM, "Humidity");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(HUM_CHILD_ID, S_HUM, "Humidity");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(BARO_CHILD_ID, S_BARO, "Pressure");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(BARO_CHILD_ID, S_BARO, "Pressure");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(FORECAST_CHILD_ID, S_CUSTOM, "Forecast");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(FORECAST_CHILD_ID, S_CUSTOM, "Forecast");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_TIME_SEND_ID, S_CUSTOM, "T&H SEND INTERVAL | Min");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_TIME_SEND_ID, S_CUSTOM, "T&H SEND INTERVAL | Min");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(MY_SEND_RESET_REASON, S_CUSTOM, "RESTART REASON");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(MY_SEND_RESET_REASON, S_CUSTOM, "RESTART REASON");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
  wait(shortWait * 2);
  sendConfig();
  wait(shortWait);
}



void setup() {
  config_Happy_node();

  if (flag_nogateway_mode == false) {
    requestTime();
    wait(2000, C_REQ, I_TIME);
    wait(300);
    sendResetReason();
  }

  metric = getControllerConfig().isMetric;

  transportDisable();

  interrupt_Init();
#ifdef WDTENABLE
  sleepTimeCount = SLEEP_TIME;
#endif

  bme_initAsleep();

  readBatt();

  blinkLed ();
}


void loop() {
  if (flag_update_transport_param == true) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == true) {
    present_only_parent();
  }
  if (isTransportReady() == true) {
    if (flag_nogateway_mode == false) {
      if (flag_find_parent_process == true) {
        find_parent_process();
      }

      if (configMode == false) {
        if (buttIntStatus == PIN_BUTTON) {
          if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
            wdt_nrfReset();
            button_flag = true;
            previousMillis = millis();
          }
          if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
            if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 4500)) {
              if (updateink1 == false) {
                einkZeropush();
                updateink1 = true;
              }
            }
            if ((millis() - previousMillis > 4500) && (millis() - previousMillis <= 5500)) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 8500)) {
              if (updateink2 == false) {
                einkOnepush();
                updateink2 = true;
                updateinkclear = false;
              }
            }
            if ((millis() - previousMillis > 8500) && (millis() - previousMillis <= 9500)) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 12500)) {
              if (updateink3 == false) {
                einkOnePluspush();
                updateink3 = true;
                updateinkclear = false;
              }
            }
            if (millis() - previousMillis > 12500) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            wdt_nrfReset();
          }
          if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {

            if ((millis() - previousMillis <= 4500) && (button_flag == true))
            {
              einkZeroend();
              reseteinkset();
              configMode = true;
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              wait(shortWait);
              NRF5_ESB_startListening();
              wait(shortWait);
              configMillis = millis();
            }
            if ((millis() - previousMillis > 5500 && millis() - previousMillis <= 8500) && button_flag == true)
            {
              einkOneend();
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              wait(shortWait);
              presentation();
              wait(shortWait);
              transportDisable();
              wait(shortWait * 10);
              change = true;
              BATT_COUNT = BATT_TIME;
#ifdef WDTENABLE
              sleepTimeCount = SLEEP_TIME;
#endif
            }
            if ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 12500) && (button_flag == true))
            {
              einkOnePlusend();
              new_device();
            }
            if ((((millis() - previousMillis > 4500) && (millis() - previousMillis <= 5500)) || ((millis() - previousMillis > 8500) && (millis() - previousMillis <= 9500)) || (millis() - previousMillis > 12500) ) && (button_flag == true) )
            {
              wdt_nrfReset();
              change = true;
              ckeck_hm = false;
#ifdef WDTENABLE
              sleepTimeCount = SLEEP_TIME;
#endif
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
            }
          }
          wdt_nrfReset();
        } else {
          wdt_nrfReset();
#ifdef WDTENABLE
          sleepTimeCount++;
          if (sleepTimeCount >= SLEEP_TIME) {
            sleepTimeCount = 0;
            readData();
            if (change == true) {
              transportReInitialise();
              wait(shortWait);
              sendData();
              transportDisable();
              wait(shortWait);
              displayUpdate(temperature, humidity, pressure, forecast, metric, ckeck_hm);
              wait(shortWait);
              change = false;
            }
          }
#else
          readData();
          if (change == true) {
            transportReInitialise();
            wait(shortWait);
            sendData();
            transportDisable();
            wait(shortWait);
            displayUpdate(temperature, humidity, pressure, forecast, metric, ckeck_hm);
            wait(shortWait);
            change = false;
          }
#endif
          nosleep = false;
        }
      } else {
        if (millis() - configMillis > 20000) {
          configMode = false;
          button_flag = false;
          buttIntStatus = 0;
          transportDisable();
          wait(shortWait * 2);
          change = true;
#ifdef WDTENABLE
          sleepTimeCount = SLEEP_TIME;
#endif
        }
        wdt_nrfReset();
      }
    } else {
      if (buttIntStatus == PIN_BUTTON) {
        if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
          wdt_nrfReset();
          epd.Reset();
          button_flag = true;
          previousMillis = millis();
        }
        if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
          if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 4500)) {
            if (updateink1 == false) {
              einkZeropush();
              updateink1 = true;
            }
          }
          if ((millis() - previousMillis > 4500) && (millis() - previousMillis <= 5500)) {
            if (updateinkclear == false) {
              clearOne();
              updateinkclear = true;
            }
          }
          if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 8500)) {
            if (updateink2 == false) {
              einkOnePluspush();
              updateink2 = true;
              updateinkclear = false;
            }
          }
          if (millis() - previousMillis > 8500) {
            if (updateinkclear == false) {
              clearOne();
              updateinkclear = true;
            }
          }
          wdt_nrfReset();
        }
        if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {
          if (millis() - previousMillis <= 4500 && button_flag == true)
          {
            wdt_nrfReset();
            einkZeroend();
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
            transportReInitialise();
            check_parent();
            cpCount = 0;
            change = true;
            BATT_COUNT = BATT_TIME;
#ifdef WDTENABLE
            sleepTimeCount = SLEEP_TIME;
#endif
          }
          if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 8500) && (button_flag == true))
          {
            einkOnePlusend();
            new_device();
          }
          if ( ( ( millis() - previousMillis > 4500 && millis() - previousMillis <= 5500 ) || ( millis() - previousMillis > 8500)) && button_flag == true)
          {
            wdt_nrfReset();
            change = true;
            ckeck_hm = false;
#ifdef WDTENABLE
            sleepTimeCount = SLEEP_TIME;
#endif
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
          }
        }
        wdt_nrfReset();
      } else {
#ifdef WDTENABLE
        sleepTimeCount++;
        if (sleepTimeCount >= SLEEP_TIME) {
          sleepTimeCount = 0;
          cpCount++;
          if (cpCount >= cpNom) {
            transportReInitialise();
            check_parent();
            cpCount = 0;
          }
          readData();
          if (change == true) {
            change = false;
            if (flag_nogateway_mode == false) {
              transportReInitialise();
            }
            wait(shortWait);
            sendData();
            if (flag_nogateway_mode == false) {
              transportDisable();
            }
            wait(shortWait);
            displayUpdate(temperature, humidity, pressure, forecast, metric, ckeck_hm);
            wait(shortWait);
          }
          sleepTimeCount = 0;
        }
#else
        cpCount++;
        if (cpCount >= cpNom) {
          transportReInitialise();
          check_parent();
          cpCount = 0;
        }
        readData();
        if (change == true) {
          change = false;
          if (flag_nogateway_mode == false) {
            transportReInitialise();
          }
          wait(shortWait);
          sendData();
          if (flag_nogateway_mode == false) {
            transportDisable();
          }
          wait(shortWait);
          displayUpdate(temperature, humidity, pressure, forecast, metric, ckeck_hm);
          wait(shortWait);
        }
#endif
        //if ((cpCount < cpNom) && (flag_nogateway_mode == true)) {
        if (cpCount < cpNom) {
          nosleep = false;
        }
      }
    }
  }
  if (_transportSM.failureCounter > 0)
  {
    _transportConfig.parentNodeId = loadState(201);
    _transportConfig.nodeId = myid;
    _transportConfig.distanceGW = loadState(202);
    mypar = _transportConfig.parentNodeId;
    nosleep = false;
    err_delivery_beat = 6;
    happy_node_mode();
    gateway_fail();
  }

  if (nosleep == false) {
    wdt_nrfReset();
    transportDisable();
#ifdef WDTENABLE
    hwSleep(SLEEP_TIME_WDT);
#else
    hwSleep(SLEEP_TIME);
#endif
    nosleep = true;
  }
}


void bme_initAsleep() {
  if (! bme.begin(&Wire)) {
    while (1);
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
  wait(500);
}


void readData() {
  bme.takeForcedMeasurement();
  wait(shortWait);
  temperature = bme.readTemperature();
  wait(shortWait);
  pressure = bme.readPressure() / 100.0F;
  forecast = sample(pressure);
  wait(shortWait);
  if (chek_h == true) {
    humidity = bme.readHumidity();
    wait(shortWait);
    if ((int)humidity < 0) {
      humidity = 0.0;
    }
    if ((int)humidity > 99) {
      humidity = 99.9;
    }
    chek_h = false;
  } else {
    chek_h = true;
  }
  if ((int)temperature < 0) {
    temperature = 0.0;
  }
  if ((int)temperature > 99) {
    temperature = 99.9;
  }
  if ((int)pressure < 300) {
    pressure = 300.0;
  }
  if ((int)pressure > 1100) {
    pressure = 1100.0;
  }
  if (forecast != old_forecast) {
    change = true;
    fch = true;
    if ((old_forecast != 5) && (forecast == 0)) {
      forecast = old_forecast;
      change = false;
      fch = false;
    }
    if ((old_forecast != 5) && (forecast != 0)) {
      old_forecast = forecast;
    }
    if (old_forecast == 5) {
      old_forecast = forecast;
    }
  }
  if (!metric) {
    temperature = temperature * 9.0 / 5.0 + 32.0;
  } else {
    pressure = pressure * 0.75006375541921;
  }
  if (abs(temperature - old_temperature) >= tempThreshold) {
    old_temperature = temperature;
    change = true;
    tch = true;
  }
  if (abs(pressure - old_pressure) >= pressThreshold) {
    old_pressure = pressure;
    change = true;
    pch = true;
  }
  if (abs(humidity - old_humidity) >= humThreshold) {
    old_humidity = humidity;
    change = true;
    hch = true;
  }
  BATT_COUNT++;
  CORE_DEBUG(PSTR("BATT_COUNT: %d\n"), BATT_COUNT);
  if (BATT_COUNT >= BATT_TIME) {
    CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
    change = true;
  }

  byte checkMinutePrint = minute();
  if (minutePrint != checkMinutePrint) {
    if (!change) {
      change = true;
      ckeck_hm = true;
    }
  }
  wdt_nrfReset();
}


void sendData() {
  bool blinkEnable = false;
  if (flag_nogateway_mode == false) {
    if (timeSend != 0) {
      if (tch == true) {
        check = send(msgTemp.setDestination(0).set(temperature, 2));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(shortWait * 4);
          check = send(msgTemp.setDestination(0).set(temperature, 2));
          if (check == false) {
            wait(shortWait * 8);
            _transportSM.failedUplinkTransmissions = 0;
            check = send(msgTemp.setDestination(0).set(temperature, 2));
            wait(shortWait * 2);
          }
        }
        tch = false;
        checkSend();
        blinkEnable = true;
      }
      if (hch == true) {
        check = send(msgHum.setDestination(0).set(humidity, 2));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(shortWait * 4);
          check = send(msgHum.setDestination(0).set(humidity, 2));
          if (check == false) {
            wait(shortWait * 8);
            _transportSM.failedUplinkTransmissions = 0;
            check = send(msgHum.setDestination(0).set(humidity, 2));
            wait(shortWait * 2);
          }
        }
        hch = false;
        checkSend();
        blinkEnable = true;
      }
      if (pch == true) {
        check = send(msgPres.setDestination(0).set(pressure, 2));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(shortWait * 4);
          check = send(msgPres.setDestination(0).set(pressure, 2));
          if (check == false) {
            wait(shortWait * 8);
            _transportSM.failedUplinkTransmissions = 0;
            check = send(msgPres.setDestination(0).set(pressure, 2));
            wait(shortWait * 2);
          }
        }
        pch = false;
        checkSend();
        blinkEnable = true;
      }
      if (fch == true) {
        check = send(forecastMsg.set(forecast));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(50);
          check = send(forecastMsg.set(forecast));
          wait(50);
        }
        fch = false;
        blinkEnable = true;
      }
    }
    if (BATT_COUNT >= BATT_TIME) {
      CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
      wait(5);
      readBatt();
      BATT_COUNT = 0;
    }
    if (bch == true) {
      if (BATT_TIME != 0) {
        batLevSend();
        ckeck_hm = false;
      }
      bch = false;
      blinkEnable = true;
    }
    if (blinkEnable == true) {
      blinkLed();
    }
  } else {
    if (BATT_COUNT >= BATT_TIME) {
      CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
      wait(5);
      readBatt();
      BATT_COUNT = 0;
      if (bch == true) {
        ckeck_hm = false;
      }
    }
    tch = false;
    hch = false;
    bch = false;
    pch = false;
    fch = false;
  }
  wdt_nrfReset();
  if (!timeReceived) {
    requestTime();
    wait(2000, C_REQ, I_TIME);
  }
  wdt_nrfReset();
}


void checkSend() {
  if (check == true) {
    err_delivery_beat = 0;
    if (flag_nogateway_mode == true) {
      flag_nogateway_mode = false;
      CORE_DEBUG(PSTR("MyS: Flag_nogateway_mode = FALSE\n"));
      CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
      err_delivery_beat = 0;
    }
    CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
    CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
  } else {
    _transportSM.failedUplinkTransmissions = 0;
    if (err_delivery_beat < 6) {
      err_delivery_beat++;
    }
    if (err_delivery_beat == 5) {
      if (flag_nogateway_mode == false) {
        happy_node_mode();
        gateway_fail();
        CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
      }
    }
  }
}


void readBatt() {
  wait(50);
  batteryVoltage = hwCPUVoltage();
  wait(10);
  battery = battery_level_in_percent(batteryVoltage);
  batteryVoltageF = (float)batteryVoltage / 1000.00;
  CORE_DEBUG(PSTR("battery voltage: %d\n"), batteryVoltage);
  CORE_DEBUG(PSTR("battery percentage: %d\n"), battery);
  if (BATT_TIME != 0) {
    bch = true;
  }
}


void batLevSend() {
  if (BATT_TIME != 0) {
    if (battery > 100) {
      battery = 100;
    }
    check = sendBatteryLevel(battery, 1);
    wait(1500, C_INTERNAL, I_BATTERY_LEVEL);
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(50);
      check = sendBatteryLevel(battery, 1);
      wait(1500, C_INTERNAL, I_BATTERY_LEVEL);
    }
    checkSend();
    wdt_nrfReset();
    lqSend();

    check = send(bvMsg.set(batteryVoltageF, 2));
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(100);
      check = send(bvMsg.set(batteryVoltageF, 2));
      _transportSM.failedUplinkTransmissions = 0;
      wait(100);
    } else {
      CORE_DEBUG(PSTR("MyS: SEND BATTERY VOLTAGE\n"));
    }
  }
}


void lqSend() {
  nRFRSSI = transportGetReceivingRSSI();
  nRFRSSI = map(nRFRSSI, -85, -40, 0, 100);
  if (nRFRSSI < 0) {
    nRFRSSI = 0;
  }
  if (nRFRSSI > 100) {
    nRFRSSI = 100;
  }

  check = send(sqMsg.set(nRFRSSI));
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait * 2);
    check = send(sqMsg.set(nRFRSSI));
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    CORE_DEBUG(PSTR("MyS: SEND LINK QUALITY\n"));
    CORE_DEBUG(PSTR("MyS: LINK QUALITY %: %d\n"), nRFRSSI);
  }
}


static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 3000)
  {
    battery_level = 100;
  }
  else if (mvolts > 2900)
  {
    battery_level = 100 - ((3000 - mvolts) * 20) / 100;
  }
  else if (mvolts > 2750)
  {
    battery_level = 80 - ((2900 - mvolts) * 30) / 150;
  }
  else if (mvolts > 2550)
  {
    battery_level = 50 - ((2750 - mvolts) * 40) / 200;
  }
  else if (mvolts > 2250)
  {
    battery_level = 10 - ((2550 - mvolts) * 10) / 300;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}



void timeConf() {
#ifdef WDTENABLE
  if (timeSend != 0) {
    SLEEP_TIME = (timeSend * minuteT / SLEEP_TIME_WDT);
  } else {
    SLEEP_TIME = (minuteT / SLEEP_TIME_WDT);
  }
  if (battSend != 0) {
    if (timeSend != 0) {
      BATT_TIME = (battSend * 60 / timeSend);
    } else {
      BATT_TIME = (battSend * 60);
    }
  } else {
    BATT_TIME = 0;
  }
  if (timeSend != 0) {
    cpNom = (60 / timeSend);
  } else {
    cpNom = 60;
  }
  CORE_DEBUG(PSTR("SLEEP_TIME: %d\n"), SLEEP_TIME);
#else
  if (timeSend != 0) {
    SLEEP_TIME = timeSend * minuteT;
  } else {
    if (battSend != 0) {
      SLEEP_TIME = battSend * minuteT * 60;
    } else {
      SLEEP_TIME = minuteT * 60 * 24;
    }
  }
  if (battSend != 0) {
    if (timeSend != 0) {
      BATT_TIME = battSend * 60 / timeSend;
    } else {
      BATT_TIME = 1;
    }
  } else {
    BATT_TIME = 0;
  }

  cpNom = 60 / timeSend;
  CORE_DEBUG(PSTR("MyS: BATT_TIME: %d\n"), BATT_TIME);
#endif
}


void sendConfig() {
  check = send(setTimeSendMsg.set(timeSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setTimeSendMsg.set(timeSend));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = send(setBattSendMsg.set(battSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setBattSendMsg.set(battSend));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }

  bool inverse = loadState(106);
  check = send(setColor.set(inverse));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(setColor.set(inverse));
    wait(shortWait * 2);
    _transportSM.failedUplinkTransmissions = 0;
  }
}


void sendResetReason() {
  String reason;
#ifdef MY_RESET_REASON_TEXT
  if (NRF_POWER->RESETREAS == 0) reason = "POWER_ON";
  else {
    if (NRF_POWER->RESETREAS & (1UL << 0)) reason += "PIN_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 1)) reason += "WDT ";
    if (NRF_POWER->RESETREAS & (1UL << 2)) reason += "SOFT_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 3)) reason += "LOCKUP";
    if (NRF_POWER->RESETREAS & (1UL << 16)) reason += "WAKEUP_GPIO ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "LPCOMP ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "WAKEUP_DEBUG";
  }
#else
  reason = NRF_POWER->RESETREAS;
#endif

  check = send(sendMsg.set(reason.c_str()));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait);
    check = send(sendMsg.set(reason.c_str()));
    if (check == false) {
      wait(shortWait * 3);
      _transportSM.failedUplinkTransmissions = 0;
      check = send(sendMsg.set(reason.c_str()));
      wait(shortWait);
    }
  }
  if (check) NRF_POWER->RESETREAS = (0xFFFFFFFF);
}


void receive(const MyMessage & message)
{
  if (configMode == true) {
    if (message.sensor == SET_TIME_SEND_ID) {
      if (message.type == V_VAR1) {
        timeSend = message.getByte();
        if (timeSend > 60) {
          timeSend = 60;
        }
        saveState(102, timeSend);
        wait(shortWait);
        send(setTimeSendMsg.set(timeSend));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        reportTimeInk();
        configMode = false;
        change = true;
        timeConf();
#ifdef WDTENABLE
        sleepTimeCount = SLEEP_TIME;
#endif
      }
    }

    if (message.sensor == SET_BATT_SEND_ID) {
      if (message.type == V_VAR1) {
        battSend = message.getByte();
        if (battSend > 24) {
          battSend = 24;
        }
        saveState(103, battSend);
        wait(shortWait);
        send(setBattSendMsg.set(battSend));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        reportBattInk();
        configMode = false;
        change = true;
        timeConf();
#ifdef WDTENABLE
        sleepTimeCount = SLEEP_TIME;
#endif
      }
    }

    if (message.sensor == SET_COLOR_ID) {
      if (message.type == V_VAR1) {
        bool colorPrintTemp = message.getBool();
        colorChange(colorPrintTemp);
        wait(shortWait);
        send(setColor.set(colorPrintTemp));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        configMode = false;
        change = true;
#ifdef WDTENABLE
        sleepTimeCount = SLEEP_TIME;
#endif
      }
    }
  }
}


void interrupt_Init() {
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_NOPULL);
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON_MASK = 1 << PIN_BUTTON;
  app_gpiote_user_register(&m_gpiote_user_id, PIN_BUTTON_MASK, PIN_BUTTON_MASK, gpiote_event_handler);
  app_gpiote_user_enable(m_gpiote_user_id);
  buttIntStatus = 0;
}

void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2);

  if (PIN_BUTTON_MASK & event_pins_high_to_low) {
    if (buttIntStatus == 0) {
      buttIntStatus = PIN_BUTTON;
    }
  }
}


static __INLINE void wdt_init(void)
{
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = 35 * 32768;
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}


static __INLINE void wdt_nrfReset() {
#ifdef WDTENABLE
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
#endif
}


void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  saveState(200, 255);
  hwReboot();
}


// ####################################################################################################
// #                                                                                                  #
// #            These functions are only included if the forecast function is enables.                #
// #          The are used to generate a weater prediction by checking if the barometric              #
// #                          pressure is rising or falling over time.                                #
// #                                                                                                  #
// ####################################################################################################

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++) {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}

// Forecast algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure) {
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185) {
    minuteCount = 6;
  }

  if (minuteCount == 5) {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { //first time initial 3 hour
      dP_dt = change; //note this is for t = 1 hour
    }
    else {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int16_t forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) { //if time is less than 35 min on the first 3 hour interval.
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25)) {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25) {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05))) {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05)) {
    forecast = STABLE;
  }
  else {
    forecast = UNKNOWN;
  }
  return forecast;
}



// ####################################################################################################
// #                                                                                                  #
// #                                            HAPPY MODE                                            #
// #                                                                                                  #
// ####################################################################################################

void happy_init() {

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 0) {
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  }
  if (loadState(200) == 0) {
    saveState(200, 255);
  }
  CORE_DEBUG(PSTR("EEPROM NODE ID: %d\n"), hwReadConfig(EEPROM_NODE_ID_ADDRESS));
  CORE_DEBUG(PSTR("USER MEMORY SECTOR NODE ID: %d\n"), loadState(200));

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 255) {
    mtwr = 0;
  } else {
    mtwr = 15000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);
}


void config_Happy_node() {
  if (mtwr == 0) {
    myid = getNodeId();
    saveState(200, myid);
    mypar = _transportConfig.parentNodeId;
    old_mypar = mypar;
    saveState(201, mypar);
    saveState(202, _transportConfig.distanceGW);
  }
  if (mtwr != 0) {
    myid = getNodeId();
    if (myid != loadState(200)) {
      saveState(200, myid);
    }
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      if (mypar != loadState(201)) {
        saveState(201, mypar);
      }
      if (_transportConfig.distanceGW != loadState(202)) {
        saveState(202, _transportConfig.distanceGW);
      }
      present_only_parent();
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 6;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
}


void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(1000, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == C_INTERNAL) {
      if (_msg.type == 8) {
        Ack_FP = true;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == true) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    Ack_FP = false;
    transportSwitchSM(stParent);
    flag_nogateway_mode = false;
    CORE_DEBUG(PSTR("MyS: Flag_nogateway_mode = FALSE\n"));
    flag_find_parent_process = true;
    problem_mode_count = 0;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
    transportDisable();
    change = true;
  }
}


void find_parent_process() {
  flag_update_transport_param = true;
  flag_find_parent_process = false;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
}


void gateway_fail() {
  flag_nogateway_mode = true;
  CORE_DEBUG(PSTR("MyS: Flag_nogateway_mode = TRUE\n"));
  flag_update_transport_param = false;
  change = true;
}


void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  _transportSM.failureCounter = 0u;
  _transportSM.uplinkOk = true;
  _transportSM.failureCounter = 0u;
  _transportSM.failedUplinkTransmissions = 0u;
  transportSwitchSM(stReady);
  CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
}


void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = false;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = true;
    }
  }
}


void update_Happy_transport() {
  CORE_DEBUG(PSTR("MyS: UPDATE TRANSPORT CONFIGURATION\n"));
  mypar = _transportConfig.parentNodeId;
  if (mypar != loadState(201))
  {
    saveState(201, mypar);
  }
  if (_transportConfig.distanceGW != loadState(202))
  {
    saveState(202, _transportConfig.distanceGW);
  }
  present_only_parent();
  wait(shortWait);
  flag_update_transport_param = false;
#ifdef WDTENABLE
  sleepTimeCount = SLEEP_TIME;
#endif
  BATT_COUNT = BATT_TIME;
  change = true;
}


void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}
