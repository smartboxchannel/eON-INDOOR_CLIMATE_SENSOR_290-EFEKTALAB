// ###################           Mini wither station with electronic ink display 2.9 Inch | nRF52            ############### //
//                                                                                                                           //
//        @filename   :   EFEKTA_THPEINK290_1.ino                                                                         //
//        @brief en   :   Wireless, battery-operated temperature,humidity and pressure sensor(SHT20, SI7020, HTU21D, BME280) //
//                        with electronic ink display(Good Display GDEH029A1 OR GDEM029T94).                                 //
//                        The extended version adds the MAX44009 light sensor, an active bizzer Works on nRF52.              //
//        @brief ru   :   Беcпроводной, батарейный датчик температуры, влажности и давления(SHT20, SI7020, HTU21D, BME280)   //
//                        с дисплеем на электронных чернилах(Good Display GDEH029A1 или GDEM029T94).                         //
//                        В расширенной версии добавлен  датчик света MAX44009, активный биззер.                             //
//                        Работает на nRF52832, nRF52840.                                                                    //
//                                                                                                                           //
//        @author     :   Andrew Lamchenko aka Berk                                                                          //
//                                                                                                                           //
//        Copyright (C) EFEKTALAB 2020                                                                                       //
//        Copyright (c) 2014-1215 Arduino LLC.  All right reserved.                                                          //
//        Copyright (c) 2016 Arduino Srl.  All right reserved.                                                               //
//        Copyright (c) 2017 Sensnology AB. All right reserved.                                                              //
//        Copyright (C) Waveshare     August 10 2017                                                                         //
//                                                                                                                           //
// ######################################################################################################################### //
#include <Wire.h>
#include "MyConfig.h"
#include "eink290.h"
#include "einkpaint.h"
#include "einkimgdata.h"
#ifdef LIGHTSENS
#include <MAX44009.h>
MAX44009 light;
#endif

const uint16_t shortWait = 10;
uint16_t minuteT = 60000;
float tempThreshold = 0.5;
float humThreshold = 2.5;
float pressThreshold = 1.0;

uint32_t stopTimer;
uint32_t startTimer;
uint32_t sleepTimeCount;
uint32_t SLEEP_TIME;
const uint32_t SLEEP_TIME_WDT = 10000;
uint32_t PRECISION_TIME_WDT;
uint32_t PRECISION_TIME;

byte clearEpaper;
bool design;
bool error_bme;
bool needPresent;
bool mesInfo;
bool mesTemp;
bool mesHum;
bool mesBaro;
bool mesForec;
bool mesSig;
bool mesBat;
bool mesBatset;
bool mesRes;
bool mesColorset;
bool mesDesignset;
#ifdef LIGHTSENS
bool mesLux;
#endif
#ifdef BIZZER
bool mesSoundset;
bool changeBiz = true;
#endif
//bool changeT = true;
bool changeB = true;
bool changeC = true;
bool changeD = true;
bool sendAfterResTask;

bool setSound = true;
bool colorPrint;
bool opposite_colorPrint;
bool change;
bool check;
bool tch;
bool hch;
bool bch;
bool pch;
bool lch;
bool fch;
bool gch;
bool qch;
bool metric;
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
bool updateink5;
bool updateinkclear;
uint8_t battery;
uint8_t old_battery;
uint8_t cpNom;
uint8_t cpCount;
uint8_t timeSend = 1;
uint8_t battSend;
uint8_t err_delivery_beat;
uint16_t batteryVoltage;
int16_t nRFRSSI;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;

#ifdef LIGHTSENS
float brightness = 0.0;
float old_brightness = -10.0;
float brightThreshold = 5.0;
#endif

uint16_t BATT_TIME;
uint16_t BATT_COUNT;
uint32_t configMillis;
uint32_t previousMillis;
float batteryVoltageF;
float temperatureSens;
float pressureSens;
float humiditySens;
float old_temperature;
float old_humidity;
float old_pressure;
unsigned char image[5000];
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
#ifdef LIGHTSENS
#define LUX_SENS_CHILD_ID 4
#endif
#define SIGNAL_Q_ID 100
#define BATTERY_VOLTAGE_ID 101
#define SET_BATT_SEND_ID 103
#define MY_SEND_RESET_REASON 105
#define SET_COLOR_ID 106
#define SET_DESIGN_ID 109
#ifdef BIZZER
#define SET_SOUND_ID 107
#endif
MyMessage msgTemp(TEMP_CHILD_ID, V_TEMP);
MyMessage msgHum(HUM_CHILD_ID, V_HUM);
MyMessage msgPres(BARO_CHILD_ID, V_PRESSURE);
MyMessage forecastMsg(FORECAST_CHILD_ID, V_VAR1);
#ifdef LIGHTSENS
MyMessage brightMsg(LUX_SENS_CHILD_ID, V_LEVEL);
#endif
#ifdef BIZZER
MyMessage setSoundMsg(SET_SOUND_ID, V_VAR1);
#endif
MyMessage sqMsg(SIGNAL_Q_ID, V_VAR1);
MyMessage bvMsg(BATTERY_VOLTAGE_ID, V_VAR1);
MyMessage setBattSendMsg(SET_BATT_SEND_ID, V_VAR1);
MyMessage sendMsg(MY_SEND_RESET_REASON, V_VAR1);
MyMessage setColor(SET_COLOR_ID, V_VAR1);
MyMessage setDesign(SET_DESIGN_ID, V_VAR1);


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


#ifdef BME280
#include <Adafruit_Sensor.h>
#include "Adafruit_BME280.h"
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
#else
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bme;
#include "Adafruit_HTU21DF.h"
Adafruit_HTU21DF sensor = Adafruit_HTU21DF();
#endif


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
 // temp = 5.6;
  //metr = false;


  if (design == false) {
#ifdef LIGHTSENS
#ifdef LANG_EN
    DrawImageWH(&paint, 3, 112, TEMPEN, 10, 72, colorPrint);
#else
    DrawImageWH(&paint, 3, 112, TEMP, 10, 72, colorPrint);
#endif

    int temperature_temp = round(temp * 10.0);

    if (metr) {
      if (temperature_temp >= 100) {

        DrawImageWH(&paint, 24, 63, NTC, 16, 20, colorPrint);

        byte one_t = temperature_temp / 100;
        byte two_t = temperature_temp % 100 / 10;
        byte three_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 14, 174, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 174, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 174, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 174, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 174, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 174, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 174, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 174, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 174, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 174, NT9, 66, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 14, 134, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 134, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 134, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 134, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 134, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 134, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 134, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 134, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 134, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 134, NT9, 66, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 69, 123, NTP, 11, 11, colorPrint);

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 14, 83, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 83, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 83, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 83, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 83, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 83, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 83, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 83, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 83, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 83, NT9, 66, 40, colorPrint);
            break;
        }
      } else {

        DrawImageWH(&paint, 22, 83, NTC, 16, 20, colorPrint);

        byte one_t = temperature_temp / 10;
        byte two_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 14, 154, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 154, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 154, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 154, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 154, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 154, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 154, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 154, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 154, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 154, NT9, 66, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 69, 143, NTP, 11, 11, colorPrint);

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 14, 103, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 103, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 103, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 103, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 103, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 103, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 103, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 103, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 103, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 103, NT9, 66, 40, colorPrint);
            break;
        }
      }
    } else {
      if (temperature_temp < 1000) {

        DrawImageWH(&paint, 22, 63, NTF, 16, 20, colorPrint);

        byte one_t = temperature_temp / 100;
        byte two_t = temperature_temp % 100 / 10;
        byte three_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 14, 174, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 174, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 174, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 174, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 174, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 174, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 174, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 174, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 174, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 174, NT9, 66, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 14, 134, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 134, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 134, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 134, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 134, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 134, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 134, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 134, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 134, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 134, NT9, 66, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 69, 123, NTP, 11, 11, colorPrint);

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 14, 83, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 83, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 83, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 83, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 83, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 83, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 83, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 83, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 83, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 83, NT9, 66, 40, colorPrint);
            break;
        }
      } else {

        DrawImageWH(&paint, 22, 68, NTF, 16, 20, colorPrint);

        byte one_t = temperature_temp / 1000;
        byte two_t = temperature_temp % 1000 / 100;
        byte three_t = temperature_temp % 100 / 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 14, 168, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 168, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 168, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 168, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 168, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 168, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 168, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 168, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 168, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 168, NT9, 66, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 14, 128, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 128, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 128, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 128, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 128, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 128, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 128, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 128, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 128, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 128, NT9, 66, 40, colorPrint);
            break;
        }

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 14, 88, NT0, 66, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 14, 88, NT1, 66, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 14, 88, NT2, 66, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 14, 88, NT3, 66, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 14, 88, NT4, 66, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 14, 88, NT5, 66, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 14, 88, NT6, 66, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 14, 88, NT7, 66, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 14, 88, NT8, 66, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 14, 88, NT9, 66, 40, colorPrint);
            break;
        }
      }
    }
#else
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
          case 0:
            DrawImageWH(&paint, 22, 174, NT0, 66, 40, colorPrint);
            break;
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
          case 0:
            DrawImageWH(&paint, 22, 154, NT0, 66, 40, colorPrint);
            break;
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
          case 0:
            DrawImageWH(&paint, 22, 174, NT0, 66, 40, colorPrint);
            break;
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
          case 0:
            DrawImageWH(&paint, 22, 168, NT0, 66, 40, colorPrint);
            break;
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
#endif
  } else {

#ifdef LIGHTSENS

    int temperature_temp = round(temp * 10.0);

    if (metr) {
      if (temperature_temp >= 100) {

        DrawImageWH(&paint, 24 + 10, 63 - 1, NNC, 16, 20, colorPrint);

        byte one_t = temperature_temp / 100;
        byte two_t = temperature_temp % 100 / 10;
        byte three_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN9, 61, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 134, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 134, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 134, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 134, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 134, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 134, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 134, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 134, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 134, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 134, NN9, 61, 40, colorPrint);
            break;
        }
        DrawImageWH(&paint, 22 + 10, 123, NNP, 61, 12, colorPrint);

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 83, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 83, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 83, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 83, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 83, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 83, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 83, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 83, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 83, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 83, NN9, 61, 40, colorPrint);
            break;
        }
      } else {

        DrawImageWH(&paint, 24 + 10, 83 - 1, NNC, 16, 20, colorPrint);

        byte one_t = temperature_temp / 10;
        byte two_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 154, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 154, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 154, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 154, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 154, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 154, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 154, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 154, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 154, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 154, NN9, 61, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 22 + 10, 143, NNP, 61, 12, colorPrint);

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 103, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 103, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 103, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 103, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 103, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 103, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 103, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 103, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 103, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 103, NN9, 61, 40, colorPrint);
            break;
        }
      }
    } else {
      if (temperature_temp < 1000) {

        DrawImageWH(&paint, 24 + 10, 63 - 1, NNF, 16, 20, colorPrint);

        byte one_t = temperature_temp / 100;
        byte two_t = temperature_temp % 100 / 10;
        byte three_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 174 + 1, NN9, 61, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 134, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 134, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 134, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 134, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 134, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 134, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 134, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 134, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 134, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 134, NN9, 61, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 22 + 10, 123, NNP, 61, 12, colorPrint);

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 83, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 83, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 83, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 83, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 83, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 83, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 83, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 83, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 83, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 83, NN9, 61, 40, colorPrint);
            break;
        }
      } else {

        DrawImageWH(&paint, 24 + 10, 68 - 2, NNF, 16, 20, colorPrint);

        byte one_t = temperature_temp / 1000;
        byte two_t = temperature_temp % 1000 / 100;
        byte three_t = temperature_temp % 100 / 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 168 + 1, NN9, 61, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 128, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 128, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 128, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 128, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 128, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 128, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 128, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 128, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 128, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 128, NN9, 61, 40, colorPrint);
            break;
        }

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22 + 10, 88 - 1, NN9, 61, 40, colorPrint);
            break;
        }
      }
    }
#else



#ifdef LANG_EN
    DrawImageWH(&paint, 6, 112, TEMPEN, 10, 72, colorPrint);
#else
    DrawImageWH(&paint, 6, 112, TEMP, 10, 72, colorPrint);
#endif

    int temperature_temp = round(temp * 10.0);

    if (metr) {
      if (temperature_temp >= 100) {

        DrawImageWH(&paint, 24, 63 - 1, NNC, 16, 20, colorPrint);

        byte one_t = temperature_temp / 100;
        byte two_t = temperature_temp % 100 / 10;
        byte three_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22, 174 + 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 174 + 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 174 + 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 174 + 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 174 + 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 174 + 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 174 + 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 174 + 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 174 + 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 174 + 1, NN9, 61, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22, 134, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 134, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 134, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 134, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 134, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 134, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 134, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 134, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 134, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 134, NN9, 61, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 22, 123, NNP, 61, 12, colorPrint);

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 22, 83, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 83, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 83, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 83, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 83, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 83, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 83, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 83, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 83, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 83, NN9, 61, 40, colorPrint);
            break;
        }
      } else {

        DrawImageWH(&paint, 24, 83 - 1, NNC, 16, 20, colorPrint);

        byte one_t = temperature_temp / 10;
        byte two_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22, 154, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 154, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 154, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 154, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 154, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 154, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 154, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 154, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 154, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 154, NN9, 61, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 22, 143, NNP, 61, 12, colorPrint);

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22, 103, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 103, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 103, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 103, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 103, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 103, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 103, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 103, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 103, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 103, NN9, 61, 40, colorPrint);
            break;
        }
      }
    } else {
      if (temperature_temp < 1000) {

        DrawImageWH(&paint, 24, 63 - 1, NNF, 16, 20, colorPrint);

        byte one_t = temperature_temp / 100;
        byte two_t = temperature_temp % 100 / 10;
        byte three_t = temperature_temp % 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22, 174 + 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 174 + 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 174 + 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 174 + 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 174 + 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 174 + 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 174 + 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 174 + 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 174 + 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 174 + 1, NN9, 61, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22, 134, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 134, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 134, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 134, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 134, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 134, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 134, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 134, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 134, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 134, NN9, 61, 40, colorPrint);
            break;
        }

        DrawImageWH(&paint, 22, 123, NNP, 61, 12, colorPrint);

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 22, 83, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 83, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 83, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 83, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 83, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 83, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 83, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 83, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 83, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 83, NN9, 61, 40, colorPrint);
            break;
        }
      } else {

        DrawImageWH(&paint, 24, 68 - 2, NNF, 16, 20, colorPrint);

        byte one_t = temperature_temp / 1000;
        byte two_t = temperature_temp % 1000 / 100;
        byte three_t = temperature_temp % 100 / 10;

        switch (one_t) {
          case 0:
            DrawImageWH(&paint, 22, 168 + 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 168 + 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 168 + 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 168 + 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 168 + 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 168 + 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 168 + 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 168 + 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 168 + 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 168 + 1, NN9, 61, 40, colorPrint);
            break;
        }

        switch (two_t) {
          case 0:
            DrawImageWH(&paint, 22, 128, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 128, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 128, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 128, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 128, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 128, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 128, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 128, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 128, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 128, NN9, 61, 40, colorPrint);
            break;
        }

        switch (three_t) {
          case 0:
            DrawImageWH(&paint, 22, 88 - 1, NN0, 61, 40, colorPrint);
            break;
          case 1:
            DrawImageWH(&paint, 22, 88 - 1, NN1, 61, 40, colorPrint);
            break;
          case 2:
            DrawImageWH(&paint, 22, 88 - 1, NN2, 61, 40, colorPrint);
            break;
          case 3:
            DrawImageWH(&paint, 22, 88 - 1, NN3, 61, 40, colorPrint);
            break;
          case 4:
            DrawImageWH(&paint, 22, 88 - 1, NN4, 61, 40, colorPrint);
            break;
          case 5:
            DrawImageWH(&paint, 22, 88 - 1, NN5, 61, 40, colorPrint);
            break;
          case 6:
            DrawImageWH(&paint, 22, 88 - 1, NN6, 61, 40, colorPrint);
            break;
          case 7:
            DrawImageWH(&paint, 22, 88 - 1, NN7, 61, 40, colorPrint);
            break;
          case 8:
            DrawImageWH(&paint, 22, 88 - 1, NN8, 61, 40, colorPrint);
            break;
          case 9:
            DrawImageWH(&paint, 22, 88 - 1, NN9, 61, 40, colorPrint);
            break;
        }
      }
    }
#endif
  }
}


#ifdef LIGHTSENS
void displayLux(float brig_temp) {

  long brig = round(brig_temp * 10.0);
  brig = 23456;


  if (design == false) {
    if (brig < 100) {
      byte one_l = brig / 10;
      byte two_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 82, 120, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 82, 120, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 82, 167, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 167, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 167, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 167, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 167, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 167, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 167, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 167, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 167, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 167, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 82, 162, CDP, 16, 5, colorPrint);

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 82, 153, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 153, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 153, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 153, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 153, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 153, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 153, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 153, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 153, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 153, CD9, 16, 9, colorPrint);
          break;
      }
    }
    if (brig >= 100 && brig < 1000) {
      byte one_l = brig / 100;
      byte two_l = brig % 10 / 10;
      byte three_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 82, 116, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 82, 116, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 82, 172, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 172, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 172, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 172, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 172, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 172, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 172, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 172, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 172, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 172, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 82, 163, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 163, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 163, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 163, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 163, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 163, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 163, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 163, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 163, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 163, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 82, 158, CDP, 16, 5, colorPrint);

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 82, 149, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 149, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 149, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 149, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 149, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 149, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 149, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 149, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 149, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 149, CD9, 16, 9, colorPrint);
          break;
      }
    }

    if (brig >= 1000 && brig < 10000) {
      byte one_l = brig / 1000;
      byte two_l = brig % 1000 / 100;
      byte three_l = brig % 100 / 10;
      byte four_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 82, 111, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 82, 111, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 82, 176, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 176, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 176, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 176, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 176, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 176, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 176, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 176, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 176, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 176, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 82, 167, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 167, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 167, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 167, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 167, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 167, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 167, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 167, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 167, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 167, CD9, 16, 9, colorPrint);
          break;
      }

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 82, 158, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 158, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 158, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 158, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 158, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 158, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 158, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 158, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 158, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 158, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 82, 153, CDP, 16, 5, colorPrint);

      switch (four_l) {
        case 0:
          DrawImageWH(&paint, 82, 144, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 144, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 144, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 144, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 144, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 144, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 144, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 144, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 144, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 144, CD9, 16, 9, colorPrint);
          break;
      }
    }

    if (brig >= 10000 && brig < 100000) {
      byte one_l = brig / 10000;
      byte two_l = brig % 10000 / 1000;
      byte three_l = brig % 1000 / 100;
      byte four_l = brig % 100 / 10;
      byte five_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 82, 106, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 82, 106, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 82, 180, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 180, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 180, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 180, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 180, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 180, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 180, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 180, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 180, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 180, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 82, 171, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 171, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 171, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 171, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 171, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 171, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 171, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 171, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 171, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 171, CD9, 16, 9, colorPrint);
          break;
      }

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 82, 162, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 162, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 162, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 162, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 162, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 162, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 162, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 162, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 162, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 162, CD9, 16, 9, colorPrint);
          break;
      }

      switch (four_l) {
        case 0:
          DrawImageWH(&paint, 82, 153, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 153, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 153, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 153, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 153, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 153, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 153, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 153, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 153, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 153, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 82, 148, CDP, 16, 5, colorPrint);

      switch (five_l) {
        case 0:
          DrawImageWH(&paint, 82, 139, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 139, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 139, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 139, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 139, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 139, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 139, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 139, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 139, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 139, CD9, 16, 9, colorPrint);
          break;
      }
    }

    if (brig >= 100000 && brig < 1000000) {
      byte one_l = brig / 100000;
      byte two_l = brig % 100000 / 10000;
      byte three_l = brig % 10000 / 1000;
      byte four_l = brig % 1000 / 100;
      byte five_l = brig % 100 / 10;
      byte six_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 82, 101, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 82, 101, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 82, 184, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 184, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 184, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 184, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 184, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 184, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 184, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 184, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 184, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 184, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 82, 175, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 175, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 175, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 175, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 175, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 175, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 175, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 175, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 175, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 175, CD9, 16, 9, colorPrint);
          break;
      }

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 82, 166, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 166, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 166, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 166, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 166, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 166, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 166, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 166, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 166, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 166, CD9, 16, 9, colorPrint);
          break;
      }

      switch (four_l) {
        case 0:
          DrawImageWH(&paint, 82, 157, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 157, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 157, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 157, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 157, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 157, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 157, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 157, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 157, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 157, CD9, 16, 9, colorPrint);
          break;
      }

      switch (five_l) {
        case 0:
          DrawImageWH(&paint, 82, 148, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 148, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 148, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 148, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 148, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 148, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 148, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 148, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 148, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 148, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 82, 143, CDP, 16, 5, colorPrint);

      switch (six_l) {
        case 0:
          DrawImageWH(&paint, 82, 134, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 82, 134, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 82, 134, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 82, 134, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 82, 134, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 82, 134, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 82, 134, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 82, 134, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 82, 134, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 82, 134, CD9, 16, 9, colorPrint);
          break;
      }
    }







  } else {

    if (brig < 100) {
      byte one_l = brig / 10;
      byte two_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 6, 120, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 6, 120, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 6, 167, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 167, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 167, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 167, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 167, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 167, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 167, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 167, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 167, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 167, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 6, 162, CDP, 16, 5, colorPrint);

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 6, 153, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 153, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 153, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 153, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 153, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 153, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 153, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 153, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 153, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 153, CD9, 16, 9, colorPrint);
          break;
      }
    }
    if (brig >= 100 && brig < 1000) {
      byte one_l = brig / 100;
      byte two_l = brig % 10 / 10;
      byte three_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 6, 116, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 6, 116, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 6, 172, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 172, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 172, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 172, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 172, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 172, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 172, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 172, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 172, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 172, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 6, 163, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 163, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 163, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 163, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 163, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 163, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 163, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 163, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 163, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 163, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 6, 158, CDP, 16, 5, colorPrint);

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 6, 149, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 149, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 149, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 149, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 149, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 149, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 149, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 149, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 149, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 149, CD9, 16, 9, colorPrint);
          break;
      }
    }

    if (brig >= 1000 && brig < 10000) {
      byte one_l = brig / 1000;
      byte two_l = brig % 1000 / 100;
      byte three_l = brig % 100 / 10;
      byte four_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 6, 111, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 6, 111, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 6, 176, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 176, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 176, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 176, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 176, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 176, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 176, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 176, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 176, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 176, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 6, 167, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 167, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 167, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 167, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 167, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 167, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 167, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 167, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 167, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 167, CD9, 16, 9, colorPrint);
          break;
      }

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 6, 158, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 158, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 158, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 158, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 158, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 158, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 158, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 158, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 158, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 158, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 6, 153, CDP, 16, 5, colorPrint);

      switch (four_l) {
        case 0:
          DrawImageWH(&paint, 6, 144, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 144, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 144, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 144, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 144, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 144, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 144, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 144, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 144, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 144, CD9, 16, 9, colorPrint);
          break;
      }
    }

    if (brig >= 10000 && brig < 100000) {
      byte one_l = brig / 10000;
      byte two_l = brig % 10000 / 1000;
      byte three_l = brig % 1000 / 100;
      byte four_l = brig % 100 / 10;
      byte five_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 6, 106, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 6, 106, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 6, 180, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 180, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 180, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 180, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 180, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 180, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 180, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 180, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 180, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 180, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 6, 171, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 171, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 171, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 171, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 171, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 171, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 171, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 171, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 171, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 171, CD9, 16, 9, colorPrint);
          break;
      }

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 6, 162, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 162, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 162, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 162, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 162, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 162, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 162, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 162, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 162, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 162, CD9, 16, 9, colorPrint);
          break;
      }

      switch (four_l) {
        case 0:
          DrawImageWH(&paint, 6, 153, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 153, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 153, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 153, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 153, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 153, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 153, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 153, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 153, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 153, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 6, 148, CDP, 16, 5, colorPrint);

      switch (five_l) {
        case 0:
          DrawImageWH(&paint, 6, 139, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 139, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 139, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 139, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 139, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 139, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 139, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 139, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 139, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 139, CD9, 16, 9, colorPrint);
          break;
      }
    }

    if (brig >= 100000 && brig < 1000000) {
      byte one_l = brig / 100000;
      byte two_l = brig % 100000 / 10000;
      byte three_l = brig % 10000 / 1000;
      byte four_l = brig % 1000 / 100;
      byte five_l = brig % 100 / 10;
      byte six_l = brig % 10;

#ifdef LANG_EN
      DrawImageWH(&paint, 6, 101, LUX, 16, 27, colorPrint);
#else
      DrawImageWH(&paint, 6, 101, LUX, 16, 27, colorPrint);
#endif

      switch (one_l) {
        case 0:
          DrawImageWH(&paint, 6, 184, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 184, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 184, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 184, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 184, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 184, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 184, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 184, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 184, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 184, CD9, 16, 9, colorPrint);
          break;
      }

      switch (two_l) {
        case 0:
          DrawImageWH(&paint, 6, 175, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 175, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 175, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 175, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 175, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 175, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 175, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 175, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 175, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 175, CD9, 16, 9, colorPrint);
          break;
      }

      switch (three_l) {
        case 0:
          DrawImageWH(&paint, 6, 166, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 166, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 166, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 166, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 166, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 166, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 166, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 166, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 166, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 166, CD9, 16, 9, colorPrint);
          break;
      }

      switch (four_l) {
        case 0:
          DrawImageWH(&paint, 6, 157, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 157, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 157, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 157, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 157, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 157, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 157, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 157, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 157, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 157, CD9, 16, 9, colorPrint);
          break;
      }

      switch (five_l) {
        case 0:
          DrawImageWH(&paint, 6, 148, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 148, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 148, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 148, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 148, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 148, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 148, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 148, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 148, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 148, CD9, 16, 9, colorPrint);
          break;
      }

      DrawImageWH(&paint, 6, 143, CDP, 16, 5, colorPrint);

      switch (six_l) {
        case 0:
          DrawImageWH(&paint, 6, 134, CD0, 16, 9, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 6, 134, CD1, 16, 9, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 6, 134, CD2, 16, 9, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 6, 134, CD3, 16, 9, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 6, 134, CD4, 16, 9, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 6, 134, CD5, 16, 9, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 6, 134, CD6, 16, 9, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 6, 134, CD7, 16, 9, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 6, 134, CD8, 16, 9, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 6, 134, CD9, 16, 9, colorPrint);
          break;
      }
    }
  }
}
#endif


void displayPres(float pres, bool metr) {


  if (design == false) {
#ifdef LANG_EN
    DrawImageWH(&paint, 43, 220, PRESEN, 10, 72, colorPrint);
#else
    DrawImageWH(&paint, 43, 220, PRES, 10, 72, colorPrint);
#endif
  } else {
    DrawImageWH(&paint, 37 - 12, 251, PRESICON, 17, 17, colorPrint);
  }

  int pressure_temp;
  byte one_p;
  byte two_p;
  byte three_p;
  byte four_p;

  if (design == false) {
    if ((int)pres < 1000) {
      pressure_temp = round(pres * 10.0);
    } else {
      pressure_temp = round(pres);
    }
    one_p = pressure_temp / 1000;
    two_p = pressure_temp % 1000 / 100;
    three_p = pressure_temp % 100 / 10;
    four_p = pressure_temp % 10;
  } else {
    pressure_temp = round(pres);
    if ((int)pres < 1000) {
      pressure_temp = pressure_temp * 10;
    }
    one_p = pressure_temp / 1000;
    two_p = pressure_temp % 1000 / 100;
    three_p = pressure_temp % 100 / 10;
    four_p = pressure_temp % 10;
  }



  if (design == false) {

#ifdef LANG_EN
    DrawImageWH(&paint, 86, 226, PAPRESEN, 10, 61, colorPrint);

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


#else
    DrawImageWH(&paint, 86, 226, MHPRES, 10, 61, colorPrint);

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
#endif

  } else {

#ifdef LANG_EN
    DrawImageWH(&paint, 86 - 12, 228, PAPRESEN, 10, 61, colorPrint);

    if (one_p == 1) {

      switch (one_p) {
        case 1:
          DrawImageWH(&paint, 59 - 12, 275, N2PH1, 25, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59 - 12, 275, N2PH2, 25, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59 - 12, 275, N2PH3, 25, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59 - 12, 275, N2PH4, 25, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59 - 12, 275, N2PH5, 25, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59 - 12, 275, N2PH6, 25, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59 - 12, 275, N2PH7, 25, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59 - 12, 275, N2PH8, 25, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59 - 12, 275, N2PH9, 25, 16, colorPrint);
          break;
      }

      switch (two_p) {
        case 0:
          DrawImageWH(&paint, 59 - 12, 259, N2PH0, 25, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59 - 12, 259, N2PH1, 25, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59 - 12, 259, N2PH2, 25, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59 - 12, 259, N2PH3, 25, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59 - 12, 259, N2PH4, 25, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59 - 12, 259, N2PH5, 25, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59 - 12, 259, N2PH6, 25, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59 - 12, 259, N2PH7, 25, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59 - 12, 259, N2PH8, 25, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59 - 12, 259, N2PH9, 25, 16, colorPrint);
          break;
      }

      switch (three_p) {
        case 0:
          DrawImageWH(&paint, 59 - 12, 243, N2PH0, 25, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59 - 12, 243, N2PH1, 25, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59 - 12, 243, N2PH2, 25, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59 - 12, 243, N2PH3, 25, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59 - 12, 243, N2PH4, 25, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59 - 12, 243, N2PH5, 25, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59 - 12, 243, N2PH6, 25, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59 - 12, 243, N2PH7, 25, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59 - 12, 243, N2PH8, 25, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59 - 12, 243, N2PH9, 25, 16, colorPrint);
          break;
      }

      switch (four_p) {
        case 0:
          DrawImageWH(&paint, 59 - 12, 227, N2PH0, 25, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59 - 12, 227, N2PH1, 25, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59 - 12, 224, N2PH2, 25, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59 - 12, 227, N2PH3, 25, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59 - 12, 227, N2PH4, 25, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59 - 12, 227, N2PH5, 25, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59 - 12, 227, N2PH6, 25, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59 - 12, 227, N2PH7, 25, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59 - 12, 227, N2PH8, 25, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59 - 12, 227, N2PH9, 25, 16, colorPrint);
          break;
      }
    } else {

      switch (one_p) {
        case 1:
          DrawImageWH(&paint, 59 - 12, 267, N2PH1, 25, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59 - 12, 267, N2PH2, 25, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59 - 12, 267, N2PH3, 25, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59 - 12, 267, N2PH4, 25, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59 - 12, 267, N2PH5, 25, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59 - 12, 267, N2PH6, 25, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59 - 12, 267, N2PH7, 25, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59 - 12, 267, N2PH8, 25, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59 - 12, 267, N2PH9, 25, 16, colorPrint);
          break;
      }

      switch (two_p) {
        case 0:
          DrawImageWH(&paint, 59 - 12, 251, N2PH0, 25, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59 - 12, 251, N2PH1, 25, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59 - 12, 251, N2PH2, 25, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59 - 12, 251, N2PH3, 25, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59 - 12, 251, N2PH4, 25, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59 - 12, 251, N2PH5, 25, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59 - 12, 251, N2PH6, 25, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59 - 12, 251, N2PH7, 25, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59 - 12, 251, N2PH8, 25, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59 - 12, 251, N2PH9, 25, 16, colorPrint);
          break;
      }

      switch (three_p) {
        case 0:
          DrawImageWH(&paint, 59 - 12, 235, N2PH0, 25, 16, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 59 - 12, 235, N2PH1, 25, 16, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 59 - 12, 235, N2PH2, 25, 16, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 59 - 12, 235, N2PH3, 25, 16, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 59 - 12, 235, N2PH4, 25, 16, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 59 - 12, 235, N2PH5, 25, 16, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 59 - 12, 235, N2PH6, 25, 16, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 59 - 12, 235, N2PH7, 25, 16, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 59 - 12, 235, N2PH8, 25, 16, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 59 - 12, 235, N2PH9, 25, 16, colorPrint);
          break;
      }
    }


#else
    DrawImageWH(&paint, 86 - 12, 228, MHPRES, 10, 61, colorPrint);

    switch (one_p) {
      case 1:
        DrawImageWH(&paint, 59 - 12, 267, N2PH1, 25, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59 - 12, 267, N2PH2, 25, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59 - 12, 267, N2PH3, 25, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59 - 12, 267, N2PH4, 25, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59 - 12, 267, N2PH5, 25, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59 - 12, 267, N2PH6, 25, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59 - 12, 267, N2PH7, 25, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59 - 12, 267, N2PH8, 25, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59 - 12, 267, N2PH9, 25, 16, colorPrint);
        break;
    }

    switch (two_p) {
      case 0:
        DrawImageWH(&paint, 59 - 12, 251, N2PH0, 25, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 59 - 12, 251, N2PH1, 25, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59 - 12, 251, N2PH2, 25, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59 - 12, 251, N2PH3, 25, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59 - 12, 251, N2PH4, 25, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59 - 12, 251, N2PH5, 25, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59 - 12, 251, N2PH6, 25, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59 - 12, 251, N2PH7, 25, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59 - 12, 251, N2PH8, 25, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59 - 12, 251, N2PH9, 25, 16, colorPrint);
        break;
    }

    switch (three_p) {
      case 0:
        DrawImageWH(&paint, 59 - 12, 235, N2PH0, 25, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 59 - 12, 235, N2PH1, 25, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59 - 12, 235, N2PH2, 25, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59 - 12, 235, N2PH3, 25, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59 - 12, 235, N2PH4, 25, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59 - 12, 235, N2PH5, 25, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59 - 12, 235, N2PH6, 25, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59 - 12, 235, N2PH7, 25, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59 - 12, 235, N2PH8, 25, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59 - 12, 235, N2PH9, 25, 16, colorPrint);
        break;
    }
#endif
  }
}


void displayHum(float hum) {

  if (design == false) {
#ifdef LANG_EN
    DrawImageWH(&paint, 43, 4, HUMEN, 10, 72, colorPrint);
#else
    DrawImageWH(&paint, 43, 4, HUM, 10, 72, colorPrint);
#endif
  } else {
    DrawImageWH(&paint, 37 - 12, 31 - 8, HUMICON, 17, 17, colorPrint);
  }

  int hum_temp = round(hum);

  byte one_h = hum_temp / 10;
  byte two_h = hum_temp % 10;


  if (design == false) {
    DrawImageWH(&paint, 86, 34 - 8, PRHUM, 9, 10, colorPrint);
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
        DrawImageWH(&paint, 592, 23, NPH0, 23, 16, colorPrint);
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
  } else {
    DrawImageWH(&paint, 86 - 12, 34 - 8, PRHUM, 9, 10, colorPrint);
    switch (one_h) {
      case 0:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH0, 25, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH1, 25, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH2, 25, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH3, 25, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH4, 25, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH5, 25, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH6, 25, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH7, 25, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH8, 25, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59 - 12, 39 - 8, N2PH6, 25, 16, colorPrint);
        break;
    }

    switch (two_h) {
      case 0:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH0, 25, 16, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH1, 25, 16, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH2, 25, 16, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH3, 25, 16, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH4, 25, 16, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH5, 25, 16, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH6, 25, 16, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH7, 25, 16, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH8, 25, 16, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 59 - 12, 23 - 8, N2PH9, 25, 16, colorPrint);
        break;
    }
  }
}


void displayLink(int8_t s) {

  if (flag_nogateway_mode == true) {
    s = 0;
  }
  if (design == false) {
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
  }

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

  if (design == false) {
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
  }

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
      DrawImageWH(&paint, 98, 76, F0EN, 30, 144, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 98, 76, F1EN, 30, 144, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 98, 76, F2EN, 30, 144, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 98, 76, F3EN, 30, 144, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 98, 76, F4EN, 30, 144, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 98, 76, F5EN, 30, 144, colorPrint);
      break;
    case 10:
      DrawImageWH(&paint, 98, 76, F10EN, 30, 144, colorPrint);
      break;
  }
#else
  switch (f) {
    case 0:
      DrawImageWH(&paint, 98, 76, F0, 30, 144, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 98, 76, F1, 30, 144, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 98, 76, F2, 30, 144, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 98, 76, F3, 30, 144, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 98, 76, F4, 30, 144, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 98, 76, F5, 30, 144, colorPrint);
      break;
    case 10:
      DrawImageWH(&paint, 98, 76, F10, 30, 144, colorPrint);
      break;
  }
#endif
}


void display_Table()
{
  paint.DrawVerticalLine(110, 0, 74, colorPrint);
  paint.DrawVerticalLine(110, 222, 74, colorPrint);
}


void displayStart() {
#ifdef EINK_V1
  epd.InitV1(lut_full_update);
#else
  epd.InitV2();
#endif
  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
#ifdef EINK_V1
  epd.DisplayFrame();
#else
  epd.DisplayFrameFull();
#endif
  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
  epd.DisplayFrame();

#ifdef EINK_V1
  epd.InitV1(lut_partial_update);
#endif
  paint.SetRotate(ROTATE_0);
  paint.SetWidth(128);
  paint.SetHeight(296);
  paint.Clear(opposite_colorPrint);
  DrawImageWH(&paint, 8, 80, LOGO, 110, 134, colorPrint);
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  //epd.DisplayFrameFull();
  epd.DisplayFrame();
  hwSleep(3000);


  // ###################################           Especially for            ################################### //
#ifdef ESPECIALLY
  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
  epd.DisplayFrame();
  paint.Clear(opposite_colorPrint);
  DrawImageWH(&paint, 0, 0, Especially, 128, 296, colorPrint);
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  hwSleep(4000);
#endif
  // ###################################           Especially for            ################################### //


  epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
  epd.DisplayFrame();
  paint.Clear(opposite_colorPrint);
#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, CONECTEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, CONECT, 64, 192, colorPrint);
#endif
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  //epd.Sleep();
}


void displayUpdate(float t, float h, float p, bool m) {
  //epd.Init(lut_partial_update);
  epd.Reset();
  paint.SetWidth(128);
  paint.SetHeight(296);

  clearEpaper++;
  if (clearEpaper == 5) {
    epd.ClearFrameMemory(0xFF);
    epd.DisplayFrame();
    clearEpaper = 0;
  }

  //epd.ClearFrameMemory(0xFF);
  //epd.DisplayFrame();

  paint.Clear(opposite_colorPrint);
  displayTemp(t, m);
  displayPres(p, m);
  displayHum(h);
#ifdef LIGHTSENS
  displayLux(brightness);
#endif
  displayBatt(battery);
  displayLink(nRFRSSI);
  displayForecast(forecast);
  display_Table();

  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
  epd.Sleep();
}



void reseteinkset() {
  updateink1 = false;
  updateink2 = false;
  updateink3 = false;
  updateink4 = false;
  updateink5 = false;
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

void einkZeropush0() {
  epd.Reset();
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

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, COLOREN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, COLOR, 64, 192, colorPrint);
#endif
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkZeroend0() {
  paint.Clear(opposite_colorPrint);
#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, COLORONEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, COLORON, 64, 192, colorPrint);
#endif
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkZeropush1() {
  paint.Clear(opposite_colorPrint);
#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, DESIGNEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, DESIGN, 64, 192, colorPrint);
#endif
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}


void einkZeroend1() {
  paint.Clear(opposite_colorPrint);

#ifdef LANG_EN
  DrawImageWH(&paint, 32, 51, DESIGNONEN, 64, 192, colorPrint);
#else
  DrawImageWH(&paint, 32, 51, DESIGNON, 64, 192, colorPrint);
#endif

  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}

void einkZeropush() {
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

/*
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
*/

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
#ifdef BUILTIN_LED
  digitalWrite(LED_BUILTIN, HIGH);
  delay(12);
  digitalWrite(LED_BUILTIN, LOW);
#endif
}


void preHwInit() {
#ifdef BIZZER
  pinMode(SOUND_PIN, OUTPUT);
  digitalWrite(SOUND_PIN, LOW);
#endif
  pinMode(PIN_BUTTON, INPUT);
  pinMode(29, INPUT);
#ifdef BUILTIN_LED
  pinMode(LED_BUILTIN, OUTPUT);
#endif
}


void before()
{
  /*
    if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
        ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos))) {
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_UICR->PSELRESET[0] = 18;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_UICR->PSELRESET[1] = 18;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NVIC_SystemReset();
    }
  */
#ifdef DCPOWER
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

  battSend = loadState(103);
  if (battSend > 24) {
    battSend = 4;
    saveState(103, battSend);
  }
  if (battSend <= 0) {
    battSend = 4;
    saveState(103, battSend);
  }
  //battSend = 1; // для теста, 1 час

  if (loadState(106) > 1) {
    saveState(106, 0);
  }
  colorChange(loadState(106));
  //colorChange(false); // для теста, true или false

  if (loadState(107) > 1) {
    saveState(107, 1);
  }

  setSound = loadState(107);

  //setSound = 1; // для теста, вкл. или выкл.


  if (loadState(109) > 1) {
    saveState(109, 0);
  }
  design = loadState(109);


  timeConf();
  blinkLed ();
  displayStart();
  blinkLed ();
  wdt_init();
}


void presentation()
{
  if (needPresent == true) {
    if (flag_nogateway_mode == false) {
      if (mesInfo == false) {
        check = sendSketchInfo(SN, SV);
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
          //check = sendSketchInfo(SN, SV);
          //if (!check) {
          // _transportSM.failedUplinkTransmissions = 0;
          //wait(shortWait * 5);
          // }
        }
        if (check) {
          mesInfo = true;
        }
      }

      if (mesTemp == false) {
        check = present(TEMP_CHILD_ID, S_TEMP, "Temperature");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesTemp = true;
        }
      }

      if (mesHum == false) {
        check = present(HUM_CHILD_ID, S_HUM, "Humidity");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesHum = true;
        }
      }

      if (mesBaro == false) {
        check = present(BARO_CHILD_ID, S_BARO, "Pressure");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesBaro = true;
        }
      }

      if (mesForec == false) {
        check = present(FORECAST_CHILD_ID, S_CUSTOM, "Forecast");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesForec = true;
        }
      }

#ifdef LIGHTSENS
      if (mesLux == false) {
        check = present(LUX_SENS_CHILD_ID, S_LIGHT_LEVEL, "LUX");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesLux = true;
        }
      }
#endif

      if (mesSig == false) {
        check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesSig = true;
        }
      }

      if (mesBat == false) {
        check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesBat = true;
        }
      }

      if (mesBatset == false) {
        check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesBatset = true;
        }
      }

      if (mesRes == false) {
        check = present(MY_SEND_RESET_REASON, S_CUSTOM, "RESTART REASON");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesRes = true;
        }
      }

      if (mesColorset == false) {
        check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesColorset = true;
        }
      }

      if (mesDesignset == false) {
        check = present(SET_DESIGN_ID, S_CUSTOM, "DESIGN");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesDesignset = true;
        }
      }

#ifdef BIZZER
      if (mesSoundset == false) {
        check = present(SET_SOUND_ID, S_CUSTOM, "SOUND ON/OFF");
        if (!check) {
          wait(shortWait * 7);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesSoundset = true;
        }
      }
#endif
      if (mesDesignset == true && mesColorset == true && mesRes == true && mesBatset == true && mesBat == true && mesSig == true && mesForec == true && mesBaro == true && mesHum == true && mesTemp == true) {
        needPresent = false;
#ifdef BIZZER
        if (mesSoundset == false) {
          needPresent = true;
        }
#endif
#ifdef LIGHTSENS
        if (mesLux == false) {
          needPresent = true;
        }
#endif
      }
      wait(shortWait * 5);
      sendAfterResTask = true;
      sendConfig();
      wait(shortWait);
    }
  }
}



void setup() {

  epd.ClearFrameMemory(0xFF);
  epd.DisplayFrame();
  epd.Sleep();

  config_Happy_node();

  if (flag_nogateway_mode == false) {
    sendResetReason();
  }

  metric = getControllerConfig().isMetric;

  //metric = false;

  transportDisable();

  interrupt_Init();

  sendAfterResTask = true;

  sleepTimeCount = SLEEP_TIME;

  bme_initAsleep();
  //wait(100);

#ifdef LIGHTSENS
  light.begin();
  //wait(50);
#endif

  readBatt();

  blinkLed();

  if (flag_nogateway_mode == false) {
#ifdef BIZZER
    Sound();
    wait(20);
#endif
  }
}


void loop() {
  if (flag_update_transport_param == true) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == true) {
    present_only_parent();
  }
  if (isTransportReady() == true) {

    if (flag_find_parent_process == true) {
      find_parent_process();
    }

    if (flag_nogateway_mode == false) {


      if (configMode == false) {
        if (buttIntStatus == PIN_BUTTON) {
          if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
            wdt_nrfReset();
            button_flag = true;
            previousMillis = millis();
          }
          if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
            if ((millis() - previousMillis > 1000) && (millis() - previousMillis <= 4500)) {
              if (updateink1 == false) {
                einkZeropush0();
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
                einkZeropush1();
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
                einkZeropush();
                updateink3 = true;
                updateinkclear = false;
              }
            }
            if ((millis() - previousMillis > 12500) && (millis() - previousMillis <= 13500)) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 13500) && (millis() - previousMillis <= 16500)) {
              if (updateink4 == false) {
                einkOnepush();
                updateink4 = true;
                updateinkclear = false;
              }
            }
            if ((millis() - previousMillis > 16500) && (millis() - previousMillis <= 17500)) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 17500) && (millis() - previousMillis <= 20500)) {
              if (updateink5 == false) {
                einkOnePluspush();
                updateink5 = true;
                updateinkclear = false;
              }
            }
            if (millis() - previousMillis > 20500) {
              if (updateinkclear == false) {
                clearOne();
                updateinkclear = true;
              }
            }
            wdt_nrfReset();
          }
          if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {

            if ((millis() - previousMillis > 1000 && millis() - previousMillis <= 4500) && button_flag == true)
            {
              einkZeroend0();
              reseteinkset();
              if (colorPrint == true) {
                colorPrint = false;
                colorChange(colorPrint);
              } else {
                colorPrint = true;
                colorChange(colorPrint);
              }
              sendAfterResTask = true;
              changeC = true;
              sleepTimeCount = 0;
              readData();
              transportReInitialise();
              wait(shortWait);
              sendData();
              transportDisable();
              wait(shortWait);
              displayUpdate(temperatureSens, humiditySens, pressureSens, metric);
              wait(shortWait);
              change = false;
              button_flag = false;
              buttIntStatus = 0;
              nosleep = false;
            }

            if ((millis() - previousMillis > 5500 && millis() - previousMillis <= 8500) && button_flag == true)
            {
              einkZeroend1();
              reseteinkset();
              if (design == true) {
                design = false;
                saveState(109, design);
              } else {
                design = true;
                saveState(109, design);
              }
              sendAfterResTask = true;
              changeD = true;
              sleepTimeCount = 0;
              readData();
              transportReInitialise();
              wait(shortWait);
              sendData();
              transportDisable();
              wait(shortWait);
              displayUpdate(temperatureSens, humiditySens, pressureSens, metric);
              wait(shortWait);
              change = false;
              button_flag = false;
              buttIntStatus = 0;
              nosleep = false;
            }

            if ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 12500) && (button_flag == true))
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

            if ((millis() - previousMillis > 13500) && (millis() - previousMillis <= 16500) && (button_flag == true))
            {
              einkOneend();
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              wait(shortWait);
              resetPresent();
              presentation();
              wait(shortWait);
              transportDisable();
              wait(shortWait * 10);
              change = true;
              BATT_COUNT = BATT_TIME;
              sleepTimeCount = SLEEP_TIME;
            }

            if ((millis() - previousMillis > 17500) && (millis() - previousMillis <= 20500) && (button_flag == true))
            {
              einkOnePlusend();
              new_device();
            }

            if (((millis() - previousMillis < 1000) || ((millis() - previousMillis > 4500) && (millis() - previousMillis <= 5500)) || ((millis() - previousMillis > 8500) && (millis() - previousMillis <= 9500)) || ((millis() - previousMillis > 12500) && (millis() - previousMillis <= 13500)) || ((millis() - previousMillis > 16500) && (millis() - previousMillis <= 17500)) || (millis() - previousMillis > 20500) ) && (button_flag == true) )
            {
              wdt_nrfReset();
              change = true;
              sleepTimeCount = SLEEP_TIME;
              reseteinkset();
              button_flag = false;
              buttIntStatus = 0;
            }

          }
          wdt_nrfReset();
        } else {
          wdt_nrfReset();
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
              displayUpdate(temperatureSens, humiditySens, pressureSens, metric);
              wait(shortWait);
              change = false;
            }
          }
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
          sleepTimeCount = SLEEP_TIME;
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
          if ((millis() - previousMillis > 1000) && (millis() - previousMillis <= 4500)) {
            if (updateink1 == false) {
              einkZeropush0();
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
              einkZeropush1();
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
              einkZeropush();
              updateink3 = true;
              updateinkclear = false;
            }
          }
          if ((millis() - previousMillis > 12500) && (millis() - previousMillis <= 13500)) {
            if (updateinkclear == false) {
              clearOne();
              updateinkclear = true;
            }
          }
          if ((millis() - previousMillis > 13500) && (millis() - previousMillis <= 16500)) {
            if (updateink4 == false) {
              einkOnePluspush();
              updateink4 = true;
              updateinkclear = false;
            }
          }
          if (millis() - previousMillis > 16500) {
            if (updateinkclear == false) {
              clearOne();
              updateinkclear = true;
            }
          }
          wdt_nrfReset();
        }
        if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {
          if ((millis() - previousMillis > 1000 && millis() - previousMillis <= 4500) && button_flag == true)
          {
            wdt_nrfReset();
            einkZeroend0();
            reseteinkset();
            if (colorPrint == true) {
              colorPrint = false;
              colorChange(colorPrint);
            } else {
              colorPrint = true;
              colorChange(colorPrint);
            }
            sendAfterResTask = true;
            changeC = true;
            sleepTimeCount = 0;
            readData();
            wait(shortWait);
            displayUpdate(temperatureSens, humiditySens, pressureSens, metric);
            wait(shortWait);
            button_flag = false;
            buttIntStatus = 0;
            nosleep = false;
          }
          if ((millis() - previousMillis > 5500) && (millis() - previousMillis <= 8500) && (button_flag == true))
          {
            einkZeroend1();
            reseteinkset();
            if (design == true) {
              design = false;
              saveState(109, design);
            } else {
              design = true;
              saveState(109, design);
            }
            sendAfterResTask = true;
            changeD = true;
            sleepTimeCount = 0;
            readData();
            wait(shortWait);
            displayUpdate(temperatureSens, humiditySens, pressureSens, metric);
            wait(shortWait);
            button_flag = false;
            buttIntStatus = 0;
            nosleep = false;
          }
          if ((millis() - previousMillis > 9500) && (millis() - previousMillis <= 12500) && (button_flag == true))
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
            sleepTimeCount = SLEEP_TIME;
          }
          if ((millis() - previousMillis > 13500) && (millis() - previousMillis <= 16500) && (button_flag == true))
          {
            einkOnePlusend();
            new_device();
          }
          if ( ( ( millis() - previousMillis < 1000) || ( millis() - previousMillis > 4500 && millis() - previousMillis <= 5500 ) || ( millis() - previousMillis > 8500 && millis() - previousMillis <= 9500 ) || ( millis() - previousMillis > 12500 && millis() - previousMillis <= 13500 ) || ( millis() - previousMillis > 16500)) && button_flag == true)
          {
            wdt_nrfReset();
            change = true;
            sleepTimeCount = SLEEP_TIME;
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
          }
        }
        wdt_nrfReset();
      } else {
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
            displayUpdate(temperatureSens, humiditySens, pressureSens, metric);
            wait(shortWait);
          }
          sleepTimeCount = 0;
        }
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

    uint32_t periodTimer;
    uint32_t quotientTimer;
    stopTimer = millis();
    if (stopTimer < startTimer) {
      periodTimer = (4294967295 - startTimer) + stopTimer;
    } else {
      periodTimer = stopTimer - startTimer;
    }
    if (periodTimer >= SLEEP_TIME_WDT) {
      PRECISION_TIME_WDT = SLEEP_TIME_WDT;
    } else {
      PRECISION_TIME_WDT = SLEEP_TIME_WDT - periodTimer;
    }
    hwSleep(PRECISION_TIME_WDT);
    startTimer = millis();
    nosleep = true;
  }
}

void resetPresent() {
  needPresent = true;
  mesInfo = false;
  mesTemp = false;
  mesHum = false;
  mesBaro = false;
  mesForec = false;
  mesSig = false;
  mesBat = false;
  mesBatset = false;
  mesRes = false;
  mesColorset = false;

#ifdef LIGHTSENS
  mesLux = false;
#endif
#ifdef BIZZER
  mesSoundset = false;
  changeBiz = true;
#endif
  sendAfterResTask = true;
  //changeT = true;
  changeB = true;
  changeC = true;
  changeD = true;
}


void bme_initAsleep() {
#ifdef BME280
  if (! bme.begin()) {
    error_bme = true;
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);
  wait(500);
#else
  if (! bme.begin()) {
    error_bme = true;
  }
  bme.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF);
  wait(50);
  sensor.begin();
#endif
}


void readData() {

  if (sendAfterResTask == true) {
    change = true;
  }


#ifdef BME280
  if (error_bme == false) {
    bme.takeForcedMeasurement();
    temperatureSens = bme.readTemperature();
    pressureSens = bme.readPressure() / 100.0F;
    humiditySens = bme.readHumidity();
    forecast = sample(pressureSens);
  } else {
    temperatureSens = -1.0;
    pressureSens = -1.0;
    humiditySens = -1.0;
    forecast = 10;
  }
#else
  if (error_bme == false) {
    bme.takeForcedMeasurement();
    //temperatureSens = bme.readTemperature();
    pressureSens = bme.readPressure() / 100.0F;
    forecast = sample(pressureSens);
  } else {
    //temperatureSens = -1.0;
    pressureSens = -1.0;
    forecast = 10;
  }
  temperatureSens = sensor.readTemperature();
  humiditySens = sensor.readHumidity();
#endif



  if ((int)humiditySens < 0) {
    humiditySens = 0.0;
  }
  if ((int)humiditySens > 99) {
    humiditySens = 99.9;
  }

  if ((int)temperatureSens < 0) {
    temperatureSens = 0.0;
  }
  if ((int)temperatureSens > 99) {
    temperatureSens = 99.9;
  }
  if ((int)pressureSens < 300) {
    pressureSens = 0.0;
  }
  if ((int)pressureSens > 1100) {
    pressureSens = 1100.0;
  }

  if (error_bme == false) {
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
  }

  if (error_bme == false) {
    if (!metric) {
      temperatureSens = temperatureSens * 9.0F / 5.0F + 32.0F;
    } else {
#ifndef LANG_EN
      pressureSens = pressureSens * 0.75006375541921F;
#endif
    }
  }

  if (abs(temperatureSens - old_temperature) >= tempThreshold) {
    old_temperature = temperatureSens;
    change = true;
    tch = true;
  }
  if (abs(pressureSens - old_pressure) >= pressThreshold) {
    old_pressure = pressureSens;
    change = true;
    pch = true;
  }
  if (abs(humiditySens - old_humidity) >= humThreshold) {
    old_humidity = humiditySens;
    change = true;
    hch = true;
  }

#ifdef LIGHTSENS
  brightness = light.get_lux() * 6.5;

  if (abs(brightness - old_brightness) >= brightThreshold) {
    old_brightness = brightness;
    change = true;
    lch = true;
  }
#endif

  BATT_COUNT++;
  CORE_DEBUG(PSTR("BATT_COUNT: %d\n"), BATT_COUNT);
  if (BATT_COUNT >= BATT_TIME) {
    CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
    change = true;
  }
  wdt_nrfReset();
}


void sendData() {
  bool blinkEnable = false;
  if (flag_nogateway_mode == false) {

    if (tch == true) {
      check = send(msgTemp.setDestination(0).set(temperatureSens, 2));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 5);
      } else {
        tch = false;
        blinkEnable = true;
      }
      checkSend();
    }

    if (hch == true) {
      wait(shortWait);
      check = send(msgHum.setDestination(0).set(humiditySens, 2));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 4);
      } else {
        hch = false;
        blinkEnable = true;
      }
      checkSend();
    }

    if (pch == true) {
      wait(shortWait);
      check = send(msgPres.setDestination(0).set(pressureSens, 2));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 4);
      } else {
        pch = false;
        blinkEnable = true;
      }
      checkSend();
    }

    if (fch == true) {
      wait(shortWait);
      check = send(forecastMsg.set(forecast));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 4);
      } else {
        fch = false;
        blinkEnable = true;
      }
      checkSend();
    }

#ifdef LIGHTSENS
    if (lch == true) {
      wait(shortWait);
      check = send(brightMsg.setDestination(0).set(brightness, 2));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 4);
      } else {
        lch = false;
        blinkEnable = true;
      }
      checkSend();
    }
#endif

    if (BATT_COUNT >= BATT_TIME) {
      CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
      //wait(5);
      readBatt();
      BATT_COUNT = 0;
    }
    if (bch == true) {
      batLevSend();
      if (bch == false) {
        blinkEnable = true;
      }
    }
    if (blinkEnable == true) {
      blinkLed();
#ifdef BIZZER
      //Sound();
#endif
    }
    if (needPresent == true) {
      presentation();
    }

    sendConfig();

  } else {
    if (BATT_COUNT >= BATT_TIME) {
      CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
      //wait(5);
      readBatt();
      BATT_COUNT = 0;
    }
    tch = false;
    hch = false;
    bch = false;
    pch = false;
    fch = false;
#ifdef LIGHTSENS
    lch = false;
#endif
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
  wait(26);
  batteryVoltage = hwCPUVoltage();
  //wait(10);
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
    wait(shortWait * 10);
    check = sendBatteryLevel(battery, true);
    wait(1000, C_INTERNAL, I_BATTERY_LEVEL);
    if (check == false) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(shortWait * 4);
    } else {
      bch = false;
      lqSend();
    }
    checkSend();
    wdt_nrfReset();

    check = send(bvMsg.set(batteryVoltageF, 2));
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(shortWait * 4);
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
  } else {
    CORE_DEBUG(PSTR("MyS: SEND LINK QUALITY\n"));
    CORE_DEBUG(PSTR("MyS: LINK QUALITY %: %d\n"), nRFRSSI);
  }
}


static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 3600)
  {
    battery_level = 100;
  }
  else if (mvolts > 2200)
  {
    battery_level = 100 - ((3600 - mvolts) * 90) / 1400;
  }
  else if (mvolts > 2100)
  {
    battery_level = 10 - ((2200 - mvolts) * 10) / 100;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}



void timeConf() {

  SLEEP_TIME = (timeSend * minuteT / SLEEP_TIME_WDT);

  BATT_TIME = (battSend * 60 / timeSend);

  cpNom = (120 / timeSend);

  CORE_DEBUG(PSTR("SLEEP_TIME: %d\n"), SLEEP_TIME);
}


void sendConfig() {
  wdt_nrfReset();
#ifdef BIZZER
  MyMessage setSoundMsg(SET_SOUND_ID, V_VAR1);
#endif
  MyMessage setBattSendMsg(SET_BATT_SEND_ID, V_VAR1);
  MyMessage setColor(SET_COLOR_ID, V_VAR1);

  if (sendAfterResTask == true) {

    if (changeB == true) {
      check = send(setBattSendMsg.set(battSend));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 5);
      } else {
        changeB = false;
      }
    }

    if (changeC == true) {
      bool inverse = loadState(106);
      check = send(setColor.set(inverse));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 5);
      } else {
        changeC = false;
      }
    }

    if (changeD == true) {
      bool design = loadState(109);
      check = send(setDesign.set(design));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 5);
      } else {
        changeD = false;
      }
    }

#ifdef BIZZER
    if (changeBiz == true) {
      check = send(setSoundMsg.set(setSound));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(shortWait * 5);
      } else {
        changeBiz = false;
      }
    }
#endif
    if (changeB == false || changeC == false || changeD == false) {
      sendAfterResTask = false;
#ifdef BIZZER
      if (changeBiz == true) {
        sendAfterResTask = true;
      }
#endif
    }
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
    wait(shortWait * 5);
  }
  if (check) NRF_POWER->RESETREAS = (0xFFFFFFFF);
}


void receive(const MyMessage & message)
{
  if (configMode == true) {

    if (message.sensor == SET_BATT_SEND_ID) {
      if (message.type == V_VAR1) {
        battSend = message.getByte();
        if (battSend > 24) {
          battSend = 24;
        }
        if (battSend <= 0) {
          battSend = 0;
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
        sendAfterResTask = true;
        changeB = true;
        timeConf();
        sleepTimeCount = SLEEP_TIME;
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
        sendAfterResTask = true;
        changeC = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }


    if (message.sensor == SET_DESIGN_ID) {
      if (message.type == V_VAR1) {
        bool designTemp = message.getBool();
        saveState(109, designTemp);
        wait(shortWait);
        send(setDesign.set(designTemp));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        configMode = false;
        change = true;
        sendAfterResTask = true;
        changeD = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }

#ifdef BIZZER
    if (message.sensor == SET_SOUND_ID) {
      if (message.type == V_VAR1) {
        setSound = message.getBool();
        saveState(107, setSound);
        wait(shortWait);
        send(setSoundMsg.set(setSound));
        wait(shortWait);
        transportDisable();
        wait(shortWait);
        configMode = false;
        change = true;
        sendAfterResTask = true;
        changeBiz = true;
        sleepTimeCount = SLEEP_TIME;
      }
    }
#endif
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

#ifdef BIZZER
void Sound() {
  if (setSound == 1) {
    myTone(400, 10);
    wait(10);
    myTone(200, 5);
  }
}


void myTone(uint32_t j, uint32_t k) {
  j = 500000 / j;
  k += millis();
  while (k > millis()) {
    digitalWrite(SOUND_PIN, HIGH); delayMicroseconds(j);
    digitalWrite(SOUND_PIN, LOW ); delayMicroseconds(j);
  }
}
#endif


static __INLINE void wdt_init(void)
{
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = 35 * 32768;
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}


static __INLINE void wdt_nrfReset() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
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
    mtwr = 25000;
    resetPresent();
  } else {
    mtwr = 8000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);
}


void config_Happy_node() {
  if (mtwr == 25000) {
    myid = getNodeId();
    saveState(200, myid);
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      old_mypar = mypar;
      saveState(201, mypar);
      saveState(202, _transportConfig.distanceGW);
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 7;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
  if (mtwr != 25000) {
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
  wait(900, C_INTERNAL, 8);
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
  sleepTimeCount = SLEEP_TIME;
  BATT_COUNT = BATT_TIME;
  change = true;
}


void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}
