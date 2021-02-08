// ###################           Mini wither station with electronic ink display 2.9 Inch | nRF52            ############### //
//                                                                                                                           //
//        @filename   :   EFEKTA_THPEINK290_0.29.ino                                                                         //
//        @brief en   :   Wireless, battery-operated temperature,humidity and pressure sensor(SHT20, SI7020, HTU21D, BME280) //
//                        with electronic ink display(Good Display GDEH029A1 OR GDEM029T94). Adds the MAX44009 light sensor, //
//                        an active bizzer Works on nRF52.                                                                   //
//        @brief ru   :   Беcпроводной, батарейный датчик температуры, влажности и давления(SHT20, SI7020, HTU21D, BME280)   //
//                        с дисплеем на электронных чернилах(Good Display GDEH029A1 или GDEM029T94). Добавлен датчик         //
//                        света MAX44009, активный биззер. Работает на nRF52832, nRF52840.                                   //
//        @author     :   Andrew Lamchenko aka Berk                                                                          //
//                                                                                                                           //
//        Copyright (C) EFEKTALAB 2020                                                                                       //
//        Copyright (c) 2014-2015 Arduino LLC.  All right reserved.                                                          //
//        Copyright (c) 2016 Arduino Srl.  All right reserved.                                                               //
//        Copyright (c) 2017 Sensnology AB. All right reserved.                                                              //
//        Copyright (C) Waveshare     August 10 2017                                                                         //
//                                                                                                                           //
// ######################################################################################################################### //


//#define EINK_V1
//#define EBYTE // pro
#define EBYTE2 // standart
#define DCPOWER
//#define LIGHTSENS
//#define BIZZER
//#define LANG_EN
//#define MY_DEBUG
//#define MY_PASSIVE_NODE
//#define MY_NODE_ID 101
#define MY_NRF5_ESB_MODE (NRF5_1MBPS)
//#define MY_NRF5_ESB_MODE (NRF5_250KBPS)
#define MY_RESET_REASON_TEXT
#define SN "EFEKTA WeatherStation 290"
#define SV "0.39"
#define MY_RADIO_NRF5_ESB
