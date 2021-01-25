// ###################           Mini wither station with electronic ink display 2.9 Inch | nRF52            ############### //
//                                                                                                                           //
//        @filename   :   EFEKTA_THPEINK290_0.29.ino                                                                         //
//        @brief en   :   Wireless, battery-operated temperature,humidity and pressure sensor(SHT20, SI7020, HTU21D, BME280) //
//                        with electronic ink display(Good Display GDEH029A1). The extended version adds the MAX44009 light  //
//                        sensor, an active bizzer Works on nRF52.                                                           //
//        @brief ru   :   Беcпроводной, батарейный датчик температуры, влажности и давления(SHT20, SI7020, HTU21D, BME280)   //
//                        с дисплеем на электронных чернилах(Good Display GDEH029A1). В расширенной версии добавлен          //
//                        датчик света MAX44009, активный биззер. Работает на nRF52832, nRF52840.                            //
//        @author     :   Andrew Lamchenko aka Berk                                                                          //
//                                                                                                                           //
//        Copyright (C) EFEKTALAB 2020                                                                                       //
//        Copyright (c) 2014-2015 Arduino LLC.  All right reserved.                                                          //
//        Copyright (c) 2016 Arduino Srl.  All right reserved.                                                               //
//        Copyright (c) 2017 Sensnology AB. All right reserved.                                                              //
//        Copyright (C) Waveshare     August 10 2017//                                                                       //
//                                                                                                                           //
// ######################################################################################################################### //


#ifndef _MYBOARDNRF5_H_
#define _MYBOARDNRF5_H_

#include "MyConfig.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//#define EBYTE

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (8u)

/*
    LEDs

    This is optional

    With My Sensors, you can use
    hwPinMode() instead of pinMode()
    hwPinMode() allows to use advanced modes like OUTPUT_H0H1 to drive LEDs.
    https://github.com/mysensors/MySensors/blob/development/drivers/NRF5/nrf5_wiring_constants.h

*/

#ifdef EBYTE
#define PIN_LED1             (2)
#elif defined EBYTE2
#define PIN_LED1             (3)
#else
#define PIN_LED1             (28)
#endif
#define BLUE_LED			       (PIN_LED1)
#define LED_BUILTIN          PIN_LED1

/*
    Buttons

    This is optional
*/

#ifdef EBYTE
#define PIN_BUTTON             (24)
#define SOUND_PIN              (30)
#elif defined EBYTE2
#define PIN_BUTTON             (22)
#define SOUND_PIN              (20)
#else
#define PIN_BUTTON             (27)
#endif

/*
   Analog ports

   If you change g_APinDescription, replace PIN_AIN0 with
   port numbers mapped by the g_APinDescription Array.
   You can add PIN_AIN0 to the g_APinDescription Array if
   you want provide analog ports MCU independed, you can add
   PIN_AIN0..PIN_AIN7 to your custom g_APinDescription Array
   defined in MyBoardNRF5.cpp
*/
static const uint8_t A0  = ADC_A0;
static const uint8_t A1  = ADC_A1;
static const uint8_t A2  = ADC_A2;
static const uint8_t A3  = ADC_A3;
static const uint8_t A4  = ADC_A4;
static const uint8_t A5  = ADC_A5;
static const uint8_t A6  = ADC_A6;
static const uint8_t A7  = ADC_A7;

/*
   Serial interfaces

   RX and TX are required.
   If you have no serial port, use unused pins
   CTS and RTS are optional.
*/

#ifdef EBYTE
#define PIN_SERIAL_RX       (10)
#define PIN_SERIAL_TX       (9)
#elif defined EBYTE2
#define PIN_SERIAL_RX       (9)
#define PIN_SERIAL_TX       (10)
#else
#define PIN_SERIAL_RX       (9)
#define PIN_SERIAL_TX       (10)
#endif


/*
   SPI Interfaces

   This is optional

   If SPI is defined MISO, MOSI, SCK are required
   SS is optional and can be used in your sketch.
*/
#define SPI_INTERFACES_COUNT 1

#ifdef EBYTE
#define PIN_SPI_MISO         (21) //not conected
#define PIN_SPI_MOSI         (15)
#define PIN_SPI_SCK          (20)
#define PIN_SPI_SS           (22)
#elif defined EBYTE2
#define PIN_SPI_MISO         (23) //not conected
#define PIN_SPI_MOSI         (5)
#define PIN_SPI_SCK          (30)
#define PIN_SPI_SS           (31)
#else
#define PIN_SPI_MISO         (2) //not conected
#define PIN_SPI_MOSI         (3)
#define PIN_SPI_SCK          (4)
#define PIN_SPI_SS           (11)
#endif

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
   Wire Interfaces

   This is optional
*/
#define WIRE_INTERFACES_COUNT 2

#ifdef EBYTE
#define PIN_WIRE_SDA         (28u)
#define PIN_WIRE_SCL         (3u)
#elif defined EBYTE2
#define PIN_WIRE_SDA         (2u)
#define PIN_WIRE_SCL         (28u)
#else
#define PIN_WIRE_SDA         (30u)
#define PIN_WIRE_SCL         (29u)
#endif

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


#ifdef __cplusplus
}
#endif

#endif
