// ###################          Mini wither station with electronic ink display 2.9 Inch | nRF52            ############### //
//                                                                                                                          //
//        @filename   :   EFEKTA_THEINK102_1.4.ino                                                                          //
//        @brief en   :   Wireless, battery-operated temperature and humidity sensor (SHT20, SI7020)                        //
//                        with electronic ink display(Good Display GDEW0102T4). Works on nRF52.                             //
//        @brief ru   :   Беcпроводной, батарейный датчик температуры и влажности(sht20, si7020)                            //
//                        с дисплеем на электронных чернилах(Good Display GDEW0102T4). Работает на nRF52.                   //
//        @author     :   Andrew Lamchenko aka Berk                                                                         //
//                                                                                                                          //
//        Copyright (C) EFEKTALAB 2020                                                                                      //
//        Copyright (c) 2014-2015 Arduino LLC.  All right reserved.                                                         //
//        Copyright (c) 2016 Arduino Srl.  All right reserved.                                                              //
//        Copyright (c) 2017 Sensnology AB. All right reserved.                                                             //
//        Copyright (C) Waveshare     August 10 2017//                                                                      //
//                                                                                                                          //
// ######################################################################################################################## //


#ifndef _MYBOARDNRF5_H_
#define _MYBOARDNRF5_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (8u)

/* 
 *  LEDs
 *  
 *  This is optional
 *  
 *  With My Sensors, you can use
 *  hwPinMode() instead of pinMode()
 *  hwPinMode() allows to use advanced modes like OUTPUT_H0H1 to drive LEDs.
 *  https://github.com/mysensors/MySensors/blob/development/drivers/NRF5/nrf5_wiring_constants.h
 *
 */

//#define RST_PIN         17
//#define DC_PIN          12
//#define CS_PIN          11
//#define BUSY_PIN        18

 
#define PIN_LED1                (28)
// #define PIN_LED2                (25)
// #define PIN_LED3                (26)
// #define PIN_LED4                (27)
// #define PIN_LED5                (12)
// #define PIN_LED6                (14)
// #define PIN_LED7                (15)
// #define PIN_LED8                (16)
// #define USER_LED             	 (PIN_LED2)
// #define RED_LED                 (PIN_LED3)
// #define GREEN_LED            	 (PIN_LED4)
#define BLUE_LED			           (PIN_LED1)
// #define BLE_LED                 BLUE_LED
#define LED_BUILTIN          PIN_LED1

/* 
 *  Buttons
 *  
 *  This is optional
 */
#define PIN_BUTTON             (27)
// #define PIN_BUTTON2             (4)
// #define PIN_BUTTON3             (5)
// #define PIN_BUTTON4             (6)
// #define PIN_BUTTON5             (7)
// #define PIN_BUTTON6             (8)
// #define PIN_BUTTON7             (9)
// #define PIN_BUTTON8             (10)

/* 
 * Analog ports
 *  
 * If you change g_APinDescription, replace PIN_AIN0 with
 * port numbers mapped by the g_APinDescription Array.
 * You can add PIN_AIN0 to the g_APinDescription Array if
 * you want provide analog ports MCU independed, you can add
 * PIN_AIN0..PIN_AIN7 to your custom g_APinDescription Array
 * defined in MyBoardNRF5.cpp
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
 * Serial interfaces
 * 
 * RX and TX are required.
 * If you have no serial port, use unused pins
 * CTS and RTS are optional.
 */
#define PIN_SERIAL_RX       (9)
#define PIN_SERIAL_TX       (10)
// #define PIN_SERIAL_CTS      (13)
// #define PIN_SERIAL_RTS      (14)

/*
 * SPI Interfaces
 * 
 * This is optional
 * 
 * If SPI is defined MISO, MOSI, SCK are required
 * SS is optional and can be used in your sketch.
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (2) //not conected
#define PIN_SPI_MOSI         (3)
#define PIN_SPI_SCK          (4)
#define PIN_SPI_SS           (11)


static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 *
 * This is optional
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (30u)
#define PIN_WIRE_SCL         (29u)


static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif

#endif
