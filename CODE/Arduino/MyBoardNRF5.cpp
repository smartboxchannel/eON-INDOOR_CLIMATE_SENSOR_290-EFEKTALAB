/*
  If you don't use an nRF5 board, you can ignore this file.
  
  This file was part of the "My Sensors nRF5 Boards" board repository
  available at https://github.com/mysensors/ArduinoBoards If you have
  questions, please refer the documentation at
  https://github.com/mysensors/ArduinoHwNRF5 first.
  
  This file is compatible with ArduinoHwNRF5 >= 0.2.0

  This file allows you to change the relation between pins referenced in
  the Arduino IDE (0..31) and pins of the nRF5 MCU (P0.00..P0.31).
  
  If you can live with addressing the GPIO pins by using the Arduino pins
  0..31 instead of a custom mapping, don't change this file. If you have
  a lot of Arduino code with fixed pin numbers and you need to map these
  pins to specific pins of the nRF5 MCU; you need to change this file.
  
  If you fill the "g_APinDescription" Array with numbers between 0..31,
  the Arduino pins 0..31 are assigned to pins P0.00..P0.31 of the MCU.
  
  As an example, if you need to change the pin mapping for Arduino pin 5
  to P0.12 of the MCU, you have to write the 12 after PORT0 into the sixth
  position in the  "g_APinDescription" Array.
   
  The extended attributes only affects the nRF5 variants provided with
  official Arduino boards. The arduino-nrf5 variant ignores the extended
  attributes.
    
  The pin mapping effects commands like "pinMode()", "digitalWrite()",
  "analogRead()" and "analogWrite()".
  
  If you change the pin mapping, you have to modify the pins in
  "MyBoardNRF5.h". Especially the analog pin mapping must be replaced with
  your pin numbers by replacing PIN_AIN0..7 with a number of your mapping
  array. You can use the constants PIN_AIN0..7 in the "g_APinDescription"
  Array if you want to reference analog ports MCU independent. You cannot
  use the pins P0.00 and P0.01 for GPIO, when the 32kHz crystal is connected.

  
  ###########################################################################

  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Arduino Srl.  All right reserved.
  Copyright (c) 2017 Sensnology AB. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/



#ifdef MYBOARDNRF5
#include <variant.h>

/*
 * Pins descriptions. Attributes are ignored by arduino-nrf5 variant. 
 * Definition taken from Arduino Primo Core with ordered ports
 */
const PinDescription g_APinDescription[]=
{
  { NOT_A_PORT, 0, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // LFCLK
  { NOT_A_PORT, 1, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // LFCLK
  { PORT0,  2, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A0, PWM4, NOT_ON_TIMER},
  { PORT0,  3, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A1, PWM5, NOT_ON_TIMER},
  { PORT0,  4, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A2, PWM6, NOT_ON_TIMER},
  { PORT0,  5, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A3, PWM7, NOT_ON_TIMER},
  { PORT0,  6, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // INT3
  { PORT0,  7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // INT4
  { PORT0,  8, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM10, NOT_ON_TIMER},    //USER_LED
  { PORT0,  9, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // NFC1
  { PORT0, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // NFC2
  { PORT0, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // TX
  { PORT0, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // RX
  { PORT0, 13, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // SDA
  { PORT0, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // SCL
  { PORT0, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // SDA1
  { PORT0, 16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // SCL1
  { PORT0, 17, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // TP4
  { PORT0, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // TP5
  { PORT0, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // INT2
  { PORT0, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // INT1
  { PORT0, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // INT1
  { PORT0, 22, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM9, NOT_ON_TIMER},
  { PORT0, 23, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM8, NOT_ON_TIMER},
  { PORT0, 24, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER}, // INT
  { PORT0, 25, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM11, NOT_ON_TIMER},   //RED_LED
  { PORT0, 26, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM11, NOT_ON_TIMER},  //GREEN_LED
  { PORT0, 27, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM11, NOT_ON_TIMER},  //BLUE_LED
  { PORT0, 28, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A4, PWM3, NOT_ON_TIMER},
  { PORT0, 29, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A5, PWM2, NOT_ON_TIMER},
  { PORT0, 30, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A6, PWM1, NOT_ON_TIMER},
  { PORT0, 31, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), ADC_A7, PWM0, NOT_ON_TIMER}
};

// Don't remove this line
#include <compat_pin_mapping.h>

#endif
